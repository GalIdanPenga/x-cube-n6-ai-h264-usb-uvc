/**
 ******************************************************************************
 * @file    fal_ai_pipe.c
 * @brief   Firmware Abstraction Layer - AI Vision Pipeline
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "fal/fal_ai_pipe.h"
#include "fal/fal_ai_model.h"
#include "fal/fal_npu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32n6xx_hal.h"
#include <string.h>

/* Configuration */
#define FAL_AI_PIPE_MAX_INSTANCES     2
#define FAL_AI_PIPE_DEFAULT_STACK     (2 * configMINIMAL_STACK_SIZE)

/**
 * @brief Internal time statistics
 */
typedef struct {
    uint64_t acc_ms;
    uint32_t count;
    uint32_t last_ms;
} fal_time_stat_t;

/**
 * @brief Internal pipeline structure
 */
struct fal_ai_pipe_s {
    /* Configuration */
    fal_ai_model_t model;
    bqueue_t *input_queue;
    bqueue_t *output_queue;
    fal_ai_pipe_output_cb_t output_cb;
    void *cb_ctx;

    /* Queue ownership */
    uint8_t owns_input_queue;
    uint8_t owns_output_queue;

    /* Thread */
    TaskHandle_t task_handle;
    StaticTask_t task_tcb;
    StackType_t task_stack[FAL_AI_PIPE_DEFAULT_STACK];

    /* State */
    volatile uint8_t running;
    volatile uint8_t stop_requested;

    /* Statistics */
    SemaphoreHandle_t stats_lock;
    StaticSemaphore_t stats_lock_buf;
    fal_time_stat_t stat_inference;
    fal_time_stat_t stat_total;
    uint32_t frames_processed;
    uint32_t frames_dropped;
    uint32_t last_fps_tick;
    uint32_t fps_frame_count;
    float current_fps;

    /* Registration */
    uint8_t allocated;
};

/* Pipeline instance pool */
static struct fal_ai_pipe_s pipe_pool[FAL_AI_PIPE_MAX_INSTANCES];

/* Helper: Update time stats */
static void stat_update(fal_ai_pipe_t pipe, fal_time_stat_t *stat, uint32_t ms)
{
    xSemaphoreTake(pipe->stats_lock, portMAX_DELAY);
    stat->last_ms = ms;
    stat->acc_ms += ms;
    stat->count++;
    xSemaphoreGive(pipe->stats_lock);
}

/* Helper: Update FPS */
static void fps_update(fal_ai_pipe_t pipe)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - pipe->last_fps_tick;

    pipe->fps_frame_count++;

    if (elapsed >= 1000)
    {
        xSemaphoreTake(pipe->stats_lock, portMAX_DELAY);
        pipe->current_fps = (float)pipe->fps_frame_count * 1000.0f / (float)elapsed;
        xSemaphoreGive(pipe->stats_lock);

        pipe->fps_frame_count = 0;
        pipe->last_fps_tick = now;
    }
}

/* Pipeline thread function */
static void pipe_thread_fct(void *arg)
{
    fal_ai_pipe_t pipe = (fal_ai_pipe_t)arg;
    fal_ai_model_info_t model_info;
    uint32_t total_ts, inference_ts;
    fal_ai_model_err_t err;

    /* Get model info */
    err = fal_ai_model_get_info(pipe->model, &model_info);
    if (err != FAL_AI_MODEL_OK)
    {
        return;
    }

    pipe->last_fps_tick = HAL_GetTick();

    /* Main processing loop */
    while (!pipe->stop_requested)
    {
        uint8_t *input_buffer;
        uint8_t *output_buffer;

        /* Wait for input */
        input_buffer = bqueue_get_ready(pipe->input_queue);
        if (input_buffer == NULL)
        {
            continue;
        }

        /* Get output buffer */
        output_buffer = bqueue_get_free(pipe->output_queue, 1);
        if (output_buffer == NULL)
        {
            /* Return input buffer unused */
            bqueue_put_free(pipe->input_queue);
            pipe->frames_dropped++;
            continue;
        }

        total_ts = HAL_GetTick();

        /* Setup I/O buffers */
        err = fal_ai_model_set_input(pipe->model, 0, input_buffer, model_info.input_size);
        if (err != FAL_AI_MODEL_OK)
        {
            bqueue_put_free(pipe->input_queue);
            bqueue_put_free(pipe->output_queue);
            pipe->frames_dropped++;
            continue;
        }

        /* Invalidate output buffer before NPU access */
        fal_npu_cache_invalidate(output_buffer, model_info.output_size);

        err = fal_ai_model_set_output(pipe->model, 0, output_buffer, model_info.output_size);
        if (err != FAL_AI_MODEL_OK)
        {
            bqueue_put_free(pipe->input_queue);
            bqueue_put_free(pipe->output_queue);
            pipe->frames_dropped++;
            continue;
        }

        /* Run inference */
        inference_ts = HAL_GetTick();
        err = fal_ai_model_run(pipe->model);
        stat_update(pipe, &pipe->stat_inference, HAL_GetTick() - inference_ts);

        if (err != FAL_AI_MODEL_OK)
        {
            bqueue_put_free(pipe->input_queue);
            bqueue_put_free(pipe->output_queue);
            pipe->frames_dropped++;
            continue;
        }

        /* Release buffers */
        bqueue_put_free(pipe->input_queue);
        bqueue_put_ready(pipe->output_queue);

        stat_update(pipe, &pipe->stat_total, HAL_GetTick() - total_ts);
        pipe->frames_processed++;
        fps_update(pipe);

        /* Optional callback */
        if (pipe->output_cb != NULL)
        {
            pipe->output_cb(output_buffer, model_info.output_size, pipe->cb_ctx);
        }
    }

    pipe->running = 0;
    vTaskDelete(NULL);
}

fal_ai_pipe_t fal_ai_pipe_create(const fal_ai_pipe_config_t *config)
{
    fal_ai_pipe_t pipe = NULL;

    if (config == NULL || config->model == NULL)
    {
        return NULL;
    }

    /* Find free slot */
    for (int i = 0; i < FAL_AI_PIPE_MAX_INSTANCES; i++)
    {
        if (!pipe_pool[i].allocated)
        {
            pipe = &pipe_pool[i];
            break;
        }
    }

    if (pipe == NULL)
    {
        return NULL;
    }

    /* Initialize */
    memset(pipe, 0, sizeof(*pipe));
    pipe->allocated = 1;
    pipe->model = config->model;
    pipe->output_cb = config->output_cb;
    pipe->cb_ctx = config->cb_ctx;

    /* Setup queues */
    if (config->input_queue != NULL)
    {
        pipe->input_queue = config->input_queue;
        pipe->owns_input_queue = 0;
    }
    else
    {
        /* Note: For simplicity, external queue required for now */
        /* TODO: Implement internal queue allocation */
        pipe->allocated = 0;
        return NULL;
    }

    if (config->output_queue != NULL)
    {
        pipe->output_queue = config->output_queue;
        pipe->owns_output_queue = 0;
    }
    else
    {
        /* Note: For simplicity, external queue required for now */
        pipe->allocated = 0;
        return NULL;
    }

    /* Create stats lock */
    pipe->stats_lock = xSemaphoreCreateMutexStatic(&pipe->stats_lock_buf);
    if (pipe->stats_lock == NULL)
    {
        pipe->allocated = 0;
        return NULL;
    }

    return pipe;
}

fal_ai_pipe_err_t fal_ai_pipe_destroy(fal_ai_pipe_t pipe)
{
    if (pipe == NULL)
    {
        return FAL_AI_PIPE_ERROR_INVALID_PARAM;
    }

    if (pipe->running)
    {
        fal_ai_pipe_stop(pipe);
    }

    /* Note: Static semaphore doesn't need deletion */
    pipe->allocated = 0;

    return FAL_AI_PIPE_OK;
}

fal_ai_pipe_err_t fal_ai_pipe_start(fal_ai_pipe_t pipe)
{
    if (pipe == NULL)
    {
        return FAL_AI_PIPE_ERROR_INVALID_PARAM;
    }

    if (pipe->running)
    {
        return FAL_AI_PIPE_ERROR_ALREADY_STARTED;
    }

    if (fal_ai_model_get_state(pipe->model) != FAL_AI_MODEL_STATE_LOADED)
    {
        return FAL_AI_PIPE_ERROR_MODEL_NOT_INIT;
    }

    pipe->stop_requested = 0;
    pipe->running = 1;

    pipe->task_handle = xTaskCreateStatic(
        pipe_thread_fct,
        "ai_pipe",
        FAL_AI_PIPE_DEFAULT_STACK,
        pipe,
        tskIDLE_PRIORITY + 4,
        pipe->task_stack,
        &pipe->task_tcb
    );

    if (pipe->task_handle == NULL)
    {
        pipe->running = 0;
        return FAL_AI_PIPE_ERROR_NO_MEMORY;
    }

    return FAL_AI_PIPE_OK;
}

fal_ai_pipe_err_t fal_ai_pipe_stop(fal_ai_pipe_t pipe)
{
    if (pipe == NULL)
    {
        return FAL_AI_PIPE_ERROR_INVALID_PARAM;
    }

    if (!pipe->running)
    {
        return FAL_AI_PIPE_ERROR_NOT_STARTED;
    }

    pipe->stop_requested = 1;

    /* Wait for thread to exit */
    while (pipe->running)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return FAL_AI_PIPE_OK;
}

bqueue_t *fal_ai_pipe_get_input_queue(fal_ai_pipe_t pipe)
{
    if (pipe == NULL)
    {
        return NULL;
    }
    return pipe->input_queue;
}

bqueue_t *fal_ai_pipe_get_output_queue(fal_ai_pipe_t pipe)
{
    if (pipe == NULL)
    {
        return NULL;
    }
    return pipe->output_queue;
}

fal_ai_pipe_err_t fal_ai_pipe_get_stats(fal_ai_pipe_t pipe, fal_ai_pipe_stats_t *stats)
{
    if (pipe == NULL || stats == NULL)
    {
        return FAL_AI_PIPE_ERROR_INVALID_PARAM;
    }

    xSemaphoreTake(pipe->stats_lock, portMAX_DELAY);

    if (pipe->stat_inference.count > 0)
    {
        stats->avg_inference_ms = (uint32_t)(pipe->stat_inference.acc_ms / pipe->stat_inference.count);
    }
    else
    {
        stats->avg_inference_ms = 0;
    }

    if (pipe->stat_total.count > 0)
    {
        stats->avg_total_ms = (uint32_t)(pipe->stat_total.acc_ms / pipe->stat_total.count);
    }
    else
    {
        stats->avg_total_ms = 0;
    }

    stats->frames_processed = pipe->frames_processed;
    stats->frames_dropped = pipe->frames_dropped;
    stats->fps = pipe->current_fps;

    xSemaphoreGive(pipe->stats_lock);

    return FAL_AI_PIPE_OK;
}

fal_ai_pipe_err_t fal_ai_pipe_reset_stats(fal_ai_pipe_t pipe)
{
    if (pipe == NULL)
    {
        return FAL_AI_PIPE_ERROR_INVALID_PARAM;
    }

    xSemaphoreTake(pipe->stats_lock, portMAX_DELAY);

    memset(&pipe->stat_inference, 0, sizeof(pipe->stat_inference));
    memset(&pipe->stat_total, 0, sizeof(pipe->stat_total));
    pipe->frames_processed = 0;
    pipe->frames_dropped = 0;
    pipe->current_fps = 0;
    pipe->fps_frame_count = 0;
    pipe->last_fps_tick = HAL_GetTick();

    xSemaphoreGive(pipe->stats_lock);

    return FAL_AI_PIPE_OK;
}
