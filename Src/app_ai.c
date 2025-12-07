/**
******************************************************************************
* @file    app_ai.c
* @author  GPM Application Team
*
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

#include "app_ai.h"
#include "app.h"
#include "app_cam.h"
#include "bqueue.h"
#include "cmw_camera.h"
#include "network.h"
#include "fal/fal_npu.h"
#include "fal/fal_ai_model.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32n6xx_hal.h"
#include <assert.h>

#define ALIGN_VALUE(_v_,_a_) (((_v_) + (_a_) - 1) & ~((_a_) - 1))

/* Model Related Info */
#define NN_BUFFER_OUT_SIZE LL_ATON_DEFAULT_OUT_1_SIZE_BYTES
#define NN_BUFFER_OUT_SIZE_ALIGN ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)

/* Local time_stat_t definition for statistics updates */
typedef struct {
  int last;
  int total;
  uint64_t acc;
  float mean;
} nn_time_stat_t;

/* NN Model Instance and Interface (for FAL registration) */
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);

/* FAL model handle */
static fal_ai_model_t ai_model = NULL;

/* NN input buffers */
static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;

/* NN output buffers */
static uint8_t nn_output_buffers[2][NN_BUFFER_OUT_SIZE_ALIGN] ALIGN_32;
static bqueue_t nn_output_queue;

/* Thread */
static StaticTask_t nn_thread;
static StackType_t nn_thread_stack[2 * configMINIMAL_STACK_SIZE];

/* Statistics pointers (passed from app.c) */
static SemaphoreHandle_t stat_lock;
static nn_time_stat_t *stat_nn_total;
static nn_time_stat_t *stat_nn_inference;

static void time_stat_update(nn_time_stat_t *p_stat, int value)
{
  int ret;

  ret = xSemaphoreTake(stat_lock, portMAX_DELAY);
  assert(ret == pdTRUE);

  p_stat->last = value;
  p_stat->acc += value;
  p_stat->total++;
  p_stat->mean = (float)p_stat->acc / p_stat->total;

  ret = xSemaphoreGive(stat_lock);
  assert(ret == pdTRUE);
}

static void nn_thread_fct(void *arg)
{
  fal_ai_model_info_t model_info;
  uint32_t nn_period_ms;
  uint32_t nn_period[2];
  uint8_t *nn_pipe_dst;
  uint32_t total_ts;
  uint32_t ts;
  fal_ai_model_err_t err;

  (void) nn_period_ms;

  /* Initialize NPU via FAL */
  err = fal_npu_init();
  assert(err == FAL_NPU_OK);

  /* Initialize model via FAL */
  err = fal_ai_model_init(ai_model);
  assert(err == FAL_AI_MODEL_OK);

  /* Get model info */
  err = fal_ai_model_get_info(ai_model, &model_info);
  assert(err == FAL_AI_MODEL_OK);
  assert(model_info.output_size == NN_BUFFER_OUT_SIZE);

  /*** App Loop ***************************************************************/
  nn_period[1] = HAL_GetTick();

  nn_pipe_dst = bqueue_get_free(&nn_input_queue, 0);
  assert(nn_pipe_dst);
  CAM_NNPipe_Start(nn_pipe_dst, CMW_MODE_CONTINUOUS);
  while (1)
  {
    uint8_t *capture_buffer;
    uint8_t *output_buffer;

    nn_period[0] = nn_period[1];
    nn_period[1] = HAL_GetTick();
    nn_period_ms = nn_period[1] - nn_period[0];

    capture_buffer = bqueue_get_ready(&nn_input_queue);
    assert(capture_buffer);
    output_buffer = bqueue_get_free(&nn_output_queue, 1);
    assert(output_buffer);

    total_ts = HAL_GetTick();

    /* Setup I/O buffers via FAL */
    ts = HAL_GetTick();
    /* Note: input buffers are only accessed by hardware (camera DMA), no cache op needed */
    err = fal_ai_model_set_input(ai_model, 0, capture_buffer, model_info.input_size);
    assert(err == FAL_AI_MODEL_OK);

    /* Invalidate output buffer before NPU accesses it */
    fal_npu_cache_invalidate(output_buffer, model_info.output_size);

    err = fal_ai_model_set_output(ai_model, 0, output_buffer, model_info.output_size);
    assert(err == FAL_AI_MODEL_OK);

    /* Run inference via FAL */
    err = fal_ai_model_run(ai_model);
    assert(err == FAL_AI_MODEL_OK);

    time_stat_update(stat_nn_inference, HAL_GetTick() - ts);

    /* release buffers */
    bqueue_put_free(&nn_input_queue);
    bqueue_put_ready(&nn_output_queue);

    time_stat_update(stat_nn_total, HAL_GetTick() - total_ts);
  }
}

void app_ai_init(SemaphoreHandle_t stat_lock_handle, void *stat_total, void *stat_inference)
{
  int ret;
  fal_ai_model_err_t err;

  /* Store statistics pointers */
  stat_lock = stat_lock_handle;
  stat_nn_total = (nn_time_stat_t *)stat_total;
  stat_nn_inference = (nn_time_stat_t *)stat_inference;

  /* Register model with FAL */
  err = fal_ai_model_register("Default", (void *)&NN_Interface_Default, &NN_Instance_Default);
  assert(err == FAL_AI_MODEL_OK);

  /* Get model handle */
  ai_model = fal_ai_model_get("Default");
  assert(ai_model != NULL);

  /* Create buffer queues */
  ret = bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
  assert(ret == 0);
  ret = bqueue_init(&nn_output_queue, 2, (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]});
  assert(ret == 0);
}

void app_ai_start(UBaseType_t priority)
{
  TaskHandle_t hdl;

  hdl = xTaskCreateStatic(nn_thread_fct, "nn", configMINIMAL_STACK_SIZE * 2, NULL, priority, nn_thread_stack,
                          &nn_thread);
  assert(hdl != NULL);
}

bqueue_t *app_ai_get_input_queue(void)
{
  return &nn_input_queue;
}

bqueue_t *app_ai_get_output_queue(void)
{
  return &nn_output_queue;
}

void *app_ai_get_nn_instance(void)
{
  return fal_ai_model_get_instance(ai_model);
}
