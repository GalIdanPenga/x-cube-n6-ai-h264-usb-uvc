/**
 ******************************************************************************
 * @file    fal_ai_pipe.h
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

#ifndef FAL_AI_PIPE_H
#define FAL_AI_PIPE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "fal/fal_ai_model.h"
#include "bqueue.h"
#include "FreeRTOS.h"

/**
 * @brief Pipeline error codes
 */
typedef enum {
    FAL_AI_PIPE_OK = 0,
    FAL_AI_PIPE_ERROR_INVALID_PARAM,
    FAL_AI_PIPE_ERROR_NO_MEMORY,
    FAL_AI_PIPE_ERROR_MODEL_NOT_INIT,
    FAL_AI_PIPE_ERROR_ALREADY_STARTED,
    FAL_AI_PIPE_ERROR_NOT_STARTED,
} fal_ai_pipe_err_t;

/**
 * @brief Opaque pipeline handle
 */
typedef struct fal_ai_pipe_s *fal_ai_pipe_t;

/**
 * @brief Pipeline statistics
 */
typedef struct {
    uint32_t avg_inference_ms;     /**< Average inference time (ms) */
    uint32_t avg_total_ms;         /**< Average total frame time (ms) */
    uint32_t frames_processed;     /**< Total frames processed */
    uint32_t frames_dropped;       /**< Total frames dropped */
    float fps;                     /**< Current frames per second */
} fal_ai_pipe_stats_t;

/**
 * @brief Output callback type
 * @param output Pointer to inference output buffer
 * @param size Output buffer size
 * @param ctx User context
 */
typedef void (*fal_ai_pipe_output_cb_t)(void *output, uint32_t size, void *ctx);

/**
 * @brief Pipeline configuration
 */
typedef struct {
    fal_ai_model_t model;               /**< Model to use (required) */
    bqueue_t *input_queue;              /**< External input queue (NULL = use internal) */
    bqueue_t *output_queue;             /**< External output queue (NULL = use internal) */
    fal_ai_pipe_output_cb_t output_cb;  /**< Optional callback on inference complete */
    void *cb_ctx;                       /**< Context passed to callback */
    UBaseType_t thread_priority;        /**< Thread priority (FreeRTOS priority) */
    uint32_t input_buf_count;           /**< Number of input buffers (if using internal queue) */
    uint32_t output_buf_count;          /**< Number of output buffers (if using internal queue) */
} fal_ai_pipe_config_t;

/**
 * @brief Create a vision pipeline
 * @param config Pipeline configuration
 * @return Pipeline handle, or NULL on error
 */
fal_ai_pipe_t fal_ai_pipe_create(const fal_ai_pipe_config_t *config);

/**
 * @brief Destroy a pipeline
 * @param pipe Pipeline handle
 * @return FAL_AI_PIPE_OK on success
 */
fal_ai_pipe_err_t fal_ai_pipe_destroy(fal_ai_pipe_t pipe);

/**
 * @brief Start pipeline processing
 * @param pipe Pipeline handle
 * @return FAL_AI_PIPE_OK on success
 */
fal_ai_pipe_err_t fal_ai_pipe_start(fal_ai_pipe_t pipe);

/**
 * @brief Stop pipeline processing
 * @param pipe Pipeline handle
 * @return FAL_AI_PIPE_OK on success
 */
fal_ai_pipe_err_t fal_ai_pipe_stop(fal_ai_pipe_t pipe);

/**
 * @brief Get input queue for pipeline
 * @param pipe Pipeline handle
 * @return Pointer to input bqueue
 */
bqueue_t *fal_ai_pipe_get_input_queue(fal_ai_pipe_t pipe);

/**
 * @brief Get output queue for pipeline
 * @param pipe Pipeline handle
 * @return Pointer to output bqueue
 */
bqueue_t *fal_ai_pipe_get_output_queue(fal_ai_pipe_t pipe);

/**
 * @brief Get pipeline statistics
 * @param pipe Pipeline handle
 * @param stats Pointer to stats structure to fill
 * @return FAL_AI_PIPE_OK on success
 */
fal_ai_pipe_err_t fal_ai_pipe_get_stats(fal_ai_pipe_t pipe, fal_ai_pipe_stats_t *stats);

/**
 * @brief Reset pipeline statistics
 * @param pipe Pipeline handle
 * @return FAL_AI_PIPE_OK on success
 */
fal_ai_pipe_err_t fal_ai_pipe_reset_stats(fal_ai_pipe_t pipe);

#ifdef __cplusplus
}
#endif

#endif /* FAL_AI_PIPE_H */
