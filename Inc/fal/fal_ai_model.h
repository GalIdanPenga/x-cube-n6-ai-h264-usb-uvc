/**
 ******************************************************************************
 * @file    fal_ai_model.h
 * @brief   Firmware Abstraction Layer - AI Model Management
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

#ifndef FAL_AI_MODEL_H
#define FAL_AI_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Model error codes
 */
typedef enum {
    FAL_AI_MODEL_OK = 0,
    FAL_AI_MODEL_ERROR_NPU_NOT_INIT,
    FAL_AI_MODEL_ERROR_INVALID_PARAM,
    FAL_AI_MODEL_ERROR_NOT_FOUND,
    FAL_AI_MODEL_ERROR_ALREADY_REGISTERED,
    FAL_AI_MODEL_ERROR_REGISTRY_FULL,
    FAL_AI_MODEL_ERROR_NOT_LOADED,
    FAL_AI_MODEL_ERROR_ALREADY_LOADED,
    FAL_AI_MODEL_ERROR_IO_SETUP,
    FAL_AI_MODEL_ERROR_BUSY,
} fal_ai_model_err_t;

/**
 * @brief Model state
 */
typedef enum {
    FAL_AI_MODEL_STATE_UNLOADED,
    FAL_AI_MODEL_STATE_LOADED,
    FAL_AI_MODEL_STATE_RUNNING,
    FAL_AI_MODEL_STATE_ERROR
} fal_ai_model_state_t;

/**
 * @brief Opaque model handle
 */
typedef struct fal_ai_model_s *fal_ai_model_t;

/**
 * @brief Model information structure
 */
typedef struct {
    const char *name;            /**< Model name */
    uint32_t num_inputs;         /**< Number of input tensors */
    uint32_t num_outputs;        /**< Number of output tensors */
    uint32_t input_size;         /**< Total input buffer size in bytes */
    uint32_t output_size;        /**< Total output buffer size in bytes */
    uint32_t input_alignment;    /**< Required input buffer alignment */
    uint32_t output_alignment;   /**< Required output buffer alignment */
} fal_ai_model_info_t;

/**
 * @brief Buffer information structure
 */
typedef struct {
    const char *name;            /**< Buffer name */
    uint32_t size;               /**< Buffer size in bytes */
    uint32_t alignment;          /**< Required alignment */
    uint8_t data_type;           /**< Data type (see LL_ATON types) */
    uint8_t num_dims;            /**< Number of dimensions */
    const uint32_t *shape;       /**< Pointer to shape array */
} fal_ai_buffer_info_t;

/**
 * @brief Register a compiled-in model with the FAL
 * @note Called at initialization time for each model
 * @param name Model name (must be unique)
 * @param interface Pointer to NN_Interface_TypeDef
 * @param instance Pointer to NN_Instance_TypeDef
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_register(const char *name, void *interface, void *instance);

/**
 * @brief Get a model handle by name
 * @param name Model name
 * @return Model handle, or NULL if not found
 */
fal_ai_model_t fal_ai_model_get(const char *name);

/**
 * @brief Initialize a model for inference
 * @note Must be called after fal_npu_init() and before running inference
 * @param model Model handle
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_init(fal_ai_model_t model);

/**
 * @brief De-initialize a model
 * @param model Model handle
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_deinit(fal_ai_model_t model);

/**
 * @brief Get model information
 * @param model Model handle
 * @param info Pointer to info structure to fill
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_get_info(fal_ai_model_t model, fal_ai_model_info_t *info);

/**
 * @brief Get current model state
 * @param model Model handle
 * @return Current state
 */
fal_ai_model_state_t fal_ai_model_get_state(fal_ai_model_t model);

/**
 * @brief Get input buffer info
 * @param model Model handle
 * @param idx Input index (0-based)
 * @param info Pointer to buffer info structure to fill
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_get_input_info(fal_ai_model_t model, uint32_t idx, fal_ai_buffer_info_t *info);

/**
 * @brief Get output buffer info
 * @param model Model handle
 * @param idx Output index (0-based)
 * @param info Pointer to buffer info structure to fill
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_get_output_info(fal_ai_model_t model, uint32_t idx, fal_ai_buffer_info_t *info);

/**
 * @brief Set input buffer for inference
 * @param model Model handle
 * @param idx Input index (0-based)
 * @param buf Pointer to input data buffer
 * @param size Buffer size in bytes
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_set_input(fal_ai_model_t model, uint32_t idx, void *buf, uint32_t size);

/**
 * @brief Set output buffer for inference
 * @param model Model handle
 * @param idx Output index (0-based)
 * @param buf Pointer to output data buffer
 * @param size Buffer size in bytes
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_set_output(fal_ai_model_t model, uint32_t idx, void *buf, uint32_t size);

/**
 * @brief Run inference (blocking)
 * @note Input/output buffers must be set before calling this
 * @param model Model handle
 * @return FAL_AI_MODEL_OK on success, error code otherwise
 */
fal_ai_model_err_t fal_ai_model_run(fal_ai_model_t model);

/**
 * @brief Get the underlying NN instance pointer
 * @note For advanced use cases that need direct LL_ATON access
 * @param model Model handle
 * @return Pointer to NN_Instance_TypeDef, or NULL
 */
void *fal_ai_model_get_instance(fal_ai_model_t model);

#ifdef __cplusplus
}
#endif

#endif /* FAL_AI_MODEL_H */
