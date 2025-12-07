/**
 ******************************************************************************
 * @file    fal_npu.h
 * @brief   Firmware Abstraction Layer - NPU Hardware Abstraction
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

#ifndef FAL_NPU_H
#define FAL_NPU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief NPU Error codes
 */
typedef enum {
    FAL_NPU_OK = 0,
    FAL_NPU_ERROR_NOT_INIT,
    FAL_NPU_ERROR_ALREADY_INIT,
    FAL_NPU_ERROR_INVALID_PARAM,
} fal_npu_err_t;

/**
 * @brief NPU capabilities structure
 */
typedef struct {
    uint32_t max_concurrent_models;   /**< Maximum models that can be registered */
    uint32_t internal_mem_size;       /**< NPU internal memory size in bytes */
    uint32_t cache_line_size;         /**< Cache line size for alignment */
} fal_npu_caps_t;

/**
 * @brief Initialize the NPU runtime
 * @note Must be called before any other fal_npu or fal_ai_model functions
 * @return FAL_NPU_OK on success, error code otherwise
 */
fal_npu_err_t fal_npu_init(void);

/**
 * @brief De-initialize the NPU runtime
 * @return FAL_NPU_OK on success, error code otherwise
 */
fal_npu_err_t fal_npu_deinit(void);

/**
 * @brief Check if NPU is initialized
 * @return 1 if initialized, 0 otherwise
 */
int fal_npu_is_init(void);

/**
 * @brief Get NPU capabilities
 * @param caps Pointer to capabilities structure to fill
 * @return FAL_NPU_OK on success, error code otherwise
 */
fal_npu_err_t fal_npu_get_caps(fal_npu_caps_t *caps);

/**
 * @brief Invalidate data cache for a memory region
 * @note Use before NPU reads from a buffer that CPU has written
 * @param addr Start address (should be cache-line aligned)
 * @param size Size in bytes (will be rounded up to cache line)
 */
void fal_npu_cache_invalidate(void *addr, uint32_t size);

/**
 * @brief Clean data cache for a memory region
 * @note Use after CPU writes to a buffer that NPU will read
 * @param addr Start address (should be cache-line aligned)
 * @param size Size in bytes (will be rounded up to cache line)
 */
void fal_npu_cache_clean(void *addr, uint32_t size);

/**
 * @brief Clean and invalidate data cache for a memory region
 * @param addr Start address (should be cache-line aligned)
 * @param size Size in bytes (will be rounded up to cache line)
 */
void fal_npu_cache_clean_invalidate(void *addr, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* FAL_NPU_H */
