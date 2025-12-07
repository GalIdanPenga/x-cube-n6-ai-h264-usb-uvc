/**
 ******************************************************************************
 * @file    fal_npu.c
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

#include "fal/fal_npu.h"
#include "ll_aton_runtime.h"
#include "stm32n6xx_hal.h"

/* Configuration */
#define FAL_NPU_MAX_MODELS        4
#define FAL_NPU_CACHE_LINE_SIZE   32

/* State */
static int npu_initialized = 0;

/* Cache enable check */
static int is_cache_enabled(void)
{
#if defined(USE_DCACHE)
    return 1;
#else
    return 0;
#endif
}

fal_npu_err_t fal_npu_init(void)
{
    if (npu_initialized)
    {
        return FAL_NPU_ERROR_ALREADY_INIT;
    }

    /* Initialize the LL_ATON runtime */
    LL_ATON_RT_RuntimeInit();

    npu_initialized = 1;
    return FAL_NPU_OK;
}

fal_npu_err_t fal_npu_deinit(void)
{
    if (!npu_initialized)
    {
        return FAL_NPU_ERROR_NOT_INIT;
    }

    LL_ATON_RT_RuntimeDeInit();

    npu_initialized = 0;
    return FAL_NPU_OK;
}

int fal_npu_is_init(void)
{
    return npu_initialized;
}

fal_npu_err_t fal_npu_get_caps(fal_npu_caps_t *caps)
{
    if (caps == NULL)
    {
        return FAL_NPU_ERROR_INVALID_PARAM;
    }

    caps->max_concurrent_models = FAL_NPU_MAX_MODELS;
    caps->internal_mem_size = 0;  /* TODO: Query from hardware if available */
    caps->cache_line_size = FAL_NPU_CACHE_LINE_SIZE;

    return FAL_NPU_OK;
}

void fal_npu_cache_invalidate(void *addr, uint32_t size)
{
    if (is_cache_enabled() && addr != NULL && size > 0)
    {
        SCB_InvalidateDCache_by_Addr(addr, size);
    }
}

void fal_npu_cache_clean(void *addr, uint32_t size)
{
    if (is_cache_enabled() && addr != NULL && size > 0)
    {
        SCB_CleanDCache_by_Addr(addr, size);
    }
}

void fal_npu_cache_clean_invalidate(void *addr, uint32_t size)
{
    if (is_cache_enabled() && addr != NULL && size > 0)
    {
        SCB_CleanInvalidateDCache_by_Addr(addr, size);
    }
}
