/**
 ******************************************************************************
 * @file    fal_ai_model.c
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

#include "fal/fal_ai_model.h"
#include "fal/fal_npu.h"
#include "ll_aton_runtime.h"
#include "ll_aton_osal.h"
#include <string.h>

/* Configuration */
#define FAL_AI_MODEL_MAX_REGISTERED  4

/**
 * @brief Internal model structure
 */
struct fal_ai_model_s {
    const char *name;
    const NN_Interface_TypeDef *interface;
    NN_Instance_TypeDef *instance;
    fal_ai_model_state_t state;
    uint8_t registered;
};

/* Model registry */
static struct fal_ai_model_s model_registry[FAL_AI_MODEL_MAX_REGISTERED];
static int registry_count = 0;

/* Helper: Run inference loop */
static void run_inference_internal(NN_Instance_TypeDef *network_instance)
{
    LL_ATON_RT_RetValues_t ret;

    do {
        ret = LL_ATON_RT_RunEpochBlock(network_instance);
        if (ret == LL_ATON_RT_WFE)
        {
            LL_ATON_OSAL_WFE();
        }
    } while (ret != LL_ATON_RT_DONE);

    LL_ATON_RT_Reset_Network(network_instance);
}

fal_ai_model_err_t fal_ai_model_register(const char *name, void *interface, void *instance)
{
    if (name == NULL || interface == NULL || instance == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    /* Check if already registered */
    for (int i = 0; i < registry_count; i++)
    {
        if (model_registry[i].registered && strcmp(model_registry[i].name, name) == 0)
        {
            return FAL_AI_MODEL_ERROR_ALREADY_REGISTERED;
        }
    }

    /* Check registry space */
    if (registry_count >= FAL_AI_MODEL_MAX_REGISTERED)
    {
        return FAL_AI_MODEL_ERROR_REGISTRY_FULL;
    }

    /* Register model */
    struct fal_ai_model_s *model = &model_registry[registry_count];
    model->name = name;
    model->interface = (const NN_Interface_TypeDef *)interface;
    model->instance = (NN_Instance_TypeDef *)instance;
    model->state = FAL_AI_MODEL_STATE_UNLOADED;
    model->registered = 1;
    registry_count++;

    return FAL_AI_MODEL_OK;
}

fal_ai_model_t fal_ai_model_get(const char *name)
{
    if (name == NULL)
    {
        return NULL;
    }

    for (int i = 0; i < registry_count; i++)
    {
        if (model_registry[i].registered && strcmp(model_registry[i].name, name) == 0)
        {
            return &model_registry[i];
        }
    }

    return NULL;
}

fal_ai_model_err_t fal_ai_model_init(fal_ai_model_t model)
{
    if (model == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    if (!fal_npu_is_init())
    {
        return FAL_AI_MODEL_ERROR_NPU_NOT_INIT;
    }

    if (model->state == FAL_AI_MODEL_STATE_LOADED)
    {
        return FAL_AI_MODEL_ERROR_ALREADY_LOADED;
    }

    /* Initialize network */
    LL_ATON_RT_Init_Network(model->instance);
    model->state = FAL_AI_MODEL_STATE_LOADED;

    return FAL_AI_MODEL_OK;
}

fal_ai_model_err_t fal_ai_model_deinit(fal_ai_model_t model)
{
    if (model == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    if (model->state == FAL_AI_MODEL_STATE_UNLOADED)
    {
        return FAL_AI_MODEL_ERROR_NOT_LOADED;
    }

    if (model->state == FAL_AI_MODEL_STATE_RUNNING)
    {
        return FAL_AI_MODEL_ERROR_BUSY;
    }

    LL_ATON_RT_DeInit_Network(model->instance);
    model->state = FAL_AI_MODEL_STATE_UNLOADED;

    return FAL_AI_MODEL_OK;
}

fal_ai_model_err_t fal_ai_model_get_info(fal_ai_model_t model, fal_ai_model_info_t *info)
{
    if (model == NULL || info == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    const LL_Buffer_InfoTypeDef *in_info = LL_ATON_Input_Buffers_Info(model->instance);
    const LL_Buffer_InfoTypeDef *out_info = LL_ATON_Output_Buffers_Info(model->instance);

    info->name = model->name;

    /* Count inputs */
    info->num_inputs = 0;
    info->input_size = 0;
    info->input_alignment = 1;
    for (const LL_Buffer_InfoTypeDef *p = in_info; p->name != NULL; p++)
    {
        info->num_inputs++;
        info->input_size += LL_Buffer_len(p);
    }

    /* Count outputs */
    info->num_outputs = 0;
    info->output_size = 0;
    info->output_alignment = 1;
    for (const LL_Buffer_InfoTypeDef *p = out_info; p->name != NULL; p++)
    {
        info->num_outputs++;
        info->output_size += LL_Buffer_len(p);
    }

    return FAL_AI_MODEL_OK;
}

fal_ai_model_state_t fal_ai_model_get_state(fal_ai_model_t model)
{
    if (model == NULL)
    {
        return FAL_AI_MODEL_STATE_ERROR;
    }
    return model->state;
}

fal_ai_model_err_t fal_ai_model_get_input_info(fal_ai_model_t model, uint32_t idx, fal_ai_buffer_info_t *info)
{
    if (model == NULL || info == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    const LL_Buffer_InfoTypeDef *buf_info = LL_ATON_Input_Buffers_Info(model->instance);

    /* Find buffer at index */
    uint32_t i = 0;
    for (const LL_Buffer_InfoTypeDef *p = buf_info; p->name != NULL; p++, i++)
    {
        if (i == idx)
        {
            info->name = p->name;
            info->size = LL_Buffer_len(p);
            info->alignment = 32; /* Default alignment */
            info->data_type = (uint8_t)p->type;
            info->num_dims = p->ndims;
            info->shape = p->shape;
            return FAL_AI_MODEL_OK;
        }
    }

    return FAL_AI_MODEL_ERROR_INVALID_PARAM;
}

fal_ai_model_err_t fal_ai_model_get_output_info(fal_ai_model_t model, uint32_t idx, fal_ai_buffer_info_t *info)
{
    if (model == NULL || info == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    const LL_Buffer_InfoTypeDef *buf_info = LL_ATON_Output_Buffers_Info(model->instance);

    /* Find buffer at index */
    uint32_t i = 0;
    for (const LL_Buffer_InfoTypeDef *p = buf_info; p->name != NULL; p++, i++)
    {
        if (i == idx)
        {
            info->name = p->name;
            info->size = LL_Buffer_len(p);
            info->alignment = 32; /* Default alignment */
            info->data_type = (uint8_t)p->type;
            info->num_dims = p->ndims;
            info->shape = p->shape;
            return FAL_AI_MODEL_OK;
        }
    }

    return FAL_AI_MODEL_ERROR_INVALID_PARAM;
}

fal_ai_model_err_t fal_ai_model_set_input(fal_ai_model_t model, uint32_t idx, void *buf, uint32_t size)
{
    if (model == NULL || buf == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    if (model->state != FAL_AI_MODEL_STATE_LOADED)
    {
        return FAL_AI_MODEL_ERROR_NOT_LOADED;
    }

    LL_ATON_User_IO_Result_t ret = LL_ATON_Set_User_Input_Buffer(model->instance, idx, buf, size);
    if (ret != LL_ATON_User_IO_NOERROR)
    {
        return FAL_AI_MODEL_ERROR_IO_SETUP;
    }

    return FAL_AI_MODEL_OK;
}

fal_ai_model_err_t fal_ai_model_set_output(fal_ai_model_t model, uint32_t idx, void *buf, uint32_t size)
{
    if (model == NULL || buf == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    if (model->state != FAL_AI_MODEL_STATE_LOADED)
    {
        return FAL_AI_MODEL_ERROR_NOT_LOADED;
    }

    LL_ATON_User_IO_Result_t ret = LL_ATON_Set_User_Output_Buffer(model->instance, idx, buf, size);
    if (ret != LL_ATON_User_IO_NOERROR)
    {
        return FAL_AI_MODEL_ERROR_IO_SETUP;
    }

    return FAL_AI_MODEL_OK;
}

fal_ai_model_err_t fal_ai_model_run(fal_ai_model_t model)
{
    if (model == NULL)
    {
        return FAL_AI_MODEL_ERROR_INVALID_PARAM;
    }

    if (model->state != FAL_AI_MODEL_STATE_LOADED)
    {
        return FAL_AI_MODEL_ERROR_NOT_LOADED;
    }

    model->state = FAL_AI_MODEL_STATE_RUNNING;
    run_inference_internal(model->instance);
    model->state = FAL_AI_MODEL_STATE_LOADED;

    return FAL_AI_MODEL_OK;
}

void *fal_ai_model_get_instance(fal_ai_model_t model)
{
    if (model == NULL)
    {
        return NULL;
    }
    return model->instance;
}
