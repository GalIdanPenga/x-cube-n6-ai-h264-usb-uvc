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
#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32n6xx_hal.h"
#include <assert.h>

#define ALIGN_VALUE(_v_,_a_) (((_v_) + (_a_) - 1) & ~((_a_) - 1))

#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

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

/* NN Model Instance and Interface */
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);

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

static int is_cache_enable(void)
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

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
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_Default();
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_Default();
  uint32_t nn_period_ms;
  uint32_t nn_period[2];
  uint8_t *nn_pipe_dst;
  uint32_t nn_out_len;
  uint32_t nn_in_len;
  uint32_t total_ts;
  uint32_t ts;
  int ret;

  (void) nn_period_ms;

  /* Initialize Cube.AI/ATON ... */
  LL_ATON_RT_RuntimeInit();
  /* ... and model instance */
  LL_ATON_RT_Init_Network(&NN_Instance_Default);

  /* setup inout buffers and size */
  nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  nn_out_len = LL_Buffer_len(&nn_out_info[0]);
  assert(nn_out_len == NN_BUFFER_OUT_SIZE);

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
    /* run ATON inference */
    ts = HAL_GetTick();
     /* Note that we don't need to clean/invalidate those input buffers since they are only access in hardware */
    ret = LL_ATON_Set_User_Input_Buffer_Default(0, capture_buffer, nn_in_len);
     /* Invalidate output buffer before Hw access it */
    CACHE_OP(SCB_InvalidateDCache_by_Addr(output_buffer, nn_out_len));
    ret = LL_ATON_Set_User_Output_Buffer_Default(0, output_buffer, nn_out_len);
    assert(ret == LL_ATON_User_IO_NOERROR);
    Run_Inference(&NN_Instance_Default);
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

  /* Store statistics pointers */
  stat_lock = stat_lock_handle;
  stat_nn_total = (nn_time_stat_t *)stat_total;
  stat_nn_inference = (nn_time_stat_t *)stat_inference;

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
  return &NN_Instance_Default;
}
