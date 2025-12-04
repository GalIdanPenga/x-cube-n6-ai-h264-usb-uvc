/**
******************************************************************************
* @file    app_ai.h
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

#ifndef APP_AI_H
#define APP_AI_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "bqueue.h"

/* Forward declaration for time_stat_t pointer */
typedef struct time_stat_s time_stat_t;

/* Initialize AI module - call before app_ai_start() */
void app_ai_init(SemaphoreHandle_t stat_lock, void *stat_nn_total, void *stat_nn_inference);

/* Start AI inference thread */
void app_ai_start(UBaseType_t priority);

/* Get input queue - camera callback feeds frames here */
bqueue_t *app_ai_get_input_queue(void);

/* Get output queue - display thread reads results here */
bqueue_t *app_ai_get_output_queue(void);

/* Get NN instance - needed by dp_thread for postprocess init */
void *app_ai_get_nn_instance(void);

#endif /* APP_AI_H */
