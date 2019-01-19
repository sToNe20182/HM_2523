/*************************************************************************
    > File Name: fusion_alg_manager.c
    > Author: wells
    > Mail: wangweicsd@126.com
    > Created Time:2018-12-25 13:21
 ************************************************************************/

#ifndef __FUSION_ALG_MANAGER_H__
#define __FUSION_ALG_MANAGER_H__

#include <stdint.h>
#include <string.h>
#include "bsp_lcd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#define FUSION_MGR_QUEUE_SIZE  100

typedef struct fusion_alg_message_struct_t {
    int         message_id;
    int         param1;
    void        *param2;
} fusion_alg_msg_struct_t;


typedef enum{
    MESSAGE_ID_RAW_SENSOR_DATA,
	MESSAGE_ID_RAW_GPS_DATA
}fusion_alg_msg_status_id;

int32_t fusion_alg_send_event(fusion_alg_msg_status_id event_id, int32_t param1, void* param2);
int32_t fusion_alg_manager_init();

#endif
