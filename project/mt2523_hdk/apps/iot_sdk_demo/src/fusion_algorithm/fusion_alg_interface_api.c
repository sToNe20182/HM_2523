/*************************************************************************
    > File Name: fusion_alg_interface_api.c
    > Author: wells
    > Mail: wangweicsd@126.com
    > Created Time:2018-12-25 13:21
 ************************************************************************/

#include <string.h>
#include "bsp_lcd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "hal_cache.h"
#include "hal_mpu.h"
#include "hal_uart.h"
#include "hal_clock.h"
#include "fusion_alg_manager.h"
#include "task_def.h"
#include "fusion_alg_interface_api.h"
#include "hal_gpt.h"
#include "main_screen.h"
#include "gnss_log.h"

TimerHandle_t g_read_sensor_timer = NULL;
extern QueueHandle_t raw_data_queue_handle;

QueueHandle_t alg_api_queue_handle;
static TaskHandle_t fusion_alg_api_task_handle;

//RawSensorData *sensor_date

void inject_sensor_data(void) 
{
	//bmi160_date_get();
    fusion_alg_send_event(MESSAGE_ID_RAW_SENSOR_DATA, 1,NULL);
	
	//fusion_alg_msg_struct_t message;
	//BaseType_t xHigherPriorityTaskWoken;
	//TickType_t ticks;
    //message.message_id = MESSAGE_ID_RAW_SENSOR_DATA;
    //message.param1 = 1;
    //message.param2 = (void*)&sensor_date;
	
	//GNSSLOGD("sensor time occurption!!!!!!!!!!!!!!!!!!!!!:ld%\n",snesor_date.sensor_sys_time);
	
	//xQueueSendFromISR(raw_data_queue_handle, &message, &xHigherPriorityTaskWoken);

}
void inject_location_data(const GNSSData *gnssData)
{
    fusion_alg_send_event(MESSAGE_ID_RAW_GPS_DATA,1,gnssData);
}

void alg_api_timer_handle(void)
{
	fusion_alg_api_msg_struct_t message;
	BaseType_t xHigherPriorityTaskWoken;
	//TickType_t ticks;
    message.message_id = MESSAGE_ID_RAW_SENSOR_DATA;
    message.param1 = 1;
    message.param2 = NULL;
	
	//GNSSLOGD("sensor time occurption!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	
	xQueueSendFromISR(alg_api_queue_handle, &message, &xHigherPriorityTaskWoken);
}

void stop_delay_time()
{
    if(g_read_sensor_timer != NULL)
    {
       xTimerStop(g_read_sensor_timer, 0);
       xTimerDelete(g_read_sensor_timer, 0);
    }
    g_read_sensor_timer = NULL;
}

void start_read_sensor_time(uint8_t time,TimerCallbackFunction_t callback)
{
    g_read_sensor_timer = xTimerCreate("delay_timer",
                                    (time/ portTICK_PERIOD_MS),
                                    pdPASS,
                                    0,
                                    callback
                                 );
    xTimerStart(g_read_sensor_timer, 0);
}

static void alg_api_msg_handler(fusion_alg_api_msg_struct_t *message)
{
    if(!message)
    {
        return;
    }
    switch(message->message_id)
    {
        case ready_to_read:
            //inject_raw_sensor_data((raw_sensor_data*) message->param2);
            //printf("________________bmi160________ready to read_________\n");
			inject_sensor_data();
            break;
        default :
            break;
    }
}


void fusion_alg_api_task(void * arg)
{
    fusion_alg_api_msg_struct_t fusion_alg_api_event_data_item;

    while(1)
    {
        if (xQueueReceive(alg_api_queue_handle, &fusion_alg_api_event_data_item, portMAX_DELAY))
        {
            alg_api_msg_handler(&fusion_alg_api_event_data_item);
        }
    }
}


int32_t fusion_alg_api_init(void)
{
    alg_api_queue_handle = xQueueCreate(FUSION_API_QUEUE_SIZE,sizeof(fusion_alg_api_msg_struct_t));
    xTaskCreate(fusion_alg_api_task, FUSION_ALG_API_TASK_NAME, FUSION_ALG_API_TASK_STACKSIZE / sizeof(StackType_t), NULL, FUSION_ALG_API_TASK_PRIO,&fusion_alg_api_task_handle);
    return 1;
}


void init_fusion_alg_data()
{
    start_read_sensor_time(50,inject_sensor_data);
	//hal_gpt_init(HAL_GPT_2);
	//hal_gpt_register_callback(HAL_GPT_2, alg_api_timer_handle, NULL);
	//hal_gpt_start_timer_ms(HAL_GPT_2, 50, HAL_GPT_TIMER_TYPE_REPEAT);

	//fusion_alg_api_init();
}

