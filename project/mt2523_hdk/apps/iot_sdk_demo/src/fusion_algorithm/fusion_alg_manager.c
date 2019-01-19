/*************************************************************************
    > File Name: fusion_alg_manager.c
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
#include "main_screen.h"
#include "fusion_alg_interface_api.h"
#include "SendGps.h"

#include "datatypes.h"
#include "GINavMain.h"
#include "GetAcc.h"

#include "Const.h"

#define GYRO_SCALE			(PI*INSUPDATE_SUBDATA_INTERVAL/1000.0/180.0/131.2)		//scale of gyroscope       PI/180*0.1/131.2  �Ƕ�����ϵ��
#define ACC_SCALE			(GRAVITY_CONST*INSUPDATE_SUBDATA_INTERVAL/1000.0/1000)	//scale of accelerometer 9.78*0.1/1000.0;  �ٶ�����ϵ��

#define sensor_buf_length 10

RawSensorData sensor_date;
extern void bmi160_date_get(void);

TimerHandle_t xTimerofSDM;
QueueHandle_t raw_data_queue_handle;
static TaskHandle_t fusion_alg_manager_task_handle;

RawSensorData *alg_sensor_data;
GNSSData alg_gnss_data;

RawSensorData sensor_buf[sensor_buf_length];

IMU_DATA_T    IMUDataBuffer;	 //gty IMUËùÓÐÖ¸Õë¾ùÖ¸Ïò¸ÃµØÖ·....
GNSS_DATA_T   GNSSDataBuffer;  //gty GPSËùÓÐÖ¸Õë¾ùÖ¸Ïò¸ÃµØÖ·....

PIMU_DATA_T   pImuData;        //gty Ö¸ÕëIMU....
PGNSS_DATA_T  pGnssData;       //gty Ö¸ÕëGPS....

BOOL GNSSDataReady;          //
BOOL IMUDataReady;         //

uint8_t   	GpsInsGetFlag;   //
uint8_t     Gnss_Get_Flag;


OUTPUT_INFO_T GINavResult;     //gty ×éºÏµ¼º½Êä³öµÄËùÓÐÐÅÏ¢

GSAV_DATA_T   g_GsavInfo;       //Gty  »ñµÃGSA,GSVÐÅÏ¢....


void Change_SensorData(uint16_t SensorData,uint8_t Change_Data[])
{
    static uint8_t  i;
	
	Change_Data[0]=SensorData/16/16/16;
	Change_Data[1]=SensorData/16/16-Change_Data[0]*16;
	Change_Data[2]=SensorData/16-Change_Data[0]*16*16-Change_Data[1]*16;
	Change_Data[3]=SensorData-Change_Data[0]*16*16*16-Change_Data[1]*16*16-Change_Data[2]*16;
	
	for(i=0;i<4;i++)
	{
		if(Change_Data[i] <= 9)
		{
		Change_Data[i] = (Change_Data[i] + '0') ;
		}
		else
		{
		Change_Data[i] = ((Change_Data[i]-10) + 'A') ;
		}   
	}
}

uint8_t ProcessCheckIMU(uint8_t DNum,uint8_t IMU_DATAT[])
{
	static  uint8_t i;
	static uint8_t CheckNum;

	CheckNum=0;
	for(i=1;i<DNum;i++)
	{
	CheckNum^=IMU_DATAT[i]; 
	}   
	return CheckNum;
}

//----------------------------------------
//void ProcessCheckToA(uint8_t CheckNum)
//----------------------------------------
void ProcessCheckToAIMU(uint8_t CheckNum,uint8_t Change_Data[])
{									
static   uint8_t  i;
  			     
   Change_Data[0] = CheckNum/16 ; 			       //Ê®Î»
   Change_Data[1] = CheckNum -Change_Data[0]*16;	   //¸öÎ»

   for(i=0;i<2;i++)
   {
    if(Change_Data[i] <= 9)
	{
	 Change_Data[i] = (Change_Data[i] + '0') ;
	}
	else
	{
	 Change_Data[i] = ((Change_Data[i]-10) + 'A') ;
	}   
   }
}

uint8_t IMUTIME[7][20];

void IMUTime_Deal(SYSTEM_STATUS *Sys_Status,int count)
{
	uint8_t  IMUTC_Data[20],Temp_Data[10],length = 0,i = 0,j=0;
	//unsigned long time = Sys_Status->current_time;

	//printf("-----------------count:%d\n",count);

	IMUTC_Data[0]='$';
	IMUTC_Data[1]='I';
	IMUTC_Data[2]='5';
	IMUTC_Data[3]='D';
	IMUTC_Data[4]='A';
	IMUTC_Data[5]='T';
	IMUTC_Data[6]=',';

	i=7;

	while(IMUDataBuffer.UtcTime.Hour)
	{
		Temp_Data[j] = IMUDataBuffer.UtcTime.Hour%10+0x30;
		IMUDataBuffer.UtcTime.Hour /= 10;
		j++;
		length++;
	}

	if(length == 1)
			IMUTC_Data[i++] = 0x30;
	else if(length == 0)
	{
		IMUTC_Data[i++] = 0x30;
		IMUTC_Data[i++] = 0x30;
	}

	for(;length>=1;length--)
	{
		IMUTC_Data[i] =  Temp_Data[length-1];
		i++;
	}

	j = 0;length = 0;

	while(IMUDataBuffer.UtcTime.Minute)
	{
		Temp_Data[j] = IMUDataBuffer.UtcTime.Minute%10+0x30;
		IMUDataBuffer.UtcTime.Minute /= 10;
		j++;
		length++;
	}

	if(length == 1)
			IMUTC_Data[i++] = 0x30;
	else if(length == 0)
	{
		IMUTC_Data[i++] = 0x30;
		IMUTC_Data[i++] = 0x30;
	}
	
	for(;length>=1;length--)
	{
		IMUTC_Data[i] =  Temp_Data[length-1];
		i++;
	}

	j = 0;length = 0;

	while(IMUDataBuffer.UtcTime.Second)
	{
		Temp_Data[j] = IMUDataBuffer.UtcTime.Second%10+0x30;
		IMUDataBuffer.UtcTime.Second /= 10;
		j++;
		length++;
	}

	
	if(length == 1)
			IMUTC_Data[i++] = 0x30;
	else if(length == 0)
	{
		IMUTC_Data[i++] = 0x30;
		IMUTC_Data[i++] = 0x30;
	}

	for(;length>=1;length--)
	{
		IMUTC_Data[i] =  Temp_Data[length-1];
		i++;
	}

	j = 0;length = 0;

	IMUTC_Data[i++] = '.';

	while(IMUDataBuffer.UtcTime.MillSecond)
	{
		Temp_Data[j] = IMUDataBuffer.UtcTime.MillSecond%10+0x30;
		IMUDataBuffer.UtcTime.MillSecond /= 10;
		j++;
		length++;
	}
		
	if(length == 2)
			IMUTC_Data[i++] = 0x30;
	else if(length == 1)
	{
		IMUTC_Data[i++] = 0x30;
		IMUTC_Data[i++] = 0x30;
	}
	else if(length == 0)
	{
		IMUTC_Data[i++] = 0x30;
		IMUTC_Data[i++] = 0x30;
		IMUTC_Data[i++] = 0x30;
	}
			

	for(;length>=1;length--)
	{
		IMUTC_Data[i] =  Temp_Data[length-1];
		i++;
	}

	j = 0;length = 0;

	i--;

	IMUTC_Data[i++] = '\n';
	IMUTC_Data[i] = 0;

	for(j=0;j<i+1;j++)
		IMUTIME[count+1][j]=IMUTC_Data[j];

	//printf("%s\n",IMUTC_Data);
}


uint8_t IMUDATA[6][60];

void IMUData_Deal(SYSTEM_STATUS *Sys_Status)
{
  uint8_t  CheckNumData=0;
  int16_t  Gyr_Temp=0;
  uint8_t  i=0;
  uint8_t  Imu_Up_Num=0,IMU_DATAT[60],Change_Data[10];
  uint8_t  pos = Sys_Status->pos;
	
/*-----------------IMUÖ¡Í·--------------------------*/
  IMU_DATAT[0] = '$';
  IMU_DATAT[1] = 'G';
  IMU_DATAT[2] = 'P';
  IMU_DATAT[3] = 'I';
  IMU_DATAT[4] = 'M';
  IMU_DATAT[5] = 'U';
  IMU_DATAT[6] = ',';
		
  Imu_Up_Num=0;
	
  Gyr_Temp=0;

  Gyr_Temp=IMUDataBuffer.GyroF[0][0];

  //printf("%d\n",sensor_buf[pos].gxyz[0]);

  Change_SensorData(Gyr_Temp,Change_Data);
  IMU_DATAT[Imu_Up_Num+7]  = Change_Data[0];
  IMU_DATAT[Imu_Up_Num+8]  = Change_Data[1];
  IMU_DATAT[Imu_Up_Num+9]  = Change_Data[2];
  IMU_DATAT[Imu_Up_Num+10] = Change_Data[3];
  IMU_DATAT[Imu_Up_Num+11] = ',';
//---------------------------------------------
  Gyr_Temp=IMUDataBuffer.GyroF[0][1];
	
	Change_SensorData(Gyr_Temp,Change_Data);
	IMU_DATAT[Imu_Up_Num+12] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+13] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+14] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+15] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+16] = ',';	
	
//---------------------------------------------
	Gyr_Temp=IMUDataBuffer.GyroF[0][2];
	
  	Change_SensorData(Gyr_Temp,Change_Data);
	IMU_DATAT[Imu_Up_Num+17] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+18] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+19] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+20] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+21] = ',';
/*-------------------ÍÓÂÝÒÇÊý¾Ý---------------------*/
//---------------------------------------------
	Gyr_Temp=IMUDataBuffer.AccF[0][0];
	
  Change_SensorData(Gyr_Temp,Change_Data);
	IMU_DATAT[Imu_Up_Num+22] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+23] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+24] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+25] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+26] = ',';
//---------------------------------------------
	Gyr_Temp=IMUDataBuffer.AccF[0][1];
		
	Change_SensorData(Gyr_Temp,Change_Data);
	IMU_DATAT[Imu_Up_Num+27] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+28] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+29] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+30] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+31] = ',';	

	//---------------------------------------------
	Gyr_Temp=IMUDataBuffer.AccF[0][2];
	
  Change_SensorData(Gyr_Temp,Change_Data);
	IMU_DATAT[Imu_Up_Num+32] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+33] = Change_Data[1];
	IMU_DATAT[Imu_Up_Num+34] = Change_Data[2];
	IMU_DATAT[Imu_Up_Num+35] = Change_Data[3];
	IMU_DATAT[Imu_Up_Num+36] = ',';
/*-------------------Ê±¼äÐòÁÐºÅ---------------------*/
	IMU_DATAT[Imu_Up_Num+37] = 0x30;
	IMU_DATAT[Imu_Up_Num+38] = Sys_Status->imu_get_count+0x30;
	IMU_DATAT[Imu_Up_Num+39] = '*';
/*-------------------Ð£ÑéÂë-------------------------*/
	
	CheckNumData    = ProcessCheckIMU(Imu_Up_Num+40,IMU_DATAT);
    ProcessCheckToAIMU(CheckNumData,Change_Data);
	IMU_DATAT[Imu_Up_Num+40] = Change_Data[0];
	IMU_DATAT[Imu_Up_Num+41] = Change_Data[1];
/*-------------------»Ø³µ»»ÐÐ-----------------------*/	
  IMU_DATAT[Imu_Up_Num+42] = '\n';	          
  IMU_DATAT[Imu_Up_Num+43] = 0;	

  	for(i=0;i<44;i++)
      IMUDATA[Sys_Status->imu_get_count][i] = IMU_DATAT[i];
  //printf("%s",IMU_DATAT);
}


void deal_buf(void)
{
	int i = 0;
	for(;i<sensor_buf_length-1;i++)
	{
		sensor_buf[i].sensor_sys_time = sensor_buf[i+1].sensor_sys_time;
		sensor_buf[i].axyz[0] = sensor_buf[i+1].axyz[0];
		sensor_buf[i].axyz[1] = sensor_buf[i+1].axyz[1];
		sensor_buf[i].axyz[2] = sensor_buf[i+1].axyz[2];
		sensor_buf[i].gxyz[0] = sensor_buf[i+1].gxyz[0];
		sensor_buf[i].gxyz[1] = sensor_buf[i+1].gxyz[1];
		sensor_buf[i].gxyz[2] = sensor_buf[i+1].gxyz[2];
	}
}

void save_imu_data(SYSTEM_STATUS *Sys_Status)
{
	int sensor_buf_count = Sys_Status->sensor_buf_count;

	if(sensor_buf_count < sensor_buf_length)
	{
		sensor_buf[sensor_buf_count].sensor_sys_time = sensor_date.sensor_sys_time;
		sensor_buf[sensor_buf_count].axyz[0] = sensor_date.axyz[0];
		sensor_buf[sensor_buf_count].axyz[1] = sensor_date.axyz[1];
		sensor_buf[sensor_buf_count].axyz[2] = sensor_date.axyz[2];
		sensor_buf[sensor_buf_count].gxyz[0] = sensor_date.gxyz[0];
		sensor_buf[sensor_buf_count].gxyz[1] = sensor_date.gxyz[1];
		sensor_buf[sensor_buf_count].gxyz[2] = sensor_date.gxyz[2];
		Sys_Status->sensor_buf_count++;
	}
	else
	{
		Sys_Status->full_flag = 1;
		deal_buf();
		sensor_buf[sensor_buf_length-1].sensor_sys_time = sensor_date.sensor_sys_time;
		sensor_buf[sensor_buf_length-1].axyz[0] = sensor_date.axyz[0];
		sensor_buf[sensor_buf_length-1].axyz[1] = sensor_date.axyz[1];
		sensor_buf[sensor_buf_length-1].axyz[2] = sensor_date.axyz[2];
		sensor_buf[sensor_buf_length-1].gxyz[0] = sensor_date.gxyz[0];
		sensor_buf[sensor_buf_length-1].gxyz[1] = sensor_date.gxyz[1];
		sensor_buf[sensor_buf_length-1].gxyz[2] = sensor_date.gxyz[2];
	}
}

int find_next(SYSTEM_STATUS *Sys_Status)
{
	int i;
	//unsigned long temp_time;

	for(i=0;i<sensor_buf_length;i++)
		if(sensor_buf[i].sensor_sys_time>Sys_Status->current_time)
			return i;

	return -1;
}

/*
void packet_imu(void)
{
	int i,pos;

	for(i=0;i<imu_get_count;i++)
	{
		pos = find_next();
		if(pos != -1)
			current_time = sensor_buf[pos].sensor_sys_time;
		else
		{
			printf("______________--ERROR_______________\n");
			break;
		}
		time[i] = current_time;
	}

	printf("imu_packet_time:_____%ld_____%ld_____%ld_____%ld_____%ld\n",time[0],time[1],time[2],time[3],time[4]);
}
*/

void Sys_Status_Init(SYSTEM_STATUS *Sys_Status)
{
	Sys_Status->current_time = 0;
	Sys_Status->full_flag = 0;
	Sys_Status->imu_get_count = 0;
	Sys_Status->sensor_buf_count = 0;
	Sys_Status->pos = 0;
}

char begin_flag;
char GNSS_flag;

static void mt_device_usleep(int usec)
{
    /* user-written implementation-specific source code */
    vTaskDelay((usec + 999) / 1000 / portTICK_RATE_MS); // sleep in 1 ms unit
    // LOGI("\nCalled mt_device_usleep=%dusec)", usec);
}

uint8_t Length(uint8_t *head)
{
	uint8_t n=0;
    
    while(head[n])
    {
       n++;
    }
    return n;
}



static void raw_data_process_msg_handler(fusion_alg_msg_struct_t *message,SYSTEM_STATUS *Sys_Status)
{
	//int pos;
    int i,blank_count = 0;;
	 // int count_j=1;
    if(!message)
    {
        return;
    }
    switch(message->message_id)
    {
        case MESSAGE_ID_RAW_SENSOR_DATA:
			bmi160_date_get();
            //alg_sensor_data = (RawSensorData*)message->param2;
			save_imu_data(Sys_Status);
			Sys_Status->imu_get_count++;
			//if(imu_get_count == 5 && full_flag)
			//{
			//	packet_imu();
			//	imu_get_count = 0;
			//}
			if(Sys_Status->full_flag || begin_flag)
			{
				Sys_Status->pos = find_next(Sys_Status);
				Sys_Status->current_time = sensor_buf[Sys_Status->pos].sensor_sys_time;
				//printf("===========wells==========MESSAGE_ID_RAW_SENSOR_DATA:%ld\n",current_time);
				Analyse_IMUData(Sys_Status);
				IMUTime_Deal(Sys_Status,Sys_Status->imu_get_count);
			    IMUData_Deal(Sys_Status);
				GINavProc(&GINavResult);
				if(Sys_Status->imu_get_count == 4 && GNSS_flag)
				{
					GNSS_flag = 0;

					//delay(500);

					printf("%s",alg_gnss_data.girmc_buf);
					mt_device_usleep(100);
					printf("%s",alg_gnss_data.gigga_buf);
					mt_device_usleep(100);
					printf("%s",alg_gnss_data.gigst_buf);
					mt_device_usleep(100);

					GetGPRMC(alg_gnss_data.gprmc_data);
					mt_device_usleep(100);
					GetLGPGGA(alg_gnss_data.gpgga_data);
					mt_device_usleep(100);
					
					printf("%s",IMUTIME[2]);
					mt_device_usleep(100);
					printf("%s",IMUDATA[1]);
					mt_device_usleep(100);
					printf("%s",IMUTIME[3]);
					mt_device_usleep(100);
					printf("%s",IMUDATA[2]);
					mt_device_usleep(100);
					printf("%s",IMUTIME[4]);
					mt_device_usleep(100);
					printf("%s",IMUDATA[3]);
					mt_device_usleep(100);
					printf("%s",IMUTIME[5]);
					mt_device_usleep(100);
					printf("%s",IMUDATA[4]);
					mt_device_usleep(100);
				}
			}
            break;
		case MESSAGE_ID_RAW_GPS_DATA:
			GNSS_flag = 1;
			//inject_raw_gps_data((raw_sensor_data*) message->param2);
			begin_flag = 1;
			Sys_Status->imu_get_count = 0;
			alg_gnss_data = *(GNSSData *)(message->param2);
		    Sys_Status->current_time = alg_gnss_data.gnss_sys_time;
			alg_gnss_data.gigga_buf[2] = 'I';
			alg_gnss_data.girmc_buf[2] = 'I';
			alg_gnss_data.gigst_buf[2] = 'I';

			for(i = Length(alg_gnss_data.gigga_buf) - 1;alg_gnss_data.gigga_buf[i] == 0x0a;i--)
				blank_count++;
			if(blank_count)
				alg_gnss_data.gigga_buf[Length(alg_gnss_data.gigga_buf) - blank_count +1] = 0;

			blank_count = 0;

			for(i = Length(alg_gnss_data.girmc_buf) - 1;alg_gnss_data.girmc_buf[i] == 0x0a;i--)
				blank_count++;
			if(blank_count)
				alg_gnss_data.girmc_buf[Length(alg_gnss_data.girmc_buf) - blank_count +1] = 0;
			/*
			blank_count = 0;

			for(i = Length(alg_gnss_data.gigst_buf) - 1;alg_gnss_data.gigst_buf[i] == 0x0a;i--)
				blank_count++;
			if(blank_count)
				alg_gnss_data.gigst_buf[Length(alg_gnss_data.gigst_buf) - blank_count +1] = 0;
			*/
			for(i = 0;alg_gnss_data.gigst_buf[i];i++)
				if(alg_gnss_data.gigst_buf[i] == '*')
				{
					alg_gnss_data.gigst_buf[i+3] = 0x0a;
					alg_gnss_data.gigst_buf[i+4] = 0;
					
				}
			
			Analyse_GpsData();
			//GetLGPGGA(alg_gnss_data->gpgga_data);
			//GetGPRMC(alg_gnss_data->gprmc_data);
			//printf("gnss_sys_time:%ld,hour:%d,Minute:%d,Second:%d,MSecond:%ld\n",alg_gnss_data->gnss_sys_time,alg_gnss_data->gprmc_data.Hour,alg_gnss_data->gprmc_data.Minute,alg_gnss_data->gprmc_data.Second,alg_gnss_data->gprmc_data.MSecond);
			//printf("Latitude:%d,BLatitude:%lf,SNth:%c,Longitude:%lf,BLongitude:%lf,WEst:%c\n",(int)alg_gnss_data->gpgga_data.Longitude,alg_gnss_data->gpgga_data.BLongitude,alg_gnss_data->gpgga_data.SNth,alg_gnss_data->gpgga_data.Latitude,alg_gnss_data->gpgga_data.BLongitude,alg_gnss_data->gpgga_data.WEst);
			break;
        default :
            break;
    }
}
// 发送消息到 fusion alg task
int32_t fusion_alg_send_event(fusion_alg_msg_status_id event_id, int32_t param1, void* param2)
{
    fusion_alg_msg_struct_t message;
	TickType_t ticks;
    message.message_id = event_id;
    message.param1 = param1;
    message.param2 = param2;
	ticks = 1000 / portTICK_PERIOD_MS;
    return xQueueSend(raw_data_queue_handle, &message, ticks);
}

void fusion_alg_mgr_task(void * arg)
{
    fusion_alg_msg_struct_t fusion_alg_event_data_item;
	SYSTEM_STATUS Sys_Status;

	Sys_Status_Init(&Sys_Status);
	GINavInit();

    while(1)
    {
        if (xQueueReceive(raw_data_queue_handle, &fusion_alg_event_data_item, portMAX_DELAY))
        {
            raw_data_process_msg_handler(&fusion_alg_event_data_item,&Sys_Status);
        }
    }
}

int32_t fusion_alg_manager_init(void)
{
    raw_data_queue_handle = xQueueCreate(FUSION_MGR_QUEUE_SIZE,sizeof(fusion_alg_msg_struct_t));
    xTaskCreate(fusion_alg_mgr_task, FUSION_ALG_MANAGER_TASK_NAME, FUSION_ALG_MANAGER_TASK_STACKSIZE / sizeof(StackType_t), NULL, FUSION_ALG_MANAGER_TASK_PRIO,&fusion_alg_manager_task_handle);
    return 1;
}
