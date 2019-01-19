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
#include "stdio.h"

#include "stdlib.h"
#include "config.h"
#include "DataProc.h"
#include "GetAcc.h"

#define sensor_buf_length 10

extern RawSensorData sensor_buf[sensor_buf_length];
extern GNSSData alg_gnss_data;

extern IMU_DATA_T    IMUDataBuffer;	 //gty IMUËùÓÐÖ¸Õë¾ùÖ¸Ïò¸ÃµØÖ·....

extern BOOL IMUDataReady;

void Get_Acc_XYZ(int16_t ACC_X,int16_t ACC_Y,int16_t ACC_Z);

void Get_UTCTime(SYSTEM_STATUS *Sys_Status)
{
	uint8_t   IMDHour;						
	uint8_t   IMDMinute;					
	uint8_t   IMDSecond;					
	uint16_t  IMDMSecond;    

	IMDHour		 = alg_gnss_data.gpgga_data.Hour;
	IMDMinute	 = alg_gnss_data.gpgga_data.Minute;
	IMDSecond	 = alg_gnss_data.gpgga_data.Second;
	IMDMSecond   = alg_gnss_data.gpgga_data.MSecond;

	IMDMSecond = IMDMSecond + (Sys_Status->imu_get_count - 1)*50;

	if(IMDMSecond==1000)
	{
		IMDMSecond = 0;

		IMDSecond++;
		if(IMDSecond==60)
		{
			IMDSecond=0;
			IMDMinute++;
			if(IMDMinute==60)
			{
				IMDMinute=0;
				IMDHour++;
				if(IMDHour==24)
				{
					IMDHour=0;
				}
			}
		}
	}

	IMUDataBuffer.UtcTime.Year   = alg_gnss_data.gprmc_data.Year;
	IMUDataBuffer.UtcTime.Month = alg_gnss_data.gprmc_data.Month;
	IMUDataBuffer.UtcTime.Day   = alg_gnss_data.gprmc_data.Day;
	
	IMUDataBuffer.UtcTime.Hour = IMDHour;
	IMUDataBuffer.UtcTime.Minute = IMDMinute;
	IMUDataBuffer.UtcTime.Second = IMDSecond;
	IMUDataBuffer.UtcTime.MillSecond = IMDMSecond;
}

void Analyse_IMUData(SYSTEM_STATUS *Sys_Status)
{
	uint8_t  pos = Sys_Status->pos;

	Get_UTCTime(Sys_Status);

	IMUDataBuffer.MsrInterval  = INSUPDATE_SUBDATA_INTERVAL;		
	//-------------------------------

	IMUDataBuffer.Gyro[0][0]  = sensor_buf[pos].gxyz[0];
	IMUDataBuffer.Gyro[0][1]  = sensor_buf[pos].gxyz[1];
	IMUDataBuffer.Gyro[0][2]  = sensor_buf[pos].gxyz[2];

	Get_Acc_XYZ(sensor_buf[pos].axyz[0],sensor_buf[pos].axyz[1],sensor_buf[pos].axyz[2]);

	IMUDataBuffer.GyroF[0][0] = IMUDataBuffer.Gyro[0][0];
	IMUDataBuffer.GyroF[0][1] = IMUDataBuffer.Gyro[0][1];
	IMUDataBuffer.GyroF[0][2] = IMUDataBuffer.Gyro[0][2];
				
	IMUDataBuffer.AccF[0][0] = IMUDataBuffer.Acc[0][0];
	IMUDataBuffer.AccF[0][1] = IMUDataBuffer.Acc[0][1];
	IMUDataBuffer.AccF[0][2] = IMUDataBuffer.Acc[0][2];

	IMUDataBuffer.Gyro[0][0]  = IMUDataBuffer.Gyro[0][0]*GYRO_SCALE;  //½Ç¶ÈÔöÁ¿
	IMUDataBuffer.Gyro[0][1]  = IMUDataBuffer.Gyro[0][1]*GYRO_SCALE;
	IMUDataBuffer.Gyro[0][2]  = IMUDataBuffer.Gyro[0][2]*GYRO_SCALE;
		
    IMUDataBuffer.Acc[0][0]   = IMUDataBuffer.Acc[0][0]*ACC_SCALE;   //ËÙ¶ÈÔöÁ¿
	IMUDataBuffer.Acc[0][1]   = IMUDataBuffer.Acc[0][1]*ACC_SCALE;
	IMUDataBuffer.Acc[0][2]   = IMUDataBuffer.Acc[0][2]*ACC_SCALE;

	IMUDataReady = True;
}


//---------------------------------

//---------------------------------
void Get_Acc_XYZ(int16_t ACC_X,int16_t ACC_Y,int16_t ACC_Z)
{   
   float   floPitchAcc;
   float   floRollAcc;
   float   floAliAcc;

   int16_t intPitchAcc;	  
   int16_t intRollAcc; 
   int16_t intAliAcc;
	
   floPitchAcc = ConvertorPAcc(ACC_X,0);
   floRollAcc  = ConvertorRAcc(ACC_Y,0);
   floAliAcc   = ConvertorZAcc(ACC_Z,0);

   IMUDataBuffer.Acc[0][0] = (FLOAT64)Checkout_p2g(floPitchAcc*1000);	  
   IMUDataBuffer.Acc[0][1]  = (FLOAT64)Checkout_r2g(floRollAcc*1000);      
   IMUDataBuffer.Acc[0][2]   = (FLOAT64)Checkout_a2g(floAliAcc*1000);    

   //printf("----------%d--------%d--------%d\n",(int)IMUDataBuffer.Acc[0][0],(int)IMUDataBuffer.Acc[0][1],(int)IMUDataBuffer.Acc[0][2]);

    //printf("%d   ",intPitchAcc);
    //printf("%d   ",intRollAcc);
    //printf("%d\n",intAliAcc);
}

//--------------------------------------------------

//--------------------------------------------------
 float ConvertorPAcc(int16_t AccData,char Reverse) 
 {
  float AcceratorData; 
  float TAcce; 
  float Tbias,Zbias,Fbias; 
  float Zscale,Fscale,ZZscale,ZFscale;  

  int16_t XAccBias=0;
  int16_t XAccScale    = 1024;
  int16_t XAccFScale   = 1024;
  

  TAcce = AccData;
  Tbias = XAccBias; 

  Zscale  = XAccScale;
  Fscale  = XAccFScale;

 
  if(TAcce>Tbias)
	{
    AcceratorData = (TAcce-Tbias)/Zscale;	
	}
	else
	{	 	
	  AcceratorData = (TAcce-Tbias)/Fscale;	 
	}
  // printf("_________________________ConvertorPAcc______________\n");
   return 	 AcceratorData;
}
 
//--------------------------------------------------

//--------------------------------------------------
 float ConvertorRAcc(int16_t AccData,char Reverse) 
 {
  float AcceratorData; 
  float TAcce; 
  float Tbias,Zbias,Fbias; 
  float Zscale,Fscale,ZZscale,ZFscale;  

  int16_t YAccBias=0;
  int16_t YAccScale    = 1024;	
  int16_t YAccFScale   = 1024;


  TAcce = AccData;
  Tbias = YAccBias;   
	 
  Zscale  = YAccScale;
  Fscale  = YAccFScale;
 
  if(TAcce>Tbias)
	{
    AcceratorData = (TAcce-Tbias)/Zscale;	
	}
	else
	{	 	
	  AcceratorData = (TAcce-Tbias)/Fscale;	 
	}

  if(Reverse)
	{
	  AcceratorData = -AcceratorData;
	}
	//printf("_________________________ConvertorRAcc______________\n");
   return 	 AcceratorData;
}

//--------------------------------------------------

//--------------------------------------------------
 float ConvertorZAcc(int16_t AccData,char Reverse) 
 {
  float AcceratorData;

	float TAcce;
	float Tbias;
	float Zscale,Fscale; 

	int16_t	ZAccBias=0;
	int16_t ZAccScale    = 1024;
    int16_t ZAccFScale   = 1024;
  
 	TAcce=AccData;
	Tbias = ZAccBias;
	 
	Zscale  = ZAccScale;	
	Fscale  = ZAccFScale;

   
  if(TAcce>Tbias)
	{
   AcceratorData = (TAcce-Tbias)/Zscale;	
	}
	else
	{	 	
	 AcceratorData = (TAcce-Tbias)/Fscale;	 
	}

  if(Reverse)
	{
	 AcceratorData =-AcceratorData;
	}
   //printf("_________________________ConvertorZAcc______________\n");
   return AcceratorData;    
}


 //-----------------------------------------

 //------------------------------------------
 int16_t Checkout_r2g(int16_t g_data)
 {
  static int8_t First=1;
  static int16_t g_data_Back;
 
  if(First)
  {
   First=0;
  }
  else
  {
	if(g_data>2000)
	  g_data=g_data_Back;
	else if(g_data<-2000)
	  g_data=g_data_Back;  
  }
 
   g_data_Back=g_data;
 
	return g_data ;
 }
 
 //-----------------------------------------

 //------------------------------------------
 int16_t Checkout_p2g(int16_t g_data)
 {
  static int8_t First=1;
  static int16_t g_data_Back;
 
  if(First)
  {
   First=0;
  }
  else
  {
	if(g_data>2000) 		 
	  g_data=g_data_Back;
	else if(g_data<-2000)
	  g_data=g_data_Back;  
  }
 
   g_data_Back=g_data;
 
	return g_data ;
 }
 
 //-----------------------------------------

 //------------------------------------------
 int16_t Checkout_a2g(int16_t g_data)
 {
  static int8_t First=1;
  static int16_t g_data_Back;
 
  if(First)
  {
   First=0;
  }
  else
  {
	if(g_data>2000)
	  g_data=g_data_Back;
	else if(g_data<-2000)
	  g_data=g_data_Back;  
  }
 
   g_data_Back=g_data;
 
	return g_data ;
 }


