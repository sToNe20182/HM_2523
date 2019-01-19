/* Copyright Statement:
 *
 * (C) 2005-2016  MediaTek Inc. All rights reserved.
 *
 * This software/firmware and related documentation (MediaTek Software) are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. (MediaTek) and/or its licensors.
 * Without the prior written permission of MediaTek and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 * You may only use, reproduce, modify, or distribute (as applicable) MediaTek Software
 * if you have agreed to and been bound by the applicable license agreement with
 * MediaTek (License Agreement) and been granted explicit permission to do so within
 * the License Agreement (Permitted User).  If you are not a Permitted User,
 * please cease any access or use of MediaTek Software immediately.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT MEDIATEK SOFTWARE RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES
 * ARE PROVIDED TO RECEIVER ON AN AS-IS BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */



#include <string.h>
#include <stdlib.h>
#include "stdio.h"
#include "gnss_api.h"
#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//#include "utils.h"
#include "gnss_app.h"
#include "event_groups.h"
#include "ring_buf.h"
#include "gnss_timer.h"
#include "gnss_log.h"
#include "task_def.h"
#include "fusion_alg_interface_api.h"

#ifdef GNSS_SUPPORT_TOOL_BRIDGE
#include "gnss_bridge.h"
#endif

//************************* Customize Configration Begin **************************/

#define GNSS_SEND_RING_BUF_SIZE (4*1024)
#define GNSS_RECIEVE_RING_BUF_SIZE (16*1024)
#define GNSS_SEND_TEMP_BUF 256
#define GNSS_RECIEVE_TEMP_BUF 256
#define GNSS_DATA_RECIVE_BIT (1 << 0)
#define GNSS_DATA_SEND_BIT (1 << 1)
#define GNSS_NEMA_SENTENCE_BUF_LEN 256
#define GNSS_QUEUE_SIZE 10

#define GNSS_LOW_POWER_MODE_NORMAL 60
#define GNSS_LOW_POWER_MODE_ON_OFF (5 * 60)
#define GNSS_LOW_POWER_ON_OFF_MODE_GAP 60

#define GNSS_VALID_ACCURACY 150

#define GNSS_WAIT_SEND_BIT 0x01
#define GNSS_WAIT_SEND_BY_OTHER_TASK 0x02

//************************** Customize Configration End ****************************/


static struct {
    QueueHandle_t gnss_task_queue;
    EventGroupHandle_t gnss_event_group;
    ring_buf_struct_t recieve_ring_buf;
    ring_buf_struct_t send_ring_buf;
} gnss_task_cntx;

/*GNSS data structure */

typedef struct{
    int32_t periodic;
    gnss_location_handle handle;
} gnss_config_struct_t;

typedef enum {
    GNSS_LOCATION_STATE_WAITING_INIT,
    GNSS_LOCATION_STATE_INIT,
    GNSS_LOCATION_STATE_START,
    GNSS_LOCATION_STATE_STOP
} gnss_location_state_enum_t;

typedef enum {
    GNSS_LOCATION_MODE_NONE_PERIODIC,
    GNSS_LOCATION_MODE_NORMAL_PERIODIC,
    GNSS_LOCATION_MODE_LLE,
    GNSS_LOCATION_MODE_ON_OFF
} gnss_location_mode_t;

typedef struct {
    gnss_sentence_info_t nmea_sentence;
    gnss_location_struct_t location_data;
    gnss_config_struct_t config;
    gnss_location_state_enum_t state;
    int32_t on_off_mode_timer_id;
    int32_t periodic_timer_id;
    gnss_location_mode_t mode;
} gnss_context_struct_t;
int32_t rx_data_ready = 1;

static GNSSData gnss_data;


static TaskHandle_t gnss_app_task_handle;
static gnss_context_struct_t g_gnss_location_context;
extern void epo_demo_send_assistance_data(int iYr, int iMo, int iDay, int iHr);
/**
* @brief       Parse location from NMEA sentence.
* @param[in]   input: NMEA sentence
* @param[in]   output: location
* @return      int32_t
*/
static int32_t gnss_get_location (gnss_sentence_info_t *input, gnss_location_struct_t *output)
{
    uint8_t *gpgga = input->GPGGA;
    uint8_t *gpacc = input->GPACC;
    int32_t i = 0;
    int32_t j = 0;
    int32_t flag = 0;
    int8_t accuracy[10] = {0};
    output->accuracy = 0xffff;
    output->latitude[0] = '\0';
    output->longitude[0] = '\0';

    memcpy(&output->nmea_sentence, input, sizeof(gnss_sentence_info_t));
    for (i = 0; i < GNSS_MAX_GPGGA_SENTENCE_LENGTH; i++) {
        if (gpgga[i] == ',') {
            if (gpgga[i + 1] == 'N') {
                int32_t len = i - j - 1;
                output->latitude[len] = 0;
                while (len > 0) {
                    output->latitude[len - 1] = gpgga[j + len];
                    len--;
                }
            }
            if (gpgga[i + 1] == 'E') {
                int32_t len = i - j - 1;
                if (gpgga[i + 3] == '0') {
                    GNSSLOGD("[GNSS Demo] Get location, invalid data!\n");
                    output->longitude[0] = 0;
                    output->latitude[0] = 0;
                    return 1;
                }
                output->longitude[len] = 0;
                while (len > 0) {
                    output->longitude[len - 1] = gpgga[j + len];
                    len--;
                }
                break;
            }
			if (gpgga[i + 1] == 'S') {
                int32_t len = i - j - 1;
				output->latitude[0] = '-';
                output->latitude[len + 1] = 0;
                while (len > 0) {
                    output->latitude[len] = gpgga[j + len];
                    len--;
                }
            }
            if (gpgga[i + 1] == 'W') {
                int32_t len = i - j - 1;
                if (gpgga[i + 3] == '0') {
                    GNSSLOGD("[GNSS Demo] Get location, invalid data!\n");
                    output->longitude[0] = 0;
                    output->latitude[0] = 0;
                    return 1;
                }
				output->longitude[0] = '-';
                output->longitude[len + 1] = 0;
                while (len >= 0) {
                    output->longitude[len] = gpgga[j + len];
                    len--;
                }
                break;
            }
            j = i;            
        }
    }
    if (output->latitude[0] == 0 && output->longitude[0] == 0) {
        return 1;
    }

    j = 0;
    for (i = 0; i < GNSS_MAX_GPACC_SENTENCE_LENGHT; i++) {
        if (gpacc[i] == '*') {
            accuracy[j] = '\0';
            break;
        } else if (flag) {
            accuracy[j++] = gpacc[i];
        } else if (gpacc[i] == ',') {
            flag = 1;
        }
    }

    output->accuracy = atoi((char*) accuracy);
    GNSSLOGD("[GNSS Demo] Get location, latitude:%s, longitude:%s, accuracy:%d\n", output->latitude, output->longitude, (int) output->accuracy);
    return 0;

}

/**
* @brief This function is used to process PMTK command, especially for PMTK response about 663.
* @param[in] nmea_buf: is point to a buffer which contains NMEA sentence.
* @param[in] buf_len: NMEA sentence length.
* @param[in] nmea_param: Parsed parameter list
* @param[in] buf_len: Parmeter number.
* @return void
*/
static void gnss_get_sentence_param(int8_t* nmea_buf, int32_t buf_len, int8_t* nmea_param[], int32_t* param_num)
{
    int32_t ind = 0;
    int32_t ret_num = 0;
    while(ind < buf_len) {
        if (nmea_buf[ind] == ',') {
            nmea_param[ret_num++] = nmea_buf + ind + 1;
            nmea_buf[ind] = 0;
        } else if (nmea_buf[ind] == '*') {
            nmea_buf[ind] = 0;
            break;
        }
        ind ++;
    }
    *param_num = ret_num;
}

/**
* @brief This function is used to process PMTK command, especially for PMTK response about 663.
* @param[in] nmea_buf: is point to a buffer which contains NMEA sentence.
* @param[in] buf_len: NMEA sentence length.
* @return void
*/
static void gnss_process_pmtk_response(int8_t*nmea_buf, int32_t buf_len)
{
    int8_t* nmea_param[60];
    int32_t param_num;

    gnss_get_sentence_param(nmea_buf, buf_len, nmea_param, &param_num);

    if ( strstr((char*) nmea_buf, "PMTK001") ) {
        if ( strstr((char*) nmea_param[0], "663") ) {
            if (atoi((char*) nmea_param[1]) == 3) {
                //
                if (atoi((char*) nmea_param[2]) == 0) {
                    //time ading
                }
                if (atoi((char*) nmea_param[3]) == 0) {
                    // loc ading
                }
                if (atoi((char*) nmea_param[4]) < 4 || atoi((char*) nmea_param[5]) < 4) {
                    // epo ading

                }
            }
        }
    }
}

#define NP_MAX_DATA_LEN			180		// maximum data length
#define MAXFIELD	25


uint8_t GetField(uint8_t *pData, uint8_t *pField, int32_t nFieldNum, int32_t nMaxFieldLen)
{
	
	int32_t    Get_i;
  int32_t    Get_i2;
  int32_t    Get_nField;

	//
	// Validate params
	//
	if(pData == NULL || pField == NULL || nMaxFieldLen <= 0)
	{
		return FALSE;		   //º¯ÊýÁ¢¼´½áÊø£¬·µ»Ø0
	}

	//
	// Go to the beginning of the selected field
	//
	Get_i = 0;
	Get_nField = 0;
	while(Get_nField != nFieldNum && pData[Get_i])	//Ö±µ½nfiledµ½´ïnFieldNum£¬while½áÊø,nFieldNum¾ÍÊÇ¸ø³öµÄÒªµ½´ïµÄ×Ö¶ÎµÄÐòºÅ
	{
		if(pData[Get_i] == ',')		            //Óöµ½¶ººÅËµÃ÷ÒÑ¾­Ìø¹ýÒ»¸ö×Ö¶Î£¬nField×Ô¼Ó1
		{
			Get_nField++;
		}

		Get_i++;
		
			
		if(Get_i >=NP_MAX_DATA_LEN)               //Èç¹ûGet_i³¬¹ýÊý×é³¤¶È£¬Ôò....ÍË³ö...20140729.....
		{
	    pField[0]  = '\0';			//ÎÞÊýÖµ£¬ÄÇÃ´pField[0]= ×Ö·û´®½áÊø			
			return FALSE;			   	//returnÍË³öµ±Ç°º¯Êý		
    }

		if(pData[Get_i] == NULL)
		{
			pField[0] = '\0';			//ÎÞÊýÖµ£¬ÄÇÃ´pField[0]= ×Ö·û´®½áÊø
			return FALSE;			   	//returnÍË³öµ±Ç°º¯Êý
		}
	}

	if(pData[Get_i] == ',' || pData[Get_i] == '*')
	{
		pField[0] = '\0';			   //ÈôÖ¸¶¨×Ö¶ÎÒ²ÎÞÊýÖµ£¬Îª,»òÕß*ºÅÔò£¬
		return FALSE;				   //Èç¹ûÌø³öwhileºóµÄµÚÒ»¸ö×Ö·û»¹ÊÇ,»òÕß*£¬·µ»Ø´íÎó
	}

	//
	// copy field from pData to Field
	//
	Get_i2 = 0;
	while(pData[Get_i] != ',' && pData[Get_i] != '*' && pData[Get_i])  //ÈôÖ¸¶¨×Ö¶ÎÓÐÊýÖµ
	{
		pField[Get_i2] = pData[Get_i];				//½«Æä´æ·ÅÓÚpDataÖÐ
		Get_i2++; Get_i++;

		//
		// check if field is too big to fit on passed parameter. If it is,
		// crop returned field to its max length.
		//
		if(Get_i2 >= nMaxFieldLen)			 //Èç¹ûi2´óÓÚ¸Ã×Ö¶ÎÓ¦ÓÐµÄ×î´ó³¤¶È£¬ÄÇÃ´ÈÃi2µÈÓÚ¸Ã×î´ó³¤¶È
		{								 //ÒòÎªÔÚ´¦ÀíÄ³Ð©×Ö¶ÎÊ±£¬×Ö¶Î»á¶à·¢Êý¾Ý£¬ÎªÁË±£Ö¤Êý¾ÝµÄ×¼È·
			Get_i2 = nMaxFieldLen-1;		 //ÕâÀï¹æ¶¨ÁËi2µÄ³¤¶È£¬Ò²¾Í¹æ¶¨ÁË×îºó·µ»Ø×Ö¶ÎµÄ³¤¶È
			break;
		}
		
		if(Get_i >=NP_MAX_DATA_LEN)               //Èç¹ûGet_i³¬¹ýÊý×é³¤¶È£¬Ôò....ÍË³ö...20140729.....
		{
	    pField[0]  = '\0';			//ÎÞÊýÖµ£¬ÄÇÃ´pField[0]= ×Ö·û´®½áÊø			
			return FALSE;			   	//returnÍË³öµ±Ç°º¯Êý		
    }
	}
	pField[Get_i2] = '\0';

	return TRUE;						 //Èç¹ûÕû¸öº¯ÊýÔËÐÐÍê³É£¬×Ö¶Î·µ»Ø³É¹¦£¬ÄÇÃ´·µ»Øtrue=1
}

uint8_t DataJudge(uint8_t *pFieldData)
{
	uint8_t D_i=0,PDataT=0,DFlag=1;
		
	for(D_i=0;D_i<MAXFIELD;D_i++)
	{
		PDataT=pFieldData[D_i];
		
	 if(PDataT!='\0')
	 {
    if(((PDataT>=0x30)&&(PDataT<=0x39))||(PDataT=='.'))
	  {
     DFlag=1;
    }
	  else
	  {
     DFlag=0;
		 break;
    }
   }
	 else
	 {
		 if(D_i==0)
		 {
      DFlag=0;
     }		 
    break;
   }
 } 
 return  DFlag;	
}

void ProcessGPGGA(uint8_t *pData)
{
uint8_t     GGApField[MAXFIELD];
char        GGApBuff[MAXFIELD];
	
static  uint8_t Gps_GError_Flag=0;
	//
	// Time
	//
	if(GetField(pData, GGApField, 1, MAXFIELD))
	{
		//GNSSLOGD("___________%c_________",GGApField[0]);
		// Hour
		GGApBuff[0] = GGApField[0];
		GGApBuff[1] = GGApField[1];
		GGApBuff[2] = '\0';
		gnss_data.gpgga_data.Hour = atoi(GGApBuff);		

		// minute
		GGApBuff[0] = GGApField[2];
		GGApBuff[1] = GGApField[3];
		GGApBuff[2] = '\0';
		gnss_data.gpgga_data.Minute = atoi(GGApBuff);

		// Second
		GGApBuff[0] = GGApField[4];
		GGApBuff[1] = GGApField[5];
		GGApBuff[2] = '\0';
		gnss_data.gpgga_data.Second = atoi(GGApBuff);

		// Second
		GGApBuff[0] = GGApField[7];
		GGApBuff[1] = GGApField[8];
		GGApBuff[2] = GGApField[9];
		GGApBuff[3] = '\0';
		gnss_data.gpgga_data.MSecond = atoi(GGApBuff);
		
		if((gnss_data.gpgga_data.Hour==0)&&(gnss_data.gpgga_data.Minute==0)&&(gnss_data.gpgga_data.Second==0))
		{
		 gnss_data.gpgga_data.Utc_Flag=0;
		}
		else
		{
		 gnss_data.gpgga_data.Utc_Flag=1;
		}
	}
	else
	{
	  gnss_data.gpgga_data.Utc_Flag=0;
	}

	
	//
	// Latitude
	//
	if(GetField(pData, GGApField, 2, MAXFIELD))
	{
		//GNSSLOGD("___________%s_________",GGApField);
		
		if(DataJudge(GGApField))
		{
		 gnss_data.gpgga_data.Latitude  = atof((char *)GGApField+2) / 60.0;
		 GGApField[2] = '\0';
		 gnss_data.gpgga_data.Latitude += atof((char *)GGApField);		
		}
	    else
		{
	     gnss_data.gpgga_data.Latitude=0.0;
	    }
	}
    else
    {
        gnss_data.gpgga_data.Latitude=0.0;
    }

	
	if(GetField(pData, GGApField, 3, MAXFIELD))
	{
	
	   gnss_data.gpgga_data.SNth=GGApField[0];

	   if(GGApField[0] == 'S')
	   {
		  gnss_data.gpgga_data.Latitude = -gnss_data.gpgga_data.Latitude;
	   }
	}
	


	//------------------------------------------------
	// Longitude
	//------------------------------------------------
	if(GetField(pData, GGApField, 4, MAXFIELD))
	{
		if(DataJudge(GGApField))
		{
		 gnss_data.gpgga_data.Longitude = atof((char *)GGApField+3) / 60.0;
		 GGApField[3] = '\0';
		 gnss_data.gpgga_data.Longitude += atof((char *)GGApField);	
		}
		else
		{
     	gnss_data.gpgga_data.Longitude=0.0;
    }
	}
	else
	{
    gnss_data.gpgga_data.Longitude=0.0;
  }

	
	if(GetField(pData, GGApField, 5, MAXFIELD))
	{
	  gnss_data.gpgga_data.WEst= GGApField[0]; 

		if(GGApField[0] == 'W')
		{
			gnss_data.gpgga_data.Longitude = -gnss_data.gpgga_data.Longitude;
		}	 
	}

		//------------------------------------------------
	// GPS quality
	//------------------------------------------------
	if(GetField(pData, GGApField, 6, MAXFIELD))
	{
		gnss_data.gpgga_data.GPSQuality = GGApField[0] - '0';
	}
	else
	{
	  gnss_data.gpgga_data.GPSQuality  = 0;
	}


	//------------------------------------------------
	// Satellites in use
	//------------------------------------------------
	if(GetField(pData, GGApField, 7, MAXFIELD))
	{
		GGApBuff[0] = GGApField[0];
		GGApBuff[1] = GGApField[1];
		GGApBuff[2] = '\0';
		gnss_data.gpgga_data.NumOfSatsInUse = atoi(GGApBuff);
	}
	else
	{
	  gnss_data.gpgga_data.NumOfSatsInUse =0;
	}

	//------------------------------------------------
	// HDOP
    //------------------------------------------------
	if(GetField(pData, GGApField, 8, MAXFIELD))
	{
		gnss_data.gpgga_data.HDOP = atof((char *)GGApField);
	}


	//------------------------------------------------
	// Altitude
	//------------------------------------------------
	if(GetField(pData, GGApField, 9, MAXFIELD))
	{
		if(DataJudge(GGApField))
		{
		 gnss_data.gpgga_data.Altitude = atof((char *)GGApField);
		}
	}

	//------------------------------------------------
	// M
	//------------------------------------------------
	if(GetField(pData, GGApField, 10, MAXFIELD))
	{
	    gnss_data.gpgga_data.Auint= GGApField[0]; 
	}

	//------------------------------------------------
	//Geoidal
	//------------------------------------------------
	if(GetField(pData, GGApField, 11, MAXFIELD))
	{
	  gnss_data.gpgga_data.Geoidal = atof((char *)GGApField);
	}

	//------------------------------------------------
	// M
	//------------------------------------------------
	if(GetField(pData, GGApField, 12, MAXFIELD))
	{
	  gnss_data.gpgga_data.Guint= GGApField[0]; 
	}
	
	//------------------------------------------------
	//Age of Diff. Corr
	//------------------------------------------------
	if(GetField(pData, GGApField, 13, MAXFIELD))
	{
	  gnss_data.gpgga_data.DTime = atof((char *)GGApField);
		//GetAgeDataFlag  = 1;   //
	}
	
	 
	//------------------------------------------------
	//Dif. Ref. Station ID
	//------------------------------------------------
	if(GetField(pData, GGApField, 14, MAXFIELD))
	{
	  gnss_data.gpgga_data.DGpsID = atoi((char *)GGApField);	
		gnss_data.gpgga_data.IDValid=1;
		
		gnss_data.gpgga_data.DFID_Data[0] = GGApField[0];
		gnss_data.gpgga_data.DFID_Data[1] = GGApField[1];
		gnss_data.gpgga_data.DFID_Data[2] = GGApField[2];
		gnss_data.gpgga_data.DFID_Data[3] = GGApField[3];
	}
	else
	{
    	gnss_data.gpgga_data.IDValid=0;
		gnss_data.gpgga_data.DFID_Data[0] ='0';
		gnss_data.gpgga_data.DFID_Data[1] ='0';
		gnss_data.gpgga_data.DFID_Data[2] ='0';
		gnss_data.gpgga_data.DFID_Data[3] ='0';	
		
  }	  

   
	 

//  if(Gps_GError_Flag==0)
   {
 	  gnss_data.gpgga_data.Count++;
	  //GGA_Get_Flag  = 1;                 //»ñµÃÊý¾Ý---´«ÊäÊý¾Ý  
   }
  
   Gps_GError_Flag = 0;                 //ÇåÁã
}

void ProcessGPRMC(uint8_t *pData)
{
uint8_t     RMCpField[MAXFIELD];
char        RMCpBuff[MAXFIELD];
static  uint8_t Gps_GError_Flag=0;
	//
	// Time
	//
	if(GetField(pData, RMCpField, 1, MAXFIELD))
	{
		// Hour
		RMCpBuff[0] = RMCpField[0];
		RMCpBuff[1] = RMCpField[1];
		RMCpBuff[2] = '\0';
		gnss_data.gprmc_data.Hour = atoi(RMCpBuff);

		// minute
		RMCpBuff[0] = RMCpField[2];
		RMCpBuff[1] = RMCpField[3];
		RMCpBuff[2] = '\0';
		gnss_data.gprmc_data.Minute = atoi(RMCpBuff);

		// Second
		RMCpBuff[0] = RMCpField[4];
		RMCpBuff[1] = RMCpField[5];
		RMCpBuff[2] = '\0';
		gnss_data.gprmc_data.Second = atoi(RMCpBuff);
		
	 // MSecond
		RMCpBuff[0] = RMCpField[7];
		RMCpBuff[1] = RMCpField[8];
		RMCpBuff[2] = RMCpField[9];
		RMCpBuff[3] = '\0';
		gnss_data.gprmc_data.MSecond = atoi(RMCpBuff);
		
		gnss_data.gprmc_data.TimeValid=1;
	}
	else
	{
	  gnss_data.gprmc_data.TimeValid=0;
	}

	//
	// Data valid
	//
	if(GetField(pData, RMCpField, 2, MAXFIELD))
	{
		gnss_data.gprmc_data.DataValid = RMCpField[0];
	}
	
	//
	// latitude
	//
	if(GetField(pData, RMCpField, 3, MAXFIELD))
	{

		if(DataJudge(RMCpField))
		{
		 gnss_data.gprmc_data.Latitude = atof((char *)RMCpField+2) / 60.0;    //°Ñ×Ö·û´®×ª»»³É¸¡µãÊý
		 RMCpField[2] = '\0';
		 gnss_data.gprmc_data.Latitude += atof((char *)RMCpField);
		}
		else
		{
			gnss_data.gprmc_data.Latitude=0.0;   //GetNass´¦Àí....
		}
	}
	else
	{
		gnss_data.gprmc_data.Latitude=0.0;   //GetNass´¦Àí....
	}
	
	if(GetField(pData, RMCpField, 4, MAXFIELD))
	{
				
	   gnss_data.gprmc_data.SNth=RMCpField[0];

		if(gnss_data.gprmc_data.SNth == 'S')
		{
		  gnss_data.gprmc_data.Latitude = -gnss_data.gprmc_data.Latitude;
		}
	 
	}

	//
	// Longitude
	//
	if(GetField(pData, RMCpField, 5, MAXFIELD))
	{
		if(DataJudge(RMCpField))
		{
		 gnss_data.gprmc_data.Longitude = atof((char *)RMCpField+3) / 60.0;
		 RMCpField[3] = '\0';
		 gnss_data.gprmc_data.Longitude += atof((char *)RMCpField);
		}
		else
		{
		 gnss_data.gprmc_data.Longitude=0.0;  //GetNass´¦Àí....
		}
	}
	else
	{
	 gnss_data.gprmc_data.Longitude=0.0;  //GetNass´¦Àí....
	}
		

	if(GetField(pData, RMCpField, 6, MAXFIELD))
	{

	   gnss_data.gprmc_data.WEst=RMCpField[0];

		if(gnss_data.gprmc_data.WEst == 'W')
		{
			gnss_data.gprmc_data.Longitude = -gnss_data.gprmc_data.Longitude;
		}
	 
	}
	   
	//
	// Date
	//
	if(GetField(pData, RMCpField, 9, MAXFIELD))
	{
		// Day
		RMCpBuff[0] = RMCpField[0];
		RMCpBuff[1] = RMCpField[1];
		RMCpBuff[2] = '\0';
		gnss_data.gprmc_data.Day = atoi(RMCpBuff);

		// Month
		RMCpBuff[0] = RMCpField[2];
		RMCpBuff[1] = RMCpField[3];
		RMCpBuff[2] = '\0';
		gnss_data.gprmc_data.Month = atoi(RMCpBuff);

		// Year (Only two digits. I wonder why?)
		RMCpBuff[0] = RMCpField[4];
		RMCpBuff[1] = RMCpField[5];
		RMCpBuff[2] = '\0';
		gnss_data.gprmc_data.Year = atoi(RMCpBuff);
		gnss_data.gprmc_data.Year += 2000;				// make 4 digit date -- What assumptions should be made here?
		
		gnss_data.gprmc_data.DayValid=1;
	}
	else
	{
	  gnss_data.gprmc_data.DayValid=0;
	}

	//
	// course over ground, degrees true
	//
	if(GetField(pData, RMCpField, 10, MAXFIELD))
	{
  		
		gnss_data.gprmc_data.MagVar = atof((char *)RMCpField);
		gnss_data.gprmc_data.MagVarValid=1;
	
	}
	else
	{
	  gnss_data.gprmc_data.MagVarValid=0;
  }
	
	if(GetField(pData, RMCpField, 11, MAXFIELD))
	{
	  if(RMCpField[0] == 'W')
	   {
	    gnss_data.gprmc_data.MagVar = -gnss_data.gprmc_data.MagVar;
	   }
		 
		 gnss_data.gprmc_data.MagWEst=RMCpField[0];
	}

	if(GetField(pData, RMCpField, 12, MAXFIELD))
	{
	   gnss_data.gprmc_data.ModeIn = RMCpField[0];	   
	}
	
	//
	// Ground speed
	//
	if(GetField(pData, RMCpField, 7, MAXFIELD))
	{
		if(DataJudge(RMCpField))
		{
		 gnss_data.gprmc_data.GroundSpeed = atof((char *)RMCpField);
		 gnss_data.gprmc_data.SpeedValid = 1;
		}
		else
		{
		 gnss_data.gprmc_data.CourseValid=0;
         gnss_data.gprmc_data.ModeIn='N';
    }
	}
	else
	{
		gnss_data.gprmc_data.SpeedValid  = 0;
	}

	//
	// course over ground, degrees true
	//
	if(GetField(pData, RMCpField, 8, MAXFIELD))
	{
		if(DataJudge(RMCpField))
		{
		 gnss_data.gprmc_data.Course = atof((char *)RMCpField);
		 gnss_data.gprmc_data.CourseValid=1;
		}
		else
		{
		 gnss_data.gprmc_data.CourseValid=0;
     	 gnss_data.gprmc_data.ModeIn='N';
    }
	}
	else
	{
	  gnss_data.gprmc_data.CourseValid=0;
  }
  //--------------------------------------------------
  // ¾À´í
  //--------------------------------------------------

  if((gnss_data.gprmc_data.SNth!='S')&&(gnss_data.gprmc_data.SNth!='N'))
  {   
   	Gps_GError_Flag=1;
  }

  if((gnss_data.gprmc_data.WEst!='W')&&(gnss_data.gprmc_data.WEst!='E'))
  {					   
    Gps_GError_Flag=1;  
  }
		 
    Gps_GError_Flag=0;
}

void ProcessGPGST(uint8_t *pData)
{
    uint8_t     GSTpField[MAXFIELD];
	
	if(GetField(pData, GSTpField, 2, MAXFIELD))
	{
		gnss_data.gpgst_data.GstRms =atof((char *)GSTpField);	
	}
	
	if(GetField(pData, GSTpField, 6, MAXFIELD))
	{
		gnss_data.gpgst_data.GstDetaLat = atof((char *)GSTpField);		
	}
	else
	{
    gnss_data.gpgst_data.GstDetaLat = 200;
  }
	
	if(GetField(pData, GSTpField, 7, MAXFIELD))
	{
		gnss_data.gpgst_data.GstDetaLon = atof((char *)GSTpField);		
	}
	else
	{
    gnss_data.gpgst_data.GstDetaLon =200;
  }
	
	if(GetField(pData, GSTpField, 8, MAXFIELD))
	{
		gnss_data.gpgst_data.GstDetaAli = atof((char *)GSTpField);		
	}
	else
	{
    gnss_data.gpgst_data.GstDetaAli =200;
  }

   //GST_Get_Flag  = 1;

}


/**
 * @brief this function used to save nema data which comes from GNSS chip
 * @param[in] nmea_buf is point to a buffer which contain nmea sentence.
 * @param[in] nmea sentence length.
*/

char ch[45];

int gga_get_flag,rmc_get_flag,gst_get_flag;
//FOR HM
static void gnss_nmea_data_process(int8_t* nmea_buf, int32_t buf_len) {
    if (strstr((char*) nmea_buf, "GPGGA")) {
		gga_get_flag = 1;
		//memset(&g_gnss_location_context.nmea_sentence, 0, sizeof(g_gnss_location_context.nmea_sentence));
		//memset(&gnss_data.gigga_buf, 0, sizeof(g_gnss_location_context.nmea_sentence));
        memcpy(gnss_data.gigga_buf, nmea_buf, buf_len < GNSS_MAX_GPGGA_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GPGGA_SENTENCE_LENGTH);
		//memcpy(g_gnss_location_context.nmea_sentence.GPGGA, nmea_buf, buf_len < GNSS_MAX_GPGGA_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GPGGA_SENTENCE_LENGTH);
		ProcessGPGGA(gnss_data.gigga_buf);
    } else if (strstr((char*) nmea_buf, "GPGSA")) {
        memcpy(g_gnss_location_context.nmea_sentence.GPGSA, nmea_buf, buf_len < GNSS_MAX_GPGSA_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GPGSA_SENTENCE_LENGTH);
    } else if (strstr((char*) nmea_buf, "GPRMC")) {
		rmc_get_flag = 1;
        memcpy(gnss_data.girmc_buf, nmea_buf, buf_len < GNSS_MAX_GPRMC_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GPRMC_SENTENCE_LENGTH);
		//GNSSLOGD("%s",g_gnss_location_context.nmea_sentence.GPRMC);
		ProcessGPRMC(gnss_data.girmc_buf);
    } else if (strstr((char*) nmea_buf, "GPVTG")) {
        memcpy(g_gnss_location_context.nmea_sentence.GPVTG, nmea_buf, buf_len < GNSS_MAX_GPVTG_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GPVTG_SENTENCE_LENGTH);
	} else if (strstr((char*) nmea_buf, "GST"))   {
        gst_get_flag = 1;
        memcpy(gnss_data.gigst_buf, nmea_buf, buf_len < GNSS_MAX_GPGST_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GPGST_SENTENCE_LENGTH);
		ProcessGPGST(gnss_data.gigst_buf);
		//buf_len < GNSS_MAX_GPGST_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GPGST_SENTENCE_LENGTH
	} else if (strstr((char*) nmea_buf, "GPGSV")) {
        memcpy(g_gnss_location_context.nmea_sentence.GPGSV + strlen((char*)g_gnss_location_context.nmea_sentence.GPGSV), nmea_buf, (buf_len + strlen((char*)g_gnss_location_context.nmea_sentence.GPGSV)) < GNSS_MAX_GPGSV_SENTENCE_LENGTH ? buf_len : (GNSS_MAX_GPGSV_SENTENCE_LENGTH - strlen((char*)g_gnss_location_context.nmea_sentence.GPGSV)));
    } else if (strstr((char*) nmea_buf, "GLGSV")) {
        memcpy(g_gnss_location_context.nmea_sentence.GLGSV, nmea_buf, buf_len < GNSS_MAX_GLGSV_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GLGSV_SENTENCE_LENGTH);
    } else if (strstr((char*) nmea_buf, "GLGSA")) {
        memcpy(g_gnss_location_context.nmea_sentence.GLGSA, nmea_buf, buf_len < GNSS_MAX_GLGSA_SENTENCE_LENGTH ? buf_len : GNSS_MAX_GLGSA_SENTENCE_LENGTH);
    } else if (strstr((char*) nmea_buf, "BDGSV")) {
        memcpy(g_gnss_location_context.nmea_sentence.BDGSV, nmea_buf, buf_len < GNSS_MAX_BDGSV_SENTENCE_LENGTH ? buf_len : GNSS_MAX_BDGSV_SENTENCE_LENGTH);
    } else if (strstr((char*) nmea_buf, "BDGSA")) {
        memcpy(g_gnss_location_context.nmea_sentence.BDGSA, nmea_buf, buf_len < GNSS_MAX_BDGSA_SENTENCE_LENGTH ? buf_len : GNSS_MAX_BDGSA_SENTENCE_LENGTH);
    } else if (strstr((char*) nmea_buf, "GPACCURACY")) {
        memcpy(g_gnss_location_context.nmea_sentence.GPACC, nmea_buf, buf_len < GNSS_MAX_GPACC_SENTENCE_LENGHT ? buf_len : GNSS_MAX_GPACC_SENTENCE_LENGHT);
    } else if (strstr((char*) nmea_buf, "PMTK")) {
		memcpy(ch,nmea_buf,buf_len<45?buf_len:45);
		GNSSLOGD("gnss______________pmtk:%s\n",ch);
        //gnss_process_pmtk_response(nmea_buf, buf_len);
    }
}

extern unsigned int get_current_time_in_ms(void);


//FOR  HM
static void gnss_recieve_data()
{
    static int8_t sentence_buf[GNSS_RECIEVE_TEMP_BUF];
    int32_t sentence_length;
	
	gnss_data.gnss_sys_time = get_current_time_in_ms();

    while (1) {
        sentence_length = gnss_read_sentence(sentence_buf, GNSS_RECIEVE_TEMP_BUF);
        if (sentence_length > 0) {
            gnss_nmea_data_process(sentence_buf, sentence_length);
			if(gga_get_flag && rmc_get_flag&& gst_get_flag && (gnss_data.gpgga_data.MSecond == gnss_data.gprmc_data.MSecond))
			{
				inject_location_data((void*)&gnss_data);
				gga_get_flag = 0;
				rmc_get_flag = 0;
				gst_get_flag = 0;
			}
        } else {
            return;
        }
    }
}

/**
* @brief Timer callback, used to control GNSS chip power on/off, and report location to user.
* @param[in] timer_id: timer id
* @return void
*/
static void gnss_on_off_timer_handle_func(int32_t timer_id)
{
    static int32_t on_off_flag;
    
    //GNSSLOGD("gnss on off handle, is_on:%d,timer_id:%d\n", (int)on_off_flag, timer_id);
    if (timer_id == g_gnss_location_context.on_off_mode_timer_id) {
        if (on_off_flag == 0) {
            on_off_flag = 1;
            gnss_power_on();
            g_gnss_location_context.on_off_mode_timer_id = gnss_start_timer(GNSS_LOW_POWER_ON_OFF_MODE_GAP * 1000, gnss_on_off_timer_handle_func);
        } else {
            on_off_flag = 0;
            gnss_power_off();
            g_gnss_location_context.on_off_mode_timer_id = 
                                      gnss_start_timer((g_gnss_location_context.config.periodic - GNSS_LOW_POWER_ON_OFF_MODE_GAP) * 1000,
                                                      gnss_on_off_timer_handle_func);
            
            memset(&g_gnss_location_context.location_data, 0, sizeof(gnss_location_struct_t));
            //gnss_get_location(&g_gnss_location_context.nmea_sentence, &g_gnss_location_context.location_data);
            if (g_gnss_location_context.config.handle != NULL) {
                g_gnss_location_context.config.handle(GNSS_LOCATION_HANDLE_TYPE_DATA, &g_gnss_location_context.location_data);
            }
        }
    }
}

/**
* @brief Timer callback, used to report location to user.
* @param[in] timer_id: timer id
* @return void
*/
static void gnss_periodic_timer_handle_func(int32_t timer_id) {
    //GNSSLOGD("gnss_periodic_timer_handle_func, timer_id:%d\n", (int)timer_id);
    if (timer_id == g_gnss_location_context.periodic_timer_id) {
        
        memset(&g_gnss_location_context.location_data, 0, sizeof(gnss_location_struct_t));
        //gnss_get_location(&g_gnss_location_context.nmea_sentence, &g_gnss_location_context.location_data);
        if (g_gnss_location_context.config.handle != NULL) {
            g_gnss_location_context.config.handle(GNSS_LOCATION_HANDLE_TYPE_DATA, &g_gnss_location_context.location_data);
        }
    }
}

/**
* @brief Send PMTK command to gnss chip, help to construct '$' + buf + '*" + checksum + '\r' + '\n'.
* @param[in] timer_id: timer id
* @return void
*/
void gnss_app_send_cmd(int8_t* buf, int32_t buf_len)
{
    //const int32_t wait_ticket = 0xFFFFFFFF;
    int32_t ret_len = 0;
    int8_t temp_buf[256];
    int8_t* ind;
    uint8_t checkSumL = 0, checkSumR;

    GNSSLOGD("gnss_app_send_cmd:%s\n", buf);

    if (buf_len + 6 > 256) {
        return;
    }

    ind = buf;
    while(ind - buf < buf_len) {
        checkSumL ^= *ind;
        ind++;
    }
    temp_buf[0] = '$';
    memcpy(temp_buf + 1, buf, buf_len);
    temp_buf[buf_len + 1] = '*';

    checkSumR = checkSumL & 0x0F;
    checkSumL = (checkSumL >> 4) & 0x0F;
    temp_buf[buf_len + 2] = checkSumL >= 10 ? checkSumL + 'A' - 10 : checkSumL + '0';
    temp_buf[buf_len + 3] = checkSumR >= 10 ? checkSumR + 'A' - 10 : checkSumR + '0';
    temp_buf[buf_len + 4] = '\r';
    temp_buf[buf_len + 5] = '\n';
    temp_buf[buf_len + 6] = '\0';
    buf_len += 6;
    while (1) {
        ret_len += gnss_send_command(temp_buf + ret_len, buf_len - ret_len);
        // uart api not ready, so waiting 3 ticket for data transfer
        vTaskDelay(3);
        if (ret_len == buf_len) {
            return;
        }

    }
}


/**
* @brief Send PMTK command to gnss chip, can be trigger by other task.
* @param[in] buf: Command buffer
* @param[in] buf_len: Command length
* @return void
*/
void gnss_app_send_cmd_by_other_task(int8_t* buf, uint32_t buf_len)
{
    gnss_message_struct_t message;
    message.message_id = GNSS_ENUM_SEND_COMMAND;
    message.param1 = buf_len;
    message.param2 = buf;
    xQueueSend(gnss_task_cntx.gnss_task_queue, &message, 0);
    xEventGroupWaitBits(gnss_task_cntx.gnss_event_group, 
        GNSS_WAIT_SEND_BY_OTHER_TASK, 
        pdTRUE, 
        pdTRUE,
        100);
}

/**
* @brief Process command send by other task.
* @param[in] buf: Command buffer
* @param[in] buf_len: Command length
* @return void
*/
static void gnss_app_send_cmd_by_other_task_int(int8_t* buf, int32_t buf_len)
{
    int ret_len = 0;
    while (1) {
        ret_len += gnss_send_command(buf + ret_len, buf_len - ret_len);
        if (ret_len == buf_len) {
            break;
        }
    }
    xEventGroupSetBits(gnss_task_cntx.gnss_event_group, GNSS_WAIT_SEND_BIT);
}


/**
* @brief Callback function, this funcion is callback by others, not running in GNSS app task, only should be pass the message back to GNSS app task.
* @param[in]    type: GNSS module event
* @param[in]    param: Callback parameter
* @return void
*/
static void gnss_driver_callback_func(gnss_notification_type_t type, void *param)
{
    gnss_message_struct_t gnss_message;
    BaseType_t xHigherPriorityTaskWoken;
	//GNSSLOGD("==wells======time========callback===== %d " + type);
    switch (type) {
        case GNSS_NOTIFICATION_TYPE_POWER_ON_CNF:
            // If any error when power on, you can send error msg here.
            gnss_message.message_id = GNSS_ENUM_POWER_ON_CNF;
            break;
        case GNSS_NOTIFICATION_TYPE_POWER_OFF_CNF:
            gnss_message.message_id = GNSS_ENUM_POWER_OFF_CNF;
            break;
        case GNSS_NOTIFICATION_TYPE_READ:
            gnss_message.message_id = GNSS_ENUM_READY_TO_READ;
            break;
        case GNSS_NOTIFICATION_TYPE_WRITE:
            return;
        case GNSS_NOTIFICATION_TYPE_DEBUG_INFO:

            return;
    }
    xQueueSendFromISR(gnss_task_cntx.gnss_task_queue, &gnss_message, &xHigherPriorityTaskWoken);
}

/**
* @brief Timeout callback, send timeout event to GNSS app task.
* @return void
*/
static void gnss_timer_expiry_notify(void)
{
    gnss_message_struct_t gnss_message;
    gnss_message.message_id = GNSS_ENUM_TIME_EXPIRY;
	//GNSSLOGD("gnss_timer_expiry_notify\n");
    xQueueSendFromISR(gnss_task_cntx.gnss_task_queue, ( void * ) &gnss_message, 0);
}

int32_t rate_cfg_ready = 1;
int32_t gst_cfg_ready = 1;


static void send_gnss_commnd(void)
{
	//int32_t ret;
	//int8_t buf[256];
	int32_t send_len = 0;
	uint8_t command1[] = "$PMTK220,200*2C\r\n";
	uint8_t command2[] = "$PMTK314,0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
	uint8_t command3[] = "$PMTK605*31\r\n";
	if(rate_cfg_ready)
	{
	   send_len = gnss_send_command(command1 , strlen(command1));
	   if (send_len == strlen(command1))
	   {
		   rate_cfg_ready = 0;
	   }
	   else
	   {
		   rate_cfg_ready = 1;
	   }
	}

	send_len = 0;
	
	if(gst_cfg_ready)
	{
	   send_len = gnss_send_command(command2 , strlen(command2));
	   if (send_len == strlen(command2))
	   {
		   gst_cfg_ready = 0;
	   }
	   else
	   {
		   gst_cfg_ready = 1;
	   }
	}

	//gnss_send_command(command3 , strlen(command3));
}



/**
* @brief Process GNSS message
* @param[in]    message: GNSS message
* @return void
* FOR HM 
*/
static void gnss_task_msg_handler(gnss_message_struct_t *message)
{
    static int32_t is_power_on;

    if (!message) {
        return;
    }
    //GNSSLOGD("gnss_msg_handler, message id:%d\n", message->message_id);
    switch (message->message_id) {
        case GNSS_ENUM_POWER_ON_REQ:
            
            GNSSLOGD("==wells===GNSS_ENUM_POWER_ON_REQ:%d\n", g_gnss_location_context.config.periodic);
            
            if (g_gnss_location_context.config.periodic >= GNSS_LOW_POWER_MODE_ON_OFF) {
                g_gnss_location_context.mode = GNSS_LOCATION_MODE_ON_OFF;
                g_gnss_location_context.on_off_mode_timer_id = 
                    gnss_start_timer((g_gnss_location_context.config.periodic - GNSS_LOW_POWER_ON_OFF_MODE_GAP) * 1000, 
                                    gnss_on_off_timer_handle_func);
            } else {
                if (g_gnss_location_context.config.periodic == 0) {
                    g_gnss_location_context.mode = GNSS_LOCATION_MODE_NONE_PERIODIC;
                } else if (g_gnss_location_context.config.periodic < GNSS_LOW_POWER_MODE_NORMAL) {
                    g_gnss_location_context.mode = GNSS_LOCATION_MODE_NORMAL_PERIODIC;
                } else if (g_gnss_location_context.config.periodic < GNSS_LOW_POWER_MODE_ON_OFF) {
                    g_gnss_location_context.mode = GNSS_LOCATION_MODE_LLE;
                }
                gnss_power_on();
				gnss_send_command((int8_t*)"$PMTK220,200*2C\r\n", strlen("$PMTK220,200*2C\r\n"));
            }
            if (g_gnss_location_context.mode != GNSS_LOCATION_MODE_NONE_PERIODIC) {
                g_gnss_location_context.periodic_timer_id = gnss_start_repeat_timer(200, gnss_periodic_timer_handle_func);
            }
            is_power_on = 1;
            break;
        case GNSS_ENUM_POWER_OFF_REQ:
            GNSSLOGD("GNSS_ENUM_POWER_OFF_REQ\n");
            gnss_power_off();
            if (g_gnss_location_context.periodic_timer_id != -1) {
                gnss_stop_timer(g_gnss_location_context.periodic_timer_id);
                g_gnss_location_context.periodic_timer_id = -1;
            }
            if (g_gnss_location_context.on_off_mode_timer_id != -1) {
                gnss_stop_timer(g_gnss_location_context.on_off_mode_timer_id);      
                g_gnss_location_context.on_off_mode_timer_id = -1;
            }
            is_power_on = 0;
            g_gnss_location_context.state = GNSS_LOCATION_STATE_STOP;
            break;
        case GNSS_ENUM_CONFIG_REQ:
            GNSSLOGD("GNSS_ENUM_CONFIG_REQ:%d,%d\n", is_power_on, (int32_t)message->param1);
            if (is_power_on == 0) {
                g_gnss_location_context.config.handle = (gnss_location_handle) message->param2;
                g_gnss_location_context.config.periodic = (int32_t)message->param1;
            } else {
                gnss_power_off();
                g_gnss_location_context.config.handle = (gnss_location_handle) message->param2;
                g_gnss_location_context.config.periodic = message->param1;
                if (g_gnss_location_context.on_off_mode_timer_id != -1) {
                    gnss_stop_timer(g_gnss_location_context.on_off_mode_timer_id);
                    g_gnss_location_context.on_off_mode_timer_id = -1;
                }
                if (g_gnss_location_context.periodic_timer_id != -1) {
                    gnss_stop_timer(g_gnss_location_context.periodic_timer_id);
                    g_gnss_location_context.periodic_timer_id = -1;
                }
                gnss_demo_app_start();
            }
            break;
        case GNSS_ENUM_POWER_ON_CNF:
            GNSSLOGD("GNSS_ENUM_POWER_ON_CNF\n");
            g_gnss_location_context.state = GNSS_LOCATION_STATE_START;
            if (is_power_on) {
                gnss_app_send_cmd((int8_t*) "PMTK663", strlen("PMTK663"));//query aiding info status
                gnss_app_send_cmd((int8_t*) "PMTK353,1,0,0,0,0", strlen("PMTK353,1,0,0,0,0"));//set chip to GNSS only mode
                if (g_gnss_location_context.mode != GNSS_LOCATION_MODE_LLE) {
                    gnss_app_send_cmd((int8_t*) "PMTK225,0,0,0", strlen("PMTK225,0,0,0"));//disable lle
                } else {
                    //enable lle, hard code for parameter
                    gnss_app_send_cmd((int8_t*) "PMTK225,2,60000,240000,60000,240000", strlen("PMTK225,1,60000,240000,60000,240000"));
                }
            }
            break;
        case GNSS_ENUM_READY_TO_READ:
            gnss_recieve_data();
			send_gnss_commnd();
            break;
        case GNSS_ENUM_READY_TO_WRITE:
            // currently no use, because the data send is blocking api.
            break;
        case GNSS_ENUM_TIME_EXPIRY:
            excute_timer();
            break;
        case GNSS_ENUM_SEND_COMMAND:
            GNSSLOGD("GNSS_ENUM_SEND_COMMAND\n");
            gnss_app_send_cmd_by_other_task_int((int8_t*)message->param2, (int32_t)message->param1);
            break;
    }
}

/**
* @brief GNSS app init
* @return void
* for HM
*/
static void gnss_task_init() {
    static int8_t send_ring_buf[GNSS_SEND_RING_BUF_SIZE];
    static int8_t recieve_ring_buf[GNSS_RECIEVE_RING_BUF_SIZE];
    g_gnss_location_context.on_off_mode_timer_id = -1;
    g_gnss_location_context.periodic_timer_id = -1;
    g_gnss_location_context.config.periodic = 1;
    g_gnss_location_context.config.handle = NULL;
    g_gnss_location_context.state = GNSS_LOCATION_STATE_WAITING_INIT;
    gnss_init(gnss_driver_callback_func);
    gnss_timer_init(gnss_timer_expiry_notify);
    ring_buf_init(&gnss_task_cntx.send_ring_buf, send_ring_buf, GNSS_SEND_RING_BUF_SIZE);
    ring_buf_init(&gnss_task_cntx.recieve_ring_buf, recieve_ring_buf, GNSS_RECIEVE_RING_BUF_SIZE);
}


/**
* @brief GNSS main loop.
* @return void
*/
static void gnss_task_main()
{
    gnss_message_struct_t queue_item;
    GNSSLOGD("gnss_task_main\n");
    //gnss_task_cntx.gnss_event_group = xEventGroupCreate();
    //xEventGroupClearBits(gnss_task_cntx.gnss_event_group, GNSS_WAIT_SEND_BIT);
    
    g_gnss_location_context.state = GNSS_LOCATION_STATE_INIT;
    while (1) {
        if (xQueueReceive(gnss_task_cntx.gnss_task_queue, &queue_item, 10)) {
            gnss_task_msg_handler(&queue_item);
        }
    }
}

/**
* @brief GNSS app entry function.
* @return void
*/
TaskHandle_t gnss_demo_app_create()
{
    TaskHandle_t task_handler;
	BaseType_t ret;
    GNSSLOGD("===wells===gnss_demo_app_create\n");
    gnss_task_init();
	gnss_task_cntx.gnss_task_queue = xQueueCreate( GNSS_QUEUE_SIZE, sizeof( gnss_message_struct_t ) );
    ret = xTaskCreate((TaskFunction_t) gnss_task_main, 
        GNSS_DEMO_TASK_NAME, 
        GNSS_DEMO_TASK_STACK_SIZE/(( uint32_t )sizeof( StackType_t )), 
        NULL, 
        GNSS_DEMO_TASK_PRIO, 
        &task_handler );
	GNSSLOGD("task handler:%d, create result:%d\n", task_handler, ret);
	if (ret != pdPASS) {
		assert(0);
	}
    while (1){
        GNSSLOGD("waiting stats: %d\n", g_gnss_location_context.state);
        if (g_gnss_location_context.state == GNSS_LOCATION_STATE_INIT)
            break;
        vTaskDelay(10);
    }
    return task_handler;
}

void gnss_demo_app_config(int32_t periodic, gnss_location_handle handle)
{
    // send config msg to gnss task.
    GNSSLOGD("gnss_demo_app_config\n");
    gnss_message_struct_t gnss_message;
    gnss_message.message_id = GNSS_ENUM_CONFIG_REQ;
    gnss_message.param1 = periodic;
    gnss_message.param2 = (void*) handle;
    xQueueSend(gnss_task_cntx.gnss_task_queue, &gnss_message, 0);
}

void gnss_demo_app_start()
{
    // send power gnss msg to gnss task.
    GNSSLOGD("wells--------gnss_demo_app_start\n");
    gnss_message_struct_t gnss_message;
    gnss_message.message_id = GNSS_ENUM_POWER_ON_REQ;
    xQueueSend(gnss_task_cntx.gnss_task_queue, &gnss_message, 0);
}
 
void gnss_demo_app_stop()
{
    GNSSLOGD("gnss_demo_app_stop\n");
    gnss_message_struct_t gnss_message;
    gnss_message.message_id = GNSS_ENUM_POWER_OFF_REQ;
    xQueueSend(gnss_task_cntx.gnss_task_queue, &gnss_message, 0);
    
    while (1){
        GNSSLOGD("waiting stats: %d\n", g_gnss_location_context.state);
        if (g_gnss_location_context.state == GNSS_LOCATION_STATE_STOP)
            break;
        vTaskDelay(10);
    }
}

void gnss_demo_app_destroy(TaskHandle_t task_handler)
{
    GNSSLOGD("gnss_demo_app_destroy\n");
	gnss_timer_deinit();
	vQueueDelete(gnss_task_cntx.gnss_task_queue);
	gnss_task_cntx.gnss_task_queue = 0;
    vTaskDelete(task_handler);
}

void gnss_demo_app_send_cmd(int8_t* buf, int32_t buf_len)
{
	gnss_app_send_cmd_by_other_task(buf, buf_len);
}

void gnss_app_location_handle(gnss_location_handle_type_t type, void* param)
{
    if (type == GNSS_LOCATION_HANDLE_TYPE_ERROR) {
      //  GNSSLOGD("==wells===[GNSS Demo] location handle error! type: %d\n", (int)param);
    } else {
        gnss_location_struct_t *location = (gnss_location_struct_t *)param;
        //GNSSLOGD("==wells===[GNSS Demo] App Get Location, latitude:%s, longitude:%s, accuracy:%d\n", location->latitude, location->longitude, (int)location->accuracy);
       // gnss_update_data(&location->nmea_sentence);
        //ui_send_event(MESSAGE_ID_GNSS_NMEA, 0, 0);
    }
}


void init_gnss_data()
{
	int32_t periodic = 1;
    gnss_app_task_handle = gnss_demo_app_create();
    gnss_demo_app_config(periodic, gnss_app_location_handle);
    gnss_demo_app_start();
}


