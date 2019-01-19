#include "fusion_alg_interface_api.h"
#include <stdio.h>
#include <fastmath.h>
#include <string.h>
#include <stdlib.h>
#include "Const.h"

#include "config.h"
#include "DataProc.h"
#include "BasicFunc.h"

extern OUTPUT_INFO_T GINavResult;    //gty

extern GNSSData alg_gnss_data;
extern GNSS_DATA_T   GNSSDataBuffer;  //gty GPS
extern BOOL GNSSDataReady;

void  Analyse_GpsData(void)
{
	double temp0, temp1, temp2, temp3;
	static uint8_t GGA_GGet_Flag=0,RMC_GGet_Flag=0,TimeNum,VTG_GGet_Flag=0,GST_GGet_Flag=0,Ins_Num=0,Gps_Num=0;
	static uint8_t MisAngleFlag=1;
	uint8_t i;

 //if(GpsDriverAnalyseFlag)
 //{
	 //GpsDriverAnalyseFlag =0;	 

 //---------------------------------
 //	
 //---------------------------------		
 //	 if(GGA_Get_Flag)
 //	{
 //   GGA_Get_Flag=0;
		
	
 //	 IWDG_ReloadCounter();       //
		
     GNSSDataBuffer.Position.Lon = alg_gnss_data.gpgga_data.Longitude*DEG2RAD;
 	 //printf("Analyse_GpsData----------GNSSDataBuffer.Position.Lon:%d\n",(int)(GNSSDataBuffer.Position.Lon*100));
	 GNSSDataBuffer.Position.Alt = alg_gnss_data.gpgga_data.Altitude;
	 GNSSDataBuffer.Position.Lat = alg_gnss_data.gpgga_data.Latitude*DEG2RAD;
	 //printf("Analyse_GpsData---------GNSSDataBuffer.Position.Lat:%d\n",(int)(GNSSDataBuffer.Position.Lat*100));
	 GNSSDataBuffer.Position.Alt = alg_gnss_data.gpgga_data.Altitude;
		
	 GNSSDataBuffer.NavStatus   = alg_gnss_data.gpgga_data.GPSQuality;
		
	 GNSSDataBuffer.SatUseNum   = alg_gnss_data.gpgga_data.NumOfSatsInUse;
	 //printf("Analyse_GpsData----------GNSSDataBuffer.SatUseNum:%d\n",GNSSDataBuffer.SatUseNum);
	 GNSSDataBuffer.Dops[1]     = alg_gnss_data.gpgga_data.HDOP;
	
 	 GNSSDataBuffer.UtcTime.Hour       = alg_gnss_data.gpgga_data.Hour;
     GNSSDataBuffer.UtcTime.Minute     = alg_gnss_data.gpgga_data.Minute;
	 GNSSDataBuffer.UtcTime.Second     = alg_gnss_data.gpgga_data.Second;
	 GNSSDataBuffer.UtcTime.MillSecond = alg_gnss_data.gpgga_data.MSecond;
		
	 if((ABS(GNSSDataBuffer.Position.Lon)>0)&&(ABS(GNSSDataBuffer.Position.Lat)>0))
	 {
	     GNSSDataBuffer.NavFlag    |=0x03;                 //bit0-Lat Lon Valid, bit 1-Alt Valid
	     //printf("Analyse_GpsData---gga----yes-------GNSSDataBuffer.NavFlag:%d\n",GNSSDataBuffer.NavFlag);
	 }
	 else
	 {
     	 GNSSDataBuffer.NavFlag    =0x00;  
		 //printf("Analyse_GpsData---gga----no-------GNSSDataBuffer.NavFlag:%d\n",GNSSDataBuffer.NavFlag);
     }

     //else
	 {
     	 GNSSDataBuffer.Frenqucy=5;
     }
	
     GGA_GGet_Flag=1;
	 
		
     GNSSDataBuffer.UtcTime.Hour       = alg_gnss_data.gprmc_data.Hour;
	 GNSSDataBuffer.UtcTime.Minute     = alg_gnss_data.gprmc_data.Minute;
	 GNSSDataBuffer.UtcTime.Second     = alg_gnss_data.gprmc_data.Second;
	 GNSSDataBuffer.UtcTime.MillSecond = alg_gnss_data.gprmc_data.MSecond;		
		
     GNSSDataBuffer.Position.Lon = alg_gnss_data.gprmc_data.Longitude*DEG2RAD;
	 GNSSDataBuffer.Position.Lat = alg_gnss_data.gprmc_data.Latitude*DEG2RAD;
		
	 if((ABS(GNSSDataBuffer.Position.Lon)>0)&&(ABS(GNSSDataBuffer.Position.Lat)>0))
	 {
	     GNSSDataBuffer.NavFlag    |=0x03;                 //bit0-Lat Lon Valid, bit 1-Alt Valid
	     //printf("Analyse_GpsData---rmc----yes-------GNSSDataBuffer.NavFlag:%d\n",GNSSDataBuffer.NavFlag);
	 }
     else
	 {
     	 GNSSDataBuffer.NavFlag    =0x00;  
		 //printf("Analyse_GpsData---rmc----no-------GNSSDataBuffer.NavFlag:%d\n",GNSSDataBuffer.NavFlag);
     }
		
	 temp0 = alg_gnss_data.gprmc_data.GroundSpeed*0.5144444444444;
		
	 if(alg_gnss_data.gprmc_data.CourseValid==1)	            //露脭路帽拢驴拢驴拢驴
	 {
		  temp1 = alg_gnss_data.gprmc_data.Course*DEG2RAD;
	      GNSSDataBuffer.RmcHeading=temp1;
	 }
	 else
	 {
		  temp1 = 0;
          GNSSDataBuffer.RmcHeading=0; 
	 }
		 
	 temp2 = sin(temp1);                                               
	 temp3 = cos(temp1);
	 GNSSDataBuffer.Velocity.Ve = temp0*temp2;  //露芦脧貌脣脵露脠
	 GNSSDataBuffer.Velocity.Vn = temp0*temp3;	//卤卤脧貌脣脵露脠	
		 
     if(alg_gnss_data.gprmc_data.CourseValid==1)	            
	 {
		  GNSSDataBuffer.NavFlag |= 0x40;            //bit 6-Course Valid
		  GNSSDataBuffer.VelValid = 1;
	 }
	 else
	 {
		  GNSSDataBuffer.NavFlag &= 0xBF;            //bit 6-Course Valid
		  GNSSDataBuffer.VelValid = 0;
     }
		 
     GNSSDataBuffer.NavFlag |= 0x10;            //bit 4-Level Velocity Valid
	 //--------------------------------------------------

     GNSSDataBuffer.UtcTime.Year   = alg_gnss_data.gprmc_data.Year;
     GNSSDataBuffer.UtcTime.Month  = alg_gnss_data.gprmc_data.Month;
     GNSSDataBuffer.UtcTime.Day    = alg_gnss_data.gprmc_data.Day;
		 
		 //--------------------------------------------------
	 if(alg_gnss_data.gprmc_data.ModeIn=='N')
	 {
     	 GNSSDataBuffer.NavType=0;
     }
     else if (alg_gnss_data.gprmc_data.ModeIn=='A')	
	 {
     	 GNSSDataBuffer.NavType=1;
     }
	 else if(alg_gnss_data.gprmc_data.ModeIn=='D')	
	 {
    	 GNSSDataBuffer.NavType=2;
     }
	 else if(alg_gnss_data.gprmc_data.ModeIn=='E')	
	 {
     	 GNSSDataBuffer.NavType=0xff;
     }
		
		//--------------------------------------------------
		// 脠莽鹿没脦脼脨搂...脭貌.....
		//--------------------------------------------------
	 if(alg_gnss_data.gprmc_data.DataValid!='A')
	 {
     	 GNSSDataBuffer.NavType   = 0;
		 GNSSDataBuffer.NavFlag   = 0;
		 GNSSDataBuffer.NavStatus = 0;
     }
		 
	 RMC_GGet_Flag=1;
	
		
 	 GNSSDataBuffer.GstDetaLat = alg_gnss_data.gpgst_data.GstDetaLat;
	 GNSSDataBuffer.GstDetaLon = alg_gnss_data.gpgst_data.GstDetaLon;
	 GNSSDataBuffer.GstDeta    = sqrt(alg_gnss_data.gpgst_data.GstDetaLon*alg_gnss_data.gpgst_data.GstDetaLon+alg_gnss_data.gpgst_data.GstDetaLat*alg_gnss_data.gpgst_data.GstDetaLat);
		
	 GST_GGet_Flag=1;

	 
	 if((GGA_GGet_Flag==1)&&((RMC_GGet_Flag==1)||(VTG_GGet_Flag==1)))      //麓脣脰隆脰禄脪陋脫脨GGA潞脥(RMC禄貌脮脽VTG)戮脥脨脨
	 {
	     GGA_GGet_Flag=0;
		 RMC_GGet_Flag=0;
		 VTG_GGet_Flag=0;
		
		 GNSSDataReady = True;		
     }
}


//------------------------------------------------
//void ProcessLSFTOA(float GGAIData,uint8_t bit)
//------------------------------------------------

uint8_t ProcessLSFTOA(float GGAIData,uint8_t bit,uint8_t *GGA_Data,uint8_t *GGB_Data,uint8_t *GGS_Data)
{
 static uint8_t AVRNum; 

 static uint8_t Degree;    
 static uint8_t ZF_Flag;  
 static uint8_t i;       

 static int16_t  Angle;       
 static float    AngleDot;


 if(GGAIData>=0)  
 {
  ZF_Flag=1;
  
 }
 else
 {
  ZF_Flag=0;  
 }

 Angle    = GGAIData;			  //-3.14  => -3
 AngleDot = GGAIData - Angle;	  //-3.14+3=> -0.14

//-------------------------------------
  if(ABS(Angle)>=100)
 {
  Degree=3;
 }
 else if(ABS(Angle)>=10)
 {
  Degree=2;
 }
 else if(ABS(Angle)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }

 //--------------------------------------------------------------
  if(ZF_Flag==0)
  {
   Angle    = - Angle;
   AngleDot = - AngleDot;
  }
 //--------------------------------------------------------------
  if(Degree==3)                                                //132
  {
   GGB_Data[0] = (Angle)/100;	                                //  
   GGB_Data[1] = (Angle)/10    - ((uint16_t)GGB_Data[0])*10;   //        
   GGB_Data[2] = (Angle)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //       		    

   ProcessSSFTOA(AngleDot,bit);							   

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;
	 GGA_Data[2] = GGB_Data[1] + 0x30;
	 GGA_Data[3] = GGB_Data[2] + 0x30;
	 GGA_Data[4] = '.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[5+i] =GGS_Data[i];
	 }

	 AVRNum = 5+bit;

	}
 //--------------------------------------------------------------		 								   
	else if(Degree==2)                                        //13
	{
	 GGB_Data[0] = (Angle)/10;	                              //   
   GGB_Data[1] = (Angle)    - ((uint16_t)GGB_Data[0])*10;     //       	

	 ProcessSSFTOA(AngleDot,bit);							   

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;
	 GGA_Data[2] = GGB_Data[1] + 0x30;	 
	 GGA_Data[3] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[4+i] =GGS_Data[i];
	 }

	 AVRNum = 4+bit;
	}
   	else if(Degree==1)                                          //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGB_Data[0] = Angle;	                                    //

	 ProcessSSFTOA(AngleDot,bit);

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;
	 GGA_Data[2] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[3+i] =GGS_Data[i];
	 }

	 AVRNum = 3 + bit;
	}
   //-----------------------------
	else
	{
     GGB_Data[0] = 0;

	 ProcessSSFTOA(AngleDot,bit);

	 if(ZF_Flag)
	 {
    GGA_Data[0]='+';
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 }

   GGA_Data[1] = GGB_Data[0] + 0x30;	 
	 GGA_Data[2] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[3+i] =GGS_Data[i];
	 }

	 AVRNum = 3+bit;
	}
  
   return AVRNum;										
}


//------------------------------------------------
//void ProcessLSFTOA(float GGAIData,uint8_t bit)
//------------------------------------------------
uint8_t ProcessPSFTOA(float GGAIData,uint8_t bit,uint8_t *GGA_Data,uint8_t *GGB_Data,uint8_t *GGS_Data)
{
 static uint8_t AVRNum; 

 static uint8_t Degree;    
 static uint8_t ZF_Flag;  
 static uint8_t i;       

 static int16_t  Angle;       
 static float    AngleDot;


 if(GGAIData>=0)  
 {
  ZF_Flag=1;
  
 }
 else
 {
  ZF_Flag=0;  
 }

 Angle    = GGAIData;			  //-3.14  => -3
 AngleDot = GGAIData - Angle;	  //-3.14+3=> -0.14

//-------------------------------------
  if(ABS(Angle)>=100)
 {
  Degree=3;
 }
 else if(ABS(Angle)>=10)
 {
  Degree=2;
 }
 else if(ABS(Angle)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }

 //--------------------------------------------------------------
  if(ZF_Flag==0)
  {
   Angle    = - Angle;
   AngleDot = - AngleDot;
  }
 //--------------------------------------------------------------
  if(Degree==3)                                                //132
  {
	 GGB_Data[0] = (Angle)/100;	                                //   
   GGB_Data[1] = (Angle)/10    - ((uint16_t)GGB_Data[0])*10;   //       
	 GGB_Data[2] = (Angle)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //        		    

	 ProcessSSFTOA(AngleDot,bit,GGS_Data);							   

	 if(ZF_Flag)
	 {
	  GGA_Data[0] = GGB_Data[0] + 0x30;
	  GGA_Data[1] = GGB_Data[1] + 0x30;
	  GGA_Data[2] = GGB_Data[2] + 0x30;
	  GGA_Data[3] = '.';

	  for(i=0;i<bit;i++)
	  {
	  GGA_Data[4+i] =GGS_Data[i];
	  }
	  AVRNum = 4+bit;
		
	 }
	 else
	 {
	  GGA_Data[0]='-';	 

    GGA_Data[1] = GGB_Data[0] + 0x30;
	  GGA_Data[2] = GGB_Data[1] + 0x30;
	  GGA_Data[3] = GGB_Data[2] + 0x30;
	  GGA_Data[4] = '.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[5+i] =GGS_Data[i];
	  }
	  AVRNum = 5+bit;
   }
	}
 //--------------------------------------------------------------		 								   
	else if(Degree==2)                                          //13
	{
	 GGB_Data[0] = (Angle)/10;	                                //   
   GGB_Data[1] = (Angle)    - ((uint16_t)GGB_Data[0])*10;     //       	

	 ProcessSSFTOA(AngleDot,bit,GGS_Data);							   

	 if(ZF_Flag)
	 {
    GGA_Data[0] = GGB_Data[0] + 0x30;
	  GGA_Data[1] = GGB_Data[1] + 0x30;	 
	  GGA_Data[2] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[3+i] =GGS_Data[i];
	  }
	   AVRNum = 3+bit;
	 }
	 else
	 {
	  GGA_Data[0]='-';
	 
    GGA_Data[1] = GGB_Data[0] + 0x30;
	  GGA_Data[2] = GGB_Data[1] + 0x30;	 
	  GGA_Data[3] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[4+i] =GGS_Data[i];
	  }
	   AVRNum = 4+bit;
   }
	}
   	else if(Degree==1)                                          //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGB_Data[0] = Angle;	                                    //

	  ProcessSSFTOA(AngleDot,bit,GGS_Data);

	  if(ZF_Flag)
	  {
     GGA_Data[0] = GGB_Data[0] + 0x30;
	   GGA_Data[1] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[2+i] =GGS_Data[i];
	  }
	  AVRNum = 2 + bit;
	 }
	 else
	 {
	  GGA_Data[0]='-';
    GGA_Data[1] = GGB_Data[0] + 0x30;
	  GGA_Data[2] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[3+i] =GGS_Data[i];
	  }
	  AVRNum = 3 + bit;
   }
	}
   //-----------------------------
	else
	{
   GGB_Data[0] = 0;

	 ProcessSSFTOA(AngleDot,bit,GGS_Data);

	 if(ZF_Flag)
	 {
    GGA_Data[0] = GGB_Data[0] + 0x30;	 
	  GGA_Data[1] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[2+i] =GGS_Data[i];
	  }

	  AVRNum = 2+bit;
	 }
	 else
	 {
	  GGA_Data[0]='-';
	  GGA_Data[1] = GGB_Data[0] + 0x30;	 
	  GGA_Data[2] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[3+i] =GGS_Data[i];
	  }

	  AVRNum = 3+bit;
	 }
  }
   return AVRNum;										
}

//------------------------------------------------
//void ProcessLatToA(double GGAIData)
//------------------------------------------------
void ProcessLatToA(double GGAIData,uint8_t *GGA_Data)
{
static   uint8_t  i;
static   uint8_t  Latitude;

static  double  DLatitudePart;
static  uint64_t LatitudePart;
	

   Latitude    = GGAIData;
   												     	   //31.2754477==>31+0.2754477*60==>31+16.52686 ==> 3116.52686;

   GGA_Data[0] = Latitude/10 ;					   //d 3
   GGA_Data[1] = Latitude    - ((uint16_t)GGA_Data[0])*10 ;      //d 1

    DLatitudePart= (GGAIData - Latitude)*60.0;
    LatitudePart = (DLatitudePart)*100000.0;      //degree;
   
    GGA_Data[2] = LatitudePart/1000000 ;                                         //m
    GGA_Data[3] = LatitudePart/100000  - GGA_Data[2]*10;                       //m
    GGA_Data[4] = 0x2E;									                              //.
    GGA_Data[5] = LatitudePart/10000   - ((uint32_t)GGA_Data[2])*100    - ((uint32_t)GGA_Data[3])*10 ;     //m
    GGA_Data[6] = LatitudePart/1000    - ((uint32_t)GGA_Data[2])*1000   - ((uint32_t)GGA_Data[3])*100    - ((uint32_t)GGA_Data[5])*10;     //m
    GGA_Data[7] = LatitudePart/100     - ((uint32_t)GGA_Data[2])*10000  - ((uint32_t)GGA_Data[3])*1000   - ((uint32_t)GGA_Data[5])*100   - ((uint32_t)GGA_Data[6])*10;     //m
    GGA_Data[8] = LatitudePart/10      - ((uint32_t)GGA_Data[2])*100000 - ((uint32_t)GGA_Data[3])*10000  - ((uint32_t)GGA_Data[5])*1000  - ((uint32_t)GGA_Data[6])*100  -  ((uint32_t)GGA_Data[7])*10;     //m
    GGA_Data[9] = LatitudePart         - ((uint32_t)GGA_Data[2])*1000000- ((uint32_t)GGA_Data[3])*100000 - ((uint32_t)GGA_Data[5])*10000 - ((uint32_t)GGA_Data[6])*1000 -  ((uint32_t)GGA_Data[7])*100 -  ((uint32_t)GGA_Data[8])*10;     //m

		 
    for (i=0;i<4;i++)
    {
     GGA_Data[i] = GGA_Data[i]+0x30;  
    }

    for (i=5;i<10;i++)
    {
     GGA_Data[i] = GGA_Data[i]+0x30;  
    }
   
}
//-------------------------------------------------
//void ProcessSSFTOA(float GGAIData,uint8_t bit)
//-------------------------------------------------
void ProcessSSFTOA(double GGAIData,uint8_t bit,uint8_t *GGS_Data)
{
 static double  AngDot;
 static uint8_t i;
 static uint64_t Anglel;
	
   AngDot= GGAIData;	
	
	
   if(bit==11)			       //bit==11
	 {
	  Anglel=AngDot*100000000000;
		 
    GGS_Data[0] = (Anglel)/10000000000;	                              //脟搂脦禄    	
    GGS_Data[1] = (Anglel)/1000000000  - ((uint32_t)GGS_Data[0]*10);     //掳脵脦禄        				                             //.
	  GGS_Data[2] = (Anglel)/100000000   - ((uint32_t)GGS_Data[0]*100)         - ((uint32_t)GGS_Data[1]*10) ;  //脢庐脢媒        	
	  GGS_Data[3] = (Anglel)/10000000    - ((uint32_t)GGS_Data[0]*1000)        - ((uint32_t)GGS_Data[1]*100)        - ((uint32_t)GGS_Data[2]*10) ;  //脢庐脢媒        	
		GGS_Data[4] = (Anglel)/1000000     - ((uint32_t)GGS_Data[0]*10000)       - ((uint32_t)GGS_Data[1]*1000)       - ((uint32_t)GGS_Data[2]*100)       - ((uint32_t)GGS_Data[3]*10) ;  //脢庐脢媒        	
    GGS_Data[5] = (Anglel)/100000      - ((uint32_t)GGS_Data[0]*100000)      - ((uint32_t)GGS_Data[1]*10000)      - ((uint32_t)GGS_Data[2]*1000)      - ((uint32_t)GGS_Data[3]*100)     - ((uint32_t)GGS_Data[4]*10);  //脢庐脢媒        	
		GGS_Data[6] = (Anglel)/10000       - ((uint32_t)GGS_Data[0]*1000000)     - ((uint32_t)GGS_Data[1]*100000)     - ((uint32_t)GGS_Data[2]*10000)     - ((uint32_t)GGS_Data[3]*1000)    - ((uint32_t)GGS_Data[4]*100)     - ((uint32_t)GGS_Data[5]*10);  //脢庐脢媒        	
		GGS_Data[7] = (Anglel)/1000        - ((uint32_t)GGS_Data[0]*10000000)    - ((uint32_t)GGS_Data[1]*1000000)    - ((uint32_t)GGS_Data[2]*100000)    - ((uint32_t)GGS_Data[3]*10000)   - ((uint32_t)GGS_Data[4]*1000)    - ((uint32_t)GGS_Data[5]*100)    - ((uint32_t)GGS_Data[6]*10);  //脢庐脢媒        	 
		GGS_Data[8] = (Anglel)/100         - ((uint32_t)GGS_Data[0]*100000000)   - ((uint32_t)GGS_Data[1]*10000000)   - ((uint32_t)GGS_Data[2]*1000000)   - ((uint32_t)GGS_Data[3]*100000)  - ((uint32_t)GGS_Data[4]*10000)   - ((uint32_t)GGS_Data[5]*1000)   - ((uint32_t)GGS_Data[6]*100)   - ((uint32_t)GGS_Data[7]*10);  //脢庐脢媒        	  
		GGS_Data[9] = (Anglel)/10          - ((uint64_t)GGS_Data[0]*1000000000)  - ((uint64_t)GGS_Data[1]*100000000)  - ((uint32_t)GGS_Data[2]*10000000)  - ((uint32_t)GGS_Data[3]*1000000) - ((uint32_t)GGS_Data[4]*100000)  - ((uint32_t)GGS_Data[5]*10000)  - ((uint32_t)GGS_Data[6]*1000)  - ((uint32_t)GGS_Data[7]*100)  - ((uint32_t)GGS_Data[8]*10);  //脢庐脢媒        	  
		GGS_Data[10]= (Anglel)             - ((uint64_t)GGS_Data[0]*10000000000) - ((uint64_t)GGS_Data[1]*1000000000) - ((uint32_t)GGS_Data[2]*100000000) - ((uint32_t)GGS_Data[3]*10000000)- ((uint32_t)GGS_Data[4]*1000000) - ((uint32_t)GGS_Data[5]*100000) - ((uint32_t)GGS_Data[6]*10000) - ((uint32_t)GGS_Data[7]*1000) - ((uint32_t)GGS_Data[8]*100)- ((uint32_t)GGS_Data[9]*10);  //脢庐脢媒        	  
		 
		for(i=0;i<11;i++)
	  {
	   if(GGS_Data[i]==0xFF)		
			GGS_Data[i]=0;
		
     GGS_Data[i] = GGS_Data[i] + 0x30;   
		}
   }
	 if(bit==9)			       //bit==11
	 {
	  Anglel=AngDot*1000000000;
		 
    GGS_Data[0] = (Anglel)/100000000;	                       	
    GGS_Data[1] = (Anglel)/10000000  - ((uint32_t)GGS_Data[0]*10);            				                          
	  GGS_Data[2] = (Anglel)/1000000   - ((uint32_t)GGS_Data[0]*100)         - ((uint32_t)GGS_Data[1]*10) ;          	
	  GGS_Data[3] = (Anglel)/100000    - ((uint32_t)GGS_Data[0]*1000)        - ((uint32_t)GGS_Data[1]*100)        - ((uint32_t)GGS_Data[2]*10) ;         	
		GGS_Data[4] = (Anglel)/10000     - ((uint32_t)GGS_Data[0]*10000)       - ((uint32_t)GGS_Data[1]*1000)       - ((uint32_t)GGS_Data[2]*100)       - ((uint32_t)GGS_Data[3]*10) ;  //        	
    GGS_Data[5] = (Anglel)/1000      - ((uint32_t)GGS_Data[0]*100000)      - ((uint32_t)GGS_Data[1]*10000)      - ((uint32_t)GGS_Data[2]*1000)      - ((uint32_t)GGS_Data[3]*100)     - ((uint32_t)GGS_Data[4]*10);  //        	
		GGS_Data[6] = (Anglel)/100       - ((uint32_t)GGS_Data[0]*1000000)     - ((uint32_t)GGS_Data[1]*100000)     - ((uint32_t)GGS_Data[2]*10000)     - ((uint32_t)GGS_Data[3]*1000)    - ((uint32_t)GGS_Data[4]*100)     - ((uint32_t)GGS_Data[5]*10);  //       	
		GGS_Data[7] = (Anglel)/10        - ((uint32_t)GGS_Data[0]*10000000)    - ((uint32_t)GGS_Data[1]*1000000)    - ((uint32_t)GGS_Data[2]*100000)    - ((uint32_t)GGS_Data[3]*10000)   - ((uint32_t)GGS_Data[4]*1000)    - ((uint32_t)GGS_Data[5]*100)    - ((uint32_t)GGS_Data[6]*10);  //       	 
		GGS_Data[8] = (Anglel)           - ((uint32_t)GGS_Data[0]*100000000)   - ((uint32_t)GGS_Data[1]*10000000)   - ((uint32_t)GGS_Data[2]*1000000)   - ((uint32_t)GGS_Data[3]*100000)  - ((uint32_t)GGS_Data[4]*10000)   - ((uint32_t)GGS_Data[5]*1000)   - ((uint32_t)GGS_Data[6]*100)   - ((uint32_t)GGS_Data[7]*10);  //      	  
		 
		for(i=0;i<9;i++)
	  {
	   GGS_Data[i] = GGS_Data[i] + 0x30;   
		}
   }
   else if(bit==4)			       //bit==4
   {
    GGS_Data[0] = (AngDot*10000)/1000;	                              //    	
    GGS_Data[1] = (AngDot*10000)/100  - ((uint16_t)GGS_Data[0])*10;     //       				                             //.
	  GGS_Data[2] = (AngDot*10000)/10   - ((uint16_t)GGS_Data[0])*100 - ((uint16_t)GGS_Data[1]*10) ;  //       	
	  GGS_Data[3] = (AngDot*10000)      - ((uint16_t)GGS_Data[0])*1000- ((uint16_t)GGS_Data[1]*100)- ((uint16_t)GGS_Data[2]*10) ;  //       	

    GGS_Data[0] = GGS_Data[0] + 0x30;   
    GGS_Data[1] = GGS_Data[1] + 0x30;   
    GGS_Data[2] = GGS_Data[2] + 0x30;       
	  GGS_Data[3] = GGS_Data[3] + 0x30;  
   }
   else	                        //bit==3
   {
    GGS_Data[0] = (AngDot*1000)/100;	                              //   	
    GGS_Data[1] = (AngDot*1000)/10  - ((uint16_t)GGS_Data[0]*10);     //        				                             //.
	  GGS_Data[2] = (AngDot*1000)     - ((uint16_t)GGS_Data[0])*100-((uint16_t)GGS_Data[1]*10);
	

    GGS_Data[0] = GGS_Data[0] + 0x30;   
    GGS_Data[1] = GGS_Data[1] + 0x30;   
    GGS_Data[2] = GGS_Data[2] + 0x30;       	  
   }													  
}

//-------------------------------------------------------
//uint8_t ProcessZLSFTOA(float GGAIData,uint8_t bit)
//-------------------------------------------------------
uint8_t ProcessZLSFTOA(float GGAIData,uint8_t bit,uint8_t *GGA_Data,uint8_t *GGB_Data,uint8_t *GGS_Data)
{
 static uint8_t AVRNum; 

 static uint8_t Degree;     
 static uint8_t i;       

 static int16_t  Angle;       
 static float    AngleDot;



 Angle    = GGAIData;			  //-3.14  => -3
 AngleDot = GGAIData - Angle;	  //-3.14+3=> -0.14

//-------------------------------------
  if(ABS(Angle)>=100)
 {
  Degree=3;
 }
 else if(ABS(Angle)>=10)
 {
  Degree=2;
 }
 else if(ABS(Angle)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }


 //--------------------------------------------------------------
  if(Degree==3)                                                //132
  {
	 GGB_Data[0] = (Angle)/100;	                                //   
     GGB_Data[1] = (Angle)/10    - ((uint16_t)GGB_Data[0])*10;   //        
	 GGB_Data[2] = (Angle)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //       		    

	 ProcessSSFTOA(AngleDot,bit,GGS_Data);							   

	
     GGA_Data[0] = GGB_Data[0] + 0x30;
	 GGA_Data[1] = GGB_Data[1] + 0x30;
	 GGA_Data[2] = GGB_Data[2] + 0x30;
	 GGA_Data[3] = '.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[4+i] =GGS_Data[i];
	 }

	 AVRNum = 4+bit;

	}
 //--------------------------------------------------------------		 								   
	else if(Degree==2)                                          //13
	{
	 GGB_Data[0] = (Angle)/10;	                                //   
   GGB_Data[1] = (Angle)    - ((uint16_t)GGB_Data[0])*10;     //        	

	 ProcessSSFTOA(AngleDot,bit,GGS_Data);							   

	 
   GGA_Data[0] = GGB_Data[0] + 0x30;
	 GGA_Data[1] = GGB_Data[1] + 0x30;	 
	 GGA_Data[2] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[3+i] =GGS_Data[i];
	 }

	 AVRNum = 3+bit;
	}
   	else if(Degree==1)                                          //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGB_Data[0] = Angle;	                                    //

	  ProcessSSFTOA(AngleDot,bit,GGS_Data);


    GGA_Data[0] = GGB_Data[0] + 0x30;
	  GGA_Data[1] ='.';

	  for(i=0;i<bit;i++)
	  {
	   GGA_Data[2+i] =GGS_Data[i];
	  }

	  AVRNum = 2 + bit;
	}
   //-----------------------------
	else
	{
    GGB_Data[0] = 0;

	  ProcessSSFTOA(AngleDot,bit,GGS_Data);	 

    GGA_Data[0] = GGB_Data[0] + 0x30;	 
	  GGA_Data[1] ='.';

	 for(i=0;i<bit;i++)
	 {
	  GGA_Data[2+i] =GGS_Data[i];
	 }

	 AVRNum = 2+bit;
	}
  
   return AVRNum;										
}
//------------------------------------------------
//void ProcessLongToA(double GGAIData)
//------------------------------------------------
void ProcessLongToA(double GGAIData,uint8_t *GGA_Data)
{
static   uint8_t  i,Longitude;
static   uint64_t LongitudePart;

   Longitude   = GGAIData;
							     	   //121.452914===121+0.452914*60 =121+27.17484==12127.17484;

   GGA_Data[0] = Longitude/100;					                       //d 1
   GGA_Data[1] = Longitude/10  - ((uint16_t)GGA_Data[0])*10;                       //d 2
   GGA_Data[2] = Longitude     - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1])*10;      //d 1

 
    LongitudePart = ((GGAIData - Longitude)*60)*100000;      //degree;
   
    GGA_Data[3] = LongitudePart/1000000;                                        //m
    GGA_Data[4] = LongitudePart/100000  - GGA_Data[3]*10;                       //m
    GGA_Data[5] = 0x2E;									                              //.
    GGA_Data[6] = LongitudePart/10000   - ((uint32_t)GGA_Data[3])*100    - ((uint32_t)GGA_Data[4])*10 ;     //m
    GGA_Data[7] = LongitudePart/1000    - ((uint32_t)GGA_Data[3])*1000   - ((uint32_t)GGA_Data[4])*100    - ((uint32_t)GGA_Data[6])*10;     //m
    GGA_Data[8] = LongitudePart/100     - ((uint32_t)GGA_Data[3])*10000  - ((uint32_t)GGA_Data[4])*1000   - ((uint32_t)GGA_Data[6])*100   - ((uint32_t)GGA_Data[7])*10;     //m
    GGA_Data[9] = LongitudePart/10      - ((uint32_t)GGA_Data[3])*100000 - ((uint32_t)GGA_Data[4])*10000  - ((uint32_t)GGA_Data[6])*1000  - ((uint32_t)GGA_Data[7])*100  -  ((uint32_t)GGA_Data[8])*10;     //m
    GGA_Data[10]= LongitudePart         - ((uint32_t)GGA_Data[3])*1000000- ((uint32_t)GGA_Data[4])*100000 - ((uint32_t)GGA_Data[6])*10000 - ((uint32_t)GGA_Data[7])*1000 -  ((uint32_t)GGA_Data[8])*100 -  ((uint32_t)GGA_Data[9])*10;     //m

		 
    for (i=0;i<5;i++)
    {
     GGA_Data[i] = GGA_Data[i]+0x30;  
    }

    for (i=6;i<11;i++)
    {
    GGA_Data[i] = GGA_Data[i]+0x30;  
    }
   
   
}


//----------------------------------------
//void ProcessCheckToA(uint8_t CheckNum)
//----------------------------------------
void ProcessCheckToA(uint8_t CheckNum,uint8_t *GGA_Data)
{									
static   uint8_t  i;
  			     
   GGA_Data[0] = CheckNum/16 ; 			       //
   GGA_Data[1] = CheckNum -GGA_Data[0]*16;	   //
   for(i=0;i<2;i++)
   {
    if(GGA_Data[i] <= 9)
	{
	 GGA_Data[i] = (GGA_Data[i] + '0') ;
	}
	else
	{
	 GGA_Data[i] = ((GGA_Data[i]-10) + 'A') ;
	}   
   }
}



//------------------------------------
//void ProcessCheck(uint8_t DNum)
//------------------------------------
uint8_t ProcessCheckResult(uint16_t UpDum,uint16_t DNum,uint8_t *TDM_TX_Data)
{
static  uint16_t i;
static  uint8_t CheckNum;

 CheckNum=0;
 for(i=UpDum+1;i<UpDum+DNum;i++)
 {
	 
  CheckNum^=TDM_TX_Data[i]; 
 }   
 return CheckNum;
}







uint8_t ProcessATOA(uint16_t GGAIData,uint8_t *GGA_Data,uint8_t *GGB_Data)
{
 static uint8_t Degree;   
	
 if(abs(GGAIData)>=100)
 {
  Degree=3;
 }
 else if(abs(GGAIData)>=10)
 {
  Degree=2;
 }
 else 
 {
  Degree=1;
 }
 
 
 if(Degree==3)                                                 //132
	{
	  GGB_Data[0] = (GGAIData)/100;	                                //   
    GGB_Data[1] = (GGAIData)/10    - ((uint16_t)GGB_Data[0])*10;    //       
	  GGB_Data[2] = (GGAIData)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //        	

	  GGA_Data[0] = 0x30;   
	  GGA_Data[1] = GGB_Data[0] + 0x30;   
    GGA_Data[2] = GGB_Data[1] + 0x30;   
    GGA_Data[3] = GGB_Data[2] + 0x30;       
	}	 								   
	else if(Degree==2)                                         //13
	{
	  GGB_Data[0] = (GGAIData)/10;	                           //   
    GGB_Data[1] = (GGAIData)    -  GGB_Data[0]*10;   //       
	   	
    GGA_Data[0] = 0x30;   
		GGA_Data[1] = 0x30;   
	  GGA_Data[2] = GGB_Data[0] + 0x30;   
    GGA_Data[3] = GGB_Data[1] + 0x30;  
	}
   	else                                                                                                                  
	{
    GGB_Data[0] = (GGAIData);	                              //      

		GGA_Data[0] = 0x30;   
		GGA_Data[1] = 0x30; 
		GGA_Data[2] = 0x30; 
	  GGA_Data[3] = GGB_Data[0] + 0x30; 
	}

 return Degree;
}
//------------------------------------------------
//void ProcessSFTOA(float GGAIData,uint8_t bit)
//------------------------------------------------
uint8_t ProcessSFTOA(float GGAIData,uint8_t bit,uint8_t *GGA_Data)
{

 static uint8_t Degree;    
 static uint8_t ZF_Flag;   
 
 if(GGAIData>=0)  
 {
  ZF_Flag=1;
 }
 else
 {
  ZF_Flag=0;
 }
//-------------------------------------
 if(abs(GGAIData)>=1000)
 {
  Degree=4;
 }
 else if(ABS(GGAIData)>=100)
 {
  Degree=3;
 }
 else if(ABS(GGAIData)>=10)
 {
  Degree=2;
 }
 else if(ABS(GGAIData)>=1)
 {
  Degree=1;
 }
 else
 {
  Degree=0;
 }

 //--------------------------------------------------------------
   if(ZF_Flag==0)
   {
    GGAIData = -GGAIData;
   }
 //--------------------------------------------------------------  
   if(bit==2)	   				                       //"两位数字" + "." = "三位"
   {
    if(Degree==2)                                      //13.2==> 013;
	{
	  GGA_Data[0] = '0';	                                  						         																  	
    GGA_Data[1] = (GGAIData)/10;	                  
    GGA_Data[2] = (GGAIData)    - ((uint16_t)GGA_Data[0])*10;    		

	  GGA_Data[1] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
	}
	else if(Degree==1)                                 //9.8==>98==>9.8
	{
    GGA_Data[0] = (GGAIData*10)/10;	                               //
    GGA_Data[1] = 0x2E;						                       //.
    GGA_Data[2] = (GGAIData*10)    - ((uint16_t)GGA_Data[0])*10;   // 

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
	}
	else                                              //0.8==>08==>0.8
	{
	  GGA_Data[0] = (GGAIData*10)/10;	                               
    GGA_Data[1] = 0x2E;						                       //.
    GGA_Data[2] = (GGAIData*10)    - ((uint16_t)GGA_Data[0])*10;   //  

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   	
	}
   }												  
//--------------------------------------------------------------
   else	 if(bit==3)									              //
   {
    if(Degree==3)                                                 //132==>132==>132
	{
	  GGA_Data[0] = (GGAIData)/100;	                                //    
    GGA_Data[1] = (GGAIData)/10    - ((uint16_t)GGA_Data[0])*10;    //       
	  GGA_Data[2] = (GGAIData)       - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //       	
	  GGA_Data[3] = 0x2E;						                          //.

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[1] = GGA_Data[1] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;       
	}	 								   
	else if(Degree==2)                                            //13.2==>132==>13.2
	{
	  GGA_Data[0] = (GGAIData*10)/100;	                              //   
    GGA_Data[1] = (GGAIData*10)/10    - ((uint16_t)GGA_Data[0])*10;   //       
	  GGA_Data[2] = 0x2E;						                          //.
	  GGA_Data[3] = (GGAIData*10)       - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //        	

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[1] = GGA_Data[1] + 0x30;   
    GGA_Data[3] = GGA_Data[3] + 0x30;       
	}
   	else                                                              //1.32==>132==>1.32  //0.32==>32==>0.32    //0.03==3==>0.03	                                                                  
	{
    GGA_Data[0] = (GGAIData*100)/100;	                              //
    GGA_Data[1] = 0x2E;						                          //.
    GGA_Data[2] = (GGAIData*100)/10    - ((uint16_t)GGA_Data[0])*10;  //       
	  GGA_Data[3] = (GGAIData*100)       - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[2])*10 ;  //        

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
    GGA_Data[2] = GGA_Data[2] + 0x30;   
    GGA_Data[3] = GGA_Data[3] + 0x30;       
	}						
  }
//--------------------------------------------------------------
   else	if(bit==4)									               
   {
    if(Degree==4)                                                     //1321==>1321==>1321.
  	{
	  GGA_Data[0] = (GGAIData)/1000;	                                   
      GGA_Data[1] = (GGAIData)/100  - ((uint16_t)GGA_Data[0])*10;             
	  GGA_Data[2] = (GGAIData)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;          	
	  GGA_Data[3] = (GGAIData)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[1]*100)- ((uint16_t)GGA_Data[2]*10) ;     	
	  GGA_Data[4] = 0x2E;						                          //.

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
      GGA_Data[1] = GGA_Data[1] + 0x30;   
      GGA_Data[2] = GGA_Data[2] + 0x30;       
	  GGA_Data[3] = GGA_Data[3] + 0x30;       
	  }	 	
   	else if(Degree==3)                                                       //132.1==>1321==>132.1
	{
      GGA_Data[0] = (GGAIData*10)/1000;	                                 //    
      GGA_Data[1] = (GGAIData*10)/100  - ((uint16_t)GGA_Data[0])*10;       //      
	  GGA_Data[2] = (GGAIData*10)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //        	
	  GGA_Data[3] = 0x2E;						                             //.
	  GGA_Data[4] = (GGAIData*10)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[1]*100)- ((uint16_t)GGA_Data[2]*10) ;  //??        	

	  GGA_Data[0] = GGA_Data[0] + 0x30;   
      GGA_Data[1] = GGA_Data[1] + 0x30;   
      GGA_Data[2] = GGA_Data[2] + 0x30;       
	  GGA_Data[4] = GGA_Data[4] + 0x30;        
	}	 
   	 else if(Degree==2)                                                   //13.21==>1321==>13.21
	 {
	      GGA_Data[0] = (GGAIData*100)/1000;	                                 //   
	      GGA_Data[1] = (GGAIData*100)/100  - ((uint16_t)GGA_Data[0])*10;      //       
		  GGA_Data[2] = 0x2E;						                             //.
		  GGA_Data[3] = (GGAIData*100)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[1]*10) ;  //       	
		  GGA_Data[4] = (GGAIData*100)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[1]*100)- ((uint16_t)GGA_Data[3]*10) ;  //        	

		  GGA_Data[0] = GGA_Data[0] + 0x30;   
	      GGA_Data[1] = GGA_Data[1] + 0x30;   
	      GGA_Data[3] = GGA_Data[3] + 0x30;       
		  GGA_Data[4] = GGA_Data[4] + 0x30;        
	 }	 
	 else if(Degree==1)				  							        //1.321==>1321==>1.321    //0.132==>132==>0.132
	 {																											
	      GGA_Data[0] = (GGAIData*1000)/1000;	                                 //    
		  GGA_Data[1] = 0x2E;													 //.
	      GGA_Data[2] = (GGAIData*1000)/100  - ((uint16_t)GGA_Data[0])*10;     //        				                             //.
		  GGA_Data[3] = (GGAIData*1000)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[2]*10) ;  //       	
		  GGA_Data[4] = (GGAIData*1000)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[2]*100)- ((uint16_t)GGA_Data[3]*10) ;  //       	

		  GGA_Data[0] = GGA_Data[0] + 0x30;   
	      GGA_Data[2] = GGA_Data[2] + 0x30;   
	      GGA_Data[3] = GGA_Data[3] + 0x30;       
		  GGA_Data[4] = GGA_Data[4] + 0x30;        
	 }
	 else		               
	 {											 	                     //0.321==>321==>0.321    //0.321==>321==>0.321
	      GGA_Data[0] = (GGAIData*1000)/1000;	                                 //    
		  GGA_Data[1] = 0x2E;													 //.
	      GGA_Data[2] = (GGAIData*1000)/100  - ((uint16_t)GGA_Data[0])*10;     //       				                             //.
		  GGA_Data[3] = (GGAIData*1000)/10   - ((uint16_t)GGA_Data[0])*100 - ((uint16_t)GGA_Data[2]*10) ;  //       	
		  GGA_Data[4] = (GGAIData*1000)      - ((uint16_t)GGA_Data[0])*1000- ((uint16_t)GGA_Data[2]*100)- ((uint16_t)GGA_Data[3]*10) ;  //        	

		  GGA_Data[0] = 0x30;   
	      GGA_Data[2] = GGA_Data[2] + 0x30;   
	      GGA_Data[3] = GGA_Data[3] + 0x30;       
		  GGA_Data[4] = GGA_Data[4] + 0x30;        		
	 }										   
   }
   
   return ZF_Flag;										
}
//------------------------------------------------
//void ProcessITOA(uint8_t GGAIData,uint8_t bit)
//------------------------------------------------
void ProcessITOA(uint8_t GGAIData,uint8_t *GGA_Data)
{									
 static  uint8_t  i;
  			     
   GGA_Data[0] = GGAIData/10 ;				   //
   GGA_Data[1] = GGAIData -GGA_Data[0]*10;	   //

   for(i=0;i<2;i++)
   {
    GGA_Data[i] = GGA_Data[i] + 0x30;   
   }
}


//------------------------------------------------
//void ProcessITOA(uint8_t GGAIData,uint8_t bit)
//------------------------------------------------
void ProcessIITOA(uint16_t GGAIData,uint8_t *GGA_Data)
{									
 static  uint8_t  i;
  			     
   GGA_Data[0] = GGAIData/1000 ;				   //
   GGA_Data[1] = GGAIData/100 - GGA_Data[0]*10;	   //
   GGA_Data[2] = GGAIData/10  - GGA_Data[0]*100  - GGA_Data[1]*10;	   //
   GGA_Data[3] = GGAIData     - GGA_Data[0]*1000 - GGA_Data[1]*100 - GGA_Data[2]*10;	   //
	 
   for(i=0;i<4;i++) 
   {
    GGA_Data[i] = GGA_Data[i] + 0x30;   
   }
}




uint8_t ProcessAgeTOA(uint16_t GGAIData,uint8_t *GGA_Data,uint8_t *GGB_Data)
{
	 static uint8_t Degree;   
		
	 if(abs(GGAIData)>=100)
	 {
	  Degree=3;
	 }
	 else if(abs(GGAIData)>=10)
	 {
	  Degree=2;
	 }
	 else 
	 {
	  Degree=1;
	 }
 
 
    if(Degree==3)                                                 //132
	{
		GGB_Data[0] = (GGAIData)/100;	                                //    
	    GGB_Data[1] = (GGAIData)/10    - ((uint16_t)GGB_Data[0])*10;    //       
		GGB_Data[2] = (GGAIData)       - ((uint16_t)GGB_Data[0])*100 - ((uint16_t)GGB_Data[1]*10) ;  //      	
		  
		GGA_Data[0] = GGB_Data[0] + 0x30;   
	    GGA_Data[1] = GGB_Data[1] + 0x30;   
	    GGA_Data[2] = 0x2E;	 
	    GGA_Data[3] = GGB_Data[2] + 0x30;       
	}	 								   
	else if(Degree==2)                                         //13
	{
	    GGB_Data[0] = (GGAIData)/10;	                           //    
        GGB_Data[1] = (GGAIData)    -  GGB_Data[0]*10;   //        
	
		GGA_Data[0] = GGB_Data[0] + 0x30;   
		GGA_Data[1] = 0x2E;	 
        GGA_Data[2] = GGB_Data[1] + 0x30; 
        GGA_Data[3] = 0x30;   
	}
    else                                                                                                                  
	{     
		GGA_Data[0] = 0x30;   
		GGA_Data[1] = 0x2E;	 
		GGA_Data[2] = 0x30; 
	    GGA_Data[3] = 0x30; 
	}

    return Degree;
}


void  GetLGPGGA(GPGGA_DATA GPGGAData)
{
	uint8_t i=0;
	uint8_t ZF_Flag=0;
	uint8_t CheckNumData;

	uint8_t   GGA_Data[20];   
	uint8_t   GGB_Data[8];        
	uint8_t   GGS_Data[15];  
	uint8_t   GGA_UP_Num=0,TDM_TX_Data[100];

	uint8_t  Hea_Data[6];         //$GPGGA    = 6            
	uint8_t  TEN_Data[1];         //*         = 1              
		
	//-----------------GGA------------------------------------------
	uint8_t  UTC_Data[9];          //hhmmss.ss  =  9            UTC
	uint8_t  Lat_Data[15];         //ddmm.mmmmmmm = 12          
	uint8_t  SNth_Data[1];		     //N S
	uint8_t  Lon_Data[15];         //dddmm.mmmmmmm= 13         
	uint8_t  WEst_Data[1];         //W E
	uint8_t  GPSQ_Data[1];         //1,2,3,4                    GPSQuality
	uint8_t  NumS_Data[2];         //08                         NumOfSatsInUse
	uint8_t  HDOP_Data[4];         //1321/0132/13.2/1.32/0.13/  HDOP	    
	uint8_t  Alti_Data[8];         //1020/102./10.2/0.10/0.01/  Altitude	
	uint8_t  Unit_Data[1];         //M                          Unites;
	uint8_t  Geoi_Data[5];         //980/098/9.8/0.9/           Geoidal		
	uint8_t  DTim_Data[4];         //                           DTime 		
	uint8_t  DFID_Data[4];         //0000                       DGPS ID					 
	float    GST_Deta;
//-------------------------------
//          GPGGA
//-------------------------------

//----------0-----6---------------

//--------------------------------

// 	if(GNGA_DGet_Flag)
//  {
	   Hea_Data[0] = '$';
	   Hea_Data[1] = 'G';
	   Hea_Data[2] = 'N';
	   Hea_Data[3] = 'G';
	   Hea_Data[4] = 'G';
	   Hea_Data[5]=  'A';
// }
// else
// {
//     Hea_Data[0] = '$';
//     Hea_Data[1] = 'G';
//     Hea_Data[2] = 'P';
//     Hea_Data[3] = 'G';
//     Hea_Data[4] = 'G';
//     Hea_Data[5]=  'A';
// }


 
//----------1-----9----------------
//UTC
//--------------------------------

    ProcessITOA(GPGGAData.Hour,GGA_Data);
	UTC_Data[0]  = GGA_Data[0];    //0
	UTC_Data[1]  = GGA_Data[1];    //1


	ProcessITOA(GPGGAData.Minute,GGA_Data);
	UTC_Data[2]  = GGA_Data[0];	//2
	UTC_Data[3] = GGA_Data[1];	//3

	ProcessITOA(GPGGAData.Second,GGA_Data);

	UTC_Data[4] = GGA_Data[0];	//4
	UTC_Data[5] = GGA_Data[1];	//5

	UTC_Data[6] = '.';	        //6

  //ProcessITOA(GINavResult.UtcTime.MillSecond/10);
	
	if(GPGGAData.MSecond>=100)
	{
	  GPGGAData.MSecond=GPGGAData.MSecond/10;
	}

    ProcessITOA(GPGGAData.MSecond,GGA_Data);
    UTC_Data[7] = GGA_Data[0];	//7
    UTC_Data[8] = GGA_Data[1];	//8

//----------2------10----------------
//纬度
//--------------------------------

	if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
	{
	   ProcessLatToA(GPGGAData.Latitude,GGA_Data);	
	}
	else
	{
       ProcessLatToA(GINavResult.Position.Lat*RAD2DEG,GGA_Data);	
	}
	
    for(i=0;i<10;i++)
    {
      Lat_Data[i] = GGA_Data[i];  
    }
	
//----------3------1----------------	
//南北
//--------------------------------
 
    SNth_Data[0] ='N';
//----------4-------11----------------	
//经度
//-------------------------------- 
	if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
	{
	   ProcessLongToA(GPGGAData.Longitude,GGA_Data);   //10-19;
	}
	else
	{
		ProcessLongToA(GINavResult.Position.Lon*RAD2DEG,GGA_Data);	  //10-19;			   
	}

    	
 
    for(i=0;i<11;i++)
    {
      Lon_Data[i] = GGA_Data[i];  
    }

  
//----------5-------1----------------	
//东西
//-----------------------------------

    WEst_Data[0] = GPGGAData.WEst;	           //35;	< 0 = West, > 0 = East

	GPSQ_Data[0] = GPGGAData.GPSQuality+0x30;  //37;				   
	
	if ((IS_INS_ALIGNED(g_GINavInfo.INSState))) 
	 {
		 if((GPGGAData.GPSQuality==0)||(g_GINavInfo.Jugde==0))
		 {
		   GPSQ_Data[0] =0x36;  //37;		 
		 }
	 } 

//----------5-------1----------------	
//星数
//----------------------------------- 

	 ProcessITOA(GPGGAData.NumOfSatsInUse,GGA_Data);	 
	 NumS_Data[0]  = GGA_Data[0];	          //39;
	 NumS_Data[1]  = GGA_Data[1];            //40;  
	
//-----------8------4----------------
//精度
//--------------------------------
      
	
  ProcessSFTOA(GPGGAData.HDOP,3,GGA_Data); 	      
  HDOP_Data[0] = GGA_Data[0];	           //42;
  HDOP_Data[1] = GGA_Data[1];              //43; 
  HDOP_Data[2] = GGA_Data[2];              //44; 
  HDOP_Data[3] = GGA_Data[3];              //45; 
  
  //----------5-------1----------------   
  //高度
  //----------------------------------- 

  if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
	 {
	  ZF_Flag=ProcessSFTOA(GPGGAData.Altitude,4,GGA_Data);  	 
	 }
	 else
	 {	
	  ZF_Flag=ProcessSFTOA(GINavResult.Position.Alt,4,GGA_Data);   
	 }  	 


	if(ZF_Flag==1)
  {
   Alti_Data[0] = GGA_Data[0];	            //47;
   Alti_Data[1] = GGA_Data[1];              //48; 
   Alti_Data[2] = GGA_Data[2];              //49; 
   Alti_Data[3] = GGA_Data[3];              //50; 
   Alti_Data[4] = GGA_Data[4];              //50;   
  }
  else
  {
   Alti_Data[0] = '-';	                    //47;
   Alti_Data[1] = GGA_Data[0];              //48; 
   Alti_Data[2] = GGA_Data[1];              //49; 
   Alti_Data[3] = GGA_Data[2];              //50; 
   Alti_Data[4] = GGA_Data[3];              //50;     
  }
	 
//-----------10----1------------	
//单位
//--------------------------------
  Unit_Data[0] = 'M';	                  //52;  

//-----------11----3------------
//水平高度
//--------------------------------
int Debug_Flag=0;

 if(Debug_Flag)
 {
 ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Pitch*RAD2DEG,4,GGA_Data);
 }
 else
 {
 ZF_Flag=ProcessSFTOA(GPGGAData.Geoidal,4,GGA_Data);
 }

  if(ZF_Flag)
  {
  Geoi_Data[0] = GGA_Data[0];	           //54;
  Geoi_Data[1] = GGA_Data[1];              //55; 
  Geoi_Data[2] = GGA_Data[2];              //56; 
  Geoi_Data[3] = GGA_Data[3];              //56; 
  Geoi_Data[4] = GGA_Data[4];              //56;  
  }
  else
  {
  Geoi_Data[0] = '-';	           //54;
  Geoi_Data[1] = GGA_Data[0];              //55; 
  Geoi_Data[2] = GGA_Data[1];              //56; 
  Geoi_Data[3] = GGA_Data[2];              //56; 
  Geoi_Data[4] = GGA_Data[3];              //56;   
  }

//------------13---1-------------	
//DPS Time		GGA处理
//--------------------------------

ZF_Flag = ProcessSFTOA(GPGGAData.DTime,3,GGA_Data);


 DTim_Data[0] = GGA_Data[0];	           //42;
 DTim_Data[1] = GGA_Data[1];             //43; 
 DTim_Data[2] = GGA_Data[2];             //44; 
 DTim_Data[3] = GGA_Data[3];             //45;  		    


 
//------------15---1-------------	
//结尾
//--------------------------------
  TEN_Data[0]  = '*';                	  //64; 
//------------15---2-------------
//CheckNum
//--------------------------------

 if(GGA_UP_Num>=100)
 GGA_UP_Num=100;

	  
 //----------0--------6---------------
 //$GPGGA			
 //-----------------------------------

  for(i=0;i<6;i++)		          
  TDM_TX_Data[GGA_UP_Num+i] = Hea_Data[i];

  TDM_TX_Data[GGA_UP_Num+6] = ',';
 
 //----------1--------9---------------
 //UTC hhmmss.ss    7-15,16
 //-----------------------------------
  for(i=0;i<9;i++)		          
  TDM_TX_Data[GGA_UP_Num+7+i] = UTC_Data[i];

  TDM_TX_Data[GGA_UP_Num+16] = ',';

 //----------2--------10---------------
 //纬度 ddmm.mmmmm    17-38,29
 //-----------------------------------
  for(i=0;i<10;i++)		          
  TDM_TX_Data[GGA_UP_Num+17+i] = Lat_Data[i];
	
  TDM_TX_Data[GGA_UP_Num+27] = ',';
 //----------3--------1---------------
 //纬度 N S           30,31
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+28] = SNth_Data[0];
  TDM_TX_Data[GGA_UP_Num+29] = ',';
 

 //----------4--------11---------------
 //经度dddmm.mmmmm     32-44,45
 //-----------------------------------
  for(i=0;i<11;i++)		          
  TDM_TX_Data[GGA_UP_Num+30+i] = Lon_Data[i];

  TDM_TX_Data[GGA_UP_Num+41] = ',';


 //----------5-----------1---------------
 //东西 W E            46,47
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+42] = 'E';
  TDM_TX_Data[GGA_UP_Num+43] = ',';

 //----------6-----------1---------------
 //GPSQuality          48,49
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+44] = GPSQ_Data[0];
  TDM_TX_Data[GGA_UP_Num+45] = ',';
 //----------7-----------2---------------
 //NumOfStar          50-51,52
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+46] = NumS_Data[0];
  TDM_TX_Data[GGA_UP_Num+47] = NumS_Data[1];
  TDM_TX_Data[GGA_UP_Num+48] = ',';
 //----------8-----------4---------------
 //HDOP               53-56,57
 //-----------------------------------
  for(i=0;i<4;i++)		          
  TDM_TX_Data[GGA_UP_Num+49+i] = HDOP_Data[i];
 
  TDM_TX_Data[GGA_UP_Num+53] = ',';

 //----------9-----------4---------------
 //Alititude          58-62,63
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[GGA_UP_Num+54+i] = Alti_Data[i];
 
  TDM_TX_Data[GGA_UP_Num+59] = ',';
 //----------10-----------1------------
 //M                   64,65
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+60] = Unit_Data[0];
  TDM_TX_Data[GGA_UP_Num+61] = ',';

 //----------11-----------3---------------
 //Geoidal            66-70,71
 //--------------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[GGA_UP_Num+62+i]= Geoi_Data[i];
  
  TDM_TX_Data[GGA_UP_Num+67] = ',';

 //----------12-----------1------------
 //M                   72,73
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+68] = Unit_Data[0];
  TDM_TX_Data[GGA_UP_Num+69] = ',';

 //----------12-----------1------------
 //DTime                74,75
 //-----------------------------------
  TDM_TX_Data[GGA_UP_Num+70] = DTim_Data[0];
  TDM_TX_Data[GGA_UP_Num+71] = DTim_Data[1];
  TDM_TX_Data[GGA_UP_Num+72] = DTim_Data[2];
  TDM_TX_Data[GGA_UP_Num+73] = DTim_Data[3];
  TDM_TX_Data[GGA_UP_Num+74] = ',';

 //----------13-----------4---------------
 //ID                 76-79,80
 //--------------------------------------

 if(GPGGAData.IDValid)
 {
  for(i=0;i<4;i++)		          
  TDM_TX_Data[GGA_UP_Num+75+i]= GPGGAData.DFID_Data[i];
 }
 else
 {
  for(i=0;i<4;i++)		          
  TDM_TX_Data[GGA_UP_Num+75+i]='0';
 }


 //----------14-----------1---------------
 //CheckSum           80,81-82
 //-------------------------------------- 
  TDM_TX_Data[GGA_UP_Num+79] = TEN_Data[0];         
  
  CheckNumData    = ProcessCheckResult(GGA_UP_Num,79,TDM_TX_Data);
  ProcessCheckToA(CheckNumData,GGA_Data);
  TDM_TX_Data[GGA_UP_Num+80] = GGA_Data[0];	
  TDM_TX_Data[GGA_UP_Num+81] = GGA_Data[1];	          		
       	  
 //----------15-----------1---------------
 //回车换行		   83 84
 //-------------------------------------- 
  TDM_TX_Data[GGA_UP_Num+82] = '\n';	          
  TDM_TX_Data[GGA_UP_Num+83] = 0x0;	    
 
  printf("%s",TDM_TX_Data);

}

void  GetGPRMC(GPRMC_DATA GPRMCData)
{
   float GroundSpeed;

   uint8_t  CheckNumData;	

   uint8_t i;
   uint8_t ZF_Flag;	
   uint8_t  Hea_Data[6];         //$GPGGA    = 6              ??
   uint8_t  TEN_Data[1];

   uint8_t   GGA_Data[20];   
   uint8_t   GGB_Data[8];        
   uint8_t   GGS_Data[15];  
   uint8_t   RMC_UP_Num=0,TDM_TX_Data[80];

   //----------------RMC-------------------------------------------------
   uint8_t  UTC_Data[9];          //hhmmss.ss  =  9            UTC??
   uint8_t  Lat_Data[15];         //ddmm.mmmmmmm = 12          ??
   uint8_t  SNth_Data[1];		     //N S
   uint8_t  Lon_Data[15];         //dddmm.mmmmmmm= 13          ??
   uint8_t  WEst_Data[1];         //W E

   uint8_t  Vad_Data[1];          //A V		 
   uint8_t  Spd_Data[5];          //138.1/13.81/1.381/0.1831   //Speed Over Ground, Knots  1.852??/??;??????100knots ?????? ???4?;
   uint8_t  Tra_Data[9];          //128.1/12.81/1.281/0.1281   //Track made good; Degree						 
   uint8_t  Dat_Data[6];          //ddmmyy                     //??
   uint8_t  Mag_Data[4];          //980/098/9.8/0.9            //Magnetic Variatio     ??????	???3?
   uint8_t  Mod_Data[1];          //A,D,E,N                    //Mode Indicator        Ublox

//-------------------------------
//          GPRMC
//-------------------------------

//--------字段--数-------------
 
  if(GPRMCData.Gp_PN_Kind==1)
   {
	Hea_Data[0] = '$';
	Hea_Data[1] = 'G';
	Hea_Data[2] = 'P';
	Hea_Data[3] = 'R';
	Hea_Data[4] = 'M';
	Hea_Data[5] = 'C';
   }
   else if(GPRMCData.Gp_PN_Kind==2)
   {
	Hea_Data[0] = '$';
	Hea_Data[1] = 'G';
	Hea_Data[2] = 'L';
	Hea_Data[3] = 'R';
	Hea_Data[4] = 'M';
	Hea_Data[5] = 'C';	
   }
   else if(GPRMCData.Gp_PN_Kind==3)
   {
	Hea_Data[0] = '$';
	Hea_Data[1] = 'B';
	Hea_Data[2] = 'D';
	Hea_Data[3] = 'R';
	Hea_Data[4] = 'M';
	Hea_Data[5] = 'C';	
   }
   else 
   {
	Hea_Data[0] = '$';
	Hea_Data[1] = 'G';
	Hea_Data[2] = 'N';
	Hea_Data[3] = 'R';
	Hea_Data[4] = 'M';
	Hea_Data[5] = 'C';	
   }

   if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
   {
	   Vad_Data[0] = GPRMCData.DataValid; 	 
   }
   else
   {
       Vad_Data[0] = GPRMCData.DataValid;  
   }
 //----------1-----9----------------
//UTC时间
//--------------------------------

   //ProcessITOA(GINavResult.UtcTime.Hour);
   ProcessITOA(GPRMCData.Hour,GGA_Data);
   UTC_Data[0]  = GGA_Data[0];    //0
   UTC_Data[1]  = GGA_Data[1];    //1

   //ProcessITOA(GINavResult.UtcTime.Minute);
   ProcessITOA(GPRMCData.Minute,GGA_Data);
   UTC_Data[2]  = GGA_Data[0];	//2
   UTC_Data[3] = GGA_Data[1];	//3
  
   //ProcessITOA(GINavResult.UtcTime.Second);
   ProcessITOA(GPRMCData.Second,GGA_Data);
   UTC_Data[4] = GGA_Data[0];	//4
   UTC_Data[5] = GGA_Data[1];	//5

   UTC_Data[6] = '.';	        //6

   //ProcessITOA(GINavResult.UtcTime.MillSecond/10);
   if(GPRMCData.MSecond>=100)
   {
	  GPRMCData.MSecond=GPRMCData.MSecond/10;
   }
	
   ProcessITOA(GPRMCData.MSecond,GGA_Data);

   UTC_Data[7] = GGA_Data[0];	//7
   UTC_Data[8] = GGA_Data[1];	//8

   //----------2------10----------------
   //纬度
   //--------------------------------	
   if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
   {
	   Vad_Data[0] = GPRMCData.DataValid; 
   }
   else
   {
   	  Vad_Data[0] = 'A'; 
   }  

//----------2------10----------------
//纬度
//--------------------------------
 if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
 {  
     ProcessLatToA(GPRMCData.Latitude,GGA_Data);   //10-19;		
 }
 else
 {		
	 ProcessLatToA(GINavResult.Position.Lat*RAD2DEG,GGA_Data);   //10-19;	  	
 } 

  for(i=0;i<10;i++)
  {
   Lat_Data[i] = GGA_Data[i];  
  }	
  
  										 
//----------3------1----------------	
//南北
//--------------------------------

  SNth_Data[0] = 'N';	         //21; < 0 = South, > 0 = North

//----------4-------11----------------	
//经度
//--------------------------------
	if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
	{
	   ProcessLongToA(GPRMCData.Longitude,GGA_Data);   //10-19	
	}
	else
	{ 
	   ProcessLongToA(GINavResult.Position.Lon*RAD2DEG,GGA_Data);   //10-19; 			
	}

  for(i=0;i<11;i++)
  {
   Lon_Data[i] = GGA_Data[i];  
  }											 
	
//----------5-------1----------------	
//东西
//--------------------------------

  WEst_Data[0] = 'E';	           //35;	< 0 = West, > 0 = East
  
//----------7-----5------------------   
// 地速
//---------------------------------
  if ((!IS_INS_ALIGNED(g_GINavInfo.INSState))) 
  {
	GroundSpeed=GPRMCData.GroundSpeed;
  }
  else
  {
	  GroundSpeed=sqrt(GINavResult.Velocity.Ve*GINavResult.Velocity.Ve+GINavResult.Velocity.Vn*GINavResult.Velocity.Vn)/0.5144444444444;  
  }


  ProcessSFTOA(GroundSpeed,4,GGA_Data);  
 
  Spd_Data[0]= GGA_Data[0];
  Spd_Data[1]= GGA_Data[1];
  Spd_Data[2]= GGA_Data[2];
  Spd_Data[3]= GGA_Data[3];
  Spd_Data[4]= GGA_Data[4];

//----------8-----5------------------
//方向
//---------------------------------
  if (!IS_INS_ALIGNED(g_GINavInfo.INSState)) 
	{
	 ZF_Flag=ProcessSFTOA(GPRMCData.Course,4,GGA_Data);  
	}
	else
	{
     ZF_Flag=ProcessSFTOA(GINavResult.Attitude.Heading*RAD2DEG,4,GGA_Data);  
	}

	
	if(ZF_Flag)
	{
	   Tra_Data[0]= GGA_Data[0];
	   Tra_Data[1]= GGA_Data[1];
	   Tra_Data[2]= GGA_Data[2];
	   Tra_Data[3]= GGA_Data[3];
	   Tra_Data[4]= GGA_Data[4];
	}
	else 
	{
	   Tra_Data[0]= '-';
	   Tra_Data[1]= GGA_Data[0];
	   Tra_Data[2]= GGA_Data[1];
	   Tra_Data[3]= GGA_Data[2];
	   Tra_Data[4]= GGA_Data[3];
    }
//----------9-----6------------------
//ddmmyy 由ProcessGPRMC处理
//----------------------------------
  ProcessITOA(GPRMCData.Day,GGA_Data);
  Dat_Data[0]  = GGA_Data[0];  //0
  Dat_Data[1]  = GGA_Data[1];  //1

  ProcessITOA(GPRMCData.Month,GGA_Data);
  Dat_Data[2]  = GGA_Data[0];	//2
  Dat_Data[3] = GGA_Data[1];	//3
 
  ProcessITOA(GPRMCData.Year-2000,GGA_Data);
  Dat_Data[4] = GGA_Data[0];	//4
  Dat_Data[5] = GGA_Data[1];	//5
//---------10-----3------------------
//磁场变化值 
//---------------------------------
  if(GPRMCData.MagVarValid)
  {
	   ProcessSFTOA(GPRMCData.MagVar,3,GGA_Data); 
	   Mag_Data[0]= GGA_Data[0];
	   Mag_Data[1]= GGA_Data[1];
	   Mag_Data[2]= GGA_Data[2];
	   Mag_Data[3]= GGA_Data[3];
  }
  else
  {
	   Mag_Data[0]= '0';
	   Mag_Data[1]= '.';
	   Mag_Data[2]= '0';
	   Mag_Data[3]= '0';
  }
//---------11-----3------------------
//Magnetic West East 由ProcessGGA处理
//-----------------------------------
  if(GPRMCData.MagVarValid)
  {
		
  }
  else
  {

  }
  
//----------12----1------------------
//ModeIndictor   
//----------------------------------
  Mod_Data[0] = GPRMCData.ModeIn;
//----------13----1------------------
//*  				 
//----------------------------------
  TEN_Data[0]='*';
//----------14----1------------------
//CheckSum
//----------------------------------

	
 //----------0--------6---------------
 //$GPRMC			 0-5,6
 //-----------------------------------

  for(i=0;i<6;i++)		          
  TDM_TX_Data[RMC_UP_Num+i] = Hea_Data[i];

  TDM_TX_Data[RMC_UP_Num+6] = ',';
 
 //----------1--------9---------------
 //UTC hhmmss.ss    7-15,16
 //-----------------------------------
  for(i=0;i<9;i++)		          
  TDM_TX_Data[RMC_UP_Num+7+i] = UTC_Data[i];

  TDM_TX_Data[RMC_UP_Num+16] = ',';

 //----------2--------1---------------
 //DataValid         17,18
 //-----------------------------------    
  TDM_TX_Data[RMC_UP_Num+17] = Vad_Data[0];
  TDM_TX_Data[RMC_UP_Num+18] = ',';

 //----------3--------10---------------
 //纬度 ddmm.mmmmm    19-30,31
 //-----------------------------------
  for(i=0;i<10;i++)		          
  TDM_TX_Data[RMC_UP_Num+19+i] = Lat_Data[i];

  TDM_TX_Data[RMC_UP_Num+29] = ',';

  
 //----------4--------1---------------
 //纬度 N S           30,31
 //-----------------------------------
  TDM_TX_Data[RMC_UP_Num+30] = SNth_Data[0];
  TDM_TX_Data[RMC_UP_Num+31] = ',';


 //----------5--------11---------------
 //经度dddmm.mmmmmmmmm     34-46,47
 //-----------------------------------
  for(i=0;i<11;i++)		          
  TDM_TX_Data[RMC_UP_Num+32+i] = Lon_Data[i];

  TDM_TX_Data[RMC_UP_Num+43] = ',';
 
 
 //----------6-----------1---------------
 //东西 W E            48,49
 //-----------------------------------
  TDM_TX_Data[RMC_UP_Num+44] = WEst_Data[0];
  TDM_TX_Data[RMC_UP_Num+45] = ',';


 //----------7-----------5---------------
 //Speed               50-53,54
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[RMC_UP_Num+46+i] = Spd_Data[i];
 
  TDM_TX_Data[RMC_UP_Num+50] = ',';

 //----------8-----------5---------------
 //Track_Mode          55-59,60
 //-----------------------------------
  for(i=0;i<5;i++)		          
  TDM_TX_Data[RMC_UP_Num+51+i] = Tra_Data[i];
	

 //-----------------------------------------
  TDM_TX_Data[RMC_UP_Num+56] = ',';
 //----------9-----------6---------------
 //Data                61-66,67
 //-----------------------------------
  for(i=0;i<6;i++)		          
  TDM_TX_Data[RMC_UP_Num+57+i] = Dat_Data[i];
 
  TDM_TX_Data[RMC_UP_Num+63] = ',';

 //----------10-----------3---------------
 //Mag_Data             68-70,71
 //-----------------------------------
 if(GPRMCData.MagVarValid)
 {
  for(i=0;i<3;i++)		          
  TDM_TX_Data[RMC_UP_Num+64+i] = Mag_Data[i];
 }
 else
 {      
  TDM_TX_Data[RMC_UP_Num+64] = 0x30;
	TDM_TX_Data[RMC_UP_Num+65] = '.';
	TDM_TX_Data[RMC_UP_Num+66] = 0x30;
 }
 
  TDM_TX_Data[RMC_UP_Num+67] = ',';

 //----------11-----------1---------------
 //东西 W E            72,73
 //-----------------------------------
  if(GPRMCData.MagVarValid)
  {
	 TDM_TX_Data[RMC_UP_Num+68] = GPRMCData.MagWEst;
  }
  else
  {
     TDM_TX_Data[RMC_UP_Num+68] = '0';
  }
	
  TDM_TX_Data[RMC_UP_Num+69] = ',';

 //----------12---------1-----------
 //ModeIndictor   		74,75,76,77
 //----------------------------------
  TDM_TX_Data[RMC_UP_Num+70] = Mod_Data[0];
  TDM_TX_Data[RMC_UP_Num+71] = TEN_Data[0];         
  
  CheckNumData    = ProcessCheckResult(RMC_UP_Num+0,71,TDM_TX_Data);
  ProcessCheckToA(CheckNumData,GGA_Data);
  TDM_TX_Data[RMC_UP_Num+72] = GGA_Data[0];	
  TDM_TX_Data[RMC_UP_Num+73] = GGA_Data[1];	          		
       	  
 //----------15-----------1---------------
 //回车换行			   74 75
 //-------------------------------------- 
  TDM_TX_Data[RMC_UP_Num+74] = '\n';	          
  TDM_TX_Data[RMC_UP_Num+75] = 0;	          	

  printf("%s",TDM_TX_Data);
}


