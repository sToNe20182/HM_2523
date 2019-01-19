#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include "Config.h"

//Basic data type define
typedef unsigned int U32;
typedef signed int S32;
typedef unsigned short U16;
typedef signed short S16;
typedef unsigned char U8;
typedef signed char S8;
typedef double FLOAT64;
typedef float FLOAT32;

#define EXTERN extern
#define STATIC static
#define CONST const
#define SIZEOF sizeof
#define MEMSET memset
#define MEMCPY memcpy

#ifndef BOOL
#define BOOL S32
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif


#define INS_INACTIVE					  0x00
#define INS_ACTIVE						  0x01	//bit1 - INS active
#define INS_LEVELATT_GOOD				0x02	//bit2 - INS Level Attitude Initialized and good
#define INS_HEADING_INIT				0x04	//bit3 - INS Heading Angle Initialized
#define INS_HEADING_GOOD				0x08	//bit4 - INS Heading Angle Good
#define INS_POSVEL_GOOD					0x10	//bit5 - INS Position and Velocity Good

#define IS_INS_ALIGNED(status) ((status & INS_LEVELATT_GOOD) == INS_LEVELATT_GOOD && (status & INS_HEADING_INIT) == INS_HEADING_INIT && (status & INS_POSVEL_GOOD) == INS_POSVEL_GOOD)

typedef enum{ Unknow = 0, Bad, Good, Excellent } PosQuality;

//UTC struct
typedef struct _UTC_T
{
	U16 Year;
	S8 Month;
	S8 Day;
	S8 Hour;
	S8 Minute;
	S8 Second;
	S16 MillSecond;
}UTC_T, *PUTC_T;

//GPS Time struct
typedef struct _GPST_T
{
	U16 WeekNumber;
	U32 WeekMillSecond;
}GPST_T, *PGPST_T;

//IMU frame struct
typedef struct _IMU_FRAME_T
{
	UTC_T UtcTime;
	U8 SampleNum;
	S16 Samples[IMU_FRAME_SAMPLENUM][6];
}IMU_FRAME_T, *PIMU_FRAME_T;

//IMU Measurement struct
typedef struct _IMU_DATA_T
{
	UTC_T UtcTime;
	U32   MsrInterval;

	FLOAT64 GyroF[INS_UPDATE_SAMPLE_NUM][3];
	FLOAT64 AccF[INS_UPDATE_SAMPLE_NUM][3];

	FLOAT64 Gyro[INS_UPDATE_SAMPLE_NUM][3];
	FLOAT64 Acc[INS_UPDATE_SAMPLE_NUM][3];
}IMU_DATA_T, *PIMU_DATA_T;

//Position struct
typedef struct _POS_T
{
	FLOAT64 Lat;
	FLOAT64 Lon;
	FLOAT64 Alt;
}POS_T, *PPOS_T;

//Velocity struct
typedef struct _VEL_T
{
	FLOAT64 Ve;
	FLOAT64 Vn;
	FLOAT64 Vu;
}VEL_T, *PVEL_T;

//Euler Angle struct
typedef struct _EULER_T
{
	FLOAT64 Phi;
	FLOAT64 Theta;
	FLOAT64 Gamma;
}EULER_T, *PEULER_T;

//Attitude Angle struct
typedef struct _ATT_T
{
	FLOAT64 Heading;
	FLOAT64 Pitch;
	FLOAT64 Roll;
}ATT_T, *PATT_T;

//Quaternion struct
typedef struct _QUAT_T
{
	FLOAT64 Scalar;
	FLOAT64 Vector[3];
}QUAT_T, *PQUAT_T;

//Cosine Matrix struct
typedef struct _COSM_T
{
	FLOAT64 C11, C12, C13;
	FLOAT64 C21, C22, C23;
	FLOAT64 C31, C32, C33;
}COSM_T, *PCOSM_T;

//GNSS Raw Data struct

#define IS_POS_VALID(flag) ((flag&0x03)==0x03)
#define IS_LEVEL_VEL_VALID(flag) ((flag&0x50)==0x50)
#define IS_UP_VEL_VALID(flag) ((flag&0x20)==0x20)

typedef struct _GNSS_DATA_T
{
	UTC_T UtcTime;
	U8 NavType;
	U8 NavFlag;		//bit0-Lat Lon Valid, bit 1-Alt Valid, bit 4-Level Velocity Valid, bit 5-Up Velocity Valid, bit 6-Course Valid
	POS_T Position;
	VEL_T Velocity;
	FLOAT32 RmcHeading;
	U8   VelValid;
	U8   NavStatus;
	U8   SatUseNum;
	FLOAT64 Sigma[6];
	FLOAT32 Dops[3];					//pdop,hdop,vdop
	
	/*U8   GSVAValid;
	U8 SatInViewNum;
	U32 SatInView[MAX_SVID / 32 + 1];		//satellite in view flag
	U32 SatUse[MAX_SVID / 32 + 1];			//satellite in use falg
	U8 SatCn0[MAX_SVID];		//cn0
	U8 SatEl[MAX_SVID];		//elevation
	U8 SatAz[MAX_SVID];		//azimuth*/
	

	U8 Frenqucy;      //?¦Ì?¨º...
	
	FLOAT32 GstDetaLat;
	FLOAT32 GstDetaLon;
	FLOAT32 GstDetaAli;
	
	FLOAT32 GstDeta;

	U8 GetAgeFlag;
	U8 ResetFlag;
	
}GNSS_DATA_T, *PGNSS_DATA_T;


typedef struct _GSAV_DATA_T
{
  U8 SatUseNum;
	U8 SatInViewNum;
	FLOAT32 SatUseRate;
	FLOAT32 MeanCn0;
	FLOAT32 Dops[3];

}GSAV_DATA_T;

//IMU Bias Struct
typedef struct _IMU_PARAM_T
{

	FLOAT64 GyroBias[3];
	FLOAT64 DiffBias;
	FLOAT64 AccBias[3];
	COSM_T InstallMat;
	U8 InstallMatInitFlag;
	U8 GyroBiasIsConverage;
}IMU_PARAM_T, *PIMU_PARAM_T;

//INS Navigation Shared Info struct
typedef struct _GINAV_INFO_T
{
	U32	Tag;
	UTC_T UtcTime;
	GPST_T GPSTime;
	U32 INSState;
	U8 SYSFlag;
	U8 INSFlag;
	U32 StaticCount;
	U32 DynaCount;
	U8  StaticCFlag;
	U8  LowSpeedFalg;
	U8  GpsHeadBack;

	U32 GNSSHaltCount;
	U32 INSAloneMsCount;
	U32 KFCount;

	PosQuality PositionQuality;
	POS_T Position;

	VEL_T Velocity;
	FLOAT32 factor;

	EULER_T Euler;
	QUAT_T Quat_bn;
	QUAT_T Quat_ne;
	COSM_T CM_bn;
	COSM_T CM_ne;
	VEL_T dVelocity;
	FLOAT64 Rm, Rn;
	FLOAT64 Wie[3];
	FLOAT64 Wen[3];
	FLOAT64 Gravity;
	FLOAT64 SF_n[3];
	FLOAT64 P[3];

	FLOAT64 LastGyro[3];
	FLOAT64 LastAcc[3];
	IMU_PARAM_T ImuCfg;
	
	U8 NavStatus;
	U8 NavType;		
  U8 Jugde;
	
	U8 SatUseNum;
	U8 StaticFlag;

	U8      GstFlag;
	U8      GstStatus;
	FLOAT32 GstDeta;
	FLOAT32 GstDetaMin;
	FLOAT32 GST_Diff;	
	FLOAT32 RmcHead;
	//------------------
		//----------------------------


	COSM_T Ini0_Mat;
	COSM_T Ini1_Mat;
	COSM_T Ini2_Mat;
	COSM_T Ini3_Mat;
	COSM_T Ini4_Mat;
	
	U8    Ini0_Flag;
	U8    Ini1_Flag;
	U8    Ini2_Flag;
	U8    Ini3_Flag;

	
	U8    Ini0_Kind;
	S16   Ini_Roll;
	S16   Ini_Pitch;
	U8    Ini_Flag;
	//----------------------------------------------
		
}GINAV_INFO_T;

typedef struct _OUTPUT_INFO_T
{
	U32 Tag;
	PosQuality PositionQuality;
	U32 INSState;
	UTC_T UtcTime;
	GPST_T GpsTime;
	POS_T Position;
	VEL_T Velocity;
	ATT_T Attitude;
	FLOAT64 Gyro[3];
	FLOAT64 Acc[3];

  U8 Jugde;
	U8 SatUseNum;
	U8  GstStatus;
	FLOAT32 GstDeta;	
	U8     StaticFlag;

}OUTPUT_INFO_T, *POUTPUT_INFO_T;
#endif
