#ifndef __CONFIG_H__
#define __CONFIG_H__


#define IMU_RAW_SAMPLE_RATE			50	//IMU原始采样率
#define IMU_COLLECT_RATE			10	//IMU采样收集速率  
#define INS_UPDATE_RATE				20	//INS更新速率      10HZ
#define INS_UPDATE_SAMPLE_NUM		1	//INS更新子样树


#define Deta_HDop		    0.85	  //阈值---水平
#define Deta_SatUseNum		5	      //阈值---5颗....
#define Deta_SatUseRatio  0.4     //阈值---使用率
#define Deta_MeanCn0      25      //阈值---载噪比
#define Deta_PDop         2.5     //阈值---PDOP
#define Deta_Heading      15      //阈值---方向值...

#define Max_Speed         50      //阈值---PDOP
#define Deta_Pos          10

#define M              39940.64       // 经线圈     单位 km
#define NN             40075.86       // 赤道圈     单位 km

#define IMU_RAWSAMPLE_INTERVAL		(1000/IMU_RAW_SAMPLE_RATE)					//IMU nominal sample period  20,即20ms.
#define IMU_FRAME_INTERVAL			(1000/IMU_COLLECT_RATE)						//IMU frame period          100,即100ms.
#define IMU_FRAME_SAMPLENUM			(IMU_RAW_SAMPLE_RATE/IMU_COLLECT_RATE+2)	//max sample number of each frame, set redundancy as 2  7
#define INSUPDATE_DATA_INTERVAL		(1000/INS_UPDATE_RATE)						//INS update period         100,即100ms.
#define INSUPDATE_SUBDATA_INTERVAL	(1000/INS_UPDATE_RATE/INS_UPDATE_SAMPLE_NUM)//INS update subdata period 100,即100ms.

//GNSS Config
#define LEAP_SECONDS		16		//Diff Second between UTC and GPSTime, modify this number if leap seconds chanaged
#define MAX_SYSTEM_NUM		3		//GNSS system number
#define MAX_SVID			160



#endif
