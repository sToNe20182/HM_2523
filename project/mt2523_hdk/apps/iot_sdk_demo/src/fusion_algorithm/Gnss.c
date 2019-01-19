#include "Gnss.h"
#include "Const.h"
#include "Config.h"
#include "DataProc.h"
#include "BasicFunc.h"
#include "GlobalVars.h"
#include "SendGPS.h"
#include <string.h>
#include <math.h>





BOOL GnssEvaluation(PGNSS_DATA_T pGnssData,PIMU_DATA_T pImuData)
{

	static S32 i = 0;

	FLOAT32 Factor, delta_RIHeading, delta_PIHeading, delta_PRHeading, VLevel, VLevelB, Deta_Level, Head_Scale, GstLowScale;
	STATIC U16 Deta_SNum=0;
	static U16 Head_BNum, Head_SNum, GyrNum;
	

	STATIC POS_T    PosBack;
	STATIC VEL_T    VolBack;
	
	STATIC U8       PosFirst=0,MaxScale=1,BigErrorFlag=0;
	STATIC U16      Length_Big_Num=0,GpsBackNum=0;
	FLOAT32  Diff_North,Diff_East,Diff,GyrNumScale,GnssScale,DiffScale;
	
	
	//-----------------------------------------------------------------------	
	if (pGnssData->Dops[0] < 0.5f)
		pGnssData->Dops[0] = 99.0f;
	if (pGnssData->Dops[1] < 0.5f)
		pGnssData->Dops[1] = 99.0f;
	if (pGnssData->Dops[2] < 0.5f)
		pGnssData->Dops[2] = 99.0f;




	pGnssData->Sigma[0] = pGnssData->Sigma[1] = 6.25;     //水平速度噪声初值
	pGnssData->Sigma[2] = 9.0;                            //高度的噪声初值
	pGnssData->Sigma[3] = pGnssData->Sigma[4] = 0.05;     //水平速度噪声初值
	pGnssData->Sigma[5] = 0.10;                           //垂直速度噪声初值

	

	return TRUE;
}
