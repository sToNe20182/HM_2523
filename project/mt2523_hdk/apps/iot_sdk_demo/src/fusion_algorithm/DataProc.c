#include "DataProc.h"
#include "BasicFunc.h"
#include <stdlib.h>
#include <string.h>

void DataProcerInit(void)
{

	IMUDataReady = FALSE;
	GNSSDataReady = FALSE;
	MEMSET(&IMUDataBuffer, 0, SIZEOF(IMU_DATA_T));
	MEMSET(&GNSSDataBuffer, 0, SIZEOF(GNSS_DATA_T));
}



PIMU_DATA_T GetIMUData(void)//if no data in buffer, GiNavPro() will return directly
{
	if (!IMUDataReady)    //gty 如果没有准备好，就返回... 如果准备好了，继续执行下面代码
		return NULL;
	IMUDataReady = FALSE; //gty 如果准备好了，重新置该标志位为FALSE，即每次清零该标志位......
	return &IMUDataBuffer;
}

void SetGNSSData(PGNSS_DATA_T pGNSSData)
{
	if (!pGNSSData)
		return;
	MEMCPY(&GNSSDataBuffer, pGNSSData, SIZEOF(GNSS_DATA_T));
	GNSSDataReady = TRUE;
}

PGNSS_DATA_T GetGNSSData(void)
{
	if (!GNSSDataReady) 
		return NULL;
	GNSSDataReady = FALSE;    //gty 每次清零该标志位...
	return &GNSSDataBuffer;
}
