#ifndef __GINAV_H__
#define __GINAV_H__
#include "DataTypes.h"
#include "stdint.h"


#ifdef __cplusplus
EXTERN "C"{
#endif

  

extern  uint8_t   	GpsInsGetFlag;   //
extern  uint8_t     Gnss_Get_Flag;

	
void   SetGNSSData(PGNSS_DATA_T pGNSSData);
void   GINavInit(void);
BOOL   GINavProc(POUTPUT_INFO_T pNavResult);

#ifdef __cplusplus
};
#endif
#endif
