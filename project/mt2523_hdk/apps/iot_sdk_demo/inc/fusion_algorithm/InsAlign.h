#ifndef __INSALIGN_H__
#define __INSALIGN_H__

#include "DataTypes.h"
#include "DataProc.h"
#include "Config.h"

#define GYRO_BIAS_SMOOTHOR_LEN		10*INS_UPDATE_RATE
#define STATIC_ALIGN_THD			10
#define STATIC_ALIGN_SMOOTHOR_LEN	(1*INS_UPDATE_RATE)
#define MAX_ATT_MAINTAIN_MS			100000
#define ALIGN_HEADING_VEL_THD		2.0
#define START_CONFIRM_HEADING_VEL_THD		2.5
#define END_CONFIRM_HEADING_VEL_THD			3.0



void InitGyroBias(PIMU_DATA_T pImuData, PGNSS_DATA_T pGnssData);
void INSAlign(PIMU_DATA_T pImuData, PGNSS_DATA_T pGnssData);
void ConfirmHeading(PGNSS_DATA_T pGnssData);

#endif
