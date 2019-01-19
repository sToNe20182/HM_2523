#ifndef __INSNAV_H__
#define __INSNAV_H__

#include "DataTypes.h"

#define DYNAMIC_SMOOTHOR_LEN 8
#define INSTALL_SMOOTHOR_LEN 10

#define IF_INS_UPDATE(status) ((status & INS_ALIGN_COMPLETE) > 0 || (status & INS_STATIC_ALIGN_WORKING) > 0)




BOOL JudgeAngle(float Angle,float Aim,float Deta);

void IMUCompensate(PIMU_DATA_T pImuData);

void Handlemisangle(PIMU_DATA_T pImuData, PGNSS_DATA_T pGnssData);

void InitInstallMat(PIMU_DATA_T pImuData);

void DynamicModeIdentify(PIMU_DATA_T pImuData);

void INSUpdate(PIMU_DATA_T pImuData, BOOL bPosUpdate, BOOL bVelUpdate, BOOL bAttUpdate);

#endif
