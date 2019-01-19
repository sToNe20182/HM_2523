#ifndef  __SENDGPS_H
#define  __SENDGPS_H

#include "fusion_alg_interface_api.h"

void  Analyse_GpsData(void);

void  GetGPRMC(GPRMC_DATA GPRMCData);
void  GetGPVTG(void);
void  GetLGPGGA(GPGGA_DATA GPGGAData);
void  GetAngle(void);


void ProcessLatToA(double GGAIData,uint8_t *GGA_Data);
void ProcessLatToA(double GGAIData,uint8_t *GGA_Data);
void ProcessITOA(uint8_t GGAIData,uint8_t *GGA_Data);
void ProcessSSFTOA(double GGAIData,uint8_t bit,uint8_t *GGS_Data);
void ProcessCheckToA(uint8_t CheckNum,uint8_t *GGA_Data);
uint8_t ProcessCheckResult(uint16_t UpDum,uint16_t DNum,uint8_t *TDM_TX_Data);
uint8_t ProcessSFTOA(float GGAIData,uint8_t bit,uint8_t *GGA_Data);

#endif

