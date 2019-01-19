/*************************************************************************
    > File Name: getacc.c
    > Author: xxxx
    > Mail: xxxx@126.com
    > Created Time:2019-1-4 13:21
 ************************************************************************/

#ifndef __GET_ACC__
#define __GET_ACC__

#include <stdint.h>
#include <string.h>
#include "bsp_lcd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

void Analyse_IMUData(SYSTEM_STATUS *Sys_Status);

void Get_Acc_XYZ(int16_t ACC_X,int16_t ACC_Y,int16_t ACC_Z);

float ConvertorPAcc(int16_t AccData,char Reverse);
float ConvertorRAcc(int16_t AccData,char Reverse);
float ConvertorZAcc(int16_t AccData,char Reverse);

int16_t Checkout_p2g(int16_t g_data);
int16_t Checkout_a2g(int16_t g_data);
int16_t Checkout_r2g(int16_t g_data);




#endif

