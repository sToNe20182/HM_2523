#include "InsNav.h"
#include "Config.h"
#include "Const.h"
#include "DataTypes.h"
#include "GlobalVars.h"
#include "BasicFunc.h"
#include "INSAlign.h"
#include <math.h>




void IMUCompensate(PIMU_DATA_T pImuData)  //gty IMU安装角矩阵补偿，并且完成偏置补偿....
{
	S32 i, j;
	FLOAT64 dt = pImuData->MsrInterval / 1000.0;
	FLOAT64 temp1Gyro[3], temp1Acc[3];
	FLOAT64 temp2Gyro[3], temp2Acc[3];


	for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
	{
		CMRotation(&g_GINavInfo.Ini0_Mat, pImuData->Gyro[i], temp1Gyro);	//Coordination
		CMRotation(&g_GINavInfo.Ini0_Mat, pImuData->Acc[i], temp1Acc);

		CMRotation(&g_GINavInfo.Ini1_Mat, temp1Gyro, temp2Gyro);	//Roll
		CMRotation(&g_GINavInfo.Ini1_Mat, temp1Acc, temp2Acc);

		CMRotation(&g_GINavInfo.Ini2_Mat, temp2Gyro, temp1Gyro);	//Pitch
		CMRotation(&g_GINavInfo.Ini2_Mat, temp2Acc, temp1Acc);

		CMRotation(&g_GINavInfo.Ini3_Mat, temp1Gyro, temp2Gyro);	//yaw
		CMRotation(&g_GINavInfo.Ini3_Mat, temp1Acc, temp2Acc);

		CMRotation(&g_GINavInfo.Ini4_Mat, temp2Gyro, temp1Gyro);	 //misangle
		CMRotation(&g_GINavInfo.Ini4_Mat, temp2Acc, temp1Acc);

		for (j = 0; j < 3; j++) //Compensate Bias
		{
			pImuData->Acc[i][j] = temp1Acc[j] - g_GINavInfo.ImuCfg.AccBias[j] * dt;          //gty 在此处已经进行过加速度偏置的补偿处理

			if (IS_INS_ALIGNED(g_GINavInfo.INSState))	//只在对准以后补偿陀螺仪Bias，对准之前将误差放过去使其进入陀螺仪Bias初始化平滑滤波器
			{
				pImuData->Gyro[i][j] = temp1Gyro[j] - g_GINavInfo.ImuCfg.GyroBias[j] * dt;     //gty 在此处已经进行过陀螺仪偏置的补偿处理

			}

		}
	}
}

//note: 车辆运动特性为低频量，当车辆处于静态时器件的低频输出很小
const FLOAT64 StaticThd[6] = { 0.0001, 0.0001, 0.0001, 0.005, 0.005, 0.005 };
void DynamicModeIdentify(PIMU_DATA_T pImuData)                //gty？ 为什么仅仅依靠IMU数据，而不是依靠GPS速度等数据.....
{
	S32 i, j;
	STATIC S32 SmoothCount = 0;
	STATIC FLOAT64 GyroSmoother[3] = { 0.0 };
	STATIC FLOAT64 AccSmoother[3] = { 0.0 };
	STATIC FLOAT64 GyroDifSmoother[3] = { 0.0 };
	STATIC FLOAT64 AccDifSmoother[3] = { 0.0 };
	FLOAT64 GyroDif, AccDif;
	U8 bDynamic = 0;                                          //gty，每次进入，bDynamic都清零，所以，....

	SmoothCount++;                                            //gty 这个数据不停增加，直到等于DYNAMIC_SMOOTHOR_LEN=8
	SmoothCount = MIN(SmoothCount, DYNAMIC_SMOOTHOR_LEN);
	for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)               //gty 采集值和平滑滤波值之差小于阈值，则认为是静态....
	{
		for (j = 0; j < 3; j++)
		{
			GyroSmoother[j] -= GyroSmoother[j] / SmoothCount;
			GyroSmoother[j] += pImuData->Gyro[i][j] / SmoothCount;
			AccSmoother[j] -= AccSmoother[j] / SmoothCount;
			AccSmoother[j] += pImuData->Acc[i][j] / SmoothCount;
			GyroDif = ABS(pImuData->Gyro[i][j] - GyroSmoother[j]);
			AccDif = ABS(pImuData->Acc[i][j] - AccSmoother[j]);
			GyroDifSmoother[j] -= GyroDifSmoother[j] / SmoothCount;
			GyroDifSmoother[j] += GyroDif / SmoothCount;
			AccDifSmoother[j] -= AccDifSmoother[j] / SmoothCount;
			AccDifSmoother[j] += AccDif / SmoothCount;
			bDynamic |= ((U8)(GyroDifSmoother[j]>StaticThd[j])) << j;
			bDynamic |= ((U8)(AccDifSmoother[j]>StaticThd[j + 3])) << (j + 3);
		}
	}

	if (SmoothCount > DYNAMIC_SMOOTHOR_LEN / 2)   //
	{
		if (bDynamic)//当6轴器件输出的低频量均很小的时候，认为载体处于静止状态
			g_GINavInfo.StaticCount = 0;   //gty 动态...
		else
			g_GINavInfo.StaticCount++;     //gty 静态
	}
	else
		g_GINavInfo.StaticCount = 0;

	if (g_GINavInfo.StaticCount > 0x00FFFFFF)
		g_GINavInfo.StaticCount = 0x00FFFFFF;
}


//------------------------------------------------------
//void ChangeCoord(PIMU_DATA_T pImuData)
//---------------------------------------------
void ChangeCoord(PIMU_DATA_T pImuData)
{
	S32 i;
	STATIC FLOAT64 AccSmoother[3] = { 0.0 };
	STATIC S32 SmoothCount = 0;
	FLOAT64 SumAcc[3] = { 0 }, gravity;



	if (g_GINavInfo.StaticCount == 0)           //gty 如果动态，则返回.....
		return;

	for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
	{
		SumAcc[0] += pImuData->Acc[i][0];
		SumAcc[1] += pImuData->Acc[i][1];
		SumAcc[2] += pImuData->Acc[i][2];
	}

	SmoothCount++;
	SmoothCount = MIN(SmoothCount, INSTALL_SMOOTHOR_LEN);

	AccSmoother[0] -= AccSmoother[0] / SmoothCount;
	AccSmoother[0] += SumAcc[0] / SmoothCount;
	AccSmoother[1] -= AccSmoother[1] / SmoothCount;
	AccSmoother[1] += SumAcc[1] / SmoothCount;
	AccSmoother[2] -= AccSmoother[2] / SmoothCount;
	AccSmoother[2] += SumAcc[2] / SmoothCount;

	if (SmoothCount < INSTALL_SMOOTHOR_LEN / 2)
		return;

	gravity = sqrt(SQR(AccSmoother[0]) + SQR(AccSmoother[1]) + SQR(AccSmoother[2]));


	if (ABS(gravity - GRAVITY_CONST / INS_UPDATE_RATE) * 10 < 5)
	{

	}

}

//---------------------------------------------
//初始化器件安装矩阵，初始时认为车辆水平姿态角为
//---------------------------------------------
void InitInstallMat(PIMU_DATA_T pImuData)
{
	S32 i;
	STATIC FLOAT64 AccSmoother[3] = { 0.0 };
	STATIC S32 SmoothCount = 0,delta_Heading;
	FLOAT64 SumAcc[3] = { 0 }, gravity;

	FLOAT64 sa, ca, sb, cb, sc, cc;
	FLOAT64 alpha, beta, yaw, deta;

	FLOAT64 temp1Gyro[3], temp1Acc[3];
	FLOAT64 temp2Gyro[3], temp2Acc[3];
	FLOAT64 temp3Gyro[3], temp3Acc[3];

	EULER_T EulerF;


		
	if (g_GINavInfo.StaticCount == 0)           //gty 如果动态，则返回.....
	 {
		 SmoothCount=0;
		 return;
	 }
  
  
	
	for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
	{
		SumAcc[0] += pImuData->Acc[i][0];
		SumAcc[1] += pImuData->Acc[i][1];
		SumAcc[2] += pImuData->Acc[i][2];
	}

	SmoothCount++;
	SmoothCount = MIN(SmoothCount, INSTALL_SMOOTHOR_LEN);

	AccSmoother[0] -= AccSmoother[0] / SmoothCount;
	AccSmoother[0] += SumAcc[0] / SmoothCount;
	AccSmoother[1] -= AccSmoother[1] / SmoothCount;
	AccSmoother[1] += SumAcc[1] / SmoothCount;
	AccSmoother[2] -= AccSmoother[2] / SmoothCount;
	AccSmoother[2] += SumAcc[2] / SmoothCount;

	if (SmoothCount < INSTALL_SMOOTHOR_LEN / 2)
		return;

	gravity = sqrt(SQR(AccSmoother[0]) + SQR(AccSmoother[1]) + SQR(AccSmoother[2]));


	if (g_GINavInfo.Ini1_Flag == 0)
	{
		if (ABS(gravity - GRAVITY_CONST / INS_UPDATE_RATE) * 10 < 5)                      //gty 这个判断一定需要吗？如果进入不了，系统如何处理？
		{
      

			
			//------------------------------------------
			//  换轴
			//------------------------------------------
			if ((ABS(AccSmoother[2])>ABS(AccSmoother[0])) && (ABS(AccSmoother[2])>ABS(AccSmoother[1]))) //如果Z轴大于X轴和Y轴
			{
				if (AccSmoother[2] > 0)
				{
					g_GINavInfo.Ini0_Mat.C11 = -1;
					g_GINavInfo.Ini0_Mat.C12 = 0;
					g_GINavInfo.Ini0_Mat.C13 = 0;

					g_GINavInfo.Ini0_Mat.C21 = 0;
					g_GINavInfo.Ini0_Mat.C22 = 1;
					g_GINavInfo.Ini0_Mat.C23 = 0;

					g_GINavInfo.Ini0_Mat.C31 = 0;
					g_GINavInfo.Ini0_Mat.C32 = 0;
					g_GINavInfo.Ini0_Mat.C33 = -1;

					g_GINavInfo.Ini0_Kind = 2;

				}
				else
				{
					g_GINavInfo.Ini0_Mat.C11 = 1;
					g_GINavInfo.Ini0_Mat.C12 = 0;
					g_GINavInfo.Ini0_Mat.C13 = 0;

					g_GINavInfo.Ini0_Mat.C21 = 0;
					g_GINavInfo.Ini0_Mat.C22 = 1;
					g_GINavInfo.Ini0_Mat.C23 = 0;

					g_GINavInfo.Ini0_Mat.C31 = 0;
					g_GINavInfo.Ini0_Mat.C32 = 0;
					g_GINavInfo.Ini0_Mat.C33 = 1;

					g_GINavInfo.Ini0_Kind = 1;
				}
			}


			if ((ABS(AccSmoother[0])>ABS(AccSmoother[1])) && (ABS(AccSmoother[0])>ABS(AccSmoother[2]))) //如果X轴大于Y轴和Z轴
			{
				if (AccSmoother[0] > 0)
				{
					g_GINavInfo.Ini0_Mat.C11 = 0;
					g_GINavInfo.Ini0_Mat.C12 = 0;
					g_GINavInfo.Ini0_Mat.C13 = 1;

					g_GINavInfo.Ini0_Mat.C21 = 0;
					g_GINavInfo.Ini0_Mat.C22 = 1;
					g_GINavInfo.Ini0_Mat.C23 = 0;

					g_GINavInfo.Ini0_Mat.C31 = -1;
					g_GINavInfo.Ini0_Mat.C32 = 0;
					g_GINavInfo.Ini0_Mat.C33 = 0;

					g_GINavInfo.Ini0_Kind = 4;

				}
				else
				{
					g_GINavInfo.Ini0_Mat.C11 = 0;
					g_GINavInfo.Ini0_Mat.C12 = 0;
					g_GINavInfo.Ini0_Mat.C13 = -1;

					g_GINavInfo.Ini0_Mat.C21 = 0;
					g_GINavInfo.Ini0_Mat.C22 = 1;
					g_GINavInfo.Ini0_Mat.C23 = 0;

					g_GINavInfo.Ini0_Mat.C31 = 1;
					g_GINavInfo.Ini0_Mat.C32 = 0;
					g_GINavInfo.Ini0_Mat.C33 = 0;

					g_GINavInfo.Ini0_Kind = 3;

				}

			}

			if ((ABS(AccSmoother[1])>ABS(AccSmoother[0])) && (ABS(AccSmoother[1])>ABS(AccSmoother[2]))) //如果Y轴大于X轴和Z轴
			{
				if (AccSmoother[1] > 0)
				{
					g_GINavInfo.Ini0_Mat.C11 = 1;
					g_GINavInfo.Ini0_Mat.C12 = 0;
					g_GINavInfo.Ini0_Mat.C13 = 0;

					g_GINavInfo.Ini0_Mat.C21 = 0;
					g_GINavInfo.Ini0_Mat.C22 = 0;
					g_GINavInfo.Ini0_Mat.C23 = 1;

					g_GINavInfo.Ini0_Mat.C31 = 0;
					g_GINavInfo.Ini0_Mat.C32 =-1;
					g_GINavInfo.Ini0_Mat.C33 = 0;

					g_GINavInfo.Ini0_Kind = 6;
				}
				else
				{
					g_GINavInfo.Ini0_Mat.C11 = 1;
					g_GINavInfo.Ini0_Mat.C12 = 0;
					g_GINavInfo.Ini0_Mat.C13 = 0;

					g_GINavInfo.Ini0_Mat.C21 = 0;
					g_GINavInfo.Ini0_Mat.C22 = 0;
					g_GINavInfo.Ini0_Mat.C23 = -1;

					g_GINavInfo.Ini0_Mat.C31 = 0;
					g_GINavInfo.Ini0_Mat.C32 = 1;
					g_GINavInfo.Ini0_Mat.C33 = 0;

					g_GINavInfo.Ini0_Kind = 5;
				}
			}

			g_GINavInfo.Ini0_Flag = 1;


			if (g_GINavInfo.Ini0_Flag == 1)
			{
				for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
				{
					CMRotation(&g_GINavInfo.Ini0_Mat, pImuData->Gyro[i], temp1Gyro);	//Install Matrix Rotation
					CMRotation(&g_GINavInfo.Ini0_Mat, pImuData->Acc[i], temp1Acc);
				}

			}

			//------------------------------------------
			//  roll
			//------------------------------------------
			//------------------------------------------------------------------
			alpha = atan2(temp1Acc[0], -temp1Acc[2]);
			beta = atan2(-temp1Acc[1], -temp1Acc[2]);                    //gty 都加负号？？？
			yaw = (0.0)*DEG2RAD;

			EulerF.Theta = alpha*RAD2DEG;
			EulerF.Gamma = beta*RAD2DEG;

			g_GINavInfo.Ini_Roll = EulerF.Gamma;

			
			sa = sin(0);
			ca = cos(0);
			sb = sin(beta);
			cb = cos(beta);

			sc = sin(0);
			cc = cos(0);

			g_GINavInfo.Ini1_Mat.C11 = cc*ca;
			g_GINavInfo.Ini1_Mat.C12 = cc*sa*sb - sc*cb;
			g_GINavInfo.Ini1_Mat.C13 = cc*sa*cb + sc*sb;

			g_GINavInfo.Ini1_Mat.C21 = sc*ca;
			g_GINavInfo.Ini1_Mat.C22 = sc*sa*sb + cc*cb;
			g_GINavInfo.Ini1_Mat.C23 = sc*sa*cb - cc*sb;

			g_GINavInfo.Ini1_Mat.C31 = -sa;
			g_GINavInfo.Ini1_Mat.C32 = ca*sb;
			g_GINavInfo.Ini1_Mat.C33 = ca*cb;

			g_GINavInfo.Ini1_Flag = 1;

		}
	
	}


	if (g_GINavInfo.Ini1_Flag == 1)
	{
		for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
		{
			CMRotation(&g_GINavInfo.Ini1_Mat, temp1Gyro, temp2Gyro);	//Install Matrix Rotation
			CMRotation(&g_GINavInfo.Ini1_Mat, temp1Acc, temp2Acc);
		}

	}

	//----------------------------------------
	//  pitch
	//------------------------------------------	

	if (g_GINavInfo.Ini1_Flag == 1)
	{

		if (g_GINavInfo.Ini2_Flag == 0)
		{
			alpha = atan2(temp2Acc[0], -temp2Acc[2]);
			beta = atan2(-temp2Acc[1], -temp2Acc[2]);                    //gty 都加负号？？？
			yaw = (0.0)*DEG2RAD;

			EulerF.Theta = alpha*RAD2DEG;
			EulerF.Gamma = beta*RAD2DEG;

			g_GINavInfo.Ini_Pitch = EulerF.Theta;
			
			sa = sin(alpha);
			ca = cos(alpha);
			sb = sin(0);
			cb = cos(0);

			sc = sin(0);
			cc = cos(0);

			g_GINavInfo.Ini2_Mat.C11 = cc*ca;
			g_GINavInfo.Ini2_Mat.C12 = cc*sa*sb - sc*cb;
			g_GINavInfo.Ini2_Mat.C13 = cc*sa*cb + sc*sb;

			g_GINavInfo.Ini2_Mat.C21 = sc*ca;
			g_GINavInfo.Ini2_Mat.C22 = sc*sa*sb + cc*cb;
			g_GINavInfo.Ini2_Mat.C23 = sc*sa*cb - cc*sb;

			g_GINavInfo.Ini2_Mat.C31 = -sa;
			g_GINavInfo.Ini2_Mat.C32 = ca*sb;
			g_GINavInfo.Ini2_Mat.C33 = ca*cb;

			g_GINavInfo.Ini2_Flag = 1;
		}


		if (g_GINavInfo.Ini2_Flag == 1)
		{

			for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
			{
				CMRotation(&g_GINavInfo.Ini2_Mat, temp2Gyro, temp1Gyro);	//Install Matrix Rotation
				CMRotation(&g_GINavInfo.Ini2_Mat, temp2Acc, temp1Acc);

			}
		}
	}


	//----------------------------------------
	//   Yaw
	//-----------------------------------------

	if (g_GINavInfo.Ini2_Flag == 1)
	{
			
		if (g_GINavInfo.Ini3_Flag == 0)
		{
			alpha = atan2(temp1Acc[0], -temp1Acc[2]);
			beta = atan2(-temp1Acc[1], -temp1Acc[2]);                    //gty 都加负号？？？


			EulerF.Phi = alpha*RAD2DEG;
			EulerF.Gamma = beta*RAD2DEG;

			yaw = (0.0)*DEG2RAD;                                       //45度 逆时针旋转

			sa = sin(0);
			ca = cos(0);
			sb = sin(0);
			cb = cos(0);

			sc = sin(yaw);
			cc = cos(yaw);


			g_GINavInfo.Ini3_Mat.C11 = cc*ca;
			g_GINavInfo.Ini3_Mat.C12 = cc*sa*sb - sc*cb;
			g_GINavInfo.Ini3_Mat.C13 = cc*sa*cb + sc*sb;

			g_GINavInfo.Ini3_Mat.C21 = sc*ca;
			g_GINavInfo.Ini3_Mat.C22 = sc*sa*sb + cc*cb;
			g_GINavInfo.Ini3_Mat.C23 = sc*sa*cb - cc*sb;

			g_GINavInfo.Ini3_Mat.C31 = -sa;
			g_GINavInfo.Ini3_Mat.C32 = ca*sb;
			g_GINavInfo.Ini3_Mat.C33 = ca*cb;

			g_GINavInfo.Ini3_Flag = 1;
      

		
	
			g_GINavInfo.ImuCfg.InstallMatInitFlag = 1;
			
			

		}

		if (g_GINavInfo.Ini3_Flag == 1)
		{

			for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
			{
				CMRotation(&g_GINavInfo.Ini3_Mat, temp1Gyro, temp2Gyro);	//Install Matrix Rotation
				CMRotation(&g_GINavInfo.Ini3_Mat, temp1Acc, temp2Acc);

			}
		}		   
	}	
 }

 
 


//多子样捷联更新算法
void INSUpdate(PIMU_DATA_T pImuData, BOOL bPosUpdate, BOOL bVelUpdate, BOOL bAttUpdate)	//n:NED Frame, b:FRD Frame
{
	S32 i;
	FLOAT64 dt = pImuData->MsrInterval / 1000.0;
	FLOAT64 MidV[3], MidWen[3],  temp2[3];
	FLOAT64 temp1[3]={0};
	FLOAT64 Coning[3] = { 0 }, dVscul[3] = { 0 }, SumGyro[3] = { 0 }, SumAcc[3] = { 0 };
	FLOAT64 SumA[3] = { 0 }, SumV[3] = { 0 }, dRscrl[3] = { 0 };
	FLOAT64 SF_b[3], SF_nn[3], SF_n[3], HumfulAcc[3];
	FLOAT64 dRot_ib[3], dRot_ni[3], dR_n[3], dRSF_n[3], dRSF_b[3], dR_rot[3];
	FLOAT64 *pGyro[1 + INS_UPDATE_SAMPLE_NUM], *pAcc[1 + INS_UPDATE_SAMPLE_NUM];
	QUAT_T dQuat_b, dQuat_n, Quat_p, tempQ;
	COSM_T tempC;


	pGyro[0] = g_GINavInfo.LastGyro;
	pAcc[0] = g_GINavInfo.LastAcc;

	//1. extrapolation velocity and wen.
	MidV[0] = g_GINavInfo.Velocity.Vn + g_GINavInfo.dVelocity.Vn * 0.5;
	MidV[1] = g_GINavInfo.Velocity.Ve + g_GINavInfo.dVelocity.Ve * 0.5;
	MidV[2] = -(g_GINavInfo.Velocity.Vu + g_GINavInfo.dVelocity.Vu * 0.5);

	MidWen[0] = MidV[1] / (g_GINavInfo.Rn + g_GINavInfo.Position.Alt); // Ve / (N+h)
	MidWen[1] = -MidV[0] / (g_GINavInfo.Rm + g_GINavInfo.Position.Alt); // -Vn / (M+h)
	MidWen[2] = MidV[1] * g_GINavInfo.CM_ne.C33 / (g_GINavInfo.CM_ne.C31*(g_GINavInfo.Rn + g_GINavInfo.Position.Alt)); // -Ve*tan(lat) / (N+h)	

	//2. calculate Coning/scul/scrl relate items here.
	for (i = 1; i <= INS_UPDATE_SAMPLE_NUM; i++)
	{
		FLOAT64 dGyro[3], dAcc[3], dSumA[3], dSumV[3];
		pGyro[i] = pImuData->Gyro[i - 1];
		pAcc[i] = pImuData->Acc[i - 1];
		//2.1 dRscrl += dVscul*dt
		dRscrl[0] += dVscul[0] * dt;
		dRscrl[1] += dVscul[1] * dt;
		dRscrl[2] += dVscul[2] * dt;

		//2.2 dRscrl += (SumGyro(i-1)-dGyro/12)/2 * (dSumV(i)-SumAcc(i-1)*dt)
		//2.2.1 dGyro = Gyro(i) - Gyro(i-1)
		dGyro[0] = pGyro[i][0] - pGyro[i - 1][0];
		dGyro[1] = pGyro[i][1] - pGyro[i - 1][1];
		dGyro[2] = pGyro[i][2] - pGyro[i - 1][2];
		//2.2.2 temp1 = (SumGyro(i-1)-dGyro/12)/2
		temp1[0] = (SumGyro[0] - dGyro[0] / 12) / 2;
		temp1[1] = (SumGyro[1] - dGyro[1] / 12) / 2;
		temp1[2] = (SumGyro[2] - dGyro[2] / 12) / 2;
		//2.2.3 dSumV = {SumAcc(i-1) + [5*Acc(i)+Acc(i-1)]/12}*dt
		dSumV[0] = (SumAcc[0] + (pAcc[i - 1][0] + 5 * pAcc[i][0]) / 12)*dt;
		dSumV[1] = (SumAcc[1] + (pAcc[i - 1][1] + 5 * pAcc[i][1]) / 12)*dt;
		dSumV[2] = (SumAcc[2] + (pAcc[i - 1][2] + 5 * pAcc[i][2]) / 12)*dt;
		//2.2.4 temp2 = dSumV(i)-SumAcc(i-1)*dt
		temp2[0] = dSumV[0] - SumAcc[0] * dt;
		temp2[1] = dSumV[1] - SumAcc[1] * dt;
		temp2[2] = dSumV[2] - SumAcc[2] * dt;
		//2.2.5 dRscrl += cross(temp1,temp2)
		dRscrl[0] += temp1[1] * temp2[2] - temp2[1] * temp1[2];
		dRscrl[1] += temp2[0] * temp1[2] - temp1[0] * temp2[2];
		dRscrl[2] += temp1[0] * temp2[1] - temp2[0] * temp1[1];

		//2.3 dRscrl += (SumAcc(i-1)-dAcc/12)/2 * (dSumA(i)-SumGyro(i-1)*dt)
		//2.3.1 dAcc = Acc(i)-Acc(i-1)
		dAcc[0] = pAcc[i][0] - pAcc[i - 1][0];
		dAcc[1] = pAcc[i][1] - pAcc[i - 1][1];
		dAcc[2] = pAcc[i][2] - pAcc[i - 1][2];
		//2.3.2 temp1 = (SumAcc(i-1) - dAcc/12)/2
		temp1[0] = (SumAcc[0] - dAcc[0] / 12) / 2;
		temp1[1] = (SumAcc[1] - dAcc[1] / 12) / 2;
		temp1[2] = (SumAcc[2] - dAcc[2] / 12) / 2;
		//2.3.3 dSumA = {SumGyro(i-1) + [5*Gyro(i)+Gyro(i-1)]/12}*dt
		dSumA[0] = (SumGyro[0] + (pGyro[i - 1][0] + 5 * pGyro[i][0]) / 12) * dt;
		dSumA[1] = (SumGyro[1] + (pGyro[i - 1][1] + 5 * pGyro[i][1]) / 12) * dt;
		dSumA[2] = (SumGyro[2] + (pGyro[i - 1][2] + 5 * pGyro[i][2]) / 12) * dt;
		//2.2.4 temp2 = dSumA(i)-SumGyro(i-1)*dt
		temp2[0] = dSumA[0] - SumGyro[0] * dt;
		temp2[1] = dSumA[1] - SumGyro[1] * dt;
		temp2[2] = dSumA[2] - SumGyro[2] * dt;
		//2.2.5 dRscrl += cross(temp,temp2)
		dRscrl[0] += temp1[1] * temp2[2] - temp2[1] * temp1[2];
		dRscrl[1] += temp2[0] * temp1[2] - temp1[0] * temp2[2];
		dRscrl[2] += temp1[0] * temp2[1] - temp2[0] * temp1[1];

		//2.4 dRscrl += cross((SumV + dAcc*dt/24)/6, Gyro(i))
		//2.4.1 temp1 = (SumV + dAcc*dt/24)/6
		temp1[0] = (SumV[0] + dAcc[0] * dt / 24) / 6;
		temp1[1] = (SumV[1] + dAcc[1] * dt / 24) / 6;
		temp1[2] = (SumV[2] + dAcc[2] * dt / 24) / 6;
		//2.4.2 dRscrl += cross(temp1,Gyro(i))
		dRscrl[0] += temp1[1] * pGyro[i][2] - pGyro[i][1] * temp1[2];
		dRscrl[1] += pGyro[i][0] * temp1[2] - temp1[0] * pGyro[i][2];
		dRscrl[2] += temp1[0] * pGyro[i][1] - pGyro[i][0] * temp1[1];

		//2.5 dRscrl -= cross((SumA + dGyro*dt/24)/6, Acc(i))
		//2.5.1 temp1 = (SumA + dGyro*dt/24)/6
		temp1[0] = (SumA[0] + dGyro[0] * dt / 24) / 6;
		temp1[1] = (SumA[1] + dGyro[1] * dt / 24) / 6;
		temp1[2] = (SumA[2] + dGyro[2] * dt / 24) / 6;
		//2.4.2 dRscrl -= cross(temp1,Gyro(i))
		dRscrl[0] -= temp1[1] * pAcc[i][2] - pAcc[i][1] * temp1[2];
		dRscrl[1] -= pAcc[i][0] * temp1[2] - temp1[0] * pAcc[i][2];
		dRscrl[2] -= temp1[0] * pAcc[i][1] - pAcc[i][0] * temp1[1];

		//2.6 dRscrl += cross(SumGyro(i-1)-dGyro/6, SumAcc(i-1)-dAcc/6)*dt/6
		//2.6.1 temp1 = SumGyro(i-1)-dGyro/6
		temp1[0] = SumGyro[0] - dGyro[0] / 6;
		temp1[1] = SumGyro[1] - dGyro[1] / 6;
		temp1[2] = SumGyro[2] - dGyro[2] / 6;
		//2.6.2 temp2 = SumAcc(i-1)-dAcc/6
		temp2[0] = SumAcc[0] - dAcc[0] / 6;
		temp2[1] = SumAcc[1] - dAcc[1] / 6;
		temp2[2] = SumAcc[2] - dAcc[2] / 6;
		//2.6.3 dRscrl += cross(temp1,temp2)*dt/6
		dRscrl[0] += (temp1[1] * temp2[2] - temp2[1] * temp1[2])*dt / 6;
		dRscrl[1] += (temp2[0] * temp1[2] - temp1[0] * temp2[2])*dt / 6;
		dRscrl[2] += (temp1[0] * temp2[1] - temp2[0] * temp1[1])*dt / 6;

		//2.7 dRscrl -= cross(dGyro,dAcc)*dt/2160
		dRscrl[0] -= (dGyro[1] * dAcc[2] - dAcc[1] * dGyro[2])*dt / 2160;
		dRscrl[1] -= (dAcc[0] * dGyro[2] - dGyro[0] * dAcc[2])*dt / 2160;
		dRscrl[2] -= (dGyro[0] * dAcc[1] - dAcc[0] * dGyro[1])*dt / 2160;

		//2.8 temp1 = 6*SumGyro(i-1)+Gyro(i-1) temp2 = 6*SumAcc(i-1)+Acc(i-1)
		temp1[0] = 6 * SumGyro[0] + pGyro[i - 1][0];
		temp1[1] = 6 * SumGyro[1] + pGyro[i - 1][1];
		temp1[2] = 6 * SumGyro[2] + pGyro[i - 1][2];
		temp2[0] = 6 * SumAcc[0] + pAcc[i - 1][0];
		temp2[1] = 6 * SumAcc[1] + pAcc[i - 1][1];
		temp2[2] = 6 * SumAcc[2] + pAcc[i - 1][2];

		//2.9 Coning += cross(temp1,gyro(i))/12
		Coning[0] += (temp1[1] * pGyro[i][2] - pGyro[i][1] * temp1[2]) / 12;
		Coning[1] += (pGyro[i][0] * temp1[2] - temp1[0] * pGyro[i][2]) / 12;
		Coning[2] += (temp1[0] * pGyro[i][1] - pGyro[i][0] * temp1[1]) / 12;

		//2.10 dVscul += cross(temp1,Acc(i))/12 + cross(temp2,Gyro(i))/12
		dVscul[0] += (temp1[1] * pAcc[i][2] - pAcc[i][1] * temp1[2]) / 12;
		dVscul[1] += (pAcc[i][0] * temp1[2] - temp1[0] * pAcc[i][2]) / 12;
		dVscul[2] += (temp1[0] * pAcc[i][1] - pAcc[i][0] * temp1[1]) / 12;
		dVscul[0] += (temp2[1] * pGyro[i][2] - pGyro[i][1] * temp2[2]) / 12;
		dVscul[1] += (pGyro[i][0] * temp2[2] - temp2[0] * pGyro[i][2]) / 12;
		dVscul[2] += (temp2[0] * pGyro[i][1] - pGyro[i][0] * temp2[1]) / 12;

		//2.11 sum item update
		SumGyro[0] += pGyro[i][0];
		SumGyro[1] += pGyro[i][1];
		SumGyro[2] += pGyro[i][2];
		SumAcc[0] += pAcc[i][0];
		SumAcc[1] += pAcc[i][1];
		SumAcc[2] += pAcc[i][2];
		SumA[0] += dSumA[0];
		SumA[1] += dSumA[1];
		SumA[2] += dSumA[2];
		SumV[0] += dSumV[0];
		SumV[1] += dSumV[1];
		SumV[2] += dSumV[2];
	}

	//3. velocity integration
	//3.1 SF_b = SumAcc + dVscul + cross(SumGyro,SumAcc)/2, special force in b frame
	SF_b[0] = SumAcc[0] + dVscul[0] + (SumGyro[1] * SumAcc[2] - SumAcc[1] * SumGyro[2]) / 2;
	SF_b[1] = SumAcc[1] + dVscul[1] + (SumAcc[0] * SumGyro[2] - SumGyro[0] * SumAcc[2]) / 2;
	SF_b[2] = SumAcc[2] + dVscul[2] + (SumGyro[0] * SumAcc[1] - SumAcc[0] * SumGyro[1]) / 2;
	//3.2 Win = (Wie+MidWen)*dt/2, and transform it to CP form
	temp1[0] = -(g_GINavInfo.Wie[0] + MidWen[0])*dt*INS_UPDATE_SAMPLE_NUM / 2;
	temp1[1] = -(g_GINavInfo.Wie[1] + MidWen[1])*dt*INS_UPDATE_SAMPLE_NUM / 2;
	temp1[2] = -(g_GINavInfo.Wie[2] + MidWen[2])*dt*INS_UPDATE_SAMPLE_NUM / 2;
	//3.3 rotate special force to n frame
	CMRotation(&g_GINavInfo.CM_bn, SF_b, SF_nn);
	tempC.C11 = 1.0;		tempC.C12 = -temp1[2];	tempC.C13 = temp1[1];
	tempC.C21 = temp1[2];	tempC.C22 = 1.0;		tempC.C23 = -temp1[0];
	tempC.C31 = -temp1[1];	tempC.C32 = temp1[0];	tempC.C33 = 1.0;
	CMRotation(&tempC, SF_nn, SF_n);
	//3.4 cross(2*Wie+MidWen, MidV)+G
	temp1[0] = 2 * g_GINavInfo.Wie[0] + MidWen[0];
	temp1[1] = 2 * g_GINavInfo.Wie[1] + MidWen[1];
	temp1[2] = 2 * g_GINavInfo.Wie[2] + MidWen[2];
	HumfulAcc[0] = (MidV[1] * temp1[2] - temp1[1] * MidV[2])*dt*INS_UPDATE_SAMPLE_NUM;
	HumfulAcc[1] = (temp1[0] * MidV[2] - MidV[0] * temp1[2])*dt*INS_UPDATE_SAMPLE_NUM;
	HumfulAcc[2] = (MidV[0] * temp1[1] - temp1[0] * MidV[1] + g_GINavInfo.Gravity)*dt*INS_UPDATE_SAMPLE_NUM;
	//3.5 get dV
	g_GINavInfo.dVelocity.Ve = SF_n[1] + HumfulAcc[1];
	g_GINavInfo.dVelocity.Vn = SF_n[0] + HumfulAcc[0];
	g_GINavInfo.dVelocity.Vu = -(SF_n[2] + HumfulAcc[2]);
	//3.6 recalculate MidV and MidWen
	MidV[0] = g_GINavInfo.Velocity.Vn + 0.5*g_GINavInfo.dVelocity.Vn;
	MidV[1] = g_GINavInfo.Velocity.Ve + 0.5*g_GINavInfo.dVelocity.Ve;
	MidV[2] = -(g_GINavInfo.Velocity.Vu + 0.5*g_GINavInfo.dVelocity.Vu);
	MidWen[0] = MidV[1] / (g_GINavInfo.Rn + g_GINavInfo.Position.Alt); // Ve / (N+h)
	MidWen[1] = -MidV[0] / (g_GINavInfo.Rm + g_GINavInfo.Position.Alt); // -Vn / (M+h)
	MidWen[2] = MidV[1] * g_GINavInfo.CM_ne.C33 / (g_GINavInfo.CM_ne.C31*(g_GINavInfo.Rn + g_GINavInfo.Position.Alt)); // -Ve*tan(lat) / (N+h)
	if (bAttUpdate)//in STATIC mode, only update velocity for KF updating
	{
		//section 4: attitude integration
		//4.1 dRot_ib = sum(wib) + Coning
		dRot_ib[0] = SumGyro[0] + Coning[0];
		dRot_ib[1] = SumGyro[1] + Coning[1];
		dRot_ib[2] = SumGyro[2] + Coning[2];
		//4.2 dRot_ni = -dRot_in = -(Wie+MidWen)*dt
		dRot_ni[0] = -(g_GINavInfo.Wie[0] + MidWen[0])*dt*INS_UPDATE_SAMPLE_NUM;
		dRot_ni[1] = -(g_GINavInfo.Wie[1] + MidWen[1])*dt*INS_UPDATE_SAMPLE_NUM;
		dRot_ni[2] = -(g_GINavInfo.Wie[2] + MidWen[2])*dt*INS_UPDATE_SAMPLE_NUM;
		//4.3 transform to Quat and calculate Quat_bn
		RotVec2Quat(dRot_ib, &dQuat_b);
		RotVec2Quat(dRot_ni, &dQuat_n);
		QuatMulti(&g_GINavInfo.Quat_bn, &dQuat_b, &tempQ);
		QuatMulti(&dQuat_n, &tempQ, &g_GINavInfo.Quat_bn);
		NormQuat(&g_GINavInfo.Quat_bn);
	}




	if (bPosUpdate)
	{
		//section 5: position integration
		//5.1 dR_n = (SumAcc(m-1) + HumfulAcc/2)*dt
		dR_n[0] = (g_GINavInfo.Velocity.Vn + 0.5*HumfulAcc[0])*dt*INS_UPDATE_SAMPLE_NUM;
		dR_n[1] = (g_GINavInfo.Velocity.Ve + 0.5*HumfulAcc[1])*dt*INS_UPDATE_SAMPLE_NUM;
		dR_n[2] = (-g_GINavInfo.Velocity.Vu + 0.5*HumfulAcc[2])*dt*INS_UPDATE_SAMPLE_NUM;
		//5.1.1 dR_rot = (1/6*eye + 1/24*[SumGyro]) * (SumA * SumAcc + SumGyro * SumV)
		tempC.C11 = 1.0 / 6.0;			tempC.C12 = -SumGyro[2] / 24;	tempC.C13 = SumGyro[1] / 24;
		tempC.C21 = SumGyro[2] / 24;	tempC.C22 = 1.0 / 6.0;			tempC.C23 = -SumGyro[0] / 24;
		tempC.C31 = -SumGyro[1] / 24;	tempC.C32 = SumGyro[0] / 24;	tempC.C33 = 1.0 / 6.0;
		temp1[0] = SumA[1] * SumAcc[2] - SumAcc[1] * SumA[2];
		temp1[1] = SumAcc[0] * SumA[2] - SumA[0] * SumAcc[2];
		temp1[2] = SumA[0] * SumAcc[1] - SumAcc[0] * SumA[1];
		temp1[0] += SumGyro[1] * SumV[2] - SumV[1] * SumGyro[2];
		temp1[1] += SumV[0] * SumGyro[2] - SumGyro[0] * SumV[2];
		temp1[2] += SumGyro[0] * SumV[1] - SumV[0] * SumGyro[1];
		CMRotation(&tempC, temp1, dR_rot);
		//5.1.2 dRSF_b = SumV + dR_rot + dRscrl
		dRSF_b[0] = SumV[0] + dR_rot[0] + dRscrl[0];
		dRSF_b[1] = SumV[1] + dR_rot[1] + dRscrl[1];
		dRSF_b[2] = SumV[2] + dR_rot[2] + dRscrl[2];
		//5.1.3 dRSF_n = Cbn*dRSF_b
		CMRotation(&g_GINavInfo.CM_bn, dRSF_b, dRSF_n);
		//5.1.4 dRSF_n += 1/6*(Cnn - eye)*SF_nn*dt
		tempC.C11 = 0;					tempC.C12 = -dRot_ni[2] / 6;	tempC.C13 = dRot_ni[1] / 6;
		tempC.C21 = dRot_ni[2] / 6;		tempC.C22 = 0;					tempC.C23 = -dRot_ni[0] / 6;
		tempC.C31 = -dRot_ni[1] / 6;	tempC.C32 = dRot_ni[0] / 6;		tempC.C33 = 0;
		CMRotation(&tempC, SF_nn, temp1);
		dRSF_n[0] += temp1[0] * dt*INS_UPDATE_SAMPLE_NUM;
		dRSF_n[1] += temp1[1] * dt*INS_UPDATE_SAMPLE_NUM;
		dRSF_n[2] += temp1[2] * dt*INS_UPDATE_SAMPLE_NUM;
		//5.1.5 dR_n += dRSF_n
		dR_n[0] += dRSF_n[0];
		dR_n[1] += dRSF_n[1];
		dR_n[2] += dRSF_n[2];
		//5.2 transport rate increment (position rotation vector); 
		//5.2.1 rotation vector: temp = wen*dt
		temp1[0] = dR_n[1] / (g_GINavInfo.Rn + g_GINavInfo.Position.Alt); // Ve / (N+h)
		temp1[1] = -dR_n[0] / (g_GINavInfo.Rm + g_GINavInfo.Position.Alt); // -Vn / (M+h)
		temp1[2] = dR_n[1] * g_GINavInfo.CM_ne.C33 / (g_GINavInfo.CM_ne.C31*(g_GINavInfo.Rn + g_GINavInfo.Position.Alt)); // -Ve*tan(lat) / (N+h) 
		//5.2.2 transform it to quat_p, quat_p--platform attitude relative to i frame
		RotVec2Quat(temp1, &Quat_p);
		tempQ.Scalar = g_GINavInfo.Quat_ne.Scalar;
		tempQ.Vector[0] = g_GINavInfo.Quat_ne.Vector[0];
		tempQ.Vector[1] = g_GINavInfo.Quat_ne.Vector[1];
		tempQ.Vector[2] = g_GINavInfo.Quat_ne.Vector[2];
		//5.2.3 update Quat_ne, and get updated position
		QuatMulti(&tempQ, &Quat_p, &g_GINavInfo.Quat_ne);
		NormQuat(&g_GINavInfo.Quat_ne);
		Quat2CM(&g_GINavInfo.Quat_ne, &g_GINavInfo.CM_ne);
		g_GINavInfo.Position.Lat = -asin(g_GINavInfo.CM_ne.C33);
		g_GINavInfo.Position.Lon = atan2(-g_GINavInfo.CM_ne.C23, -g_GINavInfo.CM_ne.C13);
		g_GINavInfo.Position.Alt -= dR_n[2];
	}



	if (bAttUpdate)
	{
		//5.2.4 update some parameters
		Quat2CM(&g_GINavInfo.Quat_bn, &g_GINavInfo.CM_bn);
		CM2Euler(&g_GINavInfo.CM_bn, &g_GINavInfo.Euler);
	}
	if (bVelUpdate)
	{
		//6. update velocity
		g_GINavInfo.Velocity.Ve += g_GINavInfo.dVelocity.Ve;
		g_GINavInfo.Velocity.Vn += g_GINavInfo.dVelocity.Vn;
		g_GINavInfo.Velocity.Vu += g_GINavInfo.dVelocity.Vu;
	}

	//7. calculate other parameters
	g_GINavInfo.SF_n[0] = SF_n[0];
	g_GINavInfo.SF_n[1] = SF_n[1];
	g_GINavInfo.SF_n[2] = SF_n[2];

	if (bPosUpdate)
	{
		g_GINavInfo.Gravity = GetLocalGravity(-g_GINavInfo.CM_ne.C33, g_GINavInfo.Position.Alt);
		temp1[0] = sqrt(1.0 - WGS_E1_SQR * g_GINavInfo.CM_ne.C33 * g_GINavInfo.CM_ne.C33);
		g_GINavInfo.Rm = WGS_AXIS_A*(1 - WGS_E1_SQR) / (temp1[0] * temp1[0] * temp1[0]);
		g_GINavInfo.Rn = WGS_AXIS_A / temp1[0];
		g_GINavInfo.Wie[0] = WGS_OMEGDOTE * g_GINavInfo.CM_ne.C31;
		g_GINavInfo.Wie[1] = 0;
		g_GINavInfo.Wie[2] = WGS_OMEGDOTE * g_GINavInfo.CM_ne.C33;
	}

	if (bVelUpdate)
	{
		g_GINavInfo.Wen[0] = g_GINavInfo.Velocity.Ve / (g_GINavInfo.Rn + g_GINavInfo.Position.Alt); // Ve / (N+h)
		g_GINavInfo.Wen[1] = -g_GINavInfo.Velocity.Vn / (g_GINavInfo.Rm + g_GINavInfo.Position.Alt); // -Vn / (M+h)
		g_GINavInfo.Wen[2] = g_GINavInfo.Velocity.Ve*g_GINavInfo.CM_ne.C33 / (g_GINavInfo.CM_ne.C31*(g_GINavInfo.Rn + g_GINavInfo.Position.Alt)); // -Ve*tan(lat) / (N+h)
	}

}
