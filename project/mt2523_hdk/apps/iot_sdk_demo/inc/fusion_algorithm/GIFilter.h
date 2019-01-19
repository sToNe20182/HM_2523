#ifndef __GIFILTER_H__
#define __GIFILTER_H__

#include "Const.h"
#include "DataTypes.h"
#include "GINavMain.h"

#define MAX_MSR_DIM		6		//Measurements Dimension
#define STATE_DIM	15		//KF State Vector Dimensio




#define PMTX_SIZE	(STATE_DIM*(STATE_DIM + 1)/2)	//P Matrix Size
#define QC_BIAS_THRE       0.02        /* QC?D¨¢???¨º?¡¤?3??T?D???D?¦Ìdeg/s*/
#define QC_CONVER_THRE     60         /* QC?D?¨¤¦Ì??¡§??¨º?¨¢2¦Ì??D?¦Ì*/



static FLOAT64 Gyroz_Bias_Buffer[QC_CONVER_THRE] = { 0.0 };
static FLOAT64 Gyro_Bias_Max;
static FLOAT64 Gyro_Bias_Min;

// P matrix arrange
//	dE:		  0
//	dN:		  1   2
//	dU:		  3   4   5
//	dVe:	  6   7   8   9
//	dVn:	 10  11  12  13  14
//	dVu:	 15  16  17  18  19  20
//	Phix;	 21  22  23  24  25  26  27
//	Phiy;	 28  29  30  31  32  33  34  35
//	Phiz;	 36  37  38  39  40  41  42  43  44
//	dBax:	 45  46  47  48  49  50  51  52  53  54
//	dBay:	 55  56  57  58  59  60  61  62  63  64  65
//	dBaz:	 66  67  68  69  70  71  72  73  74  75  76  77
//	dBgx:	 78  79  80  81  82  83  84  85  86  87  88  89  90
//	dBgy:	 91  92  93  94  95  96  97  98  99 100 101 102 103 104
//	dBgz:	105 106 107 108 109 110 111 112 113 114 115 116 117 118 119

typedef struct{
	FLOAT64 X[STATE_DIM];
	FLOAT64 Q[STATE_DIM];
	FLOAT64 P[PMTX_SIZE];
	//PHI non-zero blocks
	FLOAT64 Fpp[9];
	FLOAT64 Fpv[9];
	FLOAT64 Fvp[9];
	FLOAT64 Fvv[9];
	FLOAT64 Fva[9];
	FLOAT64 Faa[9];
	FLOAT64 Fvba[9];
	FLOAT64 Fabg[9];
	FLOAT64 Fvba_abg[9];
} GIKF_DATA_T;

// IMU Noise feature
#define ACC_CONST_BIAS				(100	*1.0e-6*GRAVITY_CONST)	//acc constant bias stability: ug
#define GYRO_CONST_BIAS				(0.025		*DEG2RAD/3600.0)		//gyro constant bias stability: deg/h
#define VEL_RAND_WALK				(500	*1.0e-6*GRAVITY_CONST)	//velocity random walk: ug/sqrt(Hz)
#define ANG_RAND_WALK				(12.5		*DEG2RAD/60.0)			//angular random walk: deg/sqrt(h)
/*#define ACC_CONST_BIAS				(762.097228	*1.0e-6*GRAVITY_CONST)	//acc constant bias stability: ug
#define GYRO_CONST_BIAS				(50.0		*DEG2RAD/3600.0)		//gyro constant bias stability: deg/h
#define VEL_RAND_WALK				(50	*1.0e-6*GRAVITY_CONST)	//velocity random walk: ug/sqrt(Hz)
#define ANG_RAND_WALK				(0.05		*DEG2RAD/60.0)			//angular random walk: deg/sqrt(h)*/
//Other Noise feature
#define POS_RAND_WALK_LEVEL			0.0
#define POS_RAND_WALK_ALT			0.0

void GIKFInit(void);
void GetQMatrix(void);
void GIKFInitPMatrix(void);
void GIKFModularizePMatrix(FLOAT64 *pBlock[][3], FLOAT64 *pmtx);
void GIKFCalcPHIMatrix(U32 MsInterval);
void GIKFPredictPMatrix(U32 MsInterval);
void GIKFBatchSolution(U32 flag, U32 MsrCount, FLOAT64* HMatrix, FLOAT64* Msr, FLOAT64* Sigma);
BOOL GIKFUpdateByGNSS(PGNSS_DATA_T pGnssData,PIMU_DATA_T pImuData);
BOOL GIKFUpdateByRMC(PGNSS_DATA_T pGnssData,PIMU_DATA_T pImuData);
BOOL GIKFUpdateByNHC(PIMU_DATA_T pImuData);
BOOL GIKFUpdateByStatic(PIMU_DATA_T pImuData);
void GIKFINSErrorFix(U8 RunKind);
BOOL GIKFCheckPMatrix(void);
BOOL GIKFCheckGIResult(void);
BOOL GIKFINSBiasFix(PGNSS_DATA_T pGnssData);

#endif
