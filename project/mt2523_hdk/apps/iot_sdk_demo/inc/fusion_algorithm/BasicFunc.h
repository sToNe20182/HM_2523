#ifndef __BASICFUNC_H__
#define __BASICFUNC_H__

#include "DataTypes.h"

#define ABS(x) (((x) > 0) ? (x) : -(x))

#define SQR(x) (x*x)

#define MIN(a,b) (a <= b ? a : b)
#define MAX(a,b) (a >= b ? a : b)

void GetGpsTime(const PUTC_T pUTCTime, PGPST_T pGPSTime);

/*S32 GetGPSTDif(const PGPST_T pGPSTime1, const PGPST_T pGPSTime2);

S32 GetUTCDif(const PUTC_T pUTCTime1, const PUTC_T pUTCTime2);

void UTCRetrieve(PUTC_T pUTCTime, S16 MsCount);*/

void CMRotation(const PCOSM_T pCM, const FLOAT64* pV, FLOAT64* pVout);

void RotVec2Quat(const FLOAT64* pRV, PQUAT_T pQout);

void QuatMulti(const PQUAT_T pQ1, const PQUAT_T pQ2, PQUAT_T pQout);

void NormQuat(PQUAT_T pQ);

void Quat2CM(const PQUAT_T pQ, PCOSM_T pCM);

void CM2Quat(const PCOSM_T pCM, PQUAT_T pQ);

void Euler2CM(const PEULER_T pEuler, PCOSM_T pCM);

void Euler2Quat(const PEULER_T pEuler, PQUAT_T pQ);

void CM2Euler(const PCOSM_T pCM, PEULER_T pEuler);

void GetNavFrameQuat(const PPOS_T pPos, PQUAT_T pQ);

FLOAT64 GetLocalGravity(FLOAT64 sLat, FLOAT64 Alt);

void AddMxMxMt(FLOAT64 **m_out, const FLOAT64* m1, FLOAT64* m2[], const FLOAT64* m3, const S32 n1, const S32 n2, const S32 n3, const S32 n4, const U8 bDiag);

void AddMxMtxMt(FLOAT64 **m_out, const FLOAT64* m1, FLOAT64* m2[], const FLOAT64* m3, const S32 n1, const S32 n2, const S32 n3, const S32 n4, const U8 bDiag);

void AddMxM(FLOAT64 **m_out, const FLOAT64* m1, FLOAT64* m2[], const S32 n1, const S32 n2, const S32 n3, const U8 bDiag);

void AddMxMt(FLOAT64 **m_out, FLOAT64* m1[], const FLOAT64* m2, const S32 n1, const S32 n2, const S32 n3, const U8 bDiag);

void AddMtxMt(FLOAT64 **m_out, FLOAT64* m1[], const FLOAT64* m2, const S32 n1, const S32 n2, const S32 n3, const U8 bDiag);

void Addequal(FLOAT64 **m_out, FLOAT64* m[], const S32 n1, const S32 n2, const U8 bDiag);

BOOL TriangleMatInv(FLOAT64* mat, U32 n);

U32 GetBitNum(U32* pBits, U8 count);

U32 GetNextSvid(U32* pBits, U32* index);

#endif
