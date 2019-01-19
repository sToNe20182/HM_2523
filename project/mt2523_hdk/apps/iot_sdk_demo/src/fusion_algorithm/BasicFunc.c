#include "BasicFunc.h"
#include "Config.h"
#include "Const.h"
#include <math.h>
#include <stdlib.h>

STATIC S32 DaysAcc[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
void GetGpsTime(const PUTC_T pUTCTime, PGPST_T pGPSTime)      //计算GPS周和周秒...
{
	S32 Years, LeapYears, TotalDays, DayMsCount;
	if (pUTCTime->Year == 0)
	{
		pGPSTime->WeekNumber = 0;
		pGPSTime->WeekMillSecond = pUTCTime->MillSecond;
		return;
	}
	DayMsCount = (((pUTCTime->Hour * 60) + pUTCTime->Minute) * 60 + pUTCTime->Second) * 1000 + pUTCTime->MillSecond;
	Years = pUTCTime->Year - 1992;
	TotalDays = DaysAcc[pUTCTime->Month - 1] + pUTCTime->Day - 1;
	if ((Years % 4) != 0 || TotalDays >= 59)
		TotalDays++;
	TotalDays += (Years % 4) * 365;
	LeapYears = Years / 4;
	DayMsCount += LEAP_SECONDS * 1000;
	if (DayMsCount >= 86400000)
	{
		DayMsCount -= 86400000;
		TotalDays++;
	}
	else if (DayMsCount < 0)
	{
		DayMsCount += 86400000;
		TotalDays--;
	}
	TotalDays += (LeapYears + 2) * (366 + 365 * 3);
	pGPSTime->WeekNumber = TotalDays / 7 + 208;
	pGPSTime->WeekMillSecond = (TotalDays % 7) * 86400000 + DayMsCount;
}

/*S32 GetGPSTDif(const PGPST_T pGPSTime1, const PGPST_T pGPSTime2)
{
	S32 DiffWeek = pGPSTime1->WeekNumber - pGPSTime2->WeekNumber;
	S32 DiffMillSecond = pGPSTime1->WeekMillSecond - pGPSTime2->WeekMillSecond;
	return DiffWeek * 604800 + DiffMillSecond;
}

S32 GetUTCDif(const PUTC_T pUTCTime1, const PUTC_T pUTCTime2)
{
	GPST_T GpsTime1, GpsTime2;
	GetGpsTime(pUTCTime1, &GpsTime1);
	GetGpsTime(pUTCTime2, &GpsTime2);
	return GetGPSTDif(&GpsTime1, &GpsTime2);
}

void UTCRetrieve(PUTC_T pUTCTime, S16 MsCount)
{
	pUTCTime->MillSecond -= MsCount;
	if (pUTCTime->MillSecond < 0)
	{
		S32 k = (-pUTCTime->MillSecond) % 1000 + 1;
		pUTCTime->Second -= k;
		pUTCTime->MillSecond += k * 1000;
		while (pUTCTime->Second < 0)
		{
			pUTCTime->Minute--;
			pUTCTime->Second += 60;
		}
		while (pUTCTime->Minute < 0)
		{
			pUTCTime->Hour--;
 			pUTCTime->Minute += 60;
		}
		while (pUTCTime->Hour < 0)
		{
			pUTCTime->Day--;
			pUTCTime->Hour += 24;
		}
		while (pUTCTime->Day < 1)
		{
			pUTCTime->Month--;
			if (pUTCTime->Month == 1 || pUTCTime->Month == 3 ||
				pUTCTime->Month == 5 || pUTCTime->Month == 7 ||
				pUTCTime->Month == 8 || pUTCTime->Month == 10 ||
				pUTCTime->Month == 12)
				pUTCTime->Day += 31;
			else
			{
				if (pUTCTime->Month != 2)
					pUTCTime->Day += 30;
				else
				{
					if (pUTCTime->Year % 4 != 0 || (pUTCTime->Year % 100 == 0 && pUTCTime->Year % 400 != 0))
						pUTCTime->Day += 28;
					else
						pUTCTime->Day += 29;
				}
			}
		}
		while (pUTCTime->Month < 1)
		{
			pUTCTime->Year--;
			pUTCTime->Month += 12;
		}
	}
}*/

void CMRotation(const PCOSM_T pCM, const FLOAT64* pV, FLOAT64* pVout)
{
	pVout[0] = pCM->C11*pV[0] + pCM->C12*pV[1] + pCM->C13*pV[2];
	pVout[1] = pCM->C21*pV[0] + pCM->C22*pV[1] + pCM->C23*pV[2];
	pVout[2] = pCM->C31*pV[0] + pCM->C32*pV[1] + pCM->C33*pV[2];
}

void RotVec2Quat(const FLOAT64* pRV, PQUAT_T pQout)
{
	FLOAT64 mag2;
	mag2 = pRV[0] * pRV[0] + pRV[1] * pRV[1] + pRV[2] * pRV[2];

	if (mag2 < PI*PI)
	{
		FLOAT64 c, s;
		 
		mag2 = mag2/4.0;

		c = 1.0 - (mag2 / 2.0) * (1.0 - (mag2 / 12.0) * (1.0 - mag2 / 30.0));
		s = 0.5 - (mag2 / 12.0) * (1.0 - (mag2 / 20.0) * (1.0 - mag2 / 42.0));
		pQout->Scalar = c;
		pQout->Vector[0] = s*pRV[0];
		pQout->Vector[1] = s*pRV[1];
		pQout->Vector[2] = s*pRV[2];
	}
	else
	{
		FLOAT64 mag, s_mag;
		mag = sqrt(mag2);
		s_mag = sin(mag / 2);
		pQout->Scalar = cos(mag / 2);
		pQout->Vector[0] = pRV[0] * s_mag / mag;
		pQout->Vector[1] = pRV[1] * s_mag / mag;
		pQout->Vector[2] = pRV[2] * s_mag / mag;
		if (pQout->Scalar < 0)
		{
			pQout->Scalar = -pQout->Scalar;
			pQout->Vector[0] = -pQout->Vector[0];
			pQout->Vector[1] = -pQout->Vector[1];
			pQout->Vector[2] = -pQout->Vector[2];
		}
	}
}

void QuatMulti(const PQUAT_T pQ1, const PQUAT_T pQ2, PQUAT_T pQout)
{
	pQout->Scalar = pQ1->Scalar * pQ2->Scalar - pQ1->Vector[0] * pQ2->Vector[0] -
		pQ1->Vector[1] * pQ2->Vector[1] - pQ1->Vector[2] * pQ2->Vector[2];
	pQout->Vector[0] = pQ1->Scalar * pQ2->Vector[0] + pQ1->Vector[0] * pQ2->Scalar +
		pQ1->Vector[1] * pQ2->Vector[2] - pQ1->Vector[2] * pQ2->Vector[1];
	pQout->Vector[1] = pQ1->Scalar * pQ2->Vector[1] + pQ1->Vector[1] * pQ2->Scalar +
		pQ1->Vector[2] * pQ2->Vector[0] - pQ1->Vector[0] * pQ2->Vector[2];
	pQout->Vector[2] = pQ1->Scalar * pQ2->Vector[2] + pQ1->Vector[2] * pQ2->Scalar +
		pQ1->Vector[0] * pQ2->Vector[1] - pQ1->Vector[1] * pQ2->Vector[0];
	
	//------------------------------------------
	//
	//------------------------------------------
	if (pQout->Scalar < 0)
	{
		pQout->Scalar = -pQout->Scalar;
		pQout->Vector[0] = -pQout->Vector[0];
		pQout->Vector[1] = -pQout->Vector[1];
		pQout->Vector[2] = -pQout->Vector[2];
	}
	
}

void NormQuat(PQUAT_T pQ)
{
	FLOAT64 e = pQ->Scalar*pQ->Scalar + pQ->Vector[0] * pQ->Vector[0] +
		pQ->Vector[1] * pQ->Vector[1] + pQ->Vector[2] * pQ->Vector[2];
	e = 1.5 - e / 2;
	pQ->Scalar *= e;
	pQ->Vector[0] *= e;
	pQ->Vector[1] *= e;
	pQ->Vector[2] *= e;
}

void Quat2CM(const PQUAT_T pQ, PCOSM_T pCM)
{
	FLOAT64 q00 = pQ->Scalar*pQ->Scalar,
		q11 = pQ->Vector[0] * pQ->Vector[0],
		q22 = pQ->Vector[1] * pQ->Vector[1],
		q33 = pQ->Vector[2] * pQ->Vector[2];
	FLOAT64 q01 = pQ->Scalar*pQ->Vector[0],
		q02 = pQ->Scalar*pQ->Vector[1],
		q03 = pQ->Scalar*pQ->Vector[2];
	FLOAT64 q12 = pQ->Vector[0] * pQ->Vector[1],
		q13 = pQ->Vector[0] * pQ->Vector[2];
	FLOAT64 q23 = pQ->Vector[1] * pQ->Vector[2];

	pCM->C11 = q00 + q11 - q22 - q33;
	pCM->C12 = 2 * (q12 - q03);
	pCM->C13 = 2 * (q13 + q02);
	pCM->C21 = 2 * (q12 + q03);
	pCM->C22 = q00 - q11 + q22 - q33;
	pCM->C23 = 2 * (q23 - q01);
	pCM->C31 = 2 * (q13 - q02);
	pCM->C32 = 2 * (q23 + q01);
	pCM->C33 = q00 - q11 - q22 + q33;
}

void CM2Quat(const PCOSM_T pCM, PQUAT_T pQ)
{
	FLOAT64 sign;
	pQ->Scalar = sqrt(1 + pCM->C11 + pCM->C22 + pCM->C33)/2.0;
	sign = pCM->C32 > pCM->C23 ? 1.0 : -1.0;
	pQ->Vector[0] = sign*sqrt(1 + pCM->C11 - pCM->C22 - pCM->C33) / 2.0;
	sign = pCM->C13 > pCM->C31 ? 1.0 : -1.0;
	pQ->Vector[1] = sign*sqrt(1 - pCM->C11 + pCM->C22 - pCM->C33) / 2.0;
	sign = pCM->C21 > pCM->C12 ? 1.0 : -1.0;
	pQ->Vector[2] = sign*sqrt(1 - pCM->C11 - pCM->C22 + pCM->C33) / 2.0;
}

void Euler2Quat(const PEULER_T pEuler, PQUAT_T pQ)
{
	FLOAT64 cPhi, sPhi, cTheta, sTheta, cGamma, sGamma;
	cPhi = cos(pEuler->Phi / 2);
	cTheta = cos(pEuler->Theta / 2);
	cGamma = cos(pEuler->Gamma / 2);
	sPhi = sin(pEuler->Phi / 2);
	sTheta = sin(pEuler->Theta / 2);
	sGamma = sin(pEuler->Gamma / 2);
	pQ->Scalar = cGamma*cTheta*cPhi + sGamma*sTheta*sPhi;
	pQ->Vector[0] = sGamma*cTheta*cPhi - cGamma*sTheta*sPhi;
	pQ->Vector[1] = cGamma*sTheta*cPhi + sGamma*cTheta*sPhi;
	pQ->Vector[2] = cGamma*cTheta*sPhi - sGamma*sTheta*cPhi;
}

void CM2Euler(const PCOSM_T pCM, PEULER_T pEuler)
{
	FLOAT64 divider = sqrt(pCM->C32*pCM->C32 + pCM->C33*pCM->C33);
	//------------------------------------------
	//
	//------------------------------------------
	if (divider > 1e-4)
		divider = divider;
   else		 
		divider = 1e-4;
	 
	pEuler->Phi = atan2(pCM->C21, pCM->C11);//(-pi, pi]
	pEuler->Theta = atan(-pCM->C31 / divider);//[-pi/2, pi/2]
	pEuler->Gamma = atan2(pCM->C32, pCM->C33);//(-pi, pi]
	if (pEuler->Phi + 1 < 1)
		pEuler->Phi += 2 * PI;
}

void GetNavFrameQuat(const PPOS_T pPos, PQUAT_T pQ)//Qne
{
	FLOAT64 c1 = cos(pPos->Lon / 2);
	FLOAT64 s1 = sin(pPos->Lon / 2);
	FLOAT64 c2 = cos(-PI / 4 - pPos->Lat / 2);
	FLOAT64 s2 = sin(-PI / 4 - pPos->Lat / 2);
	pQ->Scalar = c1 * c2;
	pQ->Vector[0] = -s1 * s2;
	pQ->Vector[1] = c1 * s2;
	pQ->Vector[2] = s1 * c2;
}

FLOAT64 GetLocalGravity(FLOAT64 sLat, FLOAT64 Alt)
{
	FLOAT64 sTheta = sLat*sLat;
	FLOAT64 s4 = sTheta*sTheta;
	return 9.7803267715/*GRAVITY_CONST*/ * (1 + 0.0052790414*sTheta + 0.0000232718*s4) +
		(-0.000003087691089 + 0.000000004397731*sTheta)*Alt + 0.000000000000721*Alt*Alt;
}

/**************************************************************************
m_out[n1*n4] += m1[n1*n2]*m2[n2*n3]*(m3[n4*n3])'
**************************************************************************/
void AddMxMxMt(FLOAT64 **m_out, const FLOAT64* m1, FLOAT64* m2[], const FLOAT64* m3, const S32 n1, const S32 n2, const S32 n3, const S32 n4, const U8 bDiag)
{
	S32 i, j, k;
	FLOAT64 *temp = (FLOAT64*)malloc(SIZEOF(FLOAT64)*n1*n3);
	if ((bDiag & 0x01) == 0)
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < n3; j++)
			{
				temp[i*n3 + j] = 0.0;
				for (k = 0; k < n2; k++)
				{
					temp[i*n3 + j] += m1[i*n2 + k] * m2[k][j];
				}
			}
		}
	}
	else
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < n3; j++)
			{
				temp[i*n3 + j] = 0.0;
				for (k = 0; k < n2; k++)
				{
					if (j <= k)
						temp[i*n3 + j] += m1[i*n2 + k] * m2[k][j];
					else
						temp[i*n3 + j] += m1[i*n2 + k] * m2[j][k];
				}
			}
		}
	}

	for (i = 0; i < n1; i++)
	{
		for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n4); j++)
		{
			for (k = 0; k < n3; k++)
				m_out[i][j] += temp[i*n3 + k] * m3[j*n3 + k];
		}
	}
	free(temp);
}

/**************************************************************************
m_out[n1*n4] += m1[n1*n2]*(m2[n3*n2])'*(m3[n4*n3])'
**************************************************************************/
void AddMxMtxMt(FLOAT64 **m_out, const FLOAT64* m1, FLOAT64* m2[], const FLOAT64* m3, const S32 n1, const S32 n2, const S32 n3, const S32 n4, const U8 bDiag)
{
	S32 i, j, k;
	FLOAT64 *temp = (FLOAT64*)malloc(SIZEOF(FLOAT64)*n1*n3);
	if ((bDiag & 0x01) == 0)
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < n3; j++)
			{
				temp[i*n3 + j] = 0.0;
				for (k = 0; k < n2; k++)
				{
					temp[i*n3 + j] += m1[i*n2 + k] * m2[j][k];
				}
			}
		}
	}
	else
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < n3; j++)
			{
				temp[i*n3 + j] = 0.0;
				for (k = 0; k < n2; k++)
				{
					if (k <= j)
						temp[i*n3 + j] += m1[i*n2 + k] * m2[j][k];
					else
						temp[i*n3 + j] += m1[i*n2 + k] * m2[k][j];
				}
			}
		}
	}

	for (i = 0; i < n1; i++)
	{
		for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n4); j++)
		{
			for (k = 0; k < n3; k++)
				m_out[i][j] += temp[i*n3 + k] * m3[j*n3 + k];
		}
	}
	free(temp);
}

/**************************************************************************
m_out[n1*n3] += m1[n1*n2]*m2[n2*n3]
**************************************************************************/
void AddMxM(FLOAT64 **m_out, const FLOAT64* m1, FLOAT64* m2[], const S32 n1, const S32 n2, const S32 n3, const U8 bDiag)
{
	S32 i, j, k;
	if ((bDiag & 0x01) == 0)
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n3); j++)
			{
				for (k = 0; k < n2; k++)
				{
					m_out[i][j] += m1[i*n2 + k] * m2[k][j];
				}
			}
		}
	}
	else
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n3); j++)
			{
				for (k = 0; k < n2; k++)
				{
					if (j <= k)
						m_out[i][j] += m1[i*n2 + k] * m2[k][j];
					else
						m_out[i][j] += m1[i*n2 + k] * m2[j][k];
				}
			}
		}
	}
}

/**************************************************************************
m_out[n1*n3] += m1[n1*n2]*(m2[n3*n2])'
**************************************************************************/
void AddMxMt(FLOAT64 **m_out, FLOAT64* m1[], const FLOAT64* m2, const S32 n1, const S32 n2, const S32 n3, const U8 bDiag)
{
	S32 i, j, k;
	if ((bDiag & 0x01) == 0)
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n3); j++)
			{
				for (k = 0; k < n2; k++)
				{
					m_out[i][j] += m1[i][k] * m2[j*n2 + k];
				}
			}
		}
	}
	else
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n3); j++)
			{
				for (k = 0; k < n2; k++)
				{
					if (k <= i)
						m_out[i][j] += m1[i][k] * m2[j*n2 + k];
					else
						m_out[i][j] += m1[k][i] * m2[j*n2 + k];
				}
			}
		}
	}
}

/**************************************************************************
m_out[n1*n3] += (m1[n2*n1])'*(m2[n3*n2])'
**************************************************************************/
void AddMtxMt(FLOAT64 **m_out, FLOAT64* m1[], const FLOAT64* m2, const S32 n1, const S32 n2, const S32 n3, const U8 bDiag)
{
	S32 i, j, k;
	if ((bDiag & 0x01) == 0)
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n3); j++)
			{
				for (k = 0; k < n2; k++)
				{
					m_out[i][j] += m1[k][i] * m2[j*n2 + k];
				}
			}
		}
	}
	else
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n3); j++)
			{
				for (k = 0; k < n2; k++)
				{
					if (i <= k)
						m_out[i][j] += m1[k][i] * m2[j*n2 + k];
					else
						m_out[i][j] += m1[i][k] * m2[j*n2 + k];
				}
			}
		}
	}
}

/**************************************************************************
m_out[n1*n2] += m1[n1*n2]
**************************************************************************/
void Addequal(FLOAT64 **m_out, FLOAT64* m[], const S32 n1, const S32 n2, const U8 bDiag)
{
	S32 i, j;
	if ((bDiag & 0x01) == 0)
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n2); j++)
			{
				m_out[i][j] += m[i][j];
			}
		}
	}
	else
	{
		for (i = 0; i < n1; i++)
		{
			for (j = 0; j < ((bDiag & 0x02) > 0 ? (i + 1) : n2); j++)
			{
				if (j <= i)
					m_out[i][j] += m[i][j];
				else
					m_out[i][j] += m[j][i];
			}
		}
	}
}


STATIC CONST U8 line_start[] = { 0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 66, 78, 91, 105, 120 };
BOOL TriangleMatInv(FLOAT64* mat, U32 n)
{
	U32 i, j, k;
	FLOAT64 w, g;
  FLOAT64* b = (FLOAT64*)malloc(n*SIZEOF(FLOAT64));
	for (k = 0; k <= n - 1; k++)
	{
		w = mat[0];
		if (fabs(w) + 1.0 == 1.0)
		{
			free(b);
			return FALSE;
		}
		for (i = 1; i <= n - 1; i++)
		{
			g = mat[line_start[i]];
			b[i] = g / w;
			if (i <= n - k - 1)
				b[i] = -b[i];
			for (j = 1; j <= i; j++)
				mat[line_start[i - 1] + j - 1] = mat[line_start[i] + j] + g*b[j];
		}
		mat[line_start[n] - 1] = 1.0 / w;
		for (i = 1; i <= n - 1; i++)
			mat[line_start[n - 1] + i - 1] = b[i];
	}
	free(b);
	return TRUE;
}

U32 GetBitNum(U32* pBits, U8 count)
{
	U32 ret = 0;
	U8 i;
	U32 j;
	for ( i = 0; i < count; i++)
	{
		for (j = 0; j < 32; j++)
		{
			if ((pBits[i] & (0x00000001 << j)) != 0)
				ret++;
		}
	}
	return ret;
}

U32 GetNextSvid(U32* pBits, U32* index)
{
static	U32 i;
	for ( i = *index + 1; i < MAX_SVID; i++)
	{
		if ((pBits[i / 32] & (0x00000001 << (i % 32))) != 0)
		{
			*index = i;
			return i;
		}
	}
	return 0XFFFF;
}
