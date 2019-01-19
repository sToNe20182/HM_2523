#ifndef __CONST_H__
#define __CONST_H__

//Commonly Used Parameters
#define PI				(3.1415926535897932)
#define DEG2RAD			(PI/180.0)
#define RAD2DEG			(180.0/PI)
//Earth Parameters
#define GRAVITY_CONST	9.7803267714
#define WGS_AXIS_A		6378137.0				// A - WGS-84 earth's semi major axis
#define	WGS_AXIS_B		6356752.3142451795		// B - WGS-84 earth's semi minor axis
#define WGS_E1_SQR		0.006694379990141317	// (A/B)^2-1, 1st numerical eccentricity
#define WGS_E2_SQR		0.006739496742276435	// 1-(B/A)^2, 2nd numerical eccentricity
#define WGS_SQRT_GM		19964981.8432173887		// square root of GM
#define WGS_OMEGDOTE	7.2921151467e-5			// earth rotate rate
#define KMPH			    1000.0/60.0/60.0       // 1公里每小时
#endif
