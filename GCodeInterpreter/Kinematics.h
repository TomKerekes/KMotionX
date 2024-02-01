// Kinematics.h: interface for the CKinematics class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_KINEMATICS_H__F0E3BA96_734F_4D32_85DD_8B2FA813C991__INCLUDED_)
#define AFX_KINEMATICS_H__F0E3BA96_734F_4D32_85DD_8B2FA813C991__INCLUDED_


#ifdef GCODEINTERPRETER_EXPORTS
#define GCODEINTERPRETER_API __declspec(dllexport)
#else
#define GCODEINTERPRETER_API __declspec(dllimport)
#endif


#include "PT2D.h"
#include "PT3D.h"



#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define MAX_ACTUATORS 8




class GCODEINTERPRETER_API CKinematics  
{
public:
	int Solve(double *A, int N);
	int GetParameter(const char *, double * v);
	void removeChar(char * s, int c);
	int MaxAccelInDirection(double dx, double dy, double dz, double da, double db, double dc, double du, double dv, double *accel);
	int MaxRateInDirection(double dx, double dy, double dz, double da, double db, double dc, double du, double dv, double *rate);
	int MaxRateInDirection(double dx, double dy, double dz, double da, double db, double dc, double *rate);
	int MaxRapidRateInDirection(double dx, double dy, double dz, double da, double db, double dc, double du, double dv, double *rate);
	int MaxRapidJerkInDirection(double dx, double dy, double dz, double da, double db, double dc, double du, double dv, double *jerk);
	int MaxRapidAccelInDirection(double dx, double dy, double dz, double da, double db, double dc, double du, double dv, double *accel);
	virtual int TransformCADtoActuators(double x, double y, double z, double a, double b, double c, double u, double v, double *Acts, bool NoGeo = false);
	virtual int TransformCADtoActuators(double x, double y, double z, double a, double b, double c, double *Acts, bool NoGeo = false);
	virtual int TransformActuatorstoCAD(double *Acts, double *x, double *y, double *z, double *a, double *b, double *c, bool NoGeo = false);
	virtual int TransformActuatorstoCAD(double *Acts, double *x, double *y, double *z, double *a, double *b, double *c, double *u, double *v, bool NoGeo = false);
	virtual int ComputeAnglesOption(int is);
	virtual int Initialize();
	int InvertTransformCADtoActuators(double *Acts, double *xr, double *yr, double *zr, double *ar, double *br, double *cr, bool NoGeo = false);
	virtual int RemapForNonStandardAxes(double *x, double *y, double *z, double *a, double *b, double *c);

	int IntersectionTwoCircles(CPT2D c0, double r0, CPT2D c1, double r1, CPT2D *q);

	virtual int ReadGeoTable(const char *name);
	virtual int GeoCorrect(double x, double y, double z, double *cx, double *cy, double *cz);
	virtual int GetSoftLimits(double *xm, double *xp, double *ym, double *yp, double *zm, double *zp,
		double *am, double *ap, double *bm, double *bp, double *cm, double *cp, double *um, double *up, double *vm, double *vp) {
		return 0;
	}
	
	CKinematics();
	virtual ~CKinematics();

	MOTION_PARAMS m_MotionParams;


	bool GeoTableValid;
	CPT3D *GeoTable;
	CString *Table2;
	int NRows,NCols;
	double GeoSpacingX,GeoSpacingY;
	double GeoOffsetX,GeoOffsetY;  // Machine coordinates of grid point row=0 col=0

	const char *MainPath;
};

#endif // !defined(AFX_KINEMATICS_H__F0E3BA96_734F_4D32_85DD_8B2FA813C991__INCLUDED_)
