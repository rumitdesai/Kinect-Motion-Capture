// motion.h defines all operations for a motion class

#ifndef MOTION_H
#define MOTION_H

#include <vector>
//#include "Dependencies/64 bit/Include/GLU/GLU.h"
#include <GLU.h>
#include "Math/DualQuaternion.h"
#include "ControlPosition.h"
#include "motionProperties.h"
#include "Math/hMatrix.h"

class Motion
{
public:
	Motion();
	virtual ~Motion();

	// Motion Type Flags
	static bool PLOTCONTROLPOSITIONS;
	static bool SCREWMOTION;
	static bool BEZIERMOTION;
	static bool RBSPLINEMOTION;
	static bool RBSPLINEINTERPOLATION;
	static bool RCONTINUOUSMOTION;
	static bool LIMBMOTION;

	// Static draw functions
	static void drawWrist(int displayMode);
	static void drawCoordinateFrame(GLUquadricObj*);
	static void drawSelectedCoordinateFrame(GLUquadricObj*);
	static void drawCylinder(GLUquadricObj*, float len);
	
	// General Motion Functions
	void setCtrlPos(vector<DualQuaternion>);
	vector<DualQuaternion> getCtrlPos();
	int getNoOfCtrlPos();
	void plotCtrlPositions(float (&color)[3]);
	virtual void plotMotion() = 0;
	double binomial(int n, int i, double t);
	double binomial(int n, int i);
	int getSpan(int n, int p, float u, vector<double>U);
	vector<float> getBasisFunction(int i, double u, int p, vector<double> U);

	// Limb Motion Functions
	void setLimbPoses(vector<DualQuaternion>, vector<float>);
	void setLimbPoses(vector<DualQuaternion>);
	vector<DualQuaternion> getLimbPoses();
	void plotKeyLimbPoses(float(&color)[3]);
	void plotKeySpinePoses(float(&color)[3]);
	virtual void plotLimbMotion() = 0;
	virtual void plotSpineMotion() = 0;
	vector<float> getLimbLengths();
	int getNoOfLimbPoses();

private:
	ControlPosition* ctrlPositions;	
	LimbPosition* limbPoses;
};

class MNoneMotion :public Motion
{
public:
	MNoneMotion();
	virtual ~MNoneMotion();
	virtual void plotMotion();
	virtual void plotLimbMotion();
	virtual void plotSpineMotion();
};

class MContinuousMotion :public Motion
{
public:
	MContinuousMotion();
	virtual ~MContinuousMotion();
	virtual void plotMotion();
	virtual void plotLimbMotion();
	virtual void plotSpineMotion();
	void plotJointMotion();
	MotionProperties ContinuousProps;
	DualQuaternion getLessDensePosition(int idx);
	vector<DualQuaternion> getAllPositions();
private:
	vector<DualQuaternion> lessDensePositions;
};

class LimbMotion:public Motion
{
public:
	LimbMotion();
	virtual ~LimbMotion();
	
	void drawLineSkeleton(vector<QVector3D> strt,vector<QVector3D> en);
	virtual void plotMotion();
	virtual void plotLimbMotion();
	virtual void plotSpineMotion();
	MotionProperties LimbMotionProps;
	DualQuaternion getLessDenseLimbPose(int idx);
	vector<DualQuaternion> getAllLimbPoses();

private:
	vector<DualQuaternion> lessDenseLimbPoses;
};

class MScrewMotion :public Motion
{
public:
	MScrewMotion();
	virtual ~MScrewMotion();
	virtual void plotMotion();
	virtual void plotLimbMotion();
	virtual void plotSpineMotion();
	MotionProperties ScrewProps;
};

class MBezierMotion :public Motion
{
public:
	MBezierMotion();
	virtual ~MBezierMotion();
	vector<double>calculateBernsteinPoly(double t);
	vector<double>calculateBernsteinPolyForLimbData(double t);
	virtual void plotMotion();
	virtual void plotLimbMotion();
	virtual void plotSpineMotion();
	MotionProperties BezierProps;
};

class MRBSplineMotion :public Motion
{
public:
	MRBSplineMotion();
	virtual ~MRBSplineMotion();
	virtual void plotMotion();
	virtual void plotLimbMotion();
	virtual void plotSpineMotion();
	void setDegree(int);
	int getDegree();
	vector<double> getKnotVector();
	vector<double> getKnotVectorForLimbData();
	MotionProperties BSplineProps;
private:
	int degree;
};

class MRBSplineInterpolation :public MRBSplineMotion
{
public:
	MRBSplineInterpolation();
	virtual ~MRBSplineInterpolation();
	virtual void plotMotion();
	virtual void plotLimbMotion();
	virtual void plotSpineMotion();
	MotionProperties BsplineInterprops;
private:
	vector<DualQuaternion> newCtrlPositions();
	vector<DualQuaternion> newCtrlPositionsForLimbData();
	vector<double> calcUkVec();
	vector<double> calcUkVecforLimbData();
};

#endif
