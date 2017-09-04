#ifndef CONTROLPOSITION_H
#define CONTROLPOSITION_H

#include <QtGui/QVector3D>
#include "Math/hMatrix.h"
#include "Math/dualQuaternion.h"

class ControlPosition
{
public:
	ControlPosition();
	~ControlPosition();
	
	void setCtrlPos(vector<DualQuaternion>);
	void addCtrlPos(DualQuaternion&);
	void addCtrlPos(hMatrix&,CAPTURETYPE);
	void addCtrlPos(QVector3D&, QVector3D&, QVector3D&, CAPTURETYPE);
	void addCtrlPos(QVector3D&,CAPTURETYPE);    // For joint motion capture

	vector<DualQuaternion> getCtrlPosInDualQuat();
	DualQuaternion getPosition(int);
	int getNoOfCtrlPos();
	void clearControlPosition();
private:
	vector<DualQuaternion> ctrlPosInDualQuat;
	int noOfCtrlPos;
};

class LimbPosition
{
public:
	LimbPosition();
	~LimbPosition();

	void setLimbPos(vector<DualQuaternion>);
	void setLimbLength(vector<float>);
	void setStartAndEndOfLimb(const QVector3D&, const QVector3D&,CAPTURETYPE);
	void addLimbPos(const QVector3D&, const QVector3D&,CAPTURETYPE);
	void addLimbPos(const QVector3D&, const QVector3D&, const QVector3D&, CAPTURETYPE);
	void addLimbPos(const DualQuaternion&);
	void addLimbLength(const float&);
	void addLimbLength(const QVector3D&, const QVector3D&,CAPTURETYPE);

	vector<DualQuaternion> getLimbPositions();
	vector<float> getLimbLength();
	int getNoOfLimbPos();
	vector<QVector3D> getstartPoint();
	vector<QVector3D> getendPoint();

	void clearLimbPos();

private:
	vector<DualQuaternion> limbPosInDualQuat;
	vector<QVector3D> start;
	vector<QVector3D> end;
	vector<float> limbLengths;
	int noOfLimbPos;
};


#endif