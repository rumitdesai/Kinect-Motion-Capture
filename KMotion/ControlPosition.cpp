#include <QtCore/QDebug>
#include "ControlPosition.h"

// ************* Class: ControlPosition ********** //
ControlPosition::ControlPosition()
{
	ctrlPosInDualQuat.clear();
}
ControlPosition::~ControlPosition()
{
	ctrlPosInDualQuat.clear();
	noOfCtrlPos = 0;
}
void ControlPosition::setCtrlPos(vector<DualQuaternion> ctrlPosVector)
{
	ctrlPosInDualQuat = ctrlPosVector;
}
void ControlPosition::addCtrlPos(DualQuaternion& pos)
{ 
	ctrlPosInDualQuat.emplace_back(pos); 
}
void ControlPosition::addCtrlPos(hMatrix& pos,CAPTURETYPE type){
	hMatrix temp;
	ctrlPosInDualQuat.emplace_back(temp.homogeneousMatrixToDualQuaternion(pos,type));
}
void ControlPosition::addCtrlPos(QVector3D& hand, QVector3D& handtip, QVector3D& handthumb, CAPTURETYPE type)
{
	hMatrix temp; hMatrix matrixForm;
	matrixForm = temp.homogeneousMatrix(hand, handtip, handthumb, type);
	ctrlPosInDualQuat.emplace_back(temp.homogeneousMatrixToDualQuaternion(matrixForm, type));
}
vector<DualQuaternion> ControlPosition::getCtrlPosInDualQuat()
{
	return ctrlPosInDualQuat;
}
DualQuaternion ControlPosition::getPosition(int idx)
{
	return ctrlPosInDualQuat.at(idx);
}
int ControlPosition::getNoOfCtrlPos()
{
	noOfCtrlPos = ctrlPosInDualQuat.size();
	return noOfCtrlPos;
}
void ControlPosition::clearControlPosition()
{
	ctrlPosInDualQuat.clear();
	noOfCtrlPos = 0;
}
//*************** For sit and stand device *************** //
void ControlPosition::addCtrlPos(QVector3D& joint,CAPTURETYPE type) 
{
	hMatrix temp; hMatrix matrixForm;
	matrixForm = temp.homogeneousMatrixForJoints(joint, type);
	
	/*qDebug() << "Checking Matrix in CONTROLPOSITION";
	qDebug() << matrixForm.m[0][0] << "\t" << matrixForm.m[0][1] << "\t" << matrixForm.m[0][2] << "\t" << matrixForm.m[0][3];
	qDebug() << matrixForm.m[1][0] << "\t" << matrixForm.m[1][1] << "\t" << matrixForm.m[1][2] << "\t" << matrixForm.m[1][3];
	qDebug() << matrixForm.m[2][0] << "\t" << matrixForm.m[2][1] << "\t" << matrixForm.m[2][2] << "\t" << matrixForm.m[2][3];
	qDebug() << matrixForm.m[3][0] << "\t" << matrixForm.m[3][1] << "\t" << matrixForm.m[3][2] << "\t" << matrixForm.m[3][3];*/
	ctrlPosInDualQuat.emplace_back(temp.homogeneousMatrixToDualQuaternion(matrixForm, type));
}

// ************* Class: LimbPosition ********** //

LimbPosition::LimbPosition()
{
	limbPosInDualQuat.clear();
	limbLengths.clear();
	start.clear();
	end.clear();
}
LimbPosition::~LimbPosition()
{
	limbPosInDualQuat.clear();
	noOfLimbPos = 0;
}
void LimbPosition::setLimbPos(vector<DualQuaternion> positions)
{
	limbPosInDualQuat = positions;
}
void LimbPosition::setLimbLength(vector<float> lengths)
{
	limbLengths = lengths;
}
void LimbPosition::setStartAndEndOfLimb(const QVector3D& strt, const QVector3D& en,CAPTURETYPE type)
{
	if (type == SPATIAL)
	{
		start.emplace_back(strt);
		end.emplace_back(en);
	}
	else if (type == PLANAR)
	{
		QVector3D startPoint = strt;
		QVector3D endPoint = en;
		startPoint.setZ(0.0f);
		endPoint.setZ(0.0f);
		start.emplace_back(startPoint);
		end.emplace_back(endPoint);

	}
}
void LimbPosition::addLimbPos(const QVector3D& moving, const QVector3D& target, CAPTURETYPE type)
{
	hMatrix tempMatrix;
	limbPosInDualQuat.emplace_back(tempMatrix.dualQuaternionForLimbs(moving, target, type));
}

void LimbPosition::addLimbPos(const QVector3D& knee, const QVector3D& ankle, const QVector3D& newKnee, CAPTURETYPE type)
{
	//qDebug() << "Entering SECOND TYPE OF ADD LIMB POS:CONTROLPOSITION";
	hMatrix tempMatrix; hMatrix spatialDisplacement;

	spatialDisplacement = tempMatrix.homogeneousMatrixForLimbs(knee, ankle, newKnee,type);

	limbPosInDualQuat.emplace_back(tempMatrix.homogeneousMatrixToDualQuaternion(spatialDisplacement, type));

	//qDebug() << "LEAVING SECOND TYPE OF ADD LIMB POS:CONTROLPOSITION";
}
void LimbPosition::addLimbPos(const DualQuaternion& pos)
{
	limbPosInDualQuat.emplace_back(pos);
}
void LimbPosition::addLimbLength(const float& len)
{
	limbLengths.emplace_back(len);
}
void LimbPosition::addLimbLength(const QVector3D& start, const QVector3D& end,CAPTURETYPE type)
{
	if (type == SPATIAL)
	{
		float len;
		len = (end - start).length();
		limbLengths.emplace_back(len);
	}
	else if (type == PLANAR)
	{
		QVector3D star = start;
		QVector3D en = end;
		star.setZ(0.0f);
		en.setZ(0.0f);

		float len;
		len = (en - star).length();
		limbLengths.emplace_back(len);
	}
}

vector<DualQuaternion> LimbPosition::getLimbPositions()
{
	return limbPosInDualQuat;
}
vector<float> LimbPosition::getLimbLength()
{
	return limbLengths;
}
int LimbPosition::getNoOfLimbPos()
{
	noOfLimbPos = limbPosInDualQuat.size();
	return noOfLimbPos;
}
vector<QVector3D> LimbPosition::getstartPoint()
{
	return start;
}
vector<QVector3D> LimbPosition::getendPoint()
{
	return end;
}
void LimbPosition::clearLimbPos()
{
	limbPosInDualQuat.clear();
	start.clear();
	end.clear();
	limbLengths.clear();
	noOfLimbPos = 0;
}