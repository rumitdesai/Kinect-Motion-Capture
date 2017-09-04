#include <QtCore/QDebug>
//#include "Dependencies\64 bit\Include\GLU\GLU.h"
//#include <GL\GLU.h>
#include <GLU.h>
#include <QtWidgets/QWidget>
#include <QtGui/QMouseEvent>
#include "maindialog.h"
#include "motion.h"

#include "glWidget.h"
#include "UI_color.h"


extern Kinect::mainDialog* mainWinPtr;

GLWidget::GLWidget(QWidget* parent) :QOpenGLWidget(parent), m_xRotate(0.0), m_yRotate(0.0), m_zRotate(0.0), m_xTrans(0.0), m_yTrans(0.0), m_zTrans(0.0)
{
}

void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	
	ContinuousMotion = NULL;

	RScrewMotion = NULL;
	RScrewFemurMotion = NULL;
	RScrewTibiaMotion = NULL;
	RScrewSpineMotion = NULL;
	RScrewRightFemurMotion = NULL;
	RScrewRightTibiaMotion = NULL;

	RBezierMotion = NULL;
	RBezierFemurMotion = NULL;
	RBezierTibiaMotion = NULL;
	RBezierSpineMotion = NULL;
	RBezierRightFemurMotion = NULL;
	RBezierRightTibiaMotion = NULL;

	RBSplineMotion = NULL;
	RBSplineFemurMotion = NULL;
	RBSplineTibiaMotion = NULL;
	RBSPlineSpineMotion = NULL;
	RBSplineRightFemurMotion = NULL;
	RBSplineRightTibiaMotion = NULL;

	RBSplineInterpolation = NULL;
	RBSplineInterpolationFemurMotion = NULL;
	RBSplineInterpolationTibiaMotion = NULL;
	RBSplineInterpolationLeftUpperArmMotion = NULL;
	RBSplineInterpolationLeftLowerArmMotion = NULL;
	RBSplineInterpolationSpineMotion = NULL;
	RBSplineInterpolationRightFemurMotion = NULL;
	RBSplineInterpolationRightTibiaMotion = NULL;

	RBSplineLeftHipInterpolation = NULL; 
	RBSplineLeftShoulderInterpolation = NULL;
	RBSplineLeftAnkleInterpolation = NULL;
	RBSplineRightAnkleInterpolation = NULL;
	RBSplineLeftElbowInterpolation = NULL;
	//RBSplineRightElbowInterpolation = NULL;

	MFemurMotion = NULL;
	MTibiaMotion = NULL;
	MLeftUpperArmMotion = NULL;
	MLeftLowerArmMotion = NULL;
	MSpineMotion = NULL;
	MRightFemurMotion = NULL;
	MRightTibiaMotion = NULL;

	ContinuousLeftHipMotion = NULL;
	ContinuousLeftShoulderMotion = NULL;
	ContinuousLeftAnkleMotion = NULL;
	ContinuousRightAnkleMotion = NULL;
	ContinuousLeftElbowMotion = NULL;
	//ContinuousRightElbowMotion = NULL;

	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	setupLight();
}

void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!mainWinPtr->kinectSensor->getKeyPositions().empty() && mainWinPtr->bsplineApproximation->isChecked())
		mainWinPtr->approximationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyFemurPositions().empty() && mainWinPtr->bsplineApproximation->isChecked())
		mainWinPtr->approximationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyFemurPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyFemurPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyFemurPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyTibiaPositions().empty() && mainWinPtr->bsplineApproximation->isChecked())
		mainWinPtr->approximationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyTibiaPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyTibiaPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyTibiaPositions().size() - 1);
	
	if (!mainWinPtr->kinectSensor->getKeySpinePositions().empty() && mainWinPtr->bsplineApproximation->isChecked())
		mainWinPtr->approximationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeySpinePositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeySpinePositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeySpinePositions().size() - 1);
	
	if (!mainWinPtr->kinectSensor->getKeyRightFemurPositions().empty() && mainWinPtr->bsplineApproximation->isChecked())
		mainWinPtr->approximationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyRightFemurPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyRightFemurPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyRightFemurPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyRightTibiaPositions().empty() && mainWinPtr->bsplineApproximation->isChecked())
		mainWinPtr->approximationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyRightTibiaPositions().size() - 1);

	if (!mainWinPtr->kinectSensor->getKeyRightTibiaPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyRightTibiaPositions().size() - 1);
	
	if (!mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions().size() - 1);
	
	if (!mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions().size() - 1);
	/*if (!mainWinPtr->kinectSensor->getShoulderPositions().empty() && mainWinPtr->bsplineApproximation->isChecked())
		mainWinPtr->approximationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getShoulderPositions().size() - 1);

		if (!mainWinPtr->kinectSensor->getShoulderPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getShoulderPositions().size() - 1);*/
	// ************************************** Joint Capture settings *********************************** //
	if (!mainWinPtr->kinectSensor->getKeyLeftElbowPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyLeftElbowPositions().size() - 1);
	if (!mainWinPtr->kinectSensor->getKeyLeftHipPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyLeftHipPositions().size() - 1);
	if (!mainWinPtr->kinectSensor->getKeyLeftShoulderPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyLeftShoulderPositions().size() - 1);
	if (!mainWinPtr->kinectSensor->getKeyRightAnklePositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyRightAnklePositions().size() - 1);
	if (!mainWinPtr->kinectSensor->getKeyLeftAnklePositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyLeftAnklePositions().size() - 1);
	/*if (!mainWinPtr->kinectSensor->getKeyRightElbowPositions().empty() && mainWinPtr->bsplineInterpolation->isChecked())
		mainWinPtr->interpolationDegreeSpinBox->setRange(0, mainWinPtr->kinectSensor->getKeyRightElbowPositions().size() - 1);*/


	if (!mainWinPtr->kinectSensor->getContinuousPositions().empty() || !mainWinPtr->kinectSensor->getFemurPosesInSensor().empty() || !mainWinPtr->kinectSensor->getTibiaPosesInSensor().empty() || !mainWinPtr->kinectSensor->getSpinePosesInSensor().empty() || !mainWinPtr->kinectSensor->getRightFemurPosesInSensor().empty() || !mainWinPtr->kinectSensor->getRightTibiaPosesInSensor().empty())
		mainWinPtr->saveContinuousPositions->setEnabled(true);
	else
		mainWinPtr->saveContinuousPositions->setEnabled(false);

	if (!mainWinPtr->kinectSensor->getKeyPositions().empty() || !mainWinPtr->kinectSensor->getKeyFemurPositions().empty() || !mainWinPtr->kinectSensor->getKeyTibiaPositions().empty() || !mainWinPtr->kinectSensor->getKeySpinePositions().empty() || !mainWinPtr->kinectSensor->getKeyRightFemurPositions().empty() || !mainWinPtr->kinectSensor->getKeyRightTibiaPositions().empty() || !mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions().empty() || !mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions().empty()
		|| !mainWinPtr->kinectSensor->getKeyLeftHipPositions().empty() || !mainWinPtr->kinectSensor->getKeyLeftShoulderPositions().empty() || !mainWinPtr->kinectSensor->getKeyLeftAnklePositions().empty() || !mainWinPtr->kinectSensor->getKeyRightAnklePositions().empty() || !mainWinPtr->kinectSensor->getKeyLeftElbowPositions().empty() /*|| !mainWinPtr->kinectSensor->getKeyRightElbowPositions().empty()*/){
		mainWinPtr->saveKeyPositions->setEnabled(true);
		mainWinPtr->keyPositions->setEnabled(true);
	}
	else
	{
		mainWinPtr->saveKeyPositions->setEnabled(false);
		mainWinPtr->keyPositions->setEnabled(false);
	}
	
	if (!mainWinPtr->kinectSensor->getContinuousPositions().empty() || !mainWinPtr->kinectSensor->getContinuousLeftHipPositions().empty() || !mainWinPtr->kinectSensor->getContinuousLeftShoulderPositions().empty() || !mainWinPtr->kinectSensor->getContinuousLeftAnklePositions().empty() || !mainWinPtr->kinectSensor->getContinuousRightAnklePositions().empty() || !mainWinPtr->kinectSensor->getContinuousLeftElbowPositions().empty() /*|| !mainWinPtr->kinectSensor->getContinuousRightElbowPositions().empty()*/)
	{
		mainWinPtr->continuousMotionCheckBox->setEnabled(true);
	}
	else
	{
		mainWinPtr->continuousMotionCheckBox->setEnabled(false);
	}

	if (!mainWinPtr->kinectSensor->getFemurPosesInSensor().empty() || !mainWinPtr->kinectSensor->getTibiaPosesInSensor().empty() || !mainWinPtr->kinectSensor->getSpinePosesInSensor().empty() || !mainWinPtr->kinectSensor->getRightFemurPosesInSensor().empty() || !mainWinPtr->kinectSensor->getRightTibiaPosesInSensor().empty() || !mainWinPtr->kinectSensor->getLeftUpperArmPosesInSensor().empty() || !mainWinPtr->kinectSensor->getLeftLowerArmPosesInSensor().empty())
	{
		mainWinPtr->limbMotionCheckBox->setEnabled(true);
	}
	else
	{
		mainWinPtr->limbMotionCheckBox->setEnabled(false);
	}

	if (mainWinPtr->GLBackground->isChecked())
	{
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
	}
	else
	{
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	
	if (mainWinPtr->planarCapture->isChecked())
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		//glOrtho(-16.0000, 16.0000, -(16.0000 / aspectRatio), (16.0000 / aspectRatio), -1000, 1000.0000);
		gluPerspective(60.0f, GLfloat(m_widgetWidth) / GLfloat(m_widgetHeight), 0.01, 1000.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glTranslatef(0.0f, 0.0f, -20.0f);   // -20

		glTranslatef(0.0f, 0.0f, -40.0f);

	}
	else if(mainWinPtr->spatialCapture->isChecked())                              
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(60.0f, GLfloat(m_widgetWidth) / (GLfloat)m_widgetHeight, 0.01, 1000.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glTranslatef(0.0f, 0.0f, -40.0f);
	}

	// Debugging Scripts
	/*glPushMatrix();
	GLUquadricObj *coordinateobj3;
	coordinateobj3 = gluNewQuadric();
	gluQuadricNormals(coordinateobj3, GLU_SMOOTH);
	gluQuadricDrawStyle(coordinateobj3, GLU_FILL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
	glPushMatrix();
	glTranslated(m_xTrans, m_yTrans, m_zTrans);
	glRotated(m_xRotate, 1.0, 0.0, 0.0);
	glRotated(m_yRotate, 0.0, 1.0, 0.0);
	glRotated(m_zRotate, 0.0, 0.0, 1.0);
	Motion::drawCoordinateFrame(coordinateobj3);
	glPopMatrix();
	glPopMatrix();*/

	// ***************************************************************** //
	/*QVector3D hip1(-0.282677, -0.238305, 2.43982);
	QVector3D knee1(0.0846661, -0.145727, 2.38809);

	float LimbLength = (knee1 - hip1).length();
	//QVector3D hip2(-0.283883, -0.248042, 2.4211);
	//QVector3D knee2(0.080374, -0.14279, 2.38644);
	glPushMatrix();
	glTranslated(m_xTrans, m_yTrans, m_zTrans);
	glRotated(m_xRotate, 1.0, 0.0, 0.0);
	glRotated(m_yRotate, 0.0, 1.0, 0.0);
	glRotated(m_zRotate, 0.0, 0.0, 1.0);
	glColor3f(1.0f, 1.0f, 0.0f);
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glVertex3f(hip1.x()*10.0f, hip1.y()*10.0f, hip1.z()*(10.0f));
	glVertex3f(knee1.x()*10.0f, knee1.y()*10.0f, knee1.z()*(10.0f));
	glEnd();
	glPopMatrix();
	
	hMatrix tempMatrix;
	hMatrix transformationMatrix = tempMatrix.homogeneousMatrixForLimbs(hip1, knee1);
	
	double MatrixForOpenGL[16];
	MatrixForOpenGL[0] = transformationMatrix.m[0][0]; MatrixForOpenGL[4] = transformationMatrix.m[0][1]; MatrixForOpenGL[8] = transformationMatrix.m[0][2]; MatrixForOpenGL[12] = transformationMatrix.m[0][3]*10.0f;
	MatrixForOpenGL[1] = transformationMatrix.m[1][0]; MatrixForOpenGL[5] = transformationMatrix.m[1][1]; MatrixForOpenGL[9] = transformationMatrix.m[1][2]; MatrixForOpenGL[13] = transformationMatrix.m[1][3]*10.0f;
	MatrixForOpenGL[2] = transformationMatrix.m[2][0]; MatrixForOpenGL[6] = transformationMatrix.m[2][1]; MatrixForOpenGL[10] = transformationMatrix.m[2][2]; MatrixForOpenGL[14] = transformationMatrix.m[2][3]*10.0f;
	MatrixForOpenGL[3] = 0.0f; MatrixForOpenGL[7] = 0.0f; MatrixForOpenGL[11] = 0.0f; MatrixForOpenGL[15] = 1.0f;

	glPushMatrix();
	GLUquadricObj *coordinateobj2;
	coordinateobj2 = gluNewQuadric();
	gluQuadricNormals(coordinateobj2, GLU_SMOOTH);
	gluQuadricDrawStyle(coordinateobj2, GLU_FILL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
	glPushMatrix();
	glTranslated(m_xTrans, m_yTrans, m_zTrans);
	glRotated(m_xRotate, 1.0, 0.0, 0.0);
	glRotated(m_yRotate, 0.0, 1.0, 0.0);
	glRotated(m_zRotate, 0.0, 0.0, 1.0);

	glMultMatrixd(MatrixForOpenGL);
	Motion::drawCylinder(coordinateobj2, LimbLength);
	glPopMatrix();
	glPopMatrix();*/

	// ***************************************************************************//
	// CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC//
	/*//QVector3D hip1(0.2, 0.2, 0.0);
	QVector3D hip2(0.4,0.2,0.0);
	//QVector3D knee1(0.2, -0.2, 0.0);
	QVector3D knee2(0.6, 0.6, 0.0);
	
	

	//QVector3D originalPos(0.0f, -1.0f, 0.0f);
	//QVector3D originalPos = knee1 - hip1;
	//QVector3D normalizedOriginalPos = (knee1 - hip1).normalized();
	QVector3D normalizedOriginalPos(1.0f, 0.0f, 0.0f);
	QVector3D finalPos = knee2 - hip2;
	QVector3D normalizedFinalPos = (knee2 - hip2).normalized();

	//float originalLimbLength = (knee1 - hip1).length();
	float originalLimbLength = 0.4;
	float finalLimbLength = (knee2 - hip2).length();

	//qDebug() << "Chekcing originalPos" << originalPos;
	
	qDebug() << "NORMALIZED ORIGINAL POSITION" << normalizedOriginalPos;
	qDebug() << "NORMALIZED FINAL POSITION" << normalizedFinalPos;

	QVector3D tempVector;
	QVector3D rotAxis = (tempVector.crossProduct(normalizedOriginalPos, normalizedFinalPos)).normalized();
	float angleOfRotation = acos(tempVector.dotProduct(normalizedOriginalPos, normalizedFinalPos));

	hMatrix tempMatrix;
	Quaternion quat = tempMatrix.QuaternionFromAxisAngle(rotAxis, angleOfRotation);
	Matrix rotMat = quat.quaternionToRotationMatrix();

	hMatrix transformationMatrix;
	transformationMatrix.m[0][0] = rotMat.m[0][0]; transformationMatrix.m[0][1] = rotMat.m[0][1]; transformationMatrix.m[0][2] = rotMat.m[0][2]; transformationMatrix.m[0][3] = hip2.x()*10.0f;
	transformationMatrix.m[1][0] = rotMat.m[1][0]; transformationMatrix.m[1][1] = rotMat.m[1][1]; transformationMatrix.m[1][2] = rotMat.m[1][2]; transformationMatrix.m[1][3] = hip2.y()*10.0f;
	transformationMatrix.m[2][0] = rotMat.m[2][0]; transformationMatrix.m[2][1] = rotMat.m[2][1]; transformationMatrix.m[2][2] = rotMat.m[2][2]; transformationMatrix.m[2][3] = hip2.z()*10.0f;
	transformationMatrix.m[3][0] = 0.0f; transformationMatrix.m[3][1] = 0.0f; transformationMatrix.m[3][2] = 0.0f; transformationMatrix.m[3][3] = 1.0f;

	qDebug() << "Checking TRANSFORMATION MATRIX";
	qDebug() << transformationMatrix.m[0][0] << transformationMatrix.m[0][1] << transformationMatrix.m[0][2] << transformationMatrix.m[0][3];
	qDebug() << transformationMatrix.m[1][0] << transformationMatrix.m[1][1] << transformationMatrix.m[1][2] << transformationMatrix.m[1][3];
	qDebug() << transformationMatrix.m[2][0] << transformationMatrix.m[2][1] << transformationMatrix.m[2][2] << transformationMatrix.m[2][3];
	qDebug() << transformationMatrix.m[3][0] << transformationMatrix.m[3][1] << transformationMatrix.m[3][2] << transformationMatrix.m[3][3];

	double MatrixForOpenGL[16];
	MatrixForOpenGL[0] = rotMat.m[0][0]; MatrixForOpenGL[4] = rotMat.m[0][1]; MatrixForOpenGL[8] = rotMat.m[0][2]; MatrixForOpenGL[12] = hip2.x()*10.0f;
	MatrixForOpenGL[1] = rotMat.m[1][0]; MatrixForOpenGL[5] = rotMat.m[1][1]; MatrixForOpenGL[9] = rotMat.m[1][2]; MatrixForOpenGL[13] = hip2.y()*10.0f;
	MatrixForOpenGL[2] = rotMat.m[2][0]; MatrixForOpenGL[6] = rotMat.m[2][1]; MatrixForOpenGL[10] = rotMat.m[2][2]; MatrixForOpenGL[14] = hip2.z()*10.0f;
	MatrixForOpenGL[3] = 0.0f; MatrixForOpenGL[7] = 0.0f; MatrixForOpenGL[11] = 0.0f; MatrixForOpenGL[15] = 1.0f;

	qDebug() << "Checking Axis of rotation" << rotAxis;
	qDebug() << "Checking Angle of rotation" << (angleOfRotation*180.0)/PI;

	qDebug() << "Original Length of Limb" << originalLimbLength;
	qDebug() << "Final Length of Limb" << finalLimbLength;

	qDebug() << "Checking MATRIX IN OPENGL";
	qDebug() << MatrixForOpenGL[0] << MatrixForOpenGL[4] << MatrixForOpenGL[8] << MatrixForOpenGL[12];
	qDebug() << MatrixForOpenGL[1] << MatrixForOpenGL[5] << MatrixForOpenGL[9] << MatrixForOpenGL[13];
	qDebug() << MatrixForOpenGL[2] << MatrixForOpenGL[6] << MatrixForOpenGL[10] << MatrixForOpenGL[14];
	qDebug() << MatrixForOpenGL[3] << MatrixForOpenGL[7] << MatrixForOpenGL[11] << MatrixForOpenGL[15];

	qDebug() << "\n";

	glPushMatrix();
	glTranslated(m_xTrans, m_yTrans, m_zTrans);
	glRotated(m_xRotate, 1.0, 0.0, 0.0);
	glRotated(m_yRotate, 0.0, 1.0, 0.0);
	glRotated(m_zRotate, 0.0, 0.0, 1.0);
	glColor3f(1.0f, 1.0f, 0.0f);
	glLineWidth(2.0f);
	//glBegin(GL_LINES);
	//glVertex3f(hip1.x()*10.0f, hip1.y()*10.0f, hip1.z()*(10.0f));
	//glVertex3f(knee1.x()*10.0f, knee1.y()*10.0f, knee1.z()*10.0f);
	//glEnd();

	glBegin(GL_LINES);
	glVertex3f(hip2.x()*10.0f, hip2.y()*10.0f, hip2.z()*10.0f);
	glVertex3f(knee2.x()*10.0f, knee2.y()*10.0f, knee2.z()*10.0f);
	glEnd();
	glPopMatrix();

	glPushMatrix();
	GLUquadricObj *coordinateobj;
	coordinateobj = gluNewQuadric();
	gluQuadricNormals(coordinateobj, GLU_SMOOTH);
	gluQuadricDrawStyle(coordinateobj, GLU_FILL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
	glPushMatrix();
	glTranslated(m_xTrans, m_yTrans, m_zTrans);
	glRotated(m_xRotate, 1.0, 0.0, 0.0);
	glRotated(m_yRotate, 0.0, 1.0, 0.0);
	glRotated(m_zRotate, 0.0, 0.0, 1.0);

	//glPushMatrix();
	//glTranslated(hip1.x()*10.0f, hip1.y()*10.0f, hip1.z()*10.0f);
	//glRotated(90.0f, 1.0f, 0.0f, 0.0f);
	Motion::drawCylinder(coordinateobj, originalLimbLength);
	glPopMatrix();
	glPopMatrix();
	//glPopMatrix();

	glPushMatrix();
	GLUquadricObj *coordinateobj2;
	coordinateobj2 = gluNewQuadric();
	gluQuadricNormals(coordinateobj2, GLU_SMOOTH);
	gluQuadricDrawStyle(coordinateobj2, GLU_FILL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
	glPushMatrix();
	glTranslated(m_xTrans, m_yTrans, m_zTrans);
	glRotated(m_xRotate, 1.0, 0.0, 0.0);
	glRotated(m_yRotate, 0.0, 1.0, 0.0);
	glRotated(m_zRotate, 0.0, 0.0, 1.0);

	
	glMultMatrixd(MatrixForOpenGL);
	Motion::drawCylinder(coordinateobj2, finalLimbLength);
	glPopMatrix();
	glPopMatrix();*/
	// CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC//

	if (mainWinPtr->keyPositions->isChecked())
	{

		if (mainWinPtr->LimbMotionCapture->isChecked())
		{
			MNoneMotion* keyFemurPoses = new MNoneMotion;
			MNoneMotion*  keyTibiaPoses = new MNoneMotion;
			MNoneMotion* keyLeftUpperArm = new MNoneMotion;
			MNoneMotion* keyLeftLowerArm = new MNoneMotion;
			MNoneMotion* keySpinePoses = new MNoneMotion;
			MNoneMotion* keyRightFemurPoses = new MNoneMotion;
			MNoneMotion* keyRightTibiaPoses = new MNoneMotion;

			keyFemurPoses->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			keyTibiaPoses->setLimbPoses(mainWinPtr->kinectSensor->getKeyTibiaPositions());
			keyLeftUpperArm->setLimbPoses(mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions());
			keyLeftLowerArm->setLimbPoses(mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions());
			keySpinePoses->setLimbPoses(mainWinPtr->kinectSensor->getKeySpinePositions());
			keyRightFemurPoses->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightFemurPositions());
			keyRightTibiaPoses->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightTibiaPositions());

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			keyFemurPoses->plotKeyLimbPoses(BROWN);
			keyTibiaPoses->plotKeyLimbPoses(GREY);
			keyLeftUpperArm->plotKeyLimbPoses(GREY);
			keyLeftLowerArm->plotKeyLimbPoses(GREY);
			keySpinePoses->plotKeySpinePoses(PLUM);
			keyRightFemurPoses->plotKeyLimbPoses(BROWN);
			keyRightTibiaPoses->plotKeyLimbPoses(GREY);


			glPopMatrix();

			delete keyFemurPoses;
			keyFemurPoses = NULL;
			delete keyTibiaPoses;
			keyTibiaPoses = NULL;
			delete keyLeftUpperArm;
			keyLeftUpperArm = NULL;
			delete keyLeftLowerArm;
			keyLeftLowerArm = NULL;
			delete keySpinePoses;
			keySpinePoses = NULL;
			delete keyRightFemurPoses;
			keyRightFemurPoses = NULL;
			delete keyRightTibiaPoses;
			keyRightTibiaPoses = NULL;
		}
		else 
		{
			MNoneMotion* noneMotion = new MNoneMotion;
			MNoneMotion* noneHipMotion = new MNoneMotion;
			MNoneMotion* noneLeftShoulderMotion = new MNoneMotion;
			MNoneMotion* noneLeftAnkleMotion = new MNoneMotion;
			MNoneMotion* noneRightAnkleMotion = new MNoneMotion;
			MNoneMotion* noneLeftElbowMotion = new MNoneMotion;
			//MNoneMotion* noneRightElbowMotion = new MNoneMotion;

			noneMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyPositions());
			noneHipMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftHipPositions());
			noneLeftShoulderMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftShoulderPositions());
			noneLeftAnkleMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftAnklePositions());
			noneRightAnkleMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyRightAnklePositions());
			noneLeftElbowMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftElbowPositions());
			//noneRightElbowMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyRightElbowPositions());

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);

			noneMotion->plotCtrlPositions(BROWN);
			noneHipMotion->plotCtrlPositions(YELLOW);
			noneLeftShoulderMotion->plotCtrlPositions(RED);
			noneLeftAnkleMotion->plotCtrlPositions(SALMON);
			noneRightAnkleMotion->plotCtrlPositions(SLATE_BLUE);
			noneLeftElbowMotion->plotCtrlPositions(SPRING_GREEN);
			//noneRightElbowMotion->plotCtrlPositions(YELLOW);

			glPopMatrix();
			delete noneMotion;
			noneMotion = NULL;
			delete noneHipMotion;
			noneHipMotion = NULL;
			delete noneLeftShoulderMotion;
			noneLeftShoulderMotion = NULL;
			delete noneLeftAnkleMotion;
			noneLeftAnkleMotion = NULL;
			delete noneRightAnkleMotion;
			noneRightAnkleMotion = NULL;
			delete noneLeftElbowMotion;
			noneLeftElbowMotion = NULL;
			//delete noneRightElbowMotion;
			//noneRightElbowMotion = NULL;
		}

		MNoneMotion* noneMotion = new MNoneMotion;
		noneMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyPositions());
		glPushMatrix();
		glTranslated(m_xTrans, m_yTrans, m_zTrans);
		glRotated(m_xRotate, 1.0, 0.0, 0.0);
		glRotated(m_yRotate, 0.0, 1.0, 0.0);
		glRotated(m_zRotate, 0.0, 0.0, 1.0);
		noneMotion->plotCtrlPositions(LIGHT_RED);
		glPopMatrix();
		delete noneMotion;
		noneMotion = NULL;
		//emit emitSizeOfpositions("Key Positions", mainWinPtr->kinectSensor->getKeyPositions().size());

	}

	

	if (Motion::SCREWMOTION == true)
	{
		if (mainWinPtr->LimbMotionCapture->isChecked())
		{
			RScrewFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RScrewFemurMotion->ScrewProps.setObjectColor(GREEN);
			RScrewFemurMotion->ScrewProps.setTrajectoryColor(LIGHT_BLUE);
			RScrewFemurMotion->ScrewProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			
			RScrewTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyTibiaPositions());
			RScrewTibiaMotion->ScrewProps.setObjectColor(RED);
			RScrewTibiaMotion->ScrewProps.setTrajectoryColor(YELLOW);
			RScrewTibiaMotion->ScrewProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			
			RScrewSpineMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeySpinePositions());
			RScrewSpineMotion->ScrewProps.setObjectColor(LIGHT_RED);
			RScrewSpineMotion->ScrewProps.setTrajectoryColor(YELLOW_GREEN);
			RScrewSpineMotion->ScrewProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

			RScrewRightFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightFemurPositions());
			RScrewRightFemurMotion->ScrewProps.setObjectColor(MANDARIN_ORANGE);
			RScrewRightFemurMotion->ScrewProps.setTrajectoryColor(LIGHT_WOOD);
			RScrewRightFemurMotion->ScrewProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			
			RScrewRightTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightTibiaPositions());
			RScrewRightTibiaMotion->ScrewProps.setObjectColor(SPICY_PINK);
			RScrewRightTibiaMotion->ScrewProps.setTrajectoryColor(FLESH);
			RScrewRightTibiaMotion->ScrewProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RScrewFemurMotion->plotLimbMotion();
			RScrewTibiaMotion->plotLimbMotion();
			RScrewSpineMotion->plotSpineMotion();
			RScrewRightFemurMotion->plotLimbMotion();
			RScrewRightTibiaMotion->plotLimbMotion();
			glPopMatrix();

			/*RScrewMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RScrewMotion->ScrewProps.setObjectColor(GREEN);
			RScrewMotion->ScrewProps.setTrajectoryColor(LIGHT_BLUE);
			RScrewMotion->ScrewProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RScrewMotion->plotLimbMotion();
			glPopMatrix();*/
		}
		else
		{
			RScrewMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyPositions());
			//RScrewMotion->setCtrlPos(mainWinPtr->kinectSensor->getShoulderPositions());
			RScrewMotion->ScrewProps.setObjectColor(GREEN);
			RScrewMotion->ScrewProps.setTrajectoryColor(LIGHT_BLUE);
			RScrewMotion->ScrewProps.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RScrewMotion->ScrewProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			//if (mainWinPtr->keyPositions->isChecked())
			//{
				//RScrewMotion->plotCtrlPositions(BROWN);
			//}
			RScrewMotion->plotMotion();
			glPopMatrix();
			//emit emitSizeOfpositions("Screw Motion", mainWinPtr->kinectSensor->getKeyPositions().size());
		}
	}

	if (Motion::BEZIERMOTION == true)
	{
		if (mainWinPtr->LimbMotionCapture->isChecked())
		{
			RBezierFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RBezierFemurMotion->BezierProps.setObjectColor(BLUE);
			RBezierFemurMotion->BezierProps.setTrajectoryColor(RED);
			RBezierFemurMotion->BezierProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			
			RBezierTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyTibiaPositions());
			RBezierTibiaMotion->BezierProps.setObjectColor(SALMON);
			RBezierTibiaMotion->BezierProps.setTrajectoryColor(SEA_GREEN);
			RBezierTibiaMotion->BezierProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			
			RBezierSpineMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeySpinePositions());
			RBezierSpineMotion->BezierProps.setObjectColor(TAN);
			RBezierSpineMotion->BezierProps.setTrajectoryColor(THISTLE);
			RBezierSpineMotion->BezierProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

			RBezierRightFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightFemurPositions());
			RBezierRightFemurMotion->BezierProps.setObjectColor(WHEAT);
			RBezierRightFemurMotion->BezierProps.setTrajectoryColor(YELLOW_GREEN);
			RBezierRightFemurMotion->BezierProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

			RBezierRightTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightTibiaPositions());
			RBezierRightTibiaMotion->BezierProps.setObjectColor(NEW_MIDNIGHT_BLUE);
			RBezierRightTibiaMotion->BezierProps.setTrajectoryColor(GREEN_COPPER);
			RBezierRightTibiaMotion->BezierProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RBezierFemurMotion->plotLimbMotion();
			RBezierTibiaMotion->plotLimbMotion();
			RBezierSpineMotion->plotSpineMotion();
			RBezierRightFemurMotion->plotLimbMotion();
			RBezierRightTibiaMotion->plotLimbMotion();
			glPopMatrix();

			/*RBezierMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RBezierMotion->BezierProps.setObjectColor(BLUE);
			RBezierMotion->BezierProps.setTrajectoryColor(RED);
			RBezierMotion->BezierProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RBezierMotion->plotLimbMotion();
			glPopMatrix();*/
		}
		else
		{
			RBezierMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyPositions());
			//RBezierMotion->setCtrlPos(mainWinPtr->kinectSensor->getShoulderPositions());
			RBezierMotion->BezierProps.setObjectColor(BLUE);
			RBezierMotion->BezierProps.setTrajectoryColor(RED);
			RBezierMotion->BezierProps.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBezierMotion->BezierProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			//if (mainWinPtr->keyPositions->isChecked())
			//{
				//RBezierMotion->plotCtrlPositions(BROWN);
			//}
			RBezierMotion->plotMotion();
			glPopMatrix();
			//emit emitSizeOfpositions("Bezier Motion", mainWinPtr->kinectSensor->getKeyPositions().size());
		}
	}

	if (Motion::RBSPLINEMOTION == true)
	{
		if (mainWinPtr->LimbMotionCapture->isChecked())
		{
			RBSplineFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RBSplineFemurMotion->BSplineProps.setObjectColor(YELLOW);
			RBSplineFemurMotion->BSplineProps.setTrajectoryColor(GREY);
			RBSplineFemurMotion->BSplineProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->approximationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyFemurPositions().size())
			{
				RBSplineFemurMotion->setDegree(mainWinPtr->approximationDegreeSpinBox->value());
			}


			RBSplineTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyTibiaPositions());
			RBSplineTibiaMotion->BSplineProps.setObjectColor(WHEAT);
			RBSplineTibiaMotion->BSplineProps.setTrajectoryColor(SUMMER_SKY);
			RBSplineTibiaMotion->BSplineProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->approximationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyTibiaPositions().size())
			{
				RBSplineTibiaMotion->setDegree(mainWinPtr->approximationDegreeSpinBox->value());
			}

			RBSPlineSpineMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeySpinePositions());
			RBSPlineSpineMotion->BSplineProps.setObjectColor(RICH_BLUE);
			RBSPlineSpineMotion->BSplineProps.setTrajectoryColor(COPPER);
			RBSPlineSpineMotion->BSplineProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->approximationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeySpinePositions().size())
			{
				RBSPlineSpineMotion->setDegree(mainWinPtr->approximationDegreeSpinBox->value());
			}

			RBSplineRightFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightFemurPositions());
			RBSplineRightFemurMotion->BSplineProps.setObjectColor(MANDARIN_ORANGE);
			RBSplineRightFemurMotion->BSplineProps.setTrajectoryColor(LIGHT_WOOD);
			RBSplineRightFemurMotion->BSplineProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->approximationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyRightFemurPositions().size())
			{
				RBSplineRightFemurMotion->setDegree(mainWinPtr->approximationDegreeSpinBox->value());
			}

			RBSplineRightTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightTibiaPositions());
			RBSplineRightTibiaMotion->BSplineProps.setObjectColor(FOREST_GREEN);
			RBSplineRightTibiaMotion->BSplineProps.setTrajectoryColor(FIRE_BRICK);
			RBSplineRightTibiaMotion->BSplineProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->approximationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyRightTibiaPositions().size())
			{
				RBSplineRightTibiaMotion->setDegree(mainWinPtr->approximationDegreeSpinBox->value());
			}

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RBSplineFemurMotion->plotLimbMotion();
			RBSplineTibiaMotion->plotLimbMotion();
			RBSPlineSpineMotion->plotSpineMotion();
			RBSplineRightFemurMotion->plotLimbMotion();
			RBSplineRightTibiaMotion->plotLimbMotion();
			glPopMatrix();


			/*RBSplineMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RBSplineMotion->BSplineProps.setObjectColor(YELLOW);
			RBSplineMotion->BSplineProps.setTrajectoryColor(GREY);
			RBSplineMotion->BSplineProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->approximationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyPositions().size())
			{
				RBSplineMotion->setDegree(mainWinPtr->approximationDegreeSpinBox->value());
			}
			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RBSplineMotion->plotLimbMotion();
			glPopMatrix();*/
		}
		else
		{
			RBSplineMotion->setCtrlPos(mainWinPtr->kinectSensor->getKeyPositions());
			//RBSplineMotion->setCtrlPos(mainWinPtr->kinectSensor->getShoulderPositions());
			RBSplineMotion->BSplineProps.setObjectColor(YELLOW);
			RBSplineMotion->BSplineProps.setTrajectoryColor(GREY);
			RBSplineMotion->BSplineProps.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineMotion->BSplineProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->approximationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyPositions().size())
			{
				RBSplineMotion->setDegree(mainWinPtr->approximationDegreeSpinBox->value());
			}

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			if (mainWinPtr->keyPositions->isChecked())
			{
				RBSplineMotion->plotCtrlPositions(BROWN);
			}
			RBSplineMotion->plotMotion();
			glPopMatrix();
		}
	}

	if (Motion::RBSPLINEINTERPOLATION == true)
	{

		if (mainWinPtr->LimbMotionCapture->isChecked())
		{

			RBSplineInterpolationFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RBSplineInterpolationFemurMotion->BsplineInterprops.setObjectColor(DARK_GREEN);
			RBSplineInterpolationFemurMotion->BsplineInterprops.setTrajectoryColor(RED);
			RBSplineInterpolationFemurMotion->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyFemurPositions().size())
			{
				RBSplineInterpolationFemurMotion->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineInterpolationTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyTibiaPositions());
			RBSplineInterpolationTibiaMotion->BsplineInterprops.setObjectColor(BRONZE);
			RBSplineInterpolationTibiaMotion->BsplineInterprops.setTrajectoryColor(SPICY_PINK);
			RBSplineInterpolationTibiaMotion->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyTibiaPositions().size())
			{
				RBSplineInterpolationTibiaMotion->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}


			RBSplineInterpolationLeftUpperArmMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions());
			RBSplineInterpolationLeftUpperArmMotion->BsplineInterprops.setObjectColor(BRONZE);
			RBSplineInterpolationLeftUpperArmMotion->BsplineInterprops.setTrajectoryColor(SPICY_PINK);
			RBSplineInterpolationLeftUpperArmMotion->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions().size())
			{
				RBSplineInterpolationLeftUpperArmMotion->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineInterpolationLeftLowerArmMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions());
			RBSplineInterpolationLeftLowerArmMotion->BsplineInterprops.setObjectColor(BRONZE);
			RBSplineInterpolationLeftLowerArmMotion->BsplineInterprops.setTrajectoryColor(SPICY_PINK);
			RBSplineInterpolationLeftLowerArmMotion->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions().size())
			{
				RBSplineInterpolationLeftLowerArmMotion->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineInterpolationSpineMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeySpinePositions());
			RBSplineInterpolationSpineMotion->BsplineInterprops.setObjectColor(DUSTY_ROSE);
			RBSplineInterpolationSpineMotion->BsplineInterprops.setTrajectoryColor(NEON_PINK);
			RBSplineInterpolationSpineMotion->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeySpinePositions().size())
			{
				RBSplineInterpolationSpineMotion->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineInterpolationRightFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightFemurPositions());
			RBSplineInterpolationRightFemurMotion->BsplineInterprops.setObjectColor(HUNTERS_GREEN);
			RBSplineInterpolationRightFemurMotion->BsplineInterprops.setTrajectoryColor(SCARLET);
			RBSplineInterpolationRightFemurMotion->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyRightFemurPositions().size())
			{
				RBSplineInterpolationRightFemurMotion->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineInterpolationRightTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getKeyRightTibiaPositions());
			RBSplineInterpolationRightTibiaMotion->BsplineInterprops.setObjectColor(PLUM);
			RBSplineInterpolationRightTibiaMotion->BsplineInterprops.setTrajectoryColor(SALMON);
			RBSplineInterpolationRightTibiaMotion->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyRightTibiaPositions().size())
			{
				RBSplineInterpolationRightTibiaMotion->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RBSplineInterpolationFemurMotion->plotLimbMotion();
			RBSplineInterpolationTibiaMotion->plotLimbMotion();
			RBSplineInterpolationLeftUpperArmMotion->plotLimbMotion();
			RBSplineInterpolationLeftLowerArmMotion->plotLimbMotion();
			RBSplineInterpolationSpineMotion->plotSpineMotion();
			RBSplineInterpolationRightFemurMotion->plotLimbMotion();
			RBSplineInterpolationRightTibiaMotion->plotLimbMotion();
			glPopMatrix();

			/*RBSplineInterpolation->setLimbPoses(mainWinPtr->kinectSensor->getKeyFemurPositions());
			RBSplineInterpolation->BsplineInterprops.setObjectColor(DARK_GREEN);
			RBSplineInterpolation->BsplineInterprops.setTrajectoryColor(RED);
			RBSplineInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyPositions().size())
			{
				RBSplineInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}
			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			RBSplineInterpolation->plotLimbMotion();
			glPopMatrix();*/
		}
		else
		{
			RBSplineInterpolation->setCtrlPos(mainWinPtr->kinectSensor->getKeyPositions());
			RBSplineInterpolation->BsplineInterprops.setObjectColor(NEON_PINK);
			RBSplineInterpolation->BsplineInterprops.setTrajectoryColor(MANDARIN_ORANGE);
			RBSplineInterpolation->BsplineInterprops.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyPositions().size())
			{
				RBSplineInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineLeftHipInterpolation->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftHipPositions());
			RBSplineLeftHipInterpolation->BsplineInterprops.setObjectColor(GREEN);
			RBSplineLeftHipInterpolation->BsplineInterprops.setTrajectoryColor(LIGHT_BLUE);
			RBSplineLeftHipInterpolation->BsplineInterprops.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineLeftHipInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyLeftHipPositions().size())
			{
				RBSplineLeftHipInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineLeftShoulderInterpolation->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftShoulderPositions());
			RBSplineLeftShoulderInterpolation->BsplineInterprops.setObjectColor(YELLOW);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.setTrajectoryColor(LIGHT_RED);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineLeftShoulderInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyLeftShoulderPositions().size())
			{
				RBSplineLeftShoulderInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineLeftAnkleInterpolation->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftAnklePositions());
			RBSplineLeftAnkleInterpolation->BsplineInterprops.setObjectColor(PLUM);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.setTrajectoryColor(SALMON);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineLeftAnkleInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyLeftAnklePositions().size())
			{
				RBSplineLeftAnkleInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineRightAnkleInterpolation->setCtrlPos(mainWinPtr->kinectSensor->getKeyRightAnklePositions());
			RBSplineRightAnkleInterpolation->BSplineProps.setObjectColor(SPRING_GREEN);
			RBSplineRightAnkleInterpolation->BsplineInterprops.setTrajectoryColor(STEEL_BLUE);
			RBSplineRightAnkleInterpolation->BsplineInterprops.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineRightAnkleInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyRightAnklePositions().size())
			{
				RBSplineRightAnkleInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			RBSplineLeftElbowInterpolation->setCtrlPos(mainWinPtr->kinectSensor->getKeyLeftElbowPositions());
			RBSplineLeftElbowInterpolation->BsplineInterprops.setObjectColor(TAN);
			RBSplineLeftElbowInterpolation->BsplineInterprops.setTrajectoryColor(THISTLE);
			RBSplineLeftElbowInterpolation->BsplineInterprops.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineLeftElbowInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyLeftElbowPositions().size())
			{
				RBSplineLeftElbowInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}

			/*RBSplineRightElbowInterpolation->setCtrlPos(mainWinPtr->kinectSensor->getKeyRightElbowPositions());
			RBSplineRightElbowInterpolation->BsplineInterprops.setObjectColor(TAN);
			RBSplineRightElbowInterpolation->BsplineInterprops.setTrajectoryColor(THISTLE);
			RBSplineRightElbowInterpolation->BsplineInterprops.setObjectSize(static_cast<float>(mainWinPtr->ObjectSizeChanger->value()) / static_cast<float>(100));
			RBSplineRightElbowInterpolation->BsplineInterprops.setNoOfInterPos(mainWinPtr->densityChanger->value());
			if (mainWinPtr->interpolationDegreeSpinBox->value() < mainWinPtr->kinectSensor->getKeyRightElbowPositions().size())
			{
				RBSplineRightElbowInterpolation->setDegree(mainWinPtr->interpolationDegreeSpinBox->value());
			}*/

			glPushMatrix();
			glTranslated(m_xTrans, m_yTrans, m_zTrans);
			glRotated(m_xRotate, 1.0, 0.0, 0.0);
			glRotated(m_yRotate, 0.0, 1.0, 0.0);
			glRotated(m_zRotate, 0.0, 0.0, 1.0);
			
			RBSplineInterpolation->plotMotion();
			RBSplineLeftHipInterpolation->plotMotion();
			RBSplineLeftShoulderInterpolation->plotMotion();
			RBSplineLeftAnkleInterpolation->plotMotion();
			RBSplineRightAnkleInterpolation->plotMotion();
			RBSplineLeftElbowInterpolation->plotMotion();
			//RBSplineRightElbowInterpolation->plotMotion();
			glPopMatrix();
		}
	}

	if (Motion::RCONTINUOUSMOTION == true)
	{	
		ContinuousMotion->setCtrlPos(mainWinPtr->kinectSensor->getContinuousPositions());
		ContinuousMotion->ContinuousProps.setObjectColor(LIGHT_RED);
		ContinuousMotion->ContinuousProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		ContinuousMotion->ContinuousProps.setTrajectoryColor(PLUM);
		
		ContinuousLeftHipMotion->setCtrlPos(mainWinPtr->kinectSensor->getContinuousLeftHipPositions());
		ContinuousLeftHipMotion->ContinuousProps.setObjectColor(DARK_GREEN);
		ContinuousLeftHipMotion->ContinuousProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		
		ContinuousLeftShoulderMotion->setCtrlPos(mainWinPtr->kinectSensor->getContinuousLeftShoulderPositions());
		ContinuousLeftShoulderMotion->ContinuousProps.setObjectColor(GREEN);
		ContinuousLeftShoulderMotion->ContinuousProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

		ContinuousLeftAnkleMotion->setCtrlPos(mainWinPtr->kinectSensor->getContinuousLeftAnklePositions());
		ContinuousLeftAnkleMotion->ContinuousProps.setObjectColor(LIGHT_GREEN);
		ContinuousLeftAnkleMotion->ContinuousProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

		ContinuousRightAnkleMotion->setCtrlPos(mainWinPtr->kinectSensor->getContinuousRightAnklePositions());
		ContinuousRightAnkleMotion->ContinuousProps.setObjectColor(LIGHT_BLUE);
		ContinuousRightAnkleMotion->ContinuousProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

		ContinuousLeftElbowMotion->setCtrlPos(mainWinPtr->kinectSensor->getContinuousLeftElbowPositions());
		ContinuousLeftElbowMotion->ContinuousProps.setObjectColor(BLUE);
		ContinuousLeftElbowMotion->ContinuousProps.setNoOfInterPos(mainWinPtr->densityChanger->value());

		/*ContinuousRightElbowMotion->setCtrlPos(mainWinPtr->kinectSensor->getContinuousRightElbowPositions());
		ContinuousRightElbowMotion->ContinuousProps.setObjectColor(BLUE);
		ContinuousRightElbowMotion->ContinuousProps.setNoOfInterPos(mainWinPtr->densityChanger->value());*/

		glPushMatrix();
		glTranslated(m_xTrans, m_yTrans, m_zTrans);
		glRotated(m_xRotate, 1.0, 0.0, 0.0);
		glRotated(m_yRotate, 0.0, 1.0, 0.0);
		glRotated(m_zRotate, 0.0, 0.0, 1.0);
		ContinuousMotion->plotMotion();
		ContinuousLeftHipMotion->plotJointMotion();
		ContinuousLeftShoulderMotion->plotJointMotion();
		ContinuousLeftAnkleMotion->plotJointMotion();
		ContinuousRightAnkleMotion->plotJointMotion();
		ContinuousLeftElbowMotion->plotJointMotion();
		//ContinuousRightElbowMotion->plotJointMotion();
		glPopMatrix();
	}

	if (Motion::LIMBMOTION == true)
	{
		MFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getFemurPosesInSensor());
		//MFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getFemurPosesInSensor(),mainWinPtr->kinectSensor->getFemurLengthInSensor());
		MFemurMotion->LimbMotionProps.setObjectColor(YELLOW);
		MFemurMotion->LimbMotionProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		MFemurMotion->LimbMotionProps.setTrajectoryColor(DARK_WOOD);
		
		MTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getTibiaPosesInSensor());
		//MTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getTibiaPosesInSensor(), mainWinPtr->kinectSensor->getTibiaLengthInSensor());
		MTibiaMotion->LimbMotionProps.setObjectColor(BLUE);
		MTibiaMotion->LimbMotionProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		MTibiaMotion->LimbMotionProps.setTrajectoryColor(BLUE);

		MLeftUpperArmMotion->setLimbPoses(mainWinPtr->kinectSensor->getLeftUpperArmPosesInSensor());
		MLeftUpperArmMotion->LimbMotionProps.setObjectColor(BLUE);
		MLeftUpperArmMotion->LimbMotionProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		MLeftUpperArmMotion->LimbMotionProps.setTrajectoryColor(BLUE);

		MLeftLowerArmMotion->setLimbPoses(mainWinPtr->kinectSensor->getLeftLowerArmPosesInSensor());
		MLeftLowerArmMotion->LimbMotionProps.setObjectColor(BLUE);
		MLeftLowerArmMotion->LimbMotionProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		MLeftLowerArmMotion->LimbMotionProps.setTrajectoryColor(BLUE);

		MSpineMotion->setLimbPoses(mainWinPtr->kinectSensor->getSpinePosesInSensor());
		MSpineMotion->LimbMotionProps.setObjectColor(GREEN);
		MSpineMotion->LimbMotionProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		MSpineMotion->LimbMotionProps.setTrajectoryColor(DARK_GREEN);

		MRightFemurMotion->setLimbPoses(mainWinPtr->kinectSensor->getRightFemurPosesInSensor());
		MRightFemurMotion->LimbMotionProps.setObjectColor(LIGHT_GREEN);
		MRightFemurMotion->LimbMotionProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		MRightFemurMotion->LimbMotionProps.setTrajectoryColor(LIGHT_BLUE);

		MRightTibiaMotion->setLimbPoses(mainWinPtr->kinectSensor->getRightTibiaPosesInSensor());
		MRightTibiaMotion->LimbMotionProps.setObjectColor(SLATE_BLUE);
		MRightTibiaMotion->LimbMotionProps.setNoOfInterPos(mainWinPtr->densityChanger->value());
		MRightTibiaMotion->LimbMotionProps.setTrajectoryColor(SLATE_BLUE);

		glPushMatrix();
		glTranslated(m_xTrans, m_yTrans, m_zTrans);
		glRotated(m_xRotate, 1.0, 0.0, 0.0);
		glRotated(m_yRotate, 0.0, 1.0, 0.0);
		glRotated(m_zRotate, 0.0, 0.0, 1.0);
		
		MTibiaMotion->plotLimbMotion();
		MFemurMotion->plotLimbMotion();
		MLeftUpperArmMotion->plotLimbMotion();
		MLeftLowerArmMotion->plotLimbMotion();
		//MSpineMotion->plotLimbMotion();
		MSpineMotion->plotSpineMotion();
		MRightFemurMotion->plotLimbMotion();
		MRightTibiaMotion->plotLimbMotion();

		MFemurMotion->drawLineSkeleton(mainWinPtr->kinectSensor->getFemurStart(), mainWinPtr->kinectSensor->getFemurEnd());
		MTibiaMotion->drawLineSkeleton(mainWinPtr->kinectSensor->getTibiaStart(), mainWinPtr->kinectSensor->getTibiaEnd());
		MLeftUpperArmMotion->drawLineSkeleton(mainWinPtr->kinectSensor->getLeftUpperArmStart(), mainWinPtr->kinectSensor->getLeftUpperArmEnd());
		MLeftLowerArmMotion->drawLineSkeleton(mainWinPtr->kinectSensor->getLeftLowerArmStart(), mainWinPtr->kinectSensor->getLeftLowerArmEnd());
		MSpineMotion->drawLineSkeleton(mainWinPtr->kinectSensor->getSpineStart(), mainWinPtr->kinectSensor->getSpineEnd());
		MRightFemurMotion->drawLineSkeleton(mainWinPtr->kinectSensor->getRightFemurStart(), mainWinPtr->kinectSensor->getRightFemurEnd());
		MRightTibiaMotion->drawLineSkeleton(mainWinPtr->kinectSensor->getRightTibiaStart(), mainWinPtr->kinectSensor->getRightTibiaEnd());

		glPopMatrix();
		
	}

	glPopMatrix();
	//glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	//glMatrixMode(GL_MODELVIEW);

}

void GLWidget::resizeGL(int w, int h)
{
	m_widgetHeight = h;
	m_widgetWidth = w;
	glViewport(0, 0, w, h);
	aspectRatio = ((double)w / (double)h);
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	//gluPerspective(60.0, GLfloat(w) / h, 0.01, 1000.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	//glTranslatef(0.0f, 0.0f, -15.0f);

	glPopMatrix();
	glPopMatrix();

	/*glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho(-16.0000, 16.0000, -(16.0000 / aspectRatio), (16.0000 / aspectRatio), 0.1000, 1000.0000);
	
	gluPerspective(60.0,GLfloat(w)/h,0.01,1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -15.0f);*/
	
}



vector<DualQuaternion> GLWidget::getPositionFromContinuousMotion()
{
	vector<DualQuaternion> result;
	if (ContinuousMotion != NULL)
		return (ContinuousMotion->getAllPositions());
	else
		return result;
}

void GLWidget::storeKeyPositionsInGLWidget(bool st)
{
	if (mainWinPtr->continuousMotionCheckBox->isChecked())
	{
		if (Motion::RCONTINUOUSMOTION == true && ContinuousMotion != NULL)
		{
			if (mainWinPtr->positionRecordingSlider->value() < ContinuousMotion->getAllPositions().size())
				mainWinPtr->kinectSensor->addNewKeyCtrlPos(ContinuousMotion->getLessDensePosition(mainWinPtr->positionRecordingSlider->value()));
		}
	}
}

void GLWidget::storeAllKeyPositionsInGLWidget(bool st)
{
	if (mainWinPtr->continuousMotionCheckBox->isChecked())
	{
		if (Motion::RCONTINUOUSMOTION == true && ContinuousMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeyPositions().empty()){
				mainWinPtr->kinectSensor->clearKeyPositionInSensor();
			}

			if (!mainWinPtr->kinectSensor->getKeyLeftHipPositions().empty()){
				mainWinPtr->kinectSensor->clearKeyLeftHipPositionsInSensor();
			}

			if (!mainWinPtr->kinectSensor->getKeyLeftShoulderPositions().empty()){
				mainWinPtr->kinectSensor->clearKeyLeftShoulderPositionsInSensor();
			}

			if (!mainWinPtr->kinectSensor->getKeyLeftAnklePositions().empty()){
				mainWinPtr->kinectSensor->clearKeyLeftAnklePositionsInSensor();
			}

			if (!mainWinPtr->kinectSensor->getKeyRightAnklePositions().empty()){
				mainWinPtr->kinectSensor->clearKeyRightAnklePositionsInSensor();
			}

			if (!mainWinPtr->kinectSensor->getKeyLeftElbowPositions().empty()){
				mainWinPtr->kinectSensor->clearKeyLeftElbowPositionsInSensor();
			}
			/*if (!mainWinPtr->kinectSensor->getKeyRightElbowPositions().empty()){
				mainWinPtr->kinectSensor->clearKeyRightElbowPositionsInSensor();
			}*/

			for (auto i = 0; i < ContinuousMotion->getAllPositions().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeyCtrlPos(ContinuousMotion->getLessDensePosition(i));
			}
			
			for (auto i = 0; i < ContinuousLeftHipMotion->getAllPositions().size(); i++){
				mainWinPtr->kinectSensor->addNewLeftHipKeyCtrlPos(ContinuousLeftHipMotion->getLessDensePosition(i));
			}

			for (auto i = 0; i < ContinuousLeftShoulderMotion->getAllPositions().size(); i++){
				mainWinPtr->kinectSensor->addNewLeftShoulderKeyCtrlPos(ContinuousLeftShoulderMotion->getLessDensePosition(i));
			}

			for (auto i = 0; i < ContinuousLeftAnkleMotion->getAllPositions().size(); i++){
				mainWinPtr->kinectSensor->addNewLeftAnkleKeyCtrlPos(ContinuousLeftAnkleMotion->getLessDensePosition(i));
			}

			for (auto i = 0; i < ContinuousRightAnkleMotion->getAllPositions().size(); i++){
				mainWinPtr->kinectSensor->addNewRightAnkleKeyCtrlPos(ContinuousRightAnkleMotion->getLessDensePosition(i));
			}
			
			for (auto i = 0; i < ContinuousLeftElbowMotion->getAllPositions().size(); i++){
				mainWinPtr->kinectSensor->addNewLeftElbowKeyCtrlPos(ContinuousLeftElbowMotion->getLessDensePosition(i));
			}

			/*for (auto i = 0; i < ContinuousRightElbowMotion->getAllPositions().size(); i++){
				mainWinPtr->kinectSensor->addNewRightElbowKeyCtrlPos(ContinuousRightElbowMotion->getLessDensePosition(i));
			}*/

		}
	}

	if (mainWinPtr->limbMotionCheckBox->isChecked())
	{
		if (Motion::LIMBMOTION == true && MFemurMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeyFemurPositions().empty())
			{
				mainWinPtr->kinectSensor->clearKeyFemurPositionInSensor();
			}
			for (auto i = 0; i < MFemurMotion->getAllLimbPoses().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeyFemurPos(MFemurMotion->getLessDenseLimbPose(i));
			}
		}
		if (Motion::LIMBMOTION == true && MTibiaMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeyTibiaPositions().empty())
			{
				mainWinPtr->kinectSensor->clearKeyTibiaPositionInSensor();
			}
			for (auto i = 0; i < MTibiaMotion->getAllLimbPoses().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeyTibiaPos(MTibiaMotion->getLessDenseLimbPose(i));
			}
		}

		if (Motion::LIMBMOTION == true && MLeftUpperArmMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions().empty())
			{
				mainWinPtr->kinectSensor->clearKeyLeftUpperArmPositionInSensor();
			}
			for (auto i = 0; i < MLeftUpperArmMotion->getAllLimbPoses().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeyLeftUpperArmPos(MLeftUpperArmMotion->getLessDenseLimbPose(i));
			}
		}

		if (Motion::LIMBMOTION == true && MLeftLowerArmMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions().empty())
			{
				mainWinPtr->kinectSensor->clearKeyLeftLowerArmPositionInSensor();
			}
			for (auto i = 0; i < MLeftLowerArmMotion->getAllLimbPoses().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeyLeftLowerArmPos(MLeftLowerArmMotion->getLessDenseLimbPose(i));
			}
		}

		if (Motion::LIMBMOTION == true && MSpineMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeySpinePositions().empty())
			{
				mainWinPtr->kinectSensor->clearKeySpinePositionInSensor();
			}
			for (auto i = 0; i < MSpineMotion->getAllLimbPoses().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeySpinePos(MSpineMotion->getLessDenseLimbPose(i));
			}
		}

		if (Motion::LIMBMOTION == true && MRightFemurMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeyRightFemurPositions().empty())
			{
				mainWinPtr->kinectSensor->clearKeyRightFemurPositionInSensor();
			}
			for (auto i = 0; i < MRightFemurMotion->getAllLimbPoses().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeyRightFemurPos(MRightFemurMotion->getLessDenseLimbPose(i));
			}
		}

		if (Motion::LIMBMOTION == true && MRightTibiaMotion != NULL)
		{
			if (!mainWinPtr->kinectSensor->getKeyRightTibiaPositions().empty())
			{
				mainWinPtr->kinectSensor->clearKeyRightTibiaPositionInSensor();
			}
			for (auto i = 0; i < MRightTibiaMotion->getAllLimbPoses().size(); i++)
			{
				mainWinPtr->kinectSensor->addNewKeyRightTibiaPos(MRightTibiaMotion->getLessDenseLimbPose(i));
			}
		}
	}
	update();
}

void GLWidget::mousePressEvent(QMouseEvent* even)
{
	button = even->button();
	cursorLocation = even->pos();
	update();
}

void GLWidget::mouseMoveEvent(QMouseEvent* even)
{
	moveCursorLocation = even->pos();
	if (button==Qt::LeftButton)
	{
		m_yRotate -= (cursorLocation.x() - moveCursorLocation.x()) / 2.0;
		m_xRotate -= (cursorLocation.y() - moveCursorLocation.y()) / 2.0;

		cursorLocation.setX(moveCursorLocation.x());
		cursorLocation.setY(moveCursorLocation.y());
	}

	if (button == Qt::RightButton)
	{
		m_xTrans -= (cursorLocation.x() - moveCursorLocation.x()) / 10.0f;
		m_yTrans += (cursorLocation.y() - moveCursorLocation.y()) / 10.0f;

		cursorLocation.setX(moveCursorLocation.x());
		cursorLocation.setY(moveCursorLocation.y());
	}

	if (button == Qt::MiddleButton)
	{
		m_zTrans -= (cursorLocation.y() - moveCursorLocation.y()) / 10.0f;

		cursorLocation.setX(moveCursorLocation.x());
		cursorLocation.setY(moveCursorLocation.y());
	}

	update();
}

void GLWidget::updateSelectedPosition(int pos)
{
	if (mainWinPtr->screwMotion->isChecked())
	{
		if (Motion::SCREWMOTION == true && RScrewMotion != NULL)
		{
			if (RScrewMotion->ScrewProps.getSpecificPosition() != pos)
			{
				RScrewMotion->ScrewProps.setSpecificPosition(pos);
			}
		}
	}

	if (mainWinPtr->bezierMotion->isChecked())
	{
		if (Motion::BEZIERMOTION == true && RBezierMotion != NULL)
		{
			if (RBezierMotion->BezierProps.getSpecificPosition() != pos)
			{
				RBezierMotion->BezierProps.setSpecificPosition(pos);
			}

		}
	}

	if (mainWinPtr->bsplineApproximation->isChecked())
	{
		if (Motion::RBSPLINEMOTION == true && RBSplineMotion != NULL)
		{
			if (RBSplineMotion->BSplineProps.getSpecificPosition() != pos)
			{
				RBSplineMotion->BSplineProps.setSpecificPosition(pos);
			}
		}
	}

	if (mainWinPtr->bsplineInterpolation->isChecked())
	{
		if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolation != NULL /*&& RBSplineHipInterpolation!=NULL*/)
		{
			if (RBSplineInterpolation->BsplineInterprops.getSpecificPosition() != pos)
			{
				RBSplineInterpolation->BsplineInterprops.setSpecificPosition(pos);
			}
			/*if (RBSplineHipInterpolation->BsplineInterprops.getSpecificPosition() != pos)
			{
			RBSplineHipInterpolation->BsplineInterprops.setSpecificPosition(pos);
			}*/
		}
	}

	if (mainWinPtr->continuousMotionCheckBox->isChecked())
	{
		if (Motion::RCONTINUOUSMOTION == true && ContinuousMotion != NULL /*&& ContinuousHipMotion!=NULL*/)
		{
			if (ContinuousMotion->ContinuousProps.getSpecificPosition() != pos)
			{
				ContinuousMotion->ContinuousProps.setSpecificPosition(pos);
			}

			/*if (ContinuousHipMotion->ContinuousProps.getSpecificPosition() != pos)
			{
			ContinuousHipMotion->ContinuousProps.setSpecificPosition(pos);
			}*/
		}
	}
	update();
}

void GLWidget::updateDensity(int val)
{
	update();
}

void GLWidget::updateGLToPlot(int st)
{
	if (mainWinPtr->screwMotion->isChecked())
	{
		Motion::SCREWMOTION = true;
		if (RScrewMotion == NULL)
		{
			RScrewMotion = new MScrewMotion;
			if (!mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked()&&!mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RScrewFemurMotion == NULL)
		{
			RScrewFemurMotion = new MScrewMotion;
			if (!mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}
		if (RScrewTibiaMotion == NULL)
		{
			RScrewTibiaMotion = new MScrewMotion;
			if (!mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RScrewSpineMotion == NULL){
			RScrewSpineMotion = new MScrewMotion;
			if (!mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RScrewRightFemurMotion == NULL){
			RScrewRightFemurMotion = new MScrewMotion;
			if (!mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RScrewRightTibiaMotion == NULL){
			RScrewRightTibiaMotion = new MScrewMotion;
			if (!mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RScrewMotion != NULL)
			RScrewMotion->ScrewProps.plotTrajectory(true);
		else 
			RScrewMotion->ScrewProps.plotTrajectory(false);


		if (mainWinPtr->coordinateFrameCheckBox->isChecked() && RScrewMotion != NULL)
			RScrewMotion->ScrewProps.plotCoordinateFrame(true);
		else 
			RScrewMotion->ScrewProps.plotCoordinateFrame(false);


		if (mainWinPtr->plotObjectCheckBox->isChecked() && RScrewMotion != NULL)
			RScrewMotion->ScrewProps.plotObject(true);
		else 
			RScrewMotion->ScrewProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RScrewFemurMotion != NULL)
			RScrewFemurMotion->ScrewProps.plotTrajectory(true);
		else 
			RScrewFemurMotion->ScrewProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RScrewFemurMotion != NULL)
			RScrewFemurMotion->ScrewProps.plotObject(true);
		else
			RScrewFemurMotion->ScrewProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RScrewTibiaMotion != NULL)
			RScrewTibiaMotion->ScrewProps.plotTrajectory(true);
		else
			RScrewTibiaMotion->ScrewProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RScrewTibiaMotion != NULL)
			RScrewTibiaMotion->ScrewProps.plotObject(true);
		else
			RScrewTibiaMotion->ScrewProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RScrewSpineMotion != NULL)
			RScrewSpineMotion->ScrewProps.plotTrajectory(true);
		else
			RScrewSpineMotion->ScrewProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RScrewSpineMotion != NULL)
			RScrewSpineMotion->ScrewProps.plotObject(true);
		else
			RScrewSpineMotion->ScrewProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RScrewRightFemurMotion != NULL)
			RScrewRightFemurMotion->ScrewProps.plotTrajectory(true);
		else
			RScrewRightFemurMotion->ScrewProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RScrewRightFemurMotion != NULL)
			RScrewRightFemurMotion->ScrewProps.plotObject(true);
		else
			RScrewRightFemurMotion->ScrewProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RScrewRightTibiaMotion != NULL)
			RScrewRightTibiaMotion->ScrewProps.plotTrajectory(true);
		else
			RScrewRightTibiaMotion->ScrewProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RScrewRightTibiaMotion != NULL)
			RScrewRightTibiaMotion->ScrewProps.plotObject(true);
		else
			RScrewRightTibiaMotion->ScrewProps.plotObject(false);
	}
	else
	{
		Motion::SCREWMOTION = false;
		if (RScrewMotion != NULL)
		{
			delete RScrewMotion;
			RScrewMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RScrewFemurMotion != NULL){
			delete RScrewFemurMotion;
			RScrewFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RScrewTibiaMotion != NULL){
			delete RScrewTibiaMotion;
			RScrewTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RScrewSpineMotion != NULL){
			delete RScrewSpineMotion;
			RScrewSpineMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RScrewRightFemurMotion != NULL){
			delete RScrewRightFemurMotion;
			RScrewRightFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RScrewRightTibiaMotion != NULL){
			delete RScrewRightTibiaMotion;
			RScrewRightTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
	}

	if (mainWinPtr->bezierMotion->isChecked())
	{
		Motion::BEZIERMOTION = true;
		if (RBezierMotion == NULL)
		{
			RBezierMotion = new MBezierMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}
		if (RBezierFemurMotion == NULL)
		{
			RBezierFemurMotion = new MBezierMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBezierTibiaMotion == NULL)
		{
			RBezierTibiaMotion = new MBezierMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBezierSpineMotion == NULL){
			RBezierSpineMotion = new MBezierMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBezierRightFemurMotion == NULL){
			RBezierRightFemurMotion = new MBezierMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBezierRightTibiaMotion == NULL){
			RBezierRightTibiaMotion = new MBezierMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBezierMotion!=NULL)
			RBezierMotion->BezierProps.plotTrajectory(true);
		else
			RBezierMotion->BezierProps.plotTrajectory(false);
		

		if (mainWinPtr->coordinateFrameCheckBox->isChecked() && RBezierMotion != NULL)
			RBezierMotion->BezierProps.plotCoordinateFrame(true);
		else
			RBezierMotion->BezierProps.plotCoordinateFrame(false);
		

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBezierMotion != NULL)
			RBezierMotion->BezierProps.plotObject(true);
		else
			RBezierMotion->BezierProps.plotObject(false);


		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBezierFemurMotion != NULL)
			RBezierFemurMotion->BezierProps.plotTrajectory(true);
		else
			RBezierFemurMotion->BezierProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBezierFemurMotion != NULL)
			RBezierFemurMotion->BezierProps.plotObject(true);
		else
			RBezierFemurMotion->BezierProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBezierTibiaMotion != NULL)
			RBezierTibiaMotion->BezierProps.plotTrajectory(true);
		else
			RBezierTibiaMotion->BezierProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBezierTibiaMotion != NULL)
			RBezierTibiaMotion->BezierProps.plotObject(true);
		else
			RBezierTibiaMotion->BezierProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBezierSpineMotion != NULL)
			RBezierSpineMotion->BezierProps.plotTrajectory(true);
		else
			RBezierSpineMotion->BezierProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBezierSpineMotion != NULL)
			RBezierSpineMotion->BezierProps.plotObject(true);
		else
			RBezierSpineMotion->BezierProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBezierRightFemurMotion != NULL)
			RBezierRightFemurMotion->BezierProps.plotTrajectory(true);
		else
			RBezierRightFemurMotion->BezierProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBezierRightFemurMotion != NULL)
			RBezierRightFemurMotion->BezierProps.plotObject(true);
		else
			RBezierRightFemurMotion->BezierProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBezierRightTibiaMotion != NULL)
			RBezierRightTibiaMotion->BezierProps.plotTrajectory(true);
		else
			RBezierRightTibiaMotion->BezierProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBezierRightTibiaMotion != NULL)
			RBezierRightTibiaMotion->BezierProps.plotObject(true);
		else
			RBezierRightTibiaMotion->BezierProps.plotObject(false);
	}
	else
	{
		Motion::BEZIERMOTION = false;
		if (RBezierMotion != NULL)
		{
			delete RBezierMotion;
			RBezierMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBezierFemurMotion != NULL)
		{
			delete RBezierFemurMotion;
			RBezierFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBezierTibiaMotion != NULL)
		{
			delete RBezierTibiaMotion;
			RBezierTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBezierSpineMotion != NULL){
			delete RBezierSpineMotion;
			RBezierSpineMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBezierRightFemurMotion != NULL){
			delete RBezierRightFemurMotion;
			RBezierRightFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBezierRightTibiaMotion != NULL){
			delete RBezierRightTibiaMotion;
			RBezierRightTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
	}

	if (mainWinPtr->bsplineApproximation->isChecked())
	{
		Motion::RBSPLINEMOTION = true;
		if (RBSplineMotion == NULL)
		{
			RBSplineMotion = new MRBSplineMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}
		if (RBSplineFemurMotion == NULL)
		{
			RBSplineFemurMotion = new MRBSplineMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}
		if (RBSplineTibiaMotion == NULL)
		{
			RBSplineTibiaMotion = new MRBSplineMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSPlineSpineMotion == NULL){
			RBSPlineSpineMotion = new MRBSplineMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineRightFemurMotion == NULL){
			RBSplineRightFemurMotion = new MRBSplineMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineRightTibiaMotion == NULL){
			RBSplineRightTibiaMotion = new MRBSplineMotion;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineInterpolation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineMotion != NULL)
			RBSplineMotion->BSplineProps.plotTrajectory(true);
		else 
			RBSplineMotion->BSplineProps.plotTrajectory(false);


		if (mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineMotion != NULL)
			RBSplineMotion->BSplineProps.plotCoordinateFrame(true);
		else 
			RBSplineMotion->BSplineProps.plotCoordinateFrame(false);


		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineMotion != NULL)
			RBSplineMotion->BSplineProps.plotObject(true);
		else
			RBSplineMotion->BSplineProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineFemurMotion != NULL)
			RBSplineFemurMotion->BSplineProps.plotTrajectory(true);
		else
			RBSplineFemurMotion->BSplineProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineFemurMotion != NULL)
			RBSplineFemurMotion->BSplineProps.plotObject(true);
		else
			RBSplineFemurMotion->BSplineProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineTibiaMotion != NULL)
			RBSplineTibiaMotion->BSplineProps.plotTrajectory(true);
		else
			RBSplineTibiaMotion->BSplineProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineTibiaMotion != NULL)
			RBSplineTibiaMotion->BSplineProps.plotObject(true);
		else
			RBSplineTibiaMotion->BSplineProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSPlineSpineMotion != NULL)
			RBSPlineSpineMotion->BSplineProps.plotTrajectory(true);
		else
			RBSPlineSpineMotion->BSplineProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSPlineSpineMotion != NULL)
			RBSPlineSpineMotion->BSplineProps.plotObject(true);
		else
			RBSPlineSpineMotion->BSplineProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineRightFemurMotion != NULL)
			RBSplineRightFemurMotion->BSplineProps.plotTrajectory(true);
		else
			RBSplineRightFemurMotion->BSplineProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineRightFemurMotion != NULL)
			RBSplineRightFemurMotion->BSplineProps.plotObject(true);
		else
			RBSplineRightFemurMotion->BSplineProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineRightTibiaMotion != NULL)
			RBSplineRightTibiaMotion->BSplineProps.plotTrajectory(true);
		else
			RBSplineRightTibiaMotion->BSplineProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineRightTibiaMotion != NULL)
			RBSplineRightTibiaMotion->BSplineProps.plotObject(true);
		else
			RBSplineRightTibiaMotion->BSplineProps.plotObject(false);

	}
	else
	{
		Motion::RBSPLINEMOTION = false;
		
		if (RBSplineMotion != NULL)
		{
			delete RBSplineMotion;
			RBSplineMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineFemurMotion != NULL)
		{
			delete RBSplineFemurMotion;
			RBSplineFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineTibiaMotion != NULL)
		{
			delete RBSplineTibiaMotion;
			RBSplineTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSPlineSpineMotion != NULL){
			delete RBSPlineSpineMotion;
			RBSPlineSpineMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineRightFemurMotion != NULL){
			delete RBSplineRightFemurMotion;
			RBSplineRightFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineRightTibiaMotion != NULL){
			delete RBSplineRightTibiaMotion;
			RBSplineRightTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
	}

	if (mainWinPtr->bsplineInterpolation->isChecked())
	{
		Motion::RBSPLINEINTERPOLATION = true;
		if (RBSplineInterpolation == NULL)
		{
			RBSplineInterpolation = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}
		if (RBSplineInterpolationFemurMotion == NULL)
		{
			RBSplineInterpolationFemurMotion = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}
		if (RBSplineInterpolationTibiaMotion == NULL)
		{
			RBSplineInterpolationTibiaMotion = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineInterpolationLeftUpperArmMotion == NULL)
		{
			RBSplineInterpolationLeftUpperArmMotion = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineInterpolationLeftLowerArmMotion == NULL)
		{
			RBSplineInterpolationLeftLowerArmMotion = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineInterpolationSpineMotion == NULL){
			RBSplineInterpolationSpineMotion = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineInterpolationRightFemurMotion == NULL){
			RBSplineInterpolationRightFemurMotion = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineInterpolationRightTibiaMotion == NULL){
			RBSplineInterpolationRightTibiaMotion = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineLeftHipInterpolation == NULL)
		{
			RBSplineLeftHipInterpolation = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineLeftShoulderInterpolation == NULL){
			RBSplineLeftShoulderInterpolation = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineLeftAnkleInterpolation == NULL){
			RBSplineLeftAnkleInterpolation = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineRightAnkleInterpolation == NULL){
			RBSplineRightAnkleInterpolation = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		if (RBSplineLeftElbowInterpolation == NULL){
			RBSplineLeftElbowInterpolation = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}

		/*if (RBSplineRightElbowInterpolation = NULL){
			RBSplineRightElbowInterpolation = new MRBSplineInterpolation;
			if (!mainWinPtr->screwMotion->isChecked() && !mainWinPtr->bezierMotion->isChecked() && !mainWinPtr->bsplineApproximation->isChecked())
				mainWinPtr->plotObjectCheckBox->setChecked(true);
		}*/

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolation != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineLeftHipInterpolation != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineLeftShoulderInterpolation != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineLeftAnkleInterpolation != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineRightAnkleInterpolation != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineLeftElbowInterpolation != NULL /*|| mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineRightElbowInterpolation != NULL*/)
		{
			RBSplineInterpolation->BsplineInterprops.plotTrajectory(true);
			RBSplineLeftHipInterpolation->BsplineInterprops.plotTrajectory(true);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.plotTrajectory(true);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.plotTrajectory(true);
			RBSplineRightAnkleInterpolation->BsplineInterprops.plotTrajectory(true);
			RBSplineLeftElbowInterpolation->BsplineInterprops.plotTrajectory(true);
			//RBSplineRightElbowInterpolation->BsplineInterprops.plotTrajectory(true);
		}
		else
		{
			RBSplineInterpolation->BsplineInterprops.plotTrajectory(false);
			RBSplineLeftHipInterpolation->BsplineInterprops.plotTrajectory(false);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.plotTrajectory(false);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.plotTrajectory(false);
			RBSplineRightAnkleInterpolation->BsplineInterprops.plotTrajectory(false);
			RBSplineLeftElbowInterpolation->BsplineInterprops.plotTrajectory(false);
			//RBSplineRightElbowInterpolation->BsplineInterprops.plotTrajectory(false);
			
		}

		if (mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineInterpolation != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineLeftHipInterpolation != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineLeftShoulderInterpolation != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineLeftAnkleInterpolation != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineRightAnkleInterpolation != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineLeftElbowInterpolation != NULL /*|| mainWinPtr->coordinateFrameCheckBox->isChecked() && RBSplineRightElbowInterpolation != NULL*/)
		{
			RBSplineInterpolation->BsplineInterprops.plotCoordinateFrame(true);
			RBSplineLeftHipInterpolation->BsplineInterprops.plotCoordinateFrame(true);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.plotCoordinateFrame(true);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.plotCoordinateFrame(true);
			RBSplineRightAnkleInterpolation->BsplineInterprops.plotCoordinateFrame(true);
			RBSplineLeftElbowInterpolation->BsplineInterprops.plotCoordinateFrame(true);
			//RBSplineRightElbowInterpolation->BsplineInterprops.plotCoordinateFrame(true);
		}
		else
		{
			RBSplineInterpolation->BsplineInterprops.plotCoordinateFrame(false);
			RBSplineLeftHipInterpolation->BsplineInterprops.plotCoordinateFrame(false);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.plotCoordinateFrame(false);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.plotCoordinateFrame(false);
			RBSplineRightAnkleInterpolation->BsplineInterprops.plotCoordinateFrame(false);
			RBSplineLeftElbowInterpolation->BsplineInterprops.plotCoordinateFrame(false);
			//RBSplineRightElbowInterpolation->BsplineInterprops.plotCoordinateFrame(false);
		}

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolation != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineLeftHipInterpolation != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineLeftShoulderInterpolation != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineLeftAnkleInterpolation != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineRightAnkleInterpolation != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineLeftElbowInterpolation != NULL /*|| mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineRightElbowInterpolation != NULL*/)
		{
			RBSplineInterpolation->BsplineInterprops.plotObject(true);
			RBSplineLeftHipInterpolation->BsplineInterprops.plotObject(true);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.plotObject(true);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.plotObject(true);
			RBSplineRightAnkleInterpolation->BsplineInterprops.plotObject(true);
			RBSplineLeftElbowInterpolation->BsplineInterprops.plotObject(true);
			//RBSplineRightElbowInterpolation->BsplineInterprops.plotObject(true);
		}
		else
		{
			RBSplineInterpolation->BsplineInterprops.plotObject(false);
			RBSplineLeftHipInterpolation->BsplineInterprops.plotObject(false);
			RBSplineLeftShoulderInterpolation->BsplineInterprops.plotObject(false);
			RBSplineLeftAnkleInterpolation->BsplineInterprops.plotObject(false);
			RBSplineRightAnkleInterpolation->BsplineInterprops.plotObject(false);
			RBSplineLeftElbowInterpolation->BsplineInterprops.plotObject(false);
			//RBSplineRightElbowInterpolation->BsplineInterprops.plotObject(false);
		}

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolationFemurMotion != NULL)
			RBSplineInterpolationFemurMotion->BsplineInterprops.plotTrajectory(true);
		else
			RBSplineInterpolationFemurMotion->BsplineInterprops.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolationFemurMotion != NULL)
			RBSplineInterpolationFemurMotion->BsplineInterprops.plotObject(true);
		else
			RBSplineInterpolationFemurMotion->BsplineInterprops.plotObject(false);


		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolationTibiaMotion != NULL)
			RBSplineInterpolationTibiaMotion->BsplineInterprops.plotTrajectory(true);
		else
			RBSplineInterpolationTibiaMotion->BsplineInterprops.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolationTibiaMotion != NULL)
			RBSplineInterpolationTibiaMotion->BsplineInterprops.plotObject(true);
		else
			RBSplineInterpolationTibiaMotion->BsplineInterprops.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolationLeftUpperArmMotion != NULL)
			RBSplineInterpolationLeftUpperArmMotion->BsplineInterprops.plotTrajectory(true);
		else
			RBSplineInterpolationLeftUpperArmMotion->BsplineInterprops.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolationLeftUpperArmMotion != NULL)
			RBSplineInterpolationLeftUpperArmMotion->BsplineInterprops.plotObject(true);
		else
			RBSplineInterpolationLeftUpperArmMotion->BsplineInterprops.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolationLeftLowerArmMotion != NULL)
			RBSplineInterpolationLeftLowerArmMotion->BsplineInterprops.plotTrajectory(true);
		else
			RBSplineInterpolationLeftLowerArmMotion->BsplineInterprops.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolationLeftLowerArmMotion != NULL)
			RBSplineInterpolationLeftLowerArmMotion->BsplineInterprops.plotObject(true);
		else
			RBSplineInterpolationLeftLowerArmMotion->BsplineInterprops.plotObject(false);


		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolationSpineMotion != NULL)
			RBSplineInterpolationSpineMotion->BsplineInterprops.plotTrajectory(true);
		else
			RBSplineInterpolationSpineMotion->BsplineInterprops.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolationSpineMotion != NULL)
			RBSplineInterpolationSpineMotion->BsplineInterprops.plotObject(true);
		else
			RBSplineInterpolationSpineMotion->BsplineInterprops.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolationRightFemurMotion != NULL)
			RBSplineInterpolationRightFemurMotion->BsplineInterprops.plotTrajectory(true);
		else
			RBSplineInterpolationRightFemurMotion->BsplineInterprops.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolationRightFemurMotion != NULL)
			RBSplineInterpolationRightFemurMotion->BsplineInterprops.plotObject(true);
		else
			RBSplineInterpolationRightFemurMotion->BsplineInterprops.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && RBSplineInterpolationRightTibiaMotion != NULL)
			RBSplineInterpolationRightTibiaMotion->BsplineInterprops.plotTrajectory(true);
		else
			RBSplineInterpolationRightTibiaMotion->BsplineInterprops.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && RBSplineInterpolationRightTibiaMotion != NULL)
			RBSplineInterpolationRightTibiaMotion->BsplineInterprops.plotObject(true);
		else
			RBSplineInterpolationRightTibiaMotion->BsplineInterprops.plotObject(false);
	}
	else
	{
		Motion::RBSPLINEINTERPOLATION = false;
		if (RBSplineInterpolation != NULL)
		{
			delete RBSplineInterpolation;
			RBSplineInterpolation = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineInterpolationFemurMotion != NULL)
		{
			delete RBSplineInterpolationFemurMotion;
			RBSplineInterpolationFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineInterpolationTibiaMotion != NULL)
		{
			delete RBSplineInterpolationTibiaMotion;
			RBSplineInterpolationTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineInterpolationLeftUpperArmMotion != NULL)
		{
			delete RBSplineInterpolationLeftUpperArmMotion;
			RBSplineInterpolationLeftUpperArmMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineInterpolationLeftLowerArmMotion != NULL)
		{
			delete RBSplineInterpolationLeftLowerArmMotion;
			RBSplineInterpolationLeftLowerArmMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineInterpolationSpineMotion != NULL){
			delete RBSplineInterpolationSpineMotion;
			RBSplineInterpolationSpineMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineInterpolationRightFemurMotion != NULL){
			delete RBSplineInterpolationRightFemurMotion;
			RBSplineInterpolationRightFemurMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineInterpolationRightTibiaMotion != NULL){
			delete RBSplineInterpolationRightTibiaMotion;
			RBSplineInterpolationRightTibiaMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineLeftHipInterpolation != NULL)
		{
			delete RBSplineLeftHipInterpolation;
			RBSplineLeftHipInterpolation = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineLeftShoulderInterpolation != NULL)
		{
			delete RBSplineLeftShoulderInterpolation;
			RBSplineLeftShoulderInterpolation = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (RBSplineLeftAnkleInterpolation != NULL){
			delete RBSplineLeftAnkleInterpolation;
			RBSplineLeftAnkleInterpolation = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineRightAnkleInterpolation != NULL){
			delete RBSplineRightAnkleInterpolation;
			RBSplineRightAnkleInterpolation = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (RBSplineLeftElbowInterpolation != NULL){
			delete RBSplineLeftElbowInterpolation;
			RBSplineLeftElbowInterpolation = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		/*if (RBSplineRightElbowInterpolation != NULL){
			delete RBSplineRightElbowInterpolation;
			RBSplineRightElbowInterpolation = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}*/
	}

	

	if (mainWinPtr->continuousMotionCheckBox->isChecked())
	{
		Motion::RCONTINUOUSMOTION = true;
		if (ContinuousMotion == NULL)
		{
			ContinuousMotion = new MContinuousMotion;
		}

		if (ContinuousLeftHipMotion == NULL)
		{
			ContinuousLeftHipMotion = new MContinuousMotion;
		}

		if (ContinuousLeftShoulderMotion == NULL)
		{
			ContinuousLeftShoulderMotion = new MContinuousMotion;
		}

		if (ContinuousLeftAnkleMotion == NULL)
		{
			ContinuousLeftAnkleMotion = new MContinuousMotion;
		}

		if (ContinuousRightAnkleMotion == NULL)
		{
			ContinuousRightAnkleMotion = new MContinuousMotion;
		}

		if (ContinuousLeftElbowMotion == NULL)
		{
			ContinuousLeftElbowMotion = new MContinuousMotion;
		}
		/*if (ContinuousRightElbowMotion == NULL)
		{
			ContinuousRightElbowMotion = new MContinuousMotion;
		}*/
		

		if (mainWinPtr->coordinateFrameCheckBox->isChecked() && ContinuousMotion != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && ContinuousLeftHipMotion != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && ContinuousLeftShoulderMotion != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && ContinuousLeftAnkleMotion != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && ContinuousRightAnkleMotion != NULL || mainWinPtr->coordinateFrameCheckBox->isChecked() && ContinuousLeftElbowMotion!=NULL)
		{
			ContinuousMotion->ContinuousProps.plotCoordinateFrame(true);
			ContinuousLeftHipMotion->ContinuousProps.plotCoordinateFrame(true);
			ContinuousLeftShoulderMotion->ContinuousProps.plotCoordinateFrame(true);
			ContinuousLeftAnkleMotion->ContinuousProps.plotCoordinateFrame(true);
			ContinuousRightAnkleMotion->ContinuousProps.plotCoordinateFrame(true);
			ContinuousLeftElbowMotion->ContinuousProps.plotCoordinateFrame(true);
		}
		else
		{
			ContinuousMotion->ContinuousProps.plotCoordinateFrame(false);
			ContinuousLeftHipMotion->ContinuousProps.plotCoordinateFrame(false);
			ContinuousLeftShoulderMotion->ContinuousProps.plotCoordinateFrame(false);
			ContinuousLeftAnkleMotion->ContinuousProps.plotCoordinateFrame(false);
			ContinuousRightAnkleMotion->ContinuousProps.plotCoordinateFrame(false);
			ContinuousLeftElbowMotion->ContinuousProps.plotCoordinateFrame(false);
		}

		if (mainWinPtr->trajectoryCheckBox->isChecked() && ContinuousMotion != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && ContinuousLeftHipMotion != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && ContinuousLeftShoulderMotion != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && ContinuousLeftAnkleMotion != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && ContinuousRightAnkleMotion != NULL || mainWinPtr->trajectoryCheckBox->isChecked() && ContinuousLeftElbowMotion != NULL /*|| mainWinPtr->trajectoryCheckBox->isChecked() && ContinuousRightElbowMotion != NULL*/)
		{
			ContinuousMotion->ContinuousProps.plotTrajectory(true);
			ContinuousLeftHipMotion->ContinuousProps.plotTrajectory(true);
			ContinuousLeftShoulderMotion->ContinuousProps.plotTrajectory(true);
			ContinuousLeftAnkleMotion->ContinuousProps.plotTrajectory(true);
			ContinuousRightAnkleMotion->ContinuousProps.plotTrajectory(true);
			ContinuousLeftElbowMotion->ContinuousProps.plotTrajectory(true);
			//ContinuousRightElbowMotion->ContinuousProps.plotTrajectory(true);
		}
		else
		{
			ContinuousMotion->ContinuousProps.plotTrajectory(false);
			ContinuousLeftHipMotion->ContinuousProps.plotTrajectory(false);
			ContinuousLeftShoulderMotion->ContinuousProps.plotTrajectory(false);
			ContinuousLeftAnkleMotion->ContinuousProps.plotTrajectory(false);
			ContinuousRightAnkleMotion->ContinuousProps.plotTrajectory(false);
			ContinuousLeftElbowMotion->ContinuousProps.plotTrajectory(false);
			//ContinuousRightElbowMotion->ContinuousProps.plotTrajectory(false);
		}


		if (mainWinPtr->plotObjectCheckBox->isChecked() && ContinuousMotion != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && ContinuousLeftHipMotion != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && ContinuousLeftShoulderMotion != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && ContinuousLeftAnkleMotion != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && ContinuousRightAnkleMotion != NULL || mainWinPtr->plotObjectCheckBox->isChecked() && ContinuousLeftElbowMotion != NULL /*|| mainWinPtr->plotObjectCheckBox->isChecked() && ContinuousRightElbowMotion != NULL*/)
		{
			ContinuousMotion->ContinuousProps.plotObject(true);
			ContinuousLeftHipMotion->ContinuousProps.plotObject(true);
			ContinuousLeftShoulderMotion->ContinuousProps.plotObject(true);
			ContinuousLeftAnkleMotion->ContinuousProps.plotObject(true);
			ContinuousRightAnkleMotion->ContinuousProps.plotObject(true);
			ContinuousLeftElbowMotion->ContinuousProps.plotObject(true);
			//ContinuousRightElbowMotion->ContinuousProps.plotObject(true);
		}
		else
		{
			ContinuousMotion->ContinuousProps.plotObject(false);
			ContinuousLeftHipMotion->ContinuousProps.plotObject(false);
			ContinuousLeftShoulderMotion->ContinuousProps.plotObject(false);
			ContinuousLeftAnkleMotion->ContinuousProps.plotObject(false);
			ContinuousRightAnkleMotion->ContinuousProps.plotObject(false);
			ContinuousLeftElbowMotion->ContinuousProps.plotObject(false);
			//ContinuousRightElbowMotion->ContinuousProps.plotObject(false);
		}
	}
	else
	{
		Motion::RCONTINUOUSMOTION = false;
		if (ContinuousMotion != NULL)
		{
			delete ContinuousMotion;
			ContinuousMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (ContinuousLeftHipMotion != NULL)
		{
			delete ContinuousLeftHipMotion;
			ContinuousLeftHipMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (ContinuousLeftShoulderMotion != NULL)
		{
			delete ContinuousLeftShoulderMotion;
			ContinuousLeftShoulderMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (ContinuousLeftAnkleMotion != NULL){
			delete ContinuousLeftAnkleMotion;
			ContinuousLeftAnkleMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		if (ContinuousRightAnkleMotion != NULL){
			delete ContinuousRightAnkleMotion;
			ContinuousRightAnkleMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}
		if (ContinuousLeftElbowMotion != NULL){
			delete ContinuousLeftElbowMotion;
			ContinuousLeftElbowMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}

		/*if (ContinuousRightElbowMotion != NULL){
			delete ContinuousRightElbowMotion;
			ContinuousRightElbowMotion = NULL;
			mainWinPtr->densityChanger->setValue(mainWinPtr->densityChanger->value());
			emit mainWinPtr->densityChanger->sliderMoved(mainWinPtr->densityChanger->value());
		}*/
	}

	if (mainWinPtr->limbMotionCheckBox->isChecked())
	{
		Motion::LIMBMOTION = true;
		if (MFemurMotion == NULL){
			MFemurMotion = new LimbMotion;
		}
		if (MTibiaMotion == NULL){
			MTibiaMotion = new LimbMotion;
		}

		if (MLeftUpperArmMotion == NULL){
			MLeftUpperArmMotion = new LimbMotion;
		}
		if (MLeftLowerArmMotion == NULL){
			MLeftLowerArmMotion = new LimbMotion;
		}

		if (MSpineMotion == NULL){
			MSpineMotion = new LimbMotion;
		}

		if (MRightFemurMotion == NULL){
			MRightFemurMotion = new LimbMotion;
		}

		if (MRightTibiaMotion == NULL){
			MRightTibiaMotion = new LimbMotion;
		}

		if (mainWinPtr->trajectoryCheckBox->isChecked() && MFemurMotion != NULL)
			MFemurMotion->LimbMotionProps.plotTrajectory(true);
		else
			MFemurMotion->LimbMotionProps.plotTrajectory(false);


		if (mainWinPtr->plotObjectCheckBox->isChecked() && MFemurMotion != NULL)
			MFemurMotion->LimbMotionProps.plotObject(true);
		else
			MFemurMotion->LimbMotionProps.plotObject(false);


		if (mainWinPtr->trajectoryCheckBox->isChecked() && MTibiaMotion != NULL)
			MTibiaMotion->LimbMotionProps.plotTrajectory(true);
		else
			MTibiaMotion->LimbMotionProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && MTibiaMotion != NULL)
			MTibiaMotion->LimbMotionProps.plotObject(true);
		else
			MTibiaMotion->LimbMotionProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && MLeftUpperArmMotion != NULL)
			MLeftUpperArmMotion->LimbMotionProps.plotTrajectory(true);
		else
			MLeftUpperArmMotion->LimbMotionProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && MLeftUpperArmMotion != NULL)
			MLeftUpperArmMotion->LimbMotionProps.plotObject(true);
		else
			MLeftUpperArmMotion->LimbMotionProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && MLeftLowerArmMotion != NULL)
			MLeftLowerArmMotion->LimbMotionProps.plotTrajectory(true);
		else
			MLeftLowerArmMotion->LimbMotionProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && MLeftLowerArmMotion != NULL)
			MLeftLowerArmMotion->LimbMotionProps.plotObject(true);
		else
			MLeftLowerArmMotion->LimbMotionProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && MSpineMotion != NULL)
			MSpineMotion->LimbMotionProps.plotTrajectory(true);
		else
			MSpineMotion->LimbMotionProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && MSpineMotion != NULL)
			MSpineMotion->LimbMotionProps.plotObject(true);
		else
			MSpineMotion->LimbMotionProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && MRightFemurMotion != NULL)
			MRightFemurMotion->LimbMotionProps.plotTrajectory(true);
		else
			MRightFemurMotion->LimbMotionProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && MRightFemurMotion != NULL)
			MRightFemurMotion->LimbMotionProps.plotObject(true);
		else
			MRightFemurMotion->LimbMotionProps.plotObject(false);

		if (mainWinPtr->trajectoryCheckBox->isChecked() && MRightTibiaMotion != NULL)
			MRightTibiaMotion->LimbMotionProps.plotTrajectory(true);
		else
			MRightTibiaMotion->LimbMotionProps.plotTrajectory(false);

		if (mainWinPtr->plotObjectCheckBox->isChecked() && MRightTibiaMotion != NULL)
			MRightTibiaMotion->LimbMotionProps.plotObject(true);
		else
			MRightTibiaMotion->LimbMotionProps.plotObject(false);

	}
	else
	{
		Motion::LIMBMOTION = false;
		if (MFemurMotion != NULL){
			delete MFemurMotion;
			MFemurMotion = NULL;
		}
		if (MTibiaMotion != NULL){
			delete MTibiaMotion;
			MTibiaMotion = NULL; 
		}

		if (MLeftUpperArmMotion != NULL){
			delete MLeftUpperArmMotion;
			MLeftUpperArmMotion = NULL;
		}

		if (MLeftLowerArmMotion != NULL){
			delete MLeftLowerArmMotion;
			MLeftLowerArmMotion = NULL;
		}

		if (MSpineMotion != NULL){
			delete MSpineMotion;
			MSpineMotion = NULL;
		}

		if (MRightFemurMotion != NULL){
			delete MRightFemurMotion;
			MRightFemurMotion = NULL;
		}

		if (MRightTibiaMotion != NULL){
			delete MRightTibiaMotion;
			MRightTibiaMotion = NULL;
		}
	}

	update();
}

void GLWidget::updateApproximationDegree(int val)
{
	if (mainWinPtr->LimbMotionCapture->isChecked())
	{
		if (val < mainWinPtr->kinectSensor->getKeyFemurPositions().size())
			if (mainWinPtr->bsplineApproximation->isChecked())
				if (Motion::RBSPLINEMOTION == true && RBSplineFemurMotion != NULL)
					if (RBSplineFemurMotion->getDegree() != val)
						RBSplineFemurMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeyTibiaPositions().size())
			if (mainWinPtr->bsplineApproximation->isChecked())
				if (Motion::RBSPLINEMOTION == true && RBSplineTibiaMotion != NULL)
					if (RBSplineTibiaMotion->getDegree() != val)
						RBSplineTibiaMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeySpinePositions().size())
			if (mainWinPtr->bsplineApproximation->isChecked())
				if (Motion::RBSPLINEMOTION == true && RBSPlineSpineMotion != NULL)
					if (RBSPlineSpineMotion->getDegree() != val)
						RBSPlineSpineMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeyRightFemurPositions().size())
			if (mainWinPtr->bsplineApproximation->isChecked())
				if (Motion::RBSPLINEMOTION == true && RBSplineRightFemurMotion != NULL)
					if (RBSplineRightFemurMotion->getDegree() != val)
						RBSplineRightFemurMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeyRightTibiaPositions().size())
			if (mainWinPtr->bsplineApproximation->isChecked())
				if (Motion::RBSPLINEMOTION == true && RBSplineRightTibiaMotion != NULL)
					if (RBSplineRightTibiaMotion->getDegree() != val)
						RBSplineRightTibiaMotion->setDegree(val);
	}
	else
	{
		if (val < mainWinPtr->kinectSensor->getKeyPositions().size())
			if (mainWinPtr->bsplineApproximation->isChecked())
				if (Motion::RBSPLINEMOTION == true && RBSplineMotion != NULL)
					if (RBSplineMotion->getDegree() != val)
						RBSplineMotion->setDegree(val);
	}
	update();
}

void GLWidget::updateInterpolationDegree(int val)
{
	if (mainWinPtr->LimbMotionCapture->isChecked())
	{
		if (val < mainWinPtr->kinectSensor->getKeyFemurPositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolationFemurMotion != NULL)
					if (RBSplineInterpolationFemurMotion->getDegree() != val)
						RBSplineInterpolationFemurMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeyTibiaPositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolationTibiaMotion != NULL)
					if (RBSplineInterpolationTibiaMotion->getDegree() != val)
						RBSplineInterpolationTibiaMotion->setDegree(val);
		
		if (val < mainWinPtr->kinectSensor->getKeyLeftUpperArmPositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolationLeftUpperArmMotion != NULL)
					if (RBSplineInterpolationLeftUpperArmMotion->getDegree() != val)
						RBSplineInterpolationLeftUpperArmMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeyLeftLowerArmPositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolationLeftLowerArmMotion != NULL)
					if (RBSplineInterpolationLeftLowerArmMotion->getDegree() != val)
						RBSplineInterpolationLeftLowerArmMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeySpinePositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolationSpineMotion != NULL)
					if (RBSplineInterpolationSpineMotion->getDegree() != val)
						RBSplineInterpolationSpineMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeyRightFemurPositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolationRightFemurMotion != NULL)
					if (RBSplineInterpolationRightFemurMotion->getDegree() != val)
						RBSplineInterpolationRightFemurMotion->setDegree(val);

		if (val < mainWinPtr->kinectSensor->getKeyRightTibiaPositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolationRightTibiaMotion != NULL)
					if (RBSplineInterpolationRightTibiaMotion->getDegree() != val)
						RBSplineInterpolationRightTibiaMotion->setDegree(val);
	}
	else
	{
		if (val < mainWinPtr->kinectSensor->getKeyPositions().size())
			if (mainWinPtr->bsplineInterpolation->isChecked())
				if (Motion::RBSPLINEINTERPOLATION == true && RBSplineInterpolation != NULL)
					if (RBSplineInterpolation->getDegree() != val)
						RBSplineInterpolation->setDegree(val);
	}
	update();
}

void GLWidget::setupLight()
{
	float LightAmbient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	float LightDiffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	float LightSpecular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	float LightPosition[] = { 3.0f, 3.0f, 3.0f, 1.0f };

	float RedSurface[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	float GreenSurface[] = { 0.0f, 1.0f, 0.0f, 1.0f };
	float BlueSurface[] = { 0.0f, 0.0f, 1.0f, 1.0f };

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel(GL_SMOOTH);

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	float no_mat[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float mat_ambient[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	float mat_ambient_color[] = { 0.8f, 0.3f, 0.7f, 1.0f };
	float mat_diffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };

	float mat_specular[] = { 0.7f, 0.7f, 0.7f, 1.0f };

	float no_shininess[] = { 0.0 };
	float low_shininess[] = { 5.0 };
	float high_shininess[] = { 100.0 };
	float mat_emission[] = { 0.2f, 0.1f, 0.1f, 0.0f };

	glMaterialfv(GL_FRONT, GL_AMBIENT, no_mat);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);
	glMaterialfv(GL_FRONT, GL_EMISSION, mat_emission);
}

void GLWidget::clearKeyGLWidget()
{
	if (mainWinPtr->screwMotion->isChecked())
	{
		Motion::SCREWMOTION = false;
		if (RScrewMotion != NULL){
			delete RScrewMotion;
			RScrewMotion = NULL;
		}

		if (RScrewFemurMotion != NULL){
			delete RScrewFemurMotion;
			RScrewFemurMotion = NULL;
		}
		if (RScrewTibiaMotion != NULL){
			delete RScrewTibiaMotion;
			RScrewTibiaMotion = NULL;
		}

		if (RScrewSpineMotion != NULL){
			delete RScrewSpineMotion;
			RScrewSpineMotion = NULL;
		}

		if (RScrewRightFemurMotion != NULL){
			delete RScrewRightFemurMotion;
			RScrewRightFemurMotion = NULL;
		}
		if (RScrewRightTibiaMotion != NULL){
			delete RScrewRightTibiaMotion;
			RScrewRightTibiaMotion = NULL;
		}
		mainWinPtr->screwMotion->setChecked(false);
	}

	if (mainWinPtr->bezierMotion->isChecked())
	{
		Motion::BEZIERMOTION = false;
		if (RBezierMotion != NULL){
			delete RBezierMotion;
			RBezierMotion = NULL;
		}
		if (RBezierFemurMotion != NULL){
			delete RBezierFemurMotion;
			RBezierFemurMotion = NULL;
		}
		if (RBezierTibiaMotion != NULL){
			delete RBezierTibiaMotion;
			RBezierTibiaMotion = NULL;
		}

		if (RBezierSpineMotion != NULL){
			delete RBezierSpineMotion;
			RBezierSpineMotion = NULL;
		}
		if (RBezierRightFemurMotion != NULL){
			delete RBezierRightFemurMotion;
			RBezierRightFemurMotion = NULL;
		}
		if (RBezierRightTibiaMotion != NULL){
			delete RBezierRightTibiaMotion;
			RBezierRightTibiaMotion = NULL;
		}
		mainWinPtr->bezierMotion->setChecked(false);
	}

	if (mainWinPtr->bsplineApproximation->isChecked())
	{
		Motion::RBSPLINEMOTION = false;
		if (RBSplineMotion != NULL){
			delete RBSplineMotion;
			RBSplineMotion = NULL;
		}
		if (RBSplineFemurMotion != NULL){
			delete RBSplineFemurMotion;
			RBSplineFemurMotion = NULL;
		}
		if (RBSplineTibiaMotion != NULL){
			delete RBSplineTibiaMotion;
			RBSplineTibiaMotion = NULL;
		}
		if (RBSPlineSpineMotion != NULL){
			delete RBSPlineSpineMotion;
			RBSPlineSpineMotion = NULL;
		}

		if (RBSplineRightFemurMotion != NULL){
			delete RBSplineRightFemurMotion;
			RBSplineRightFemurMotion = NULL;
		}
		if (RBSplineRightTibiaMotion != NULL){
			delete RBSplineRightTibiaMotion;
			RBSplineRightTibiaMotion = NULL;
		}
		mainWinPtr->bsplineApproximation->setChecked(false);
	}

	if (mainWinPtr->bsplineInterpolation->isChecked())
	{
		Motion::RBSPLINEINTERPOLATION = false;
		if (RBSplineInterpolation != NULL){
			delete RBSplineInterpolation;
			RBSplineInterpolation = NULL;
		}
		if (RBSplineInterpolationFemurMotion != NULL){
			delete RBSplineInterpolationFemurMotion;
			RBSplineInterpolationFemurMotion = NULL;
		}
		if (RBSplineInterpolationTibiaMotion != NULL){
			delete RBSplineInterpolationTibiaMotion;
			RBSplineInterpolationTibiaMotion = NULL;
		}
		if (RBSplineInterpolationLeftUpperArmMotion != NULL){
			delete RBSplineInterpolationLeftUpperArmMotion;
			RBSplineInterpolationLeftUpperArmMotion = NULL;
		}

		if (RBSplineInterpolationLeftLowerArmMotion != NULL){
			delete RBSplineInterpolationLeftLowerArmMotion;
			RBSplineInterpolationLeftLowerArmMotion = NULL;
		}

		if (RBSplineInterpolationSpineMotion != NULL){
			delete RBSplineInterpolationSpineMotion;
			RBSplineInterpolationSpineMotion = NULL;
		}

		if (RBSplineInterpolationRightFemurMotion != NULL){
			delete RBSplineInterpolationRightFemurMotion;
			RBSplineInterpolationRightFemurMotion = NULL;
		}

		if (RBSplineInterpolationRightTibiaMotion != NULL){
			delete RBSplineInterpolationRightTibiaMotion;
			RBSplineInterpolationRightTibiaMotion = NULL;
		}

		if (RBSplineLeftHipInterpolation != NULL){
			delete RBSplineLeftHipInterpolation;
			RBSplineLeftHipInterpolation = NULL;
		}
		if (RBSplineLeftShoulderInterpolation != NULL){
			delete RBSplineLeftShoulderInterpolation;
			RBSplineLeftShoulderInterpolation = NULL;
		}
		if (RBSplineLeftAnkleInterpolation != NULL){
			delete RBSplineLeftAnkleInterpolation;
			RBSplineLeftAnkleInterpolation = NULL;
		}
		if (RBSplineRightAnkleInterpolation != NULL){
			delete RBSplineRightAnkleInterpolation;
			RBSplineRightAnkleInterpolation = NULL;
		}
		if (RBSplineLeftElbowInterpolation != NULL){
			delete RBSplineLeftElbowInterpolation;
			RBSplineLeftElbowInterpolation = NULL;
		}
		/*if (RBSplineRightElbowInterpolation != NULL){
			delete RBSplineRightElbowInterpolation;
			RBSplineRightElbowInterpolation = NULL;
		}*/

		mainWinPtr->bsplineInterpolation->setChecked(false);
	}

	if (mainWinPtr->keyPositions->isChecked())
	{
		mainWinPtr->keyPositions->setChecked(false);
	}
	m_xRotate = 0.0; m_yRotate = 0.0; m_zRotate = 0.0;
	m_xTrans = 0.0; m_yTrans = 0.0; m_zTrans = 0.0;
	update();
}

void GLWidget::clearContinuousGLWidget()
{
	if (mainWinPtr->continuousMotionCheckBox->isChecked())
	{
		Motion::RCONTINUOUSMOTION = false;
		if (ContinuousMotion != NULL){
			delete ContinuousMotion;
			ContinuousMotion = NULL;
		}
		if (ContinuousLeftHipMotion != NULL){
			delete ContinuousLeftHipMotion;
			ContinuousLeftHipMotion = NULL;
		}
		if (ContinuousLeftShoulderMotion != NULL){
			delete ContinuousLeftShoulderMotion;
			ContinuousLeftShoulderMotion = NULL;
		}
		if (ContinuousLeftAnkleMotion != NULL){
			delete ContinuousLeftAnkleMotion;
			ContinuousLeftAnkleMotion = NULL;
		}
		if (ContinuousRightAnkleMotion != NULL){
			delete ContinuousRightAnkleMotion;
			ContinuousRightAnkleMotion = NULL;
		}
		if (ContinuousLeftElbowMotion != NULL){
			delete ContinuousLeftElbowMotion;
			ContinuousLeftElbowMotion = NULL;
		}
		/*if (ContinuousRightElbowMotion != NULL){
			delete ContinuousRightElbowMotion;
			ContinuousRightElbowMotion = NULL;
		}*/
		mainWinPtr->continuousMotionCheckBox->setChecked(false);
	}

	if (mainWinPtr->limbMotionCheckBox->isChecked())
	{
		Motion::LIMBMOTION = false;
		if (MFemurMotion != NULL)
		{
			delete MFemurMotion;
			MFemurMotion = NULL;
		}

		if (MTibiaMotion != NULL)
		{
			delete MTibiaMotion;
			MTibiaMotion = NULL;
		}

		if (MLeftUpperArmMotion != NULL)
		{
			delete MLeftUpperArmMotion;
			MLeftUpperArmMotion = NULL;
		}

		if (MLeftLowerArmMotion != NULL)
		{
			delete MLeftLowerArmMotion;
			MLeftLowerArmMotion = NULL;
		}

		if (MSpineMotion != NULL){
			delete MSpineMotion;
			MSpineMotion = NULL;
		}
		if (MRightFemurMotion != NULL){
			delete MRightFemurMotion;
			MRightFemurMotion = NULL;
		}

		if (MRightTibiaMotion != NULL){
			delete MRightTibiaMotion;
			MRightTibiaMotion = NULL;
		}
	}

	m_xRotate = 0.0; m_yRotate = 0.0; m_zRotate = 0.0;
	m_xTrans = 0.0; m_yTrans = 0.0; m_zTrans = 0.0;
	update();
}

void GLWidget::resetInteraction()
{
	m_xRotate = 0.0; m_yRotate = 0.0; m_zRotate = 0.0;
	m_xTrans = 0.0; m_yTrans = 0.0; m_zTrans = 0.0;
	update();
}
