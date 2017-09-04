#include <iostream>
#include <fstream>
#include <vector>
#include <QtCore/QDebug>
//#include "Dependencies/64 bit/Include/GLU/GLU.h"
#include <GLU.h>

#include "maindialog.h"
#include "motion.h"
#include "Math/Quaternion.h"
//#include "hMatrix.h"
#include "Math/hPoint.h"
#include "UI_color.h"

extern Kinect::mainDialog* mainWinPtr;

bool Motion::PLOTCONTROLPOSITIONS = false;
bool Motion::SCREWMOTION = false;
bool Motion::BEZIERMOTION = false;
bool Motion::RBSPLINEMOTION = false;
bool Motion::RBSPLINEINTERPOLATION = false;
bool Motion::RCONTINUOUSMOTION = false;
bool Motion::LIMBMOTION = false;

void Motion::drawCylinder(GLUquadricObj* qobj,float len)
{
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, YELLOW);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	gluCylinder(qobj, 0.1, 0.1, len*10.0f, 20, 20);
	glPopMatrix();
}

void Motion::drawCoordinateFrame(GLUquadricObj* qobj)
{
	
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MAGENTA);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);      
	gluCylinder(qobj, 0.1, 0.1, 2.0f, 20, 20);
	glPopMatrix();

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, GREEN);
	glPushMatrix();
	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	gluCylinder(qobj, 0.1, 0.1, 2.0f, 20, 20);
	glPopMatrix();

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BLUE);
	glPushMatrix();
	gluCylinder(qobj, 0.1, 0.1, 2.0f, 20, 20);
	glPopMatrix();
}
void Motion::drawSelectedCoordinateFrame(GLUquadricObj* qobj)
{
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	gluCylinder(qobj, 0.1, 0.1, 2.0f, 20, 20);
	glPopMatrix();

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE);
	glPushMatrix();
	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	gluCylinder(qobj, 0.1, 0.1, 2.0f, 20, 20);
	glPopMatrix();

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE);
	glPushMatrix();
	gluCylinder(qobj, 0.1, 0.1, 2.0f, 20, 20);
	glPopMatrix();

}

void Motion::drawWrist(int displayMode)
{
	static struct {
		int num;
		float n[3];
		float v[10][3];
	} polygon[30];
	static int first = 0;
	FILE *fp;
	static int number_of_polygons;
	int i, j;
	static float scalex, scaley, scalez;
	char string[20];
	static int cylinder_on;

	if (first == 0) {
		fopen_s(&fp, "Object/wrist_smallsize.dat", "r");

		if (!fp)
		{
			printf("\n File to read geometry of Robotic End Effector not found");
			return;
		}
		i = 0;
		fscanf_s(fp, "%s %f %f %f", string, sizeof string, &scalex, &scaley, &scalez);
		fscanf_s(fp, "%s %d", string, sizeof string, &cylinder_on);
		while (1) {
			if (fscanf_s(fp, "%s %d", string, sizeof string, &(polygon[i].num)) != EOF) {
				fscanf_s(fp, "%s %f %f %f", string, sizeof string, &(polygon[i].n[0]), &(polygon[i].n[1]), &(polygon[i].n[2]));
				for (j = 0; j < polygon[i].num; j++) {
					fscanf_s(fp, "%s %f %f %f", string, sizeof string, &(polygon[i].v[j][0]), &(polygon[i].v[j][1]), &(polygon[i].v[j][2]));
				}
				i++;
			}
			else {
				number_of_polygons = i;
				first = 1;
				fclose(fp);
				break;
			}
		}
	}

	GLUquadricObj *qobj;
	qobj = gluNewQuadric();
	gluQuadricNormals(qobj, GLU_SMOOTH);

	if (displayMode == 1)
	{
		glDisable(GL_LIGHTING);
		gluQuadricDrawStyle(qobj, GLU_LINE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glEnable(GL_LIGHTING);
	}
	else if (displayMode == 0)
	{
		gluQuadricDrawStyle(qobj, GLU_FILL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	glPushMatrix();

	glScalef(0.5, 0.5, 0.5);
	glTranslatef(0.0f, 0.0f, -1.125f);

	glTranslatef(-1.0, -0.5, 0.0f);
	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	

	for (i = 0; i < number_of_polygons; i++) {
		glBegin(GL_POLYGON);
		glNormal3fv(polygon[i].n);
		for (j = 0; j < polygon[i].num; j++)
		{
			glEdgeFlag(GL_TRUE);
			glVertex3fv(polygon[i].v[j]);
		}
		glEnd();
	}
	glRotatef(90.0, 0, 1, 0);
	glTranslated(0.0, 0.5, 1.0);
	glRotatef(90.0, 0, 1, 0);
	if (cylinder_on)
	{
		gluCylinder(qobj, 0.25, 0.25, 1.0, 20, 20);

		glPushMatrix();
		glRotated(180.0f, 0.0f, 1.0f, 0.0f);
		gluCylinder(qobj, 0.25, 0.0, 1e-5, 20, 20);
		glPopMatrix();

		glPushMatrix();
		glTranslated(0.0f, 0.0f, 1.0);
		gluCylinder(qobj, 0.25, 0.0, 1e-5, 20, 20);
		glPopMatrix();
	}
	glPopMatrix();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}




Motion::Motion()
{
	ctrlPositions = new ControlPosition;
	limbPoses = new LimbPosition;
}
int Motion::getSpan(int n, int p, float u, vector<double>U)
{
	if (u == U[n + 1])
		return(n);

	int low, high, mid;
	low = p;
	high = n + 1;
	mid = (low + high) / 2;

	for (; u<U[mid] || u >= U[mid + 1];)
	{
		if (u<U[mid])
		{
			high = mid;
			mid = (low + high) / 2;
		}
		else

		{
			low = mid;
			mid = (low + high) / 2;
		}
	}
	return(mid);
}
vector<float> Motion::getBasisFunction(int i, double u, int p, vector<double> U)
{
	//Note: From P2.2 of NURBS book, in any knot span [u|j, u|j+1) at most p+1 of the N|i,p are non zero
	//namely the functions N|j-p, p,...N|jp. Hence the following vector size. {"|" symbol indicates subscript.
	vector<float> N(p + 1);
	vector<float> left(p + 1);
	vector<float> right(p + 1);
	float saved, temp;
	N[0] = 1.0;
	int j;

	for (j = 1; j <= p; j++)
	{
		left[j] = u - U[i + 1 - j];
		right[j] = U[i + j] - u;
		saved = 0.0;
		for (int r = 0; r<j; r++)
		{
			temp = N[r] / (right[r + 1] + left[j - r]);
			N[r] = saved + right[r + 1] * temp;
			saved = left[j - r] * temp;
		}
		N[j] = saved;
	}
	return(N);
}
void Motion::setCtrlPos(vector<DualQuaternion> controlPosition)
{
	ctrlPositions->setCtrlPos(controlPosition);
}
vector<DualQuaternion> Motion::getCtrlPos()
{ 
	return (ctrlPositions->getCtrlPosInDualQuat()); 
}
int Motion::getNoOfCtrlPos()
{
	return ctrlPositions->getNoOfCtrlPos();
}
void Motion::plotCtrlPositions(float(&color)[3])
{
	vector<DualQuaternion> pos = getCtrlPos();
	for (auto i = 0; i < ctrlPositions->getNoOfCtrlPos(); i++)
	{
		hMatrix transformationMat;
		transformationMat = pos.at(i).dualQuaternionToHomogeneousMatrix().transpose();
		double MatrixForOpenGLStack[16];
		for (auto i1 = 0; i1 < 4; i1++)
			for (auto i2 = 0; i2 < 4; i2++)
				MatrixForOpenGLStack[4 * i1 + i2] = transformationMat.m[i1][i2];


		MatrixForOpenGLStack[12] *= 10.0f;
		MatrixForOpenGLStack[13] *= 10.0f;
		MatrixForOpenGLStack[14] *= 10.0f;

		glPushMatrix();
		glMultMatrixd(MatrixForOpenGLStack);

		//glPushMatrix();
		GLUquadricObj *qobj;
		qobj = gluNewQuadric();
		gluQuadricNormals(qobj, GLU_SMOOTH);
		gluQuadricDrawStyle(qobj, GLU_FILL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPushMatrix();
		glScalef(0.5, 0.5, 0.5);
		glTranslatef(0.0f, 0.0f, -1.125f);
		drawCoordinateFrame(qobj);
		glPopMatrix();

		glMaterialfv(GL_FRONT, GL_DIFFUSE, color);

		drawWrist(0);
		glPopMatrix();
	}
}
void Motion::setLimbPoses(vector<DualQuaternion> limbPositions)
{
	limbPoses->setLimbPos(limbPositions);
}
void Motion::setLimbPoses(vector<DualQuaternion> limbPosition, vector<float> lengths)
{
	limbPoses->setLimbPos(limbPosition);
	limbPoses->setLimbLength(lengths);
}
vector<DualQuaternion> Motion::getLimbPoses()
{
	return limbPoses->getLimbPositions();
}
vector<float> Motion::getLimbLengths()
{
	return limbPoses->getLimbLength();
}
int Motion::getNoOfLimbPoses()
{
	return limbPoses->getNoOfLimbPos();
}

void Motion::plotKeyLimbPoses(float(&color)[3])
{
	vector<DualQuaternion> poses = getLimbPoses();
	int numberOfPoses = getNoOfLimbPoses();

	for (auto i = 0; i < numberOfPoses; i++)
	{
		hMatrix transformationMatrix;
		transformationMatrix = poses.at(i).dualQuaternionToHomogeneousMatrix().transpose();
		double MatrixForOpenGLStack[16];
		for (auto i1 = 0; i1 < 4; i1++)
			for (auto i2 = 0; i2 < 4; i2++)
				MatrixForOpenGLStack[4 * i1 + i2] = transformationMatrix.m[i1][i2];

		MatrixForOpenGLStack[12] *= 10.0f;
		MatrixForOpenGLStack[13] *= 10.0f;
		MatrixForOpenGLStack[14] *= 10.0f;

		glPushMatrix();
		glMultMatrixd(MatrixForOpenGLStack);

		GLUquadricObj *qobj;
		qobj = gluNewQuadric();
		gluQuadricNormals(qobj, GLU_SMOOTH);
		gluQuadricDrawStyle(qobj, GLU_FILL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		//glPushMatrix();
		//glScalef(0.5, 0.5, 0.5);
		//glTranslatef(0.0f, 0.0f, -1.125f);
		//drawCoordinateFrame(qobj);
		//glPopMatrix();
		glMaterialfv(GL_FRONT, GL_DIFFUSE, color);

		if (mainWinPtr->planarCapture->isChecked())
		{
			glRotated(90.0f, 0.0f, 1.0f, 0.0f);
			gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
			//Motion::drawCylinder(qobj, 0.3f);
		}
		else
		{
			gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
		}
		
		glPopMatrix();
	}

}
void Motion::plotKeySpinePoses(float(&color)[3])
{
	vector<DualQuaternion> poses = getLimbPoses();
	int numberOfPoses = getNoOfLimbPoses();

	for (auto i = 0; i < numberOfPoses; i++)
	{
		hMatrix transformationMatrix;
		transformationMatrix = poses.at(i).dualQuaternionToHomogeneousMatrix().transpose();
		double MatrixForOpenGLStack[16];
		for (auto i1 = 0; i1 < 4; i1++)
			for (auto i2 = 0; i2 < 4; i2++)
				MatrixForOpenGLStack[4 * i1 + i2] = transformationMatrix.m[i1][i2];

		MatrixForOpenGLStack[12] *= 10.0f;
		MatrixForOpenGLStack[13] *= 10.0f;
		MatrixForOpenGLStack[14] *= 10.0f;

		glPushMatrix();
		glMultMatrixd(MatrixForOpenGLStack);

		GLUquadricObj *qobj;
		qobj = gluNewQuadric();
		gluQuadricNormals(qobj, GLU_SMOOTH);
		gluQuadricDrawStyle(qobj, GLU_FILL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		//glPushMatrix();
		//glScalef(0.5, 0.5, 0.5);
		//glTranslatef(0.0f, 0.0f, -1.125f);
		//drawCoordinateFrame(qobj);
		//glPopMatrix();
		glMaterialfv(GL_FRONT, GL_DIFFUSE, color);

		if (mainWinPtr->planarCapture->isChecked())
		{
			glRotated(90.0f, 0.0f, 1.0f, 0.0f);
			gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
			//Motion::drawCylinder(qobj, 0.3f);
		}
		else
		{
			gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
		}

		glPopMatrix();
	}
}
Motion::~Motion()
{
	if (ctrlPositions != NULL){
		delete ctrlPositions;
		ctrlPositions = NULL;
	}
	if (limbPoses != NULL){
		delete limbPoses;
		limbPoses = NULL;
	}
}


// None Motion
MNoneMotion::MNoneMotion()
{

}
MNoneMotion::~MNoneMotion()
{

}
void MNoneMotion::plotMotion()
{

}
void MNoneMotion::plotLimbMotion()
{

}
void MNoneMotion::plotSpineMotion()
{

}

// Continuous Motion 
MContinuousMotion::MContinuousMotion()
{

}
MContinuousMotion::~MContinuousMotion()
{

}
DualQuaternion MContinuousMotion::getLessDensePosition(int idx)
{
	return (lessDensePositions.at(idx));
}
vector<DualQuaternion> MContinuousMotion::getAllPositions()
{
	return (lessDensePositions);
}
void MContinuousMotion::plotMotion()
{

	lessDensePositions.clear();
	vector<DualQuaternion> positions;
	positions = getCtrlPos();
	int numberOfPositions = getNoOfCtrlPos();
	int positionCounter = 0;
	if (positions.size() > 0)
	{
		int totalInterPos; int increment;


		totalInterPos = static_cast<int> (((static_cast<float>(ContinuousProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions))));
		if (totalInterPos > 0)
		{
			increment = numberOfPositions / totalInterPos;
		}
		else
		{
			increment = numberOfPositions;
		}

		for (auto i = 0; i < positions.size(); i += increment)
		{
			lessDensePositions.push_back(positions.at(i));
			hMatrix transformationMat;
			double MatrixForOpenGL[16];
			transformationMat = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();
			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];


			MatrixForOpenGL[12] *= 10.0f;
			MatrixForOpenGL[13] *= 10.0f;
			MatrixForOpenGL[14] *= 10.0f;

			glPushMatrix();
			glMultMatrixd(MatrixForOpenGL);
			if (ContinuousProps.isPlotCoordinateFrame())
			{
				GLUquadricObj *qobj;
				qobj = gluNewQuadric();
				gluQuadricNormals(qobj, GLU_SMOOTH);
				gluQuadricDrawStyle(qobj, GLU_FILL);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glPushMatrix();
				glScalef(0.5, 0.5, 0.5);
				glTranslatef(0.0f, 0.0f, -1.125f);
				if (positionCounter == ContinuousProps.getSpecificPosition())
				{
					drawSelectedCoordinateFrame(qobj);
				}
				else
				{
					drawCoordinateFrame(qobj);
				}

				glPopMatrix();
			}

			if (positionCounter == ContinuousProps.getSpecificPosition())
			{
				glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
			}
			else
			{
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ContinuousProps.getObjectColor());
			}

			if (ContinuousProps.isPlotObject())
			{
				drawWrist(0);
			}

			glPopMatrix();
			positionCounter++;
		}

		if (ContinuousProps.isPlotTrajectory() /*&& positions.size() > 0*/)
		{
			vector<vector<double> > collectionOfTranslation;
			//qDebug() << "Checking Positions size" << positions.size();
			
			for (auto i = 0; i < positions.size(); i++)
			{
				hMatrix transformationMatrix;
				double MatrixForOpenGL[16];

				transformationMatrix = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						MatrixForOpenGL[4 * i1 + i2] = transformationMatrix.m[i1][i2];


				MatrixForOpenGL[12] *= 10.0f;
				MatrixForOpenGL[13] *= 10.0f;
				MatrixForOpenGL[14] *= 10.0f;

				vector<double> translationOfInterPos;
				translationOfInterPos.emplace_back(MatrixForOpenGL[12]);
				translationOfInterPos.emplace_back(MatrixForOpenGL[13]);
				translationOfInterPos.emplace_back(MatrixForOpenGL[14]);
				collectionOfTranslation.emplace_back(translationOfInterPos);

			}

			glDisable(GL_LIGHTING);
			glColor3fv(ContinuousProps.getTrajectoryColor());
			glLineWidth(2.0);
			glBegin(GL_LINES);

			for (auto i = 0; i < positions.size() - 1; i++)
			{
				glVertex3d(collectionOfTranslation.at(i).at(0), collectionOfTranslation.at(i).at(1), collectionOfTranslation.at(i).at(2));
				glVertex3d(collectionOfTranslation.at(i + 1).at(0), collectionOfTranslation.at(i + 1).at(1), collectionOfTranslation.at(i + 1).at(2));
			}
			glEnd();
			glEnable(GL_LIGHTING);
		}

		mainWinPtr->positionRecordingSlider->setRange(0, lessDensePositions.size() - 1);
	}
}
void MContinuousMotion::plotLimbMotion()
{

}
void MContinuousMotion::plotSpineMotion()
{

}
void MContinuousMotion::plotJointMotion()
{
	lessDensePositions.clear();
	vector<DualQuaternion> positions;
	positions = getCtrlPos();
	int numberOfPositions = getNoOfCtrlPos();
	int positionCounter = 0;
	if (positions.size() > 0)
	{
		int totalInterPos; int increment;


		totalInterPos = static_cast<int> (((static_cast<float>(ContinuousProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions))));
		if (totalInterPos > 0)
		{
			increment = numberOfPositions / totalInterPos;
		}
		else
		{
			increment = numberOfPositions;
		}

		for (auto i = 0; i < positions.size(); i += increment)
		{
			lessDensePositions.push_back(positions.at(i));
			hMatrix transformationMat;
			double MatrixForOpenGL[16];
			transformationMat = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();
			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];


			MatrixForOpenGL[12] *= 10.0f;
			MatrixForOpenGL[13] *= 10.0f;
			MatrixForOpenGL[14] *= 10.0f;

			glPushMatrix();
			glMultMatrixd(MatrixForOpenGL);
			if (ContinuousProps.isPlotCoordinateFrame())
			{
				GLUquadricObj *qobj;
				qobj = gluNewQuadric();
				gluQuadricNormals(qobj, GLU_SMOOTH);
				gluQuadricDrawStyle(qobj, GLU_FILL);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glPushMatrix();
				glScalef(0.5, 0.5, 0.5);
				glTranslatef(0.0f, 0.0f, -1.125f);
				if (positionCounter == ContinuousProps.getSpecificPosition())
				{
					drawSelectedCoordinateFrame(qobj);
				}
				else
				{
					drawCoordinateFrame(qobj);
				}

				glPopMatrix();
			}

			/*if (positionCounter == ContinuousProps.getSpecificPosition())
			{
				glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
			}
			else*/
			glMaterialfv(GL_FRONT, GL_DIFFUSE, ContinuousProps.getObjectColor());


			if (ContinuousProps.isPlotObject())
			{
				drawWrist(0);
			}

			glPopMatrix();
			positionCounter++;
		}

		//qDebug() << "Checking positions size before entering" << positions.size();
		if (ContinuousProps.isPlotTrajectory() /*&& positions.size() > 0*/)
		{
			vector<vector<double> > collectionOfTranslation;
			//qDebug() << "Checking Positions size" << positions.size();

			for (auto i = 0; i < positions.size(); i++)
			{
				hMatrix transformationMatrix;
				double MatrixForOpenGL[16];

				transformationMatrix = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						MatrixForOpenGL[4 * i1 + i2] = transformationMatrix.m[i1][i2];

				MatrixForOpenGL[12] *= 10.0f;
				MatrixForOpenGL[13] *= 10.0f;
				MatrixForOpenGL[14] *= 10.0f;
									
				vector<double> translationOfInterPos;
				translationOfInterPos.emplace_back(MatrixForOpenGL[12]);
				translationOfInterPos.emplace_back(MatrixForOpenGL[13]);
				translationOfInterPos.emplace_back(MatrixForOpenGL[14]);
				collectionOfTranslation.emplace_back(translationOfInterPos);

			}

			glDisable(GL_LIGHTING);
			glColor3fv(ContinuousProps.getTrajectoryColor());
			glLineWidth(2.0);
			glBegin(GL_LINES);

			for (auto i = 0; i < positions.size() - 1; i++)
			{
				glVertex3d(collectionOfTranslation.at(i).at(0), collectionOfTranslation.at(i).at(1), collectionOfTranslation.at(i).at(2));
				glVertex3d(collectionOfTranslation.at(i + 1).at(0), collectionOfTranslation.at(i + 1).at(1), collectionOfTranslation.at(i + 1).at(2));
			}


		

			vector<DualQuaternion> tempPositions = getCtrlPos();
			int numberOfPositions = getNoOfCtrlPos();
			hMatrix lastCtrlPosMat;
			double MatrixForOpenGLForLastCtrlPos[16];
			lastCtrlPosMat = tempPositions.at(numberOfPositions - 1).dualQuaternionToHomogeneousMatrix().transpose();
			double transOpenGLMat[16];
			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					transOpenGLMat[4 * i1 + i2] = lastCtrlPosMat.m[i1][i2];

			transOpenGLMat[12] *= 10.0f;
			transOpenGLMat[13] *= 10.0f;
			transOpenGLMat[14] *= 10.0f;

			glVertex3d(collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(0), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(1), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(2));
			glVertex3d(transOpenGLMat[12], transOpenGLMat[13], transOpenGLMat[14]);


			glEnd();
			glEnable(GL_LIGHTING);
		}

		mainWinPtr->positionRecordingSlider->setRange(0, lessDensePositions.size() - 1);
	}
}

// Limb Motion Capture
LimbMotion::LimbMotion(){
}
LimbMotion::~LimbMotion(){
	
}

void LimbMotion::drawLineSkeleton(vector<QVector3D> strt,vector<QVector3D> en)
{
	int numberOfPositions = getNoOfLimbPoses();

	int totalInterPos; int increment;

	totalInterPos = static_cast<int> (((static_cast<float>(LimbMotionProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions))));
	if (totalInterPos > 0)
	{
		increment = numberOfPositions / totalInterPos;
	}
	else
	{
		increment = numberOfPositions;
	}
	for (auto i = 0; i < strt.size(); i+=increment)
	{

		glPushMatrix();
		glColor3f(1.0f, 0.0f, 0.0f);
		glBegin(GL_LINES);
		glVertex3f(strt.at(i).x()*10.0f, strt.at(i).y()*10.0f, strt.at(i).z()*(10.0f));
		glVertex3f(en.at(i).x()*10.0f, en.at(i).y()*10.0f, en.at(i).z()*(10.0f));
		glEnd();
	}
		/*glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_LINES);
		glVertex3f(pos.at(1).x()*10.0f, pos.at(1).y()*10.0f, pos.at(1).z()*(-10.0f));
		glVertex3f(pos.at(2).x()*10.0f, pos.at(2).y()*10.0f, pos.at(2).z()*(-10.0f));
		glEnd();
		glPopMatrix();*/

}
void LimbMotion::plotLimbMotion()
{
	lessDenseLimbPoses.clear();
	vector<DualQuaternion> positions = getLimbPoses();
	int numberOfPositions = getNoOfLimbPoses();


	if (positions.size() > 0)
	{
		int totalInterPos; int increment;

		totalInterPos = static_cast<int> (((static_cast<float>(LimbMotionProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions))));
		if (totalInterPos > 0)
		{
			increment = numberOfPositions / totalInterPos;
		}
		else
		{
			increment = numberOfPositions;
		}

		for (auto i = 0; i < positions.size(); i += increment)
		{
			lessDenseLimbPoses.emplace_back(positions.at(i));
			hMatrix transformationMatrix;
			double MatrixForOpenGL[16];
			transformationMatrix = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();

			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					MatrixForOpenGL[4 * i1 + i2] = transformationMatrix.m[i1][i2];

			MatrixForOpenGL[12] *= 10.0f;
			MatrixForOpenGL[13] *= 10.0f;
			MatrixForOpenGL[14] *= 10.0f;

			
			glPushMatrix();
			glMultMatrixd(MatrixForOpenGL);
			if (LimbMotionProps.isPlotObject())
			{
				GLUquadricObj *qobj;
				qobj = gluNewQuadric();
				gluQuadricNormals(qobj, GLU_SMOOTH);
				gluQuadricDrawStyle(qobj, GLU_FILL);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, LimbMotionProps.getObjectColor());

				if (mainWinPtr->planarCapture->isChecked())
				{
					glRotated(90.0f, 0.0f, 1.0f, 0.0f);
					gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
					//Motion::drawCylinder(qobj, 0.3f);
				}
				else
				{
					gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
				}

			}

			glPopMatrix();
		}

		if (LimbMotionProps.isPlotTrajectory())
		{
			vector<vector<double> > collectionOfTranslationForStart;
			//vector<vector<double> > collectionOfTranslationForEnd;

			for (auto i = 0; i < positions.size(); i++)
			{
				hMatrix transformationMatrixForTranslation;
				//hMatrix transformationMatrixForEndTranslation;
				double MatrixForOpenGLForTranslation[16];

				transformationMatrixForTranslation = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						MatrixForOpenGLForTranslation[4 * i1 + i2] = transformationMatrixForTranslation.m[i1][i2];

				MatrixForOpenGLForTranslation[12] *= 10.0f;
				MatrixForOpenGLForTranslation[13] *= 10.0f;
				MatrixForOpenGLForTranslation[14] *= 10.0f;

				vector<double> startTranslation;
				startTranslation.emplace_back(MatrixForOpenGLForTranslation[12]);
				startTranslation.emplace_back(MatrixForOpenGLForTranslation[13]);
				startTranslation.emplace_back(MatrixForOpenGLForTranslation[14]);
				collectionOfTranslationForStart.emplace_back(startTranslation);
				startTranslation.clear();

			}

			glDisable(GL_LIGHTING);
			glColor3fv(LimbMotionProps.getTrajectoryColor());
			glLineWidth(2.0);
			glBegin(GL_LINES);

			for (auto i = 0; i < positions.size() - 1; i++)
			{
				glVertex3d(collectionOfTranslationForStart.at(i).at(0), collectionOfTranslationForStart.at(i).at(1), collectionOfTranslationForStart.at(i).at(2));
				glVertex3d(collectionOfTranslationForStart.at(i + 1).at(0), collectionOfTranslationForStart.at(i + 1).at(1), collectionOfTranslationForStart.at(i + 1).at(2));
			}
			glEnd();

			glEnable(GL_LIGHTING);
		}
	}
}

void LimbMotion::plotMotion()
{
	
}

void LimbMotion::plotSpineMotion()
{
	lessDenseLimbPoses.clear();
	vector<DualQuaternion> positions = getLimbPoses();
	//vector<float> lengths = getLimbLengths();
	//vector<float> limbLengths = getLimbLengths();
	int numberOfPositions = getNoOfLimbPoses();


	if (positions.size() > 0)
	{
		int totalInterPos; int increment;

		totalInterPos = static_cast<int> (((static_cast<float>(LimbMotionProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions))));
		if (totalInterPos > 0)
		{
			increment = numberOfPositions / totalInterPos;
		}
		else
		{
			increment = numberOfPositions;
		}

		for (auto i = 0; i < positions.size(); i += increment)
		{
			lessDenseLimbPoses.emplace_back(positions.at(i));
			hMatrix transformationMatrix;
			double MatrixForOpenGL[16];
			transformationMatrix = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();

			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					MatrixForOpenGL[4 * i1 + i2] = transformationMatrix.m[i1][i2];

			MatrixForOpenGL[12] *= 10.0f;
			MatrixForOpenGL[13] *= 10.0f;
			MatrixForOpenGL[14] *= 10.0f;

			//glPushMatrix();
			//glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
			glPushMatrix();

			//glTranslated(MatrixForOpenGL[12], MatrixForOpenGL[13], MatrixForOpenGL[14]);
			glMultMatrixd(MatrixForOpenGL);
			if (LimbMotionProps.isPlotObject())
			{
				GLUquadricObj *qobj;
				qobj = gluNewQuadric();
				gluQuadricNormals(qobj, GLU_SMOOTH);
				gluQuadricDrawStyle(qobj, GLU_FILL);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, LimbMotionProps.getObjectColor());

				if (mainWinPtr->planarCapture->isChecked())
				{
					glRotated(90.0f, 0.0f, 1.0f, 0.0f);
					gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
					//Motion::drawCylinder(qobj, 0.3f);
				}
				else
				{
					gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
				}

			}

			glPopMatrix();
		}

		if (LimbMotionProps.isPlotTrajectory())
		{
			vector<vector<double> > collectionOfTranslationForStart;
			//vector<vector<double> > collectionOfTranslationForEnd;

			for (auto i = 0; i < positions.size(); i++)
			{
				hMatrix transformationMatrixForTranslation;
				//hMatrix transformationMatrixForEndTranslation;
				double MatrixForOpenGLForTranslation[16];

				transformationMatrixForTranslation = positions.at(i).dualQuaternionToHomogeneousMatrix().transpose();
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						MatrixForOpenGLForTranslation[4 * i1 + i2] = transformationMatrixForTranslation.m[i1][i2];

				MatrixForOpenGLForTranslation[12] *= 10.0f;
				MatrixForOpenGLForTranslation[13] *= 10.0f;
				MatrixForOpenGLForTranslation[14] *= 10.0f;

				vector<double> startTranslation;
				startTranslation.emplace_back(MatrixForOpenGLForTranslation[12]);
				startTranslation.emplace_back(MatrixForOpenGLForTranslation[13]);
				startTranslation.emplace_back(MatrixForOpenGLForTranslation[14]);
				collectionOfTranslationForStart.emplace_back(startTranslation);
				startTranslation.clear();

			}


			glDisable(GL_LIGHTING);
			glColor3fv(LimbMotionProps.getTrajectoryColor());
			glLineWidth(2.0);
			glBegin(GL_LINES);

			for (auto i = 0; i < positions.size() - 1; i++)
			{
				glVertex3d(collectionOfTranslationForStart.at(i).at(0), collectionOfTranslationForStart.at(i).at(1), collectionOfTranslationForStart.at(i).at(2));
				glVertex3d(collectionOfTranslationForStart.at(i + 1).at(0), collectionOfTranslationForStart.at(i + 1).at(1), collectionOfTranslationForStart.at(i + 1).at(2));
			}
			glEnd();

			glEnable(GL_LIGHTING);
		}
	}
}


DualQuaternion LimbMotion::getLessDenseLimbPose(int idx)
{
	return lessDenseLimbPoses.at(idx);
}
vector<DualQuaternion> LimbMotion::getAllLimbPoses()
{
	return lessDenseLimbPoses;
}


// Screw Motion
MScrewMotion::MScrewMotion()
{
}
MScrewMotion::~MScrewMotion()
{
}
void MScrewMotion::plotMotion()
{
	int numberOfPositions = getNoOfCtrlPos();

	int positionCounter = 0;
	if (numberOfPositions > 1)
	{
		vector<DualQuaternion> positions;
		positions = getCtrlPos();
		positions.emplace_back(positions.at(0));

		
		float totalInterpos = ((static_cast<float>(ScrewProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
		float increment = 1/(totalInterpos/(numberOfPositions-1));
		
		vector<vector<double> > collectionOfTranslation;
		
		
		for (int i = 0; i < getNoOfCtrlPos()-1; i++)
		{
			for (float t = 0.00; t<= 1.0; t += increment)
			{
				hMatrix transformationMatScrew;
				DualQuaternion screwInterpolatedPosition;
				screwInterpolatedPosition = (1.0 - t)*positions.at(i) + t*positions.at(i + 1);
				transformationMatScrew = screwInterpolatedPosition.dualQuaternionToHomogeneousMatrix().transpose();

				double MatrixForOpenGLScrew[16];
				for (auto i1 = 0; i1 < 4; i1++){
					for (auto i2 = 0; i2 < 4; i2++){
						MatrixForOpenGLScrew[4 * i1 + i2] = transformationMatScrew.m[i1][i2];
					}
				}

				MatrixForOpenGLScrew[12] *= 10.0f;
				MatrixForOpenGLScrew[13] *= 10.0f;
				MatrixForOpenGLScrew[14] *= 10.0f;

				glPushMatrix();
				glMultMatrixd(MatrixForOpenGLScrew);

				if (ScrewProps.isPlotCoordinateFrame())
				{
					GLUquadricObj *qobj;
					qobj = gluNewQuadric();
					gluQuadricNormals(qobj, GLU_SMOOTH);
					gluQuadricDrawStyle(qobj, GLU_FILL);
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					glPushMatrix();
					glScalef(0.5, 0.5, 0.5);
					glTranslatef(0.0f, 0.0f, -1.125f);
					if (positionCounter == ScrewProps.getSpecificPosition())
					{
						drawSelectedCoordinateFrame(qobj);
					}
					else
					{
						drawCoordinateFrame(qobj);
					}
					
					glPopMatrix();
				}

				if (positionCounter == ScrewProps.getSpecificPosition()){
					glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
				}
				else
				{
					glMaterialfv(GL_FRONT, GL_DIFFUSE, ScrewProps.getObjectColor());
				}

				if (ScrewProps.isPlotObject())
				{
					glScalef(ScrewProps.getObjectSize(), ScrewProps.getObjectSize(), ScrewProps.getObjectSize());
					drawWrist(0);
				}

				glPopMatrix();
				
				positionCounter++;
			}
			
			for (float t = 0.00; t <= 1.0; t += 0.02)
			{
				hMatrix transformationMatTransScrew;
				DualQuaternion intermediatePosition;

				intermediatePosition = (1.0 - t)*positions.at(i) + t*positions.at(i + 1);
				transformationMatTransScrew = intermediatePosition.dualQuaternionToHomogeneousMatrix().transpose();

				double MatrixForOpenGLTrans[16];
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						MatrixForOpenGLTrans[4 * i1 + i2] = transformationMatTransScrew.m[i1][i2];
					
				
				MatrixForOpenGLTrans[12] *= 10.0f;
				MatrixForOpenGLTrans[13] *= 10.0f;
				MatrixForOpenGLTrans[14] *= 10.0f;

				vector<double> translationOfInterPos;
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[12]);
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[13]);
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[14]);
				collectionOfTranslation.emplace_back(translationOfInterPos);
				translationOfInterPos.clear();
			}
		}
		
		if (ScrewProps.isPlotTrajectory()) 
		{
			glDisable(GL_LIGHTING);
			glColor3fv(ScrewProps.getTrajectoryColor());
			glLineWidth(2.0);
			glBegin(GL_LINES);

			for (auto i = 0; i < collectionOfTranslation.size()- 1; i++)                              
			{
				glVertex3d(collectionOfTranslation.at(i).at(0), collectionOfTranslation.at(i).at(1), collectionOfTranslation.at(i).at(2));
				glVertex3d(collectionOfTranslation.at(i + 1).at(0), collectionOfTranslation.at(i + 1).at(1), collectionOfTranslation.at(i + 1).at(2));
			}

		

			vector<DualQuaternion> tempPositions = getCtrlPos();
			int numberOfPositions = getNoOfCtrlPos();
			hMatrix lastCtrlPosMat;
			double MatrixForOpenGLForLastCtrlPos[16];
			lastCtrlPosMat = tempPositions.at(numberOfPositions - 1).dualQuaternionToHomogeneousMatrix().transpose();
			double transOpenGLMat[16];
			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					transOpenGLMat[4 * i1 + i2] = lastCtrlPosMat.m[i1][i2];

			transOpenGLMat[12] *= 10.0f;
			transOpenGLMat[13] *= 10.0f;
			transOpenGLMat[14] *= 10.0f;

			glVertex3d(collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(0), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(1), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(2));
			glVertex3d(transOpenGLMat[12], transOpenGLMat[13], transOpenGLMat[14]);

			glEnd();
			glEnable(GL_LIGHTING);
		}
		
	}
}
void MScrewMotion::plotLimbMotion()
{
	int numberOfPositions = getNoOfLimbPoses();
	int positionCounter = 0;

	if (numberOfPositions > 1)
	{
		vector<DualQuaternion> positions;
		positions = getLimbPoses();
		positions.emplace_back(positions.at(0));

		float totalInterpos = ((static_cast<float>(ScrewProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
		float increment = 1 / (totalInterpos / (numberOfPositions - 1));

		vector<vector<double> > collectionOfTranslationStart;
		//vector<vector<double> > collectionOfTranslationEnd;

		for (int i = 0; i < getNoOfLimbPoses() - 1; i++)
		{
			for (float t = 0.00; t <= 1.0; t += increment)
			{
				hMatrix transformationMatScrew;
				DualQuaternion screwInterpolatedPosition;
				screwInterpolatedPosition = (1.0 - t)*positions.at(i) + t*positions.at(i + 1);
				transformationMatScrew = screwInterpolatedPosition.dualQuaternionToHomogeneousMatrix().transpose();

				double MatrixForOpenGLScrew[16];
				for (auto i1 = 0; i1 < 4; i1++){
					for (auto i2 = 0; i2 < 4; i2++){
						MatrixForOpenGLScrew[4 * i1 + i2] = transformationMatScrew.m[i1][i2];
					}
				}

				MatrixForOpenGLScrew[12] *= 10.0f;
				MatrixForOpenGLScrew[13] *= 10.0f;
				MatrixForOpenGLScrew[14] *= 10.0f;

				glPushMatrix();
				glMultMatrixd(MatrixForOpenGLScrew);


				if (ScrewProps.isPlotObject())
				{
					GLUquadricObj *qobj;
					qobj = gluNewQuadric();
					gluQuadricNormals(qobj, GLU_SMOOTH);
					gluQuadricDrawStyle(qobj, GLU_FILL);
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, ScrewProps.getObjectColor());
					
					if (mainWinPtr->planarCapture->isChecked())
					{
						glRotated(90.0f, 0.0f, 1.0f, 0.0f);
						gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
					}
					else
					{
						gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
					}
				}

				glPopMatrix();

				positionCounter++;
			}
			
			for (float t = 0.00; t <= 1.0; t += 0.02)
			{
				hMatrix transformationMatTransScrew;
				//hMatrix transformationMatTransScrewEnd;
				DualQuaternion intermediatePosition;

				intermediatePosition = (1.0 - t)*positions.at(i) + t*positions.at(i + 1);
				transformationMatTransScrew = intermediatePosition.dualQuaternionToHomogeneousMatrix().transpose();

				double MatrixForOpenGLTrans[16];
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						MatrixForOpenGLTrans[4 * i1 + i2] = transformationMatTransScrew.m[i1][i2];


				MatrixForOpenGLTrans[12] *= 10.0f;
				MatrixForOpenGLTrans[13] *= 10.0f;
				MatrixForOpenGLTrans[14] *= 10.0f;

				vector<double> translationOfInterPos;
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[12]);
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[13]);
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[14]);
				collectionOfTranslationStart.emplace_back(translationOfInterPos);
				translationOfInterPos.clear();

			}
		}

		if (ScrewProps.isPlotTrajectory())
		{
			glDisable(GL_LIGHTING);
			glColor3fv(ScrewProps.getTrajectoryColor());
			glLineWidth(2.0);
			glBegin(GL_LINES);

			for (auto i = 0; i < collectionOfTranslationStart.size() - 1; i++)
			{
				glVertex3d(collectionOfTranslationStart.at(i).at(0), collectionOfTranslationStart.at(i).at(1), collectionOfTranslationStart.at(i).at(2));
				glVertex3d(collectionOfTranslationStart.at(i + 1).at(0), collectionOfTranslationStart.at(i + 1).at(1), collectionOfTranslationStart.at(i + 1).at(2));
			}

			vector<DualQuaternion> tempPositions = getLimbPoses();
			int numberOfPositions = getNoOfLimbPoses();
			hMatrix lastCtrlPosMat;
			lastCtrlPosMat = tempPositions.at(numberOfPositions - 1).dualQuaternionToHomogeneousMatrix().transpose();
			double transOpenGLMat[16];
			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					transOpenGLMat[4 * i1 + i2] = lastCtrlPosMat.m[i1][i2];

			transOpenGLMat[12] *= 10.0f;
			transOpenGLMat[13] *= 10.0f;
			transOpenGLMat[14] *= 10.0f;

			glVertex3d(collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(0), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(1), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(2));
			glVertex3d(transOpenGLMat[12], transOpenGLMat[13], transOpenGLMat[14]);
			glEnd();

		
			glEnable(GL_LIGHTING);
		}
	}
}
void MScrewMotion::plotSpineMotion()
{
	int numberOfPositions = getNoOfLimbPoses();
	int positionCounter = 0;

	if (numberOfPositions > 1)
	{
		vector<DualQuaternion> positions;
		positions = getLimbPoses();
		positions.emplace_back(positions.at(0));

		float totalInterpos = ((static_cast<float>(ScrewProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
		float increment = 1 / (totalInterpos / (numberOfPositions - 1));

		vector<vector<double> > collectionOfTranslationStart;
		//vector<vector<double> > collectionOfTranslationEnd;

		for (int i = 0; i < getNoOfLimbPoses() - 1; i++)
		{
			for (float t = 0.00; t <= 1.0; t += increment)
			{
				hMatrix transformationMatScrew;
				DualQuaternion screwInterpolatedPosition;
				screwInterpolatedPosition = (1.0 - t)*positions.at(i) + t*positions.at(i + 1);
				transformationMatScrew = screwInterpolatedPosition.dualQuaternionToHomogeneousMatrix().transpose();

				double MatrixForOpenGLScrew[16];
				for (auto i1 = 0; i1 < 4; i1++){
					for (auto i2 = 0; i2 < 4; i2++){
						MatrixForOpenGLScrew[4 * i1 + i2] = transformationMatScrew.m[i1][i2];
					}
				}

				MatrixForOpenGLScrew[12] *= 10.0f;
				MatrixForOpenGLScrew[13] *= 10.0f;
				MatrixForOpenGLScrew[14] *= 10.0f;

				glPushMatrix();
				glMultMatrixd(MatrixForOpenGLScrew);

				if (ScrewProps.isPlotObject())
				{
					GLUquadricObj *qobj;
					qobj = gluNewQuadric();
					gluQuadricNormals(qobj, GLU_SMOOTH);
					gluQuadricDrawStyle(qobj, GLU_FILL);
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, ScrewProps.getObjectColor());

					if (mainWinPtr->planarCapture->isChecked())
					{
						glRotated(90.0f, 0.0f, 1.0f, 0.0f);
						gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
					}
					else
					{
						gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
					}
				}

				glPopMatrix();

				positionCounter++;
			}

			for (float t = 0.00; t <= 1.0; t += 0.02)
			{
				hMatrix transformationMatTransScrew;
				//hMatrix transformationMatTransScrewEnd;
				DualQuaternion intermediatePosition;

				intermediatePosition = (1.0 - t)*positions.at(i) + t*positions.at(i + 1);
				transformationMatTransScrew = intermediatePosition.dualQuaternionToHomogeneousMatrix().transpose();

				double MatrixForOpenGLTrans[16];
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						MatrixForOpenGLTrans[4 * i1 + i2] = transformationMatTransScrew.m[i1][i2];


				MatrixForOpenGLTrans[12] *= 10.0f;
				MatrixForOpenGLTrans[13] *= 10.0f;
				MatrixForOpenGLTrans[14] *= 10.0f;

				vector<double> translationOfInterPos;
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[12]);
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[13]);
				translationOfInterPos.emplace_back(MatrixForOpenGLTrans[14]);
				collectionOfTranslationStart.emplace_back(translationOfInterPos);
				translationOfInterPos.clear();

			}
		}

		if (ScrewProps.isPlotTrajectory())
		{
			glDisable(GL_LIGHTING);
			glColor3fv(ScrewProps.getTrajectoryColor());
			glLineWidth(2.0);
			glBegin(GL_LINES);

			for (auto i = 0; i < collectionOfTranslationStart.size() - 1; i++)
			{
				glVertex3d(collectionOfTranslationStart.at(i).at(0), collectionOfTranslationStart.at(i).at(1), collectionOfTranslationStart.at(i).at(2));
				glVertex3d(collectionOfTranslationStart.at(i + 1).at(0), collectionOfTranslationStart.at(i + 1).at(1), collectionOfTranslationStart.at(i + 1).at(2));
			}

			vector<DualQuaternion> tempPositions = getLimbPoses();
			int numberOfPositions = getNoOfLimbPoses();
			hMatrix lastCtrlPosMat;
			lastCtrlPosMat = tempPositions.at(numberOfPositions - 1).dualQuaternionToHomogeneousMatrix().transpose();
			double transOpenGLMat[16];
			for (auto i1 = 0; i1 < 4; i1++)
				for (auto i2 = 0; i2 < 4; i2++)
					transOpenGLMat[4 * i1 + i2] = lastCtrlPosMat.m[i1][i2];

			transOpenGLMat[12] *= 10.0f;
			transOpenGLMat[13] *= 10.0f;
			transOpenGLMat[14] *= 10.0f;

			glVertex3d(collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(0), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(1), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(2));
			glVertex3d(transOpenGLMat[12], transOpenGLMat[13], transOpenGLMat[14]);
			glEnd();

			glEnable(GL_LIGHTING);
		}
	}
}

// Bezier Motion
MBezierMotion::MBezierMotion()
{
}
MBezierMotion::~MBezierMotion()
{
}
vector<double> MBezierMotion::calculateBernsteinPoly(double t)
{
	int n;
	n = getNoOfCtrlPos() - 1;
	double t1 = 1.0 - t;

	vector<double> bernPoly;
	bernPoly.clear();

	for (int i = 0; i <= n; i++)
	{
		bernPoly.emplace_back(0.00);
	}
	bernPoly[0] = 1.0;

	double saved;
	double temp;

	for (int i = 1; i <= n; i++)
	{
		saved = 0.0;
		for (int j = 0; j<i; j++)
		{
			temp = bernPoly[j];
			bernPoly[j] = saved + t1*temp;
			saved = t*temp;
		}
		bernPoly[i] = saved;
	}
	return bernPoly;
}
vector<double> MBezierMotion::calculateBernsteinPolyForLimbData(double t)
{
	int n;
	n = getNoOfLimbPoses() - 1;
	double t1 = 1.0 - t;

	vector<double> bernPoly;
	bernPoly.clear();

	for (int i = 0; i <= n; i++)
	{
		bernPoly.emplace_back(0.00);
	}
	bernPoly[0] = 1.0;

	double saved;
	double temp;

	for (int i = 1; i <= n; i++)
	{
		saved = 0.0;
		for (int j = 0; j<i; j++)
		{
			temp = bernPoly[j];
			bernPoly[j] = saved + t1*temp;
			saved = t*temp;
		}
		bernPoly[i] = saved;
	}
	return bernPoly;
}
void MBezierMotion::plotMotion()
{
	int numberOfPositions = getNoOfCtrlPos();
	int positionCounter = 0;
	
	if (numberOfPositions > 1)
	{
		vector<DualQuaternion> positions;
		positions = getCtrlPos();

		if (positions.size() > 1)
		{
			vector<double> bernPoly;

			hMatrix transformationMat;
			int n;
			n = getNoOfCtrlPos() - 1;

			float totalInterpos = ((static_cast<float>(BezierProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
			float increment = 1 / (totalInterpos);
			vector<vector<double> > collectionOfTranslation;
			for (double t = 0.00; t <= 1.0; t += increment)
			{
				bernPoly.clear();
				bernPoly = calculateBernsteinPoly(t);
				DualQuaternion tempdQ;
				double MatrixForOpenGL[16];

				for (int i = 0; i <= n; i++)
				{
					tempdQ = tempdQ + (bernPoly.at(i))*positions.at(i);
				}
				transformationMat = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

				for (int i1 = 0; i1 < 4; i1++)
					for (int i2 = 0; i2 < 4; i2++)
						MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];

				MatrixForOpenGL[12] *= 10.0f;
				MatrixForOpenGL[13] *= 10.0f;
				MatrixForOpenGL[14] *= 10.0f;

				glPushMatrix();
				glMultMatrixd(MatrixForOpenGL);
				
				if (BezierProps.isPlotCoordinateFrame())
				{
					GLUquadricObj *qobj;
					qobj = gluNewQuadric();
					gluQuadricNormals(qobj, GLU_SMOOTH);
					gluQuadricDrawStyle(qobj, GLU_FILL);
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					glPushMatrix();
					glScalef(0.5, 0.5, 0.5);
					glTranslatef(0.0f, 0.0f, -1.125f);
					if (positionCounter == BezierProps.getSpecificPosition())
					{
						drawSelectedCoordinateFrame(qobj);
					}
					else
					{
						drawCoordinateFrame(qobj);
					}
					
					glPopMatrix();
				}

				if (positionCounter == BezierProps.getSpecificPosition())
				{
					glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
				}
				else
				{
					glMaterialfv(GL_FRONT, GL_DIFFUSE, BezierProps.getObjectColor());
				}

				if (BezierProps.isPlotObject())
				{
					glScalef(BezierProps.getObjectSize(), BezierProps.getObjectSize(), BezierProps.getObjectSize());
					drawWrist(0);
				}
				glPopMatrix();
				positionCounter++;
			}

			if (BezierProps.isPlotTrajectory())
			{
				
				for (double t = 0.00; t <= 1.0; t += 0.005)
				{
					bernPoly.clear();
					bernPoly = calculateBernsteinPoly(t);
					DualQuaternion tempdQTrans;
					hMatrix transformationMatTransBezier;
					double MatrixForOpenGLTransBezier[16];

					for (int i = 0; i <= n; i++)
					{
						tempdQTrans = tempdQTrans + ((bernPoly.at(i))*positions.at(i));
					}

					transformationMatTransBezier = tempdQTrans.dualQuaternionToHomogeneousMatrix().transpose();
					

					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							MatrixForOpenGLTransBezier[4 * i1 + i2] = transformationMatTransBezier.m[i1][i2];

					MatrixForOpenGLTransBezier[12] *= 10.0f;
					MatrixForOpenGLTransBezier[13] *= 10.0f;
					MatrixForOpenGLTransBezier[14] *= 10.0f;

					vector<double> translationOfInterPos;
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[12]);
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[13]);
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[14]);
					collectionOfTranslation.emplace_back(translationOfInterPos);
					translationOfInterPos.clear();
				}
				glDisable(GL_LIGHTING);
				glColor4fv(BezierProps.getTrajectoryColor());
				glLineWidth(2.0);
				glBegin(GL_LINES);

				for (int i = 0; i < collectionOfTranslation.size()-1; i++)                                  
				{
					glVertex3d(collectionOfTranslation.at(i).at(0), collectionOfTranslation.at(i).at(1), collectionOfTranslation.at(i).at(2));
					glVertex3d(collectionOfTranslation.at(i + 1).at(0), collectionOfTranslation.at(i + 1).at(1), collectionOfTranslation.at(i + 1).at(2));
				}
				vector<DualQuaternion> tempLastCtrlPos = getCtrlPos();
				int numberOfPositionsLast = getNoOfCtrlPos();
				hMatrix trMat;
				double MatrixForOpenGL[16];
				trMat = tempLastCtrlPos.at(numberOfPositionsLast - 1).dualQuaternionToHomogeneousMatrix().transpose();
				double transOpenGLMatLastTrans[16];
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						transOpenGLMatLastTrans[4 * i1 + i2] = trMat.m[i1][i2];

				transOpenGLMatLastTrans[12] *= 10.0f;
				transOpenGLMatLastTrans[13] *= 10.0f;
				transOpenGLMatLastTrans[14] *= 10.0f;

				glVertex3d(collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(0), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(1), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(2));
				glVertex3d(transOpenGLMatLastTrans[12], transOpenGLMatLastTrans[13], transOpenGLMatLastTrans[14]);
				glEnd();
				glEnable(GL_LIGHTING);
			}
		}
	}
}
void MBezierMotion::plotLimbMotion()
{
	int numberOfPositions = getNoOfLimbPoses();
	int positionCounter = 0;

	if (numberOfPositions > 1)
	{
		vector<DualQuaternion> positions;
		positions = getLimbPoses();

		if (positions.size() > 1)
		{
			vector<double> bernPoly;

			hMatrix transformationMat;
			int n;
			n = getNoOfLimbPoses() - 1;

			float totalInterpos = ((static_cast<float>(BezierProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
			float increment = 1 / (totalInterpos);
			
			for (double t = 0.00; t <= 1.0; t += increment)
			{
				bernPoly.clear();
				bernPoly = calculateBernsteinPolyForLimbData(t);
				DualQuaternion tempdQ;
				double MatrixForOpenGL[16];

				for (int i = 0; i <= n; i++)
				{
					tempdQ = tempdQ + (bernPoly.at(i))*positions.at(i);
				}
				transformationMat = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

				for (int i1 = 0; i1 < 4; i1++)
					for (int i2 = 0; i2 < 4; i2++)
						MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];

				MatrixForOpenGL[12] *= 10.0f;
				MatrixForOpenGL[13] *= 10.0f;
				MatrixForOpenGL[14] *= 10.0f;

				glPushMatrix();
				glMultMatrixd(MatrixForOpenGL);

				

				if (BezierProps.isPlotObject())
				{
					GLUquadricObj *qobj;
					qobj = gluNewQuadric();
					gluQuadricNormals(qobj, GLU_SMOOTH);
					gluQuadricDrawStyle(qobj, GLU_FILL);
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BezierProps.getObjectColor());
					
					if (mainWinPtr->planarCapture->isChecked())
					{
						glRotated(90.0f, 0.0f, 1.0f, 0.0f);
						gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
						//Motion::drawCylinder(qobj, 0.3f);
					}
					else
					{
						gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
					}
				}
				glPopMatrix();
				positionCounter++;
			}

			vector<vector<double> > collectionOfTranslationStart;
			//vector<vector<double> >collectionOfTranslationEnd;
			if (BezierProps.isPlotTrajectory())
			{
				for (double t = 0.00; t <= 1.0; t += 0.005)
				{
					bernPoly.clear();
					bernPoly = calculateBernsteinPolyForLimbData(t);
					DualQuaternion tempdQTrans;
					hMatrix transformationMatTransBezier;
					//hMatrix transformationMatTransBezierEnd;
					double MatrixForOpenGLTransBezier[16];

					for (int i = 0; i <= n; i++)
					{
						tempdQTrans = tempdQTrans + ((bernPoly.at(i))*positions.at(i));
					}

					transformationMatTransBezier = tempdQTrans.dualQuaternionToHomogeneousMatrix().transpose();
					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							MatrixForOpenGLTransBezier[4 * i1 + i2] = transformationMatTransBezier.m[i1][i2];

					MatrixForOpenGLTransBezier[12] *= 10.0f;
					MatrixForOpenGLTransBezier[13] *= 10.0f;
					MatrixForOpenGLTransBezier[14] *= 10.0f;

					vector<double> translationOfInterPos;
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[12]);
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[13]);
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[14]);
					collectionOfTranslationStart.emplace_back(translationOfInterPos);
					translationOfInterPos.clear();

					
				}
				glDisable(GL_LIGHTING);
				glColor4fv(BezierProps.getTrajectoryColor());
				glLineWidth(2.0);
				glBegin(GL_LINES);

				for (int i = 0; i < collectionOfTranslationStart.size() - 1; i++)
				{
					glVertex3d(collectionOfTranslationStart.at(i).at(0), collectionOfTranslationStart.at(i).at(1), collectionOfTranslationStart.at(i).at(2));
					glVertex3d(collectionOfTranslationStart.at(i + 1).at(0), collectionOfTranslationStart.at(i + 1).at(1), collectionOfTranslationStart.at(i + 1).at(2));
				}
				vector<DualQuaternion> tempLastCtrlPos = getLimbPoses();
				int numberOfPositionsLast = getNoOfLimbPoses();
				hMatrix trMat;
				trMat = tempLastCtrlPos.at(numberOfPositionsLast - 1).dualQuaternionToHomogeneousMatrix().transpose();
				double transOpenGLMatLastTrans[16];
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						transOpenGLMatLastTrans[4 * i1 + i2] = trMat.m[i1][i2];

				transOpenGLMatLastTrans[12] *= 10.0f;
				transOpenGLMatLastTrans[13] *= 10.0f;
				transOpenGLMatLastTrans[14] *= 10.0f;

				glVertex3d(collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(0), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(1), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(2));
				glVertex3d(transOpenGLMatLastTrans[12], transOpenGLMatLastTrans[13], transOpenGLMatLastTrans[14]);
				glEnd();

				glEnable(GL_LIGHTING);
			}
		}
	}
}
void MBezierMotion::plotSpineMotion()
{
	int numberOfPositions = getNoOfLimbPoses();
	int positionCounter = 0;

	if (numberOfPositions > 1)
	{
		vector<DualQuaternion> positions;
		positions = getLimbPoses();

		if (positions.size() > 1)
		{
			vector<double> bernPoly;

			hMatrix transformationMat;
			int n;
			n = getNoOfLimbPoses() - 1;

			float totalInterpos = ((static_cast<float>(BezierProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
			float increment = 1 / (totalInterpos);

			for (double t = 0.00; t <= 1.0; t += increment)
			{
				bernPoly.clear();
				bernPoly = calculateBernsteinPolyForLimbData(t);
				DualQuaternion tempdQ;
				double MatrixForOpenGL[16];

				for (int i = 0; i <= n; i++)
				{
					tempdQ = tempdQ + (bernPoly.at(i))*positions.at(i);
				}
				transformationMat = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

				for (int i1 = 0; i1 < 4; i1++)
					for (int i2 = 0; i2 < 4; i2++)
						MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];

				MatrixForOpenGL[12] *= 10.0f;
				MatrixForOpenGL[13] *= 10.0f;
				MatrixForOpenGL[14] *= 10.0f;

				glPushMatrix();
				glMultMatrixd(MatrixForOpenGL);

				

				if (BezierProps.isPlotObject())
				{
					GLUquadricObj *qobj;
					qobj = gluNewQuadric();
					gluQuadricNormals(qobj, GLU_SMOOTH);
					gluQuadricDrawStyle(qobj, GLU_FILL);
					glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BezierProps.getObjectColor());

					if (mainWinPtr->planarCapture->isChecked())
					{
						glRotated(90.0f, 0.0f, 1.0f, 0.0f);
						gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
						//Motion::drawCylinder(qobj, 0.3f);
					}
					else
					{
						gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
					}
				}
				glPopMatrix();
				positionCounter++;
			}

			vector<vector<double> > collectionOfTranslationStart;
			//vector<vector<double> >collectionOfTranslationEnd;
			if (BezierProps.isPlotTrajectory())
			{
				for (double t = 0.00; t <= 1.0; t += 0.005)
				{
					bernPoly.clear();
					bernPoly = calculateBernsteinPolyForLimbData(t);
					DualQuaternion tempdQTrans;
					hMatrix transformationMatTransBezier;
					//hMatrix transformationMatTransBezierEnd;
					double MatrixForOpenGLTransBezier[16];

					for (int i = 0; i <= n; i++)
					{
						tempdQTrans = tempdQTrans + ((bernPoly.at(i))*positions.at(i));
					}

					transformationMatTransBezier = tempdQTrans.dualQuaternionToHomogeneousMatrix().transpose();
					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							MatrixForOpenGLTransBezier[4 * i1 + i2] = transformationMatTransBezier.m[i1][i2];

					MatrixForOpenGLTransBezier[12] *= 10.0f;
					MatrixForOpenGLTransBezier[13] *= 10.0f;
					MatrixForOpenGLTransBezier[14] *= 10.0f;

					vector<double> translationOfInterPos;
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[12]);
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[13]);
					translationOfInterPos.emplace_back(MatrixForOpenGLTransBezier[14]);
					collectionOfTranslationStart.emplace_back(translationOfInterPos);
					translationOfInterPos.clear();

				}
				glDisable(GL_LIGHTING);
				glColor4fv(BezierProps.getTrajectoryColor());
				glLineWidth(2.0);
				glBegin(GL_LINES);

				for (int i = 0; i < collectionOfTranslationStart.size() - 1; i++)
				{
					glVertex3d(collectionOfTranslationStart.at(i).at(0), collectionOfTranslationStart.at(i).at(1), collectionOfTranslationStart.at(i).at(2));
					glVertex3d(collectionOfTranslationStart.at(i + 1).at(0), collectionOfTranslationStart.at(i + 1).at(1), collectionOfTranslationStart.at(i + 1).at(2));
				}
				vector<DualQuaternion> tempLastCtrlPos = getLimbPoses();
				int numberOfPositionsLast = getNoOfLimbPoses();
				hMatrix trMat;
				trMat = tempLastCtrlPos.at(numberOfPositionsLast - 1).dualQuaternionToHomogeneousMatrix().transpose();
				double transOpenGLMatLastTrans[16];
				for (auto i1 = 0; i1 < 4; i1++)
					for (auto i2 = 0; i2 < 4; i2++)
						transOpenGLMatLastTrans[4 * i1 + i2] = trMat.m[i1][i2];

				transOpenGLMatLastTrans[12] *= 10.0f;
				transOpenGLMatLastTrans[13] *= 10.0f;
				transOpenGLMatLastTrans[14] *= 10.0f;

				glVertex3d(collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(0), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(1), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(2));
				glVertex3d(transOpenGLMatLastTrans[12], transOpenGLMatLastTrans[13], transOpenGLMatLastTrans[14]);
				glEnd();

				glEnable(GL_LIGHTING);
			}
		}
	}
}

// B-Spline Approximated Motion
MRBSplineMotion::MRBSplineMotion()
{
	degree = 3;
}
MRBSplineMotion::~MRBSplineMotion()
{
}
void MRBSplineMotion::setDegree(int deg)
{
	degree = deg;
}
int MRBSplineMotion::getDegree()
{
	return degree;
}
vector<double> MRBSplineMotion::getKnotVector()
{
	int n = getNoOfCtrlPos() - 1;
	int p = getDegree();
	int numberOfKnots = n + p + 1;

	vector<double> knotVector(numberOfKnots + 1);

	for (int i = 0; i <= p; i++)
	{
		knotVector[i] = 0;
	}

	float j = 0.0;
	for (int i = p + 1; i <(numberOfKnots - p); i++)
	{
		j = j + 1.0;
		knotVector[i] = j / (numberOfKnots - 2 * p);
	}

	for (int i = numberOfKnots; i >= (numberOfKnots - p); i--)
	{
		knotVector[i] = 1;
	}

	return (knotVector);
}
vector<double> MRBSplineMotion::getKnotVectorForLimbData()
{
	int n = getNoOfLimbPoses() - 1;
	int p = getDegree();
	int numberOfKnots = n + p + 1;

	vector<double> knotVector(numberOfKnots + 1);

	for (int i = 0; i <= p; i++)
	{
		knotVector[i] = 0;
	}

	float j = 0.0;
	for (int i = p + 1; i <(numberOfKnots - p); i++)
	{
		j = j + 1.0;
		knotVector[i] = j / (numberOfKnots - 2 * p);
	}

	for (int i = numberOfKnots; i >= (numberOfKnots - p); i--)
	{
		knotVector[i] = 1;
	}

	return (knotVector);
}
void MRBSplineMotion::plotMotion()
{
	int numberOfPositions = getNoOfCtrlPos();
	int positionCounter = 0;
	
	if (numberOfPositions > 2)
	{
		vector<DualQuaternion> positions;
		positions = getCtrlPos();

		if (positions.size() > 1)
		{
			hMatrix transformationMat;

			int n = getNoOfCtrlPos() - 1;
			int p = getDegree();
			vector<double> U = getKnotVector();

			float totalInterpos = ((static_cast<float>(BSplineProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
			float increment = 1 / (totalInterpos);
			vector<vector<double> > collectionOfTranslation;
		
			if (p >= 2)
			{
				for (auto t = 0.0; t < 1.0; t += increment)
				{
					int span = getSpan(n, p, t, U);
					vector<float> N = getBasisFunction(span, t, p, U);
					double MatrixForOpenGL[16];

					DualQuaternion tempdQ;
					for (auto i = 0; i <= p; i++)
					{
						tempdQ = tempdQ + (N[i] * positions[span - p + i]);
					}
					//interpolatedPositions = tempdQ;
					transformationMat = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];

					MatrixForOpenGL[12] *= 10.0f;
					MatrixForOpenGL[13] *= 10.0f;
					MatrixForOpenGL[14] *= 10.0f;

					glPushMatrix();
					glMultMatrixd(MatrixForOpenGL);
					if (BSplineProps.isPlotCoordinateFrame())
					{
						GLUquadricObj *qobj;
						qobj = gluNewQuadric();
						gluQuadricNormals(qobj, GLU_SMOOTH);
						gluQuadricDrawStyle(qobj, GLU_FILL);
						glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
						glPushMatrix();
						glScalef(0.5, 0.5, 0.5);
						glTranslatef(0.0f, 0.0f, -1.125f);
						if (positionCounter == BSplineProps.getSpecificPosition())
						{
							drawSelectedCoordinateFrame(qobj);
						}
						else
						{
							drawCoordinateFrame(qobj);
						}

						glPopMatrix();
					}

					if (positionCounter == BSplineProps.getSpecificPosition())
					{
						glMaterialfv(GL_FRONT, GL_DIFFUSE, RED);
					}
					else
					{
						glMaterialfv(GL_FRONT, GL_DIFFUSE, BSplineProps.getObjectColor());
					}

					if (BSplineProps.isPlotObject())
					{
						//glScalef(0.5f, 0.5f, 0.5f);
						glScalef(BSplineProps.getObjectSize(), BSplineProps.getObjectSize(), BSplineProps.getObjectSize());
						drawWrist(0);
					}
					glPopMatrix();
					positionCounter++;
				}

				if (BSplineProps.isPlotTrajectory())
				{

					for (auto t = 0.0; t < 1.0; t += 0.005)
					{
						int span = getSpan(n, p, t, U);
						vector<float> N = getBasisFunction(span, t, p, U);
						double MatrixForOpenGL[16];
						vector<double> translationOfInterPos;
						DualQuaternion tempdQ;
						for (auto i = 0; i <= p; i++)
						{
							tempdQ = tempdQ + (N[i] * positions[span - p + i]);
						}
						//interpolatedPositions = tempdQ;
						transformationMat = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

						for (auto i1 = 0; i1 < 4; i1++)
							for (auto i2 = 0; i2 < 4; i2++)
								MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];

						MatrixForOpenGL[12] *= 10.0f;
						MatrixForOpenGL[13] *= 10.0f;
						MatrixForOpenGL[14] *= 10.0f;

						translationOfInterPos.emplace_back(MatrixForOpenGL[12]);
						translationOfInterPos.emplace_back(MatrixForOpenGL[13]);
						translationOfInterPos.emplace_back(MatrixForOpenGL[14]);

						collectionOfTranslation.emplace_back(translationOfInterPos);
					}

					glDisable(GL_LIGHTING);
					glColor4fv(BSplineProps.getTrajectoryColor());
					glLineWidth(4.0);
					glBegin(GL_LINES);

					for (auto i = 0; i < collectionOfTranslation.size() - 1; i++)                                            // BSplineProps.getNoOfInterPos()
					{
						glVertex3d(collectionOfTranslation.at(i).at(0), collectionOfTranslation.at(i).at(1), collectionOfTranslation.at(i).at(2));
						glVertex3d(collectionOfTranslation.at(i + 1).at(0), collectionOfTranslation.at(i + 1).at(1), collectionOfTranslation.at(i + 1).at(2));
					}

					vector<DualQuaternion> temp = getCtrlPos();
					int numberOfPositions = getNoOfCtrlPos();
					hMatrix trMat;
					double MatrixForOpenGL[16];
					trMat = temp.at(numberOfPositions - 1).dualQuaternionToHomogeneousMatrix().transpose();
					double transOpenGLMat[16];
					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							transOpenGLMat[4 * i1 + i2] = trMat.m[i1][i2];

					transOpenGLMat[12] *= 10.0f;
					transOpenGLMat[13] *= 10.0f;
					transOpenGLMat[14] *= 10.0f;

					glVertex3d(collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(0), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(1), collectionOfTranslation.at(collectionOfTranslation.size() - 1).at(2));
					glVertex3d(transOpenGLMat[12], transOpenGLMat[13], transOpenGLMat[14]);

					glEnd();
					glEnable(GL_LIGHTING);
				}
			}
			else if (p==1)
			{
				MScrewMotion* newScrewMotion = new MScrewMotion;
				newScrewMotion->setCtrlPos(this->getCtrlPos());

				if (this->BSplineProps.isPlotMotion())
				{
					newScrewMotion->ScrewProps.plotMotion(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotMotion(false);
				}

				if (this->BSplineProps.isPlotTrajectory())
				{
					newScrewMotion->ScrewProps.plotTrajectory(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotTrajectory(false);
				}

				if (this->BSplineProps.isPlotCoordinateFrame())
				{
					newScrewMotion->ScrewProps.plotCoordinateFrame(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotCoordinateFrame(false);
				}

				if (this->BSplineProps.isPlotObject())
				{
					newScrewMotion->ScrewProps.plotObject(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotObject(false);
				}

				newScrewMotion->ScrewProps.setNoOfInterPos(this->BSplineProps.getNoOfInterPos());
				newScrewMotion->ScrewProps.setSpecificPosition(this->BSplineProps.getSpecificPosition());
				newScrewMotion->ScrewProps.setObjectSize(this->BSplineProps.getObjectSize());
				float newScrewObjectcolor[3];
				newScrewObjectcolor[0] = *this->BSplineProps.getObjectColor();
				newScrewObjectcolor[1] = *(this->BSplineProps.getObjectColor() + 1);
				newScrewObjectcolor[2] = *(this->BSplineProps.getObjectColor() + 2);
				newScrewMotion->ScrewProps.setObjectColor(newScrewObjectcolor);

				float newScrewTrajectoryColor[3];
				newScrewTrajectoryColor[0] = *this->BSplineProps.getTrajectoryColor();
				newScrewTrajectoryColor[1] = *(this->BSplineProps.getTrajectoryColor() + 1);
				newScrewTrajectoryColor[2] = *(this->BSplineProps.getTrajectoryColor() + 2);

				newScrewMotion->plotMotion();

				delete newScrewMotion;
				newScrewMotion = NULL;
			}
		}
	}
}
void MRBSplineMotion::plotLimbMotion()
{
	int numberOfPositions = getNoOfLimbPoses();
	int positionCounter = 0;

	if (numberOfPositions > 2)
	{
		vector<DualQuaternion> positions;
		positions = getLimbPoses();

		if (positions.size() > 1)
		{
			hMatrix transformationMat;

			int n = getNoOfLimbPoses() - 1;
			int p = getDegree();
			vector<double> U = getKnotVectorForLimbData();

			float totalInterpos = ((static_cast<float>(BSplineProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
			float increment = 1 / (totalInterpos);
			//vector<vector<double> > collectionOfTranslation;

			if (p >= 2)
			{
				for (auto t = 0.0; t < 1.0; t += increment)
				{
					int span = getSpan(n, p, t, U);
					vector<float> N = getBasisFunction(span, t, p, U);
					double MatrixForOpenGL[16];

					DualQuaternion tempdQ;
					for (auto i = 0; i <= p; i++)
					{
						tempdQ = tempdQ + (N[i] * positions[span - p + i]);
					}
					transformationMat = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];

					MatrixForOpenGL[12] *= 10.0f;
					MatrixForOpenGL[13] *= 10.0f;
					MatrixForOpenGL[14] *= 10.0f;

					glPushMatrix();
					glMultMatrixd(MatrixForOpenGL);

					if (BSplineProps.isPlotObject())
					{
						GLUquadricObj *qobj;
						qobj = gluNewQuadric();
						gluQuadricNormals(qobj, GLU_SMOOTH);
						gluQuadricDrawStyle(qobj, GLU_FILL);
						glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
						glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BSplineProps.getObjectColor());
						if (mainWinPtr->planarCapture->isChecked())
						{
							glRotated(90.0f, 0.0f, 1.0f, 0.0f);
							gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
						}
						else
						{
							gluCylinder(qobj, 0.1, 0.1, 0.3f*10.0f, 20, 20);
						}
					}
					glPopMatrix();
					positionCounter++;
				}

				vector<vector<double> > collectionOfTranslationStart;
				//vector<vector<double> > collectionOfTranslationEnd;
				if (BSplineProps.isPlotTrajectory())
				{
					for (auto t = 0.0; t < 1.0; t += 0.005)
					{
						int span = getSpan(n, p, t, U);
						vector<float> N = getBasisFunction(span, t, p, U);
						double MatrixForOpenGLStart[16];
						hMatrix transMatForStart;
						//hMatrix transMatForEnd;
						DualQuaternion tempdQ;
						for (auto i = 0; i <= p; i++)
						{
							tempdQ = tempdQ + (N[i] * positions[span - p + i]);
						}
						transMatForStart = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();
						
						for (auto i1 = 0; i1 < 4; i1++)
							for (auto i2 = 0; i2 < 4; i2++)
								MatrixForOpenGLStart[4 * i1 + i2] = transMatForStart.m[i1][i2];

						MatrixForOpenGLStart[12] *= 10.0f;
						MatrixForOpenGLStart[13] *= 10.0f;
						MatrixForOpenGLStart[14] *= 10.0f;

						vector<double> translationOfInterPos;
						translationOfInterPos.emplace_back(MatrixForOpenGLStart[12]);
						translationOfInterPos.emplace_back(MatrixForOpenGLStart[13]);
						translationOfInterPos.emplace_back(MatrixForOpenGLStart[14]);
						collectionOfTranslationStart.emplace_back(translationOfInterPos);
						translationOfInterPos.clear();

						
					}

					glDisable(GL_LIGHTING);
					glColor4fv(BSplineProps.getTrajectoryColor());
					glLineWidth(4.0);
					
					glBegin(GL_LINES);
					for (auto i = 0; i < collectionOfTranslationStart.size() - 1; i++)                                            // BSplineProps.getNoOfInterPos()
					{
						glVertex3d(collectionOfTranslationStart.at(i).at(0), collectionOfTranslationStart.at(i).at(1), collectionOfTranslationStart.at(i).at(2));
						glVertex3d(collectionOfTranslationStart.at(i + 1).at(0), collectionOfTranslationStart.at(i + 1).at(1), collectionOfTranslationStart.at(i + 1).at(2));
					}

					vector<DualQuaternion> temp = getLimbPoses();
					int numberOfPositions = getNoOfLimbPoses();
					hMatrix trMat;
					trMat = temp.at(numberOfPositions - 1).dualQuaternionToHomogeneousMatrix().transpose();
					double transOpenGLMat[16];
					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							transOpenGLMat[4 * i1 + i2] = trMat.m[i1][i2];

					transOpenGLMat[12] *= 10.0f;
					transOpenGLMat[13] *= 10.0f;
					transOpenGLMat[14] *= 10.0f;

					glVertex3d(collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(0), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(1), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(2));
					glVertex3d(transOpenGLMat[12], transOpenGLMat[13], transOpenGLMat[14]);
					glEnd();

					glEnable(GL_LIGHTING);
				}
			}
			else if (p == 1)
			{
				MScrewMotion* newScrewMotion = new MScrewMotion;
				
				newScrewMotion->setLimbPoses(this->getLimbPoses());

				if (this->BSplineProps.isPlotTrajectory())
				{
					newScrewMotion->ScrewProps.plotTrajectory(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotTrajectory(false);
				}

				if (this->BSplineProps.isPlotObject())
				{
					newScrewMotion->ScrewProps.plotObject(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotObject(false);
				}

				newScrewMotion->ScrewProps.setNoOfInterPos(this->BSplineProps.getNoOfInterPos());
				newScrewMotion->ScrewProps.setSpecificPosition(this->BSplineProps.getSpecificPosition());
				//newScrewMotion->ScrewProps.setObjectSize(this->BSplineProps.getObjectSize());
				float newScrewObjectcolor[3];
				newScrewObjectcolor[0] = *this->BSplineProps.getObjectColor();
				newScrewObjectcolor[1] = *(this->BSplineProps.getObjectColor() + 1);
				newScrewObjectcolor[2] = *(this->BSplineProps.getObjectColor() + 2);
				newScrewMotion->ScrewProps.setObjectColor(newScrewObjectcolor);

				float newScrewTrajectoryColor[3];
				newScrewTrajectoryColor[0] = *this->BSplineProps.getTrajectoryColor();
				newScrewTrajectoryColor[1] = *(this->BSplineProps.getTrajectoryColor() + 1);
				newScrewTrajectoryColor[2] = *(this->BSplineProps.getTrajectoryColor() + 2);

				newScrewMotion->plotLimbMotion();

				delete newScrewMotion;
				newScrewMotion = NULL;
			}
		}
	}
}
void MRBSplineMotion::plotSpineMotion()
{
	int numberOfPositions = getNoOfLimbPoses();
	int positionCounter = 0;

	if (numberOfPositions > 2)
	{
		vector<DualQuaternion> positions;
		positions = getLimbPoses();

		if (positions.size() > 1)
		{
			hMatrix transformationMat;

			int n = getNoOfLimbPoses() - 1;
			int p = getDegree();
			vector<double> U = getKnotVectorForLimbData();

			float totalInterpos = ((static_cast<float>(BSplineProps.getNoOfInterPos()) / static_cast<float>(100))*(static_cast<float>(numberOfPositions)*static_cast<float>(20)));
			float increment = 1 / (totalInterpos);
			//vector<vector<double> > collectionOfTranslation;

			if (p >= 2)
			{
				for (auto t = 0.0; t < 1.0; t += increment)
				{
					int span = getSpan(n, p, t, U);
					vector<float> N = getBasisFunction(span, t, p, U);
					double MatrixForOpenGL[16];

					DualQuaternion tempdQ;
					for (auto i = 0; i <= p; i++)
					{
						tempdQ = tempdQ + (N[i] * positions[span - p + i]);
					}
					transformationMat = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							MatrixForOpenGL[4 * i1 + i2] = transformationMat.m[i1][i2];

					MatrixForOpenGL[12] *= 10.0f;
					MatrixForOpenGL[13] *= 10.0f;
					MatrixForOpenGL[14] *= 10.0f;

					glPushMatrix();
					glMultMatrixd(MatrixForOpenGL);
					if (BSplineProps.isPlotObject())
					{
						GLUquadricObj *qobj;
						qobj = gluNewQuadric();
						gluQuadricNormals(qobj, GLU_SMOOTH);
						gluQuadricDrawStyle(qobj, GLU_FILL);
						glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
						glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BSplineProps.getObjectColor());
						if (mainWinPtr->planarCapture->isChecked())
						{
							glRotated(90.0f, 0.0f, 1.0f, 0.0f);
							gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
						}
						else
						{
							gluCylinder(qobj, 0.1, 0.1, 0.2f*10.0f, 20, 20);
						}
					}
					glPopMatrix();
					positionCounter++;
				}

				vector<vector<double> > collectionOfTranslationStart;
				//vector<vector<double> > collectionOfTranslationEnd;
				if (BSplineProps.isPlotTrajectory())
				{
					for (auto t = 0.0; t < 1.0; t += 0.005)
					{
						int span = getSpan(n, p, t, U);
						vector<float> N = getBasisFunction(span, t, p, U);
						double MatrixForOpenGLStart[16];
						hMatrix transMatForStart;
						//hMatrix transMatForEnd;
						DualQuaternion tempdQ;
						for (auto i = 0; i <= p; i++)
						{
							tempdQ = tempdQ + (N[i] * positions[span - p + i]);
						}
						transMatForStart = tempdQ.dualQuaternionToHomogeneousMatrix().transpose();

						for (auto i1 = 0; i1 < 4; i1++)
							for (auto i2 = 0; i2 < 4; i2++)
								MatrixForOpenGLStart[4 * i1 + i2] = transMatForStart.m[i1][i2];

						MatrixForOpenGLStart[12] *= 10.0f;
						MatrixForOpenGLStart[13] *= 10.0f;
						MatrixForOpenGLStart[14] *= 10.0f;

						vector<double> translationOfInterPos;
						translationOfInterPos.emplace_back(MatrixForOpenGLStart[12]);
						translationOfInterPos.emplace_back(MatrixForOpenGLStart[13]);
						translationOfInterPos.emplace_back(MatrixForOpenGLStart[14]);
						collectionOfTranslationStart.emplace_back(translationOfInterPos);
						translationOfInterPos.clear();

					}

					glDisable(GL_LIGHTING);
					glColor4fv(BSplineProps.getTrajectoryColor());
					glLineWidth(4.0);

					glBegin(GL_LINES);
					for (auto i = 0; i < collectionOfTranslationStart.size() - 1; i++)                                            // BSplineProps.getNoOfInterPos()
					{
						glVertex3d(collectionOfTranslationStart.at(i).at(0), collectionOfTranslationStart.at(i).at(1), collectionOfTranslationStart.at(i).at(2));
						glVertex3d(collectionOfTranslationStart.at(i + 1).at(0), collectionOfTranslationStart.at(i + 1).at(1), collectionOfTranslationStart.at(i + 1).at(2));
					}

					vector<DualQuaternion> temp = getLimbPoses();
					int numberOfPositions = getNoOfLimbPoses();
					hMatrix trMat;
					trMat = temp.at(numberOfPositions - 1).dualQuaternionToHomogeneousMatrix().transpose();
					double transOpenGLMat[16];
					for (auto i1 = 0; i1 < 4; i1++)
						for (auto i2 = 0; i2 < 4; i2++)
							transOpenGLMat[4 * i1 + i2] = trMat.m[i1][i2];

					transOpenGLMat[12] *= 10.0f;
					transOpenGLMat[13] *= 10.0f;
					transOpenGLMat[14] *= 10.0f;

					glVertex3d(collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(0), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(1), collectionOfTranslationStart.at(collectionOfTranslationStart.size() - 1).at(2));
					glVertex3d(transOpenGLMat[12], transOpenGLMat[13], transOpenGLMat[14]);
					glEnd();

					glEnable(GL_LIGHTING);
				}
			}
			else if (p == 1)
			{
				MScrewMotion* newScrewMotion = new MScrewMotion;

				newScrewMotion->setLimbPoses(this->getLimbPoses());

				if (this->BSplineProps.isPlotTrajectory())
				{
					newScrewMotion->ScrewProps.plotTrajectory(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotTrajectory(false);
				}

				

				if (this->BSplineProps.isPlotObject())
				{
					newScrewMotion->ScrewProps.plotObject(true);
				}
				else
				{
					newScrewMotion->ScrewProps.plotObject(false);
				}

				newScrewMotion->ScrewProps.setNoOfInterPos(this->BSplineProps.getNoOfInterPos());
				newScrewMotion->ScrewProps.setSpecificPosition(this->BSplineProps.getSpecificPosition());
				//newScrewMotion->ScrewProps.setObjectSize(this->BSplineProps.getObjectSize());
				float newScrewObjectcolor[3];
				newScrewObjectcolor[0] = *this->BSplineProps.getObjectColor();
				newScrewObjectcolor[1] = *(this->BSplineProps.getObjectColor() + 1);
				newScrewObjectcolor[2] = *(this->BSplineProps.getObjectColor() + 2);
				newScrewMotion->ScrewProps.setObjectColor(newScrewObjectcolor);

				float newScrewTrajectoryColor[3];
				newScrewTrajectoryColor[0] = *this->BSplineProps.getTrajectoryColor();
				newScrewTrajectoryColor[1] = *(this->BSplineProps.getTrajectoryColor() + 1);
				newScrewTrajectoryColor[2] = *(this->BSplineProps.getTrajectoryColor() + 2);

				newScrewMotion->plotSpineMotion();

				delete newScrewMotion;
				newScrewMotion = NULL;
			}
		}
	}
}

// B-Spline Interpolated Motion
MRBSplineInterpolation::MRBSplineInterpolation()
{
}
MRBSplineInterpolation::~MRBSplineInterpolation()
{
}
vector<double> MRBSplineInterpolation::calcUkVec()
{
	int noOfPos = getNoOfCtrlPos();
	int n = noOfPos - 1;
	vector<double> ukVec;
	ukVec.clear();
	ukVec.push_back(0);
	for (double k = 1; k <= n - 1; k++)
	{
		ukVec.push_back(k / n);
	}
	ukVec.push_back(1.0);

	return ukVec;
}
vector<double> MRBSplineInterpolation::calcUkVecforLimbData()
{
	int noOfPos = getNoOfLimbPoses();
	int n = noOfPos - 1;
	vector<double> ukVec;
	ukVec.clear();
	ukVec.push_back(0);
	for (double k = 1; k <= n - 1; k++)
	{
		ukVec.push_back(k / n);
	}
	ukVec.push_back(1.0);

	return ukVec;
}
vector<DualQuaternion> MRBSplineInterpolation::newCtrlPositions()
{
	int noOfPositions = getNoOfCtrlPos();
	int n = noOfPositions - 1;
	int p = getDegree();

	vector<double>ukVec = calcUkVec();
	vector<double> knotVec = getKnotVector();

	Matrix N(n + 1, n + 1);

	N.Clear();

	for (auto i = 0; i <= n; i++)
	{
		int span = getSpan(n, p, ukVec[i], knotVec);
		vector<float> temp = getBasisFunction(span, ukVec[i], p, knotVec);

		for (auto j = 0; j <= p; j++)
		{
			N.m[i][span - p + j] = temp[j];
		}
	}

	Matrix NInverse(n + 1, n + 1);
	NInverse = N.Inverse();

	vector<DualQuaternion> Qip = this->getCtrlPos();
	DualQuaternion RD;

	vector<DualQuaternion> GFinal;

	for (auto i = 0; i < noOfPositions; i++)
	{
		for (auto j = 0; j < noOfPositions; j++)
		{
			RD = RD + NInverse.m[i][j] * Qip[j];
			if (j == noOfPositions - 1)
			{
				GFinal.emplace_back(RD);
				for (auto i = 0; i < 4; i++)
					RD.dual[i].SetupDual(0.0, 0.0);
			}
		}
	}
	return (GFinal);
}
vector<DualQuaternion> MRBSplineInterpolation::newCtrlPositionsForLimbData()
{
	int noOfPositions = getNoOfLimbPoses();
	int n = noOfPositions - 1;
	int p = getDegree();

	vector<double>ukVec = calcUkVecforLimbData();
	vector<double> knotVec = getKnotVectorForLimbData();

	Matrix N(n + 1, n + 1);

	N.Clear();

	for (auto i = 0; i <= n; i++)
	{
		int span = getSpan(n, p, ukVec[i], knotVec);
		vector<float> temp = getBasisFunction(span, ukVec[i], p, knotVec);

		for (auto j = 0; j <= p; j++)
		{
			N.m[i][span - p + j] = temp[j];
		}
	}

	Matrix NInverse(n + 1, n + 1);
	NInverse = N.Inverse();

	vector<DualQuaternion> Qip = this->getLimbPoses();
	DualQuaternion RD;

	vector<DualQuaternion> GFinal;

	for (auto i = 0; i < noOfPositions; i++)
	{
		for (auto j = 0; j < noOfPositions; j++)
		{
			RD = RD + NInverse.m[i][j] * Qip[j];
			if (j == noOfPositions - 1)
			{
				GFinal.emplace_back(RD);
				for (auto i = 0; i < 4; i++)
					RD.dual[i].SetupDual(0.0, 0.0);
			}
		}
	}
	return (GFinal);
}
void MRBSplineInterpolation::plotMotion()
{
	int noOfPositions = getNoOfCtrlPos();
	
	if (noOfPositions > 1)
	{
		
			MRBSplineMotion* newBSplineMotion = new MRBSplineMotion;
			vector<double> ukVec = calcUkVec();
			vector<DualQuaternion> nPositions = newCtrlPositions();
			newBSplineMotion->setCtrlPos(nPositions);
			newBSplineMotion->setDegree(this->getDegree());
			newBSplineMotion->plotCtrlPositions(YELLOW);
			if (this->BsplineInterprops.isPlotMotion())
			{

				newBSplineMotion->BSplineProps.plotMotion(true);
			}
			else
			{
				newBSplineMotion->BSplineProps.plotMotion(false);
			}

			if (this->BsplineInterprops.isPlotTrajectory())
			{
				newBSplineMotion->BSplineProps.plotTrajectory(true);
			}
			else
			{
				newBSplineMotion->BSplineProps.plotTrajectory(false);
			}

			if (this->BsplineInterprops.isPlotCoordinateFrame())
			{
				newBSplineMotion->BSplineProps.plotCoordinateFrame(true);
			}
			else
			{
				newBSplineMotion->BSplineProps.plotCoordinateFrame(false);
			}

			if (this->BsplineInterprops.isPlotObject())
			{
				newBSplineMotion->BSplineProps.plotObject(true);
			}
			else
			{
				newBSplineMotion->BSplineProps.plotObject(false);
			}

			newBSplineMotion->BSplineProps.setNoOfInterPos(this->BsplineInterprops.getNoOfInterPos());
			newBSplineMotion->BSplineProps.setSpecificPosition(this->BsplineInterprops.getSpecificPosition());
			newBSplineMotion->BSplineProps.setObjectSize(this->BsplineInterprops.getObjectSize());
			float newBsplineObjectColor[3];
			newBsplineObjectColor[0] = *this->BsplineInterprops.getObjectColor();
			newBsplineObjectColor[1] = *(this->BsplineInterprops.getObjectColor() + 1);
			newBsplineObjectColor[2] = *(this->BsplineInterprops.getObjectColor() + 2);
			newBSplineMotion->BSplineProps.setObjectColor(newBsplineObjectColor);

			float newBsplineTrajectoryColor[3];
			newBsplineTrajectoryColor[0] = *this->BsplineInterprops.getTrajectoryColor();
			newBsplineTrajectoryColor[1] = *(this->BsplineInterprops.getTrajectoryColor() + 1);
			newBsplineTrajectoryColor[2] = *(this->BsplineInterprops.getTrajectoryColor() + 2);
			newBSplineMotion->BSplineProps.setTrajectoryColor(newBsplineTrajectoryColor);

			newBSplineMotion->plotMotion();
			delete newBSplineMotion;
			newBSplineMotion = NULL;
		//}
	}
}
void MRBSplineInterpolation::plotLimbMotion()
{
	int noOfPositions = getNoOfLimbPoses();

	if (noOfPositions > 1)
	{

		MRBSplineMotion* newBSplineMotion = new MRBSplineMotion;
		vector<double> ukVec = calcUkVecforLimbData();
		vector<DualQuaternion> nPositions = newCtrlPositionsForLimbData();
		newBSplineMotion->setLimbPoses(nPositions);
		newBSplineMotion->setDegree(this->getDegree());


		if (this->BsplineInterprops.isPlotTrajectory())
		{
			newBSplineMotion->BSplineProps.plotTrajectory(true);
		}
		else
		{
			newBSplineMotion->BSplineProps.plotTrajectory(false);
		}


		if (this->BsplineInterprops.isPlotObject())
		{
			newBSplineMotion->BSplineProps.plotObject(true);
		}
		else
		{
			newBSplineMotion->BSplineProps.plotObject(false);
		}

		newBSplineMotion->BSplineProps.setNoOfInterPos(this->BsplineInterprops.getNoOfInterPos());
		newBSplineMotion->BSplineProps.setSpecificPosition(this->BsplineInterprops.getSpecificPosition());
		//newBSplineMotion->BSplineProps.setObjectSize(this->BsplineInterprops.getObjectSize());
		float newBsplineObjectColor[3];
		newBsplineObjectColor[0] = *this->BsplineInterprops.getObjectColor();
		newBsplineObjectColor[1] = *(this->BsplineInterprops.getObjectColor() + 1);
		newBsplineObjectColor[2] = *(this->BsplineInterprops.getObjectColor() + 2);
		newBSplineMotion->BSplineProps.setObjectColor(newBsplineObjectColor);

		float newBsplineTrajectoryColor[3];
		newBsplineTrajectoryColor[0] = *this->BsplineInterprops.getTrajectoryColor();
		newBsplineTrajectoryColor[1] = *(this->BsplineInterprops.getTrajectoryColor() + 1);
		newBsplineTrajectoryColor[2] = *(this->BsplineInterprops.getTrajectoryColor() + 2);
		newBSplineMotion->BSplineProps.setTrajectoryColor(newBsplineTrajectoryColor);

		newBSplineMotion->plotLimbMotion();
		delete newBSplineMotion;
		newBSplineMotion = NULL;
	}
}
void MRBSplineInterpolation::plotSpineMotion()
{
	int noOfPositions = getNoOfLimbPoses();

	if (noOfPositions > 1)
	{

		MRBSplineMotion* newBSplineMotion = new MRBSplineMotion;
		vector<double> ukVec = calcUkVecforLimbData();
		vector<DualQuaternion> nPositions = newCtrlPositionsForLimbData();
		newBSplineMotion->setLimbPoses(nPositions);
		newBSplineMotion->setDegree(this->getDegree());

		

		if (this->BsplineInterprops.isPlotTrajectory())
		{
			newBSplineMotion->BSplineProps.plotTrajectory(true);
		}
		else
		{
			newBSplineMotion->BSplineProps.plotTrajectory(false);
		}


		if (this->BsplineInterprops.isPlotObject())
		{
			newBSplineMotion->BSplineProps.plotObject(true);
		}
		else
		{
			newBSplineMotion->BSplineProps.plotObject(false);
		}

		newBSplineMotion->BSplineProps.setNoOfInterPos(this->BsplineInterprops.getNoOfInterPos());
		newBSplineMotion->BSplineProps.setSpecificPosition(this->BsplineInterprops.getSpecificPosition());
		//newBSplineMotion->BSplineProps.setObjectSize(this->BsplineInterprops.getObjectSize());
		float newBsplineObjectColor[3];
		newBsplineObjectColor[0] = *this->BsplineInterprops.getObjectColor();
		newBsplineObjectColor[1] = *(this->BsplineInterprops.getObjectColor() + 1);
		newBsplineObjectColor[2] = *(this->BsplineInterprops.getObjectColor() + 2);
		newBSplineMotion->BSplineProps.setObjectColor(newBsplineObjectColor);

		float newBsplineTrajectoryColor[3];
		newBsplineTrajectoryColor[0] = *this->BsplineInterprops.getTrajectoryColor();
		newBsplineTrajectoryColor[1] = *(this->BsplineInterprops.getTrajectoryColor() + 1);
		newBsplineTrajectoryColor[2] = *(this->BsplineInterprops.getTrajectoryColor() + 2);
		newBSplineMotion->BSplineProps.setTrajectoryColor(newBsplineTrajectoryColor);

		newBSplineMotion->plotSpineMotion();
		delete newBSplineMotion;
		newBSplineMotion = NULL;
	}
}


