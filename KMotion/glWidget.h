#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QOpenglWidget>
#include <QtGui/QOpenglFunctions>
#include "Math/dualQuaternion.h"

class QWidget;
class QScrollArea;
class QSlider;
class QLabel;
class QGridLayout;
class QVBoxLayout;
class QHBoxLayout;
class QPointF;
class DualQuaternion;
class Motion;
class MScrewMotion;
class MBezierMotion;
class MRBSplineMotion;
class MRBSplineInterpolation;
class MNoneMotion;
class MContinuousMotion;
class LimbMotion;
class Point2D;


class GLWidget :public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	GLWidget(QWidget* parent = 0);
	void setupLight();
	void clearKeyGLWidget();
	void clearContinuousGLWidget();
	vector<DualQuaternion> getPositionFromContinuousMotion();

public slots:
	void updateSelectedPosition(int);
	void updateDensity(int);
	void updateGLToPlot(int);
	void updateApproximationDegree(int);
	void updateInterpolationDegree(int);
	void resetInteraction();
	void storeKeyPositionsInGLWidget(bool);
	void storeAllKeyPositionsInGLWidget(bool);


	
protected:
	void initializeGL()Q_DECL_OVERRIDE;
	void paintGL()Q_DECL_OVERRIDE;
	void resizeGL(int w, int h)Q_DECL_OVERRIDE;
	void mousePressEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	

private:
	double aspectRatio;
	int m_widgetHeight;
	int m_widgetWidth;

	int button;
	QPointF cursorLocation;
	QPointF moveCursorLocation;

	double m_xRotate;
	double m_yRotate;
	double m_zRotate;
	double m_xTrans;
	double m_yTrans;
	double m_zTrans;

	MContinuousMotion* ContinuousMotion;
	MScrewMotion* RScrewMotion;
	MScrewMotion* RScrewFemurMotion;
	MScrewMotion* RScrewTibiaMotion;
	MScrewMotion* RScrewSpineMotion;
	MScrewMotion* RScrewRightFemurMotion;
	MScrewMotion* RScrewRightTibiaMotion;

	MBezierMotion* RBezierMotion;
	MBezierMotion* RBezierFemurMotion;
	MBezierMotion* RBezierTibiaMotion;
	MBezierMotion* RBezierSpineMotion;
	MBezierMotion* RBezierRightFemurMotion;
	MBezierMotion* RBezierRightTibiaMotion;

	MRBSplineMotion* RBSplineMotion;
	MRBSplineMotion* RBSplineFemurMotion;
	MRBSplineMotion* RBSplineTibiaMotion;
	MRBSplineMotion* RBSPlineSpineMotion;
	MRBSplineMotion* RBSplineRightFemurMotion;
	MRBSplineMotion* RBSplineRightTibiaMotion;

	MRBSplineInterpolation* RBSplineInterpolation;
	MRBSplineInterpolation* RBSplineInterpolationFemurMotion;
	MRBSplineInterpolation* RBSplineInterpolationTibiaMotion;
	MRBSplineInterpolation* RBSplineInterpolationLeftUpperArmMotion;
	MRBSplineInterpolation* RBSplineInterpolationLeftLowerArmMotion;
	MRBSplineInterpolation* RBSplineInterpolationSpineMotion;
	MRBSplineInterpolation* RBSplineInterpolationRightFemurMotion;
	MRBSplineInterpolation* RBSplineInterpolationRightTibiaMotion;

	MRBSplineInterpolation* RBSplineLeftHipInterpolation;
	MRBSplineInterpolation* RBSplineLeftShoulderInterpolation;
	MRBSplineInterpolation* RBSplineLeftAnkleInterpolation;
	MRBSplineInterpolation* RBSplineRightAnkleInterpolation;
	MRBSplineInterpolation* RBSplineLeftElbowInterpolation;
	//MRBSplineInterpolation* RBSplineRightElbowInterpolation;

	LimbMotion* MFemurMotion;
	LimbMotion* MTibiaMotion;
	LimbMotion* MLeftUpperArmMotion;
	LimbMotion* MLeftLowerArmMotion;
	LimbMotion* MSpineMotion;
	LimbMotion* MRightFemurMotion;
	LimbMotion* MRightTibiaMotion;

	MContinuousMotion* ContinuousLeftHipMotion;
	MContinuousMotion* ContinuousLeftShoulderMotion;
	MContinuousMotion* ContinuousLeftAnkleMotion;
	MContinuousMotion* ContinuousRightAnkleMotion;
	MContinuousMotion* ContinuousLeftElbowMotion;
	//MContinuousMotion* ContinuousRightElbowMotion;
};


#endif