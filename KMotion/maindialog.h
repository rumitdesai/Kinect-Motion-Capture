#ifndef MAINDIALOG_H
#define MAINDIALOG_H


//class QMainWindow;
#include <QtWidgets/QMainWindow>
//class QGridLayout;

//#include <QGridLayout>
//#include <QScrollArea>
//#include <QGroupBox>
//#include <QListWidget>
//#include <QCheckBox>
//#include <QPainter>
//#include <QPen>
//#include <QPixmap>
#include <Qtwidgets/QSlider>
//#include <QPolygon>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
//#include <QVector3D>
#include <Kinect.h>

#include "colorthread.h"
#include "skeletonthread.h"
#include "glWidget.h"
#include "Sensor.h"
#include "pugixml.hpp"

#include "JointSelectionDialog.h"
#include "limbSelectionDlg.h"







namespace Kinect{
	class colorDisplay;
	class mainDialog : public QMainWindow
	{
		Q_OBJECT
	public:                                 
		mainDialog();
		void createConnections();
		void initialize();
		void startThreads();
		void stopThreads();
		~mainDialog();
		
	//signals:
		//void positionSelected(int);
	public slots:	
		void updateDensityLabel(int);
		void updateObjectSizeLabel(int);
		void enableDisableStartRecording(int);
		void enableDisablePlanarSpatialbutton(bool);
		void enableDisablePositionSelectionSlider(bool);
		void enableDisableBatchPositionSelectionBtn(bool);
		void enableDisableScrewBezierBApprox(bool);
		void enableDisableGroup4(int);
		void enableDisableObjectSize(int);
		void enableDisableApproximationSpinBox(int);
		void enableDisableInterpoaltionSpinBox(int);

		
		void saveKeyPositionsPugi(bool);
		void saveContinuousMotionPugi(bool);
		void loadPugiFile(bool);
		void intermediateReset();
		void continuousReset();
		
		//void updateDebuggingList(QString);

		//void saveFile(bool);		
		
public:

	// *********************** Members of main UI widgets for interface  ************************** 
		
		QWidget *centralWidget;
		
		// colorFrameLabel and colorScrollArea  
		QLabel* colorFrameLabel;     
		QScrollArea *colorScrollArea;
		

		Sensor* kinectSensor;
		
		// Widgets related to skeleton data
		QLabel* skeletonFrameLabel;
		QScrollArea *skeletonScrollArea;
		
		// Widgets for OpenGL and Slider
		GLWidget *glWidget;
		QScrollArea *glWidgetScrollArea;
		QSlider* positionRecordingSlider;
		QLabel* positionRecordingSliderLabel;
		QPushButton* positionSelectPushBtn;
		QPushButton* batchPositionSelectBtn;
		

		// All QGroupBox's
		QGroupBox* group1;                           
		QGroupBox* group3;
		QGroupBox* group2;
		QGroupBox* group4;


		// Widgets for group1
		QCheckBox* planarCapture;
		QCheckBox* spatialCapture;
		QCheckBox* continuousCapture;
		QCheckBox* keyPositionCapture;
		QCheckBox* LimbMotionCapture;
		QCheckBox* jointMotionCapture;
		QCheckBox* startRecording;
		

		// Widgets for group3
		

		QPushButton* saveKeyPositions;
		
		
		//QPushButton* loadContinuousMotion;

		QPushButton* xmlType;
		QPushButton* saveContinuousPositions;
		QPushButton* loadFile;
		//QPushButton* allDataExport;

		QPushButton* resetKeyPositionsButton;
		QPushButton* resetContinuousButton;
		QPushButton* resetInteractionButton;

		// Widgets for group2
		
		QCheckBox* screwMotion;
		QCheckBox* bezierMotion;
		QCheckBox* bsplineApproximation;
		QCheckBox* bsplineInterpolation;
		//QCheckBox* shoulderTrajectory;
		
		QLabel* approximationSpinBoxLabel;
		QSpinBox* approximationDegreeSpinBox;
		QLabel* interpolationSpinBoxLabel;
		QSpinBox* interpolationDegreeSpinBox;

		// Widgets for group4
		QCheckBox* keyPositions;
		QCheckBox* continuousMotionCheckBox;
		QCheckBox* limbMotionCheckBox;
		QCheckBox* trajectoryCheckBox;
		QCheckBox* coordinateFrameCheckBox;
		QCheckBox* plotObjectCheckBox;
		QCheckBox* GLBackground;
		QSlider* densityChanger;
		QLabel* densityChangerLabel;
		QLabel* densityResultDisplayLabel;
		QSlider* ObjectSizeChanger;
		QLabel* ObjectSizeChangerLabel;
		QLabel* ObjectSizeDisplayLabel;
		QPushButton* selectJointsBtn;
		QPushButton* selectLimbsBtn;
		

		// Widget for debugging purpose
		//QListWidget* listToCheckPositionRecording;

		// *********************** Members pertaining to layout **************************

		// Layout for QMainWindow
		QVBoxLayout* centralLayout;
		
		// Layout for upper part of QMainWindow
		QHBoxLayout* colorSkeltonLayout;
		
		// Layouts for Slider and GLWidget
		QVBoxLayout* sliderGLWidgetLayout;
		QHBoxLayout* sliderLabelLayout;
		
		// Layout for lower horizontal part of QMainWindow
		QHBoxLayout* lowerHorizontalLayout;

		// Layout for grouping all below groups
		QGridLayout* controlsLayout;
		
		// Layouts for group1
		QGridLayout* captureTypeLayout;

		// Layouts for group3
		QVBoxLayout* dataExportLayout;
		
		// Layouts for group2
		QGridLayout* motionTypeLayout;
		QHBoxLayout* approximationSpinBoxLabelLayout;
		QHBoxLayout* interpolationSpinBoxLayout;

		// Layout for group4
		QVBoxLayout* motionPropertiesLayout;
		QHBoxLayout* densityResultLabelLayout;
		QVBoxLayout* densityChangerFinalLayout;
		QHBoxLayout* objectSizeResultLabelLayout;
		QVBoxLayout* objectSizeChangerFinalLayout;
		
// *********************** Data members pertaining to other classes defined in this project **************************

		// ColorThread and ColorDisplay variables
		colorThread* m_pcolorThread;                
		
		// SkeletonThread for skeleton data
		skeletonThread* m_pskeletonThread;

		JointSelection jointSelectionDlg;
		LimbSelection limbSelectionDlg;
//////////////////////////////// Generic data members 
	private:
		QString saveFileName;
		QString openFileName;

		void selectJointsDialog();
		void selectLimbsDialog();
		
	};
}

#endif // MAINDIALOG_H
