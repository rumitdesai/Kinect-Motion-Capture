//#include <QMainWindow>
#include <QtWidgets/QApplication>
#include <QtCore/QDebug>
#include <QtWidgets/QMenu>
#include <QtWidgets/QSizePolicy>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QCheckBox>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGui/QPixmap>
#include <QtGui/QPolygon>
#include <QtGui/QVector3D>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtWidgets/QFileDialog>
#include <QtCore/QSize>
#include <QtCore/QDir>
#include <QtCore/QDateTime>
#include <QtCore/QRect>
#include "maindialog.h"




namespace Kinect
{
	mainDialog::mainDialog()
	{
		if (this->objectName().isEmpty())
			this->setObjectName(QStringLiteral("main_dialog"));
		this->resize(1920, 1080);
		
		centralWidget = new QWidget;
		setCentralWidget(centralWidget);

		

		centralLayout = new QVBoxLayout(centralWidget);
		
		// Settings for coloFrameLabel to display color frame //30 
		colorFrameLabel = new QLabel(centralWidget);
		colorScrollArea = new QScrollArea;
		colorScrollArea->setWidget(colorFrameLabel);
		colorScrollArea->setWidgetResizable(true);
		colorScrollArea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding); //colorScrollArea->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
		//colorScrollArea->setMinimumSize(500, 350); //(500,350)
		colorScrollArea->setMinimumSize(400, 200);
		//colorScrollArea->setMinimumSize(900, 400);
		//colorScrollArea->setMinimumSize(940, 440);
		//colorScrollArea->setMaximumSize(940, 440);
		colorScrollArea->setMaximumSize(1540, 840);
		
		//centralLayout->addWidget(colorScrollArea);

		// skeletonFrameLabel and skeletonPixMap
		skeletonFrameLabel = new QLabel(centralWidget);
		skeletonScrollArea = new QScrollArea;
		skeletonScrollArea->setWidget(skeletonFrameLabel);
		skeletonScrollArea->setWidgetResizable(true);
		skeletonScrollArea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);  // skeletonScrollArea->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
		//skeletonScrollArea->setMinimumSize(500, 350); //(500,350)
		skeletonScrollArea->setMinimumSize(400, 200);
		//skeletonScrollArea->setMinimumSize(900, 400);
		//skeletonScrollArea->setMinimumSize(940, 440);
		//skeletonScrollArea->setMaximumSize(940, 440);  // 940,440
		skeletonScrollArea->setMaximumSize(1540, 840);

		colorSkeltonLayout = new QHBoxLayout(centralWidget);
		colorSkeltonLayout->setSpacing(6);
		colorSkeltonLayout->addWidget(colorScrollArea);
		colorSkeltonLayout->addWidget(skeletonScrollArea);
		centralLayout->addLayout(colorSkeltonLayout);
		

		// Widgets for GLWidget and QSlider
		glWidget = new GLWidget(centralWidget);
		glWidgetScrollArea = new QScrollArea;
		glWidgetScrollArea->setWidget(glWidget);
		glWidgetScrollArea->setWidgetResizable(true);
		glWidgetScrollArea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding); //glWidgetScrollArea->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
		//glWidgetScrollArea->setMinimumSize(500, 350);// (500,200), (500,350)
		glWidgetScrollArea->setMinimumSize(400, 200);
		//glWidgetScrollArea->setMinimumSize(900, 400);
		//glWidgetScrollArea->setMinimumSize(940, 440);
		//glWidgetScrollArea->setMaximumSize(940, 440);
		glWidgetScrollArea->setMaximumSize(1540, 840);

		sliderLabelLayout = new QHBoxLayout;
		positionRecordingSliderLabel = new QLabel("Select Position", centralWidget);
		positionRecordingSliderLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);  // preferred,preferred

		positionRecordingSlider = new QSlider(Qt::Horizontal,centralWidget);
		positionRecordingSlider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);  // expanding,fixed
		positionRecordingSlider->setSingleStep(1);
		positionRecordingSlider->setEnabled(false);

		positionSelectPushBtn = new QPushButton(tr("Select position"));
		positionSelectPushBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		positionSelectPushBtn->setMinimumHeight(20);
		positionSelectPushBtn->setEnabled(false);

		batchPositionSelectBtn = new QPushButton(tr("Select shown positions"));
		batchPositionSelectBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		batchPositionSelectBtn->setMinimumHeight(20);
		batchPositionSelectBtn->setEnabled(false);

		sliderLabelLayout->addWidget(positionRecordingSliderLabel);
		sliderLabelLayout->addWidget(positionRecordingSlider);
		sliderLabelLayout->addWidget(positionSelectPushBtn);
		sliderLabelLayout->addWidget(batchPositionSelectBtn);

		sliderGLWidgetLayout = new QVBoxLayout;
		sliderGLWidgetLayout->addWidget(glWidgetScrollArea);
		sliderGLWidgetLayout->addLayout(sliderLabelLayout);

		
		
		
		// QHBoxLayout for lowerHorizontalLayout
		lowerHorizontalLayout = new QHBoxLayout;
		lowerHorizontalLayout->setSpacing(6);
		lowerHorizontalLayout->addLayout(sliderGLWidgetLayout);
		
		// QSizePolicy for QGroupBoxes
		QSizePolicy sizePolicyForGroupBoxes(QSizePolicy::Expanding, QSizePolicy::Expanding);
		sizePolicyForGroupBoxes.setHorizontalStretch(0);
		sizePolicyForGroupBoxes.setVerticalStretch(0);

		// QGridLayout for all controls 
		controlsLayout = new QGridLayout;
		controlsLayout->setSpacing(6);


		// QGroupBox for group1
		group1 = new QGroupBox(tr("Type Of Capture"), centralWidget);
		group1->setSizePolicy(sizePolicyForGroupBoxes);
		sizePolicyForGroupBoxes.setHeightForWidth(group1->sizePolicy().hasHeightForWidth());
		
		
		captureTypeLayout = new QGridLayout(group1);
		captureTypeLayout->setSpacing(6);

		
		continuousCapture = new QCheckBox(tr("Continuous Motion Capture"));
		keyPositionCapture = new QCheckBox(tr("Key position Capture"));
		LimbMotionCapture = new QCheckBox(tr("Limb Motion Capture"));
		jointMotionCapture = new QCheckBox(tr("Joint Motion Capture"));
		planarCapture = new QCheckBox(tr("Planar"));
		spatialCapture = new QCheckBox(tr("Spatial"));
		startRecording = new QCheckBox(tr("Start Recording"));
		planarCapture->setEnabled(false);
		spatialCapture->setEnabled(false);
		startRecording->setEnabled(false);


		captureTypeLayout->addWidget(continuousCapture, 0, 0, 1, 1);
		captureTypeLayout->addWidget(keyPositionCapture, 1, 0, 1, 1);
		captureTypeLayout->addWidget(LimbMotionCapture, 2, 0, 1, 1);
		captureTypeLayout->addWidget(jointMotionCapture, 3, 0, 1, 1);
		captureTypeLayout->addWidget(planarCapture, 0, 1, 1, 1);
		captureTypeLayout->addWidget(spatialCapture, 1, 1, 1, 1);
		captureTypeLayout->addWidget(startRecording, 4, 0, 1, 1);
		

		group1->setLayout(captureTypeLayout);

		controlsLayout->addWidget(group1, 0, 0, 1, 1);


		// QGroupBox for group3
		group3 = new QGroupBox(tr("General Settings"), centralWidget);
		group3->setSizePolicy(sizePolicyForGroupBoxes);
		sizePolicyForGroupBoxes.setHeightForWidth(group3->sizePolicy().hasHeightForWidth());

		saveKeyPositions = new QPushButton(tr("Export Positions"));
		saveKeyPositions->setEnabled(false);
		saveContinuousPositions = new QPushButton(tr("Export Captured Motion"));
		saveContinuousPositions -> setEnabled(false);
		loadFile = new QPushButton(tr("Load File"));
		//loadContinuousMotion = new QPushButton(tr("Load Continuous Motion"));

		xmlType = new QPushButton(tr("Export key positions"));
		xmlType->setEnabled(false);
		saveContinuousPositions = new QPushButton(tr("Export captured motion"));
		saveContinuousPositions->setEnabled(false);
		loadFile = new QPushButton(tr("Load file"));
		//allDataExport = new QPushButton(tr("Export shown positions to XML"));

		
		resetKeyPositionsButton = new QPushButton(tr("Reset Key Position"));
		resetContinuousButton = new QPushButton(tr("Reset Continuous Motion"));
		resetInteractionButton = new QPushButton(tr("Reset To Original Position"));
		dataExportLayout = new QVBoxLayout(group3);
		dataExportLayout->addWidget(resetKeyPositionsButton);
		dataExportLayout->addWidget(resetContinuousButton);
		dataExportLayout->addWidget(resetInteractionButton);

		dataExportLayout->addWidget(saveKeyPositions);

		dataExportLayout->addWidget(xmlType);

		dataExportLayout->addWidget(saveContinuousPositions);
		dataExportLayout->addWidget(loadFile);
		//dataExportLayout->addWidget(allDataExport);
		
		group3->setLayout(dataExportLayout);

		controlsLayout->addWidget(group3, 0, 2, 1, 1);

		// QGroupBox for group2 
		group2 = new QGroupBox(tr("Type Of Motion"),centralWidget);
		group2->setSizePolicy(sizePolicyForGroupBoxes);
		sizePolicyForGroupBoxes.setHeightForWidth(group2->sizePolicy().hasHeightForWidth());
		
		screwMotion = new QCheckBox(tr("Screw"));
		bezierMotion = new QCheckBox(tr("Bezier"));
		bsplineApproximation = new QCheckBox(tr("B-Spline Approximation"));
		approximationSpinBoxLabel = new QLabel(tr("Degree"));
		approximationDegreeSpinBox = new QSpinBox;
		approximationDegreeSpinBox->setRange(0, 10);
		approximationDegreeSpinBox->setSingleStep(1);
		approximationDegreeSpinBox->setValue(3);

		approximationDegreeSpinBox->setEnabled(false);
		approximationSpinBoxLabelLayout = new QHBoxLayout;
		approximationSpinBoxLabelLayout->addWidget(approximationSpinBoxLabel);
		approximationSpinBoxLabelLayout->addWidget(approximationDegreeSpinBox);

		bsplineInterpolation = new QCheckBox(tr("B-Spline Interpolation"));
		interpolationSpinBoxLabel = new QLabel(tr("Degree"));
		interpolationDegreeSpinBox = new QSpinBox;
		interpolationDegreeSpinBox->setRange(0, 10);
		interpolationDegreeSpinBox->setSingleStep(1);
		interpolationDegreeSpinBox->setValue(3);

		interpolationDegreeSpinBox->setEnabled(false);
		interpolationSpinBoxLayout = new QHBoxLayout;
		interpolationSpinBoxLayout->addWidget(interpolationSpinBoxLabel);
		interpolationSpinBoxLayout->addWidget(interpolationDegreeSpinBox);
		

		motionTypeLayout = new QGridLayout(group2);
		
		motionTypeLayout->addWidget(screwMotion,0,0,1,1);
		motionTypeLayout->addWidget(bezierMotion,1,0,1,1);
		motionTypeLayout->addWidget(bsplineApproximation,2,0,1,1);
		motionTypeLayout->addWidget(bsplineInterpolation,3,0,1,1);
		motionTypeLayout->addLayout(approximationSpinBoxLabelLayout, 2, 1, 1, 1);
		motionTypeLayout->addLayout(interpolationSpinBoxLayout,3,1,1,1);
		
		group2->setLayout(motionTypeLayout);

		controlsLayout->addWidget(group2, 1, 0, 1, 1);

		// QGroupBox for group4
		group4 = new QGroupBox(tr("Motion Properties"),centralWidget);
		group4->setSizePolicy(sizePolicyForGroupBoxes);
		sizePolicyForGroupBoxes.setHeightForWidth(group4->sizePolicy().hasHeightForWidth());
		keyPositions = new QCheckBox(tr("Key Positions"));
		keyPositions->setEnabled(false);
		continuousMotionCheckBox = new QCheckBox(tr("Continuous Motion"));
		continuousMotionCheckBox->setEnabled(false);
		limbMotionCheckBox = new QCheckBox(tr("Limb Motion"));
		limbMotionCheckBox->setEnabled(false);
		trajectoryCheckBox = new QCheckBox(tr("Trajectory"));
		coordinateFrameCheckBox = new QCheckBox(tr("Coordinate Frames"));
		plotObjectCheckBox = new QCheckBox(tr("Object"));
		GLBackground = new QCheckBox("White Background");
		selectJointsBtn = new QPushButton(tr("Select Joints"));
		selectJointsBtn->setEnabled(false);
		selectLimbsBtn = new QPushButton(tr("Select Limbs"));
		selectLimbsBtn->setEnabled(false);
		////////////////////////////////////////////////////////////
		densityResultLabelLayout = new QHBoxLayout;
		densityChangerLabel = new QLabel("Density", group4);
		densityChangerLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

		densityResultDisplayLabel = new QLabel("100 %", group4);
		densityResultDisplayLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

		densityChanger = new QSlider(Qt::Horizontal, group4);
		densityChanger->setRange(0, 100);
		densityChanger->setSingleStep(1);
		densityChanger->setSliderPosition(100);
		//densityChanger->setEnabled(false);
		densityChanger->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);

		densityResultLabelLayout->addWidget(densityChanger);
		densityResultLabelLayout->addWidget(densityResultDisplayLabel);

		densityChangerFinalLayout = new QVBoxLayout;
		densityChangerFinalLayout->addWidget(densityChangerLabel);
		densityChangerFinalLayout->addLayout(densityResultLabelLayout);
		////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////
		objectSizeResultLabelLayout = new QHBoxLayout;
		ObjectSizeChangerLabel = new QLabel("Object Size", group4);
		ObjectSizeChangerLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

		ObjectSizeDisplayLabel= new QLabel("70 %", group4);
		ObjectSizeDisplayLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

		ObjectSizeChanger= new QSlider(Qt::Horizontal, group4);
		ObjectSizeChanger->setRange(1, 100);
		ObjectSizeChanger->setSingleStep(1);
		ObjectSizeChanger->setSliderPosition(70);
		ObjectSizeChanger->setEnabled(false);
		ObjectSizeChanger->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);

		objectSizeResultLabelLayout->addWidget(ObjectSizeChanger);
		objectSizeResultLabelLayout->addWidget(ObjectSizeDisplayLabel);

		objectSizeChangerFinalLayout= new QVBoxLayout;
		objectSizeChangerFinalLayout->addWidget(ObjectSizeChangerLabel);
		objectSizeChangerFinalLayout->addLayout(objectSizeResultLabelLayout);

		////////////////////////////////////////////////////////////////////////


		motionPropertiesLayout = new QVBoxLayout(group4);
		motionPropertiesLayout->addWidget(continuousMotionCheckBox);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(keyPositions);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(limbMotionCheckBox);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(trajectoryCheckBox);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(coordinateFrameCheckBox);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(plotObjectCheckBox);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(GLBackground);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addLayout(densityChangerFinalLayout);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addLayout(objectSizeChangerFinalLayout);
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(selectJointsBtn); /// Take care of this pushbutton
		motionPropertiesLayout->addStretch(1);
		motionPropertiesLayout->addWidget(selectLimbsBtn);
		group4->setLayout(motionPropertiesLayout);

		
		trajectoryCheckBox->setEnabled(false);
		coordinateFrameCheckBox->setEnabled(false);
		plotObjectCheckBox->setEnabled(false);
		densityChanger->setEnabled(false);
		ObjectSizeChanger->setEnabled(false);
		
		controlsLayout->addWidget(group4, 0, 1, 2,1);

        // QListWidget for debugging 
		//listToCheckPositionRecording = new QListWidget(centralWidget);
		//controlsLayout->addWidget(listToCheckPositionRecording, 1, 2, 1, 1);
		//QListWidgetItem* item1 = new QListWidgetItem("Shoulder", listToCheckPositionRecording);

		//listToCheckPositionRecording->addItem(item1);
		//listToCheckPositionRecording->setse
		lowerHorizontalLayout->addLayout(controlsLayout);


		centralLayout->addLayout(lowerHorizontalLayout);

		centralLayout->addLayout(lowerHorizontalLayout);
		
		
		QMainWindow::setWindowTitle("Kinect Motion Capture");
		
		

		initialize();
		createConnections();
	}

	

	void mainDialog::initialize()
	{
		kinectSensor = new Sensor(colorFrameLabel, skeletonFrameLabel, this);
		m_pcolorThread = new colorThread(this);
		m_pskeletonThread = new skeletonThread(this);
		connect(m_pcolorThread, SIGNAL(updatedColorFrame()), kinectSensor, SLOT(updateColorFrame()));
		connect(m_pskeletonThread, SIGNAL(updatedSkeletonFrame()), kinectSensor, SLOT(updateSkeletonFrame()));
		startThreads();
	}

	void mainDialog::createConnections()
	{

		// Connections for continuousCapture and keyPositionCapture
		connect(continuousCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisablePlanarSpatialbutton(bool)));
		connect(continuousCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisablePositionSelectionSlider(bool)));
		
		
		connect(keyPositionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisablePlanarSpatialbutton(bool)));
		
		connect(LimbMotionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisablePlanarSpatialbutton(bool)));
		connect(LimbMotionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisableBatchPositionSelectionBtn(bool)));
		connect(LimbMotionCapture, SIGNAL(stateChanged(int)), this, SLOT(enableDisableGroup4(int)));
		
		connect(jointMotionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisablePlanarSpatialbutton(bool)));
		//connect(jointMotionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisablePositionSelectionSlider(bool)));
		connect(jointMotionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisableBatchPositionSelectionBtn(bool)));
		connect(jointMotionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisableScrewBezierBApprox(bool)));
		connect(jointMotionCapture, SIGNAL(stateChanged(int)), this, SLOT(enableDisableGroup4(int)));

		// Connections for planarCapture and spatialCapture
		connect(planarCapture, SIGNAL(stateChanged(int)), this, SLOT(enableDisableStartRecording(int)));
		connect(spatialCapture, SIGNAL(stateChanged(int)), this, SLOT(enableDisableStartRecording(int)));

		connect(screwMotion, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));
		connect(screwMotion, SIGNAL(stateChanged(int)), this, SLOT(enableDisableGroup4(int)));
		connect(screwMotion, SIGNAL(stateChanged(int)), this, SLOT(enableDisableObjectSize(int)));

		connect(bezierMotion, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));
		connect(bezierMotion, SIGNAL(stateChanged(int)), this, SLOT(enableDisableGroup4(int)));
		connect(bezierMotion, SIGNAL(stateChanged(int)), this, SLOT(enableDisableObjectSize(int)));

		connect(bsplineApproximation, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));
		connect(bsplineApproximation, SIGNAL(stateChanged(int)), this, SLOT(enableDisableGroup4(int)));
		connect(bsplineApproximation, SIGNAL(stateChanged(int)), this, SLOT(enableDisableObjectSize(int)));
		connect(bsplineApproximation, SIGNAL(stateChanged(int)), this, SLOT(enableDisableApproximationSpinBox(int)));

		connect(bsplineInterpolation, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));
		connect(bsplineInterpolation, SIGNAL(stateChanged(int)), this, SLOT(enableDisableGroup4(int)));
		connect(bsplineInterpolation, SIGNAL(stateChanged(int)), this, SLOT(enableDisableObjectSize(int)));
		connect(bsplineInterpolation, SIGNAL(stateChanged(int)), this, SLOT(enableDisableInterpoaltionSpinBox(int)));

		connect(approximationDegreeSpinBox, SIGNAL(valueChanged(int)), glWidget, SLOT(updateApproximationDegree(int)));
		connect(interpolationDegreeSpinBox, SIGNAL(valueChanged(int)), glWidget, SLOT(updateInterpolationDegree(int)));

		connect(continuousMotionCheckBox, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));
		connect(continuousMotionCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enableDisableGroup4(int)));
		
		connect(keyPositions, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));

		connect(limbMotionCheckBox, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));

		connect(trajectoryCheckBox, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));

		connect(coordinateFrameCheckBox, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));

		connect(plotObjectCheckBox, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));
		connect(plotObjectCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enableDisableObjectSize(int)));

		connect(GLBackground, SIGNAL(stateChanged(int)), glWidget, SLOT(updateGLToPlot(int)));

		connect(densityChanger, SIGNAL(sliderMoved(int)), this, SLOT(updateDensityLabel(int)));
		connect(densityChanger, SIGNAL(sliderMoved(int)), glWidget, SLOT(updateDensity(int)));
		
		connect(ObjectSizeChanger, SIGNAL(sliderMoved(int)), glWidget, SLOT(updateGLToPlot(int)));
		connect(ObjectSizeChanger, SIGNAL(sliderMoved(int)), this, SLOT(updateObjectSizeLabel(int)));
		
		connect(selectJointsBtn, SIGNAL(pressed()), this, SLOT(selectJointsDialog()));
		connect(selectLimbsBtn, SIGNAL(pressed()), this, SLOT(selectLimbsDialog()));

		connect(positionRecordingSlider, SIGNAL(sliderMoved(int)), glWidget, SLOT(updateSelectedPosition(int)));
		connect(positionSelectPushBtn, SIGNAL(clicked(bool)), glWidget, SLOT(storeKeyPositionsInGLWidget(bool)));
		connect(batchPositionSelectBtn, SIGNAL(clicked(bool)), glWidget, SLOT(storeAllKeyPositionsInGLWidget(bool)));
		


		//connect(positionSelectPushBtn, SIGNAL(clicked(bool)), kinectSensor, SLOT(StoreKeyPosition(bool)));
		
		

		

		// Connections for continuousCapture and keyPositionCapture
		connect(continuousCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisableContinuousSpatialbutton(bool)));
		connect(keyPositionCapture, SIGNAL(toggled(bool)), this, SLOT(enableDisableContinuousSpatialbutton(bool)));
		connect(continuousCapture,SIGNAL(toggled(bool)),this,SLOT(enableDisablePositionSelectionSlider(bool)));
		// Connections for planarCapture and spatialCapture
		connect(planarCapture, SIGNAL(stateChanged(int)), this, SLOT(enableDisableStartRecordingAndResetButton(int)));
		connect(spatialCapture, SIGNAL(stateChanged(int)), this, SLOT(enableDisableStartRecordingAndResetButton(int)));

		//connect(allDataExport, SIGNAL(clicked(bool)), this,SLOT(saveAllPositionsInFile(bool)));
		//connect(xmlType, SIGNAL(clicked(bool)), this, SLOT(saveFile(bool)));
		connect(xmlType, SIGNAL(clicked(bool)), this, SLOT(saveKeyPositionsPugi(bool)));


		connect(resetKeyPositionsButton, SIGNAL(pressed()), this, SLOT(intermediateReset()));
		connect(resetContinuousButton, SIGNAL(pressed()), this, SLOT(continuousReset()));
		connect(resetInteractionButton, SIGNAL(pressed()), glWidget, SLOT(resetInteraction()));
		connect(saveKeyPositions, SIGNAL(clicked(bool)), this, SLOT(saveKeyPositionsPugi(bool)));
		connect(saveContinuousPositions, SIGNAL(clicked(bool)), this, SLOT(saveContinuousMotionPugi(bool)));
		connect(loadFile, SIGNAL(clicked(bool)), this, SLOT(loadPugiFile(bool)));

		//connect(kinectSensor, SIGNAL(debuggingCheck(QString)), this, SLOT(updateDebuggingList(QString)));
		//connect(glWidget, SIGNAL(emitSizeOfpositions(QString,int)), this, SLOT(updateDebuggingList(QString,int)));
		//connect(kinectSensor, SIGNAL(clearDebuggingCheck(QString)), this, SLOT(clearDebugginList(QString)));

		
	}

	void mainDialog::startThreads()
	{
		m_pcolorThread->start();
		m_pskeletonThread->start();
	}

	void mainDialog::stopThreads()
	{
		m_pcolorThread->stop();
		m_pskeletonThread->stop();
	}

	mainDialog::~mainDialog()
	{
		stopThreads();
		if (kinectSensor != NULL){
			//qDebug() << "coming here to delete kinectsensor";
			delete kinectSensor;
			kinectSensor = NULL;
		}
	}

	void mainDialog::selectJointsDialog()
	{
		jointSelectionDlg.exec();
	}

	void mainDialog::selectLimbsDialog()
	{
		limbSelectionDlg.exec();
	}
	

	void mainDialog::updateDensityLabel(int val)
	{
		emit densityResultDisplayLabel->setText(QString("%1 %").arg(val));
	}

	void mainDialog::updateObjectSizeLabel(int val)
	{
		emit ObjectSizeDisplayLabel->setText(QString("%1 %").arg(val));
	}

	void mainDialog::enableDisablePlanarSpatialbutton(bool state)
	{
		if (continuousCapture->isChecked())
		{
			planarCapture->setEnabled(true);
			spatialCapture->setEnabled(true);
			keyPositionCapture->setEnabled(false);
			LimbMotionCapture->setEnabled(false);
			jointMotionCapture->setEnabled(false);
		}
		else
		{
			if (!keyPositionCapture->isChecked() && !LimbMotionCapture->isChecked() && !jointMotionCapture->isChecked())
			{
				planarCapture->setEnabled(false);
				spatialCapture->setEnabled(false);
				keyPositionCapture->setEnabled(true);
				LimbMotionCapture->setEnabled(true);
				jointMotionCapture->setEnabled(true);
			}
		}

		if (keyPositionCapture->isChecked())
		{
			planarCapture->setEnabled(true);
			spatialCapture->setEnabled(true);
			continuousCapture->setEnabled(false);
			LimbMotionCapture->setEnabled(false);
			jointMotionCapture->setEnabled(false);
		}
		else
		{
			if (!continuousCapture->isChecked() && !LimbMotionCapture->isChecked() && !jointMotionCapture->isChecked())
			{
				planarCapture->setEnabled(false);
				spatialCapture->setEnabled(false);
				continuousCapture->setEnabled(true);
				LimbMotionCapture->setEnabled(true);
				jointMotionCapture->setEnabled(true);
			}
		}

		if (LimbMotionCapture->isChecked())
		{
			planarCapture->setEnabled(true);
			spatialCapture->setEnabled(true);
			continuousCapture->setEnabled(false);
			keyPositionCapture->setEnabled(false);
			jointMotionCapture->setEnabled(false);
		}
		else
		{
			if (!continuousCapture->isChecked() && !keyPositionCapture->isChecked() && !jointMotionCapture->isChecked())
			{
				planarCapture->setEnabled(false);
				spatialCapture->setEnabled(false);
				continuousCapture->setEnabled(true);
				keyPositionCapture->setEnabled(true);
				jointMotionCapture->setEnabled(true);
			}
		}

		if (jointMotionCapture->isChecked())
		{
			planarCapture->setEnabled(true);
			spatialCapture->setEnabled(true);
			continuousCapture->setEnabled(false);
			keyPositionCapture->setEnabled(false);
			LimbMotionCapture->setEnabled(false);
		}
		else
		{
			if (!continuousCapture->isChecked() && !keyPositionCapture->isChecked() && !LimbMotionCapture->isChecked())
			{
				planarCapture->setEnabled(false);
				spatialCapture->setEnabled(false);
				continuousCapture->setEnabled(true);
				keyPositionCapture->setEnabled(true);
				LimbMotionCapture->setEnabled(true);
			}
		}
	}

	void mainDialog::enableDisableStartRecording(int st)
	{
			if (planarCapture->isChecked()){
				spatialCapture->setEnabled(false);
				startRecording->setEnabled(true);
			}
			else if (spatialCapture->isChecked()){
				planarCapture->setEnabled(false);
				startRecording->setEnabled(true);
			}
			else if (!spatialCapture->isChecked() && !planarCapture->isEnabled()){
				planarCapture->setEnabled(true);
				startRecording->setEnabled(false);
			}
			else if (!planarCapture->isChecked() && !spatialCapture->isEnabled()){
				spatialCapture->setEnabled(true);
				startRecording->setEnabled(false);
			}
	}

	void mainDialog::enableDisableGroup4(int st)
	{
		if (screwMotion->isChecked() || bezierMotion->isChecked() || bsplineApproximation->isChecked() || bsplineInterpolation->isChecked() || continuousMotionCheckBox->isChecked()){
			trajectoryCheckBox->setEnabled(true);
			coordinateFrameCheckBox->setEnabled(true);
			plotObjectCheckBox->setEnabled(true);
			densityChanger->setEnabled(true);
			//ObjectSizeChanger->setEnabled(true);
		}
		else
		{
			trajectoryCheckBox->setEnabled(false);
			coordinateFrameCheckBox->setEnabled(false);
			plotObjectCheckBox->setEnabled(false);
			densityChanger->setEnabled(false);
			//ObjectSizeChanger->setEnabled(false);
		}

		if (LimbMotionCapture->isChecked())
		{
			densityChanger->setEnabled(true);
			trajectoryCheckBox->setEnabled(true);
			plotObjectCheckBox->setEnabled(true);
			selectLimbsBtn->setEnabled(true);
		}

		if (jointMotionCapture->isChecked())
		{
			selectJointsBtn->setEnabled(true);
		}
	}

	void mainDialog::enableDisableObjectSize(int)
	{
		if (screwMotion->isChecked() && plotObjectCheckBox->isChecked() || bezierMotion->isChecked() && plotObjectCheckBox->isChecked() || 
			bsplineApproximation->isChecked() && plotObjectCheckBox->isChecked() || bsplineInterpolation->isChecked() && plotObjectCheckBox->isChecked())
			ObjectSizeChanger->setEnabled(true);
		else
			ObjectSizeChanger->setEnabled(false);
	}

	void mainDialog::enableDisableApproximationSpinBox(int st)
	{
		if (bsplineApproximation->isChecked())
		{
			approximationDegreeSpinBox->setEnabled(true);
		}
		else
		{
			approximationDegreeSpinBox->setEnabled(false);
		}
	}

	void mainDialog::enableDisableInterpoaltionSpinBox(int st)
	{
		if (bsplineInterpolation->isChecked()){
			interpolationDegreeSpinBox->setEnabled(true);
		}
		else
		{
			interpolationDegreeSpinBox->setEnabled(false);
		}
	}

	void mainDialog::enableDisablePositionSelectionSlider(bool st)
	{
		if (continuousCapture->isChecked() )
		{
			positionRecordingSlider->setEnabled(true);
			positionSelectPushBtn->setEnabled(true);
			batchPositionSelectBtn->setEnabled(true);
		}
		else
		{
			positionRecordingSlider->setEnabled(false);
			positionSelectPushBtn->setEnabled(false);
			batchPositionSelectBtn->setEnabled(false);
		}
	}

	void mainDialog::enableDisableBatchPositionSelectionBtn(bool)
	{
		if (LimbMotionCapture->isChecked() || jointMotionCapture->isChecked())
			batchPositionSelectBtn->setEnabled(true);
		else
			batchPositionSelectBtn->setEnabled(false);
	}

	void mainDialog::enableDisableScrewBezierBApprox(bool)
	{
		if (jointMotionCapture->isChecked())
		{
			screwMotion->setEnabled(false);
			bezierMotion->setEnabled(false);
			bsplineApproximation->setEnabled(false);
		}
		else
		{
			screwMotion->setEnabled(true);
			bezierMotion->setEnabled(true);
			bsplineApproximation->setEnabled(true);
		}

	}

	void mainDialog::intermediateReset()
	{
		if (keyPositions->isChecked())
		{
			if (trajectoryCheckBox->isChecked())
			{
				trajectoryCheckBox->setChecked(false);
				trajectoryCheckBox->setEnabled(false);
			}
			if (coordinateFrameCheckBox->isChecked())
			{
				coordinateFrameCheckBox->setChecked(false);
			}

			if (plotObjectCheckBox->isChecked())
			{
				plotObjectCheckBox->setChecked(false);
			}

			approximationDegreeSpinBox->setValue(3);
			interpolationDegreeSpinBox->setValue(3);
			keyPositions->setChecked(false);
			if (keyPositions->isEnabled()){
				keyPositions->setEnabled(false);
			}
		}
		
		
		glWidget->clearKeyGLWidget();
		kinectSensor->clearKeyPositionInSensor();
		kinectSensor->clearKeyFemurPositionInSensor();
		kinectSensor->clearKeyTibiaPositionInSensor();
		kinectSensor->clearKeyLeftUpperArmPositionInSensor();
		kinectSensor->clearKeyLeftLowerArmPositionInSensor();
		kinectSensor->clearKeySpinePositionInSensor();
		kinectSensor->clearKeyLeftHipPositionsInSensor();
		kinectSensor->clearKeyLeftShoulderPositionsInSensor();
		kinectSensor->clearKeyLeftAnklePositionsInSensor();
		kinectSensor->clearKeyRightAnklePositionsInSensor();
		kinectSensor->clearKeyLeftElbowPositionsInSensor();
		//kinectSensor->clearKeyRightElbowPositionsInSensor();
		kinectSensor->clearKeyRightFemurPositionInSensor();
		kinectSensor->clearKeyRightTibiaPositionInSensor();
	}

	void mainDialog::continuousReset()
	{
		if (continuousMotionCheckBox->isChecked())
		{
			if (trajectoryCheckBox->isChecked())
			{
				trajectoryCheckBox->setChecked(false);
			}
			if (coordinateFrameCheckBox->isChecked())
			{
				coordinateFrameCheckBox->setChecked(false);
			}

			if (plotObjectCheckBox->isChecked())
			{
				plotObjectCheckBox->setChecked(false);
			}
			continuousMotionCheckBox->setChecked(false);
		}

		if (limbMotionCheckBox->isChecked())
		{
			if (trajectoryCheckBox->isChecked())
			{
				trajectoryCheckBox->setChecked(false);
			}

			if (plotObjectCheckBox->isChecked())
			{
				plotObjectCheckBox->setChecked(false);
			}
			limbMotionCheckBox->setChecked(false);
		}
		densityChanger->setValue(100);
		emit densityResultDisplayLabel->setText(QString("%1 %").arg(100));
		ObjectSizeChanger->setValue(70);
		emit ObjectSizeDisplayLabel->setText(QString("%1 %").arg(70));

		approximationDegreeSpinBox->setValue(3);
		interpolationDegreeSpinBox->setValue(3);

		if (continuousMotionCheckBox->isEnabled()){
			continuousMotionCheckBox->setEnabled(false);
		}

		glWidget->clearContinuousGLWidget();
		kinectSensor->clearContinuousPositionsInSensor();
	}

	

	/*void mainDialog::saveKeyPositionsPugi(bool st)
	{
		pugi::xml_document doc;
		if (!saveFileName.isEmpty())
			saveFileName.clear();

		const QDateTime now = QDateTime::currentDateTime();
		const QString timestamp = now.toString(QLatin1String("yyyy_MM_dd-hh_mm_ss"));
		const QString dataName = "Dual Quaternion";
		const QString dateTime = QString::fromLatin1("-%1").arg(timestamp);

		saveFileName = QFileDialog::getSaveFileName(this, tr("Save File"), dataName + dateTime, "*.xml");

		pugi::xml_node positions = doc.append_child("Positions");
		if (!saveFileName.isEmpty())
		{
			if (kinectSensor->getKeyPositions().size() > 0)
			{
				vector<DualQuaternion> finalPositions;
				finalPositions = kinectSensor->getKeyPositions();
				
				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Planar_Dual_Quaternion");
						dualQuaternion.append_attribute("RotationX") = finalPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalPositions.at(i).GetReal().q[3];

						//pugi::xml_node dualPart = positions.append_child("Translation Quaternion");
						dualQuaternion.append_attribute("TranslationX") = finalPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalPositions.at(i).GetDual().q[3];
					}
				}
				else
				{
					for (auto i = 0; i < finalPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Spatial_Dual_Quaternion");
						dualQuaternion.append_attribute("RotationX") = finalPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalPositions.at(i).GetReal().q[3];

						//pugi::xml_node dualPart = positions.append_child("Translation Quaternion");
						dualQuaternion.append_attribute("TranslationX") = finalPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalPositions.at(i).GetDual().q[3];
					}
				}
				
			}

			if (kinectSensor->getKeyFemurPositions().size()>0)
			{
				vector<DualQuaternion> finalFemurPositions;
				finalFemurPositions = kinectSensor->getKeyFemurPositions();
				
				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalFemurPositions.size(); i++)
					{
						pugi::xml_node femurdualQuaternion = positions.append_child("Femur_Planar_Dual_Quaternion");
						femurdualQuaternion.append_attribute("RotationX") = finalFemurPositions.at(i).GetReal().q[0];
						femurdualQuaternion.append_attribute("RotationY") = finalFemurPositions.at(i).GetReal().q[1];
						femurdualQuaternion.append_attribute("RotationZ") = finalFemurPositions.at(i).GetReal().q[2];
						femurdualQuaternion.append_attribute("RotationW") = finalFemurPositions.at(i).GetReal().q[3];

						femurdualQuaternion.append_attribute("TranslationX") = finalFemurPositions.at(i).GetDual().q[0];
						femurdualQuaternion.append_attribute("TranslationY") = finalFemurPositions.at(i).GetDual().q[1];
						femurdualQuaternion.append_attribute("TranslationZ") = finalFemurPositions.at(i).GetDual().q[2];
						femurdualQuaternion.append_attribute("TranslationW") = finalFemurPositions.at(i).GetDual().q[3];
					}
				}
				else
				{
					for (auto i = 0; i < finalFemurPositions.size(); i++)
					{
						pugi::xml_node femurdualQuaternion = positions.append_child("Femur_Spatial_Dual_Quaternion");
						femurdualQuaternion.append_attribute("RotationX") = finalFemurPositions.at(i).GetReal().q[0];
						femurdualQuaternion.append_attribute("RotationY") = finalFemurPositions.at(i).GetReal().q[1];
						femurdualQuaternion.append_attribute("RotationZ") = finalFemurPositions.at(i).GetReal().q[2];
						femurdualQuaternion.append_attribute("RotationW") = finalFemurPositions.at(i).GetReal().q[3];

						femurdualQuaternion.append_attribute("TranslationX") = finalFemurPositions.at(i).GetDual().q[0];
						femurdualQuaternion.append_attribute("TranslationY") = finalFemurPositions.at(i).GetDual().q[1];
						femurdualQuaternion.append_attribute("TranslationZ") = finalFemurPositions.at(i).GetDual().q[2];
						femurdualQuaternion.append_attribute("TranslationW") = finalFemurPositions.at(i).GetDual().q[3];
					}
				}
			}

			if (kinectSensor->getKeyTibiaPositions().size() > 0)
			{
				vector<DualQuaternion> finalTibiaPositions;
				finalTibiaPositions = kinectSensor->getKeyTibiaPositions();
				
				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalTibiaPositions.size(); i++)
					{
						pugi::xml_node TibiadualQuaternion = positions.append_child("Tibia_Planar_Dual_Quaternion");
						TibiadualQuaternion.append_attribute("RotationX") = finalTibiaPositions.at(i).GetReal().q[0];
						TibiadualQuaternion.append_attribute("RotationY") = finalTibiaPositions.at(i).GetReal().q[1];
						TibiadualQuaternion.append_attribute("RotationZ") = finalTibiaPositions.at(i).GetReal().q[2];
						TibiadualQuaternion.append_attribute("RotationW") = finalTibiaPositions.at(i).GetReal().q[3];

						TibiadualQuaternion.append_attribute("TranslationX") = finalTibiaPositions.at(i).GetDual().q[0];
						TibiadualQuaternion.append_attribute("TranslationY") = finalTibiaPositions.at(i).GetDual().q[1];
						TibiadualQuaternion.append_attribute("TranslationZ") = finalTibiaPositions.at(i).GetDual().q[2];
						TibiadualQuaternion.append_attribute("TranslationW") = finalTibiaPositions.at(i).GetDual().q[3];
					}
				}
				else
				{
					for (auto i = 0; i < finalTibiaPositions.size(); i++)
					{
						pugi::xml_node TibiadualQuaternion = positions.append_child("Tibia_Spatial_Dual_Quaternion");
						TibiadualQuaternion.append_attribute("RotationX") = finalTibiaPositions.at(i).GetReal().q[0];
						TibiadualQuaternion.append_attribute("RotationY") = finalTibiaPositions.at(i).GetReal().q[1];
						TibiadualQuaternion.append_attribute("RotationZ") = finalTibiaPositions.at(i).GetReal().q[2];
						TibiadualQuaternion.append_attribute("RotationW") = finalTibiaPositions.at(i).GetReal().q[3];

						TibiadualQuaternion.append_attribute("TranslationX") = finalTibiaPositions.at(i).GetDual().q[0];
						TibiadualQuaternion.append_attribute("TranslationY") = finalTibiaPositions.at(i).GetDual().q[1];
						TibiadualQuaternion.append_attribute("TranslationZ") = finalTibiaPositions.at(i).GetDual().q[2];
						TibiadualQuaternion.append_attribute("TranslationW") = finalTibiaPositions.at(i).GetDual().q[3];
					}
				}
			}

			if (kinectSensor->getKeySpinePositions().size() > 0)
			{
				vector<DualQuaternion> finalSpinePositions;
				finalSpinePositions = kinectSensor->getKeySpinePositions();
				
				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalSpinePositions.size(); i++)
					{
						pugi::xml_node SpinedualQuaternion = positions.append_child("Spine_Planar_Dual_Quaternion");
						SpinedualQuaternion.append_attribute("RotationX") = finalSpinePositions.at(i).GetReal().q[0];
						SpinedualQuaternion.append_attribute("RotationY") = finalSpinePositions.at(i).GetReal().q[1];
						SpinedualQuaternion.append_attribute("RotationZ") = finalSpinePositions.at(i).GetReal().q[2];
						SpinedualQuaternion.append_attribute("RotationW") = finalSpinePositions.at(i).GetReal().q[3];

						SpinedualQuaternion.append_attribute("TranslationX") = finalSpinePositions.at(i).GetDual().q[0];
						SpinedualQuaternion.append_attribute("TranslationY") = finalSpinePositions.at(i).GetDual().q[1];
						SpinedualQuaternion.append_attribute("TranslationZ") = finalSpinePositions.at(i).GetDual().q[2];
						SpinedualQuaternion.append_attribute("TranslationW") = finalSpinePositions.at(i).GetDual().q[3];

					}
				}
				else
				{
					for (auto i = 0; i < finalSpinePositions.size(); i++)
					{
						pugi::xml_node SpinedualQuaternion = positions.append_child("Spine_Spatial_Dual_Quaternion");
						SpinedualQuaternion.append_attribute("RotationX") = finalSpinePositions.at(i).GetReal().q[0];
						SpinedualQuaternion.append_attribute("RotationY") = finalSpinePositions.at(i).GetReal().q[1];
						SpinedualQuaternion.append_attribute("RotationZ") = finalSpinePositions.at(i).GetReal().q[2];
						SpinedualQuaternion.append_attribute("RotationW") = finalSpinePositions.at(i).GetReal().q[3];

						SpinedualQuaternion.append_attribute("TranslationX") = finalSpinePositions.at(i).GetDual().q[0];
						SpinedualQuaternion.append_attribute("TranslationY") = finalSpinePositions.at(i).GetDual().q[1];
						SpinedualQuaternion.append_attribute("TranslationZ") = finalSpinePositions.at(i).GetDual().q[2];
						SpinedualQuaternion.append_attribute("TranslationW") = finalSpinePositions.at(i).GetDual().q[3];

					}
				}
			}
			doc.save_file(saveFileName.toStdString().c_str());
		}
	}*/

	void mainDialog::saveContinuousMotionPugi(bool st)
	{
		pugi::xml_document doc;
		if (!saveFileName.isEmpty())
			saveFileName.clear();

		const QDateTime now = QDateTime::currentDateTime();
		const QString timestamp = now.toString(QLatin1String("yyyy_MM_dd-hh_mm_ss"));
		const QString dataName = "Dual Quaternion";
		const QString dateTime = QString::fromLatin1("-%1").arg(timestamp);

		saveFileName = QFileDialog::getSaveFileName(this, tr("Save File"), dataName + dateTime, "*.xml");

		pugi::xml_node positions = doc.append_child("Positions");
		if (!saveFileName.isEmpty())
		{
			if (kinectSensor->getContinuousPositions().size() > 0)
			{
				vector<DualQuaternion> finalPositions;
				finalPositions = kinectSensor->getContinuousPositions();

				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Planar_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalPositions.at(i).GetReal().q[3];

						//pugi::xml_node dualPart = positions.append_child("Translation Quaternion");
						dualQuaternion.append_attribute("TranslationX") = finalPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalPositions.at(i).GetDual().q[3];
					}
				}
				else 
				{
					for (auto i = 0; i < finalPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Spatial_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalPositions.at(i).GetReal().q[3];

						//pugi::xml_node dualPart = positions.append_child("Translation Quaternion");
						dualQuaternion.append_attribute("TranslationX") = finalPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalPositions.at(i).GetDual().q[3];
					}
				}
			}

			if (kinectSensor->getFemurPosesInSensor().size()>0)
			{
				vector<DualQuaternion> finalFemurPositions;
				finalFemurPositions = kinectSensor->getFemurPosesInSensor();

				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalFemurPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Femur_Planar_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalFemurPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalFemurPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalFemurPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalFemurPositions.at(i).GetReal().q[3];

						dualQuaternion.append_attribute("TranslationX") = finalFemurPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalFemurPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalFemurPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalFemurPositions.at(i).GetDual().q[3];
					}
				}
				else
				{
					for (auto i = 0; i < finalFemurPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Femur_Spatial_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalFemurPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalFemurPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalFemurPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalFemurPositions.at(i).GetReal().q[3];

						dualQuaternion.append_attribute("TranslationX") = finalFemurPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalFemurPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalFemurPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalFemurPositions.at(i).GetDual().q[3];
					}
				}
			}

			if (kinectSensor->getTibiaPosesInSensor().size()>0)
			{
				vector<DualQuaternion> finalTibiaPositions;
				finalTibiaPositions = kinectSensor->getTibiaPosesInSensor();

				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalTibiaPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Tibia_Planar_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalTibiaPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalTibiaPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalTibiaPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalTibiaPositions.at(i).GetReal().q[3];

						dualQuaternion.append_attribute("TranslationX") = finalTibiaPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalTibiaPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalTibiaPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalTibiaPositions.at(i).GetDual().q[3];
					}
				}
				else
				{
					for (auto i = 0; i < finalTibiaPositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Tibia_Spatial_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalTibiaPositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalTibiaPositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalTibiaPositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalTibiaPositions.at(i).GetReal().q[3];

						dualQuaternion.append_attribute("TranslationX") = finalTibiaPositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalTibiaPositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalTibiaPositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalTibiaPositions.at(i).GetDual().q[3];
					}
				}
			}

			if (kinectSensor->getSpinePosesInSensor().size()>0)
			{
				vector<DualQuaternion> finalSpinePositions;
				finalSpinePositions = kinectSensor->getSpinePosesInSensor();

				if (planarCapture->isChecked())
				{
					for (auto i = 0; i < finalSpinePositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Spine_Planar_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalSpinePositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalSpinePositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalSpinePositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalSpinePositions.at(i).GetReal().q[3];

						dualQuaternion.append_attribute("TranslationX") = finalSpinePositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalSpinePositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalSpinePositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalSpinePositions.at(i).GetDual().q[3];
					}
				}
				else
				{
					for (auto i = 0; i < finalSpinePositions.size(); i++)
					{
						pugi::xml_node dualQuaternion = positions.append_child("Spine_Spatial_Dual_Quaternion_M");
						dualQuaternion.append_attribute("RotationX") = finalSpinePositions.at(i).GetReal().q[0];
						dualQuaternion.append_attribute("RotationY") = finalSpinePositions.at(i).GetReal().q[1];
						dualQuaternion.append_attribute("RotationZ") = finalSpinePositions.at(i).GetReal().q[2];
						dualQuaternion.append_attribute("RotationW") = finalSpinePositions.at(i).GetReal().q[3];

						dualQuaternion.append_attribute("TranslationX") = finalSpinePositions.at(i).GetDual().q[0];
						dualQuaternion.append_attribute("TranslationY") = finalSpinePositions.at(i).GetDual().q[1];
						dualQuaternion.append_attribute("TranslationZ") = finalSpinePositions.at(i).GetDual().q[2];
						dualQuaternion.append_attribute("TranslationW") = finalSpinePositions.at(i).GetDual().q[3];
					}
				}
			}

			doc.save_file(saveFileName.toStdString().c_str());
		}
	}


	void mainDialog::loadPugiFile(bool st)
	{
		openFileName = QFileDialog::getOpenFileName(this, tr("Open File"),"","*.xml");
		if (openFileName.isEmpty())
			return;

		pugi::xml_document openDoc;
		pugi::xml_parse_result openFileResult = openDoc.load_file(openFileName.toStdString().c_str());
		//qDebug() << "Load Result" << "\t" << openFileResult.description()<<"\n"<<"Mesh Name: "<<openDoc.child("Dual_Quaternion").attribute("Rotation x");
		if (openFileResult)
		{
			vector<DualQuaternion> loadedData;
			int spatialDataFlag = 0;
			int planarDataFlag = 0;
			int keypositionFlag = 0;
			int continuousMotionFlag = 0;
			int limbMotionFlag = 0;
			//if (openDoc.child("Positions"))
			//{
			if (LimbMotionCapture->isChecked())
			{
				if (openDoc.child("Positions"))
				{
					if (openDoc.child("Positions").child("Femur_Planar_Dual_Quaternion"))
					{
						limbSelectionDlg.checkLeftFemur();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Femur_Planar_Dual_Quaternion"); dq; dq = dq.next_sibling("Femur_Planar_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Planar Femur Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setKeyFemurPositions(loadedData);
						if (planarDataFlag == 0)
							planarDataFlag = 1;
						if (keypositionFlag == 0)
							keypositionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Femur_Spatial_Dual_Quaternion"))
					{
						limbSelectionDlg.checkLeftFemur();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Femur_Spatial_Dual_Quaternion"); dq; dq = dq.next_sibling("Femur_Spatial_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spatial Femur Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setKeyFemurPositions(loadedData);
						if (spatialDataFlag == 0)
							spatialDataFlag = 1;
						if (keypositionFlag == 0)
							keypositionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Femur_Planar_Dual_Quaternion_M"))
					{
						limbSelectionDlg.checkLeftFemur();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Femur_Planar_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Femur_Planar_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Planar Femur Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setFemurPosesInSensor(loadedData);
						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);
						if (planarDataFlag == 0)
							planarDataFlag = 1;
						if (limbMotionFlag == 0)
							limbMotionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Femur_Spatial_Dual_Quaternion_M"))
					{
						limbSelectionDlg.checkLeftFemur();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Femur_Spatial_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Femur_Spatial_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spatial Femur Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setFemurPosesInSensor(loadedData);
						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);
						if (spatialDataFlag == 0)
							spatialDataFlag = 1;
						if (limbMotionFlag == 0)
							limbMotionFlag = 1;
					}

					if (openDoc.child("Positions").child("Tibia_Planar_Dual_Quaternion"))
					{
						limbSelectionDlg.checkLeftTibia();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Tibia_Planar_Dual_Quaternion"); dq; dq = dq.next_sibling("Tibia_Planar_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Planar Tibia Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setKeyTibiaPositions(loadedData);
						if (planarDataFlag == 0)
							planarDataFlag = 1;
						if (keypositionFlag == 0)
							keypositionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Tibia_Spatial_Dual_Quaternion"))
					{
						limbSelectionDlg.checkLeftTibia();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Tibia_Spatial_Dual_Quaternion"); dq; dq = dq.next_sibling("Tibia_Spatial_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spatial Tibia Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setKeyTibiaPositions(loadedData);
						if (spatialDataFlag == 0)
							spatialDataFlag = 1;
						if (keypositionFlag == 0)
							keypositionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Tibia_Planar_Dual_Quaternion_M"))
					{
						limbSelectionDlg.checkLeftTibia();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Tibia_Planar_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Tibia_Planar_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Planar Tibia Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setTibiaPosesInSensor(loadedData);
						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);
						if (planarDataFlag == 0)
							planarDataFlag = 1;
						if (limbMotionFlag == 0)
							limbMotionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Tibia_Spatial_Dual_Quaternion_M"))
					{
						limbSelectionDlg.checkLeftTibia();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Tibia_Spatial_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Tibia_Spatial_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spatial Tibia Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setTibiaPosesInSensor(loadedData);
						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);
						if (spatialDataFlag == 0)
							spatialDataFlag = 1;
						if (limbMotionFlag == 0)
							limbMotionFlag = 1;
					}



					if (openDoc.child("Positions").child("Spine_Planar_Dual_Quaternion"))
					{
						limbSelectionDlg.checkSpine();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Spine_Planar_Dual_Quaternion"); dq; dq = dq.next_sibling("Spine_Planar_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spine Planar Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setKeySpinePositions(loadedData);
						if (planarDataFlag == 0)
							planarDataFlag = 1;
						if (keypositionFlag == 0)
							keypositionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Spine_Spatial_Dual_Quaternion"))
					{
						limbSelectionDlg.checkSpine();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Spine_Spatial_Dual_Quaternion"); dq; dq = dq.next_sibling("Spine_Spatial_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spine Spatial Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setKeySpinePositions(loadedData);
						if (spatialDataFlag == 0)
							spatialDataFlag = 1;
						if (keypositionFlag == 0)
							keypositionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Spine_Planar_Dual_Quaternion_M"))
					{
						limbSelectionDlg.checkSpine();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Spine_Planar_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Spine_Planar_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Planar Spine Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setSpinePosesInSensor(loadedData);
						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);
						if (planarDataFlag == 0)
							planarDataFlag = 1;
						if (limbMotionFlag == 0)
							limbMotionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Spine_Spatial_Dual_Quaternion_M"))
					{
						limbSelectionDlg.checkSpine();
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Spine_Spatial_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Spine_Spatial_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spatial Spine Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
							//	dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setSpinePosesInSensor(loadedData);
						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);
						if (spatialDataFlag == 0)
							spatialDataFlag = 1;
						if (limbMotionFlag == 0)
							limbMotionFlag = 1;
					}


					if (keypositionFlag){
						if (!keyPositions->isEnabled())
							keyPositions->setEnabled(true);
						keyPositions->setChecked(true);
					}
					else if (limbMotionFlag){
						if (!limbMotionCheckBox->isEnabled())
							limbMotionCheckBox->setEnabled(true);
						limbMotionCheckBox->setChecked(true);
					}

					if (planarDataFlag)
						planarCapture->setChecked(true);
					else if (spatialDataFlag)
						spatialCapture->setChecked(true);
				}
			}
			else
			{
				if (openDoc.child("Positions"))
				{
					if (openDoc.child("Positions").child("Planar_Dual_Quaternion"))
					{
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Planar_Dual_Quaternion"); dq; dq = dq.next_sibling("Planar_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Planar Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}

						kinectSensor->setKeyPositions(loadedData);

						if (planarDataFlag == 0)
							planarDataFlag = 1;
						if (keypositionFlag == 0)
							keypositionFlag = 1;

					}
					else if (openDoc.child("Positions").child("Spatial_Dual_Quaternion"))
					{
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Spatial_Dual_Quaternion"); dq; dq = dq.next_sibling("Spatial_Dual_Quaternion"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spatial Dual Quaternion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}

						kinectSensor->setKeyPositions(loadedData);

						if (spatialDataFlag == 0)
							spatialDataFlag = 1;
						
						if (keypositionFlag == 0)
							keypositionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Planar_Dual_Quaternion_M"))
					{
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Planar_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Planar_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Planar Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setContinuousMotion(loadedData);

						if (planarDataFlag == 0)
							planarDataFlag = 1;

						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);

						if (continuousMotionFlag == 0)
							continuousMotionFlag = 1;
					}
					else if (openDoc.child("Positions").child("Spatial_Dual_Quaternion_M"))
					{
						loadedData.clear();
						for (pugi::xml_node dq = openDoc.child("Positions").child("Spatial_Dual_Quaternion_M"); dq; dq = dq.next_sibling("Spatial_Dual_Quaternion_M"))
						{
							double Xr, Yr, Zr, Wr;
							double Xt, Yt, Zt, Wt;

							Xr = dq.attribute("RotationX").as_double();
							Yr = dq.attribute("RotationY").as_double();
							Zr = dq.attribute("RotationZ").as_double();
							Wr = dq.attribute("RotationW").as_double();

							Xt = dq.attribute("TranslationX").as_double();
							Yt = dq.attribute("TranslationY").as_double();
							Zt = dq.attribute("TranslationZ").as_double();
							Wt = dq.attribute("TranslationW").as_double();

							Quaternion R(Xr, Yr, Zr, Wr);
							Quaternion T(Xt, Yt, Zt, Wt);

							DualQuaternion dat(R, T);

							//qDebug() << "Checking Spatial Dual Quaternion Motion Data" << "\n" << dat.GetReal().q[0] << "\t" << dat.GetReal().q[1] << "\t" << dat.GetReal().q[2] << "\t" << dat.GetReal().q[3] << "\n" <<
								//dat.GetDual().q[0] << "\t" << dat.GetDual().q[1] << "\t" << dat.GetDual().q[2] << "\t" << dat.GetDual().q[3];

							loadedData.emplace_back(dat);
						}
						kinectSensor->setContinuousMotion(loadedData);

						if (spatialDataFlag == 0)
							spatialDataFlag = 1;

						if (!plotObjectCheckBox->isChecked())
							plotObjectCheckBox->setChecked(true);

						if (continuousMotionFlag == 0)
							continuousMotionFlag = 1;
					}

					if (keypositionFlag){
						if (!keyPositions->isEnabled())
							keyPositions->setEnabled(true);

						keyPositions->setChecked(true);
					}
					else if (continuousMotionFlag){
						if (!continuousMotionCheckBox->isEnabled())
							continuousMotionCheckBox->setEnabled(true);
						continuousMotionCheckBox->setChecked(true);
					}
					

					if (planarDataFlag){
						//planarCapture->setEnabled(true);
						planarCapture->setChecked(true);
					}
					else if (spatialDataFlag){
						spatialCapture->setChecked(true);
					}
				}
			}
		}

	}

	void mainDialog::saveKeyPositionsPugi(bool st)
	{
		pugi::xml_document doc;
		if (!saveFileName.isEmpty())
			saveFileName.clear();

		const QDateTime now = QDateTime::currentDateTime();
		const QString timestamp = now.toString(QLatin1String("yyyy_MM_dd-hh_mm_ss"));
		const QString dataName = "Dual Quaternion";
		const QString dateTime = QString::fromLatin1("-%1").arg(timestamp);

		saveFileName = QFileDialog::getSaveFileName(this, tr("Save File"), dataName + dateTime, "*.xml");

		pugi::xml_node positions = doc.append_child("Positions");
		if (!saveFileName.isEmpty())
		{
			if (kinectSensor->getKeyPositions().size() > 0)
			{
				vector<DualQuaternion> finalPositions;
				finalPositions = kinectSensor->getKeyPositions();
				for (auto i = 0; i < finalPositions.size(); i++)
				{
					pugi::xml_node dualQuaternion = positions.append_child("Dual_Quaternion");
					dualQuaternion.append_attribute("RotationX") = finalPositions.at(i).GetReal().q[0];
					dualQuaternion.append_attribute("RotationY") = finalPositions.at(i).GetReal().q[1];
					dualQuaternion.append_attribute("RotationZ") = finalPositions.at(i).GetReal().q[2];
					dualQuaternion.append_attribute("RotationW") = finalPositions.at(i).GetReal().q[3];

					//pugi::xml_node dualPart = positions.append_child("Translation Quaternion");
					dualQuaternion.append_attribute("TranslationX") = finalPositions.at(i).GetDual().q[0];
					dualQuaternion.append_attribute("TranslationY") = finalPositions.at(i).GetDual().q[1];
					dualQuaternion.append_attribute("TranslationZ") = finalPositions.at(i).GetDual().q[2];
					dualQuaternion.append_attribute("TranslationW") = finalPositions.at(i).GetDual().q[3];
				}
			}
			doc.save_file(saveFileName.toStdString().c_str());
		}
	}

}

