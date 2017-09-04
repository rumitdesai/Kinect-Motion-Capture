#ifndef KINECTSENSOR_H
#define KINECTSENSOR_H

#include <QtCore/QObject>
#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <Qtwidgets/QScrollArea>

#include <Kinect.h>

#include "Math/hMatrix.h"
#include "Math/Quaternion.h"
#include "ControlPosition.h"

static const int cDepthWidth = 512;
static const int cDepthHeight = 424;
static const int cColorWidth = 1920;
static const int cColorHeight = 1080;

namespace Kinect{
	class colorDisplay;
	class colorThread;
	class Sensor:public QObject{
		Q_OBJECT
	public:
		Sensor(QLabel* parentColorLabel, QLabel* parentSkeletonLabel, QObject* parent = NULL);
		HRESULT initializeSensor();
		//void closeSensor();
		void processSkeletonData(int BodyCount, IBody** ppBodies);
		QPoint kinectFrameToScreenFrame(const CameraSpacePoint& bodyPoint, int width, int height);
		QVector3D cameraSpaceToQVector3D(CameraSpacePoint joint);
		void drawSkeleton(const Joint* joints, const QPoint* jointData);
		void setKeyPositions(vector<DualQuaternion> pos);  // For loading files 
		void setContinuousMotion(vector<DualQuaternion>pos); // for loading continuous motion
		


		vector<DualQuaternion> getKeyPositions();
		vector<DualQuaternion> getContinuousPositions();
		void addNewKeyCtrlPos(DualQuaternion);
		void clearKeyPositionInSensor();
		void clearContinuousPositionsInSensor();

		// ********************* Limb Motion related member functions ****************************//
		vector<DualQuaternion> getFemurPosesInSensor();        // equivalent to getContinuousPositions() but for limbs
		vector<DualQuaternion> getTibiaPosesInSensor();        // equivalent to getContinuousPositions() but for limbs
		vector<DualQuaternion> getLeftUpperArmPosesInSensor();
		vector<DualQuaternion> getLeftLowerArmPosesInSensor();
		vector<DualQuaternion> getSpinePosesInSensor();
		vector<DualQuaternion> getRightFemurPosesInSensor();
		vector<DualQuaternion> getRightTibiaPosesInSensor();
		vector<DualQuaternion> getRightUpperArmPosesInSensor();
		vector<DualQuaternion> getRightLowerArmPosesInSensor();

		void setFemurPosesInSensor(vector<DualQuaternion>pos);
		void setTibiaPosesInSensor(vector<DualQuaternion>pos);
		void setSpinePosesInSensor(vector<DualQuaternion>pos);

		vector<DualQuaternion> getKeyFemurPositions();
		vector<DualQuaternion> getKeyTibiaPositions();
		vector<DualQuaternion> getKeyLeftUpperArmPositions();
		vector<DualQuaternion> getKeyLeftLowerArmPositions();
		vector<DualQuaternion> getKeySpinePositions();
		vector<DualQuaternion> getKeyRightFemurPositions();
		vector<DualQuaternion> getKeyRightTibiaPositions();
		vector<DualQuaternion> getKeyRightUpperArmPositions();
		vector<DualQuaternion> getKeyRightLowerArmPositions();

		vector<float> getFemurLengthInSensor();
		vector<float> getTibiaLengthInSensor();
		vector<float> getLeftUpperArmLengthInSensor();
		vector<float> getLeftLowerArmLengthInSensor();
		vector<float> getSpineLengthInSensor();
		vector<float> getRightFemurLengthInSensor();
		vector<float> getRightTibiaLengthInSensor();
		vector<float> getRightUpperArmLengthInSensor();
		vector<float> getRightLowerArmLengthInSensor();

		vector<QVector3D> getFemurStart();
		vector<QVector3D> getFemurEnd();
		vector<QVector3D> getTibiaStart();
		vector<QVector3D> getTibiaEnd();
		vector<QVector3D> getLeftUpperArmStart();
		vector<QVector3D> getLeftUpperArmEnd();
		vector<QVector3D> getLeftLowerArmStart();
		vector<QVector3D> getLeftLowerArmEnd();
		vector<QVector3D> getSpineStart();
		vector<QVector3D> getSpineEnd();
		vector<QVector3D> getRightFemurStart();
		vector<QVector3D> getRightFemurEnd();
		vector<QVector3D> getRightTibiaStart();
		vector<QVector3D> getRightTibiaEnd();
		vector<QVector3D> getRightUpperArmStart();
		vector<QVector3D> getRightUpperArmEnd();
		vector<QVector3D> getRightLowerArmStart();
		vector<QVector3D> getRightLowerArmEnd();

		void addNewKeyFemurPos(DualQuaternion);
		void addNewKeyTibiaPos(DualQuaternion);
		void addNewKeyLeftUpperArmPos(DualQuaternion);
		void addNewKeyLeftLowerArmPos(DualQuaternion);
		void addNewKeySpinePos(DualQuaternion);
		void addNewKeyRightFemurPos(DualQuaternion);
		void addNewKeyRightTibiaPos(DualQuaternion);
		void addNewKeyRightUpperArmPos(DualQuaternion);
		void addNewKeyRightLowerArmPos(DualQuaternion);
		
		void clearKeyFemurPositionInSensor();
		void clearKeyTibiaPositionInSensor();
		void clearKeyLeftUpperArmPositionInSensor();
		void clearKeyLeftLowerArmPositionInSensor();
		void clearKeySpinePositionInSensor();
		void clearKeyRightFemurPositionInSensor();
		void clearKeyRightTibiaPositionInSensor();
		void clearKeyRightUpperArmPositionInSensor();
		void clearKeyRightLowerArmPositionInSensor();

		void setKeyFemurPositions(vector<DualQuaternion> pos);
		void setKeyTibiaPositions(vector<DualQuaternion> pos);
		void setKeySpinePositions(vector<DualQuaternion> pos);
		// ********************************************* Joint capture related functions ********************************//

		// *************** Left Hip Functions ****************//
		vector<DualQuaternion> getKeyLeftHipPositions();
		vector<DualQuaternion> getContinuousLeftHipPositions();
		void addNewLeftHipKeyCtrlPos(DualQuaternion);
		void clearKeyLeftHipPositionsInSensor();		
		
		// *************** Left Shoulder Functions ****************//
		vector<DualQuaternion> getKeyLeftShoulderPositions();
		vector<DualQuaternion> getContinuousLeftShoulderPositions();
		void addNewLeftShoulderKeyCtrlPos(DualQuaternion);
		void clearKeyLeftShoulderPositionsInSensor();

		// *************** Left Ankle Functions ****************//
		vector<DualQuaternion> getKeyLeftAnklePositions();
		vector<DualQuaternion> getContinuousLeftAnklePositions();
		void addNewLeftAnkleKeyCtrlPos(DualQuaternion);
		void clearKeyLeftAnklePositionsInSensor();

		// *************** Right Ankle Functions ****************//
		vector<DualQuaternion> getKeyRightAnklePositions();
		vector<DualQuaternion> getContinuousRightAnklePositions();
		void addNewRightAnkleKeyCtrlPos(DualQuaternion);
		void clearKeyRightAnklePositionsInSensor();

		// *************** Left Elbow Functions ****************//
		vector<DualQuaternion> getKeyLeftElbowPositions();
		vector<DualQuaternion> getContinuousLeftElbowPositions();
		void addNewLeftElbowKeyCtrlPos(DualQuaternion);
		void clearKeyLeftElbowPositionsInSensor();

		// *************** Right Elbow Functions ****************//
		//vector<DualQuaternion> getKeyRightElbowPositions();
		//vector<DualQuaternion> getContinuousRightElbowPositions();
		//void addNewRightElbowKeyCtrlPos(DualQuaternion);
		//void clearKeyRightElbowPositionsInSensor();

		~Sensor();
	
	public slots:
		void updateColorFrame();
		void updateSkeletonFrame();
		void StoreKeyPosition(bool);
		
		//void motionRecordingChecked(int);

	//signals:
		//void debuggingCheck(QString);
		//void clearDebuggingCheck(QString);
	private:
		IKinectSensor* m_pKinectSensor;
		IColorFrameReader* m_pColorFrameReader;
		ICoordinateMapper* m_pCoordinateMapper;
		IBodyFrameReader* m_pBodyFrameReader;
		RGBQUAD* m_pColorRGBX;
		bool m_pKinectInitialized;

		
		colorDisplay* m_pcolorFrame;
		QLabel* colorFrameLabel;
		QLabel* skeletonFrameLabel;
		QPixmap* skeletonPixmap;
		QPainter* skeletonPainter;

		ControlPosition* keyPositions;
		ControlPosition* continuousPositions;
		Quaternion G_previous_frame, H_previous_frame;

		
		QVector3D m_spineShoulder3D, m_spineMid3D,m_head3D,m_neck3D,m_spineBase3D;

		QVector3D m_leftShoulder3D, m_leftElbow3D, m_leftWrsit3D, m_leftHand3D, m_leftHandtip3D, m_leftHandthumb3D;	 // 30
		QVector3D m_leftHip3D, m_leftKnee3D, m_leftAnkle3D, m_leftFoot3D;
		
		QVector3D m_rightShoulder3D, m_rightElbow3D, m_rightWrist3D, m_rightHand3D, m_rightHandTip3D,m_rightHandThumb3D;
		QVector3D  m_rightHip3D, m_rightKnee3D, m_rightAnkle3D, m_rightFoot3D;

		// ******* Left Ankle Control Position *****//
		ControlPosition* keyLeftAnklePositions;
		ControlPosition* continuousLeftAnklePositions;
		
		// ******* Right Ankle Control Position *****//
		ControlPosition* keyRightAnklePositions;
		ControlPosition* continuousRightAnklePositions;

		// ******* Left Elbow Control Position *****//
		ControlPosition* keyLeftElbowPositions;
		ControlPosition* continuousLeftElbowPositions;

		// ******* Right Elbow Control Position *****//
		//ControlPosition* keyRightElbowPositions;
		//ControlPosition* continuousRightElbowPositions;

		// ******* Left Shoulder Control Position *****//
		ControlPosition* keyLeftShoulderPositions;
		ControlPosition* continuousLeftShoulderPositions;

		// ******* Left Hip Control Position ******//
		ControlPosition* keyLeftHipPositions;
		ControlPosition* continuousLeftHipPositions;
		

		//******** For Limb Positions ************//
		LimbPosition* LeftFemurPositions;
		LimbPosition* LeftTibiaPositions;
		LimbPosition* LeftUpperArmPositions;
		LimbPosition* LeftLowerArmPositions;
		LimbPosition* spinePositions;
		LimbPosition* RightFemurPositions;
		LimbPosition* RightTibiaPositions;
		LimbPosition* RightUpperArmPositions;
		LimbPosition* RightLowerArmPositions;

		ControlPosition* keyLeftFemurPositions;
		ControlPosition* keyLeftTibiaPositions;
		ControlPosition* keyLeftUpperArmPositions;
		ControlPosition* keyLeftLowerArmPositions;
		ControlPosition* keySpinePositions;
		ControlPosition* keyRightFemurPositions;
		ControlPosition* keyRightTibiaPositions;
		ControlPosition* keyRightLowerArmPositions;
		ControlPosition* keyRightUpperArmPositions;
		//float lengthOfThigh;
		//float lengthOfLowerLeg;
		//float lengthOfSpine;
	};

}


#endif