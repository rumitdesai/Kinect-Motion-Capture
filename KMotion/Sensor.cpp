#include <QtWidgets/QWidget>
#include <QtCore/QDebug>
#include <QtWidgets/QSizePolicy>
#include <QtGui/QPixmap>
#include <QtGui/QPainter>
#include <QtCore/QPoint>
#include <QtGui/QVector3D>
#include "Sensor.h"
#include "maindialog.h"
#include "colorDisplay.h"
#include "motion.h"


int frame_count = 0;
int flag = 0;
extern Kinect::mainDialog* mainWinPtr;
 

template<typename Interface>
void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}



namespace Kinect{
	Sensor::Sensor(QLabel* parentColorLabel, QLabel* parentSkeletonLabel, QObject* parent) : m_pKinectInitialized(false), m_pColorRGBX(NULL)
	{
		colorFrameLabel = parentColorLabel;
		skeletonFrameLabel = parentSkeletonLabel;
		skeletonPixmap = NULL;
		skeletonPainter = NULL;

		keyPositions = new ControlPosition;
		continuousPositions = new ControlPosition;

		// ******************** Limb related class initialization ***********************//
		keyLeftFemurPositions = new ControlPosition;
		keyLeftTibiaPositions = new ControlPosition;
		keyLeftUpperArmPositions = new ControlPosition;
		keyLeftLowerArmPositions = new ControlPosition;
		keySpinePositions = new ControlPosition;
		keyRightFemurPositions = new ControlPosition;
		keyRightTibiaPositions = new ControlPosition;
		keyRightUpperArmPositions = new ControlPosition;
		keyRightLowerArmPositions = new ControlPosition;

		LeftFemurPositions = new LimbPosition;
		LeftTibiaPositions = new LimbPosition;
		LeftUpperArmPositions = new LimbPosition;
		LeftLowerArmPositions = new LimbPosition;
		spinePositions = new LimbPosition;
		RightFemurPositions = new LimbPosition;
		RightTibiaPositions = new LimbPosition;
		RightUpperArmPositions = new LimbPosition;
		RightLowerArmPositions = new LimbPosition;

		keyLeftHipPositions = new ControlPosition;
		continuousLeftHipPositions = new ControlPosition;
		
		keyLeftShoulderPositions = new ControlPosition;
		continuousLeftShoulderPositions = new ControlPosition;
		
		keyLeftAnklePositions = new ControlPosition;
		continuousLeftAnklePositions = new ControlPosition;
		
		keyRightAnklePositions = new ControlPosition;
		continuousRightAnklePositions = new ControlPosition;

		keyLeftElbowPositions = new ControlPosition;
		continuousLeftElbowPositions = new ControlPosition;

		//keyRightElbowPositions = new ControlPosition;
		//continuousRightElbowPositions = new ControlPosition;

		initializeSensor();
		
		m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
		
	}

	HRESULT Sensor::initializeSensor()
	{
		HRESULT hr;
		/*if (m_pKinectInitialized)
			return hr;*/
		hr = GetDefaultKinectSensor(&m_pKinectSensor);

		if (FAILED(hr))
			return hr;

		if (m_pKinectSensor)
		{
			IColorFrameSource* pColorFrameSource = NULL;
			IBodyFrameSource* pBodyFrameSource = NULL;

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
			}
			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
				hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
			}
			if (SUCCEEDED(hr))
			{
				hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
				hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
			}
			
			m_pcolorFrame = new colorDisplay(colorFrameLabel, cColorWidth, cColorHeight, this);

			m_pKinectInitialized = true;

		}
		if (!m_pKinectSensor || FAILED(hr))
		{
			qDebug() << "Sensor not working";
		}
		return hr;
	}
	
	void Sensor::processSkeletonData(int BodyCount, IBody** ppBodies)
	{
		HRESULT hr;
		if (m_pCoordinateMapper)
		{
			int width = skeletonFrameLabel->width();
			int height = skeletonFrameLabel->height();

			for (auto i = 0; i < BodyCount; ++i)
			{
				IBody* pBody = ppBodies[i];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					if (SUCCEEDED(hr) && bTracked)
					{
						Joint joints[JointType_Count];
						QPoint jointPoints[JointType_Count];
						HandState leftHandState = HandState_Unknown;
						HandState rightHandState = HandState_Unknown;

						pBody->get_HandLeftState(&leftHandState);
						pBody->get_HandRightState(&rightHandState);

						hr = pBody->GetJoints(_countof(joints), joints);
						if (SUCCEEDED(hr))
						{
							//cursorsInGL.clear();
							hMatrix tempCursor;
							for (auto j = 0; j < _countof(joints); j++)
							{
								jointPoints[j] = kinectFrameToScreenFrame(joints[j].Position, width, height);

								switch (joints[j].JointType)
								{
								case JointType_ShoulderLeft:
									m_leftShoulder3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_ElbowLeft:
									m_leftElbow3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_WristLeft:
									m_leftWrsit3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_HandLeft:
									m_leftHand3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_HandTipLeft:
									m_leftHandtip3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_ThumbLeft:
									m_leftHandthumb3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_HipLeft:
									m_leftHip3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_KneeLeft:
									m_leftKnee3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_AnkleLeft:
									m_leftAnkle3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_FootLeft:
									m_leftFoot3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_Head:
									m_head3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_Neck:
									m_neck3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_SpineShoulder:
									m_spineShoulder3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_SpineMid:
									m_spineMid3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_SpineBase:
									m_spineBase3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_ShoulderRight:
									m_rightShoulder3D == cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_ElbowRight:
									m_rightElbow3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_WristRight:
									m_rightWrist3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_HandRight:
									m_rightHand3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_HandTipRight:
									m_rightHandTip3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_ThumbRight:
									m_rightHandThumb3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_HipRight:
									m_rightHip3D = cameraSpaceToQVector3D(joints[j].Position);;
									break;
								case JointType_KneeRight:
									m_rightKnee3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_AnkleRight:
									m_rightAnkle3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								case JointType_FootRight:
									m_rightFoot3D = cameraSpaceToQVector3D(joints[j].Position);
									break;
								}
							}
				
							drawSkeleton(joints, jointPoints);
							
							
							/*if (mainWinPtr->skeletonMotionCheckBox->isChecked()){
								emit mainWinPtr->skeletonMotionCheckBox->stateChanged(2);
							}*/

							if (!mainWinPtr->startRecording->isChecked())
							{
								if (rightHandState==HandState_Closed /*&& flag==0*/)
								{
									//qDebug() << "Checking the handstate";
									//flag = 1;
									//emit debuggingCheck("Close: Recording");
									mainWinPtr->startRecording->setChecked(true);
									if (!mainWinPtr->keyPositions->isChecked())
									{
										mainWinPtr->keyPositions->setChecked(true);
									}
									else
									{
										mainWinPtr->keyPositions->stateChanged(2);
									}
								}
							}
							else if (mainWinPtr->startRecording->isChecked())
							{
								if (rightHandState == HandState_Open /*&& flag == 1*/)
								{
									//flag = 0;
									mainWinPtr->startRecording->setChecked(false);
									//emit clearDebuggingCheck("Open: No Recording");
								}

							} // comment here

							if (mainWinPtr->startRecording->isChecked()) // double commenting
							{ 
								if (mainWinPtr->continuousCapture->isChecked())
								{
									if (mainWinPtr->spatialCapture->isChecked())
									{
										continuousPositions->addCtrlPos(m_leftHand3D, m_leftHandtip3D, m_leftHandthumb3D, CAPTURETYPE(0));
										//continuousPositions->addCtrlPos(m_shoulder3D);
										//continuousHipPositions->addCtrlPos(m_hip3D);
										//qDebug() << "\n";
									}
									else if (mainWinPtr->planarCapture->isChecked())
									{
										continuousPositions->addCtrlPos(m_leftHand3D, m_leftHandtip3D, m_leftHandthumb3D, CAPTURETYPE(1));
										///continuousPositions->addCtrlPos(m_shoulder3D);
										///continuousHipPositions->addCtrlPos(m_hip3D);
										//qDebug() << "\n";
									}

									
									if (!mainWinPtr->continuousMotionCheckBox->isChecked() )
									{
										mainWinPtr->continuousMotionCheckBox->setChecked(true);
										
										if (!mainWinPtr->coordinateFrameCheckBox->isChecked())
										{
											mainWinPtr->coordinateFrameCheckBox->setChecked(true);
										}
									}
									else
									{
										emit mainWinPtr->continuousMotionCheckBox->stateChanged(2);
									}
								}
								else if (mainWinPtr->keyPositionCapture->isChecked())
								{
									hMatrix currentMatrix; hMatrix auxMatrix;
									if (mainWinPtr->spatialCapture->isChecked())
									{
										currentMatrix = auxMatrix.homogeneousMatrix(m_leftHand3D, m_leftHandtip3D, m_leftHandthumb3D, CAPTURETYPE(0));
									}
									else if (mainWinPtr->planarCapture->isChecked())
									{
										currentMatrix = auxMatrix.homogeneousMatrix(m_leftHand3D, m_leftHandtip3D, m_leftHandthumb3D, CAPTURETYPE(1));
									}

									Quaternion tempQuaternion; hMatrix temphMatrix;
									Quaternion G = tempQuaternion.calculateG(temphMatrix.rotationMatrixToQuaternionEigen(currentMatrix), m_leftHand3D, 20.0);
									Quaternion H = tempQuaternion.calculateH(temphMatrix.rotationMatrixToQuaternionEigen(currentMatrix), m_leftHand3D, 20.0);
									
									double difference = sqrt((G_previous_frame - G).dot(G_previous_frame - G) + (H_previous_frame - H).dot(H_previous_frame - H));
									if (difference < 0.2)
									{
										frame_count++;
										if (frame_count == 100)
										{
											if (mainWinPtr->spatialCapture->isChecked())
											{
												keyPositions->addCtrlPos(currentMatrix,CAPTURETYPE(0));
											}
											else if (mainWinPtr->planarCapture->isChecked())
											{
												keyPositions->addCtrlPos(currentMatrix, CAPTURETYPE(1));
											}

											frame_count = 0;
											if (!mainWinPtr->keyPositions->isChecked())
											{
												mainWinPtr->keyPositions->setChecked(true);
											}
											else
											{
												emit mainWinPtr->keyPositions->stateChanged(2);
											}
											
										}
									}
									else
									{
										frame_count = 0;
										G_previous_frame = G;
										H_previous_frame = H;
									}
								}
								else if (mainWinPtr->LimbMotionCapture->isChecked())
								{
									if (mainWinPtr->spatialCapture->isChecked())
									{
										//lengthOfThigh = (m_leftKnee3D - m_leftHip3D).length();
										//lengthOfLowerLeg = (m_leftAnkle3D - m_leftKnee3D).length();
										//lengthOfSpine = (m_spineMid3D - m_spineShoulder3D).length();
										
										//qDebug() << "Checking hip Position" << "\t" << m_hip3D;
										//qDebug() << "Checking Knee Position" << "\t" << m_knee3D;
										//qDebug() << "Checking Ankle Position" << "\t" << m_ankle3D;
										//qDebug() << "Checking SpineMID PSOITION" << "\t" << m_spineMid3D;
										//qDebug() << "Length of Thigh limb" << "\t" << (m_knee3D - m_hip3D).length();
										//qDebug() << "\n";
										//femurPositions->addLimbLength(lengthOfThigh);
										//tibiaPositions->addLimbLength(lengthOfLowerLeg);

										if (mainWinPtr->limbSelectionDlg.isLeftFemurChecked())
										{
											LeftFemurPositions->addLimbLength(m_leftHip3D, m_leftKnee3D, CAPTURETYPE(0));
											LeftFemurPositions->setStartAndEndOfLimb(m_leftHip3D, m_leftKnee3D, CAPTURETYPE(0));
											LeftFemurPositions->addLimbPos(m_leftHip3D, m_leftKnee3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isLeftTibiaChecked())
										{
											LeftTibiaPositions->addLimbLength(m_leftKnee3D, m_leftAnkle3D, CAPTURETYPE(0));
											LeftTibiaPositions->setStartAndEndOfLimb(m_leftKnee3D, m_leftAnkle3D, CAPTURETYPE(0));
											LeftTibiaPositions->addLimbPos(m_leftKnee3D, m_leftAnkle3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isLeftUpperArmChecked())
										{
											LeftUpperArmPositions->addLimbLength(m_leftShoulder3D, m_leftElbow3D, CAPTURETYPE(0));
											LeftUpperArmPositions->setStartAndEndOfLimb(m_leftShoulder3D, m_leftElbow3D, CAPTURETYPE(0));
											LeftUpperArmPositions->addLimbPos(m_leftShoulder3D, m_leftElbow3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isLeftLowerArmChecked())
										{
											LeftLowerArmPositions->addLimbLength(m_leftElbow3D, m_leftWrsit3D, CAPTURETYPE(0));
											LeftLowerArmPositions->setStartAndEndOfLimb(m_leftElbow3D, m_leftWrsit3D, CAPTURETYPE(0));
											LeftLowerArmPositions->addLimbPos(m_leftElbow3D, m_leftWrsit3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isSpineChecked())
										{
											spinePositions->addLimbLength(m_spineShoulder3D, m_spineMid3D, CAPTURETYPE(0));
											spinePositions->setStartAndEndOfLimb(m_spineShoulder3D, m_spineMid3D, CAPTURETYPE(0));
											spinePositions->addLimbPos(m_spineShoulder3D, m_spineMid3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isRightFemurChecked())
										{
											RightFemurPositions->addLimbLength(m_rightHip3D, m_rightKnee3D, CAPTURETYPE(0));
											RightFemurPositions->setStartAndEndOfLimb(m_rightHip3D, m_rightKnee3D, CAPTURETYPE(0));
											RightFemurPositions->addLimbPos(m_rightHip3D, m_rightKnee3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isRightTibiaChecked())
										{
											RightTibiaPositions->addLimbLength(m_rightKnee3D, m_rightAnkle3D, CAPTURETYPE(0));
											RightTibiaPositions->setStartAndEndOfLimb(m_rightKnee3D, m_rightAnkle3D, CAPTURETYPE(0));
											RightTibiaPositions->addLimbPos(m_rightKnee3D, m_rightAnkle3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isRightUpperArmChecked())
										{
											RightUpperArmPositions->addLimbLength(m_rightShoulder3D, m_rightElbow3D, CAPTURETYPE(0));
											RightUpperArmPositions->setStartAndEndOfLimb(m_rightShoulder3D, m_rightElbow3D, CAPTURETYPE(0));
											RightUpperArmPositions->addLimbPos(m_rightShoulder3D, m_rightElbow3D, CAPTURETYPE(0));
										}
										if (mainWinPtr->limbSelectionDlg.isRightLowerArmChecked())
										{
											RightLowerArmPositions->addLimbLength(m_rightElbow3D, m_rightWrist3D, CAPTURETYPE(0));
											RightLowerArmPositions->setStartAndEndOfLimb(m_rightElbow3D, m_rightWrist3D, CAPTURETYPE(0));
											RightLowerArmPositions->addLimbPos(m_rightElbow3D, m_rightWrist3D, CAPTURETYPE(0));
										}

										
									}
									else if (mainWinPtr->planarCapture->isChecked())
									{
										if (mainWinPtr->limbSelectionDlg.isLeftFemurChecked())
										{
											LeftFemurPositions->addLimbLength(m_leftHip3D, m_leftKnee3D, CAPTURETYPE(1));
											LeftFemurPositions->setStartAndEndOfLimb(m_leftHip3D, m_leftKnee3D, CAPTURETYPE(1));
											LeftFemurPositions->addLimbPos(m_leftHip3D, m_leftKnee3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isLeftTibiaChecked())
										{
											LeftTibiaPositions->addLimbLength(m_leftKnee3D, m_leftAnkle3D, CAPTURETYPE(1));
											LeftTibiaPositions->setStartAndEndOfLimb(m_leftKnee3D, m_leftAnkle3D, CAPTURETYPE(1));
											LeftTibiaPositions->addLimbPos(m_leftKnee3D, m_leftAnkle3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isLeftUpperArmChecked())
										{
											LeftUpperArmPositions->addLimbLength(m_leftShoulder3D, m_leftElbow3D, CAPTURETYPE(1));
											LeftUpperArmPositions->setStartAndEndOfLimb(m_leftShoulder3D, m_leftElbow3D, CAPTURETYPE(1));
											LeftUpperArmPositions->addLimbPos(m_leftShoulder3D, m_leftElbow3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isLeftLowerArmChecked())
										{
											LeftLowerArmPositions->addLimbLength(m_leftElbow3D, m_leftWrsit3D, CAPTURETYPE(1));
											LeftLowerArmPositions->setStartAndEndOfLimb(m_leftElbow3D, m_leftWrsit3D, CAPTURETYPE(1));
											LeftLowerArmPositions->addLimbPos(m_leftElbow3D, m_leftWrsit3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isSpineChecked())
										{
											spinePositions->addLimbLength(m_spineShoulder3D, m_spineMid3D, CAPTURETYPE(1));
											spinePositions->setStartAndEndOfLimb(m_spineShoulder3D, m_spineMid3D, CAPTURETYPE(1));
											spinePositions->addLimbPos(m_spineShoulder3D, m_spineMid3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isRightFemurChecked())
										{
											RightFemurPositions->addLimbLength(m_rightHip3D, m_rightKnee3D, CAPTURETYPE(1));
											RightFemurPositions->setStartAndEndOfLimb(m_rightHip3D, m_rightKnee3D, CAPTURETYPE(1));
											RightFemurPositions->addLimbPos(m_rightHip3D, m_rightKnee3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isRightTibiaChecked())
										{
											RightTibiaPositions->addLimbLength(m_rightKnee3D, m_rightAnkle3D, CAPTURETYPE(1));
											RightTibiaPositions->setStartAndEndOfLimb(m_rightKnee3D, m_rightAnkle3D, CAPTURETYPE(1));
											RightTibiaPositions->addLimbPos(m_rightKnee3D, m_rightAnkle3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isRightUpperArmChecked())
										{
											RightUpperArmPositions->addLimbLength(m_rightShoulder3D, m_rightElbow3D, CAPTURETYPE(1));
											RightUpperArmPositions->setStartAndEndOfLimb(m_rightShoulder3D, m_rightElbow3D, CAPTURETYPE(1));
											RightUpperArmPositions->addLimbPos(m_rightShoulder3D, m_rightElbow3D, CAPTURETYPE(1));
										}
										if (mainWinPtr->limbSelectionDlg.isRightLowerArmChecked())
										{
											RightLowerArmPositions->addLimbLength(m_rightElbow3D, m_rightWrist3D, CAPTURETYPE(1));
											RightLowerArmPositions->setStartAndEndOfLimb(m_rightElbow3D, m_rightWrist3D, CAPTURETYPE(1));
											RightLowerArmPositions->addLimbPos(m_rightElbow3D, m_rightWrist3D, CAPTURETYPE(1));
										}
										
									}
									
									if (!mainWinPtr->limbMotionCheckBox->isChecked())
									{
										mainWinPtr->limbMotionCheckBox->setChecked(true);

										if (!mainWinPtr->plotObjectCheckBox->isChecked())
											mainWinPtr->plotObjectCheckBox->setChecked(true);
									}
									else
									{
										emit mainWinPtr->limbMotionCheckBox->stateChanged(2);
									}

								}
								else if (mainWinPtr->jointMotionCapture->isChecked())
								{
									if (mainWinPtr->planarCapture->isChecked())
									{
										if (mainWinPtr->jointSelectionDlg.isLeftHipchecked())
											continuousLeftHipPositions->addCtrlPos(m_leftHip3D, CAPTURETYPE(1));
										if (mainWinPtr->jointSelectionDlg.isLeftShoulderchecked())
											continuousLeftShoulderPositions->addCtrlPos(m_leftShoulder3D, CAPTURETYPE(1));
										if (mainWinPtr->jointSelectionDlg.isLeftAnkleChecked())
											continuousLeftAnklePositions->addCtrlPos(m_leftAnkle3D, CAPTURETYPE(1));
										if (mainWinPtr->jointSelectionDlg.isRightAnkleChecked())
											continuousRightAnklePositions->addCtrlPos(m_rightAnkle3D, CAPTURETYPE(1));
										if (mainWinPtr->jointSelectionDlg.isLeftElbowChecked())
											continuousLeftElbowPositions->addCtrlPos(m_leftElbow3D, CAPTURETYPE(1));
										//if (mainWinPtr->jointSelectionDlg.isRightElbowchecked())
											//continuousRightElbowPositions->addCtrlPos(m_rightElbow3D, CAPTURETYPE(1));
									}
									else if (mainWinPtr->spatialCapture->isChecked())
									{
										if (mainWinPtr->jointSelectionDlg.isLeftHipchecked())
											continuousLeftHipPositions->addCtrlPos(m_leftHip3D, CAPTURETYPE(0));
										if (mainWinPtr->jointSelectionDlg.isLeftShoulderchecked())
											continuousLeftShoulderPositions->addCtrlPos(m_leftShoulder3D, CAPTURETYPE(0));
										if (mainWinPtr->jointSelectionDlg.isLeftAnkleChecked())
											continuousLeftAnklePositions->addCtrlPos(m_leftAnkle3D, CAPTURETYPE(0));
										if (mainWinPtr->jointSelectionDlg.isRightAnkleChecked())
											continuousRightAnklePositions->addCtrlPos(m_rightAnkle3D, CAPTURETYPE(0));
										if (mainWinPtr->jointSelectionDlg.isLeftElbowChecked())
											continuousLeftElbowPositions->addCtrlPos(m_leftElbow3D, CAPTURETYPE(0));
										//if (mainWinPtr->jointSelectionDlg.isRightElbowchecked())
											//continuousRightElbowPositions->addCtrlPos(m_rightElbow3D, CAPTURETYPE(0));
									}


									if (!mainWinPtr->continuousMotionCheckBox->isChecked()){
										mainWinPtr->continuousMotionCheckBox->setChecked(true);
										if (!mainWinPtr->plotObjectCheckBox->isChecked())
										{
											mainWinPtr->plotObjectCheckBox->setChecked(true);
										}
									}
									else{
										emit mainWinPtr->continuousMotionCheckBox->stateChanged(2);
									}
								}
							}
						}
					}
				}
			}
		}
	}
	QPoint Sensor::kinectFrameToScreenFrame(const CameraSpacePoint& bodyPoint, int width, int height)
	{
		QPoint result;
		DepthSpacePoint depthPoint = { 0 };
		m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

		result.setX(static_cast<float>(depthPoint.X*width) / cDepthWidth);
		result.setY(static_cast<float>(depthPoint.Y*height) / cDepthHeight);

		return result;
	}
	QVector3D Sensor::cameraSpaceToQVector3D(CameraSpacePoint joint)
	{
		QVector3D result;
		result.setX(joint.X);
		result.setY(joint.Y);
		result.setZ(joint.Z);
		return result;
	}
	void Sensor::drawSkeleton(const Joint* joints, const QPoint* jointData)
	{
		//skeletonPixmap = new QPixmap(skeletonScrollArea->width(), skeletonScrollArea->height());
		
		//mainWinPtr->skeletonScrollArea->size().width()
		//skeletonPixmap = new QPixmap(938,438);
		//skeletonPixmap = new QPixmap(626, 348);
		skeletonPixmap = new QPixmap(mainWinPtr->skeletonScrollArea->width(), mainWinPtr->skeletonScrollArea->height());
		skeletonPixmap->fill(Qt::black);
		skeletonPainter = new QPainter(skeletonPixmap);
		skeletonPainter->setPen(Qt::green);
		
		skeletonPainter->drawLine(jointData[15], jointData[14]);
		skeletonPainter->drawLine(jointData[14], jointData[13]);
		skeletonPainter->drawLine(jointData[13], jointData[12]);
		skeletonPainter->drawLine(jointData[12], jointData[0]);
		skeletonPainter->drawLine(jointData[0], jointData[16]);
		skeletonPainter->drawLine(jointData[16], jointData[17]);
		skeletonPainter->drawLine(jointData[17], jointData[18]);
		skeletonPainter->drawLine(jointData[18], jointData[19]);


		skeletonPainter->drawLine(jointData[0], jointData[1]);
		skeletonPainter->drawLine(jointData[1], jointData[20]);
		skeletonPainter->drawLine(jointData[20], jointData[2]);
		skeletonPainter->drawLine(jointData[2], jointData[3]);

		skeletonPainter->drawLine(jointData[20], jointData[4]);
		skeletonPainter->drawLine(jointData[4], jointData[5]);
		skeletonPainter->drawLine(jointData[5], jointData[6]);
		skeletonPainter->drawLine(jointData[6], jointData[7]);
		skeletonPainter->drawLine(jointData[7], jointData[22]);
		skeletonPainter->drawLine(jointData[7], jointData[21]);

		skeletonPainter->drawLine(jointData[20], jointData[8]);
		skeletonPainter->drawLine(jointData[8], jointData[9]);
		skeletonPainter->drawLine(jointData[9], jointData[10]);
		skeletonPainter->drawLine(jointData[10], jointData[11]);
		skeletonPainter->drawLine(jointData[11], jointData[23]);
		skeletonPainter->drawLine(jointData[11], jointData[24]);

		for (auto i = 0; i < JointType_Count; ++i)
		{
			skeletonPainter->drawEllipse(jointData[i],5,5);
			
		}
		skeletonFrameLabel->setPixmap(*skeletonPixmap);

		if (skeletonPainter != NULL){
			delete skeletonPainter;
			skeletonPainter = NULL;
		}

		if (skeletonPixmap != NULL){
			delete skeletonPixmap;
			skeletonPixmap = NULL;
		}
	}
	void Sensor::setKeyPositions(vector<DualQuaternion> pos)
	{
		keyPositions->clearControlPosition();

		for (auto i = 0; i < pos.size(); i++)
			keyPositions->addCtrlPos(pos.at(i));

	}
	void Sensor::setContinuousMotion(vector<DualQuaternion>pos)
	{
		continuousPositions->clearControlPosition();

		for (auto i = 0; i < pos.size(); i++)
			continuousPositions->addCtrlPos(pos.at(i));

	}

	vector<DualQuaternion> Sensor::getKeyPositions()
	{
		return keyPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion>Sensor::getContinuousPositions()
	{
		return continuousPositions->getCtrlPosInDualQuat();
	}
	void Sensor::addNewKeyCtrlPos(DualQuaternion newPositions)
	{
		keyPositions->addCtrlPos(newPositions);
	}
	void Sensor::clearKeyPositionInSensor()
	{
		keyPositions->clearControlPosition();
	}
	void Sensor::clearContinuousPositionsInSensor()
	{
		continuousPositions->clearControlPosition();
		continuousLeftHipPositions->clearControlPosition();
		continuousLeftShoulderPositions->clearControlPosition();
		continuousLeftAnklePositions->clearControlPosition();
		continuousRightAnklePositions->clearControlPosition();
		continuousLeftElbowPositions->clearControlPosition();
		//continuousRightElbowPositions->clearControlPosition();
		LeftFemurPositions->clearLimbPos();
		LeftTibiaPositions->clearLimbPos();
		LeftUpperArmPositions->clearLimbPos();
		LeftLowerArmPositions->clearLimbPos();
		spinePositions->clearLimbPos();
		RightFemurPositions->clearLimbPos();
		RightTibiaPositions->clearLimbPos();
		RightUpperArmPositions->clearLimbPos();
		RightLowerArmPositions->clearLimbPos();
	}
	
	vector<DualQuaternion> Sensor::getFemurPosesInSensor()
	{
		return LeftFemurPositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getTibiaPosesInSensor()
	{
		return LeftTibiaPositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getLeftUpperArmPosesInSensor()
	{
		return LeftUpperArmPositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getLeftLowerArmPosesInSensor()
	{
		return LeftLowerArmPositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getSpinePosesInSensor()
	{
		return spinePositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getRightFemurPosesInSensor()
	{
		return RightFemurPositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getRightTibiaPosesInSensor()
	{
		return RightTibiaPositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getRightUpperArmPosesInSensor()
	{
		return RightUpperArmPositions->getLimbPositions();
	}
	vector<DualQuaternion> Sensor::getRightLowerArmPosesInSensor()
	{
		return RightFemurPositions->getLimbPositions();
	}

	void Sensor::setFemurPosesInSensor(vector<DualQuaternion>pos)
	{
		LeftFemurPositions->clearLimbPos();
		for (auto i = 0; i < pos.size(); i++)
			LeftFemurPositions->addLimbPos(pos.at(i));
	}
	void Sensor::setTibiaPosesInSensor(vector<DualQuaternion> pos)
	{
		LeftTibiaPositions->clearLimbPos();
		for (auto i = 0; i < pos.size(); i++)
			LeftTibiaPositions->addLimbPos(pos.at(i));
	}
	void Sensor::setSpinePosesInSensor(vector<DualQuaternion> pos)
	{
		spinePositions->clearLimbPos();
		for (auto i = 0; i < pos.size(); i++)
			spinePositions->addLimbPos(pos.at(i));
	}

	vector<DualQuaternion> Sensor::getKeyFemurPositions()
	{
		return keyLeftFemurPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeyTibiaPositions()
	{
		return keyLeftTibiaPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeyLeftUpperArmPositions()
	{
		return keyLeftUpperArmPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeyLeftLowerArmPositions()
	{
		return keyLeftLowerArmPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeySpinePositions()
	{
		return keySpinePositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeyRightFemurPositions()
	{
		return keyRightFemurPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeyRightTibiaPositions()
	{
		return keyRightTibiaPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeyRightUpperArmPositions()
	{
		return keyRightUpperArmPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion>Sensor::getKeyRightLowerArmPositions()
	{
		return keyRightLowerArmPositions->getCtrlPosInDualQuat();
	}


	vector<float> Sensor::getFemurLengthInSensor()
	{
		return LeftFemurPositions->getLimbLength();
	}
	vector<float> Sensor::getTibiaLengthInSensor()
	{
		return LeftTibiaPositions->getLimbLength();
	}
	vector<float> Sensor::getLeftUpperArmLengthInSensor()
	{
		return LeftUpperArmPositions->getLimbLength();
	}
	vector<float> Sensor::getLeftLowerArmLengthInSensor()
	{
		return LeftLowerArmPositions->getLimbLength();
	}
	vector<float> Sensor::getSpineLengthInSensor()
	{
		return spinePositions->getLimbLength();
	}
	vector<float>Sensor::getRightFemurLengthInSensor()
	{
		return RightFemurPositions->getLimbLength();
	}
	vector<float> Sensor::getRightTibiaLengthInSensor()
	{
		return RightTibiaPositions->getLimbLength();
	}
	vector<float>Sensor::getRightUpperArmLengthInSensor()
	{
		return RightUpperArmPositions->getLimbLength();
	}
	vector<float>Sensor::getRightLowerArmLengthInSensor()
	{
		return RightLowerArmPositions->getLimbLength();
	}


	vector<QVector3D> Sensor::getFemurStart()
	{
		return LeftFemurPositions->getstartPoint();
	}
	vector<QVector3D> Sensor::getFemurEnd()
	{
		return LeftFemurPositions->getendPoint();
	}
	vector<QVector3D> Sensor::getTibiaStart()
	{
		return LeftTibiaPositions->getstartPoint();
	}
	vector<QVector3D> Sensor::getTibiaEnd()
	{
		return LeftTibiaPositions->getendPoint();
	}
	vector<QVector3D> Sensor::getLeftUpperArmStart()
	{
		return LeftUpperArmPositions->getstartPoint();
	}
	vector<QVector3D> Sensor::getLeftUpperArmEnd()
	{
		return LeftUpperArmPositions->getendPoint();
	}
	vector<QVector3D> Sensor::getLeftLowerArmStart()
	{
		return LeftLowerArmPositions->getstartPoint();
	}
	vector<QVector3D> Sensor::getLeftLowerArmEnd()
	{
		return LeftLowerArmPositions->getendPoint();
	}
	vector<QVector3D> Sensor::getSpineStart()
	{
		return spinePositions->getstartPoint();
	}
	vector<QVector3D> Sensor::getSpineEnd()
	{
		return spinePositions->getendPoint();
	}
	vector<QVector3D>Sensor::getRightFemurStart()
	{
		return RightFemurPositions->getstartPoint();
	}
	vector<QVector3D>Sensor::getRightFemurEnd()
	{
		return RightFemurPositions->getendPoint();
	}
	vector<QVector3D>Sensor::getRightTibiaStart()
	{
		return RightTibiaPositions->getstartPoint();
	}
	vector<QVector3D>Sensor::getRightTibiaEnd()
	{
		return RightTibiaPositions->getendPoint();
	}
	vector<QVector3D> Sensor::getRightUpperArmStart()
	{
		return RightUpperArmPositions->getstartPoint();
	}
	vector<QVector3D>Sensor::getRightUpperArmEnd()
	{
		return RightUpperArmPositions->getendPoint();
	}
	vector<QVector3D>Sensor::getRightLowerArmStart()
	{
		return RightLowerArmPositions->getstartPoint();
	}
	vector<QVector3D>Sensor::getRightLowerArmEnd()
	{
		return RightLowerArmPositions->getendPoint();
	}

	void Sensor::addNewKeyFemurPos(DualQuaternion newFemurPositions)
	{
		keyLeftFemurPositions->addCtrlPos(newFemurPositions);
	}
	void Sensor::addNewKeyTibiaPos(DualQuaternion newTibiaPositions)
	{
		keyLeftTibiaPositions->addCtrlPos(newTibiaPositions);
	}
	void Sensor::addNewKeyLeftUpperArmPos(DualQuaternion newLeftUpperArmPositions)
	{
		keyLeftUpperArmPositions->addCtrlPos(newLeftUpperArmPositions);
	}
	void Sensor::addNewKeyLeftLowerArmPos(DualQuaternion newLeftLowerArmPositions)
	{
		keyLeftLowerArmPositions->addCtrlPos(newLeftLowerArmPositions);
	}
	void Sensor::addNewKeySpinePos(DualQuaternion newSpinePosition)
	{
		keySpinePositions->addCtrlPos(newSpinePosition);
	}
	void Sensor::addNewKeyRightFemurPos(DualQuaternion newRightFemurPos)
	{
		keyRightFemurPositions->addCtrlPos(newRightFemurPos);
	}
	void Sensor::addNewKeyRightTibiaPos(DualQuaternion newRightTibiaPos)
	{
		keyRightTibiaPositions->addCtrlPos(newRightTibiaPos);
	}
	void Sensor::addNewKeyRightUpperArmPos(DualQuaternion newRightUpperArmPositions)
	{
		keyRightUpperArmPositions->addCtrlPos(newRightUpperArmPositions);
	}
	void Sensor::addNewKeyRightLowerArmPos(DualQuaternion newRightLowerArmPositions)
	{
		keyRightLowerArmPositions->addCtrlPos(newRightLowerArmPositions);
	}

	void Sensor::clearKeyFemurPositionInSensor()
	{
		keyLeftFemurPositions->clearControlPosition();
	}
	void Sensor::clearKeyTibiaPositionInSensor()
	{
		keyLeftTibiaPositions->clearControlPosition();
	}
	void Sensor::clearKeyLeftUpperArmPositionInSensor()
	{
		keyLeftUpperArmPositions->clearControlPosition();
	}
	void Sensor::clearKeyLeftLowerArmPositionInSensor()
	{
		keyLeftLowerArmPositions->clearControlPosition();
	}
	void Sensor::clearKeySpinePositionInSensor()
	{
		keySpinePositions->clearControlPosition();
	}
	void Sensor::clearKeyRightFemurPositionInSensor()
	{
		keyRightFemurPositions->clearControlPosition();
	}
	void Sensor::clearKeyRightTibiaPositionInSensor()
	{
		keyRightTibiaPositions->clearControlPosition();
	}
	void Sensor::clearKeyRightUpperArmPositionInSensor()
	{
		keyRightUpperArmPositions->clearControlPosition();
	}
	void Sensor::clearKeyRightLowerArmPositionInSensor()
	{
		keyRightLowerArmPositions->clearControlPosition();
	}

	void Sensor::setKeyFemurPositions(vector<DualQuaternion> pos)
	{
		keyLeftFemurPositions->clearControlPosition();

		for (auto i = 0; i < pos.size(); i++)
			keyLeftFemurPositions->addCtrlPos(pos.at(i));
	}
	void Sensor::setKeyTibiaPositions(vector<DualQuaternion> pos)
	{
		keyLeftTibiaPositions->clearControlPosition();
		for (auto i = 0; i < pos.size(); i++)
			keyLeftTibiaPositions->addCtrlPos(pos.at(i));
	}
	void Sensor::setKeySpinePositions(vector<DualQuaternion> pos)
	{
		keySpinePositions->clearControlPosition();
		for (auto i = 0; i < pos.size(); i++)
			keySpinePositions->addCtrlPos(pos.at(i));
	}

	vector<DualQuaternion> Sensor::getContinuousLeftHipPositions()
	{
		return continuousLeftHipPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getKeyLeftHipPositions()
	{
		return keyLeftHipPositions->getCtrlPosInDualQuat();
	}
	void Sensor::addNewLeftHipKeyCtrlPos(DualQuaternion newHipPosition)
	{
		keyLeftHipPositions->addCtrlPos(newHipPosition);
	}
	void Sensor::clearKeyLeftHipPositionsInSensor()
	{
		keyLeftHipPositions->clearControlPosition();
	}
	
	vector<DualQuaternion> Sensor::getKeyLeftShoulderPositions()
	{
		return keyLeftShoulderPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getContinuousLeftShoulderPositions()
	{
		return continuousLeftShoulderPositions->getCtrlPosInDualQuat();
	}
	void Sensor::addNewLeftShoulderKeyCtrlPos (DualQuaternion newShoulderPosition)
	{
		keyLeftShoulderPositions->addCtrlPos(newShoulderPosition);
	}
	void Sensor::clearKeyLeftShoulderPositionsInSensor()
	{
		keyLeftShoulderPositions->clearControlPosition();
	}
	
	vector<DualQuaternion> Sensor::getKeyLeftAnklePositions(){
		return keyLeftAnklePositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getContinuousLeftAnklePositions(){
		return continuousLeftAnklePositions->getCtrlPosInDualQuat();
	}
	void Sensor::addNewLeftAnkleKeyCtrlPos(DualQuaternion newLeftAnklePosition){
		keyLeftAnklePositions->addCtrlPos(newLeftAnklePosition);
	}
	void Sensor::clearKeyLeftAnklePositionsInSensor(){
		keyLeftAnklePositions->clearControlPosition();
	}

	vector<DualQuaternion> Sensor::getKeyRightAnklePositions()
	{
		return keyRightAnklePositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getContinuousRightAnklePositions()
	{
		return continuousRightAnklePositions->getCtrlPosInDualQuat();
	}
	void Sensor::addNewRightAnkleKeyCtrlPos(DualQuaternion newRightAnklePosition)
	{
		keyRightAnklePositions->addCtrlPos(newRightAnklePosition);
	}
	void Sensor::clearKeyRightAnklePositionsInSensor()
	{
		keyRightAnklePositions->clearControlPosition();
	}

	vector<DualQuaternion> Sensor::getKeyLeftElbowPositions()
	{
		return keyLeftElbowPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion> Sensor::getContinuousLeftElbowPositions()
	{
		return continuousLeftElbowPositions->getCtrlPosInDualQuat();
	}
	void Sensor::addNewLeftElbowKeyCtrlPos(DualQuaternion newLeftElbowPosition)
	{
		keyLeftElbowPositions->addCtrlPos(newLeftElbowPosition);
	}
	void Sensor::clearKeyLeftElbowPositionsInSensor()
	{
		keyLeftElbowPositions->clearControlPosition();
	}

	/*vector<DualQuaternion>Sensor::getKeyRightElbowPositions()
	{
		return keyRightElbowPositions->getCtrlPosInDualQuat();
	}
	vector<DualQuaternion>Sensor::getContinuousRightElbowPositions()
	{
		return continuousRightElbowPositions->getCtrlPosInDualQuat();
	}
	void Sensor::addNewRightElbowKeyCtrlPos(DualQuaternion newRightElbowPosition)
	{
		keyRightElbowPositions->addCtrlPos(newRightElbowPosition);
	}
	void Sensor::clearKeyRightElbowPositionsInSensor()
	{
		keyRightElbowPositions->clearControlPosition();
	}*/

	Sensor::~Sensor()
	{
		if (m_pColorRGBX)
		{
			delete[] m_pColorRGBX;
			m_pColorRGBX = NULL;
		}

		SafeRelease(m_pColorFrameReader);

		if (m_pKinectSensor)
		{
			m_pKinectSensor->Close();
		}
		SafeRelease(m_pKinectSensor);

		if (keyPositions != NULL){
			delete keyPositions;
			keyPositions = NULL;
		}
		if (continuousPositions != NULL){
			delete continuousPositions;
			continuousPositions = NULL;
		}

		if (keyLeftTibiaPositions != NULL){
			delete keyLeftTibiaPositions;
			keyLeftTibiaPositions = NULL;
		}

		if (keyLeftFemurPositions != NULL){
			delete keyLeftFemurPositions;
			keyLeftFemurPositions = NULL;
		}
		if (LeftFemurPositions != NULL){
			delete LeftFemurPositions;
			LeftFemurPositions = NULL;
		}
		if (LeftTibiaPositions != NULL){
			delete LeftTibiaPositions;
			LeftTibiaPositions = NULL;
		}

		if (LeftUpperArmPositions != NULL){
			delete LeftUpperArmPositions;
			LeftUpperArmPositions = NULL;
		}
		if (keyLeftUpperArmPositions != NULL){
			delete keyLeftUpperArmPositions;
			keyLeftUpperArmPositions = NULL;
		}
		if (LeftLowerArmPositions != NULL){
			delete LeftLowerArmPositions;
			LeftLowerArmPositions = NULL;
		}
		if (keyLeftLowerArmPositions != NULL){
			delete keyLeftLowerArmPositions;
			keyLeftLowerArmPositions = NULL;
		}

		if (spinePositions != NULL){
			delete spinePositions;
			spinePositions = NULL;
		}
		if (keySpinePositions != NULL){
			delete keySpinePositions;
			keySpinePositions = NULL;
		}

		if (RightFemurPositions != NULL){
			delete RightFemurPositions;
			RightFemurPositions = NULL;
		}
		if (RightTibiaPositions != NULL){
			delete RightTibiaPositions;
			RightTibiaPositions = NULL;
		}
		if (keyRightFemurPositions != NULL){
			delete keyRightFemurPositions;
			keyRightFemurPositions = NULL;
		}
		if (keyRightTibiaPositions != NULL){
			delete keyRightTibiaPositions;
			keyRightTibiaPositions = NULL;
		}

		if (RightUpperArmPositions != NULL){
			delete RightUpperArmPositions;
			RightUpperArmPositions = NULL;
		}
		if (keyRightUpperArmPositions != NULL){
			delete keyRightUpperArmPositions;
			keyRightUpperArmPositions = NULL;
		}
		if (RightLowerArmPositions != NULL){
			delete RightLowerArmPositions;
			RightLowerArmPositions = NULL;
		}
		if (keyRightLowerArmPositions != NULL){
			delete keyRightLowerArmPositions;
			keyRightLowerArmPositions = NULL;
		}




		if (continuousLeftHipPositions != NULL)
		{
			delete continuousLeftHipPositions;
			continuousLeftHipPositions = NULL;
		}
		if (keyLeftHipPositions != NULL){
			delete keyLeftHipPositions;
			keyLeftHipPositions = NULL;
		}

		if (continuousLeftShoulderPositions != NULL){
			delete continuousLeftShoulderPositions;
			continuousLeftShoulderPositions = NULL;
		}
		if (keyLeftShoulderPositions != NULL){
			delete keyLeftShoulderPositions;
			keyLeftShoulderPositions = NULL;
		}

		if (continuousLeftAnklePositions != NULL){
			delete continuousLeftAnklePositions;
			continuousLeftAnklePositions = NULL;
		}
		if (keyLeftAnklePositions != NULL){
			delete keyLeftAnklePositions;
			keyLeftAnklePositions = NULL;
		}

		if (continuousRightAnklePositions != NULL){
			delete continuousRightAnklePositions;
			continuousRightAnklePositions = NULL;
		}
		if (keyRightAnklePositions != NULL){
			delete keyRightAnklePositions;
			keyRightAnklePositions = NULL;
		}

		if (keyLeftElbowPositions != NULL){
			delete keyLeftElbowPositions;
			keyLeftElbowPositions = NULL;
		}
		if (continuousLeftElbowPositions != NULL){
			delete continuousLeftElbowPositions;
			continuousLeftElbowPositions = NULL;
		}

		/*if (keyRightElbowPositions != NULL){
			delete keyRightElbowPositions;
			keyRightElbowPositions = NULL;
		}*/
		
		/*if (continuousRightElbowPositions != NULL){
			delete continuousRightElbowPositions;
			continuousRightElbowPositions = NULL;
		}*/
	}
	// *********************************** All Slot Definitions ******************************************//
	void Sensor::updateColorFrame()
	{
		if (!m_pColorFrameReader)
		{
			return;
		}
		IColorFrame* pColorFrame = NULL;
		HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

		if (SUCCEEDED(hr))
		{
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nBufferSize = 0;
			RGBQUAD* pBuffer = NULL;

			hr = pColorFrame->get_FrameDescription(&pFrameDescription);

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Height(&nHeight);
			}
			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
				}
				else if (m_pColorRGBX)
				{
					pBuffer = m_pColorRGBX;
					nBufferSize = cColorWidth*cColorHeight*sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
				}
				else
				{
					hr = E_FAIL;
				}
			}
			if (SUCCEEDED(hr))
			{
				m_pcolorFrame->drawColorFrame((const unsigned char*)pBuffer);
			}
			SafeRelease(pFrameDescription);
		}
		SafeRelease(pColorFrame);
	}
	void Sensor::updateSkeletonFrame()
	{
		if (!m_pBodyFrameReader)
		{
			return;
		}

		IBodyFrame* pBodyFrame = NULL;

		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

		if (SUCCEEDED(hr))
		{
			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}
			if (SUCCEEDED(hr))
			{
				processSkeletonData(BODY_COUNT, ppBodies);
			}
			for (auto i = 0; i < _countof(ppBodies); i++)
				SafeRelease(ppBodies[i]);
		}
		SafeRelease(pBodyFrame);
	}
	void Sensor::StoreKeyPosition(bool idx)
	{
		if (mainWinPtr->positionRecordingSlider->value() < continuousPositions->getNoOfCtrlPos())
		{
			keyPositions->addCtrlPos(continuousPositions->getPosition(mainWinPtr->positionRecordingSlider->value()));
		}
	}
}