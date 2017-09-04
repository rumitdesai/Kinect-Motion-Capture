#include "JointSelectionDialog.h"


JointSelection::JointSelection(QWidget* parent) :QDialog(parent)
{
	if (this->objectName().isEmpty())
		this->setObjectName("Joint_Selection");
	this->resize(300, 400);

	QSizePolicy sizePolicyForJointGroupBox(QSizePolicy::Expanding, QSizePolicy::Expanding);
	sizePolicyForJointGroupBox.setHorizontalStretch(0);
	sizePolicyForJointGroupBox.setVerticalStretch(0);

	JointGroupBox = new QGroupBox(tr("Type of Joints"), this);
	JointGroupBox->setSizePolicy(sizePolicyForJointGroupBox);

	LeftAnkle = new QCheckBox("Left Ankle",this);
	RightAnkle = new QCheckBox("Right Ankle",this);
	LeftElbow = new QCheckBox("Left Elbow",this);
	RightElbow = new QCheckBox("Right Elbow",this);
	/*LeftFoot = new QCheckBox("Left Foot",this);
	RightFoot = new QCheckBox("Right Foot",this);
	LeftHand=new QCheckBox("Left Hand",this);
	RightHand = new QCheckBox("Right Hand",this);
	LeftHandTip = new QCheckBox("Left Hand Tip",this);
	RightHandTip = new QCheckBox("Right Hand Tip",this);
	Head = new QCheckBox("Head",this);*/
	LeftHip = new QCheckBox("Left Hip",this);
	/*RightHip = new QCheckBox("Right Hip",this);
	LeftKnee = new QCheckBox("Left Knee",this);
	RightKnee = new QCheckBox("Right Knee",this);
	Neck = new QCheckBox("Neck",this);*/
	LeftShoulder = new QCheckBox("Left Shoulder",this);
	/*RightShoulder = new QCheckBox("Right Shoulder",this);
	SpineBase = new QCheckBox("Spine Base",this);
	SpineMid = new QCheckBox("Spine Mid",this);
	SpineShoulder = new QCheckBox("Spine Shoulder",this);
	LeftThumb = new QCheckBox("Left Thumb",this);
	RightThumb = new QCheckBox("Right Thumb",this);
	LeftWrist = new QCheckBox("Left Wrist",this);
	RightWrist = new QCheckBox("Right Wrist",this);*/

	checkBoxesLayout = new QGridLayout(this);
	checkBoxesLayout->addWidget(LeftAnkle, 0, 0, 1, 1);
	checkBoxesLayout->addWidget(RightAnkle, 1, 0, 1, 1);
	checkBoxesLayout->addWidget(LeftElbow, 2, 0, 1, 1);
	checkBoxesLayout->addWidget(RightElbow, 3, 0, 1, 1);
	/*checkBoxesLayout->addWidget(LeftFoot, 4, 0, 1, 1);
	checkBoxesLayout->addWidget(RightFoot, 5, 0, 1, 1);
	checkBoxesLayout->addWidget(LeftHand, 6, 0, 1, 1);
	checkBoxesLayout->addWidget(RightHand, 7, 0, 1, 1);*/

	/*checkBoxesLayout->addWidget(LeftHandTip, 0, 1, 1, 1);
	checkBoxesLayout->addWidget(RightHandTip, 1, 1, 1, 1);
	checkBoxesLayout->addWidget(Head, 2, 1, 1, 1);*/
	checkBoxesLayout->addWidget(LeftHip, 3, 1, 1, 1);
	/*checkBoxesLayout->addWidget(RightHip, 4, 1, 1, 1);
	checkBoxesLayout->addWidget(LeftKnee, 5, 1, 1, 1);
	checkBoxesLayout->addWidget(RightKnee, 6, 1, 1, 1);
	checkBoxesLayout->addWidget(Neck, 7, 1, 1, 1);*/

	checkBoxesLayout->addWidget(LeftShoulder, 0, 2, 1, 1);
	/*checkBoxesLayout->addWidget(RightShoulder, 1, 2, 1, 1);
	checkBoxesLayout->addWidget(SpineBase, 2, 2, 1, 1);
	checkBoxesLayout->addWidget(SpineMid, 3, 2, 1, 1);
	checkBoxesLayout->addWidget(SpineShoulder, 4, 2, 1, 1);
	checkBoxesLayout->addWidget(LeftThumb, 5, 2, 1, 1);
	checkBoxesLayout->addWidget(RightThumb, 6, 2, 1, 1);
	checkBoxesLayout->addWidget(LeftWrist, 7, 2, 1, 1);

	checkBoxesLayout->addWidget(RightWrist, 0, 3, 1, 1);*/

	JointGroupBox->setLayout(checkBoxesLayout);

	
	//OKButton = new QPushButton("Ok",this);
	//OKButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
	
	
	centralLayout = new QVBoxLayout(this);
	//centralLayout->addWidget(NameEdit);
	centralLayout->addWidget(JointGroupBox);
	//centralLayout->addWidget(OKButton);
	
	QDialog::setWindowTitle("Select Joints");

	setLayout(centralLayout);
}


JointSelection::~JointSelection()
{

}

bool JointSelection::isLeftAnkleChecked()
{
	return LeftAnkle->isChecked();
}
bool JointSelection::isRightAnkleChecked()
{
	return RightAnkle->isChecked();
}
bool JointSelection::isLeftElbowChecked()
{
	return LeftElbow->isChecked();
}
bool JointSelection::isRightElbowchecked()
{
	return RightElbow->isChecked();
}
bool JointSelection::isLeftFootchecked()
{
	return LeftFoot->isChecked();
}
bool JointSelection::isRightFootchecked()
{
	return RightFoot->isChecked();
}
bool JointSelection::isLeftHandchecked()
{
	return LeftHand->isChecked();
}
bool JointSelection::isRightHandchecked()
{
	return RightHand->isChecked();
}
bool JointSelection::isLeftHandTipchecked()
{
	return LeftHandTip->isChecked();
}
bool JointSelection::isRightHandTipchecked()
{
	return RightHandTip->isChecked();
}
bool JointSelection::isHeadchecked()
{
	return Head->isChecked();
}
bool JointSelection::isLeftHipchecked()
{
	return LeftHip->isChecked();
}
bool JointSelection::isRightHipchecked()
{
	return RightHip->isChecked();
}
bool JointSelection::isLeftKneechecked()
{
	return LeftKnee->isChecked();
}
bool JointSelection::isRightKneechecked()
{
	return RightKnee->isChecked();
}
bool JointSelection::isNeckchecked()
{
	return Neck->isChecked();
}
bool JointSelection::isLeftShoulderchecked()
{
	return LeftShoulder->isChecked();
}
bool JointSelection::isRightShoulderchecked()
{
	return RightShoulder->isChecked();
}
bool JointSelection::isSpineBasechecked()
{
	return SpineBase->isChecked();
}
bool JointSelection::isSpineMidchecked()
{
	return SpineMid->isChecked();
}
bool JointSelection::isSpineShoulderchecked()
{
	return SpineShoulder->isChecked();
}
bool JointSelection::isLeftThumbchecked()
{
	return LeftThumb->isChecked();
}
bool JointSelection::isRightThumbchecked()
{
	return RightThumb->isChecked();
}
bool JointSelection::isLeftWristchecked()
{
	return LeftWrist->isChecked();
}
bool JointSelection::isRightWristchecked()
{
	return RightWrist->isChecked();
}

/*bool JointSelection::ifAnyJointSelected()
{
	if (isLeftAnkleChecked())
		return true;
	else if (isRightAnkleChecked())
		return true;
	else if (isLeftElbowChecked())
		return true;
	else if (isRightElbowchecked())
		return true;
	else if (isLeftFootchecked())
		return true;
	else if (isRightFootchecked())
		return true;
	else if (isLeftHandchecked())
		return true;
	else if (isRightHandchecked())
		return true;
	else if (isLeftHandTipchecked())
		return true;
	else if (isRightHandTipchecked())
		return true;
	else if (isHeadchecked())
		return true;
	else if (isLeftHipchecked())
		return true;
	else if (isRightHipchecked())
		return true;
	else if (isLeftKneechecked())
		return true;
	else if (isRightKneechecked())
		return true;
	else if (isNeckchecked())
		return true;
	else if (isLeftShoulderchecked())
		return true;
	else if (isRightShoulderchecked())
		return true;
	else if (isSpineBasechecked())
		return true;
	else if (isSpineMidchecked())
		return true;
	else if (isSpineShoulderchecked())
		return true;
	else if (isLeftThumbchecked())
		return true;
	else if (isRightThumbchecked())
		return true;
	else if (isLeftWristchecked())
		return true;
	else if (isRightWristchecked())
		return true;
	else
		return false;
}*/


//#include "moc_JoinSelectionDialog.cpp"