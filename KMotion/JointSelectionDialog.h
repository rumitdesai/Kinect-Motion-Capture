//#ifndef JOINTSELECTION
//#define JOINTSELECTION

#include <QtWidgets/QDialog>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QGridLayout>
//#include <QLineEdit>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QSizePolicy>


class JointSelection :public QDialog
{
	Q_OBJECT
public:
	JointSelection(QWidget*parent=0);
	~JointSelection();
	bool isLeftAnkleChecked();
	bool isRightAnkleChecked();
	bool isLeftElbowChecked();
	bool isRightElbowchecked();
	bool isLeftFootchecked();
	bool isRightFootchecked();
	bool isLeftHandchecked();
	bool isRightHandchecked();
	bool isLeftHandTipchecked();
	bool isRightHandTipchecked();
	bool isHeadchecked();
	bool isLeftHipchecked();
	bool isRightHipchecked();
	bool isLeftKneechecked();
	bool isRightKneechecked();
	bool isNeckchecked();
	bool isLeftShoulderchecked();
	bool isRightShoulderchecked();
	bool isSpineBasechecked();
	bool isSpineMidchecked();
	bool isSpineShoulderchecked();
	bool isLeftThumbchecked();
	bool isRightThumbchecked();
	bool isLeftWristchecked();
	bool isRightWristchecked();

	//bool ifAnyJointSelected();

private:
	//QPushButton* OKButton;
	//QLineEdit* NameEdit;
	
	QVBoxLayout* centralLayout;
	QGridLayout* checkBoxesLayout;
	QCheckBox* LeftAnkle;
	QCheckBox* RightAnkle;
	QCheckBox* LeftElbow;
	QCheckBox* RightElbow;
	QCheckBox* LeftFoot;
	QCheckBox* RightFoot;
	QCheckBox* LeftHand;
	QCheckBox* RightHand;
	QCheckBox* LeftHandTip;
	QCheckBox* RightHandTip;
	QCheckBox* Head;
	QCheckBox* LeftHip;
	QCheckBox* RightHip;
	QCheckBox* LeftKnee;
	QCheckBox* RightKnee;
	QCheckBox*  Neck;
	QCheckBox* LeftShoulder;
	QCheckBox* RightShoulder;
	QCheckBox* SpineBase;
	QCheckBox* SpineMid;
	QCheckBox* SpineShoulder;
	QCheckBox* LeftThumb;
	QCheckBox* RightThumb;
	QCheckBox* LeftWrist;
	QCheckBox* RightWrist;
	QGroupBox* JointGroupBox;

};

//#endif