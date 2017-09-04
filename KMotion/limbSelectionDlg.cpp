#include "limbSelectionDlg.h"


LimbSelection::LimbSelection(QWidget* parent) :QDialog(parent)
{
	if (this->objectName().isEmpty())
		this->setObjectName("Limb_Selection");
	this->resize(300, 300);

	QSizePolicy sizePolicyForLimbGroupBox(QSizePolicy::Expanding, QSizePolicy::Expanding);
	sizePolicyForLimbGroupBox.setHorizontalStretch(0);
	sizePolicyForLimbGroupBox.setVerticalStretch(0);

	LimbGroupBox = new QGroupBox(tr("Type of Limbs"), this);
	LimbGroupBox->setSizePolicy(sizePolicyForLimbGroupBox);

	LeftFemurLimb = new QCheckBox(tr("Left Femur Limb"), this);
	LeftTibiaLimb = new QCheckBox(tr("Left Tibia Limb"), this);
	SpineLimb = new QCheckBox(tr("Spine Limb"), this);
	RightFemurLimb = new QCheckBox(tr("Right Femur Limb"), this);
	RightTibiaLimb = new QCheckBox(tr("Right Tibia Limb"), this);
	RightUpperArmLimb = new QCheckBox(tr("Right Upper Arm"), this);
	RightLowerArmLimb = new QCheckBox(tr("Right Lower Arm"), this);
	LeftUpperArmLimb = new QCheckBox(tr("Left Upper Arm"), this);
	LeftLowerArmLimb = new QCheckBox(tr("Left Lower Arm"), this);

	checkBoxesLayout = new QGridLayout(this);
	checkBoxesLayout->addWidget(LeftFemurLimb, 0, 0, 1, 1);
	checkBoxesLayout->addWidget(LeftTibiaLimb, 1, 0, 1, 1);
	checkBoxesLayout->addWidget(RightFemurLimb, 0, 1, 1, 1);
	checkBoxesLayout->addWidget(RightTibiaLimb, 1, 1, 1, 1);
	checkBoxesLayout->addWidget(RightUpperArmLimb, 2, 1, 1, 1);
	checkBoxesLayout->addWidget(RightLowerArmLimb, 3, 1, 1, 1);
	checkBoxesLayout->addWidget(LeftUpperArmLimb, 2, 0, 1, 1);
	checkBoxesLayout->addWidget(LeftLowerArmLimb, 3, 0, 1, 1);
	checkBoxesLayout->addWidget(SpineLimb, 4, 0, 1, 1);
	
	/*OkButton = new QPushButton("Ok",this);
	OkButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);*/

	LimbGroupBox->setLayout(checkBoxesLayout);

	centralLayout = new QVBoxLayout(this);
	centralLayout->addWidget(LimbGroupBox);
	QDialog::setWindowTitle("Select Limbs");
	setLayout(centralLayout);
}

LimbSelection::~LimbSelection()
{

}


bool LimbSelection::isLeftFemurChecked()
{
	return LeftFemurLimb->isChecked();
}
bool LimbSelection::isLeftTibiaChecked()
{
	return LeftTibiaLimb->isChecked();
}
bool LimbSelection::isLeftUpperArmChecked()
{
	return LeftUpperArmLimb->isChecked();
}
bool LimbSelection::isLeftLowerArmChecked()
{
	return LeftLowerArmLimb->isChecked();
}
bool LimbSelection::isRightUpperArmChecked()
{
	return RightUpperArmLimb->isChecked();
}
bool LimbSelection::isRightLowerArmChecked()
{
	return RightLowerArmLimb->isChecked();
}
bool LimbSelection::isSpineChecked()
{
	return SpineLimb->isChecked();
}
bool LimbSelection::isRightFemurChecked()
{
	return RightFemurLimb->isChecked();
}
bool LimbSelection::isRightTibiaChecked()
{
	return RightTibiaLimb->isChecked();
}


void LimbSelection::checkLeftFemur()
{
	if (!LeftFemurLimb->isChecked())
		LeftFemurLimb->setChecked(true);
}

void LimbSelection::checkLeftTibia()
{
	if (!LeftTibiaLimb->isChecked())
		LeftTibiaLimb->setChecked(true);
}

void LimbSelection::checkSpine()
{
	if (!SpineLimb->isChecked())
		SpineLimb->setChecked(true);
}

//#include "moc_limbSelectionDlg.cpp"