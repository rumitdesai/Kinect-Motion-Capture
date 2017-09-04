//#ifndef LIMBSELECTION
//#define LIMBSELECTION

#include <QtWidgets/QDialog>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QSizePolicy>



class LimbSelection :public QDialog
{
	Q_OBJECT
public:
	LimbSelection(QWidget* parent=0);
	~LimbSelection();

	bool isLeftFemurChecked();
	bool isLeftTibiaChecked();
	bool isLeftUpperArmChecked();
	bool isLeftLowerArmChecked();
	bool isSpineChecked();
	bool isRightFemurChecked();
	bool isRightTibiaChecked();
	bool isRightUpperArmChecked();
	bool isRightLowerArmChecked();

	void checkLeftFemur();
	void checkLeftTibia();
	void checkSpine();
private:
	QPushButton* OkButton;
	QVBoxLayout* centralLayout;
	QGridLayout* checkBoxesLayout;
	QGroupBox* LimbGroupBox;

	QCheckBox* LeftFemurLimb;
	QCheckBox* LeftTibiaLimb;
	QCheckBox* SpineLimb;
	QCheckBox* RightFemurLimb;
	QCheckBox* RightTibiaLimb;
	QCheckBox* LeftUpperArmLimb;
	QCheckBox* LeftLowerArmLimb;
	QCheckBox* RightUpperArmLimb;
	QCheckBox* RightLowerArmLimb;

};

//#endif