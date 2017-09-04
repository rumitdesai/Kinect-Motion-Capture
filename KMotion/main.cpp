#include <QtWidgets/QApplication>


#include "maindialog.h"

Kinect::mainDialog* mainWinPtr;

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Kinect::mainDialog mainWin;
	mainWinPtr = &mainWin;
	mainWin.showMaximized();
	return a.exec();
}
