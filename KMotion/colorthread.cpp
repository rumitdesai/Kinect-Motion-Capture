#include "colorthread.h"

namespace Kinect
{
	colorThread::colorThread(QObject* parent):
		QThread(parent), m_running(false)
	{

	}

	colorThread::~colorThread(){}

	void colorThread::run()
	{
		m_running = true;

		while (m_running)
		{
			//emit updatedColorFrame();
			usleep(150);
		}
	}

	void colorThread::stop()
	{
		m_running = false;
	}
}