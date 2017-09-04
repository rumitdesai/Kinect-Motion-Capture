#include "skeletonthread.h"

namespace Kinect
{
	skeletonThread::skeletonThread(QObject* parent) :
		QThread(parent), m_running(false)
	{

	}

	skeletonThread::~skeletonThread(){}

	void skeletonThread::run()
	{
		m_running = true;

		while (m_running)
		{
			//emit updatedSkeletonFrame();
			usleep(150);
		}
	}

	void skeletonThread::stop()
	{
		m_running = false;
	}
}