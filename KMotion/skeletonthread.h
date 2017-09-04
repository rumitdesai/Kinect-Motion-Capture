#ifndef SKELETONTHREAD_H
#define SKELETONTHREAD_H

#include <QtCore/QThread>

namespace Kinect
{
	class skeletonThread :public QThread
	{
		Q_OBJECT
	public:
		skeletonThread(QObject* parent = NULL);
		void stop();
		virtual ~skeletonThread();
	protected:
		void run();

	signals:
		void updatedSkeletonFrame();
	
	protected:
		bool m_running;
	};
}

#endif