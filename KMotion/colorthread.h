#ifndef COLORTHREAD_H
#define COLORTHREAD_H

#include <QtCore/QThread>

namespace Kinect
{
	class colorThread :public QThread
	{
		Q_OBJECT

	public:
		colorThread(QObject* parent = NULL);
		void stop();
		virtual ~colorThread();

	protected:
		void run();

	signals:
		void updatedColorFrame();

	protected:
		bool m_running;
	};
}
#endif