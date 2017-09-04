#ifndef COLORDISPLAY_H
#define COLORDISPLAY_H

#include <QtCore/QObject>
#include <QtWidgets/QLabel>


namespace Kinect
{
	class colorDisplay :public QObject
	{
	public:
		colorDisplay(QLabel* label, unsigned int width, unsigned int height, QObject* parent = NULL);
		void drawColorFrame(const unsigned char* data);
		virtual ~colorDisplay();
	protected:
		QLabel* m_label;
		unsigned int m_width;
		unsigned int m_height;
		unsigned char* m_buffer;
	};
}


#endif