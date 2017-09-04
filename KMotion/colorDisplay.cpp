#include <QtGui/QImage>
#include "colorDisplay.h"

namespace Kinect
{
	colorDisplay::colorDisplay(QLabel* label, unsigned int width, unsigned int height, QObject* parent)
		:m_label(label), m_width(width), m_height(height), QObject(parent)
	{
		m_buffer = new unsigned char[width*height * 4];
	}

	colorDisplay::~colorDisplay()
	{
		delete[](m_buffer);
	}

	void colorDisplay::drawColorFrame(const unsigned char* data)
	{
		QImage image((const unsigned char*)data, m_width, m_height, QImage::Format_RGB32);
		m_label->setPixmap(QPixmap::fromImage(image.scaled(m_label->frameSize())));
	}
}