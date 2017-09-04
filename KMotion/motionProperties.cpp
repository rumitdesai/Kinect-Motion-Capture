#include "motionProperties.h"


MotionProperties::MotionProperties()
{
	TRAJECTORY_COLOR[0] = static_cast<float>(0.7);
	TRAJECTORY_COLOR[1] = static_cast<float>(0.5);
	TRAJECTORY_COLOR[2] = static_cast<float>(0.2);
	//TRAJECTORY_COLOR[3] = static_cast<float>(1.0);

	OBJECT_COLOR[0] = static_cast<float>(0.5);
	OBJECT_COLOR[1] = static_cast<float>(0.8);
	OBJECT_COLOR[2] = static_cast<float>(1.0);
	OBJECT_COLOR[3] = static_cast<float>(0.4);

	PLOT_TRAJECTORY = false;
	PLOT_MOTION = true;
	PLOT_COORDINATE_FRAME = false;
	PLOT_KEYPOSITIONS = false;
	PLOT_OBJECT = false;
	NO_OF_INTERPOS = 10;
	SPECIFC_POSITION = 0;
	OBJECT_SIZE = 0.7f;
}

MotionProperties::~MotionProperties()
{
}

float* MotionProperties::getTrajectoryColor()
{
	return TRAJECTORY_COLOR;
}

void MotionProperties::setTrajectoryColor(float (&color)[3])
{
	for (auto i = 0; i < 3; i++)
	{
		TRAJECTORY_COLOR[i] = color[i];
	}
}

void MotionProperties::setObjectColor(float (&color)[3])
{
	for (auto i = 0; i < 3; i++)
	{
		OBJECT_COLOR[i] = color[i];
	}
}

float* MotionProperties::getObjectColor()
{
	return (OBJECT_COLOR);
}

void MotionProperties::setObjectSize(float sz)
{
	OBJECT_SIZE = sz;
}

float MotionProperties::getObjectSize()
{
	return (OBJECT_SIZE);
}

void MotionProperties::plotTrajectory(bool flag)
{
	PLOT_TRAJECTORY = flag;
}

void MotionProperties::plotObject(bool flag)
{
	PLOT_OBJECT = flag;
}

bool MotionProperties::isPlotObject()
{
	return (PLOT_OBJECT);
}

bool MotionProperties::isPlotTrajectory()
{
	return (PLOT_TRAJECTORY);
}

void MotionProperties::plotMotion(bool flag)
{
	PLOT_MOTION = flag;
}

bool MotionProperties::isPlotMotion()
{
	return (PLOT_MOTION);
}

void MotionProperties::plotCoordinateFrame(bool flag)
{
	PLOT_COORDINATE_FRAME = flag;
}

bool MotionProperties::isPlotCoordinateFrame()
{
	return (PLOT_COORDINATE_FRAME);
}

int MotionProperties::getNoOfInterPos()
{
	return (NO_OF_INTERPOS);
}

void MotionProperties::setNoOfInterPos(int number)
{
	NO_OF_INTERPOS = number;
}

int MotionProperties::getSpecificPosition()
{
	return (SPECIFC_POSITION);
}

void MotionProperties::setSpecificPosition(int num)
{
	SPECIFC_POSITION = num;
}