#ifndef MOTIONPROPERTIES_H
#define MOTIONPROPERTIES_H


class MotionProperties
{
public:
	MotionProperties();
	~MotionProperties();
	float* getTrajectoryColor();
	void setTrajectoryColor(float(&color)[3]);
	
	void setObjectColor(float (&color)[3]);
	float* getObjectColor();

	void setObjectSize(float);
	float getObjectSize();

	void plotTrajectory(bool);
	bool isPlotTrajectory();
	
	void plotObject(bool);
	bool isPlotObject();

	void plotMotion(bool);
	bool isPlotMotion();
	
	void plotCoordinateFrame(bool);
	bool isPlotCoordinateFrame();
	
	int getNoOfInterPos();
	void setNoOfInterPos(int n);
	
	int getSpecificPosition();
	void setSpecificPosition(int num);
private:
	float TRAJECTORY_COLOR[3];
	float OBJECT_COLOR[3];
	float OBJECT_SIZE;
	bool PLOT_TRAJECTORY;
	bool PLOT_MOTION;
	bool PLOT_KEYPOSITIONS;
	bool PLOT_OBJECT;
	bool PLOT_COORDINATE_FRAME;
	int NO_OF_INTERPOS;
	int SPECIFC_POSITION;
};

#endif