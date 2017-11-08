#ifndef _driver_H
#define _driver_H

#include

class Drive{
	public:
	void drive(int deltaX, int deltaY);
	void rotate(int numDegrees); //contains a rotate PID loop

	private:
	//3 motors+1 accelerometer

}