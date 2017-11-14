#pragma once

class Drive{
	public:
	Drive();
	void drive(int deltaX, int deltaY);
	void rotate(int numDegrees); //contains a rotate PID loop

	private:
	//3 motors+1 accelerometer
}