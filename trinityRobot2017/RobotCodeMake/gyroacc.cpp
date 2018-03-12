#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <cmath>
#include "gyroacc.h"

#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R

#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R

#define MPU6050_PWR_MGMT_1         0x6B   // R/W
//#define MPU6050_I2C_ADDRESS        0x68   // I2C
#define MPU6050_I2C_ADDRESS        0x29   // I2C

MotionSensor::MotionSensor(int fd) {
	//fd = wiringPiI2CSetup(MPU6050_I2C_ADDRESS);
	this->fd = fd;
	if (fd == -1) {
		return;
	}
	zAdjust = 0;
	zAdjust = getGyroZ();
	wiringPiI2CReadReg8(fd, MPU6050_PWR_MGMT_1);
	wiringPiI2CWriteReg16(fd, MPU6050_PWR_MGMT_1, 0);
}

int8_t MotionSensor::getGyroX() {
	return (int8_t)wiringPiI2CReadReg8(fd, MPU6050_GYRO_XOUT_H);
}

int8_t MotionSensor::getGyroY() {
	return (int8_t)wiringPiI2CReadReg8(fd, MPU6050_GYRO_YOUT_H);
}

int8_t MotionSensor::getGyroZ() {
	return (int8_t)wiringPiI2CReadReg8(fd, MPU6050_GYRO_ZOUT_H) - zAdjust;
}

int8_t MotionSensor::getAccelX() {
	return (int8_t)wiringPiI2CReadReg8(fd, MPU6050_ACCEL_XOUT_H);
}

int8_t MotionSensor::getAccelY() {
	return (int8_t)wiringPiI2CReadReg8(fd, MPU6050_ACCEL_YOUT_H);
}

int8_t MotionSensor::getAccelZ() {
	return (int8_t)wiringPiI2CReadReg8(fd, MPU6050_ACCEL_ZOUT_H);
}

float MotionSensor::getAngleX() {
	int x = getAccelX();
	int y = getAccelY();
	int z = getAccelZ();
	float ax = atan(x / (sqrt(y*y + z*z))) * 180 / M_PI;
	return ax;
}

float MotionSensor::getAngleY() {
	int x = getAccelX();
	int y = getAccelY();
	int z = getAccelZ();
	float ay = atan(y / (sqrt(x*x + z*z))) * 180 / M_PI;
	return ay;
}

float MotionSensor::getAngleZ() {
	int x = getAccelX();
	int y = getAccelY();
	int z = getAccelZ();
	float az = atan((sqrt(y*y + x*x)) / z) * 180 / M_PI;
	return az;
}
