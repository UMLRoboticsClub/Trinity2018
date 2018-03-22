#include "gyroacc.h"
#include "logger.h"

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <pigpiod_if2.h>

#define MPU6050_GYRO_XOUT_H  0x43   // R
#define MPU6050_GYRO_YOUT_H  0x45   // R
#define MPU6050_GYRO_ZOUT_H  0x47   // R

#define MPU6050_ACCEL_XOUT_H 0x3B   // R
#define MPU6050_ACCEL_YOUT_H 0x3D   // R
#define MPU6050_ACCEL_ZOUT_H 0x3F   // R

#define MPU6050_PWR_MGMT_1   0x6B   // R/W
//#define MPU6050_I2C_ADDRESS   0x68   // I2C
#define MPU6050_I2C_ADDRESS  0x29   // I2C

#define I2CBUS 1

MotionSensor::MotionSensor():
    fd(i2c_open(0, I2CBUS, MPU6050_I2C_ADDRESS, 0)){
        if(fd < 0){
            Logger::log("Unable to open i2c bus!"); 
            return;
        }
        zAdjust = 0;
        zAdjust = getGyroZ();
        //wiringPiI2CReadReg8(fd, MPU6050_PWR_MGMT_1);
        //wiringPiI2CWriteReg16(fd, MPU6050_PWR_MGMT_1, 0);
    }

MotionSensor::~MotionSensor(){
    i2c_close(0, fd);
}

int8_t MotionSensor::getGyroX() {
    return i2c_read_byte_data(0, fd, MPU6050_GYRO_XOUT_H);
}

int8_t MotionSensor::getGyroY() {
    return i2c_read_byte_data(0, fd, MPU6050_GYRO_YOUT_H);
}

int8_t MotionSensor::getGyroZ() {
    return i2c_read_byte_data(0, fd, MPU6050_GYRO_ZOUT_H) - zAdjust;
}

int8_t MotionSensor::getAccelX() {
    return i2c_read_byte_data(0, fd, MPU6050_ACCEL_XOUT_H);
}

int8_t MotionSensor::getAccelY() {
    return i2c_read_byte_data(0, fd, MPU6050_ACCEL_YOUT_H);
}

int8_t MotionSensor::getAccelZ() {
    return i2c_read_byte_data(0, fd, MPU6050_ACCEL_ZOUT_H);
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
