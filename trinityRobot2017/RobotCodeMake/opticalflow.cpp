#include "opticalflow.h"

#include "pins.h"
#include <pigpiod_if2.h>

#include <iostream>

using std::cout;
using std::endl;

void opticalFlow::init(){
    handle = spi_open(0, 0, 2e6, 0);
    if(handle < 0){
        cout << "that'll do it: " << handle << endl;
        cout << (handle == PI_BAD_SPI_CHANNEL) << endl;
        cout << (handle == PI_BAD_SPI_SPEED) << endl;
        cout << (handle == PI_BAD_FLAGS) << endl;
        cout << (handle == PI_NO_AUX_SPI) << endl;
        cout << (handle == PI_SPI_OPEN_FAILED) << endl;
        exit(1);
    }
    gpio_write(0, oflowMOUSECAM_CS, 1);
    reset();
    writeReg(ADNS3080_CONFIGURATION_BITS, 0x19);
}

void opticalFlow::reset(){
    gpio_write(0, oflowMOUSECAM_RESET, 1);
    time_sleep(.001);
    gpio_write(0, oflowMOUSECAM_RESET, 0);
    time_sleep(.035);
}

DoublePoint opticalFlow::readMotion(){
    MD md;
    gpio_write(0, oflowMOUSECAM_CS, 0);
    transferBits(ADNS3080_MOTION_BURST);
    time_sleep(0.000075);
    md.motion = transferBits(0xff);
    md.dx = transferBits(0xff);
    md.dy = transferBits(0xff);
    md.squal = transferBits(0xff);
    md.shutter = transferBits(0xff) << 8;
    md.shutter |= transferBits(0xff);
    md.max_pix = transferBits(0xff);
    gpio_write(0, oflowMOUSECAM_CS, 1);

    DoublePoint data(md.dx, md.dy);
    if(data.x > 127)
        data.x -= 256;
    if(data.y > 127)
        data.y -=256;
    data*=-0.1178;
//    data.y *= -1;
//    cout << data.x << " " << data.y << " " ;
    return data;
}

void opticalFlow::writeReg(int reg, int val){
    gpio_write(0, oflowMOUSECAM_CS, 0);
    transferBits(reg | 0x80);
    transferBits(val);
    gpio_write(0, oflowMOUSECAM_CS, 1);
    time_sleep(.00005);
}

int opticalFlow::readReg(int reg){
    gpio_write(0, oflowMOUSECAM_CS, 0);
    transferBits(reg);
    time_sleep(.000075);
    int ret = transferBits(0xff);
    gpio_write(1, oflowMOUSECAM_CS, 1);
    time_sleep(.000001);
    return ret;
}

char opticalFlow::transferBits(char toTransfer){
    char ret = 7;
    spi_xfer(0, handle, &toTransfer, &ret, 1);
    return ret;
}

char opticalFlow::transferBits(int toTransfer){
    char ret = 7;
    char toTransferC = char(toTransfer);
    spi_xfer(0, handle, &toTransferC, &ret, 1);
    return ret;
}
