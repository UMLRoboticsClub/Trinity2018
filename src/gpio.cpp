#include "gpio.h"
#include "logger.h"

#include <pigpiod_if2.h>

GPIO::GPIO(){
    if((pi = pigpio_start(0, 0)) < 0){
        Logger::log("Error: Unable to connect to pigpiod");
        exit(1);
    }
}

GPIO::~GPIO(){
    pigpio_stop(0);
}
