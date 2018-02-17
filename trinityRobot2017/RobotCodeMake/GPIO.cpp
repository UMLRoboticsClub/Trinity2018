#include "GPIO.h"

#include <iostream>
#include <pigpiod_if2.h>

GPIO::GPIO(){
    if((pi = pigpio_start(0, 0)) < 0){
        std::cerr << "Error: Unable to connect to pigpiod" << std::endl;  
        exit(1);
    }
}

GPIO::~GPIO(){
    pigpio_stop(0);
}
