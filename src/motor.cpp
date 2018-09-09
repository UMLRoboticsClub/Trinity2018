#include "motor.h"

#include <iostream>
#include <pigpiod_if2.h>

Motor::Motor(unsigned pinA, unsigned pinB):
    pinA(pinA), pinB(pinB)
{
    set_mode(0, pinA, PI_OUTPUT);  
    set_mode(0, pinB, PI_OUTPUT);  
}

Motor::~Motor(){
    gpio_write(0, pinA, 0);
    gpio_write(0, pinB, 0);
}

void Motor::set(int power){
    if(power > 255){
        std::cerr << "Error: motor power >255" << std::endl;
        power = 255;
    } else if(power < -255){
        std::cerr << "Error: motor power <-255" << std::endl;
        power = -255;
    }

    if(power < 0){
        set_PWM_dutycycle(0, pinA, 0);
        set_PWM_dutycycle(0, pinB, -power);
    } else {
        set_PWM_dutycycle(0, pinB, 0);
        set_PWM_dutycycle(0, pinA, power);
    }
}
