#include <iostream>
#include <wiringPi.h>

const int pin = 3;

void interrupt(){
    system("sudo shutdown -h now 'Powering off...'");
    exit(0);
}

int main(){
    wiringPiSetupGpio();

    if(wiringPiISR(pin, INT_EDGE_BOTH, &interrupt) < 0){
        std::cerr << "Unable to set ISR" << std::endl;
        return 1;
    }

    while(1){
        delay(10000);
    }
}
