#include <iostream>
#include <csignal>
#include <wiringPi.h>
#include <softPwm.h>

const int lightpin = 2;
const int delayms = 400;

const int maxBrightness = 100;

void signalHandler(int signum){
    digitalWrite(lightpin, LOW);

    exit(signum);
}

int main(){
    wiringPiSetupGpio();

    softPwmCreate(lightpin, 0, maxBrightness);

    signal(SIGINT , signalHandler);
    signal(SIGABRT, signalHandler);
    signal(SIGFPE , signalHandler);
    signal(SIGILL , signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGHUP , signalHandler);
    
    unsigned int brightness = 0;
    int increm = 1;
    while(1){
        softPwmWrite(lightpin, brightness);
        brightness += increm;
        if(brightness == maxBrightness - 15){
            increm = -1;
        } else if(brightness == 5){
            increm = 1;
        }
        delay(20);
    }
}
