#include "lidar.h"
#include <iostream>
#include <csignal>

Lidar lidar;
bool running = true;

void signalHandler(int signum){
    running = false;
}

int main(){
    signal(SIGINT , signalHandler);
    signal(SIGABRT, signalHandler);
    signal(SIGFPE , signalHandler);
    signal(SIGILL , signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGHUP , signalHandler);

    while(running){
        lidar.scan();
    }
    cout << "\nAborting..." << endl;
}
