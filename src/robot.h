#pragma once

#include "point.h"
#include "mazemapper.h"
#include "gamestate.h"
#include "globals.h"
#include "constants.h"
#include "irsensor.h"
#include "gpio.h"
#include "drive.h"

#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <atomic>
#include <cmath>

#define SIDE_LEFT "side_left"
#define SIDE_RIGHT "side_right"
#define ROBOT_SAFE_TURN_CM 5

using std::string;

class Robot {
    public:
        Robot();
        ~Robot();

        void start();

        void robotLoop();

        void robotDrive(std::vector<Point> instructions);
        void getBaby(Point targetPoint);
        void tossBaby(Point targetPoint);
        void blowCandle(Point targetPoint);
        void spinAndScan();
        void hallwaySweep(Point targetPoint);
        void hallwaySimple();
        void goToFrontFromSide(Point targetPoint, string side);
        void leaveRoom();

        void rotateTowards(DoublePoint targetPoint);

        static std::atomic<bool> done;

        //private:
        GPIO gpio; //needs to be initialized before all sensors
        std::thread laserScanInputThread;

        MazeMapper  mazeMapper;
        GameState	gameState;
        Point		safeZoneLocation;
        Drive       drive;
};
