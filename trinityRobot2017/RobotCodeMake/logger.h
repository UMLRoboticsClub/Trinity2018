#pragma once

#include <iostream>
#include <string>

#define LOGGING

#define RST "\033[0m"
#define BOLD "\033[1m"
#define RED "\033[31m"
#define GRN "\033[32m"
#define BLU "\033[34m"
#define YEL "\033[33m"

namespace Logger {
    enum priority { LOW, MED, HIGH };

    static void log(std::string str, int priority = LOW){
#ifdef LOGGING
        std::cout << BOLD << BLU << "DEBUG: " << RST;
        switch(priority){
            case LOW:
                break;
            case MED:
                std::cout << YEL;
                break;
            case HIGH:
                std::cout << RED;
                break;
        };
        std::cout << str << RST << std::endl;
#endif
    }
}
