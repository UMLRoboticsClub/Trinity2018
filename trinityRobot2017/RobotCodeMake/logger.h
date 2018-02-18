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
        cout << BOLD << BLU << "DEBUG: " << RST;
        switch(priority){
            case LOW:
                break;
            case MED:
                cout << YEL;
                break;
            case HIGH:
                cout << RED;
                break;
        };
        cout << str << RST << endl;
#endif
    }
}
