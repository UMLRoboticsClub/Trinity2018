#pragma once

class Motor {
    public:
        Motor(unsigned pinA, unsigned pinB);
        ~Motor();
        void set(int power);

    private:
        unsigned pinA, pinB;
};
