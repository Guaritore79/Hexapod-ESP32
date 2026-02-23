#ifndef HEXAPOD_LEG_H
#define HEXAPOD_LEG_H

#include <Arduino.h>

class HexapodLeg{
    private:
        float coxaLength, femurLength, tibiaLength;
        float coxaDeg, femurDeg, tibiaDeg;
        float safeAcos(float x);

    public:
        HexapodLeg(float lenC = 40, float lenF = 80, float lenT = 62);
        void inverseKinematic(float xInput, float yInput, float zInput);

        int getValCoxa(bool invert = false);
        int getValFemur(bool invert = false);
        int getValTibia(bool invert = false);
};

#endif