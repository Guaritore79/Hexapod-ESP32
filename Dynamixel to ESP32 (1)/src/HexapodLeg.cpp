#include "HexapodLeg.h"

HexapodLeg::HexapodLeg(float lenC, float lenF, float lenT){
    coxaLength = lenC;
    femurLength = lenF;
    tibiaLength = lenT;
}

float HexapodLeg::safeAcos(float x){
    return acos(constrain(x, -1.0f, 1.0f));
}

void HexapodLeg::inverseKinematic(float xInput, float yInput, float zInput){
    float yRest = 100; //posisi femur diam
    float zRest = -40; //posisi tibia diam

    float yTotal = yRest - yInput;
    float zTotal = zRest - zInput;

    float gamma = atan2(xInput, yTotal);
    this->coxaDeg = degrees(gamma);

    float totalHorizontal = sqrt(sq(xInput) + sq(yTotal));
    float effHorizontal = totalHorizontal - coxaLength;

    float l = sqrt(sq(effHorizontal) + sq(zTotal));

    float maxReach = femurLength + tibiaLength;
    if(l > maxReach) l = maxReach;
    if(l <1.0f) l =1.0f;

    // float tibiaAngle = acos((sq(femurLength) + sq(tibiaLength) - sq(l)) / (2 * femurLength * tibiaLength)); // tibiaAngle += (PI / 2.0);
    float cosTibia = (sq(femurLength) + sq(tibiaLength) - sq(l)) / (2 * femurLength * tibiaLength);

    float tibiaAngle = safeAcos(cosTibia);

    float cosFemur = (sq(femurLength) + sq(l) - sq(tibiaLength)) / (2 * femurLength * l);
    float vb = safeAcos(cosFemur);

    // float vb = acos((sq(femurLength) + sq(l) - sq(tibiaLength)) / ( 2 * femurLength * l));
    float va = atan2(zTotal, effHorizontal);
    float femurAngle = va + vb + PI;

    this->femurDeg = degrees(femurAngle);
    this->tibiaDeg = degrees(tibiaAngle);

}

int HexapodLeg::getValCoxa(bool invert){
    float val = coxaDeg + 170;
    return invert ? map(val, 0, 360, 4095, 0) : map(val, 0, 360, 0, 4095);
}

int HexapodLeg::getValFemur(bool invert){
    return invert ? map(femurDeg, 0, 360, 4095, 0) : map(femurDeg, 0, 360, 0, 4095);
}

int HexapodLeg::getValTibia(bool invert){
    float val = tibiaDeg + 120;
    return invert ? map(val, 0, 360, 4095, 0) : map(val, 0, 360, 0, 4095);
}