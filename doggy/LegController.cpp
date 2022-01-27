#include "LegController.h"

LegController::LegController(int pin_servo_shoulder, bool _invertS, int pin_servo_elbow, bool _invertE)
{
    ServoShoulder.attach(pin_servo_shoulder);
    ServoElbow.attach(pin_servo_elbow);

    LegController::invertS = _invertS;
    LegController::invertE = _invertE;
}

double LegController::init(void)
{
    LegController::MoveLegPosition(20.0, 35.0);
}

double LegController::getXp(void)
{
    return UPPERARM * cos(theta_s) + FOREARM * cos(theta_e);
}

double LegController::getYp(void)
{
    return UPPERARM * sin(theta_s) + FOREARM * sin(theta_e);
}

double LegController::getThetaShoulder(void)
{
    return LegController::theta_s;
}

double LegController::getThetaElbow(void)
{
    return LegController::theta_e;
}

void LegController::MoveLegPosition(double _Xp, double _Yp)
{
    LegController::Xp = _Xp;
    LegController::Yp = _Yp;

    LegController::theta_s = _PI - asin((Xp*Xp+Yp*Yp+UPPERARM*UPPERARM-FOREARM*FOREARM)/(2*UPPERARM*sqrt(Xp*Xp+Yp*Yp))) - atan(Xp/Yp);

    LegController::theta_e = asin((Yp-UPPERARM*sin(theta_s))/FOREARM);

    LegController::servoWriteShoulder(Degrees(theta_s));
    double theta_e_servo = (_PI - theta_s) + theta_e;
    LegController::servoWriteElbow(Degrees(theta_e_servo));
}

double LegController::degInvert(double deg, bool invert)
{
    if(invert) return 180.0 - deg;
    else return deg;
}

void LegController::servoWriteShoulder(double deg)
{
    ServoShoulder.write(degInvert(deg, LegController::invertS));
}

void LegController::servoWriteElbow(double deg)
{
    ServoElbow.write(degInvert(deg, LegController::invertE));
}