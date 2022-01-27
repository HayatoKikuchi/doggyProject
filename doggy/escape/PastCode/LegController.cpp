#include "LegController.h"

LegController::LegController(int pin_servo_shoulder, int pin_servo_elbow)
{
    ServoShoulder.attach(pin_servo_shoulder, 1000, 2000);
    ServoElbow.attach(pin_servo_elbow, 1000, 2000);

    theta_s = Radians(90.0);
    theta_e = Radians(90.0);

    LegController::servoWriteShoulder(Degrees(theta_s));
    LegController::servoWriteElbow(Degrees(theta_e));

    LegController::Xp = LegController::getXp();
    LegController::Yp = LegController::getYp();
}

double LegController::getXp(void)
{
    //return UPPERARM * cos(theta_s) + FOREARM * cos(theta_s + theta_e);
    return UPPERARM * cos(theta_s) + FOREARM * cos(theta_e);
}

double LegController::getYp(void)
{
    //return UPPERARM * sin(theta_s) + FOREARM * sin(theta_s + theta_e);
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

    //LegController::theta_s = asin((FOREARM*FOREARM-UPPERARM*UPPERARM-Xp*Xp-Yp*Yp)/(2*UPPERARM*sqrt(Xp*Xp+Yp*Yp))) - atan(-Xp/Yp);
    LegController::theta_s = asin((Xp*Xp+Yp*Yp+UPPERARM*UPPERARM-FOREARM*FOREARM)/(2*UPPERARM*sqrt(Xp*Xp+Yp*Yp))) - atan(Xp/Yp);
    theta_s = _PI - theta_s;

    //LegController::theta_e = asin((Yp-UPPERARM*sin(theta_s))/FOREARM) - theta_s;
    LegController::theta_e = asin((Yp-UPPERARM*sin(theta_s))/FOREARM);

    LegController::servoWriteShoulder(Degrees(theta_s));
    LegController::servoWriteElbow(Degrees((_PI - theta_s) + theta_e));
}

void LegController::servoWriteShoulder(double deg)
{
    ServoShoulder.write(deg);
}

void LegController::servoWriteElbow(double deg)
{
    ServoElbow.write(deg);
}