#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include <math.h>
#include <Servo.h>
#include "Tool.h"

#define UPPERARM 32.7 //[mm] //ダミー
#define FOREARM 48.5 //[mm] //ダミー

class LegController
{
public:

    LegController(int pin_servo_shoulder, int pin_servo_elbow);

    double getXp(void);
    double getYp(void);

    double getThetaShoulder(void);
    double getThetaElbow(void);

    void MoveLegPosition(double _Xp, double _Yp);

    void servoWriteShoulder(double deg);
    void servoWriteElbow(double deg);

private:

    Servo ServoShoulder, ServoElbow;

    double theta_s, theta_e; //[rad]
    double Xp, Yp; //[mm]

};

#endif