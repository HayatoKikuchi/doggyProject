#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include <math.h>
#include <Servo.h>
#include "Tool.h"

#define UPPERARM 37.2 //[mm]
#define FOREARM 49.9 //[mm]

class LegController
{
public:

    LegController(int pin_servo_shoulder, bool _invertS, int pin_servo_elbow, bool _invertE);

    double init(void);

    double getXp(void);
    double getYp(void);

    double getThetaShoulder(void);
    double getThetaElbow(void);

    void MoveLegPosition(double _Xp, double _Yp);

private:

    Servo ServoShoulder, ServoElbow;
    
    double degInvert(double cmd, bool invert);
    void servoWriteShoulder(double deg);
    void servoWriteElbow(double deg);


    double theta_s, theta_e; //[rad]
    double Xp, Yp; //[mm]
    bool invertS, invertE;

};

#endif