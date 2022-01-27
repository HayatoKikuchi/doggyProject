#ifndef ROUTECREATER_H
#define ROUTECREATER_H

#include "Tool.h"

#define MODE_TRIANGLE 1

#define X1 1
#define X2 2
#define X3 3
#define Y1 4
#define Y2 5

class RouteCreater
{
public:
    RouteCreater(int mode, double _int_time);

    void calcuParam(double _vel);
    void getFirstPoint(double *Xp, double *Yp);
    void getPosition(double *Xp1, double *Yp1, double *Xp2, double *Yp2, double velocity);

    void setRouteMode(int _route_mode);

    void setParam(double setNum, int kind_of_param, int _route_mode);

private: // general

    bool flag_init;
    bool param_changed;
    int route_mode;
    double velocity;
    double period_2;
    double int_time;

    double getPositionX(double time_seconds);
    double getPositionY(double time_seconds);

private: // for MODE_TRIANGLE

    double x1, x2, x3, y1, y2;
    double t1, t2, t3;
    double kx1, kx2, kx3, ky1, ky2;
    double ratio_a, ratio_b;
};

#endif
