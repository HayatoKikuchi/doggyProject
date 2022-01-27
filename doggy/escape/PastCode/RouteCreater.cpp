#include "RouteCreater.h"
#include <math.h>

RouteCreater::RouteCreater(int mode, double _int_time)
{
    RouteCreater::flag_init = false;
    RouteCreater::param_changed = true;
    RouteCreater::velocity = 0.0;
    RouteCreater::route_mode = mode;
    RouteCreater::int_time = _int_time;
}

void RouteCreater::setRouteMode(int _route_mode)
{
    RouteCreater::route_mode = _route_mode;
    RouteCreater::param_changed = true;
}

void RouteCreater::calcuParam(double _vel)
{
    if(_vel != velocity) RouteCreater::param_changed = true;
    RouteCreater::velocity = _vel;

    if(RouteCreater::param_changed)
    {
        switch (RouteCreater::route_mode)
        {
        case MODE_TRIANGLE:

            RouteCreater::ratio_a = fabs(x3 - x2);
            RouteCreater::ratio_b = fabs(x1 - x3);

            RouteCreater::t1 = fabs(x2 - x1) / velocity;
            RouteCreater::t2 = t1 + ratio_a / (ratio_a + ratio_b) * t1;
            RouteCreater::t3 = t2 + ratio_b / (ratio_a + ratio_b) * t1;

            RouteCreater::kx1 = (x2 - x1) / t1;
            RouteCreater::kx2 = (x3 - x2) / (t2 - t1);
            RouteCreater::kx3 = (x1 - x3) / (t3 - t2);

            RouteCreater::ky1 = (y2 - y1) / (t2 - t1);
            RouteCreater::ky2 = (y1 - y2) / (t3 - t2);

            RouteCreater::period_2 = t1;

            break;
        
        default:
            break;
        }

        param_changed = false;
    }
}

void RouteCreater::getPosition(double *Xp1, double *Yp1, double *Xp2, double *Yp2, double velocity)
{
    RouteCreater::calcuParam(velocity);

    static double t_p1 = 0.0;
    static double t_p2 = RouteCreater::period_2;

    t_p1 += int_time;
    t_p2 += int_time;
    
    switch (RouteCreater::route_mode)
    {
    case MODE_TRIANGLE: 
        if(t_p1 > t3) t_p1 = 0.0;
        if(t_p2 > t3) t_p1 = 0.0;
        break;
    
    default:
        break;
    }

    *Xp1 = RouteCreater::getPositionX(t_p1);
    *Yp1 = RouteCreater::getPositionY(t_p1);

    *Xp2 = RouteCreater::getPositionX(t_p2);
    *Yp2 = RouteCreater::getPositionY(t_p2);}

double RouteCreater::getPositionX(double time_seconds)
{
    double cmd;

    switch (RouteCreater::route_mode)
    {
    case MODE_TRIANGLE:

        if(0 <= time_seconds && time_seconds < t1) cmd = kx1 * time_seconds + x2 - kx1 * t1;
        else if(t1 <= time_seconds && time_seconds < t2) cmd = kx2 * time_seconds + x3 - kx2 * t2;
        else if(t2 <= time_seconds && time_seconds < t3) cmd = kx3 * time_seconds + x1 -kx3 * t3;

        break;
    
    default:
        break;
    }

    return cmd;
}

double RouteCreater::getPositionY(double time_seconds)
{
    double cmd;

    switch (route_mode)
    {
    case MODE_TRIANGLE:

        if(0 <= time_seconds && time_seconds < t1) cmd = y1;
        else if(t1 <= time_seconds && time_seconds < t2) cmd = ky1 * time_seconds + y2 - ky1 * t2;
        else if(t2 <= time_seconds && time_seconds < t3) cmd = ky2 * time_seconds + y1 - ky2 * t3;

        break;
    
    default:
        break;
    }

    return cmd;
}

void RouteCreater::setParam(double setNum, int kind_of_param, int _route_mode)
{
    param_changed = true;

    switch (_route_mode)
    {
    case MODE_TRIANGLE:
        switch (kind_of_param)
        {
        case X1: RouteCreater::x1 = setNum; break;
        case X2: RouteCreater::x2 = setNum; break;
        case X3: RouteCreater::x3 = setNum; break;
        case Y1: RouteCreater::y1 = setNum; break;
        case Y2: RouteCreater::y2 = setNum; break;
        
        default:
            break;
        }
        
        break;
    
    default:
        break;
    }
}

void RouteCreater::getFirstPoint(double *Xp, double *Yp)
{
    *Xp = RouteCreater::getPositionX(0.0);
    *Yp = RouteCreater::getPositionY(0.0);
}