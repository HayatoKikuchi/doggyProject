#include "RouteCreater.h"
#include <math.h>

RouteCreater::RouteCreater(int mode)
{
    RouteCreater::flag_init = false;
    RouteCreater::param_changed = true;
    RouteCreater::route_mode = mode;
}

void RouteCreater::setRouteMode(int _route_mode)
{
    RouteCreater::route_mode = _route_mode;
    RouteCreater::param_changed = true;
}

void RouteCreater::init(double vel)
{
    RouteCreater::updateParam(vel);
}

void RouteCreater::updateParam(double _vel)
{
    if(_vel != velocity) RouteCreater::param_changed = true;
    RouteCreater::velocity = _vel;

    if(RouteCreater::param_changed)
    {
        switch (RouteCreater::route_mode)
        {
        case MODE_TRIANGLE:

            RouteCreater::triangle.ratio_a = fabs(triangle.x3 - triangle.x2);
            RouteCreater::triangle.ratio_b = fabs(triangle.x1 - triangle.x3);

            RouteCreater::triangle.t1 = fabs(triangle.x2 - triangle.x1) / velocity;
            RouteCreater::triangle.t2 = triangle.t1 + triangle.ratio_a / (triangle.ratio_a + triangle.ratio_b) * triangle.t1;
            RouteCreater::triangle.t3 = triangle.t2 + triangle.ratio_b / (triangle.ratio_a + triangle.ratio_b) * triangle.t1;

            RouteCreater::triangle.kx1 = (triangle.x2 - triangle.x1) / triangle.t1;
            RouteCreater::triangle.kx2 = (triangle.x3 - triangle.x2) / (triangle.t2 - triangle.t1);
            RouteCreater::triangle.kx3 = (triangle.x1 - triangle.x3) / (triangle.t3 - triangle.t2);

            RouteCreater::triangle.ky1 = (triangle.y2 - triangle.y1) / (triangle.t2 - triangle.t1);
            RouteCreater::triangle.ky2 = (triangle.y1 - triangle.y2) / (triangle.t3 - triangle.t2);

            break;
        
        case MODE_ELLIPSE:

            RouteCreater::ellipse.a = fabs(ellipse.x2 - ellipse.x1) / 2.0;
            RouteCreater::ellipse.b = fabs(ellipse.y2 - ellipse.y1);
            RouteCreater::ellipse.T = fabs(ellipse.x2 - ellipse.x1) / velocity * 2.0;
            RouteCreater::ellipse.k = (ellipse.x2 - ellipse.x1) / (ellipse.T / 2.0);
        
        default:
            break;
        }

        param_changed = false;
    }
}

void RouteCreater::getPosition(double *Xp, double *Yp, double *time_seconds)
{
    double _t = *time_seconds;
    
    switch (RouteCreater::route_mode)
    {
    case MODE_TRIANGLE: 
        if(_t > triangle.t3) *time_seconds = _t = 0.0;
        break;
    
    case MODE_ELLIPSE:
        if(_t > ellipse.T) *time_seconds = _t = 0.0;
    
    default:
        break;
    }

    *Xp = RouteCreater::getPositionX(_t);
    *Yp = RouteCreater::getPositionY(_t);
}

double RouteCreater::getPositionX(double time_seconds)
{
    double cmd;

    switch (RouteCreater::route_mode)
    {
    case MODE_TRIANGLE:

        if(0 <= time_seconds && time_seconds < triangle.t1)
            cmd = triangle.kx1 * time_seconds + triangle.x2 - triangle.kx1 * triangle.t1;

        else if(triangle.t1 <= time_seconds && time_seconds < triangle.t2) 
            cmd = triangle.kx2 * time_seconds + triangle.x3 - triangle.kx2 * triangle.t2;

        else if(triangle.t2 <= time_seconds && time_seconds < triangle.t3) 
            cmd = triangle.kx3 * time_seconds + triangle.x1 -triangle.kx3 * triangle.t3;

        break;
    
    case MODE_ELLIPSE:

        if(0 <= time_seconds && time_seconds < ellipse.T/2.0)
            cmd = ellipse.k * time_seconds + ellipse.x1;

        else if(ellipse.T/2.0 <= time_seconds && time_seconds <= ellipse.T)
            cmd = ellipse.a * cos(2.0 * _PI / ellipse.T * time_seconds) + (ellipse.x1 - ellipse.a);
        
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

        if(0 <= time_seconds && time_seconds < triangle.t1)
            cmd = triangle.y1;

        else if(triangle.t1 <= time_seconds && time_seconds < triangle.t2) 
            cmd = triangle.ky1 * time_seconds + triangle.y2 - triangle.ky1 * triangle.t2;

        else if(triangle.t2 <= time_seconds && time_seconds < triangle.t3) 
            cmd = triangle.ky2 * time_seconds + triangle.y1 - triangle.ky2 * triangle.t3;

        break;

    case MODE_ELLIPSE:

        if(0 <= time_seconds && time_seconds < ellipse.T/2.0)
            cmd = ellipse.y1;

        else if(ellipse.T/2.0 <= time_seconds && time_seconds <= ellipse.T)
            cmd = ellipse.a * sin(2.0 * _PI / ellipse.T * time_seconds) + ellipse.y1;
    
    default:
        break;
    }

    return cmd;
}

void RouteCreater::setParam(double setNum, int kind_of_param, int _route_mode)
{
    RouteCreater::param_changed = true;

    switch (_route_mode)
    {
    case MODE_TRIANGLE:

        switch (kind_of_param)
        {

            case X1: RouteCreater::triangle.x1 = setNum; break;
            case X2: RouteCreater::triangle.x2 = setNum; break;
            case X3: RouteCreater::triangle.x3 = setNum; break;
            case Y1: RouteCreater::triangle.y1 = setNum; break;
            case Y2: RouteCreater::triangle.y2 = setNum; break;
            
            default:
                break;
        }

        break;
    
    case MODE_ELLIPSE:

        switch (kind_of_param)
        {

            case X1: RouteCreater::ellipse.x1 = setNum; break;
            case X2: RouteCreater::ellipse.x2 = setNum; break;
            case Y1: RouteCreater::ellipse.y1 = setNum; break;
            case Y2: RouteCreater::ellipse.y2 = setNum; break;
            
            default:
                break;
        }
        
        break;
    
    default:
        break;
    }
}

void RouteCreater::setParam_triangle(double _x1, double _x2, double _x3, double _y1, double _y2)
{
    RouteCreater::param_changed = true;

    RouteCreater::triangle.x1 = _x1;
    RouteCreater::triangle.x2 = _x2;
    RouteCreater::triangle.x3 = _x3;
    RouteCreater::triangle.y1 = _y1;
    RouteCreater::triangle.y2 = _y2;
}

void RouteCreater::setParam_ellipse(double _x1, double _x2, double _y1, double _y2)
{
    RouteCreater::param_changed = true;

    RouteCreater::ellipse.x1 = _x1;
    RouteCreater::ellipse.x2 = _x2;
    RouteCreater::ellipse.y1 = _y1;
    RouteCreater::ellipse.y2 = _y2;
}

double RouteCreater::getPeriod(void)
{
    switch (RouteCreater::route_mode)
    {

    case MODE_TRIANGLE: return RouteCreater::triangle.t1; break;
    case MODE_ELLIPSE: return RouteCreater::ellipse.T; break;
    
    default:
        break;
    }
}