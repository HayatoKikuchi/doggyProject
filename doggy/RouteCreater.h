#ifndef ROUTECREATER_H
#define ROUTECREATER_H

#include "Tool.h"

#define MODE_TRIANGLE 1
#define MODE_ELLIPSE 2

#define X1 1
#define X2 2
#define X3 3
#define Y1 4
#define Y2 5

class RouteCreater
{
public:
    RouteCreater(int mode);

    void setRouteMode(int _route_mode);
    void setParam(double setNum, int kind_of_param, int _route_mode);
    void init(double vel);
    void updateParam(double _vel);

    double getPositionX(double time_seconds);
    double getPositionY(double time_seconds);
    void getPosition(double *Xp, double *Yp, double *time_seconds);

    double getPeriod(void);

    /*
     @param _x1 踏み出し位置
     @param _x2 蹴り位置
     @param _x3 変曲点
     @param _y1 地面まで相対距離
     @param _y2 足上げ位置
    */
    void setParam_triangle(double _x1, double _x2, double _x3, double _y1, double _y2);

    /*
     @param _x1 踏み出し位置
     @param _x2 蹴り位置
     @param _y1 地面まで相対距離
     @param _y2 足上げ位置
    */
    void setParam_ellipse(double _x1, double _x2, double _y1, double _y2);

private: // general


    bool flag_init;
    bool param_changed;
    int route_mode;
    double velocity;

private: // for MODE_TRIANGLE

    struct Triangle
    {
        double x1, x2, x3, y1, y2;
        double t1, t2, t3;
        double kx1, kx2, kx3, ky1, ky2;
        double ratio_a, ratio_b;
    };

    Triangle triangle;

private: // for MODE_ELLIPSE

    struct Ellipse
    {
        double x1, x2, y1, y2;
        double T;
        double k;
        double a, b;
    };
    
    Ellipse ellipse;
};

#endif