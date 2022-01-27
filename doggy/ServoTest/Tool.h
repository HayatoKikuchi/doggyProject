#ifndef TOOL_H
#define TOOL_H

#define Abs(x) ((x)>0?(x):-(x))
#define _PI 3.1415926
#define Radians(x) x/360.0*2.0*_PI
#define Degrees(x) x/(2.0*_PI)*360.0
#define ErrorRate(x,y) Abs(x-y)/y

struct coord
{
    double x;
    double y;
};

class SecondOrderLag
{
    public:
        SecondOrderLag();

        void setup(double _omega, double _zeta, int _first_num, double _int_time);
        double getCmd(double num);
        void init(double num);

    private:
        double int_time;
        double omega, zeta, first_num;
        double pre_num1, pre_num2;

};

class PushCounter
{
    public:
        PushCounter();

        void setup(int digitalIn);
        void update(int digitalIn);
        int getCount(void);
        bool CountChanged(void);

    private:
        int counts;
        int sw, pre_sw;
        bool count_changed;
};

#endif