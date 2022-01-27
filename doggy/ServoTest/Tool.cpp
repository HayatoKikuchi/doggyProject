#include "Tool.h"

SecondOrderLag::SecondOrderLag()
{
}

void SecondOrderLag::setup(double _omega, double _zeta, int _first_num, double _int_time)
{
    SecondOrderLag::omega = _omega;
    SecondOrderLag::zeta = _zeta;
    SecondOrderLag::first_num = _first_num;
    SecondOrderLag::pre_num2 = SecondOrderLag::pre_num1 = _first_num;
    SecondOrderLag::int_time = _int_time;
}

double SecondOrderLag::getCmd(double num)
{
    double ret = (2*pre_num1*(1+zeta*omega*int_time) - pre_num2 + omega*omega*int_time*int_time*num)
            / (1 + 2*zeta*omega*int_time + omega*omega*int_time*int_time);
    pre_num2 = pre_num1;
    pre_num1 = ret;

    return ret;
}

void SecondOrderLag::init(double num)
{
    SecondOrderLag::first_num = num;
    SecondOrderLag::pre_num2 = SecondOrderLag::pre_num1 = num;
}


PushCounter::PushCounter()
{
    PushCounter::counts = 0;
    PushCounter::count_changed = false;
}

void PushCounter::setup(int digitalIn)
{
    PushCounter::pre_sw = digitalIn;
}

void PushCounter::update(int digitalIn)
{
    sw = digitalIn;
    if(sw < pre_sw)
    {
        counts++;
        count_changed = true;
    }
    else count_changed = false;
    pre_sw = sw;
}

int PushCounter::getCount(void)
{
    return PushCounter::counts;
}

bool PushCounter::CountChanged(void)
{
    return PushCounter::count_changed;
}