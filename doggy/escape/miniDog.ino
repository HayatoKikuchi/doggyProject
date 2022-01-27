#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include "MBED_RPi_Pico_TimerInterrupt.h"
#include "LegController.h"
#include "RouteCreater.h"

#define INT_TIME 0.01 // [s]
#define INT_TIME_MS 10 // [ms]

MBED_RPI_PICO_Timer ITimer1(1);

LegController leg(2, false, 3, true); //ダミー
//LegController ForeFootR(2, true, 3, true); //cheacked //FR
//LegController ForeFootL(4, false, 5, true); //cheacked //FL 
//LegController HindLegR(6, false, 7, true); // cheacked //HR
//LegController HindLegL(8, false, 9, true); //cheaked //HL

RouteCreater route(MODE_ELLIPSE);

bool flag_10ms = false;
double time_seconds = 0.0;
int time_count = 0;
double velocity = 50.0; //[mm/s]
double x_fr = 0.0, y_fr = 0.0;

void interrupt(uint alarm_num)
{
    // Always call this for MBED RP2040 before processing ISR
    TIMER_ISR_START(alarm_num);

    flag_10ms = true;
    time_seconds += INT_TIME;
    time_count++;

    // Always call this for MBED RP2040 after processing ISR
    TIMER_ISR_END(alarm_num);
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    route.setParam_triangle(15.0, -10.0, 7.5, 70.0, 60.0);
    route.setParam_ellipse(15.0, -10.0, 70.0, 60.0);

    x_fr = ForeFootR.getXp();
    y_fr = ForeFootR.getYp();

    Serial.begin(115200);
    
    if (ITimer1.attachInterruptInterval(INT_TIME_MS * 1000, interrupt))
    {
        Serial.print(F("Starting ITimer1 OK, millis() = ")); Serial.println(millis());
    }
    else Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

}

void loop()
{
    if(flag_10ms)
    {
        route.getPosition(&x_fr, &y_fr, velocity, &time_seconds);
        leg.MoveLegPosition(x_fr, y_fr);

        double theta_fr_shoulder = leg.getThetaShoulder();
        double theta_fr_elbow = leg.getThetaElbow();
        double pointX = ForeFootR.getXp();
        double pointY = ForeFootR.getYp();
/*
        Serial.print(Degrees(theta_fr_shoulder));
        Serial.print("  ");
        Serial.print(Degrees(theta_fr_elbow));
        Serial.print("  ");
        Serial.print(x_fr);
        Serial.print(",");
        Serial.print(pointX);
        Serial.print("  ");
        Serial.print(y_fr);
        Serial.print(",");
        Serial.println(pointY);
*/
        Serial.print(x_fr);
        Serial.print("\t");
        Serial.println(y_fr);

        LEDrgb(2);

        flag_10ms = false;
    }
}

void LEDrgb(int num)
{
    static int _time_count = 0;
    if(_time_count < 255)
    {
        analogWrite(LEDR, _time_count);
        analogWrite(LEDB, 255 - _time_count);
    }else if(_time_count < 255 * 2)
    {
        analogWrite(LEDG, _time_count - 255);
        analogWrite(LEDR, 255 * 2 - _time_count);
    }else if(_time_count < 255 * 3)
    {
        analogWrite(LEDB, _time_count - 255 * 2);
        analogWrite(LEDG, 255 * 3 - _time_count);
    }else
    {
        _time_count = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    _time_count += num;
}