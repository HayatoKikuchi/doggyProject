/* miniDog.ino として使用する */

#include <WiFiNINA.h>
#include "MBED_RPi_Pico_TimerInterrupt.h" // timer interrupt calss
#include "LegController.h"
#include "RouteCreater.h"

#define INT_TIME 0.01 // [s]
#define INT_TIME_MS 10 // [ms]

// Init RPI_PICO_Timer
MBED_RPI_PICO_Timer ITimer1(1);

LegController ForeLegR(2, 3); //ダミー

RouteCreater route(MODE_TRIANGLE,INT_TIME);

bool flag_10ms = false;
double velocity = 25.0; //[mm/s]
double x_fr, y_fr, index1, index2;

int time_count = 0;

void interrupt(uint alarm_num)
{
    // Always call this for MBED RP2040 before processing ISR
    TIMER_ISR_START(alarm_num);

    flag_10ms = true;
    time_count++;

    // Always call this for MBED RP2040 after processing ISR
    TIMER_ISR_END(alarm_num);
}

void setup()
{
    route.setParam(15.0, X1, MODE_TRIANGLE);
    route.setParam(-10.0, X2, MODE_TRIANGLE);
    route.setParam(7.5, X3, MODE_TRIANGLE);
    route.setParam(70.0, Y1, MODE_TRIANGLE);
    route.setParam(60.0, Y2, MODE_TRIANGLE);

    x_fr = ForeLegR.getXp();
    y_fr = ForeLegR.getYp();

    pinMode(LED_BUILTIN, OUTPUT);

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
        route.getPosition(&x_fr, &y_fr,&index1, &index2 ,velocity); //足先の位置生成
        ForeLegR.MoveLegPosition(x_fr, y_fr); //足先の位置制御

        double theta_fr_shoulder = ForeLegR.getThetaShoulder();
        double theta_fr_elbow = ForeLegR.getThetaElbow();
        double pointX = ForeLegR.getXp();
        double pointY = ForeLegR.getYp();

        //Serial.print(Degrees(theta_fr_shoulder));
        //Serial.print("\t");
        //Serial.print(Degrees(theta_fr_elbow));
        //Serial.println(Degrees((_PI - theta_fr_shoulder) + theta_fr_elbow));
        //Serial.print("  ");
        Serial.print(x_fr);
        Serial.print(",");
        Serial.print(pointX);
        Serial.print("  ");
        Serial.print(y_fr);
        Serial.print(",");
        Serial.println(pointY);

        LEDrgb(time_count * 2);

        flag_10ms = false;
    }
}

void LEDrgb(int _time_count)
{
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
}