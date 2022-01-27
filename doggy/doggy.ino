#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include "MBED_RPi_Pico_TimerInterrupt.h"
#include "LegController.h"
#include "RouteCreater.h"
#include "Tool.h"

#define INT_TIME 0.01 // [s]
#define INT_TIME_MS 10 // [ms]

/* the macro of each array number */
#define FR 0
#define FL 1
#define HR 2
#define HL 3
#define XAXIS 0
#define YAXIS 1

MBED_RPI_PICO_Timer ITimer1(1); //timer interrupt

/* class for controlling the motin of each leg */
LegController ForeFootR(2, true, 11, false);
LegController ForeFootL(4, false, 5, true);
LegController HindLegR(6, true, 7, false);
LegController HindLegL(8, false, 9, true);

/* class for creating the coordinate of each leg's toes */
RouteCreater routeFR(MODE_ELLIPSE);
RouteCreater routeFL(MODE_ELLIPSE);
RouteCreater routeHR(MODE_ELLIPSE);
RouteCreater routeHL(MODE_ELLIPSE);

SecondOrderLag SOL[4][2];
PushCounter SW12, SW2;

/* are used in the interrupt func and the loop func */
bool flag_10ms = false;
double timeFR = 0.0, timeFL = 0.0, timeHR = 0.0, timeHL = 0.0;
int time_count = 0;

/* are used in the setup func or the loop func */
double first_x = 10.0, first_y = 65.0;
double velocity = 25.0; //[mm/s]
double x_axis[4], y_axis[4];
double _x_axis[4], _y_axis[4];
int control_phase = 1;
int control_mode = 0;


/* timer interrupt function */
void interrupt(uint alarm_num)
{
    // Always call this for MBED RP2040 before processing ISR
    TIMER_ISR_START(alarm_num);

    /* update each value of variable depends on control period */
    flag_10ms = true;
    if((control_mode == 1) && (control_phase == 2))
    {
        timeFR += INT_TIME;
        timeFL += INT_TIME;
        timeHR += INT_TIME;
        timeHL += INT_TIME;
    }
    else
    {
        timeHR = 0.0; // 基準
        timeHL = routeHL.getPeriod() / 2.0; //基準から半周期ずらす
        timeFR = routeFR.getPeriod() * 0.875; // 半周期の1/4遅らせる
        timeFL = routeFL.getPeriod() * 0.375; //半周期の1/4遅らせる
    }

    time_count++;

    // Always call this for MBED RP2040 after processing ISR
    TIMER_ISR_END(alarm_num);
}

void setup()
{
    delay(1000);
    pinMode(LED_BUILTIN, OUTPUT); // setup of the user's LED
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(12, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    /* set the coordinate of the route of leg's toe */
    routeFR.setParam_ellipse(35.0, 5.0, 80.0, 50.0); routeFR.init(velocity);
    routeFL.setParam_ellipse(35.0, 5.0, 80.0, 50.0); routeFL.init(velocity);
    routeHR.setParam_ellipse(35.0, 5.0, 80.0, 50.0); routeHR.init(velocity);
    routeHL.setParam_ellipse(35.0, 5.0, 80.0, 50.0); routeHL.init(velocity);

    timeFL = routeFR.getPeriod() / 2.0; // substitute the harf cycle

    /* initialize each coordiate of leg's toes */
    ForeFootR.init();
    ForeFootL.init();
    HindLegR.init();
    HindLegL.init();

    x_axis[FR] = ForeFootR.getXp(); y_axis[FR] = ForeFootR.getYp();
    x_axis[FL] = ForeFootL.getXp(); y_axis[FL] = ForeFootL.getYp();
    x_axis[HR] = HindLegR.getXp(); y_axis[HR] = HindLegR.getYp();
    x_axis[HL] = HindLegL.getXp(); y_axis[HL] = HindLegL.getYp();

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if(j == XAXIS)  SOL[i][j].setup(6.0, 1.0, ForeFootR.getXp(), INT_TIME);
            else if(j == YAXIS) SOL[i][j].setup(6.0, 1.0, ForeFootR.getYp(), INT_TIME);
        }
    }

    bool ready_to_start = false;
    while (!ready_to_start)
    {
        static int stop = 0;
        bool flag_start = true;
        if(stop > 199)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    if(j == XAXIS) 
                    {
                        x_axis[i] = SOL[i][j].getCmd(first_x);
                        flag_start *= Abs( x_axis[i] - first_x) < 1.0;
                    }
                    else if(j == YAXIS)
                    { 
                        y_axis[i] = SOL[i][j].getCmd(first_y);
                        flag_start *= Abs(y_axis[i] - first_y) < 1.0;
                    }
                }
            }

            ForeFootR.MoveLegPosition(x_axis[FR], y_axis[FR]);
            ForeFootL.MoveLegPosition(x_axis[FL], y_axis[FL]);
            HindLegR.MoveLegPosition(x_axis[HR], y_axis[HR]);
            HindLegL.MoveLegPosition(x_axis[HL], y_axis[HL]);

            if(flag_start) ready_to_start = true;
        }
        else stop++;

        static int led_counts = 0;
        if(led_counts++ > 49)
        {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            led_counts = 0;
        }

        delay(INT_TIME_MS);
    }

    digitalWrite(LED_BUILTIN, LOW);

    SW12.setup(digitalRead(12));
    SW2.setup(digitalRead(2));

    Serial.begin(115200); // begin to communicate with pc's serial monitor
    
    /* start timer interrupt */
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
        SW12.update(digitalRead(12));
        SW2.update(digitalRead(2));
        CreateVelocity();
        routeFR.updateParam(velocity);
        routeFL.updateParam(velocity);
        routeHR.updateParam(velocity);
        routeHL.updateParam(velocity);

        control_mode = SW12.getCount() % 2;
        static bool phase_changed = true;

        switch (control_mode)
        {
        case 0: // 停止モード
            {
                static double stop_time = 0.0;

                if(SW12.CountChanged())
                {
                    stop_time = 0.0;
                    control_phase = 1;
                }

                switch (control_phase)
                {
                case 1:
                    {
                        stop_time += INT_TIME;
                        if(stop_time > 1.0) control_phase = 2;
                        phase_changed = true;
                    }
                    break; //停止モード，フェーズ1のbreak
                
                case 2:
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            for (int j = 0; j < 2; j++)
                            {
                                if(j == XAXIS)
                                {
                                    if(phase_changed) SOL[i][j].init(x_axis[i]);
                                    x_axis[i] = SOL[i][j].getCmd(first_x);
                                } 
                                else if(j == YAXIS)
                                {
                                    if(phase_changed) SOL[i][j].init(y_axis[i]);
                                    y_axis[i] = SOL[i][j].getCmd(first_y);
                                }
                            }
                        }
                        phase_changed = false;
                    }
                    break; //停止モード，フェーズ2のbreak
                
                default: //停止モード，フェーズ処理の終端
                    break;
                }
                
                static int led_counts = 0;
                if(led_counts++ > 49)
                {
                    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                    led_counts = 0;
                }
            }
            break; //停止モードの終端
        
        case 1: //歩行モード
            {
                if(SW12.CountChanged())
                {
                    control_phase = 1;
                    phase_changed = true;
                }

                switch (control_phase)
                {
                case 1:
                    {
                        if(phase_changed)
                        {
                            routeFR.getPosition(_x_axis + FR, _y_axis + FR, &timeFR);
                            routeFL.getPosition(_x_axis + FL, _y_axis + FL, &timeFL);
                            routeHR.getPosition(_x_axis + HR, _y_axis + HR, &timeHR);
                            routeHL.getPosition(_x_axis + HL, _y_axis + HL, &timeHL);
                        }
                        
                        bool flag_converge = true;
                        for (int i = 0; i < 4; i++)
                        {
                            for (int j = 0; j < 2; j++)
                            {
                                if(j == XAXIS)
                                {
                                    if(phase_changed) SOL[i][j].init(first_x);
                                    x_axis[i] = SOL[i][j].getCmd(_x_axis[i]);
                                    flag_converge *= Abs(x_axis[i] - _x_axis[i]) < 1.0;
                                } 
                                else if(j == YAXIS)
                                {
                                    if(phase_changed) SOL[i][j].init(first_y);
                                    y_axis[i] = SOL[i][j].getCmd(_y_axis[i]);
                                    flag_converge *= Abs(y_axis[i] - _y_axis[i]) < 1.0;
                                }
                            }
                        }
                        phase_changed = false;

                        if(flag_converge) control_phase = 2;
                    }
                    break; //歩行モード，フェーズ1のbreak
                
                case 2:
                    {
                        /* get positions of each leg's toes */
                        routeFR.getPosition(&x_axis[FR], &y_axis[FR], &timeFR);
                        routeHL.getPosition(&x_axis[HL], &y_axis[HL], &timeHL);
                        routeFL.getPosition(&x_axis[FL], &y_axis[FL], &timeFL);
                        routeHR.getPosition(&x_axis[HR], &y_axis[HR], &timeHR);
                    }
                    break; //歩行モード，フェーズ2のbreak
                
                default: //歩行モード，フェーズ処理の終端
                    break;
                }

                static int led_counts = 0;
                if(led_counts++ > 24)
                {
                    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                    led_counts = 0;
                }
            }
            break;//歩行モードのbreak
        
            default: //コントロールモードの終端
                break;
        }
    
        /* move each legs (move each servo motors)*/
        ForeFootR.MoveLegPosition(x_axis[FR], y_axis[FR]);
        ForeFootL.MoveLegPosition(x_axis[FL], y_axis[FL]);
        HindLegR.MoveLegPosition(x_axis[HR], y_axis[HR]);
        HindLegL.MoveLegPosition(x_axis[HL], y_axis[HL]);

        /* get cmds and check them on the serial monitor*/
        double theta_fr_shoulder = ForeFootR.getThetaShoulder();
        double theta_fr_elbow = ForeFootR.getThetaElbow();
        double pointX_fr = ForeFootR.getXp();
        double pointY_fr = ForeFootR.getYp();
        // Serial.print(theta_fr_shoulder);
        // Serial.print("\t");
        // Serial.print(theta_fr_elbow);
        // Serial.print("\t");
        // Serial.print(pointX_fr);
        // Serial.print("\t");
        // Serial.print(pointY_fr);
        // Serial.print("\t");

        double theta_fl_shoulder = ForeFootL.getThetaShoulder();
        double theta_fl_elbow = ForeFootL.getThetaElbow();
        double pointX_fl = ForeFootL.getXp();
        double pointY_fl = ForeFootL.getYp();
        // Serial.print(theta_fl_shoulder);
        // Serial.print("\t");
        // Serial.print(theta_fl_elbow);
        // Serial.print("\t");
        // Serial.print(pointX_fl);
        // Serial.print("\t");
        // Serial.print(pointY_fl);
        // Serial.print("\t");

        double theta_hr_shoulder = HindLegR.getThetaShoulder();
        double theta_hr_elbow = HindLegR.getThetaElbow();
        double pointX_hr = HindLegR.getXp();
        double pointY_hr = HindLegR.getYp();
        // Serial.print(theta_hr_shoulder);
        // Serial.print("\t");
        // Serial.print(theta_hr_elbow);
        // Serial.print("\t");
        // Serial.print(pointX_hr);
        // Serial.print("\t");
        // Serial.print(pointY_hr);
        // Serial.print("\t");

        double theta_hl_shoulder = HindLegL.getThetaShoulder();
        double theta_hl_elbow = HindLegL.getThetaElbow();
        double pointX_hl = HindLegL.getXp();
        double pointY_hl = HindLegL.getYp();
        // Serial.print(theta_hl_shoulder);
        // Serial.print("\t");
        // Serial.print(theta_hl_elbow);
        // Serial.print("\t");
        // Serial.print(pointX_hl);
        // Serial.print("\t");
        // Serial.print(pointY_hl);
        // Serial.print("\t");

        // Serial.print(control_mode);
        // Serial.print("\t");
        // Serial.print(control_phase);
        // Serial.print("\t");
        // Serial.print(velocity);
        // Serial.print("\t");
        
        // Serial.print(timeFR);
        // Serial.print("\t");
        // Serial.print(timeFL);
        // Serial.print("\t");
        // Serial.print(timeHR);
        // Serial.print("\t");
        // Serial.print(timeHL);
        // Serial.print("\n");
        
        LEDrgb(2); // RGB LED

        flag_10ms = false;
    }

    delay(1);
}

void CreateVelocity()
{
    int analogVal = analogRead(A7);
    velocity = 25.0 + map((double)analogVal, 0.0, 1023.0, 0.0, 125.0);
}

/* function for RGB LED */
void LEDrgb(int num)
{
    static int _time_count = 0;
    _time_count += num;
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
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}