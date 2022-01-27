#include <Servo.h>
#include "Tool.h"

#define SETUPMODE_1 1 //サーボの回転方向と位置の調整
#define SETUPMODE_2 2 //原点合わせ

#define SETUPMODE SETUPMODE_2

#if SETUPMODE == SETUPMODE_1
Servo servo[8];
#elif SETUPMODE == SETUPMODE_2
Servo fr_s, fr_e, fl_s, fl_e;
Servo hr_s, hr_e, hl_s, hl_e; 
#endif

PushCounter SW10, SW12, SW2;


void setup()
{
    Serial.begin(115200);

    #if SETUPMODE == SETUPMODE_1

    for (int i = 0; i < 8; i++)
    {
        if(i == 1) servo[i].attach(11);
        else servo[i].attach(i + 2);
    }

    #elif SETUPMODE == SETUPMODE_2

    fr_s.attach(2); fr_e.attach(11); fl_s.attach(4); fl_e.attach(5);
    hr_s.attach(6); hr_e.attach(7); hl_s.attach(8); hl_e.attach(9); 
    
    #endif

    pinMode(10, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(13, INPUT_PULLUP);

    SW10.setup(digitalRead(10));
    SW12.setup(digitalRead(12));
    SW2.setup(digitalRead(2));
}

void loop()
{
    SW10.update(digitalRead(10));
    SW12.update(digitalRead(12));
    SW2.update(digitalRead(2));

    #if SETUPMODE == SETUPMODE_1

    double deg1, cmd;
    int invert;

    switch (SW12.getCount() % 3)
    {
    case 0: deg1 = 0.0; break;
    case 1: deg1 = 90.0; break;
    case 2: deg1 = 180.0; break;
    
    default:
        break;
    }

    switch (SW2.getCount() % 2)
    {
    case 0:
        invert = 0;
        cmd = deg1;
        break;

    case 1:
        invert = 1;
        cmd = 180 - deg1;
        break;
    
    default:
        break;
    }

    servo[SW10.getCount() % 8].write(cmd);

    Serial.print(invert);
    Serial.print(" servo:");
    Serial.print(SW10.getCount() % 8);
    Serial.print(" cmd:");
    Serial.print(cmd);
    Serial.print(" deg:");
    Serial.println(deg1);

    #elif SETUPMODE == SETUPMODE_2

    static double deg1 = 0.0;
    switch (SW12.getCount() % 3)
    {
    case 0: deg1 = 0.0; break;
    case 1: deg1 = 90.0; break;
    case 2: deg1 = 180.0; break;
    
    default:
        break;
    }

    switch (SW10.getCount() % 8)
    {
    case 0: fr_s.write(180 - deg1); break;
    case 1: fr_e.write(deg1); break;
    case 2: fl_s.write(deg1); break;
    case 3: fl_e.write(180 - deg1); break;
    case 4: hr_s.write(180 - deg1); break;
    case 5: hr_e.write(deg1); break;
    case 6: hl_s.write(deg1); break;
    case 7: hl_e.write(180 - deg1); break;
    
    default:
        break;
    }

    Serial.print(" servo:");
    Serial.print(SW10.getCount() % 8);
    Serial.print(" deg:");
    Serial.println(deg1);

    #endif

    delay(10);
}

