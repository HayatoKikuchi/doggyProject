#include <WiFiNINA.h>
#include <Servo.h>
#include <Arduino_LSM6DSOX.h>
#include <math.h>
//#include <FlexiTimer2.h>

Servo servo;

void interrupt()
{
    static int count = 0;
    static int count_flag = 0;
    count += 2;
    count_flag++;

    if(count < 255)
    {
        analogWrite(LEDR, count);
        analogWrite(LEDB, 255 - count);
    }else if(count < 255 * 2)
    {
        analogWrite(LEDG, count - 255);
        analogWrite(LEDR, 255 * 2 - count);
    }else if(count < 255 * 3)
    {
        analogWrite(LEDB, count - 255 * 2);
        analogWrite(LEDG, 255 * 3 - count);
    }else
    {
        count = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);

    servo.attach(2);
    servo.write(100);

    if(!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while(1);
    }


    Serial.begin(115200);
    // FlexiTimer2::set(10, interrupt);
    // FlexiTimer2::start();
}

void loop()
{
    static float gyroX, gyroY, gyroZ;
    static double omegaX = 0.0, omegaY = 0.0, omegaZ = 0.0;
    static double loop_num = 1.0;
    if(IMU.gyroscopeAvailable()) 
    {
        double rate = IMU.accelerationSampleRate();
        IMU.readGyroscope(gyroX, gyroY, gyroZ);

        omegaX += gyroX * (1.0/rate * loop_num);
        omegaY += gyroY * (1.0/rate * loop_num);
        omegaZ += gyroZ * (1.0/rate * loop_num);

        loop_num = 1.0;
    }
    else
    {
        loop_num += 1.0;
    }
    //if (IMU.accelerationAvailable()) IMU.readAcceleration(gyroX, gyroY, gyroZ);

    Serial.print("X:");
    Serial.print(log(gyroX));
    Serial.print("  Y:");
    Serial.print(log(gyroY));
    Serial.print("  Z:");
    Serial.println(log(gyroZ));

    interrupt();
    delay(100);
}