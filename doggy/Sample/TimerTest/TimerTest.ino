#include "MBED_RPi_Pico_TimerInterrupt.h"

unsigned int outputPin1 = LED_BUILTIN;

// Init RPI_PICO_Timer
MBED_RPI_PICO_Timer ITimer1(1);
MBED_RPI_PICO_Timer ITimer2(2);

#define TIMER1_INTERVAL_MS    1000

int counts = 0;
bool flag = true;

// Never use Serial.print inside this mbed ISR. Will hang the system
void TimerHandler1(uint alarm_num)
{
  static bool toggle1 = false;

  ///////////////////////////////////////////////////////////
  // Always call this for MBED RP2040 before processing ISR
  TIMER_ISR_START(alarm_num);
  ///////////////////////////////////////////////////////////

  flag = true;
  digitalWrite(LED_BUILTIN, toggle1);
  toggle1 = !toggle1;

  counts  += 1;

  ////////////////////////////////////////////////////////////
  // Always call this for MBED RP2040 after processing ISR
  TIMER_ISR_END(alarm_num);
  ////////////////////////////////////////////////////////////
}

#define TIMER2_INTERVAL_MS    2000

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("\nStarting Argument_Simple on ")); Serial.println(BOARD_NAME);

  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1))
  {
    Serial.print(F("Starting ITimer1 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
}

void loop()
{
  if(flag)
  {
    Serial.println(counts);
    flag = false;
  }
}