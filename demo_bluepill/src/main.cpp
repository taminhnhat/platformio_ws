#include <Arduino.h>

HardwareTimer timer(TIM1);

bool ledOn = true;
uint32_t timer_count = 0;
void OnTimer1Interrupt()
{
  timer_count++;
  if (timer_count % 1000 == 0)
  {
    ledOn ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);
    ledOn = !ledOn;
    Serial.println(millis());
    Serial.println(timer.getCount());
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  timer.setPrescaleFactor(7200);
  timer.setOverflow(10);
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.refresh();
  timer.resume();
  Serial.println(timer.getTimerClkFreq());
}

void loop()
{
}