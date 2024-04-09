#include <Arduino.h>

HardwareTimer timer(TIM1);

bool ledOn = true;
uint32_t timer_count = 0;
void OnTimer1Interrupt()
{
  timer_count++;
  if (timer_count % 1000 == 0)
  {
    ledOn ? digitalWrite(PB2, HIGH) : digitalWrite(PB2, LOW);
    ledOn = !ledOn;
    Serial.println(millis());
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(PB2, OUTPUT);
  timer.setPrescaleFactor(7200);
  timer.setOverflow(10);
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.refresh();
  timer.resume();
  Serial.println(timer.getTimerClkFreq());

  analogWrite(A1, 50);
  analogWrite(A2, 100);
  analogWrite(A3, 150);
  analogWrite(A6, 200);
  analogWrite(A7, 100);
}

void loop()
{
}