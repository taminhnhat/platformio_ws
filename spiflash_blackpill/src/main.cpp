#include <Arduino.h>
#include <SPI.h>

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
    // Serial.println(millis());
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  timer.setPrescaleFactor(8400);
  timer.setOverflow(10);
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.refresh();
  timer.resume();
  Serial.println(timer.getTimerClkFreq());

  SPI.begin(PA4);
  SPI.setBitOrder(MSBFIRST);

  pinMode(PA4, OUTPUT);
  pinMode(PA5, OUTPUT);
  pinMode(PA7, OUTPUT);
}

void loop()
{
  uint8_t readStr[5];
  digitalWrite(PA4, LOW);                             // Write our Slave select low to enable the SHift register to begin listening for data
  readStr[0] = SPI.transfer(PA4, 0x90, SPI_CONTINUE); // Transfer the 8-bit value of data to shift register, remembering that the least significant bit goes first
  readStr[1] = SPI.transfer(PA4, 0x00, SPI_CONTINUE); //
  readStr[2] = SPI.transfer(PA4, 0x00, SPI_CONTINUE); //
  readStr[3] = SPI.transfer(PA4, 0x00, SPI_CONTINUE); //
  readStr[4] = SPI.transfer(PA4, 0x00, SPI_LAST);     //
  digitalWrite(PA4, HIGH);                            // Once the transfer is complete, set the latch back to high to stop the shift register listening for data
  Serial.print(readStr[0]);
  Serial.print("-");
  Serial.print(readStr[1]);
  Serial.print("-");
  Serial.print(readStr[2]);
  Serial.print("-");
  Serial.print(readStr[3]);
  Serial.print("-");
  Serial.println(readStr[4]);
  delay(1000);
}