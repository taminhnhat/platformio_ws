#include <Arduino.h>
#include <SPI.h>

#define SS_PIN PA4
#define CLOCK_PIN PA5
#define MISO_PIN PA6
#define MOSI_PIN PA7

HardwareTimer timer(TIM1);

bool ledOn = true;
uint32_t timer_count = 0;
uint8_t temp_num = 1;
void readManufacturer();
void readUniqueID();
void readStatus();
uint8_t readData(uint32_t);
void writeData(uint32_t, uint8_t);
void eraseData(uint32_t);
void OnTimer1Interrupt()
{
  timer_count++;
  if (timer_count % 2000 == 0)
  {
    ledOn ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);
    ledOn = !ledOn;
    // readManufacturer();
    // readUniqueID();

    uint32_t add = 100;
    uint8_t dat = 64;
    Serial.printf("writing %d to register %d\r\n", dat, add);
    writeData(add, dat);
    Serial.print("read: ");
    Serial.println(readData(add));
    Serial.println(readData(add + 1));
    Serial.println(readData(add - 1));
    // temp_num += 1;
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  timer.setPrescaleFactor(9600);
  timer.setOverflow(10);
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.pause();
  Serial.print("Timer Frequency: ");
  Serial.println(timer.getTimerClkFreq());

  pinMode(SS_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(200);
  SPI.setDataMode(3);
  SPI.setSCLK(CLOCK_PIN);
  SPI.setMISO(MISO_PIN);
  SPI.setMOSI(MOSI_PIN);
  SPI.setSSEL(SS_PIN);
  SPI.begin(SS_PIN);

  readStatus();

  timer.refresh();
  timer.resume();

  delay(500);
}

void loop()
{
  // delay(1000);
}

void readManufacturer()
{
  uint8_t readStr[6];
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITRECEIVE));
  readStr[0] = SPI.transfer(SS_PIN, 0x90, SPI_CONTINUE); // Transfer the 8-bit value of data to shift register, remembering that the least significant bit goes first
  readStr[1] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[2] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[3] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[4] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[5] = SPI.transfer(SS_PIN, 0x00, SPI_LAST);     //
  SPI.endTransaction();
  Serial.print("Manufactorer ID: ");
  Serial.print(readStr[4]);
  Serial.print("   Device ID: ");
  Serial.println(readStr[5]);
}

/**
 * @brief read serial number
 *
 */
void readUniqueID()
{
  uint8_t readStr[14];
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITRECEIVE));
  readStr[0] = SPI.transfer(SS_PIN, 0x4b, SPI_CONTINUE);  //
  readStr[1] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[2] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[3] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[4] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[5] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[6] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[7] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[8] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[9] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[10] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[11] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[12] = SPI.transfer(SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction();
  String strout = "Serial Numer: ";
  for (int i = 5; i <= 11; i++)
  {
    strout += readStr[i];
    strout += "-";
  }
  strout += readStr[12];
  Serial.println(strout);
}

void readStatus()
{
  uint8_t data[3];
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITRECEIVE));
  data[0] = SPI.transfer(SS_PIN, 0x05, SPI_CONTINUE);
  data[1] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);
  data[2] = SPI.transfer(SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction();
  Serial.print("Status Register 1: ");
  Serial.print(data[1]);
  Serial.print("-");
  Serial.println(data[2]);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITRECEIVE));
  data[0] = SPI.transfer(SS_PIN, 0x35, SPI_CONTINUE);
  data[1] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);
  data[2] = SPI.transfer(SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction();
  Serial.print("Status Register 2: ");
  Serial.print(data[1]);
  Serial.print("-");
  Serial.println(data[2]);
}

uint8_t readData(uint32_t address)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  uint8_t data[5];
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITRECEIVE));
  data[0] = SPI.transfer(SS_PIN, 0x03, SPI_CONTINUE);
  data[3] = SPI.transfer(SS_PIN, lsb_address, SPI_CONTINUE);
  data[2] = SPI.transfer(SS_PIN, mid_address, SPI_CONTINUE);
  data[1] = SPI.transfer(SS_PIN, msb_address, SPI_CONTINUE);
  data[4] = SPI.transfer(SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction();
  return data[4];
}

void writeData(uint32_t address, uint8_t data)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  Serial.printf("address: %d - %d %d %d\r\n", address, msb_address, mid_address, lsb_address);
  // enable write
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITONLY));
  SPI.transfer(SS_PIN, 0x06, SPI_LAST);
  SPI.endTransaction();
  readStatus();
  // // erase
  // Serial.println("erasing register");
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITONLY));
  // SPI.transfer(SS_PIN, 0x20, SPI_CONTINUE);
  // SPI.transfer(SS_PIN, lsb_address, SPI_CONTINUE);
  // SPI.transfer(SS_PIN, mid_address, SPI_CONTINUE);
  // SPI.transfer(SS_PIN, msb_address, SPI_LAST);
  // SPI.endTransaction();
  // readStatus();
  // write process
  Serial.println("writing to register");
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITONLY));
  SPI.transfer(SS_PIN, 0x02, SPI_CONTINUE);
  SPI.transfer(SS_PIN, lsb_address, SPI_CONTINUE);
  SPI.transfer(SS_PIN, mid_address, SPI_CONTINUE);
  SPI.transfer(SS_PIN, msb_address, SPI_CONTINUE);
  SPI.transfer(SS_PIN, data, SPI_LAST);
  SPI.endTransaction();

  delay(100);
  // disable write
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITONLY));
  SPI.transfer(SS_PIN, 0x04, SPI_LAST);
  SPI.endTransaction();
  readStatus();
}

void eraseData(uint32_t address)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3, SPI_TRANSMITONLY));
  SPI.transfer(SS_PIN, 0x06, SPI_LAST);
  SPI.endTransaction();
}
