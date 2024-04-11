#include <Arduino.h>
#include <SPI.h>

#define W25Q_SS_PIN PA4
#define W25Q_CLOCK_PIN PA5
#define W25Q_MISO_PIN PA6
#define W25Q_MOSI_PIN PA7

#define MAX_SPI_FREQUENCY 1000000

HardwareTimer timer(TIM1);

bool ledOn = true;
uint32_t timer_count = 0;
uint8_t temp_num = 1;
uint32_t add = 256;
uint8_t dat = 35;

void readManufacturer();
void readUniqueID();
void readJedecID();
void readStatus1();
uint8_t readData(uint32_t);
void writeData(uint32_t, uint8_t);
void eraseChip();
void OnTimer1Interrupt()
{
  timer_count++;
  if (timer_count % 3000 == 0)
  {
    ledOn ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);
    ledOn = !ledOn;

    // digitalWrite(W25Q_SS_PIN, HIGH);
    // delay(500);
    // digitalWrite(W25Q_SS_PIN, LOW);

    // readManufacturer();
    // readUniqueID();
    // readJedecID();
    Serial.print("Read: ");
    Serial.println(readData(0));
    Serial.println(readData(256));
    Serial.println(readData(512));
    // Serial.println(readData(add + 1));
    // Serial.println(readData(add - 1));
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

  pinMode(W25Q_SS_PIN, OUTPUT);
  pinMode(W25Q_CLOCK_PIN, OUTPUT);
  pinMode(W25Q_MOSI_PIN, OUTPUT);
  // pinMode(W25Q_MISO_PIN, INPUT);
  digitalWrite(W25Q_SS_PIN, HIGH);

  // SPI.setBitOrder(MSBFIRST);
  // SPI.setDataMode(SPI_MODE0);
  // SPI.setSCLK(W25Q_CLOCK_PIN);
  // SPI.setMISO(W25Q_MISO_PIN);
  // SPI.setMOSI(W25Q_MOSI_PIN);
  // SPI.setSSEL(W25Q_SS_PIN);
  // SPI.setDataMode(SPI_MODE0);
  // SPI.begin(W25Q_SS_PIN);
  Serial.printf("ss pin: %d\r\n", W25Q_SS_PIN);

  // readStatus1();

  timer.refresh();
  timer.resume();

  eraseChip();
  delay(500);

  Serial.printf("==================================> writing %d to register %d\r\n", dat, add);
  // Serial.print("Read: ");
  // Serial.println(readData(add));
  writeData(add, dat);
}

void loop()
{
  // delay(1000);
}

void readManufacturer()
{
  uint8_t readStr[6];
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  readStr[0] = SPI.transfer(W25Q_SS_PIN, 0x90, SPI_CONTINUE); // Transfer the 8-bit value of data to shift register, remembering that the least significant bit goes first
  readStr[1] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[2] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[3] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[4] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[5] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST);     //
  SPI.endTransaction(W25Q_SS_PIN);
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
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  readStr[0] = SPI.transfer(W25Q_SS_PIN, 0x4b, SPI_CONTINUE);  //
  readStr[1] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[2] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[3] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[4] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[5] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[6] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[7] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[8] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[9] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);  //
  readStr[10] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[11] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[12] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  String strout = "Serial Numer: ";
  for (int i = 5; i <= 11; i++)
  {
    strout += readStr[i];
    strout += "-";
  }
  strout += readStr[12];
  Serial.println(strout);
}

void readJedecID()
{
  uint8_t readStr[4];
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  readStr[0] = SPI.transfer(W25Q_SS_PIN, 0x9f, SPI_CONTINUE); //
  readStr[1] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[2] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE); //
  readStr[3] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST);     //
  SPI.endTransaction(W25Q_SS_PIN);
  Serial.printf("Manufacturer ID: %d   Memory Type: %d   Capacity: %d\r\n", readStr[1], readStr[2], readStr[3]);
}

void readStatus1()
{
  uint8_t data[3];
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  data[0] = SPI.transfer(W25Q_SS_PIN, 0x05, SPI_CONTINUE);
  data[1] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);
  data[2] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  Serial.print("Status Register 1: ");
  Serial.print(data[1]);
  Serial.print("-");
  Serial.println(data[2]);

  // SPI.beginTransaction(SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  // data[0] = SPI.transfer(SS_PIN, 0x35, SPI_CONTINUE);
  // data[1] = SPI.transfer(SS_PIN, 0x00, SPI_CONTINUE);
  // data[2] = SPI.transfer(SS_PIN, 0x00, SPI_LAST);
  // SPI.endTransaction(SS_PIN);
  // Serial.print("Status Register 2: ");
  // Serial.print(data[1]);
  // Serial.print("-");
  // Serial.println(data[2]);
}

uint8_t readData(uint32_t address)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  uint8_t data[5];
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  data[0] = SPI.transfer(W25Q_SS_PIN, 0x03, SPI_CONTINUE);
  data[1] = SPI.transfer(W25Q_SS_PIN, msb_address, SPI_CONTINUE);
  data[2] = SPI.transfer(W25Q_SS_PIN, mid_address, SPI_CONTINUE);
  data[3] = SPI.transfer(W25Q_SS_PIN, lsb_address, SPI_CONTINUE);
  for (int i = 1; i <= 255; i++)
    Serial.printf("%d ", SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE));
  data[4] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  Serial.println(data[4]);
  return data[4];
}

void writeData(uint32_t address, uint8_t data)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  // Serial.printf("address: %d - %d %d %d\r\n", address, msb_address, mid_address, lsb_address);
  // enable write
  Serial.println("1. Enable Write");
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x06, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // delay(100);
  readStatus1();
  // erase
  // Serial.println("2. Erase Block");
  // SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  // SPI.transfer(W25Q_SS_PIN, 0xd8, SPI_CONTINUE);
  // SPI.transfer(W25Q_SS_PIN, msb_address, SPI_CONTINUE);
  // SPI.transfer(W25Q_SS_PIN, mid_address, SPI_CONTINUE);
  // SPI.transfer(W25Q_SS_PIN, lsb_address, SPI_LAST);
  // SPI.endTransaction(W25Q_SS_PIN);
  // delay(100);
  // readStatus1();
  // write process
  Serial.printf("2. Writing to %d\r\n", address);
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x02, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, msb_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, mid_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, lsb_address, SPI_CONTINUE);
  // for (int i = 1; i <= 255; i++)
  //   SPI.transfer(W25Q_SS_PIN, data, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, data, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // delay(100);
  // readStatus1();

  // disable write
  Serial.println("3. Disable Write");
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x04, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // delay(100);
  // readStatus1();
}

void eraseChip()
{
  // enable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x06, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // erase
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x60, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // disable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x04, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
}
