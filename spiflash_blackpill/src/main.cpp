#include <Arduino.h>
#include <SPI.h>

#define W25Q_SS_PIN PA4
#define W25Q_CLOCK_PIN PA5
#define W25Q_MISO_PIN PA6
#define W25Q_MOSI_PIN PA7

#define MAX_SPI_FREQUENCY 100000000

HardwareTimer timer(TIM1);

bool ledOn = true;
uint32_t timer_count = 0;
uint8_t temp_num = 1;
uint32_t add = 4096;
uint8_t dat = 71;
uint8_t myData[4] = {192, 168, 1, 42};

void readManufacturer();
void readUniqueID();
void readJedecID();
void readStatus1();
uint8_t ready();
void readData(uint32_t, uint8_t);
void readPage(uint32_t);
void writeData(uint32_t, uint8_t);
void writeData(uint32_t, uint8_t *, uint8_t);
void eraseChip();
void eraseSector(uint32_t);
void OnTimer1Interrupt()
{
  timer_count++;
  if (timer_count % 5000 == 0)
  {
    ledOn ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);
    ledOn = !ledOn;

    while (ready() != 0)
    {
      delay(10);
    }
    // readPage(0x00);
    readData(0x00, 10);
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

  // readStatus1();

  timer.refresh();
  timer.resume();

  uint32_t start_t = millis();
  // eraseChip();
  eraseSector(0);
  Serial.printf("Erasing complete in %d ms\r\n", millis() - start_t);

  writeData(0x00, myData, 4);
  readPage(0x00);
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

uint8_t ready()
{
  uint8_t data[3];
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  data[0] = SPI.transfer(W25Q_SS_PIN, 0x05, SPI_CONTINUE);
  data[1] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE);
  data[2] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  return data[2] & 0b11111110;
}

/**
 * @brief read data from register
 *
 * @param address register address
 * @param data_lenght from 1 to 255 bytes
 */
void readData(uint32_t address, uint8_t data_lenght = 1)
{
  if (data_lenght == 0)
    return;
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  uint8_t data[5];
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  data[0] = SPI.transfer(W25Q_SS_PIN, 0x03, SPI_CONTINUE);
  data[1] = SPI.transfer(W25Q_SS_PIN, msb_address, SPI_CONTINUE);
  data[2] = SPI.transfer(W25Q_SS_PIN, mid_address, SPI_CONTINUE);
  data[3] = SPI.transfer(W25Q_SS_PIN, lsb_address, SPI_CONTINUE);
  for (int i = 0; i < data_lenght; i++)
    Serial.printf("%d ", SPI.transfer(W25Q_SS_PIN, 0x00, SPI_CONTINUE));
  data[4] = SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
}

/**
 * @brief read a page (256 bytes) from register
 *
 * @param address register address of first byte
 */
void readPage(uint32_t address)
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
  Serial.println(SPI.transfer(W25Q_SS_PIN, 0x00, SPI_LAST));
}

/**
 * @brief Write 1 byte to register (have to erase register first)
 *
 * @param address register address
 * @param data from 0 to
 */
void writeData(uint32_t address, uint8_t data)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  // enable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x06, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // write process
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x02, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, msb_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, mid_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, lsb_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, data, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // disable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x04, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
}

void writeData(uint32_t address, uint8_t *data, uint8_t data_size)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  // enable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x06, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // write process
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x02, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, msb_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, mid_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, lsb_address, SPI_CONTINUE);
  for (int i = 0; i < data_size - 1; i++)
    SPI.transfer(W25Q_SS_PIN, data[i], SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, data[data_size - 1], SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // disable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x04, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
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
  // wait
  while (ready() != 0)
  {
    delay(5);
  }
}

void eraseSector(uint32_t address)
{
  uint8_t msb_address = (address >> 16) & 0xff;
  uint8_t mid_address = (address >> 8) & 0xff;
  uint8_t lsb_address = address & 0xff;
  // enable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x06, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // erase
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x20, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, msb_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, mid_address, SPI_CONTINUE);
  SPI.transfer(W25Q_SS_PIN, lsb_address, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // disable write
  SPI.beginTransaction(W25Q_SS_PIN, SPISettings(MAX_SPI_FREQUENCY, MSBFIRST, SPI_MODE0, SPI_TRANSMITRECEIVE));
  SPI.transfer(W25Q_SS_PIN, 0x04, SPI_LAST);
  SPI.endTransaction(W25Q_SS_PIN);
  // wait
  while (ready() != 0)
  {
    delay(5);
  }
}
