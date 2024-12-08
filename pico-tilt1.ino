// Arduino for Pi Pico: read out AD7747 sensor
// Uses Philhower Pico board package v4.3.1
// and modified parts of https://github.com/jankop2/Arduino-AD7747
// 8-Dec-2024 J.Beale
// I2C address 0x48 : AD7747 24-bit capacitance sensor chip

#define VERSION "Version 0.11 2024-12-08 jpb"
#include <Wire.h>

const byte AD7747_ADDRESS = 0x48;  // address of device
const byte SDA0 = 20;  // I2C SDA on Pico GPIO 20 (physical board pin 26)
const byte SCL0 = 21;  // I2C SCL on Pico GPIO 21 (physical board pin 27)
const byte RDY0 = 16;  // chip /RDY signal, GPIO16 (board pin 21)
const byte PICO_LED = 25;
// ----------------------------------------------------------------------------------
const uint8_t AD774X_ADDRESS = 0x48;// AD774X I2C address

//*********************** Settings! ********************************
// AD7747 default registers definition for the differential input configuration
// +- 8.192pF  or for the single-ended input configuration 0 - 8.192pF.
// This setting is compatible with automatic offset using the OO command.
// Registers definitions are for AD7747 only! You must use yours own
// registers settings for AD7745 or AD7746 here!
const bool    AD7747         = true;       // Set your IC type, true = AD7747, false = AD7745/46
const uint8_t DATA_CAP_SETUP = B10100000;  // 7  0xA0 CAPEN+0+CAPDIF+00000 according to the datasheet AD7747
const uint8_t DATA_VT_SETUP  = B10000001;  // 8  0x81 VTEN+000000+VTCHOP for internal temperature
const uint8_t DATA_EXC_SETUP = B00001110;  // 9  0x0E 0000+EXDAC+EXCEN+EXCLVL1+0 according to the datasheet AD7747
const uint8_t DATA_CFG       = B10100000;  // 10 0xA0 VTFS1+0+CAPFS2+00000 = Idle mode, conversion time VT-62.1ms; CAP-124ms
const uint8_t DATA_CAPDACA   = B00000000;  // 11 0x00 CAPDACA OFF
const uint8_t DATA_CAPDACB   = B00000000;  // 12 0x00 CAPDACB OFF
const uint8_t DATA_CAP_OFFH  = B10000000;  // 13 0x80 OFFSET 0x8000 - the middle of the interval (the full range is is approximately +- 1 pF)
const uint8_t DATA_CAP_OFFL  = B00000000;  // 14 0x00     "                 "
//******************** Settings end  ********************************

// AD774X Register address Definition
const uint8_t ADR_STATUS     =  0;  // Read Only
const uint8_t ADR_CAP_DATAH  =  1;  // Read Only
const uint8_t ADR_CAP_DATAM  =  2;  // Read Only
const uint8_t ADR_CAP_DATAL  =  3;  // Read Only
const uint8_t ADR_VT_DATAH   =  4;  // Read Only
const uint8_t ADR_VT_DATAM   =  5;  // Read Only
const uint8_t ADR_VT_DATAL   =  6;  // Read Only
const uint8_t ADR_CAP_SETUP  =  7;  // CAP SETUP REGISTER
const uint8_t ADR_VT_SETUP   =  8;  // VT SETUP REGISTER
const uint8_t ADR_EXC_SETUP  =  9;  // EXC SETUP REGISTER
const uint8_t ADR_CFG        = 10;  // CONFIGURATION REGISTER
const uint8_t ADR_CAPDACA    = 11;  // CAP DAC A REGISTER
const uint8_t ADR_CAPDACB    = 12;  // CAP DAC B REGISTER
const uint8_t ADR_CAP_OFFH   = 13;  // CAP OFFSET CALIBRATION REGISTER HIGH
const uint8_t ADR_CAP_OFFL   = 14;  // CAP OFFSET CALIBRATION REGISTER LOW
const uint8_t ADR_CAP_GAINH  = 15;  // factory calibration
const uint8_t ADR_CAP_GAINL  = 16;  // factory calibration
const uint8_t ADR_VOLT_GAINH = 17;  // factory calibration
const uint8_t ADR_VOLT_GAINL = 18;  // factory calibration
//-------------------------------------------------------------------
const uint8_t MODES          = B11111000;  // 0xF8 is "AND" mask for preset convert mode
const uint8_t SINGLE         = B00000010;  // 0x02 is "OR" mask for start single convert mode
const uint8_t CONTIN         = B00000001;  // 0x01 is "OR" mask for start continual convert mode
const uint8_t CAPDAC_ON      = B10000000;  // 0x80 is "OR" mask for CAPDAC ON
const uint8_t CAPDAC_OFF     = B00000000;  // 0x00 is value for CAPDAC OFF
const uint8_t CAP_RDY        = B00000001;  // 0x01 is "AND" mask for CAP READY
const uint8_t REFERENCE      = B11101111;  // 0xEF is "AND" mask for unconditionally set internal reference
const uint8_t EEPROMStart    = 0;          // first address of EEPROM - EEPROM/PROGMEM flag (one byte of EEPROM)
const uint8_t EEPROMAddrSamplePeriod = 1;  // address for SamplePeriod variable (two bytes of EEPROM)
const uint8_t OneByte = 1;                 // auxiliary variables
const uint8_t HowManySteps = 5;            // number of samples per phase of automatic offset adjustment, the phases are four
uint8_t StepByStep = 0;                    // auxiliary variables to phase the offset adjustment process
const unsigned long AD774XTimeOut = 1000;  // response waiting time when offset is setting
unsigned long TimeTemp;                    // auxiliary variables for timing

const uint8_t SxBuffLength = 8;            // length of input buffer for parsing commands
uint8_t SxBuff[SxBuffLength + 1];          // input buffer for serial parser
uint8_t RTxBuff[20];                       // I/O buffer for AD774X registers
uint8_t I2C_State = 0;                     // status of I2C bus, 0 = without error

unsigned int SamplePeriod = 5000;          // sample period in [ms]
float C1 = 0, C2 = 0;                      // auxiliary variables for zero correction calculation
float Capacitance = 0.0, Temperature = 0.0;// real data
bool EnablePeriodicSampling = true;       // periodic sampling with output to serial port
bool EnableSerialTerminal = true;          // enable input from serial port
bool EnableOffsetAutomatic = false;        // enable automatic offset, better said automatic zero setting, is stopped as default

// the indexes in the DefaultRegisters field correspond to the addresses of each AD774X registry
const uint8_t DefaultRegisters[] = {0, 0, 0, 0, 0, 0, 0, DATA_CAP_SETUP, DATA_VT_SETUP, DATA_EXC_SETUP, DATA_CFG,
                                            DATA_CAPDACA, DATA_CAPDACB, DATA_CAP_OFFH, DATA_CAP_OFFL, 0, 0, 0, 0
                                           };


// ==================================================================================
void setup() {
  pinMode (PICO_LED, OUTPUT);
  Wire.setSDA(SDA0);
  Wire.setSCL(SCL0);
  Wire.begin();
  pinMode(RDY0, INPUT_PULLUP); // signal /RDY from AD7747

  Serial.begin(115200);
  delay(2000);
  Serial.println("Pi pico I2C test");
  Serial.println(VERSION);

  AD774X_Reset();
  if (I2C_State != 0) {
    Serial.print(F("\r\nAD774X not responding !"));
  }
  AD774X_Write_Single_Register(ADR_CAP_SETUP, 0xA0);  // CAP setup
  AD774X_Write_Single_Register(ADR_VT_SETUP,  0x81);  // VT setup
  AD774X_Write_Single_Register(ADR_EXC_SETUP, 0x0E);  // EXC setup
  // AD774X_Write_Single_Register(ADR_CFG,       0xA1);  // continuous, /RDY = 5.38 Hz
  AD774X_Write_Single_Register(ADR_CFG,       0x39);  // continuous, slow cap rate /RDY: 4.17 Hz

  // load AD7747 registers to correct initial values
  //for (uint8_t i = 0; i<8; i++) {
  //  RTxBuff[i] = DefaultRegisters[i+ADR_CAP_SETUP];
  //}
  //AD774X_Write_Registers(ADR_CAP_SETUP, RTxBuff, 8);
}

// read each AD7747 register and display its value in hex
void showRegisters() {

    AD774X_Read_Registers(ADR_STATUS, RTxBuff, 19);
    //Serial.println(F("\r\nList all registers 00 - 18:"));
    for (uint8_t i = ADR_STATUS; i < 19; i++) {
      Serial.print(F("R"));
      if (i < 10)Serial.print(F("0"));
      Serial.print(i);
      Serial.print(F("="));
      if (RTxBuff[i] < 0x10)Serial.print(F("0"));
      Serial.print(RTxBuff[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();  // return cursor to left after printout
}

// scan I2C bus for active devices
void findDevice() { 
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }


  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done");
} 

// --------------------------------------------------------------
void loop() {

  while (digitalRead(RDY0)) {}   // wait for /RDY signal to go low

  digitalWrite (PICO_LED, HIGH);
  //delay(500);
  //showRegisters();  // display the register values
  //delay(100);

  unsigned long msec = millis();
  float sec = msec/1000.0;
  Serial.print(sec,1); // print elapsed time in seconds
  Serial.print(", ");
  AD774X_Read_Registers(ADR_CAP_DATAH, RTxBuff, 6);
  SerialPrintData();

  while (!digitalRead(RDY0)) {}   // wait for /RDY signal to go high
  digitalWrite (PICO_LED, LOW);

} // end loop()

