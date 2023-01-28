// SDL_Arduino_WeatherSenseWR3
// SwitchDoc Labs WEatherRack3



#define TXDEBUG
//#undef TXDEBUG
#include <JeeLib.h>

#include "MemoryFree.h"




#include "Entropy.h"


#include <EEPROM.h>

// WeatherSenseProtocol of 8 is SolarMAX LiPo   BatV < 7V
// WeatherSenseProtocol of 10 is SolarMAX LeadAcid   BatV > 7V LoRa version
// WeatherSenseProtocol of 11 is SolarMAX4 LeadAcid BatV > 7V
// WeatherSenseProtocol of 16 is WeatherSense ThunderBoard 433MHz
// WeatherSenseProtocol of 17 is Generic data
// WeatherSenseProtocol of 18 is WeatherSense AfterShock
// WeatherSenseProtocol of 19 is WeatherSense Radiation
// WeatherSenseProtocol of 20 is WeatherSense WeatherRack3
// WeatherSenseProtocol of 21 is WeatherSense WeatherRack3 Power

#define WEATHERSENSEPROTOCOL 20
#define WEATHERSENSEPROTOCOLWR3P 21
#define LED 13
// Software version
#define SOFTWAREVERSION 8

// unique ID of this WeatherSenseWeatherRack3 system - change if you have multiple WeatherSenseWeatherRack3 systems
unsigned int myID;
// Which WeatherSense WeatherRack3 Protocol Version
#define WEATHERSENSEPROTOCOLVERSION 1



// Number of milliseconds between wake up  30 seconds.   - if you move this over 60000 ms, you will need to add the watchdog in the sleep loop -
// see SDL_Arduino_WeatherRack3 ResetWatchDog

#define SLEEPCYCLE 30000
#define WAKEUPS 1


#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;
/*
ISR(_vect) {
  Sleepy::watchdogEvent();
}
*/
#include <RH_ASK.h>

#include <avr/sleep.h>
#include <avr/power.h>



// the three channels of the INA3221 named for INA3221 Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3


// Other Pins
#define WATCHDOG_1 5

// 433MHz pins
#define TXPIN 8
#define RXPIN 10

#define WEATHERSTATIONPOWERPIN 9

// MODBUS RTU Address for WR3
#define WINDSPEED_ADDRESS 0x1F4
#define WINDFORCE_ADDRESS 0x1F5
#define WINDDIRECTIONNUMBER_ADDRESS 0x1F6
#define WINDDIRECTIONDEGREES_ADDRESS 0x1F7
#define HUMIDITY_ADDRESS 0x1F8
#define TEMPERATURE_ADDRESS 0x1F9
#define NOISE_ADDRESS 0x1FA
#define PM2_5_ADDRESS 0x1FB
#define PM10_ADDRESS 0x1FC
#define PRESSURE_ADDRESS 0x1FD
#define HLUX_ADDRESS 0x1FE
#define LLUX_ADDRESS 0x1FF
#define N20WLIGHT_ADDRESS 0x200
#define RAIN_ADDRESS 0x201

long successfulmessages = 0;
long unsuccessfulmessages = 0;
long badchecksummessages = 0;


bool renogyState = true;

// MODBUS RTU Addresses for Renogy
// device information
#define Controller_voltage_rating_Volts 0x00A
#define Controller_current_rating_Amps 0x00A
#define Controller_discharge_current_rating_Amps 0x00B
#define Controller_Type 0x00B

#define Controller_model_name_0 0x00C
#define Controller_model_name_1 0x00D
#define Controller_model_name_2 0x00E
#define Controller_model_name_3 0x00F
#define Controller_model_name_4 0x010
#define Controller_model_name_5 0x011
#define Controller_model_name_6 0x012
#define Controller_model_name_7 0x013

#define Controller_software_version_0 0x014
#define Controller_software_version_1 0x015
#define Controller_hardware_version_0 0x016
#define Controller_hardware_version_1 0x017
#define Controller_serial_number_0 0x018
#define Controller_serial_number_1 0x019
#define Controller_MODBUS_address 0x01A
// supported registers for state data:

#define Battery_Capacity_Percent 0x100
#define Battery_Voltage_Volts 0x101
#define Battery_Charge_Current_Amps 0x102
#define Battery_Temperature_Celcius 0x103
#define Controller_Temperature_Celcius 0x103
#define Load_Voltage_Volts 0x104
#define Load_Current_Amps 0x105
#define Load_Power_Watts 0x106
#define Solar_Panel_PV_Voltage_Volts 0x107
#define Solar_Panel_PV_Current_Amps 0x108
#define Solar_Panel_PV_Power_Watts 0x109
#define Min_Battery_Voltage_Today_Volts_H 0x10B
#define Min_Battery_Voltage_Today_Volts_L 0x10C
#define Max_Charge_Current_Today_Amps 0x10D
#define Max_Discharge_Current_Today_Amps 0x10E
#define Max_Charge_Power_Today_Watts 0x10F
#define Max_Discharge_Power_Today_Watts 0x110
#define Charge_Amp_Hrs_Today_Amp_Hours 0x111
#define Discharge_Amp_Hrs_Today_Amp_Hours 0x112
#define Charge_Watt_Hrs_Today_Watt_Hours 0x113
#define Discharge_Watt_Hrs_Today_Watt_Hours 0x114
#define Controller_Uptime_Days 0x115
#define Total_Battery_Over_Charges_Count 0x116
#define Total_Battery_Full_Charges_Count 0x117

#define Load_Turn_On_Address 0x010A

#include <SoftwareSerial.h>

// Renogy

#define TX_RENOGY A2
#define RX_RENOGY A3

SoftwareSerial myRenogySerial(RX_RENOGY, TX_RENOGY);


// WeatherRack3

#define TX_WR3 2
#define RX_WR3 3

SoftwareSerial myWR3Serial(RX_WR3, TX_WR3);


struct WeatherRack3Data {
  unsigned int windspeed;
  unsigned int windforce;
  unsigned int winddirection;
  unsigned int winddirectiondegrees;
  unsigned int humidity;
  unsigned int temperature;
  unsigned int noise;
  unsigned int PM2_5;
  unsigned int PM10;
  unsigned int pressure;
  unsigned int hwlux;
  unsigned int lwlux;
  unsigned int lightvalue20W;
  unsigned int rain;
};

WeatherRack3Data WR3Data;




RH_ASK driver(2000, RXPIN, TXPIN);

unsigned long MessageCount = 0;

#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>


//#include <, returnMessage, myWR3Serial.h>

typedef enum {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;

int state = 0;
unsigned long wakeCount;
// Device Present State Variables


bool WeatherRack3_Present;

byte byteBuffer[100];  // contains string to be sent to RX unit

// State Variables

long TimeStamp;



// State Status Renogy
struct RenogyDataStructure {
  unsigned int D_Controller_voltage_rating_Volts;
  unsigned int D_Controller_current_rating_Amps;
  unsigned int D_Controller_discharge_current_rating_Amps;
  unsigned int D_Controller_Type;

  unsigned int D_Controller_model_name_0;
  unsigned int D_Controller_model_name_1;
  unsigned int D_Controller_model_name_2;
  unsigned int D_Controller_model_name_3;
  unsigned int D_Controller_model_name_4;
  unsigned int D_Controller_model_name_5;
  unsigned int D_Controller_model_name_6;
  unsigned int D_Controller_model_name_7;

  unsigned int D_Controller_software_version_0;
  unsigned int D_Controller_software_version_1;
  unsigned int D_Controller_hardware_version_0;
  unsigned int D_Controller_hardware_version_1;
  unsigned int D_Controller_serial_number_0;
  unsigned int D_Controller_serial_number_1;

  unsigned int D_Battery_Capacity_Percent;
  unsigned int D_Battery_Voltage_Volts;
  unsigned int D_Battery_Charge_Current_Amps;
  unsigned int D_Battery_Temperature_Celcius;
  unsigned int D_Controller_Temperature_Celcius;
  unsigned int D_Load_Voltage_Volts;
  unsigned int D_Load_Current_Amps;
  unsigned int D_Load_Power_Watts;
  unsigned int D_Solar_Panel_PV_Voltage_Volts;
  unsigned int D_Solar_Panel_PV_Current_Amps;
  unsigned int D_Solar_Panel_PV_Power_Watts;
  unsigned int D_Min_Battery_Voltage_Today_Volts_H;
  unsigned int D_Min_Battery_Voltage_Today_Volts_L;
  unsigned int D_Max_Charge_Current_Today_Amps;
  unsigned int D_Max_Discharge_Current_Today_Amps;
  unsigned int D_Max_Charge_Power_Today_Watts;
  unsigned int D_Max_Discharge_Power_Today_Watts;
  unsigned int D_Charge_Amp_Hrs_Today_Amp_Hours;
  unsigned int D_Discharge_Amp_Hrs_Today_Amp_Hours;
  unsigned int D_Charge_Watt_Hrs_Today_Watt_Hours;
  unsigned int D_Discharge_Watt_Hrs_Today_Watt_Hours;
  unsigned int D_Controller_Uptime_Days;
  unsigned int D_Total_Battery_Over_Charges_Count;
  unsigned int D_Total_Battery_Full_Charges_Count;
};
byte AuxA;
byte SoftwareVersion;

RenogyDataStructure RenogyData;

// AuxA has state information
// coded in the byte
// 0000DCBA


// A = 1, WEatherRack3 Present, 0 not present
// B = 1, IN3221 (Solar) Present, 0 not present

// D = 1, Just rebooted






wakestate wakeState;  // who woke us up?


long nextSleepLength;

bool weatherStationState = false;
// Relay Power On/Off






void beginWR3() {
  pinMode(RX_WR3, INPUT);
  pinMode(TX_WR3, OUTPUT);
  myWR3Serial.begin(4800);
}
void beginRenogy() {
  pinMode(RX_RENOGY, INPUT);
  pinMode(TX_RENOGY, OUTPUT);
  myRenogySerial.begin(9600);

  myRenogySerial.listen();  // must listen to the port
 byte returnMessage[10];
 
// Now reset the unit to factory defaults
sendRenogyCommand(0x78, 0x0000, 0x0001, returnMessage);
delay(500);
sendRenogyCommand(0x78, 0x0000, 0x0001, returnMessage);
  // Now send the turn on command twice
 
  // 01 06 010A 0001 69F4
  sendRenogyCommand(0x06, 0x010A, 0x0001, returnMessage);
  delay(500);
  sendRenogyCommand(0x06, 0x010A, 0x0001, returnMessage);
}

// Compute the MODBUS RTU CRC
unsigned int ModRTU_CRC(byte *buf, int len) {
  unsigned int crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (unsigned int)buf[pos];  // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {  // Loop over each bit
      if ((crc & 0x0001) != 0) {    // If the LSB is set
        crc >>= 1;                  // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else        // Else LSB is not set
        crc >>= 1;  // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}



unsigned int sendRenogyCommand(byte command, unsigned int startingaddress, unsigned int commanddata, byte *returnMessage) {

  byte myMessage[10];
  //delay(1000);




  Serial.print(F("Send Renogy Command Address=0x"));
  Serial.println(startingaddress, HEX);

  myMessage[0] = 0x01;
  myMessage[1] = command;
  myMessage[2] = startingaddress >> 8;
  myMessage[3] = startingaddress & 0XFF;
  myMessage[4] = commanddata >> 8;
  myMessage[5] = commanddata & 0XFF; 

  unsigned int checksumValue;
  checksumValue = ModRTU_CRC(myMessage, 6);

  Serial.print(F("Sending ModChecksum="));
  checksumValue = (checksumValue << 8) | (checksumValue & 0xFF00) >> 8;
  Serial.println(checksumValue, HEX);
  myMessage[6] = checksumValue >> 8;
  myMessage[7] = checksumValue & 0xFF;

  Serial.println(F("Sending Message"));
  int i;
  for (i = 0; i < 8; i++) {
    myRenogySerial.write(myMessage[i]);
  }


  i = 0;

  delay(100);
  unsigned int _len;
  _len = 0;

  Serial.print(F("myRenogySerial.available()="));
  Serial.println(myRenogySerial.available());
  while (myRenogySerial.available() > _len) {
    _len = myRenogySerial.available();
    delayMicroseconds(15);
  }

  Serial.print(F("_len="));
  Serial.println(_len);
  if (_len == 0) {
    unsuccessfulmessages++;
    return 0;
  }
  for (i = 0; i < _len; i++) returnMessage[i] = myRenogySerial.read();

  for (i = 0; i < _len; i++) {
    Serial.print(F("returnMessage["));
    Serial.print(i);
    Serial.print(F("]="));
    Serial.println(returnMessage[i], HEX);
  }


  checksumValue = ModRTU_CRC(returnMessage, 6);
  Serial.print(F("ModChecksum="));
  checksumValue = (checksumValue << 8) | (checksumValue & 0xFF00) >> 8;
  Serial.println(checksumValue, HEX);
  Serial.println(0xFFFF & (returnMessage[6] << 8 | returnMessage[7]), HEX);

  if (checksumValue == (0xFFFF & (returnMessage[6] << 8 | returnMessage[7]))) {
    successfulmessages++;
    return 1;
  } else {
    Serial.println(F(">>>>>>Bad received Checksum"));
    unsuccessfulmessages++;
    badchecksummessages++;
    return 0;
  }
}



unsigned int sendreceiveRenogyMessage(unsigned int startingaddress, byte *returnMessage) {

  byte myMessage[10];
  //delay(1000);




  Serial.print(F("Address=0x"));
  Serial.println(startingaddress, HEX);

  myMessage[0] = 0x01;
  myMessage[1] = 0x03;
  myMessage[2] = startingaddress >> 8;
  myMessage[3] = startingaddress & 0XFF;
  myMessage[4] = 0x00;
  myMessage[5] = 0x01;
  unsigned int checksumValue;
  checksumValue = ModRTU_CRC(myMessage, 6);

  Serial.print(F("Sending ModChecksum="));
  checksumValue = (checksumValue << 8) | (checksumValue & 0xFF00) >> 8;
  Serial.println(checksumValue, HEX);
  myMessage[6] = checksumValue >> 8;
  myMessage[7] = checksumValue & 0xFF;

  Serial.println(F("Sending Message"));
  int i;
  for (i = 0; i < 8; i++) {
    myRenogySerial.write(myMessage[i]);
  }


  i = 0;

  delay(100);
  unsigned int _len;
  _len = 0;

  Serial.print(F("myRenogySerial.available()="));
  Serial.println(myRenogySerial.available());
  while (myRenogySerial.available() > _len) {
    _len = myRenogySerial.available();
    delayMicroseconds(15);
  }

  Serial.print(F("_len="));
  Serial.println(_len);
  if (_len == 0) {
    unsuccessfulmessages++;
    return 0;
  }
  for (i = 0; i < _len; i++) returnMessage[i] = myRenogySerial.read();

  for (i = 0; i < _len; i++) {
    Serial.print(F("returnMessage["));
    Serial.print(i);
    Serial.print(F("]="));
    Serial.println(returnMessage[i], HEX);
  }


  checksumValue = ModRTU_CRC(returnMessage, 5);
  Serial.print(F("ModChecksum="));
  checksumValue = (checksumValue << 8) | (checksumValue & 0xFF00) >> 8;
  Serial.println(checksumValue, HEX);
  Serial.println(0xFFFF & (returnMessage[5] << 8 | returnMessage[6]), HEX);

  if (checksumValue == (0xFFFF & (returnMessage[5] << 8 | returnMessage[6]))) {
    successfulmessages++;
    return 1;
  } else {
    Serial.println(F(">>>>>>Bad received Checksum"));
    unsuccessfulmessages++;
    badchecksummessages++;
    return 0;
  }
}





unsigned int sendreceiveWR3Message(unsigned int startingaddress, byte *returnMessage) {

  byte myMessage[10];
  //delay(1000);




  Serial.print(F("Address=0x"));
  Serial.println(startingaddress, HEX);

  myMessage[0] = 0x01;
  myMessage[1] = 0x03;
  myMessage[2] = startingaddress >> 8;
  myMessage[3] = startingaddress & 0XFF;
  myMessage[4] = 0x00;
  myMessage[5] = 0x01;
  unsigned int checksumValue;
  checksumValue = ModRTU_CRC(myMessage, 6);

  Serial.print(F("Sending ModChecksum="));
  checksumValue = (checksumValue << 8) | (checksumValue & 0xFF00) >> 8;
  Serial.println(checksumValue, HEX);
  myMessage[6] = checksumValue >> 8;
  myMessage[7] = checksumValue & 0xFF;

  Serial.println(F("Sending Message"));
  int i;
  for (i = 0; i < 8; i++) {
    myWR3Serial.write(myMessage[i]);
  }


  i = 0;

  delay(100);
  unsigned int _len;
  _len = 0;

  Serial.print(F("myWR3Serial.available()="));
  Serial.println(myWR3Serial.available());
  while (myWR3Serial.available() > _len) {
    _len = myWR3Serial.available();
    delayMicroseconds(15);
  }

  Serial.print(F("_len="));
  Serial.println(_len);
  if (_len == 0) {
    unsuccessfulmessages++;
    return 0;
  }
  for (i = 0; i < _len; i++) returnMessage[i] = myWR3Serial.read();

  for (i = 0; i < _len; i++) {
    Serial.print(F("returnMessage["));
    Serial.print(i);
    Serial.print(F("]="));
    Serial.println(returnMessage[i], HEX);
  }


  checksumValue = ModRTU_CRC(returnMessage, 5);
  Serial.print(F("ModChecksum="));
  checksumValue = (checksumValue << 8) | (checksumValue & 0xFF00) >> 8;
  Serial.println(checksumValue, HEX);
  Serial.println(0xFFFF & (returnMessage[5] << 8 | returnMessage[6]), HEX);

  if (checksumValue == (0xFFFF & (returnMessage[5] << 8 | returnMessage[6]))) {
    successfulmessages++;
    return 1;
  } else {
    Serial.println(F(">>>>>>Bad received Checksum"));
    unsuccessfulmessages++;
    badchecksummessages++;
    return 0;
  }
}

int convert4ByteLongVariables(int bufferCount, long myVariable) {

  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;
}

int convert4ByteFloatVariables(int bufferCount, float myVariable) {
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];


    bufferCount++;
  }

  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable) {


  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;


  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;

  return bufferCount;
}

int convert1ByteVariables(int bufferCount, int myVariable) {


  byteBuffer[bufferCount] = (byte)myVariable;
  bufferCount++;
  return bufferCount;
}

int checkSum(int bufferCount) {
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}



// variables






int buildProtocolMessage() {

  int bufferCount;


  bufferCount = 0;

  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);
  byteBuffer[bufferCount] = 0;  // WeatherSense unique ID - back wards compatible with older weathersense ids
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOL;  // Type of WeatherSense System
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOLVERSION;  // WeatherSense WeatherRack protocol version
  bufferCount++;

  bufferCount = convert2ByteVariables(bufferCount, myID);

  bufferCount = convert2ByteVariables(bufferCount, WR3Data.windspeed);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.windforce);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.winddirectiondegrees);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.humidity);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.temperature);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.noise);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.PM2_5);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.PM10);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.pressure);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.hwlux);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.lwlux);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.lightvalue20W);
  bufferCount = convert2ByteVariables(bufferCount, WR3Data.rain);


  byteBuffer[bufferCount] = AuxA;  // Aux
  bufferCount++;
  byteBuffer[bufferCount] = SOFTWAREVERSION;
  bufferCount++;



  return bufferCount;
}



int buildProtocol2Message()  // for power system reporting
{

  int bufferCount;


  bufferCount = 0;

  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);


  byteBuffer[bufferCount] = 0;  // WeatherSense unique ID - for backwards compatiblity
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOLWR3P;  // Type of WeatherSense System
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOLVERSION;  // WeatherSense WR3 protocol version
  bufferCount++;
  bufferCount = convert2ByteVariables(bufferCount, myID);

  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Battery_Capacity_Percent);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Battery_Voltage_Volts);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Battery_Charge_Current_Amps);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Load_Voltage_Volts);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Load_Current_Amps);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Solar_Panel_PV_Voltage_Volts);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Solar_Panel_PV_Current_Amps);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Min_Battery_Voltage_Today_Volts_H);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Min_Battery_Voltage_Today_Volts_L);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Max_Charge_Current_Today_Amps);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Max_Discharge_Current_Today_Amps);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Charge_Amp_Hrs_Today_Amp_Hours);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Discharge_Amp_Hrs_Today_Amp_Hours);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Charge_Watt_Hrs_Today_Watt_Hours);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Discharge_Watt_Hrs_Today_Watt_Hours);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Controller_Uptime_Days);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Total_Battery_Over_Charges_Count);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Total_Battery_Full_Charges_Count);
  bufferCount = convert2ByteVariables(bufferCount, RenogyData.D_Controller_Type);

  bufferCount = convert4ByteLongVariables(bufferCount, wakeCount);

  byteBuffer[bufferCount] = AuxA;  // Aux
  bufferCount++;
  byteBuffer[bufferCount] = SOFTWAREVERSION;
  bufferCount++;



  return bufferCount;
}

unsigned int returnValue(byte *returnMessage) {
  unsigned int returnValue;
  returnValue = returnMessage[3] << 8 | returnMessage[4];
  Serial.print(F("returnValue="));
  Serial.println(returnValue, HEX);
  return returnValue;
}



void readRenogy() {

  delay(1000);
  byte returnMessage[10];
  Serial.println(F("reading Renogy"));

  myRenogySerial.listen();  // must listen to the port



  int success;
  success = sendreceiveRenogyMessage(Battery_Voltage_Volts, returnMessage);  // start transfers - dummy



  success = sendreceiveRenogyMessage(Battery_Capacity_Percent, returnMessage);
  if (success) {
    RenogyData.D_Battery_Capacity_Percent = (returnMessage[3] << 8 | returnMessage[4]);
  }
  success = sendreceiveRenogyMessage(Battery_Voltage_Volts, returnMessage);
  if (success) {
    RenogyData.D_Battery_Voltage_Volts = (returnMessage[3] << 8 | returnMessage[4]);
  }
  success = sendreceiveRenogyMessage(Battery_Charge_Current_Amps, returnMessage);
  if (success) {
    RenogyData.D_Battery_Charge_Current_Amps = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Battery_Temperature_Celcius, returnMessage);
  if (success) {
    RenogyData.D_Battery_Temperature_Celcius = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Load_Voltage_Volts, returnMessage);
  if (success) {
    RenogyData.D_Load_Voltage_Volts = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Load_Current_Amps, returnMessage);
  if (success) {
    RenogyData.D_Load_Current_Amps = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Solar_Panel_PV_Voltage_Volts, returnMessage);
  if (success) {
    RenogyData.D_Solar_Panel_PV_Voltage_Volts = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Solar_Panel_PV_Current_Amps, returnMessage);
  if (success) {
    RenogyData.D_Solar_Panel_PV_Current_Amps = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Min_Battery_Voltage_Today_Volts_H, returnMessage);
  if (success) {
    RenogyData.D_Min_Battery_Voltage_Today_Volts_H = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Min_Battery_Voltage_Today_Volts_L, returnMessage);
  if (success) {
    RenogyData.D_Min_Battery_Voltage_Today_Volts_L = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Max_Charge_Current_Today_Amps, returnMessage);
  if (success) {
    RenogyData.D_Max_Charge_Current_Today_Amps = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Max_Discharge_Current_Today_Amps, returnMessage);
  if (success) {
    RenogyData.D_Max_Discharge_Current_Today_Amps = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Max_Charge_Power_Today_Watts, returnMessage);
  if (success) {
    RenogyData.D_Max_Charge_Power_Today_Watts = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Max_Discharge_Power_Today_Watts, returnMessage);
  if (success) {
    RenogyData.D_Max_Discharge_Power_Today_Watts = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Charge_Amp_Hrs_Today_Amp_Hours, returnMessage);
  if (success) {
    RenogyData.D_Charge_Amp_Hrs_Today_Amp_Hours = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Discharge_Amp_Hrs_Today_Amp_Hours, returnMessage);
  if (success) {
    RenogyData.D_Discharge_Amp_Hrs_Today_Amp_Hours = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Charge_Watt_Hrs_Today_Watt_Hours, returnMessage);
  if (success) {
    RenogyData.D_Charge_Watt_Hrs_Today_Watt_Hours = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Discharge_Watt_Hrs_Today_Watt_Hours, returnMessage);
  if (success) {
    RenogyData.D_Discharge_Watt_Hrs_Today_Watt_Hours = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Controller_Uptime_Days, returnMessage);
  if (success) {
    RenogyData.D_Controller_Uptime_Days = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Total_Battery_Over_Charges_Count, returnMessage);
  if (success) {
    RenogyData.D_Total_Battery_Over_Charges_Count = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Total_Battery_Full_Charges_Count, returnMessage);
  if (success) {
    RenogyData.D_Total_Battery_Full_Charges_Count = (returnMessage[3] << 8 | returnMessage[4]);
  }

  success = sendreceiveRenogyMessage(Controller_Type, returnMessage);
  if (success) {
    RenogyData.D_Controller_Type = (returnMessage[3] << 8 | returnMessage[4]);
  }

  Serial.print("unsuccessfulmessages = ");
  Serial.print(unsuccessfulmessages);
  Serial.print(" successfulmessages = ");
  Serial.print(successfulmessages);
  Serial.print(" badchecksummessages = ");
  Serial.println(badchecksummessages);
}




void readWeatherRack3() {
  //delay(2000);
  delay(1000);
  byte returnMessage[10];

  int success;
  myWR3Serial.listen();  // must listen to the port
  // two dummy reads
  //sendreceiveWR3Message(HUMIDITY_ADDRESS, returnMessage, myWR3Serial);

  sendreceiveWR3Message(HUMIDITY_ADDRESS, returnMessage);

  success = sendreceiveWR3Message(WINDSPEED_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.windspeed = returnValue(returnMessage);

  success = sendreceiveWR3Message(WINDFORCE_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.windforce = returnValue(returnMessage);

  //success = sendreceiveWR3Message(WINDDIRECTIONNUMBER_ADDRESS, returnMessage, myWR3Serial);
  //if (success == true)
  //  WR3Data.winddirection = returnValue(returnMessage);

  success = sendreceiveWR3Message(WINDDIRECTIONDEGREES_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.winddirectiondegrees = returnValue(returnMessage);

  success = sendreceiveWR3Message(HUMIDITY_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.humidity = returnValue(returnMessage);

  success = sendreceiveWR3Message(TEMPERATURE_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.temperature = returnValue(returnMessage);

  success = sendreceiveWR3Message(NOISE_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.noise = returnValue(returnMessage);

  success = sendreceiveWR3Message(PM2_5_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.PM2_5 = returnValue(returnMessage);

  success = sendreceiveWR3Message(PM10_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.PM10 = returnValue(returnMessage);

  success = sendreceiveWR3Message(PRESSURE_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.pressure = returnValue(returnMessage);

  success = sendreceiveWR3Message(HLUX_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.hwlux = returnValue(returnMessage);

  success = sendreceiveWR3Message(LLUX_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.lwlux = returnValue(returnMessage);

  success = sendreceiveWR3Message(N20WLIGHT_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.lightvalue20W = returnValue(returnMessage);

  success = sendreceiveWR3Message(RAIN_ADDRESS, returnMessage);
  if (success == true)
    WR3Data.rain = returnValue(returnMessage);

  Serial.print(F("unsuccessfulmessages = "));
  Serial.print(unsuccessfulmessages);
  Serial.print(F(" successfulmessages = "));
  Serial.print(successfulmessages);
  Serial.print(F(" badchecksummessages = "));
  Serial.println(badchecksummessages);

  // test transmitting
}




void return2Digits(char returnString[], char *buffer2, int digits) {
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);
}



void ResetWatchdog() {


  digitalWrite(WATCHDOG_1, LOW);
  delay(200);
  digitalWrite(WATCHDOG_1, HIGH);


  //Serial.println(F("Watchdog1 Reset - Patted the Dog"));
}

//
//
//







void sendMessage() {



#if defined(TXDEBUG)
  Serial.println(F("###############"));
  Serial.print(F(" MessageCount="));
  Serial.println(MessageCount);
  Serial.print(F(" myID=0x"));
  Serial.println(myID, HEX);
  Serial.print(F(" STATUS - WeatherSenseProtocol:"));
  Serial.println(WEATHERSENSEPROTOCOL);
  Serial.print(F(" WakeState="));
  Serial.println(wakeState);
  Serial.print(F(" wakeCount="));
  Serial.println(wakeCount);


  Serial.print(F(" WR3Data.windspeed="));
  Serial.println(WR3Data.windspeed);
  Serial.print(F(" WR3Data.windforce="));
  Serial.println(WR3Data.windforce);
  Serial.print(F(" WR3Data.winddirection="));
  Serial.println(WR3Data.winddirection);
  Serial.print(F(" WR3Data.winddirectiondegrees="));
  Serial.println(WR3Data.winddirectiondegrees);
  Serial.print(F(" WR3Data.humidity="));
  Serial.println(WR3Data.humidity);
  Serial.print(F(" WR3Data.temperature="));
  Serial.println(WR3Data.temperature);
  Serial.print(F(" WR3Data.noise="));
  Serial.println(WR3Data.noise);
  Serial.print(F(" WR3Data.PM2_5="));
  Serial.println(WR3Data.PM2_5);
  Serial.print(F(" WR3Data.PM10="));
  Serial.println(WR3Data.PM10);
  Serial.print(F(" WR3Data.pressure="));
  Serial.println(WR3Data.pressure);
  Serial.print(F(" WR3Data.hwlux="));
  Serial.println(WR3Data.hwlux);
  Serial.print(F(" WR3Data.lwlux="));
  Serial.println(WR3Data.lwlux);
  Serial.print(F(" WR3Data.lightvalue20W="));
  Serial.println(WR3Data.lightvalue20W);
  Serial.print(F(" WR3Data.rain="));
  Serial.println(WR3Data.rain);

  Serial.print(F(" Battery Voltage:  "));
  Serial.print(RenogyData.D_Battery_Voltage_Volts * 0.1);
  Serial.println(F(" V"));
  Serial.print(F(" Battery Current:       "));
  Serial.print(RenogyData.D_Battery_Charge_Current_Amps * 0.01);
  Serial.println(F(" A"));
  Serial.print(F(" Solar Panel Voltage:   "));
  Serial.print(RenogyData.D_Solar_Panel_PV_Voltage_Volts * 0.1);
  Serial.println(F(" V"));
  Serial.print(F(" Solar Current:  "));
  Serial.print(RenogyData.D_Solar_Panel_PV_Current_Amps * 0.01);
  Serial.println(F(" A"));
  Serial.print(F(" Load Voltage:  "));
  Serial.print(RenogyData.D_Load_Voltage_Volts * 0.1);
  Serial.println(F(" V"));
  Serial.print(F(" Load Current:       "));
  Serial.print(RenogyData.D_Load_Current_Amps * 0.01);
  Serial.println(F(" A"));
  Serial.print(F(" Battery Capacity:       "));
  Serial.print(RenogyData.D_Battery_Capacity_Percent);
  Serial.println(F(" %"));
  Serial.print(F(" Currentmillis() = "));
  Serial.println(millis());

  Serial.print(F("  AuxA State:"));
  Serial.print(AuxA);
  Serial.print(F(" "));
  Serial.println(AuxA, HEX);

  Serial.println(F("###############"));
#endif
  // write out the current protocol to message and send.
  int bufferLength;


  Serial.println(F("----------Sending packets----------"));

  bufferLength = buildProtocol2Message();

  // Send the base message

  Serial.print(F("bufferlength="));
  Serial.println(bufferLength);

  driver.send(byteBuffer, bufferLength);

  if (!driver.waitPacketSent(6000)) {
    //Serial.println(F("Timeout on transmission"));
    // re-initialize board
    if (!driver.init()) {
      //Serial.println(F("init failed"));
      while (1)
        ;
    }
    //Serial.println(F("----------Board Reinitialized----------"));
  }

  Serial.println(F("----------After Sending Power packet----------"));

  for (int i = 0; i < bufferLength; i++) {
    Serial.print(F(" "));
    if (byteBuffer[i] < 16) {
      Serial.print(F("0"));
    }
    Serial.print(byteBuffer[i], HEX);  //  write buffer to hardware serial port
  }
  Serial.println();


  Serial.println(F("----------After Wait Sending Power packet----------"));
  delay(5000);

  bufferLength = buildProtocolMessage();

  // Send the base message

  Serial.print(F("bufferlength="));
  Serial.println(bufferLength);

  driver.send(byteBuffer, bufferLength);

  if (!driver.waitPacketSent(6000)) {
    //Serial.println(F("Timeout on transmission"));
    // re-initialize board
    if (!driver.init()) {
      //Serial.println(F("init failed"));
      while (1)
        ;
    }
    //Serial.println(F("----------Board Reinitialized----------"));
  }
  Serial.println(F("----------After Sending WR3 packet----------"));

  for (int i = 0; i < bufferLength; i++) {
    Serial.print(F(" "));
    if (byteBuffer[i] < 16) {
      Serial.print(F("0"));
    }
    Serial.print(byteBuffer[i], HEX);  //  write buffer to hardware serial port
  }
  Serial.println();


  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);


  MessageCount++;
  // set to MessageCount
  EEPROM.put(0, MessageCount);
}


void setup() {



  Serial.begin(115200);  // TXDEBUGging only

    AuxA = 0x00;

  Serial.println();
  Serial.println();
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println(F("WeatherSense WeatherRack3 / Renogy Controller"));
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);

  Serial.print(F("FreeMemory="));
  Serial.println(freeMemory());

  // Pat the WatchDog
  ResetWatchdog();

  beginWR3();

  beginRenogy();




  wakeCount = 0;
  Entropy.initialize();




  /*  
  ISR(_vect) {
  Sleepy::watchdogEvent();
}
*/

  unsigned long tempLong;









  EEPROM.get(4, tempLong);
  delay(2000);

  if (tempLong == 0xFFFFFFFF)  // uninitialized
  {
    Serial.print(F("Writing MessageCount="));
    Serial.println(MessageCount, HEX);
    // set to messageID
    EEPROM.put(0, MessageCount);
  } else {
    EEPROM.get(0, MessageCount);
    // read the message value
  }
  Serial.print(F("MessageCount="));
  Serial.println(MessageCount);

  if (tempLong == 0xFFFFFFFF)  // uninitialized
  {
    myID = Entropy.randomWord();
    Serial.print(F("Writing myID="));
    Serial.println(myID, HEX);
    // set to messageID
    EEPROM.put(4, myID);
  } else {

    EEPROM.get(4, myID);
    // read the message value
  }
  Serial.print(F("Unit ID: 0x"));
  Serial.println(myID, HEX);

  if (!driver.init()) {
    Serial.println(F("init failed"));
    while (1)
      ;
  }

  Serial.print(F("max message length="));
  Serial.println(driver.maxMessageLength());

  // read the values of messageCount from EEPROM






  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);

  // setup initial values of variables

  wakeState = REBOOT;

  nextSleepLength = SLEEPCYCLE;



  TimeStamp = 0;


  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);



  int error;

  WeatherRack3_Present = true;
  error = 0;

  if (error == 0) {
    Serial.println(F("WeatherRack3 device found"));
    WeatherRack3_Present = true;
    // State Variable
    AuxA = AuxA | 0X01;
  } else if (error == 4) {
    Serial.println(F("WeatherRack3 device Not Found"));
    WeatherRack3_Present = false;
  }
  //start D7S connection






  // Pat the WatchDog
  ResetWatchdog();


  Serial.println(F("\nReady for Weather!"));
}





void loop() {




  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState="));
  Serial.println(wakeState);
#endif


  if ((wakeState == SLEEP_INTERRUPT) || (wakeState == REBOOT)) {

    wakeState = NO_INTERRUPT;






    TimeStamp = millis();



    if (wakeCount > 1)
      AuxA = AuxA & 0xF7;
    else
      AuxA = AuxA | 0x08;



    bool readyToTransmit = false;



    // check if it is time to send message (every 10 minutes or on interrupt) - 30 seconds pre check

    if (((wakeCount % WAKEUPS) == 0)) {

      // Pat the WatchDog
      ResetWatchdog();
      readyToTransmit = true;

      readWeatherRack3();

      readRenogy();

      Serial.print(F("AuxA = 0x"));
      Serial.println(AuxA, HEX);
      // Pat the WatchDog
      ResetWatchdog();
    }
    Serial.println();

    if (readyToTransmit)  // ready to transmit
    {
      // transmit




      Serial.println(F(">>>>>>>>>>>>>>>Transmitting WeatherRack3 message<<<<<<<<<<<<"));
      sendMessage();

      Serial.println(F("----------Packet Sent.  Sleeping Now----------"));
    }
  }



  // Pat the WatchDog
  ResetWatchdog();

  if (wakeState != REBOOT)
    wakeState = SLEEP_INTERRUPT;
  long timeBefore;
  long timeAfter;
  timeBefore = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeBeforeSleep="));
  Serial.println(timeBefore);
#endif
  delay(100);


  //Sleepy::loseSomeTime(nextSleepLength);
  for (long i = 0; i < nextSleepLength / 16; ++i) {
    Sleepy::loseSomeTime(16);
  }



  wakeState = SLEEP_INTERRUPT;

  wakeCount++;

#if defined(TXDEBUG)
  Serial.print(F("Awake now: "));
#endif
  timeAfter = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeAfterSleep="));
  Serial.println(timeAfter);

  Serial.print(F("SleepTime = "));
  Serial.println(timeAfter - timeBefore);

  Serial.print(F("Millis Time: "));
#endif
  long time;
  time = millis();
#if defined(TXDEBUG)
  //prints time since program started
  Serial.println(time / 1000.0);
  Serial.print(F("2wakeState="));
  Serial.println(wakeState);
#endif





  // Pat the WatchDog
  ResetWatchdog();
}
