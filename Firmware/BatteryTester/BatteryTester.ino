/** @brief Arduino Battery capacity tester
    @author Pinski1
    @date 20/03/2016

    @detailed Discharges a large battery (36V 10-16Ah) via a load, logging voltage and current regularly for capacity calculation.
    Needs:
      Arduino Uno
      Adafruit DataLogger shield or similar
      Relay
      Load (2x paralleled 10 ohm 200 Watt power resistors)
      Voltage and current monitors (potential divider and ACS712)
      DS18B20 temperature sensors
*/
#include <SD.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <elapsedMillis.h>

/** Settings */
#define VOLTAGE_MAX   42000 // in milliVolts
#define VOLTAGE_MIN   28000 // in milliVolts
#define LOG_INTERVAL  10000 // in milliseconds
#define V_RATIO       46 // R1 = 100k, R2 = 12k, result in milliVolts, +0.94% error
#define I_RATIO       49 // ACS712, 20A, result in milliAmps, +0.35% error
unsigned char num_temp = 0;

/** Pin Definitions */
#define LED_RED       3
#define LED_GRN       4 // is actually red on my board!
#define LOAD_ENB      8
#define SD_SEL        10
#define V_SEN         A3
#define I_SEN         A1
#define BUTTON        6
#define ONE_WIRE_BUS  2

/** Class instantiations */
File logFile;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
char fileName[] = "LOGGER00.CSV";
int offsetCurrent = 512; // set default
void printMilliValues(unsigned int numb);
elapsedMillis timeElapsed;

void setup(void) {

  pinMode(SD_SEL, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LOAD_ENB, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(V_SEN, INPUT);
  pinMode(I_SEN, INPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GRN, LOW);
  digitalWrite(LOAD_ENB, LOW);

  Serial.begin(115200);
  tempSensor.begin();


  /** Sensor Calibration */
  Serial.println(F("\r\nCalibrating sensors..."));
  if ((analogRead(V_SEN) * V_RATIO) != 0)
  {
    Serial.print(F("Sensor calibration failed. Reading: "));
    printMilliValues(analogRead(V_SEN) * V_RATIO);
    Serial.print(F("V and "));
    printMilliValues((analogRead(I_SEN) - offsetCurrent) * I_RATIO);
    Serial.print(F("A.\r\n"));
    digitalWrite(LED_RED, HIGH);
    while (1);
  }
  else
  {
    for (int i = 0; i < 16; i++)
    {
      offsetCurrent += analogRead(I_SEN);
      delay(1);
    }
    offsetCurrent >>= 4; // average out
    // ideally should be around 512
    Serial.print(F("Offset is: "));
    printMilliValues((offsetCurrent * I_RATIO) / 10);
    Serial.println(F("V"));
    Serial.println(F("Sensors calibrated."));
  }

  /** Setup temperature sensor(s) */
  num_temp = tempSensor.getDeviceCount();
  if (tempSensor.getDeviceCount() < 1)
  {
    Serial.println(F("Couldn't talk to DS18B20 temperature sensors."));
    digitalWrite(LED_RED, HIGH);
    //while(1);
  }
  else
  {
    tempSensor.requestTemperatures();
    Serial.print(F("Temperature is "));
    Serial.print(tempSensor.getTempCByIndex(0), DEC);
    Serial.println("C");
  }

  /** Start setting up log file on SD Card */
  Serial.println(F("Initializing SD card..."));
  if (!SD.begin(SD_SEL))
  {
    Serial.println(F("SD card failed, or not present"));
    digitalWrite(LED_RED, HIGH);
    while (1);
  }
  Serial.println(F("SD card initialized."));

  for (uint8_t i = 0; i < 100; i++)
  {
    fileName[6] = i / 10 + '0';
    fileName[7] = i % 10 + '0';
    if (!SD.exists(fileName))
    {
      // only open a new file if it doesn't exist
      logFile = SD.open(fileName, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  if (!logFile)
  {
    Serial.print(F("Couldn't create log file on SD card."));
    digitalWrite(LED_RED, HIGH);
    while (1);
  }

  /** Write header to file */
  logFile.println("milliseconds,Voltage (mV),Current (mA),Power (W),Delta,Temperature (C),");
  logFile.close();

  /** Ready to start test */
  Serial.println(F("Connect battery..."));
  digitalWrite(LED_GRN, HIGH);
  delay(10);
  Serial.print(F("Waiting for Go button...\t"));
  while (digitalRead(BUTTON) == HIGH); // wait for button press
  delay(10);
  digitalWrite(LED_GRN, LOW);
  digitalWrite(LOAD_ENB, HIGH); // enable the load
  Serial.println("Going!");
  timeElapsed = 0;

}

void loop(void) {

  static unsigned int lineCounter = 2;

  if (timeElapsed > LOG_INTERVAL) {
    timeElapsed -= LOG_INTERVAL;

    digitalWrite(LED_GRN, HIGH);

    // sample time
    unsigned long tempMillis = millis();

    // sample voltage and current
    unsigned int tempVoltage = 0x00;
    int tempCurrent = 0x00;

    for (int i = 0x00; i < 4; i++)
    {
      tempVoltage += analogRead(V_SEN);
      tempCurrent += analogRead(I_SEN);
      delay(1);
    }

    tempVoltage = (tempVoltage >> 2) * V_RATIO; // in millivolts
    tempCurrent = (((tempCurrent >> 2) - offsetCurrent) * I_RATIO); // in milliamps
	  
    // sample temperature
    tempSensor.requestTemperatures();
    tempSensor.getTempCByIndex(0); // returns temperature as floating point
    tempSensor.getTemp(0); // returns temperature in 1/128 of a degC

    if ((lineCounter - 2) % 5 == 0)
    {
      Serial.print("Load is at: "); printMilliValues(tempVoltage);
      Serial.print("V "); printMilliValues(tempCurrent);
      Serial.println("A.");
    }
    else
    {
      Serial.println(F("Took a reading."));
    }

    logFile = SD.open(fileName, FILE_WRITE); // open up file
    if (!logFile)
    {
      Serial.print(F("Couldn't open log file on SD card."));
      digitalWrite(LED_RED, HIGH);
      while (1);
    }

    // if within min/max limits then continue
    if ((tempVoltage < VOLTAGE_MAX) & (tempVoltage > VOLTAGE_MIN) & (lineCounter <= 65535))
    {
      lineCounter = updateLog(tempMillis, tempVoltage, tempCurrent, -300, lineCounter); // save to SD card
      digitalWrite(LED_GRN, LOW);
    }
    else
    {
      // if min/max limits exceeded then finish
      if (lineCounter > 3)
      {
        logFile.print(",,,,=SUM(E3:E"); logFile.print(lineCounter - 1, DEC); logFile.print("),WHr");
        logFile.println(); lineCounter++;
        logFile.print(",,,,=E"); logFile.print(lineCounter - 1, DEC); logFile.println("*36,AHr");
        logFile.println(); lineCounter++;
        logFile.close(); // close it to avoid corruption
      }

      digitalWrite(LED_RED, HIGH); // too high or too low!
      digitalWrite(LOAD_ENB, LOW); // disable load
      Serial.println(F("Finished test!"));
      while (1); // stop test!
    }
  }

  /* Poor attempt at emergency stop
  
  if (digitalRead(BUTTON) != 0 &&)
  {
    // go button pushed, request stop!
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LOAD_ENB, LOW); // disable load
    Serial.println(F("Finished test!"));
    while (1); // stop test!
  }
  */
}

void printMilliValues(unsigned int numb) {

  Serial.print(numb / 1000, DEC);
  Serial.print(".");
  if ((numb % 1000) < 10) Serial.print("00");
  if ((numb % 1000) < 100) Serial.print("0");
  Serial.print(numb % 1000, DEC);
}


unsigned long updateLog(unsigned long currentMillis, unsigned int voltage, int current, int temperature, unsigned long lineCount)
{
  logFile.print(currentMillis); logFile.print(", "); // arduino millis time
  logFile.print(voltage); logFile.print(", "); // voltage
  logFile.print(current); logFile.print(","); // current
  logFile.print("=(B"); logFile.print(lineCount, DEC); logFile.print("/1000)*(C"); logFile.print(lineCount, DEC); logFile.print("/1000)"); logFile.print(","); // power
  if (lineCount <= 2); logFile.print("=(D"); logFile.print(lineCount - 1, DEC); logFile.print("-D"); logFile.print(lineCount, DEC); logFile.print(")*((A"); logFile.print(lineCount, DEC); logFile.print("-A"); logFile.print(lineCount - 1, DEC); logFile.print(")/3600000)"); logFile.print(","); // incremental power
  //logFile.print(); logFile.print// temperature
  logFile.println(temperature); lineCount++;
  logFile.close(); // close it to avoid corruption

  return lineCount;
}

