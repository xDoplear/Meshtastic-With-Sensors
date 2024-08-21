#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "Zanshin_BME680.h"

SensirionI2CScd4x scd4x;
BME680_Class BME680;

int delayTimeMin = 1; // Sets time interval in minutes to send data
int delayTimeMilli = delayTimeMin * 60000;
unsigned long previousTime = 0;
const int RAIN_PIN = 1; // Pin for rain gauge
const int GEO_PIN = 3; // Pin for 
volatile int total = 0;
float geoSensor = 0;
bool updateSerial = false;

// Root Mean Square to get better analog values from geophone
const int numSamples = 100; // Number of samples to calculate RMS
float geoReadings[numSamples];
int sampleIndex = 0;

//Set seaLevel for altitude function below
float altitude(const int32_t press, const float seaLevel = 1013.25);

//setup function
void setup() {
  Serial.begin(9600);   //baud rate setup for serial ports
  Serial1.begin(9600, SERIAL_8N1);
  pinMode(RAIN_PIN, INPUT);   //rain pin setup
  pinMode(GEO_PIN, INPUT);  //geo pin setup

  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), count, RISING);

  /*#ifdef __AVR_ATmega32U4__ //for if a board is an ATmega32U4 chip (not needed)
  delay(3000);
  #endif*/

  /*while (!Serial) { //Uncomment this to debug in serial console
      delay(100);
  }*/

  Wire.begin();

  //finds BME688 if not detected
  while (!BME680.begin(I2C_STANDARD_MODE)) {
      Serial.print(F("-  Unable to find BME688. Trying again in 5 seconds.\n"));
      delay(5000);
  }

  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  BME680.setIIRFilter(IIR4);
  BME680.setGas(320, 150);

  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);

  //Various error messages for SCD41 setup
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint16_t serial0, serial1, serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
      printSerialNumber(serial0, serial1, serial2);
  }

  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  Serial.println("Waiting for first measurement...");
}

//main loop function
void loop() {
  static char buf[16];
  int32_t temp, humidity, pressure, gas;
  BME680.getSensorData(temp, humidity, pressure, gas); //gets BME688 data

  uint16_t co2 = 0;
  float scd4xTemperature = 0.0f;
  float scd4xHumidity = 0.0f;
  bool isDataReady = false;
  uint16_t error;
  char errorMessage[256];
  unsigned long currentTime = millis(); //use to measure total time running
  geoSensor = (analogRead(GEO_PIN) / 1023.00) * 3.3; //convert analog pin value to voltage
  volatile int delayTime;
  
  // Store the reading and update the index
  geoReadings[sampleIndex] = geoSensor;
  sampleIndex = (sampleIndex + 1) % numSamples; // Wrap around the index

  // Calculate the RMS value
  float geoRMS = calculateRMS(geoReadings, numSamples);
  //Serial.println(geoRMS, 2); //Prints the analog data from pin 3

 if (geoRMS < 0.25) {
    delayTime = 10000; //send data every 10 seconds when geophone vibrating
  }
  else if (geoRMS > 0.25) {
    delayTime = delayTimeMilli;
  }

  //Check if SCD41 data is ready
  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
    Serial.print("Error trying to execute getDataReadyFlag(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    return;
  }

  if (!isDataReady) {
      return;
  }

  //Sends data using the user selected interval
  if (currentTime - previousTime >= delayTime) {
    previousTime = currentTime; 
    //prints all data whenever the SCD41 is ready
    if (isDataReady) {
      error = scd4x.readMeasurement(co2, scd4xTemperature, scd4xHumidity);
      if (error) {
        Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
      } 
      else if (co2 == 0) { //invalid reading when co2 reading is 0
        Serial.println("Invalid sample detected, skipping.");
        Serial1.println("Invalid sample detected, skipping.");
      }
      else {
        //Prints SCD41 data & pressure from BME688
        Serial.println(String(scd4xTemperature) + " " + String(scd4xHumidity) + " " + String(co2)); //main serial console
        Serial1.println(String(scd4xTemperature) + " " + String(scd4xHumidity) + " " + String(co2)); //UART port

        //Prints all BME688 data 
        static float alt = altitude(pressure);
        Serial.print(String(temp / 100) + "." + String(temp % 100) + " ");            //main serial console
        Serial.print(String(humidity / 1000) + "." + String(humidity % 1000) + " ");
        Serial.print(String(pressure / 100) + "." + String(pressure % 100) + " ");
        Serial.print(String(alt, 2) + " ");
        Serial.println(String(gas  / 100) + "." + String(gas % 100));
        
        Serial1.print(String(temp / 100) + "." + String(temp % 100) + " ");           //UART serial port
        Serial1.print(String(humidity / 1000) + "." + String(humidity % 1000) + " ");
        Serial1.print(String(pressure / 100) + "." + String(pressure % 100) + " "); 
        Serial1.print(String(alt, 2) + " ");
        Serial1.println(String(gas  / 100) + "." + String(gas % 100) + " ");
      }
    }
  }
  if (updateSerial) {
    rainTotal();
    delay(10);
  }
}

//SCD41 setup stuff
void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

//more SCD41 setup stuff
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}

//prints rain gauge data
void rainTotal() {
  noInterrupts(); // Temporarily disable interrupts to safely read 'total'
  float currentRain = total * 0.01;
  updateSerial = false; // Reset the flag
  interrupts(); // Re-enable interrupts

  Serial.println(String(currentRain));
  Serial1.println(String(currentRain));
}

//Counts up for how many times rain gauge tips
void count() {
  total += 1;
  updateSerial = true;
}

//function to calculate altitude (may need to be modified)
float altitude(const int32_t press, const float seaLevel) {
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}

// Function to calculate RMS of an array of float values
float calculateRMS(float* data, int numSamples) {
  float sumOfSquares = 0.0;
  
  for (int i = 0; i < numSamples; i++) {
    sumOfSquares += data[i] * data[i];
  }
  
  float meanSquare = sumOfSquares / numSamples;
  return sqrt(meanSquare);
}