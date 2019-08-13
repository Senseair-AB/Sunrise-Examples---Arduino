/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_i2c_single.ino
 * @brief       Example for reading sensor data in single measurement mode.
 *              Only for revision.
 *              
 *              Based on the "I2C on Senseair Sunrise" documentation 
 *              (available on the www.senseair.com website). This example mainly
 *              covers operations in single measurement mode.
 * @details     Tested on Arduino Mega 2560     
 *              
 * @author      William Sandkvist
 * @version     0.06
 * @date        2019-08-12
 *
 *******************************************************************************
 */

#include <I2C.h>
#include <SoftwareSerial.h>

/* Define serial EN pin */
const int       SUNRISE_EN              = 8;

/* Sunrise communication address, both for Modbus and I2C */
const uint8_t   SUNRISE_ADDR            = 0x68;

/* Amount of wakeup attempts before time-out */
const int       ATTEMPTS                 = 50;

/* Register Addresses */
const uint8_t ERROR_STATUS             = 0x01;
const uint8_t MEASUREMENT_MODE         = 0x95;
const uint8_t START_MEASUREMENT        = 0xC3;
const uint8_t ABC_TIME                 = 0xC4;

/* Measurement modes */
const uint16_t CONTINUOUS               = 0x0000;
const uint16_t SINGLE                   = 0x0001;

/* Delays in milliseconds*/
const int STABILIZATION_MS              = 35;
const int WAIT_FOR_PIN_MS               = 2000;

/* Reading period, in milliseconds. Default is 4 seconds */
int readPeriodMs = 4000;

/* 
 * Variable for keeping track of how many hours the sensor
 * has been running. Used for increasing the ABC Time register.
 */
int abc = 1;

/* Array for storing sensor state data */
uint8_t state[24];

/** 
 * @brief  Wakes up the sensor by initializing a write operation
 *         with no data.
 * 
 * @param  target:      The sensor's communication address
 * @note   This example shows a simple way to wake up the sensor.
 * @retval true if successful, false if failed
 */
bool _wakeup(uint8_t target) {
  int counter = 0;

  /* On success the write function will return 0 */
  while(I2c.write(target, 0) != 0) {
    counter++;
    if(counter == ATTEMPTS) {
      return false;
    }
  }
  return true;
}

/**
 * @brief  This function runs once at the start.
 *
 * @retval None
 */
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  /*
   * Currently the program only works if the EN pin is defined as INPUT, 
   * even though it should be defined as OUTPUT
   */
  pinMode(SUNRISE_EN, INPUT);
  
  /* I2C */
  /* Initialize I2C and use default pins defined for the board */
  I2c.begin();

  /*  
   * Set I2C clock to 100kHz 
   * Based on Data rate from documentation
   */
  I2c.setSpeed(0);  
  Serial.begin(115200);

  Serial.println("Initialization complete\n");

  /* Read the sensor's configs */
  Serial.println("Sensor Measurement Configurations:");
  read_sensor_config(SUNRISE_ADDR);
  Serial.println();

  /* Change measurement mode if continuous */
  change_measurement_mode(SUNRISE_ADDR);

  /* Initial measurement */
  Serial.println("Saving Sensor State");
  save_state(SUNRISE_ADDR);

  delay(readPeriodMs);
}

/**
 * @brief  Reads and prints the sensor's current measurement mode,
 *         measurement period and number of samples.
 * 
 * @param  target:      The sensor's communication address
 * @note   This example shows a simple way to read the sensor's
 *         measurement configurations.
 * @retval None
 */
void read_sensor_config(uint8_t target) {
  /* Function variables */
  int error;
  int numBytes = 5;

  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    return;
  }

  /* Request values */
  if((error = I2c.read(target, MEASUREMENT_MODE, numBytes)) != 0) {
    Serial.print("Failed to write to target. Error code : ");
    Serial.println(error);
    return;
  }

  /* Read values */
  /* Measurement mode */
  uint8_t measMode = I2c.receive();

  /* Measurement period */
  uint8_t byteHi = I2c.receive();
  uint8_t byteLo = I2c.receive();
  uint16_t measPeriod = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;
  
  /* Number of samples */
  byteHi = I2c.receive();
  byteLo = I2c.receive();
  uint16_t numSamples = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;

  Serial.print("Measurement Mode: ");
  Serial.println(measMode);

  Serial.print("Measurement Period: ");
  Serial.println(measPeriod);
  readPeriodMs = measPeriod * 1000;

  Serial.print("Number of Samples: ");
  Serial.println(numSamples);

  
}

/**
 * @brief  Changes the sensor's current measurement mode, if it's
 *         currently in single mode. 
 * 
 * @param  target:      The sensor's communication address
 * @note   This example shows a simple way to change the sensor's
 *         measurement mode. The sensor has to be manually restarted after the
 *         changes.
 * @retval None
 */
void change_measurement_mode(uint8_t target) {
  /* Function variables */
  int error;
  int numReg = 1;
  
  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    /* FATAL ERROR */
    while(true);
  }

  /* Read Value */
  if(error = I2c.read(target, MEASUREMENT_MODE, numReg) != 0) {
    Serial.print("Failed to read measurement mode. Error code: ");
    Serial.println(error);
    /* FATAL ERROR */
    while(true);
  }

  /* Change mode if continuous */
  if(I2c.receive() != SINGLE) {
    /* Wakeup */
    if(_wakeup(target)) {
      Serial.print("Failed to wake up sensor.");
      /* FATAL ERROR */
      while(true);
   }
    
    Serial.println("Changing Measurement Mode to Single...");
    if((error = I2c.write(target, MEASUREMENT_MODE, SINGLE)) != 0) {
      Serial.print("Failed to send request. Error code: ");
      Serial.println(error); 
      /* FATAL ERROR */
      while(true);
    }
    Serial.println("Sensor restart is required to apply changes");
    /* FATAL ERROR */
    while(true);
  }
}

/**
 * @brief  Reads the sensors current state data.
 * 
 * @param  target:      The sensor's communication address
 * @note   If host device has no state data, it is very important 
 *         that host do not write '0' to address 0xC2 - 0xDB the 
 *         first time it starts a measurement.
 * @retval None
 */
void save_state(uint8_t target) {
  /* Function variables */
  int error;

  int numReg = 24;

  /* Drive EN pin HIGH */
  digitalWrite(SUNRISE_EN, HIGH);

  /* Wait for sensor start-up and stabilization */
  delay(STABILIZATION_MS);


  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    /* FATAL ERROR */
    digitalWrite(SUNRISE_EN, LOW);
    while(true);
  }

  /* Request state data */
  if((error =I2c.read(target, ABC_TIME, numReg)) != 0) {
    Serial.print("Failed to read measurements command. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    /* FATAL ERROR */
    while(true);
  }

  /* Read and save state data */
  for(int n = 0 ; n < numReg ; n++) {
    state[n] = I2c.receive();
  }

  /* Drive EN pin LOW */
  digitalWrite(SUNRISE_EN, LOW);

  Serial.println("Saved Sensor State Successfully\n");
}

/**
 * @brief  Reads and prints the sensor's current CO2 value and
 *         error status.
 * 
 * @param  target:      The sensor's communication address
 * @note   This example shows a simple way to read the sensor's
 *         CO2 measurement and error status.
 * @retval None
 */
void read_sensor_measurements(uint8_t target) {
  /* Function variables */
  int error;

  int numRegCmd = 25;
  int numRegRead = 7;
  int numRegState = 24;

  uint8_t cmdArray[25];

  cmdArray[0] = 0x01;

  for(int n = 1 ; n < numRegCmd ; n++) {
    cmdArray[n] = state[n-1];
  }

  /* Drive EN pin HIGH */
  digitalWrite(SUNRISE_EN, HIGH);

  /* Wait for sensor start-up and stabilization */
  delay(STABILIZATION_MS);

  /* Wakeup */
  if(!_wakeup(target)) {
    Serial.print("Failed to wake up sensor.");
    return;
  }

  /* Write measurement command and sensor state to 0xC3 */
  if((error = I2c.write(target, START_MEASUREMENT, cmdArray, numRegCmd)) != 0) {
    Serial.print("Failed to send measurement command. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }

  /* Wait until ready pin goes low */
  delay(WAIT_FOR_PIN_MS);

  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }

  /* Request values */
  if(I2c.read(target, ERROR_STATUS, numRegRead) != 0) {
    Serial.print("Failed to read measurements command. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }

  /* Read values */
  /* Error Status */
  uint8_t eStatus = I2c.receive();

  /* Reserved */
  uint8_t byteHi = I2c.receive();
  uint8_t byteLo = I2c.receive();

  byteHi = I2c.receive();
  byteLo = I2c.receive();

  /* CO2 value */
  byteHi = I2c.receive();
  byteLo = I2c.receive();
  uint16_t co2Val = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;

  /* Wakeup */
  if(!_wakeup(target)) {
    Serial.print("Failed to wake up sensor.");
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }

  /* Read sensor state data from 0xC4-0xDB and save it for next measurement */
  if(I2c.read(target, ABC_TIME, numRegState) != 0) {
    Serial.print("Failed to read measurements command. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }
  
  for(int n = 0 ; n < numRegState ; n++) {
    state[n] = I2c.receive();
  }

  /* Drive EN pin LOW */
  digitalWrite(SUNRISE_EN, LOW);

  /* Print values */
  Serial.print("CO2: ");
  Serial.print(co2Val);
  Serial.println(" ppm");

  Serial.print("Error Status: 0x");
  Serial.println(eStatus, HEX);
  Serial.println();
}

/**
 * @brief  Increases the ABC Time by one.
 * 
 * @param  target: The sensor's communication address
 * @note   This example shows a simple way to 
 *         increase the sensor's ABC Time
 * @retval None
 */
int increase_abc(uint8_t target) {
  /* Function variables */
  int error = 0;
  int numReg = 2;

  /*Drive EN pin HIGH */
  digitalWrite(SUNRISE_EN, HIGH);

  /* Wait for sensor start-up and stabilization */
  delay(STABILIZATION_MS);

  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    digitalWrite(SUNRISE_EN, LOW);
    abc--;
    return;
  }

  /* Read current ABC Time value from 0x88 - 0x89 */
  if((error = I2c.read(target, ABC_TIME, numReg)) != 0) {
    Serial.print("Failed to send read request. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    abc--;
    return;
  }

  /* Read value */
  uint8_t byteHi = I2c.receive();
  uint8_t byteLo = I2c.receive();
  uint16_t abcTime = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;

  /* Increase current ABC Time by one */
  abcTime++;

  uint8_t abcHi = (abcTime >> 8);
  uint8_t abcLo = abcTime & 0xFF;

  uint8_t newAbc[] = {abcHi, abcLo};

  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    digitalWrite(SUNRISE_EN, LOW);
    abc--;
    return;
  }

  /* Write new value back to HR35 */
  if((error = I2c.write(target, ABC_TIME, newAbc, numReg)) != 0) {
    Serial.print("Failed to write to register. Error code: ");
    Serial.println(error);
    abc--;
    return;
  }

  /* Drive EN pin low */
  digitalWrite(SUNRISE_EN, LOW);

  Serial.println("\nABC Time updated\n");
}

/**
 * @brief  The main function loop. Reads the sensor's current
 *         CO2 value and error status and prints them to the 
 *         Serial Monitor.
 * 
 * @retval None
 */
void loop() {
  static int pin_value = HIGH;

  /* Read measurements */
  read_sensor_measurements(SUNRISE_ADDR);

  /* Delay between readings */
  Serial.println("\nWaiting...\n");
  delay(readPeriodMs);

  /* When an hour has passed, increase ABC Time */  
  if((abc*3600000) <= millis()) {
    increase_abc(SUNRISE_ADDR);
    abc++;
  }

 /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}