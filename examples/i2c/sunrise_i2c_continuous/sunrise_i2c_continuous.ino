/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_i2c_continuous.ino
 * @brief       Example functions to perform different the different operations 
 *              descrived in the "I2C on Senseair Sunrise" documentation 
 *              (available on the www.senseair.com website). This example mainly 
 *              covers operations in continuous measurement mode.
 * @details     Tested on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.02
 * @date        2019-08-09
 * 
 *******************************************************************************
 */

#include <I2C.h>

/* Sunrise communication address, both for Modbus and I2C */
const uint8_t   SUNRISE_ADDR            = 0x68;

/*
 * Amount of wakeup attempts before time-out
 */
const int       ATTEMPTS                 = 50;

/* Reading period, in milliseconds. Default is 4 seconds */
int readPeriod = 4000;

/** 
 * @brief  Wakes up the sensor by initializing a write operation
 *         with no data.
 * 
 * @param  target: The sensor's communication address
 * @note   This example shows a simple way to wake up the sensor.
 * @retval Error code encountered
 */
int _wakeup(uint8_t target) {
  int error;
  int counter = 0;

  /* On success the write function will return either 32 or 0 */
  while(error = I2c.write(target, target) != 32 && error != 0) {
    counter++;
    if(counter == ATTEMPTS) {
      return error;
    }
  }
  return error;
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

  /* I2C */
  /* Initialize I2C and use default pins defined for the board */
  I2c.begin();

  /* Set I2C clock to 100kHz */
  I2c.setSpeed(0);  
  Serial.begin(115200);

  Serial.println("Initialization complete\n");

  /* Read the sensor's configs */
  Serial.println("Sensor Measurement Configurations:");
  read_sensor_config(SUNRISE_ADDR);
  Serial.println();

  /* Change measurement mode if single */
  change_measurement_mode(SUNRISE_ADDR);

  delay(readPeriod);
}

/**
 * @brief  Reads and prints the sensor's current measurement mode,
 *         measurement period and number of samples.
 * 
 * @param  target: The sensor's communication address
 * @note   This example shows a simple way to read the sensor's
 *         measurement configurations.
 * @retval None
 */
void read_sensor_config(uint8_t target) {
  /* Function variables */
  int error;
  uint8_t reg = 0x95;
  int numBytes = 5;
  int counter = 0;

  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    return;
  }

  /* Request values */
  if(error = I2c.read(target, reg, numBytes) != 0) {
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

  Serial.print("Number of Samples: ");
  Serial.println(numSamples);  
}

/**
 * @brief  Changes the sensor's current measurement mode, if it's
 *         currently in single mode. 
 * 
 * @param  target: The sensor's communication address
 * @note   This example shows a simple way to change the sensor's
 *         measurement mode. The sensor has to be manually restarted after the
 *         changes.
 * @retval None
 */
void change_measurement_mode(uint8_t target) {
  /* Function variables */
  int error;
  uint8_t reg = 0x95;
  uint8_t continuous = 0;
  uint8_t single = 1;
  
  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    while(true) {
      delay(600000);
    }
  }

  /* Read Value */
  if(error = I2c.read(target, reg, 1) != 0) {
    Serial.print("Failed to read measurement mode. Error code: ");
    Serial.println(error);
    while(true) {
      delay(600000);
    }
  }

  /* Change mode if single */
  if(I2c.receive() == single) {
    /* Wakeup */
    if(error = _wakeup(target) != 32 && error != 0) {
      Serial.print("Failed to wake up sensor. Error code: ");
      Serial.println(error);
      while(true) {
        delay(600000);
      }
    }
    
    Serial.println("Changing Measurement Mode to Continuous...");
    if(error = I2c.write(target, reg, continuous) != 0) {
      Serial.print("Failed to send request. Error code: ");
      Serial.println(error); 
      while(true){
        delay(600000);
      }
    }
    Serial.println("Sensor restart is required to apply changes");
    while(true){
      delay(600000);
    }
  }
}

/**
 * @brief  Reads and prints the sensor's current CO2 value and
 *         error status.
 * 
 * @param  target: The sensor's communication address
 * @note   This example shows a simple way to read the sensor's
 *         CO2 measurement and error status.
 * @retval None
 */
void read_sensor_measurements(uint8_t target) {
  /* Function variables */ 
  int error;
  uint8_t reg = 0x01;
  int numBytes = 7;
  int counter = 0;

  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    return;
  }

  /* Request values */
  if(error = I2c.read(target, reg, numBytes) != 0) {
    Serial.print("Failed to read values. Error code: ");
    Serial.println(error);
    return;
  }

  /* Read values */
  /* Error status */
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

  Serial.print("CO2: ");
  Serial.print(co2Val);
  Serial.println(" ppm");

  Serial.print("Error Status: 0x");
  Serial.println(eStatus, HEX);
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
  delay(readPeriod);

 /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}