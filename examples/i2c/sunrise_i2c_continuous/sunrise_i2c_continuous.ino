/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_i2c_continuous.ino
 * @brief       Example functions to perform the different operations 
 *              descrived in the "I2C on Senseair Sunrise" documentation 
 *              (available on the www.senseair.com website). This example mainly 
 *              covers operations in continuous measurement mode.
 * @details     Tested on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.05
 * @date        2019-08-13
 * 
 *******************************************************************************
 */

#include <Wire.h>

/* Sunrise communication address, both for Modbus and I2C */
const uint8_t   SUNRISE_ADDR            = 0x68;

/* Amount of wakeup attempts before time-out */
const int       ATTEMPTS                 = 5;

/* Register Addresses */
const uint8_t ERROR_STATUS             = 0x01;
const uint8_t MEASUREMENT_MODE         = 0x95;

/* Measurement modes */
const uint16_t CONTINUOUS               = 0x0000;
const uint16_t SINGLE                   = 0x0001;

/* Reading period, in milliseconds. Default is 4 seconds */
int readPeriodMs = 4000;

/* Initialize I2C bus and pins */
void  reInitI2C() {
  /* Initialize I2C and use default pins defined for the board */
  Wire.begin();
  /* Setup I2C clock to 100kHz */
  Wire.setClock(100000);  
}


/** 
 * @brief  Wakes up the sensor by initializing a write operation
 *         with no data.
 * 
 * @param  target:      The sensor's communication address
 * @note   This example shows a simple way to wake up the sensor.
 * @retval true if successful, false if failed
 */
bool _wakeup(uint8_t target)
{
  int attemps = ATTEMPTS;
  int error;
 
  do {
    uint8_t byte_0;    
    /* */
    Wire.beginTransmission(target);
    error = Wire.endTransmission(true);
  } while(((error != 0 /*success */) && (error != 2 /*Received NACK on transmit of address*/) && (error != 1 /* BUG in STM32 library*/)) && (--attemps > 0)); 
  /* STM32 driver can stack under some conditions */
  if(error == 4) {
    /* Reinitialize I2C*/
    reInitI2C();
    return false;
  } 
  return (attemps > 0);
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
  reInitI2C();
 
  Serial.begin(115200);

  Serial.println("Initialization complete\n");

  /* Read the sensor's configs */
  Serial.println("Sensor Measurement Configurations:");
  read_sensor_config(SUNRISE_ADDR);
  Serial.println();

  /* Change measurement mode if single */
  change_measurement_mode(SUNRISE_ADDR);

  delay(readPeriodMs);
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
  int numBytes = 5;

  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    return;
  }

  /* Request values */
  error = Wire.requestFrom((uint8_t)target, (uint8_t)numBytes /* how many bytes */, (uint32_t)MEASUREMENT_MODE /* from address*/, (uint8_t)1/* Address size - 1 byte*/, true /* STOP*/);    
  if(error != numBytes ) {
    Serial.print("Failed to write to target. Error code : ");
    Serial.println(error);
    return;
  }

  /* Read values */
  /* Measurement mode */
  uint8_t measMode = Wire.read();

  /* Measurement period */
  uint8_t byteHi = Wire.read();
  uint8_t byteLo = Wire.read();
  uint16_t measPeriod = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;

  /* Number of samples */
  byteHi = Wire.read();
  byteLo = Wire.read();
  uint16_t numSamples = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;

  Serial.print("Measurement Mode: ");
  Serial.println(measMode);
  readPeriodMs = measPeriod * 1000;

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
  int numBytes = 1;
  
  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    /* FATAL ERROR */
    while(true);
  }

  /* Read Value */
  error = Wire.requestFrom((uint8_t)target, (uint8_t)numBytes /* how many bytes */, (uint32_t)MEASUREMENT_MODE /* from address*/, (uint8_t)1/* Address size - 1 byte*/, true /* STOP*/);    
  if(error != numBytes ) {  
    Serial.print("Failed to read measurement mode. Error code: ");
    Serial.println(error);
    /* FATAL ERROR */
    while(true);
  }

  /* Change mode if single */
  if(Wire.read() != CONTINUOUS) {
    /* Wakeup */
    if(!(_wakeup(target))) {
      Serial.print("Failed to wake up sensor.");
      /* FATAL ERROR */
      while(true);
    }
    
    Serial.println("Changing Measurement Mode to Continuous...");
    
    Wire.beginTransmission(target);
    Wire.write(MEASUREMENT_MODE);
    Wire.write(CONTINUOUS);
    error = Wire.endTransmission(true);
    
    if(error != 0) {
      Serial.print("Failed to send request. Error code: ");
      Serial.println(error); 
      /* FATAL ERROR */
      while(true);
    }
    Serial.println("Sensor restart is required to apply changes");
    while(true);
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
  int numBytes = 7;

  /* Wakeup */
  if(!(_wakeup(target))) {
    Serial.print("Failed to wake up sensor.");
    return;
  }

  /* Request values */
    error = Wire.requestFrom((uint8_t)target, (uint8_t)numBytes /* how many bytes */, (uint32_t)ERROR_STATUS /* from address*/, (uint8_t)1/* Address size - 1 byte*/, true /* STOP*/);    
  if(error != numBytes ) {  
    Serial.print("Failed to read values. Error code: ");
    Serial.println(error);
    return;
  }

  /* Read values */
  /* Error status */
  uint8_t eStatus = Wire.read();

  /* Reserved */
  uint8_t byteHi = Wire.read();
  uint8_t byteLo = Wire.read();

  byteHi = Wire.read();
  byteLo = Wire.read();

  /* CO2 value */
  byteHi = Wire.read();
  byteLo = Wire.read();
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
  delay(readPeriodMs);

 /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}
