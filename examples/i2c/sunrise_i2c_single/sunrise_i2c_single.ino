/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_i2c_single.ino
 * @brief       Example functions to perform different the different operations 
 *              descrived in the "I2C on Senseair Sunrise" documentation
 *              (available on the www.senseair.com website). This example mainly 
 *              covers operations in single measurement mode.
 * @details     Tested on Arduino Mega 2560     
 *              
 * @author      William Sandkvist
 * @version     0.05
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

/* Reading period, in milliseconds. Default is 4 seconds */
int readPeriod = 4000;

/* 
 * Variable for keeping track of time passed, in hours, since 
 * last ABC calibration.
 */
unsigned long int abc = 0;

/* Array for storing sensor state data */
uint8_t state[24];

/** 
 * @brief  Wakes up the sensor by initializing a write operation
 *         with no data.
 * 
 * @param  target:      The sensor's communication address
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

  delay(readPeriod);
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
 * @param  target:      The sensor's communication address
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

  /* Change mode if continuous */
  if(I2c.receive() == continuous) {
    /* Wakeup */
    if(error = _wakeup(target) != 32 && error != 0) {
      Serial.print("Failed to wake up sensor. Error code: ");
      Serial.println(error);
      while(true) {
        delay(600000);
      }
    }
    
    Serial.println("Changing Measurement Mode to Single...");
    if(error = I2c.write(target, reg, single) != 0) {
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

  uint8_t regAddr = 0xC4;
  int numReg = 24;

  /* Drive EN pin HIGH */
  digitalWrite(SUNRISE_EN, HIGH);

  /* Wait for sensor start-up and stabilization */
  delay(35);


  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    return;
  }

  /* Request state data */
  if(I2c.read(target, regAddr, numReg) != 0) {
    Serial.print("Failed to read measurements command. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }

  /* Read and save state data */
  for(int n = 0 ; n < 24 ; n++) {
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

  uint8_t regAddrCmd = 0xC3;
  int numRegCmd = 25;

  uint8_t regAddrRead = 0x01;
  int numRegRead = 7;

  uint8_t regAddrState = 0xC4;
  int numRegState = 24;

  uint8_t cmdArray[25];

  cmdArray[0] = 0x01;

  for(int n = 1 ; n < 25 ; n++) {
    cmdArray[n] = state[n-1];
  }

  /* Drive EN pin HIGH */
  digitalWrite(SUNRISE_EN, HIGH);

  /* Wait for sensor start-up and stabilization */
  delay(35);

  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    return;
  }

  /* Write measurement command and sensor state to 0xC3 */
  if(error = I2c.write(target, regAddrCmd, cmdArray, numRegCmd) != 0) {
    Serial.print("Failed to send measurement command. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }

  /* Wait until ready pin goes low */
  delay(2000);

  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    return;
  }

  /* Request values */
  if(I2c.read(target, regAddrRead, numRegRead) != 0) {
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

  Serial.print("CO2: ");
  Serial.print(co2Val);
  Serial.println(" ppm");

  Serial.print("Error Status: 0x");
  Serial.println(eStatus, HEX);

  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    return;
  }

  /* Read sensor state data from 0xC4-0xDB and save it for next measurement */
  if(I2c.read(target, regAddrState, numRegState) != 0) {
    Serial.print("Failed to read measurements command. Error code: ");
    Serial.println(error);
    digitalWrite(SUNRISE_EN, LOW);
    return;
  }
  
  for(int n = 0 ; n < 24 ; n++) {
    state[n] = I2c.receive();
  }

  /* Drive EN pin LOW */
  digitalWrite(SUNRISE_EN, LOW);

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
  int error;
  
  uint8_t regAddr = 0x88;
  int numReg = 2;

  /* Wakeup */
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    abc = 3600000;
    return;
  }

  /* Read current ABC Time value from 0x88 - 0x89 */
  if(I2c.read(target, regAddr, numReg) != 0) {
    Serial.print("Failed to send read request. Error code: ");
    Serial.println(error);
    abc = 3600000;
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
  if(error = _wakeup(target) != 32 && error != 0) {
    Serial.print("Failed to wake up sensor. Error code: ");
    Serial.println(error);
    abc = 3600000;
    return;
  }

  /* Write new value back to HR35 */
  if(error = I2c.write(target, regAddr, newAbc, numReg) != 0) {
    Serial.print("Failed to write to register. Error code: ");
    Serial.println(error);
    abc = 3600000;
    return;
  }

  Serial.println("\n ABC Time updated\n");
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

  /* 
   *  The read function takes about 2090 ms to run. So the abc
   *  variable has to be increased by  2 + readPeriod ms.  
   */
  abc += (readPeriod + 2090);

  /* 
   *  When abc value has reached 3 600 000  
   *  HR35 has to be increased by 1.
   */
  if(abc >= 3600000) {
    abc = 0;
    increase_abc(SUNRISE_ADDR);
  }

 /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}