/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_i2c_rev2.ino
 * @brief       Simple example to show how to comminicate with Senseair Sunrise sensor (rev.2) using I2C bus 
 * @details     Should work fine on Arduino Uno, Arduino Mega, Nucleo 32 STM32L432 
 *              
 * @author      Dmitrii Rudakov
 * @version     0.01
 * @date        2019-03-06
 *
 *******************************************************************************
 */
#include <Wire.h>

/* Reading period */
const int     MIN_READ_PERIOD_MS    = 2000;
/* Sunrise communication address, both for Modbus and I2C */
const uint8_t SUNRISE_ADDR          = 0x68;
/* Default wake-up attemps */
const int     WAKEUP_ATTEMPS        = 1;

/* Sensor state */
int16_t   co2_value;    /* CO2 concentration, ppm*/
uint16_t  error_status; /* Errors */
/* Sensor configuration */
uint8_t   meas_mode;      /*0 - continiuos mode, 1 - single mode*/
uint16_t  meas_period_s;  /* measurement period, seconds */
uint16_t  meas_numbers;   /* number of measurements by measurement period */

/* Initialize I2C bus and pins */
void  reInitI2C() {
  /* Initialize I2C and use default pins defined for the board */
  Wire.begin();
  /* Setup I2C clock to 100kHz */
  Wire.setClock(100000);  
}

/* Setup I2C communications and other  */
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  /* I2C */
  reInitI2C();
  /**/
  Serial.begin(115200);
  /* Some delay to have time to read data */
  delay(500);
  Serial.println("Initialize complete");
  /* Read sensor configuration */
  if((wakeupSensor(SUNRISE_ADDR) > 0) && (readSensorConfiguration(SUNRISE_ADDR) > 0)) {
      /* Show results */
      Serial.print("MeasPeriod = ");
      Serial.print(meas_period_s);
      Serial.println(" sec");
      Serial.print("MeasNum = ");
      Serial.println(meas_numbers);
      Serial.print("MeasMode = ");
      Serial.println(meas_mode);
  } else {
    Serial.println("Communication failure, configuration reading failed!");
  }
}


/** 
 *  Wake up Sunrise sensor rev. 2, returns above 0 if succed 
 **/
int wakeupSensor(uint8_t target)
{
  int attemps = WAKEUP_ATTEMPS;
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
  } 
  return attemps;
}

/* Read sensor's configuration */
int readSensorConfiguration(uint8_t target) {
    int error = Wire.requestFrom((uint8_t)SUNRISE_ADDR, (uint8_t)5 /* how many bytes */, (uint32_t)0x95U /* from address*/, (uint8_t)1/* Address size - 1 byte*/, true /* STOP*/);    
    if(error >= 5 ) {
      uint8_t byte_hi, byte_lo;
      /* Measurement Mode*/
      meas_mode = Wire.read();
      /* Measurement period */
      byte_hi = Wire.read();
      byte_lo = Wire.read();
      meas_period_s  = (((uint16_t)(int8_t)byte_hi)<<8) | (uint16_t)byte_lo;
      /* Number of measurements */
      byte_hi = Wire.read();
      byte_lo = Wire.read();
      /* Convert bytes to word */
      meas_numbers =  (((int16_t)(int8_t)byte_hi)<<8) | (uint16_t)byte_lo;
    } else {
      error = -1;
    }
    /* Read any bytes if exists */
    while(Wire.available()) {
      Wire.read();
    }

    return (error);
}


/* Read sensor's data */
int readSensorData(uint8_t target) {
    /* Try to read ErrorStatus and CO2 concentration */
    int error = Wire.requestFrom((uint8_t)SUNRISE_ADDR, (uint8_t)8 /* how many bytes */, (uint32_t)0x00 /* from address*/, (uint8_t)1/* Address size - 1 byte*/, true /* STOP*/);    
    if(error >= 8 ) {
      uint8_t byte_hi, byte_lo;
      /* ErrorStatus*/
      byte_hi = Wire.read();
      byte_lo = Wire.read();
      /* Convert bytes to word */
      error_status =  (((int16_t)(int8_t)byte_hi)<<8) | (uint16_t)byte_lo;
      /* Reserved*/
      byte_hi = Wire.read();
      byte_lo = Wire.read();
      /* Reserved*/
      byte_hi = Wire.read();
      byte_lo = Wire.read();
      /* CO2 Value*/
      byte_hi = Wire.read();
      byte_lo = Wire.read();
      /* Convert bytes to word */
      co2_value =  (((int16_t)(int8_t)byte_hi)<<8) | (uint16_t)byte_lo;
    } else {
      error = -1;
    }
    /* Read any bytes if exists */
    while(Wire.available()) {
      Wire.read();
    }

    return (error);
}

/* Main loop */
void loop()
{
  static int pin_value = HIGH;
 
  Serial.println("Waiting some time...");
  delay(MIN_READ_PERIOD_MS);  

  if(wakeupSensor(SUNRISE_ADDR) > 0) {
    if(readSensorData(SUNRISE_ADDR) > 0) {
      /* Show results */
      Serial.print("CO2 = ");
      Serial.print(co2_value);
      Serial.print(" ppm (E: 0x");
      Serial.print(error_status, HEX);
      Serial.println(")");
    } else {
      Serial.println("Communication failure!");
    }
  } else {    
    Serial.println("Could not awake the sensor, will try next time...");
  }

  /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
  
}
