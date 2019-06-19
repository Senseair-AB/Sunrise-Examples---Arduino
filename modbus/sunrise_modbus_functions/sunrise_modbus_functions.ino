/*
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_modbus_functions.ino
 * @brief       Example functions to perform different operations based on the
 *              "Modbus on Senseair Sunrise" documentation
 * @details     Should work fine on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.04
 * @date        2019-06-19
 *
 *******************************************************************************
 */

#include <SoftwareSerial.h>

/* Define serial pins for communication */
#define SUNRISE_TX 12
#define SUNRISE_RX 11
SoftwareSerial SunriseSerial = SoftwareSerial(SUNRISE_TX, SUNRISE_RX);

/* Reading period */
const int     MIN_READ_PERIOD_MS      = 2000;
/* Sunrise communication address, both for Modbus and I2C */
const uint8_t SUNRISE_ADDR           = 0x68;

/*********************************************************************************************/
/*********************************************************************************************/
/*********************************************************************************************/

/* Read Holding Register, Function code: 0x03 */
uint16_t value = 0;

uint16_t readHoldingRegister(uint8_t comAddr, uint8_t regAddr) {
  byte modbusPDU[] = {comAddr, 0x03, 0x00, regAddr, 0x00, 0x01};

  /* Create CRC */
  uint16_t crc = generateCRC(modbusPDU, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  byte modbusSerialPDU[] = {comAddr, 0x03, 0x00, regAddr, 0x00, 0x01, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, 8);

  /* Wait for response */
  int timeout = 0;
  while(SunriseSerial.available() < 6) {
    delay(1);
    timeout++;

    if(timeout > 180) {
      Serial.println("Response time-out");
      return value;      
    }
  }

  delay(50);
  
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  byte response [responseSize];
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check for corrupt data in the response */
  crc = generateCRC(response, (responseSize - 2));
  crcHi = (crc >> 8);
  crcLo = crc & 0xFF;
  

  if(crcLo != response[responseSize - 2] || crcHi != response[responseSize - 1]) {
    Serial.println("Error: Corrupted data. CRC in response did not match the rest of the PDU.");
    return value;
  }

  /* Check response for exceptions */
  if(response[1] == 0x83) {
    switch (response[2]) {
      case 0x01:
        Serial.println("Exception Code: Illegal Function");
        return value;
      case 0x02:
        Serial.println("Exception Code: Illegal Data Address");
        return value;
      case 0x03:
        Serial.println("Exception Code: Illegal Data Value");
        return value;
    }
  }

  /* Combine the bytes containing the requested values into a word */
  value = ((int16_t)(int8_t) response[3] << 8) | (uint16_t)response[4];
  
  return value;
}

/* Read Input Register, Function code: 0x04 */
uint16_t readInputRegister(uint8_t comAddr, uint8_t regAddr) {
  uint16_t value = 0;
  
  byte modbusPDU[] = {comAddr, 0x04, 0x00, regAddr, 0x00, 0x01};

  /* Create CRC */
  uint16_t crc = generateCRC(modbusPDU, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  byte modbusSerialPDU[] = {comAddr, 0x04, 0x00, regAddr, 0x00, 0x01, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, 8);

  /* Wait for response */
  int timeout = 0;
  while(SunriseSerial.available() < 6) {
    delay(1);
    timeout++;

    if(timeout > 180) {
      Serial.println("Response time-out");
      return value;
    }
  }

  /* To prevent corrupt data */
  delay(50);

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  byte response [responseSize];
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check for corrupt data in the response */
  crc = generateCRC(response, (responseSize - 2));
  crcHi = (crc >> 8);
  crcLo = crc & 0xFF;
  

  if(crcLo != response[responseSize - 2] || crcHi != response[responseSize - 1]) {
    Serial.println("Error: Corrupted data. CRC in response did not match the rest of the PDU.");
    return value;
  }


  /* Check response for exceptions */
  if(response[1] == 0x84) {
    switch (response[2]) {
      case 0x01:
        Serial.println("Exception Code: Illegal Function");
        return value;
      case 0x02:
        Serial.println("Exception Code: Illegal Data Address");
        return value;
      case 0x03:
        Serial.println("Exception Code: Illegal Data Value");
        return value;
    }
  }

  /* Combine the bytes containing the requested values into a word */
  value = ((int16_t)(int8_t) response[3] << 8) | (uint16_t)response[4];
  
  return value;
}


/* Write Multiple Registers 0x10 */
void writeToRegister(uint8_t comAddr, uint8_t regAddr, uint16_t writeVal) {
  uint8_t writeValHi = (writeVal >> 8);
  uint8_t writeValLo = writeVal & 0xFF;
  
  byte modbusPDU[] = {comAddr, 0x10, 0x00, regAddr, 0x00, 0x01, 0x02, writeValHi, writeValLo};

  /* Create CRC */
  uint16_t crc = generateCRC(modbusPDU, 9);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  byte modbusSerialPDU[] = {comAddr, 0x10, 0x00, regAddr, 0x00, 0x01, 0x02 , writeValHi, writeValLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, 11);

  /* Wait for response */
  int timeout = 0;
  while(SunriseSerial.available() < 8) {
    delay(1);
    timeout++;

    if(timeout > 180) {
      Serial.println("Response time-out");
      return;      
    }
  }

  delay(100);

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  byte response [responseSize];
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }  
  
  /* Check for corrupt data in the response */
  crc = generateCRC(response, (responseSize - 2));
  crcHi = (crc >> 8);
  crcLo = crc & 0xFF;
  

  if(crcLo != response[responseSize - 2] || crcHi != response[responseSize - 1]) {
    Serial.println("Error: Corrupted data. CRC in response did not match the rest of the PDU.");
    return;
  }

  /* Check response for exceptions */
  if(response[1] == 0x90) {
    switch (response[2]) {
      case 0x01:
        Serial.println("Exception Code: Illegal Function");
        return;
      case 0x02:
        Serial.println("Exception Code: Illegal Data Address");
        return;
      case 0x03:
        Serial.println("Exception Code: Illegal Data Value");
        return;
    }
  }
}

/* Generate CRC for any given request array */
uint16_t generateCRC(byte pdu[], int len)
{
  uint16_t crc = 0xFFFF;
  
  for(int pos = 0 ; pos < len ; pos++) {
    /* XOR the byte into the least significant byte of crc */
    crc ^= (uint16_t)pdu[pos];

    /* Loop through the entire message */
    for(int n = 8 ; n != 0 ; n--) {
      /* If the LSB is 1, shift right and XOR 0xA001 */
      /* Otherwise, just shift right */
      if((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
        
      }else {
        crc >>= 1;
      }
    }
  }
  return crc;  
}

/*********************************************************************************************/
/*********************************************************************************************/
/*********************************************************************************************/

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  /* Begin serial communication */
  Serial.begin(9600);
  SunriseSerial.begin(9600);

  while(SunriseSerial.available() > 0) {
    SunriseSerial.read();
  }
  
  Serial.println("Initialization complete");

  readSensorConfig(SUNRISE_ADDR);

  delay(MIN_READ_PERIOD_MS);

  /* Change measurement mode */
  //changeMeasurementMode(SUNRISE_ADDR);
}

/*********************************************************************************************/
/*********************************************************************************************/
/*********************************************************************************************/

/* Read the sensor's measurement configurations and print them */
void readSensorConfig(uint8_t target) {
  /* Read measurement mode (0 = Continuous, 1 = Single) */
  uint16_t measMode = readHoldingRegister(SUNRISE_ADDR, 0x0A);
  Serial.print("Measurement Mode: ");
  Serial.println(measMode);

  /* Read measurement period */
  uint16_t measPeriod = readHoldingRegister(SUNRISE_ADDR, 0x0B);
  Serial.print("Measurement Period: ");
  Serial.print(measPeriod);
  Serial.println(" sec");

  /* Read measurement samples */
  uint16_t measSamples = readHoldingRegister(SUNRISE_ADDR, 0x0C);
  Serial.print("Measurement Samples: ");
  Serial.println(measSamples);
}


/* Read the sensor's current CO2 value and Error Status and print them */
void readSensorMeasurement(uint8_t target) {
  /* Read CO2 value */
  uint16_t co2Value = readInputRegister(target, 0x03);
  Serial.print("CO2: ");
  Serial.print(co2Value);
  Serial.println(" ppm");

  /* Read error status */
  uint16_t eStatus = readInputRegister(target, 0x00);
  Serial.print("Error Status: 0x");
  Serial.println(eStatus, HEX);
}


/* Change measurement mode */
void changeMeasurementMode(uint8_t target) {
  if(readHoldingRegister(SUNRISE_ADDR, 0x0A) == 0) {
    writeToRegister(target, 0x0A, (uint16_t)0x0001);
    Serial.println("Measurement mode changed to Single Mode");
  }else if(readHoldingRegister(SUNRISE_ADDR, 0x0A) == 1) {
    writeToRegister(target, 0x0A, (uint16_t)0x0000);
    Serial.println("Measurement mode changed to Continuous Mode");
  } 
}

/*********************************************************************************************/
/*********************************************************************************************/
/*********************************************************************************************/

/* Main loop */
void loop()
{
  static int pin_value = HIGH;
  
  /* Delay between readings */
  Serial.println("Waiting...");
  delay(MIN_READ_PERIOD_MS);

  /* Read measurements */
  readSensorMeasurement(SUNRISE_ADDR);
  
  /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}
