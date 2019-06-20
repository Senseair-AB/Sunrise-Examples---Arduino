/*
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_modbus_functions.ino
 * @brief       Example functions to perform different operations based on the
 *              "Modbus on Senseair Sunrise" documentation
 * @details     Should work fine on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.05
 * @date        2019-06-19
 *
 *******************************************************************************
 */

#include <SoftwareSerial.h>

/* Define serial pins for communication */
const int     SUNRISE_TX              = 12;
const int     SUNRISE_RX              = 11;
SoftwareSerial SunriseSerial = SoftwareSerial(SUNRISE_TX, SUNRISE_RX);

/* Reading period */
const int     MIN_READ_PERIOD_MS      = 2000;
/* Sunrise communication address, both for Modbus and I2C */
const uint8_t SUNRISE_ADDR            = 0x68;
/* Delay when waiting for responses, in milliseconds.
 * Based on the documentation, "Modbus on Senseair Sunrise", headline 
 * "1.3 Bus timing" on page 4.
 */
const int     WAIT                    = 180;


/* Read Holding Register */
/*************************/
uint16_t read_holding_register(uint8_t comAddr, uint16_t regAddr) {
  /* Return variable */
  uint16_t returnValue = 0;

  /* PDU variables */
  uint8_t funCode = 0x03;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
  
  uint8_t numRegHi = 0x00;
  uint8_t numRegLo = 0x01;

  /* Define Modbus PDU */
  uint8_t modbusPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo};

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, 8);

  /* Wait for response */
  delay(WAIT);

  if(SunriseSerial.available() <= 0) {
    Serial.println("Response time-out");
    return returnValue;
  }
  
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response [responseSize];

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  if(_handler(response, funCode, responseSize) < 0) {
    return returnValue;
  }

  /* Combine the bytes containing the requested values into a word */
  returnValue = ((int16_t)(int8_t) response[3] << 8) | (uint16_t)response[4];
  
  return returnValue;
}

/* Read Input Register */
/***********************/
uint16_t read_input_register(uint8_t comAddr, uint16_t regAddr) {
  /* Return variable */
  uint16_t returnValue = 0;

  /* PDU variables */
  uint8_t funCode = 0x04;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
  
  uint8_t numRegHi = 0x00;
  uint8_t numRegLo = 0x01;

  /* Define Modbus PDU */
  uint8_t modbusPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo};

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddr, numRegHi, numRegLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, 8);

  /* Wait for response */
  delay(WAIT);

  if(SunriseSerial.available() <= 0) {
    Serial.println("Response time-out");
    return returnValue;
  }

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response [responseSize];

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  if(_handler(response, funCode, responseSize) < 0) {
    return returnValue;
  }

  /* Combine the bytes containing the requested values into a word */
  returnValue = ((int16_t)(int8_t) response[3] << 8) | (uint16_t)response[4];
  
  return returnValue;
}


/* Write Multiple Registers */
/*********************************/
void write_to_register(uint8_t comAddr, uint8_t regAddr, uint16_t writeVal) {
  /* PDU variables */
  uint8_t funCode = 0x10;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
  
  uint8_t numRegHi = 0x00;
  uint8_t numRegLo = 0x01;

  uint8_t numBytes = 0x02;
  
  uint8_t writeValHi = (writeVal >> 8);
  uint8_t writeValLo = writeVal & 0xFF;
  
  uint8_t modbusPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, numBytes, writeValHi, writeValLo};

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, 9);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, numBytes, writeValHi, writeValLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, 11);

  /* Wait for response */
  delay(WAIT);

  if(SunriseSerial.available() <= 0) {
    Serial.println("Response time-out");
    return;
  }

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response [responseSize];
  
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  _handler(response, funCode, responseSize);
}

/*********************************************************************************************/
/*********************************************************************************************/
/*********************************************************************************************/

/* FUNCTION: EXCEPTION AND ERROR HANDLER */
/*****************************************/
int _handler(uint8_t pdu[], uint8_t funCode, int len) {
  /* Return variable */
  int error = 0;

  /* Function variables */
  uint8_t exceptionFunCode = funCode + 0x80; 

  /* Exception codes */
  const uint8_t illegalFunction = 0x01;
  const uint8_t illegalDataAddress = 0x02;
  const uint8_t illegalDataValue = 0x03;

  /* Check for corrupt data in the response */
  uint16_t crc = _generate_crc(pdu, (len - 2));
  uint8_t crcHi = (crc >> 8);
  uint8_t crcLo = crc & 0xFF;

  if(crcLo != pdu[len - 2] || crcHi != pdu[len - 1]) {
    Serial.println("Error: Corrupted data. CRC in response did not match the rest of the PDU.");
    error = -1;
    return error;
  }

  /* Check response for exceptions */  
  if(pdu[1] == exceptionFunCode) {
    switch (pdu[2]) {
      case illegalFunction:
        Serial.println("Exception Code: Illegal Function");
        error = -1;
        break;

      case illegalDataAddress:
        Serial.println("Exception Code: Illegal Data Address");
        error = -1;
        break;

      case illegalDataValue:
        Serial.println("Exception Code: Illegal Data Value");
        error = -1;
        break;

      default:
        break;   
    }
  }
  return error;
}

/* GENERATE CRC */
/********************************************/
uint16_t _generate_crc(uint8_t pdu[], int len)
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
  uint16_t measMode = read_holding_register(SUNRISE_ADDR, 0x000A);
  Serial.print("Measurement Mode: ");
  Serial.println(measMode);

  /* Read measurement period */
  uint16_t measPeriod = read_holding_register(SUNRISE_ADDR, 0x000B);
  Serial.print("Measurement Period: ");
  Serial.print(measPeriod);
  Serial.println(" sec");

  /* Read measurement samples */
  uint16_t measSamples = read_holding_register(SUNRISE_ADDR, 0x000C);
  Serial.print("Measurement Samples: ");
  Serial.println(measSamples);
}


/* Read the sensor's current CO2 value and Error Status and print them */
void readSensorMeasurement(uint8_t target) {
  /* Read CO2 value */
  uint16_t co2Value = read_input_register(target, 0x0003);
  Serial.print("CO2: ");
  Serial.print(co2Value);
  Serial.println(" ppm");

  /* Read error status */
  uint16_t eStatus = read_input_register(target, 0x0000);
  Serial.print("Error Status: 0x");
  Serial.println(eStatus, HEX);
}


/* Change measurement mode */
void changeMeasurementMode(uint8_t target) {
  if(read_holding_register(SUNRISE_ADDR, 0x0A) == 0) {
    write_to_register(target, 0x000A, (uint16_t)0x0001);
    Serial.println("Measurement mode changed to Single Mode");
    
  }else if(read_holding_register(SUNRISE_ADDR, 0x000A) == 1) {
    write_to_register(target, 0x0A, (uint16_t)0x0000);
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
