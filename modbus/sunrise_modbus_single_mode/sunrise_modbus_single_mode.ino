/*
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_modbus_single_mode.ino
 * @brief       Example for reading sensor data in single measurement mode.
 *              Based on the "Modbus on Senseair Sunrise" documentation.
 * @details     Tested on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.03
 * @date        2019-06-25
 *
 *******************************************************************************
 */

#include <SoftwareSerial.h>

/* Define serial pins for communication */
const int       SUNRISE_TX              = 12;
const int       SUNRISE_RX              = 11;
SoftwareSerial SunriseSerial = SoftwareSerial(SUNRISE_TX, SUNRISE_RX);

/* Reading period */
const int       MIN_READ_PERIOD_MS      = 2000;

/* Sunrise communication address, both for Modbus and I2C */
const uint8_t   SUNRISE_ADDR            = 0x68;

/* 
 * Delay when waiting for responses, in milliseconds.
 * Length based on the documentation, "Modbus on Senseair 
 * Sunrise", headline "1.3 Bus timing", page 4.
 */
const int       WAIT                    = 180;

/* Maximum attempts for a request */
const int       MAX_ATTEMPTS            = 5;

/* An array for storing the sensor state */
uint16_t        sensorState[12];

/* Variable for keeping track of failed attempts */
int attempt = 0;

/**
  * @brief  Reads a holding register.
  * 
  * @param  comAddr: Communication address
  *         regAddr: Register address
  * @retval Register value
  */
uint16_t holding_register_read(uint8_t comAddr, uint16_t regAddr) {
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
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

  /* Wait for response */
  while(SunriseSerial.available() <= 0) {
    delay(WAIT);
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Response time-out");
      return returnValue; 
    }
  }
  
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response[responseSize];

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  while(_handler(response, funCode, responseSize) < 0) {
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Reached max attempts. Request cancelled.");
      return returnValue;
    }
    /* Re-send PDU */
    SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

    /* Store response bytes into a response array */
    responseSize = SunriseSerial.available();
    response[responseSize];
  
    for(int n = 0 ; n < responseSize ; n++) {
      response[n] = SunriseSerial.read();
    }
  }

  /* Reset attempt on success */
  attempt = 0;

  /* Combine the bytes containing the requested values into a word */
  returnValue = ((int16_t)(int8_t) response[3] << 8) | (uint16_t)response[4];

  return returnValue;
}

/**
  * @brief  Reads an input register.
  * 
  * @param  comAddr: Communication address
  *         regAddr: Register address
  * @retval Register value
  */
uint16_t input_register_read(uint8_t comAddr, uint16_t regAddr) {
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
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddr, numRegHi, numRegLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

  /* Wait for response */
  while(SunriseSerial.available() <= 0) {
    delay(WAIT);
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Response time-out");
      return returnValue; 
    }
  }

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response[responseSize];

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  while(_handler(response, funCode, responseSize) < 0) {
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Reached max attempts. Request cancelled.");
      return returnValue;
    }
    /* Re-send PDU */
    SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

    /* Store response bytes into a response array */
    responseSize = SunriseSerial.available();
    response[responseSize];
  
    for(int n = 0 ; n < responseSize ; n++) {
      response[n] = SunriseSerial.read();
    }
  }

  /* Reset attempt on success */
  attempt = 0;

  /* Combine the bytes containing the requested values into a word */
  returnValue = ((int16_t)(int8_t) response[3] << 8) | (uint16_t)response[4];

  return returnValue;
}


/**
  * @brief  Writes a value to a register
  * 
  * @param  comAddr:  Communication address
  *         regAddr:  Register address
  *         writeVal: The value to write to the register
  * @retval None
  */
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

  /* Define Modbus PDU */
  uint8_t modbusPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, numBytes, writeValHi, writeValLo};

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, numBytes, writeValHi, writeValLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

  /* Wait for response */
  while(SunriseSerial.available() <= 0) {
    delay(WAIT);
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Response time-out");
      return; 
    }
  }

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response[responseSize];
  
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  while(_handler(response, funCode, responseSize) < 0) {
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Reached max attempts. Request cancelled.");
      return;
    }
    /* Re-send PDU */
    SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

    /* Store response bytes into a response array */
    responseSize = SunriseSerial.available();
    response[responseSize];

    for(int n = 0 ; n < responseSize ; n++) {
      response[n] = SunriseSerial.read();
    }
  }

  /* Reset attempt on success */
  attempt = 0;
}

/**
  * @brief  Reads multiple holding registers.
  * 
  * @param  comAddr:      Communication address
  *         regAddr:      Register address
  *         numReg:       Number of registers to read from
  *         returnValues: The array the register values are written to
  * @note   This function "returns" its values through an array that is
  *         passed to the function as an argument.
  * @retval None
  */
void read_holding_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg, uint16_t returnValues[]) {
  /* PDU variables */
  uint8_t funCode = 0x03;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
  
  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;

  /* Define Modbus PDU */
  uint8_t modbusPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo};

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

  /* Wait for response */
  while(SunriseSerial.available() <= 0) {
    delay(WAIT);
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Response time-out");
      return; 
    }
  }
  
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response[responseSize];

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  while(_handler(response, funCode, responseSize) < 0) {
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Reached max attempts. Request cancelled.");
      return;
    }
    /* Re-send PDU */
    SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

    /* Store response bytes into a response array */
    responseSize = SunriseSerial.available();
    response[responseSize];
  
    for(int n = 0 ; n < responseSize ; n++) {
      response[n] = SunriseSerial.read();
    }
  }

  /* Reset attempt on success */
  attempt = 0;

  /* Combine the bytes containing the requested values into words */
  int counter = 0;
  int slot = 3;
    while(counter < ((responseSize - 5)/2)) {
    returnValues[counter] = ((int16_t)(int8_t) response[slot] << 8) | (uint16_t)response[slot + 1];

    counter++;
    slot = slot + 2;
  }
}


/**
  * @brief  Reads multiple input registers.
  * 
  * @param  comAddr:      Communication address
  *         regAddr:      Register address
  *         numReg:       Number of registers to read from
  *         returnValues: The array the register values are written to
  * @note   This function "returns" its values through an array that is
  *         passed to the function as an argument.
  * @retval None
  */
void read_input_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg, uint16_t returnValues[]) {
  /* PDU variables */
  uint8_t funCode = 0x04;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
  
  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;

  /* Define Modbus PDU */
  uint8_t modbusPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo};

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, regAddrHi, regAddrLo, numRegHi, numRegLo, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

  /* Wait for response */
  while(SunriseSerial.available() <= 0) {
    delay(WAIT);
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Response time-out");
      return; 
    }
  }
  
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response[responseSize];

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  while(_handler(response, funCode, responseSize) < 0) {
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Reached max attempts. Request cancelled.");
      return;
    }
    /* Re-send PDU */
    SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

    /* Store response bytes into a response array */
    responseSize = SunriseSerial.available();
    response[responseSize];
  
    for(int n = 0 ; n < responseSize ; n++) {
      response[n] = SunriseSerial.read();
    }
  }

  /* Reset attempt on success */
  attempt = 0;

  /* Combine the bytes containing the requested values into words */
  int counter = 0;
  int slot = 3;
    while(counter < ((responseSize - 5)/2)) {
    returnValues[counter] = ((int16_t)(int8_t) response[slot] << 8) | (uint16_t)response[slot + 1];

    counter++;
    slot = slot + 2;
  }
}

/**
  * @brief  Writes to multiple holding registers.
  * 
  * @param  comAddr:  Communication address
  *         regAddr:  Register address
  *         numReg:   Number of registers to write to
  *         writeVal: The array the register values are written to
  * @note   This function "returns" its values through an array that is
  *         passed to the function as an argument.
  * @retval None
  */
void write_multiple_registers(uint8_t comAddr, uint8_t regAddr, uint16_t numReg, uint16_t writeVal[]) {
  /* PDU variables */
  uint8_t funCode = 0x10;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
  
  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;

  uint8_t numBytes = numReg * 2;
  
  uint8_t writeValHi;
  uint8_t writeValLo;
  uint8_t modbusPDU[7 + numBytes];

  /* Assign the first 7 bytes to the request array */
  modbusPDU[0] = comAddr;
  modbusPDU[1] = funCode;
  modbusPDU[2] = regAddrHi;
  modbusPDU[3] = regAddrLo;
  modbusPDU[4] = numRegHi;
  modbusPDU[5] = numRegLo;
  modbusPDU[6] = numBytes;

  /* Convert the words to be written into 2 bytes and assign them to the request array */
  int counter = 7;
  for(int n = 0 ; n < numBytes ; n++) {
    modbusPDU[counter] = (writeVal[n] >> 8);
    modbusPDU[counter+1] = writeVal[n] & 0xFF;
    counter += 2;
  }

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[sizeof(modbusPDU) + 2];
  for(int n = 0 ; n < sizeof(modbusPDU) ; n++) {
    modbusSerialPDU[n] = modbusPDU[n];
  }
  modbusSerialPDU[sizeof(modbusPDU) + 1] = crcHi;
  modbusSerialPDU[sizeof(modbusPDU)] = crcLo;

  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

  /* Wait for response */
  while(SunriseSerial.available() <= 0) {
    delay(WAIT);
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Response time-out");
      return; 
    }
  }

  /* Reset attempt on success */
  attempt = 0;

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response[responseSize];
  
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Check the response for errors and exceptions */
  while(_handler(response, funCode, responseSize) < 0) {
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Reached max attempts. Request cancelled.");
      return;
    }
    /* Re-send PDU */
    SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

    /* Store response bytes into a response array */
    responseSize = SunriseSerial.available();
    response[responseSize];
  
    for(int n = 0 ; n < responseSize ; n++) {
      response[n] = SunriseSerial.read();
    }
  }

  /* Reset attempt on success */
  attempt = 0;
}

/**
  * @brief  Reads one of the device's ID objects.
  * 
  * @param  comAddr:      Communication address
  *         regAddr:      Register address
  *         numReg:       Number of registers to write to
  *         returnValues: Array of values to be written to the registers        
  * @retval None
  */
void read_device_id(uint8_t comAddr, uint8_t objectId, char returnValue[]) {
  /* PDU variables */
  uint8_t funCode = 0x2B;

  uint8_t meiType = 0x0E;

  uint8_t idCode = 0x04;

  /* Define Modbus PDU */
  uint8_t modbusPDU[] = {comAddr, funCode, meiType, idCode, objectId};

  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  uint8_t modbusSerialPDU[] = {comAddr, funCode, meiType, idCode, objectId, crcLo, crcHi};
  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

  /* Wait for response */
  while(SunriseSerial.available() <= 0) {
    delay(WAIT);
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Response time-out");
      return; 
    }
  }
  
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  uint8_t response[responseSize];

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }
  
  /* Check the response for errors and exceptions */
  while(_handler(response, funCode, responseSize) < 0) {
    attempt++;
    if(attempt >= MAX_ATTEMPTS) {
      Serial.println("Reached max attempts. Request cancelled.");
      return;
    }
    /* Re-send PDU */
    SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

    /* Store response bytes into a response array */
    responseSize = SunriseSerial.available();
    response[responseSize];
  
    for(int n = 0 ; n < responseSize ; n++) {
      response[n] = SunriseSerial.read();
    }
  }

  /* Reset attempt on success */
  attempt = 0;

  /* Combine the bytes containing the requested values into words */
  int counter = 0;
  int slot = 10;
    while(counter < response[9]) {
    returnValue[counter] = response[slot];

    counter++;
    slot++;
  }
  returnValue[counter] = '\0';
}

/**
  * @brief  A handler for possible exceptions and errors.
  * 
  * @param  pdu[]: An array containing the response from a request
  *         funCode: The function code used for the request
  *         len: The length of the pdu
  * @retval (int) -1 if an error or exception ocurred, otherwise 0
  */
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

/**
  * @brief  Computes a Modbus RTU message CRC, for a given message.
  * 
  * @param  pdu[]: An array containing the message
  *         len: the length of the array
  * @note   The bytes in the return value needs to be switched for
  *         them to be in the right order in the message.
  * @retval The CRC for the message
  */
uint16_t _generate_crc(uint8_t pdu[], int len) {
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

/**
  * @brief  This function runs once at the start.
  *
  * @retval None
  */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(11, OUTPUT);

  /* Begin serial communication */
  Serial.begin(115200);
  SunriseSerial.begin(9600);
  
  Serial.println("Initialization complete\n");

  /* Read the sensor's Device ID */
  Serial.println("Sensor's Device ID");
  read_sensor_id(SUNRISE_ADDR);
  Serial.println(' ');

  /* Read the sensor's configs */
  Serial.println("Sensor's Measurement Configs");
  read_sensor_config(SUNRISE_ADDR);
  Serial.println(' ');

  delay(MIN_READ_PERIOD_MS);

  /* Change measurement mode */
  change_measurement_mode(SUNRISE_ADDR);

  /* Read and save sensor state data from HR35 - HR46 */
  read_holding_registers(SUNRISE_ADDR, 0x0022, 0x000C, sensorState);
}

/**
  * @brief  Reads and prints the sensor's device identification.
  * 
  * @param  target: The sensor's communication address
  * @note   This example shows a simple way to read and print the 
  *         sensor's Vendor Name, ProductCode and MajorMinorRevision, 
  *         through one read request for each object.
  * @retval None
  */
void read_sensor_id(uint8_t target) {
  /* Array for Vendor Name */
  char value1[9];
  /* Array for ProductCode */
  char value2[8];
  /* Array for MajorMinorRevision */
  char value3[4];

  /* Reads the values */
  read_device_id(target, 0, value1);
  read_device_id(target, 1, value2);
  read_device_id(target, 2, value3);
  
  /* Prints the values */
  Serial.println(value1);
  Serial.println(value2);
  Serial.println(value3);
}

/**
  * @brief  Reads and prints the sensor's current measurement mode,
  *         measurement period and number of samples.
  * 
  * @param  target: The sensor's communication address
  * @note   This example shows a simple way to read the sensor's
  *         measurement configurations, using one request for
  *         reading the holding registers where they are located.
  * @retval None
  */
void read_sensor_config(uint8_t target) {
  /* Function variables */
  uint16_t regAddrMode = 0x000A;
  uint16_t numReg = 0x0003;
  
  uint16_t values[numReg];
  read_holding_registers(target, regAddrMode, numReg, values);

  uint16_t measMode = values[0];
  uint16_t measPeriod = values[1];
  uint16_t measSamples = values[2];

  /* Read measurement mode (0 = Continuous, 1 = Single) */
  Serial.print("Measurement Mode: ");
  Serial.println(measMode);

  /* Read measurement period */
  Serial.print("Measurement Period: ");
  Serial.print(measPeriod);
  Serial.println(" sec");

  /* Read measurement samples */
  Serial.print("Measurement Samples: ");
  Serial.println(measSamples);
  
}

/**
  * @brief  Reads the sensor's current measurement mode, and if
  *         it's continuous (register value 0) changes it to 
  *         single mode (register value 1).
  * 
  * @param  target: The sensor's communication address
  * @note   This example shows a simple way to change the sensor's
  *         measurement mode from continuous to single, using one 
  *         request for reading the current measurement mode and
  *         another write request if it has to be changed.
  *         
  *         The sensor has to be manually restarted after the
  *         change.
  * @retval None
  */
void change_measurement_mode(uint8_t target) {
  /* Function variables */
  uint16_t regAddr = 0x000A;
  uint16_t continuous = 0x0000;
  uint16_t single = 0x0001;
  
  if(holding_register_read(target, regAddr) == continuous) {
    write_to_register(target, regAddr, single);
    Serial.println("Measurement mode changed to Single Mode");
    Serial.println("Sensor restart is required to apply changes");
  }
}

/**
  * @brief  Reads and prints the sensor's current CO2 value and
  *         error status.
  * 
  * @param  target: The sensor's communication address
  *         state[]: The sensor state
  * @note   This example shows a simple way to read the sensor's
  *         CO2 measurement in single measurement mode.
  * @retval None
  */
void read_sensor_measurements(uint8_t target, uint16_t state[]) {
  /* Function variables */
  uint16_t regAddr = 0x0021;
  uint16_t numReg = 0x000D;
  
  uint16_t regAddrCo2 = 0x0003;
  uint16_t regAddrEStatus = 0x0000;

  uint16_t command[13];

  /* Copy the sensor state into the command array */
  command[0] = 0x0001;
  for(int n = 1 ; n < 13 ; n++) {
    command[n] = sensorState[n - 1];
  }

  /* Drive EN-pin high */
  digitalWrite(11, HIGH);

  /* Wait for sensor to wake up, minimum 35 ms */
  delay(35);

  write_multiple_registers(target, regAddr, numReg, command);

  /* Wait for ready pin to go low, default 2 sec */
  delay(2000);

  /* Read CO2 value */
  uint16_t co2Value = input_register_read(target, regAddrCo2);
  Serial.print("CO2: ");
  Serial.print(co2Value);
  Serial.println(" ppm");

  /* Read error status */
  uint16_t eStatus = input_register_read(target, regAddrEStatus);
  Serial.print("Error Status: 0x");
  Serial.println(eStatus, HEX);

  /* Read sensor state for next measurement */
  read_holding_registers(target, 0x0022, 0x000C, sensorState);

  /* Drive EN-pin low */
  digitalWrite(11, LOW);
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
  
  /* Delay between readings */
  Serial.println("Waiting...");
  delay(MIN_READ_PERIOD_MS);

  /* Read measurements */
  read_sensor_measurements(SUNRISE_ADDR, sensorState);
  Serial.println(' ');
  
  /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}