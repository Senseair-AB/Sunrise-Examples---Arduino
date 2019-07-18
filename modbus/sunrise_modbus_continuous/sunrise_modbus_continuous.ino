/*
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_modbus_continuous.ino
 * @brief       Example functions to perform different the different operations 
 *              descrived in the "Modbus on Senseair Sunrise" documentation.
 *              This example mainly covers operations in continuous measurement
 *              mode.
 * @details     Tested on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.11
 * @date        2019-07-11
 *
 *******************************************************************************
 */

#include <SoftwareSerial.h>

/* Define serial pins for communication */
const int       SUNRISE_TX              = 12;
const int       SUNRISE_RX              = 11;
SoftwareSerial SunriseSerial = SoftwareSerial(SUNRISE_TX, SUNRISE_RX);

/* Sunrise communication address, both for Modbus and I2C */
const uint8_t   SUNRISE_ADDR            = 0x68;

/* 
 * The delay when waiting for responses, in milliseconds.
 * Length based on the documentation, "Modbus on Senseair 
 * Sunrise", headline "1.3 Bus timing", page 4.
 */
const int       WAIT                    = 180;

/* Reading period, in milliseconds. Default is 4 seconds */
int readPeriod = 4000;

/**  
 * Arrays for request, responses and register values
 * Consequently: 
 * RS232 / RS485 ADU  = 253 bytes + Server address (1 byte) + CRC (2 bytes) = 256 bytes. 
 * TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes
 * From: MODBUS APPLICATION PROTOCOL SPECIFICATION V1.1b3 TODO: FIXA SOURCE
 */
uint8_t request[256];
uint8_t response[256];
uint16_t values[256];
char device[12];

/**
 * @brief  Reads multiple holding registers.
 * 
 * @param  comAddr:      Communication address
 *         regAddr:      Starting register address
 *         numReg:       Number of registers to read from
 * @note   This function stores the values read through a global
 *         araay, which can then be read to obtain the values.
 * @retval Error status, 0 on success, -1 on communication error
 *         or time-out, and 1 - 3 for exceptions.
 */
int read_holding_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg) {
  /* Return variable */
  int error = 0;

  /* PDU variables */
  uint8_t funCode = 0x03;
  
  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
    
  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;
  

  /* Define Modbus PDU */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = regAddrHi;
  request[3] = regAddrLo;
  request[4] = numRegHi;
  request[5] = numRegLo;

  /* Create CRC */
  uint16_t crc = _generate_crc(request, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  request[6] = crcLo;
  request[7] = crcHi;

  /* Send request */
  SunriseSerial.write(request, 8);

  /* Number of bytes to wait for */
  int waitBytes = 5 + (numReg * 2);

  /* Wait for response */
  int counter = 0;

  for(int n = 1 ; n < (waitBytes + 1) ; n++) {
    while(SunriseSerial.available() < n) {
      delay(1);
      counter++;

      /* If the request times-out */
      if(counter > WAIT) {

        /* If there are 5 bytes available, an exception 
         * most likely occurred.
         */
        if(SunriseSerial.available() == 5) {
          /* Store response bytes into a response array */
          int responseSize = SunriseSerial.available();

          for(int o = 0 ; o < responseSize ; o++) {
            response[o] = SunriseSerial.read();
          }

          /* Check the response for errors and exceptions */
          error = _handler(response, funCode, responseSize);
     
          return error;
        }

        /* Otherwise the request just timed out*/
        return -1;
      }
    }
  }

  /* If the request was Successful */
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Combine the bytes containing the requested values into words */
  counter = 0;
  int slot = 3;
    while(counter < ((responseSize - 5)/2)) {
    values[counter] = ((int16_t)(int8_t) response[slot] << 8) | (uint16_t)response[slot + 1];

    counter++;
    slot = slot + 2;
  }

  return error;
}

/**
 * @brief  Reads multiple input registers.
 * 
 * @param  comAddr:      Communication address
 *         regAddr:      Starting register address
 *         numReg:       Number of registers to read from
 * @note   This function stores the values read through a global
 *         araay, which can then be read to obtain the values.
 * @retval Error status, 0 on success, -1 on communication error
 *         or time-out, and 1 - 3 for exceptions.
 */
int read_input_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg) {
  /* Return variable */
  int error = 0;
  /* PDU variables */
  uint8_t funCode = 0x04;
  
  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;
    
  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;
  

  /* Define Modbus PDU */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = regAddrHi;
  request[3] = regAddrLo;
  request[4] = numRegHi;
  request[5] = numRegLo;

  /* Create CRC */
  uint16_t crc = _generate_crc(request, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  request[6] = crcLo;
  request[7] = crcHi;

  /* Send request */
  SunriseSerial.write(request, 8);

  /* Number of bytes to wait for */
  int waitBytes = 5 + (numReg * 2);

  /* Wait for response */
  int counter = 0;

  for(int n = 1 ; n < (waitBytes + 1) ; n++) {
    while(SunriseSerial.available() < n) {
      delay(1);
      counter++;

      /* If the request times-out */
      if(counter > WAIT) {

        /* If there are 5 bytes available, an exception 
         * most likely occurred.
         */
        if(SunriseSerial.available() == 5) {
          /* Store response bytes into a response array */
          int responseSize = SunriseSerial.available();

          for(int o = 0 ; o < responseSize ; o++) {
            response[o] = SunriseSerial.read();
          }

          /* Check the response for errors and exceptions */
          error = _handler(response, funCode, responseSize);
     
          return error;
        }

        /* Otherwise the request just timed out*/
        return -1;
      }
    }
  }

  /* If the request was Successful */
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Combine the bytes containing the requested values into words */
  counter = 0;
  int slot = 3;
    while(counter < ((responseSize - 5)/2)) {
    values[counter] = ((int16_t)(int8_t) response[slot] << 8) | (uint16_t)response[slot + 1];

    counter++;
    slot = slot + 2;
  }

  return error;
}

/**
 * @brief  Writes to multiple holding registers.
 * 
 * @param  comAddr:  Communication address
 *         regAddr:  Register address
 *         numReg:   Number of registers to write to
 *         writeVal: The values to write to the registers
 * @retval Error status, 0 on success, -1 on communication error
 *         or time-out, and 1 - 3 for exceptions.
 */
int write_multiple_registers(uint8_t comAddr, uint8_t regAddr, uint16_t numReg, uint16_t writeVal[]) {
  /* Return variable */
  int error = 0;

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

  int requestSize = 7 + numBytes;

  /* Assign the first 7 bytes to the request array */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = regAddrHi;
  request[3] = regAddrLo;
  request[4] = numRegHi;
  request[5] = numRegLo;
  request[6] = numBytes;

  /* Convert the words to be written into 2 bytes and assign them to the request array */
  int counter = 7;
  for(int n = 0 ; n < numBytes ; n++) {
    request[counter] = (writeVal[n] >> 8);
    request[counter+1] = writeVal[n] & 0xFF;
    counter += 2;
  }

  /* Create CRC */
  uint16_t crc = _generate_crc(request, requestSize);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  request[requestSize] = crcLo;
  request[requestSize+1] = crcHi;

  requestSize +=2;

  SunriseSerial.write(request, requestSize);

  /* Number of bytes to wait for */
  int waitBytes = 8;

  /* Wait for response */
  counter = 0;
  
  for(int n = 1 ; n < (waitBytes + 1) ; n++) {
    while(SunriseSerial.available() < n) {
      delay(1);
      counter++;

      /* If the request times-out */
      if(counter > WAIT) {

        /* If there are 5 bytes available, an exception 
         * most likely occurred.
         */
        if(SunriseSerial.available() == 5) {
          /* Store response bytes into a response array */
          int responseSize = SunriseSerial.available();

          for(int o = 0 ; o < responseSize ; o++) {
            response[o] = SunriseSerial.read();
          }

          /* Check the response for errors and exceptions */
          error = _handler(response, funCode, responseSize);
     
          return error;
        }

        /* Otherwise the request just timed out*/
        return -1;
      }
    }
  }

  /* If the request was Successful */
  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();
  
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  return error;
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
int read_device_id(uint8_t comAddr, uint8_t objId) {
  /* Return variable */
  int error = 0;

  /* PDU variables */
  uint8_t funCode = 0x2B;

  uint8_t meiType = 0x0E;

  uint8_t idCode = 0x04;

  /* Define Modbus PDU */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = meiType;
  request[3] = idCode;
  request[4] = objId;

  /* Create CRC */
  uint16_t crc = _generate_crc(request, 5);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  request[5] = crcLo;
  request[6] = crcHi;
  
  /* Send request */
  SunriseSerial.write(request, 7);

  /* Number of bytes to wait for */
  int objLen = 0;
  if(objId == 0) {
    objLen = 8;
  }else if(objId == 1) {
    objLen = 7;
  }else if(objId == 2) {
    objLen = 4;
  }
  int waitBytes = 12 + objLen;

  /* Wait for response */
  int counter = 0;

  for(int n = 1 ; n < (waitBytes + 1) ; n++) {
    while(SunriseSerial.available() < n) {
      delay(1);
      counter++;
      if(counter > WAIT) {
        if(SunriseSerial.available() == 5) {
          /* Store response bytes into a response array */
          int responseSize = SunriseSerial.available();

          for(int o = 0 ; o < responseSize ; o++) {
            response[o] = SunriseSerial.read();
          }

          /* Check the response for errors and exceptions */
          error = _handler(response, funCode, responseSize);
     
          return error;
        }
        return -1;
      }
    }
  }

  /* Store response bytes into a response array */
  int responseSize = SunriseSerial.available();

  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SunriseSerial.read();
  }

  /* Combine the bytes containing the requested values into words */
  int objLength = response[9];
  counter = 0;
  int slot = 10;
    while(counter < objLength) {
    device[counter] = response[slot];

    counter++;
    slot++;
  }
  device[counter] = '\0';

  return error;
}

/**
 * @brief  A handler for possible exceptions and errors.
 * 
 * @param  pdu[]: An array containing the response from a request
 *         funCode: The function code used for the request
 *         len: The length of the pdu
 * @retval  -1 for incorrect CRC, 0 on success, 1 for Illegal
 *          Function, 2 for Illegal Data Address, and 3 for
 *          Illegal Data Value
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
    error = -1;
    return error;
  }

  /* Check response for exceptions */  
  if(pdu[1] == exceptionFunCode) {
    switch (pdu[2]) {
      case illegalFunction:
        error = 1;
        break;

      case illegalDataAddress:
        error = 2;
        break;

      case illegalDataValue:
        error = 3;
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

  /* Begin serial communication */
  Serial.begin(115200);
  SunriseSerial.begin(9600);

  Serial.println("Initialization complete \n");

  /* Read the sensor's Device ID */
  Serial.println("Sensor's Device ID");
  read_sensor_id(SUNRISE_ADDR);
  Serial.println();

  /* Read the sensor's configs */
  Serial.println("Sensor's Measurement Configs");
  read_sensor_config(SUNRISE_ADDR);
  Serial.println();

  /* Change measurement mode if single */
  change_measurement_mode(SUNRISE_ADDR);
  Serial.println();

  delay(readPeriod);
}

/**
 * @brief  Reads and prints the sensor's device identification.
 * 
 * @param  target: The sensor's communication address.
 * @note   This example shows a simple way to read and print the 
 *         sensor's Vendor Name, ProductCode and MajorMinorRevision.
 * @retval None
 */
void read_sensor_id(uint8_t target) {
  /* Vendor Name */
  if(read_device_id(target, 0) != 0) {
    Serial.println("EXCEPTION: Failed to read Vendor Name");
  }else {
    Serial.print("Vendor Name: ");
    Serial.println(device); 
  }

  /* ProductCode */
  if(read_device_id(target, 1) != 0) {
    Serial.println("EXCEPTION: Failed to read ProductCode");
  }else {
    Serial.print("ProductCode: ");
    Serial.println(device);
  }

  /* MajorMinorRevision */
  if(read_device_id(target, 2) != 0) {
    Serial.println("EXCEPTION: Failed to read MajorMinorRevision");
  }else {
    Serial.print("MajorMinorRevision: ");
    Serial.println(device);
  }
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
  uint16_t regAddr = 0x000A;
  uint16_t numReg = 0x0003;

  if(read_holding_registers(target, regAddr, numReg) != 0) {
    Serial.println("EXCEPTION: Failed to read Sensor Configurations");
    return;
  }

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
  readPeriod = measPeriod * 1000;

  /* Read measurement samples */
  Serial.print("Number of Samples: ");
  Serial.println(measSamples); 
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
  uint16_t regAddr = 0x000A;

  uint16_t numReg = 0x0001;

  uint16_t continuous[] = {0x0000};
  uint16_t single = 0x0001;

  if(read_holding_registers(target, regAddr, numReg) != 0) {
    Serial.println("EXCEPTION: Faled to read Measurement Mode");
    Serial.println("Failed to change Measurement Mode");
    return;
  }

  if(values[0] == single) {
    Serial.println("Changing Measurement Mode to Continuous...");
    if(write_multiple_registers(target, regAddr, numReg, continuous) != 0) {
      Serial.println("EXCEPTION: Failed to change Measurement Mode");
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
  uint16_t regAddr = 0x0000;
  uint16_t numReg = 0x0004;

  /* Read values */
  if(read_input_registers(target, regAddr, numReg) != 0) {
    Serial.println("EXCEPTION: Failed to read Measurements");
  }else {
    /* Read CO2 concentration */
    Serial.print("CO2: ");
    Serial.print(values[3]);
    Serial.println(" ppm");

    /* Read error status */
    Serial.print("Error Status: 0x");
    Serial.println(values[0], HEX);
  }
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
  Serial.println("Waiting...\n");
  delay(readPeriod);
  
  /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}