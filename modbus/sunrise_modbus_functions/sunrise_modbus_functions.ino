/*
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_modbus_functions.ino
 * @brief       Example functions to perform different operations based on the
 *              "Modbus on Senseair Sunrise" documentation
 * @details     Tested on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.08
 * @date        2019-06-19
 *
 *******************************************************************************
 */

#include <SoftwareSerial.h>

/* Define serial pins for communication */
const int       SUNRISE_TX              = 12;
const int       SUNRISE_RX              = 11;
SoftwareSerial SunriseSerial = SoftwareSerial(SUNRISE_TX, SUNRISE_RX);

/* Reading period */
const int       MIN_READ_PERIOD_MS      = 2000; /** Detailed description after variable* /

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
  if(_handler(response, funCode, responseSize) < 0) {
    return;
  }

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
  if(_handler(response, funCode, responseSize) < 0) {
    return;
  }

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
  int counter = 7;
  for(int n = 0 ; n < 7 + numBytes ; n++) {
    if(n == 0) {
      modbusPDU[n] = comAddr;
    }else if(n == 1) {
      modbusPDU[n] = funCode;
    }else if(n == 2) {
      modbusPDU[n] = regAddrHi;
    }else if(n == 3) {
      modbusPDU[n] = regAddrLo;
    }else if(n == 4) {
      modbusPDU[n] = numRegHi;
    }else if(n == 5) {
      modbusPDU[n] = numRegLo;
    }else if(n == 6) {
      modbusPDU[n] = numBytes;
    }else{
      modbusPDU[counter] = (writeVal[n - 7] >> 8);
      modbusPDU[counter+1] = writeVal[n - 7] & 0xFF;
      counter += 2;
    }
  }
  
  /* Create CRC */
  uint16_t crc = _generate_crc(modbusPDU, sizeof(modbusPDU));
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);


  /* Request (HEX) */
  uint8_t modbusSerialPDU[sizeof(modbusPDU)+2];
  for(int n = 0 ; n < sizeof(modbusPDU) ; n++) {
    modbusSerialPDU[n] = modbusPDU[n];
  }
  modbusSerialPDU[sizeof(modbusPDU) + 1] = crcHi;
  modbusSerialPDU[sizeof(modbusPDU)] = crcLo;

  SunriseSerial.write(modbusSerialPDU, sizeof(modbusSerialPDU));

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
  if(_handler(response, funCode, responseSize) < 0) {
    return;
  }

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

  /* Begin serial communication */
  Serial.begin(115200);
  SunriseSerial.begin(9600);

  Serial.println("Initialization complete \n");

  /* Read the sensor's Device ID */
  Serial.println("Sensor's Device ID");
  read_sensor_id(SUNRISE_ADDR);
  Serial.println(' ');

  /* Read the sensor's configs */
  Serial.println("Sensor's Measurement Configs");
  read_sensor_config(SUNRISE_ADDR);
  Serial.println(' ');

  delay(MIN_READ_PERIOD_MS);


  /* Change measurement mode to continuous */
  change_measurement_mode(SUNRISE_ADDR);

  /* Change measurement configs */
  //change_measurement_config(SUNRISE_ADDR);
}

/**
  * @brief  Reads and prints the sensor's current measurement mode,
  *         measurement period and number of samples.
  * 
  * @param  target: The sensor's communication address.
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
  * @brief  Reads and prints the sensor's current CO2 value and
  *         error status.
  * 
  * @param  target: The sensor's communication address.
  * @note   This example shows a simple way to read the sensor's
  *         CO2 measurement and error status, using two requests 
  *         for reading the input registers where they are
  *         located.
  * @retval None
  */
void read_sensor_measurements(uint8_t target) {
  /* Function variables */
  uint16_t regAddrCo2 = 0x0003;
  uint16_t regAddrEStatus = 0x0000;

  /* Read CO2 value */
  uint16_t co2Value = input_register_read(target, regAddrCo2);
  Serial.print("CO2: ");
  Serial.print(co2Value);
  Serial.println(" ppm");

  /* Read error status */
  uint16_t eStatus = input_register_read(target, regAddrEStatus);
  Serial.print("Error Status: 0x");
  Serial.println(eStatus, HEX);
}

/**
  * @brief  Reads the sensor's current measurement mode, and if
  *         it's single mode (register value 1) changes it to 
  *         contiuous mode (register value 0).
  * 
  * @param  target: The sensor's communication address.
  * @note   This example shows a simple way to change the sensor's
  *         measurement mode from single to contiuous, using one 
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
  
  /*if(holding_register_read(target, regAddr) == continuous) {
    write_to_register(target, regAddr, (uint16_t)single);
    Serial.println("Measurement mode changed to Single Mode");
    
  }else */if(holding_register_read(target, regAddr) == single) {
    write_to_register(target, regAddr, (uint16_t)continuous);
    Serial.println("Measurement mode changed to Continuous Mode");
    Serial.println("Sensor restart is required to apply changes");
  } 
}

/**
  * @brief  Changes the sensor's current measurement mode,
  *         measurement period and number of samples.
  * 
  * @param  target: The sensor's communication address.
  * @note   This example shows a simple way to change the sensor's
  *         measurement configurations, using one request for 
  *         reading each current measurement config and then a 
  *         write request to change all of them.
  *         
  *         The sensor has to be manually restarted after the
  *         changes.
  * @retval None
  */
void change_measurement_config(uint8_t target) {
  /* Function variables */
  uint16_t regAddrMode = 0x000A;
  uint16_t regAddrPeriod = 0x000B;
  uint16_t regAddrSamples = 0x000C;

  uint16_t numReg = 0x0003;

  uint16_t continuous = 0x0000;
  uint16_t single = 0x0001;

  uint16_t periodLong = 0x0004;
  uint16_t periodShort =0x0002;

  uint16_t samplesMany = 0x0010;
  uint16_t samplesFew =0x0008;

  uint16_t input[3];

  /* Check current values and change input accordingly */
  if(holding_register_read(target, regAddrMode) == continuous) {
    input[0] = single;
    Serial.println("Measurement mode changed to Single Mode");
    
  }else if(holding_register_read(target, regAddrMode) == single) {
    input[0] = continuous;
    Serial.println("Measurement mode changed to Continuous Mode");
  } 

  if(holding_register_read(target, regAddrPeriod) == periodShort) {
    input[1] = periodLong;
    Serial.println("Measurement period changed to 4 sec");
  }else if(holding_register_read(target, regAddrPeriod) == periodLong) {
    input[1] = periodShort;
    Serial.println("Measurement period changed to 2 sec");
  }

  if(holding_register_read(target, regAddrSamples) == samplesFew) {
    input[2] = samplesMany;
    Serial.println("Measurement samples changed to 16");
  }else if(holding_register_read(target, regAddrSamples) == samplesMany) {
    input[2] = samplesFew;
    Serial.println("Measurement samples changed to 8");
  }

  /* Send a request to change the values */
  write_multiple_registers(target, regAddrMode, numReg, input);
  Serial.println("Sensor restart is required to apply changes");
}

/**
  * @brief  Reads and printsthe sensor's device identification
  * 
  * @param  target: The sensor's communication address.
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
  read_sensor_measurements(SUNRISE_ADDR);
  Serial.println(' ');
  
  /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}