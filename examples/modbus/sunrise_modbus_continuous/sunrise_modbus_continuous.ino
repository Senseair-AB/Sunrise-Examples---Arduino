/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        sunrise_modbus_continuous.ino
 * @brief       Example functions to perform the different operations 
 *              descrived in the "Modbus on Senseair Sunrise" documentation 
 *              (available on the www.senseair.com website). This example mainly 
 *              covers operations in continuous measurement mode.
 * @details     Tested on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.13
 * @date        2019-08-13
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
 * Sunrise".
 */
const int       WAIT_MS                   = 180;
/* For baudrate equal 9600 the Modbus 3.5T interval is close to 3.5 ms, we round it to 4 ms*/
const int       INTER_PACKET_INTERVAL_MS  = 4;

/* Error codes */
const int COMMUNICATION_ERROR           = -1;
const int ILLEGAL_FUNCTION              = 1;
const int ILLEGAL_DATA_ADDRESS          = 2;
const int ILLEGAL_DATA_VALUE            = 3;

/* Function codes */

/* Register addresses */
const uint16_t ERROR_STATUS             = 0x0000;
const uint16_t MEASUREMENT_MODE         = 0x000A;

/* Measurement modes */
const uint16_t CONTINUOUS               = 0x0000;
const uint16_t SINGLE                   = 0x0001;

/* Reading period, in milliseconds. Default is 4 seconds */
int readPeriodMs = 4000;

/**  
 * Arrays for request, responses and register values
 * Sizes based on:
 * MODBUS APPLICATION PROTOCOL SPECIFICATION V1.1b3 
 * Available on the www.modbus.org website
 */
/* Array for sending requests */
uint8_t request[256];
/* Array for receiving responses */
uint8_t response[256];
/* Array for storing register values from responses */
uint16_t values[256];
/* Array for storing strings from reading objects */
char device[12];

/**
 * @brief  Reads slave response.
 * 
 * @param  waitBytes:    Number of expected bytes for receive packet
 *         funCode:      Function code
 * @note   This function stores the values read through a global
 *         araay, which can then be read to obtain the values.
 * @retval Error status, >0 on success (response size), -1 on communication error
 *         or time-out, and 1 - 3 for exceptions.
 */
int modbus_read_response(int waitBytes, uint8_t funCode) {
  /* Time-out variable */
  unsigned long byteTime = millis();
  int           available_bytes;
  unsigned long timestamp;
  /* Return variable */
  int error;

  /* Wait for first byte in packet */
  while((available_bytes = SunriseSerial.available()) == 0) {
    unsigned long timeout = (unsigned long)((long)millis() - (long)byteTime);
    if(WAIT_MS < timeout) {
      return COMMUNICATION_ERROR;
    }
  }
  
  byteTime = millis();    
  
  do {
    int new_available_bytes = SunriseSerial.available();
        
    timestamp = millis();
        
    if(available_bytes != new_available_bytes) {
      byteTime = timestamp;
      available_bytes = new_available_bytes;
    }
  } while(INTER_PACKET_INTERVAL_MS > (unsigned long)((long)timestamp - (long)byteTime));


  for(int n = 0 ; n < available_bytes ; n++) {
    response[n] = SunriseSerial.read();
  }
  
  /* Check response for exceptions */
  error = _handler(response, funCode, available_bytes);
  return ((error == 0) ? available_bytes : error);
}

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
  int error;

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
  error = modbus_read_response(waitBytes, funCode);
  
  /* If no error were encountered, combine the bytes containing the requested values into words */
  if(error > 0) {
    int counter = 0;
    int slot = 3;
      while(counter < ((error - 5)/2)) {
      values[counter] = ((int16_t)(int8_t) response[slot] << 8) | (uint16_t)response[slot + 1];

      counter++;
      slot = slot + 2;
    }
  } else {
    return error;
  }
  
  return 0;
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
  error = modbus_read_response(waitBytes, funCode);

  /* If no error were encountered, combine the bytes containing the requested values into words */
  if(error > 0) {
    int counter = 0;
    int slot = 3;
      while(counter < ((error - 5)/2)) {
      values[counter] = ((int16_t)(int8_t) response[slot] << 8) | (uint16_t)response[slot + 1];

      counter++;
      slot = slot + 2;
    }
  } else {
    return error;
  }

  return 0;
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
int write_multiple_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg, uint16_t writeVal[]) {
  /* Return variable */
  int error = 0;

  /* PDU variables */
  uint8_t funCode = 0x10;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;

  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;

  uint8_t numBytes = numReg * 2;

  /* Check if request fits in buffer */
  if(numBytes >= 249) {
    return;
  }

  uint8_t writeValHi;
  uint8_t writeValLo;

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

  requestSize += 2;

  SunriseSerial.write(request, requestSize);

  /* Number of bytes to wait for */
  int waitBytes = 8;
  /* Wait for response */
  error = modbus_read_response(waitBytes, funCode);
  
  return (error > 0) ? 0 : error;
}

/**
 * @brief  Reads one of the device's ID objects.
 * 
 * @param  comAddr:      Communication address
 *         objId:        The type of object that is to be read
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
  error = modbus_read_response(waitBytes, funCode);
  if(error > 0) {
    /* Combine the bytes containing the requested values into words */
    int objLength = response[9];
    int slot = 10;
    for(int n = 0 ; n < objLength ; n++) {
      device[n] = response[slot];
  
      slot++;
    }
    device[objLength] = '\0';
    return 0;
  } else {
    return error;
  }
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
  /* Check for malformed packet */
  if(len >= 4) {
    /* Check for corrupt data in the response */
    uint16_t crc = _generate_crc(pdu, (len - 2));
    uint8_t crcHi = (crc >> 8);
    uint8_t crcLo = crc & 0xFF;
  
    if(crcLo != pdu[len - 2] || crcHi != pdu[len - 1]) {
      return COMMUNICATION_ERROR;
    }
  
    /* Check response for exceptions */  
    if(pdu[1] == exceptionFunCode) {
      switch (pdu[2]) {
        case ILLEGAL_FUNCTION:
          error = -ILLEGAL_FUNCTION;
          break;
  
        case ILLEGAL_DATA_ADDRESS:
          error = -ILLEGAL_DATA_ADDRESS;
          break;
  
        case ILLEGAL_DATA_VALUE:
          error = -ILLEGAL_DATA_VALUE;
          break;
  
        default:
          error = COMMUNICATION_ERROR;
          break;   
      }
    }
  } else {
    error = COMMUNICATION_ERROR;
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

  delay(readPeriodMs);
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
  uint16_t numReg = 0x0003;

  if(read_holding_registers(target, MEASUREMENT_MODE, numReg) != 0) {
    Serial.println("EXCEPTION: Failed to read Sensor Configurations");
    return;
  }

  uint16_t measMode = values[0];
  uint16_t measPeriod = values[1];
  uint16_t measSamples = values[2];

  /* Read measurement mode */
  Serial.print("Measurement Mode: ");
  Serial.println(measMode);

  /* Read measurement period */
  Serial.print("Measurement Period: ");
  Serial.print(measPeriod);
  Serial.println(" sec");
  readPeriodMs = measPeriod * 1000;

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
  uint16_t numReg = 0x0001;
  uint16_t change[] = {CONTINUOUS};

  if(read_holding_registers(target, MEASUREMENT_MODE, numReg) != 0) {
    Serial.println("EXCEPTION: Faled to read Measurement Mode");
    Serial.println("Failed to change Measurement Mode");
    /* FATAL ERROR */
    while(true);
  }

  if(values[0] != CONTINUOUS) {
    Serial.println("Changing Measurement Mode to Continuous...");
    if(write_multiple_registers(target, MEASUREMENT_MODE, numReg, change) != 0) {
      Serial.println("EXCEPTION: Failed to change Measurement Mode");
      /* FATAL ERROR */
      while(true);
    }
    Serial.println("Sensor restart is required to apply changes");
    /* FATAL ERROR */
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
  uint16_t numReg = 0x0004;

  /* Read values */
  if((error = read_input_registers(target, ERROR_STATUS, numReg)) != 0) {
    Serial.print("EXCEPTION! Failed to read input registers. Error code: ");
    Serial.println(error);
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
  delay(readPeriodMs);
  
  /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);
}
