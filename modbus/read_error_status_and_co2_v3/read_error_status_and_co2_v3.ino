/*
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        read_error_status_and_co2_v3.ino
 * @brief       Simple example to show how to read CO2 concentration and error
 *              status using Modbus
 * @details     Should work fine on Arduino Mega 2560
 *              
 * @author      William Sandkvist
 * @version     0.03
 * @date        2019-06-18
 *
 *******************************************************************************
 */

#include <SoftwareSerial.h>

/* Define serial pins for communication */
#define Sunrise_TX 12
#define Sunrise_RX 11
SoftwareSerial SUNRISE_SERIAL = SoftwareSerial(Sunrise_TX, Sunrise_RX);

/* Reading period */
const int     MIN_READ_PERIOD_MS      = 2000;
/* Sunrise communication address, both for Modbus and I2C */
const uint8_t SUNRISE_ADDR           = 0x68;

/* Sensor state */
int16_t   co2_value;      /* CO2 concentration, ppm */
uint16_t  error_status;   /* Errors */
/* Sensor configuration */
uint8_t   meas_mode;      /*0 - continuous mode, 1 - single mode */
uint16_t  meas_period_s;  /* measurement period, seconds */
uint16_t  meas_numbers;   /* number of measurements by measurement period */


/* Request registers to read */
int requestRegisters(uint8_t target, char func[], uint8_t startReg, uint8_t numReg) {
  int error = -1;
  uint8_t funcByte;
  
  /* Create function byte */
  if(func == "HR") {
    funcByte = 0x03;
    
  } else if(func == "IR") {
    funcByte = 0x04;
    
  }else {
    Serial.println("Invalid register type");
    return error;
  }

  /* Generate CRC */
  byte modbus_pdu[] = {target, funcByte, 0x00, startReg, 0x00, numReg};
  uint16_t crc = generateCRC(modbus_pdu, 6);
  
  uint8_t crc_low = crc & 0xFF;
  uint8_t crc_high = (crc >> 8);
  
  /* Create request array */
  byte readRequest[] = {target, funcByte, 0x00, startReg, 0x00, numReg, crc_low, crc_high};

  /* Request values */
  error = SUNRISE_SERIAL.write(readRequest, 8);
  
  /* Waiting for data to be available to read */
  while(SUNRISE_SERIAL.available() <= 0);
  delay(500);

  return error;
}


/* Generate CRC for any given request array */
uint16_t generateCRC(byte msg[], int len)
{
  uint16_t crc = 0xFFFF;
  
  for(int pos = 0 ; pos < len ; pos++) {
    /* XOR the byte into the least significant byte of crc */
    crc ^= (uint16_t)msg[pos];

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


/* Read responses from a read request */
void readResponse(int16_t values[], int numReg) {
  int pos = 3; 
  int len = 0;
  
  /* Store response bytes into a response array */
  int responseSize = SUNRISE_SERIAL.available();
  byte response [responseSize];
  for(int n = 0 ; n < responseSize ; n++) {
    response[n] = SUNRISE_SERIAL.read();
  }

  /* Combine bytes into words and store them in the array that is to be returned */
  /* The first 3 read response bytes have the same values as the request */
  while(len < numReg) {
    values[len] = ((int16_t)(int8_t) response[pos] << 8) | (uint16_t)response[pos + 1];
    pos = pos + 2;
    len++;
  }
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  /* Begin serial communication */
  Serial.begin(9600);
  SUNRISE_SERIAL.begin(9600);
  
  Serial.println("Initialization complete");

  /* Read sensor's configuration */
  readSensorConfig(SUNRISE_ADDR);
  /* Show results */
  Serial.print("MeasPeriod = ");
  Serial.print(meas_period_s);
  Serial.println(" sec");
  Serial.print("MeasNum = ");
  Serial.println(meas_numbers);
  Serial.print("MeasMode = ");
  Serial.println(meas_mode);
}


/* Read sensor's configuration */
void readSensorConfig(uint8_t target) {
  /* Request config settings */
  int error = requestRegisters((uint8_t) target /* communication address */, "HR" /* register type (HR or IR) */, (uint8_t) 0x0A /* starting register address */, (uint8_t) 3 /* number of registers */);

  if (error > 0) {
    /* Store responses into an array */
    int16_t response[3];
    readResponse(response, 3);

    /* Assign the values */
    meas_mode = response[0];
    meas_period_s = response[1];
    meas_numbers = response[2]; 
    
  }else {
    Serial.println("Failed to communicate with sensor");
  }
}


/* Read sensor's data*/
void readSensorData(uint8_t  target) {
  /* Request error status and CO2 values */
  int error = requestRegisters(target /* communication address */, "IR" /* register type (HR or IR) */, 0x00 /* starting register address */, 4 /* number of registers */);

  if (error > 0) {
    /* Store responses into an array */
    int16_t response[4];
    readResponse(response, 4);

    /* Assign the values */
    error_status = response[0];
    /* IR2 and IR3 (response[1] & response[2]) are reserved registers */
    co2_value = response[3]; 
    
  }else {
    Serial.println("Failed to communicate with sensor");
  }
}


/* Main loop */
void loop()
{
  static int pin_value = HIGH;
  
  /* Delay between readings */
  Serial.println("Waiting...");
  delay(MIN_READ_PERIOD_MS);

  /* Read sensor's data */
  readSensorData(SUNRISE_ADDR);

  /* Print values */
  Serial.print("CO2 Concentration: ");
  Serial.print(co2_value);
  Serial.print(" ppm (E: 0x");
  Serial.print(error_status, HEX);
  Serial.println(")");
  
  /* Indicate working state */
  digitalWrite(LED_BUILTIN, pin_value);
  pin_value  = ((pin_value == HIGH) ? LOW : HIGH);

}
