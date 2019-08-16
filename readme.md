# Senseair Sunrise Examples - Arduino

Examples built in Arduino for the Senseair Sunrise sensor

## Application Description

These examples show how to implement I2C and Modbus communication with a Senseair Sunrise Sensor, using the Arduino IDE.

The I2C and Modbus examples uses Wire and SoftwareSerial libraries, 
where the first is only used in the I2C examples and the latter is used in all but one of the I2C examples.

### Hardware and Software Enviroment
- The programs has been tested using the Arduino Mega 2560 board.
- The programs are built using the Arduino IDE (www.arduino.cc).

## Directory Contents

 - sunrise-examples-arduino/examlpes/modbus/		Example for Modbus communication (connection diagram - [wiring/sunrise_modbus_wiring.pdf](wiring/sunrise_modbus_wiring.pdf))
 - sunrise-examples-arduino/examlpes/i2c/			Examples for I2C communication (connection diagram - [wiring/sunrise_i2c_wiring.pdf](wiring/sunrise_i2c_wiring.pdf))
 - sunrise-examples-arduino/wiring/                 Pictures of the wiring for the examples

### How to Run the Examples

In order to make the examples work, you must do the following:
 - Open one of the examlpes in the Arduino IDE
 - Upload the file into the target memory
 - Run the example

## Author

Senseair FW Team