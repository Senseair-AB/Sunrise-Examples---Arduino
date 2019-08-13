# Senseair Sunrise Examples - Arduino

Examples built in Arduino for the Senseair Sunrise sensor

## Application Description

These examples show how to implement I2C and Modbus communication with a Senseair Sunrise Sensor, using the Arduino IDE.

The I2C and Modbus examples uses the DSS Circuits Arduino I2C Master(www.dsscircuits.com) and SoftwareSerial libraries, 
where the first is only used in the I2C examples and the latter is used in all but one of the I2C examples.

### Hardware and Software Enviroment
- The programs has been tested using the Arduino Mega 2560 board.
- The programs are built using the Arduino IDE (www.arduino.cc).

## Directory Contents

 - sunrise-examples-arduino/examlpes/modbus/		Example for Modbus communication
 - sunrise-examples-arduino/examlpes/i2c/			Examples for I2C communication
 - sunrise-examples-arduino/libraries/I2C			The DSS Circuits Arduino I2C Master Library
 - sunrise-examples-arduino/wiring/                 Pictures of the wiring for the examples

### How to Run the Examples

In order to make the examples work, you must do the following:
 - Place the DSS Circuits Arduino I2C Master library in your Arduino IDE's library folder
 - Open one of the examlpes in the Arduino IDE
 - Upload the file into the target memory
 - Run the example

## Author

Senseair FW Team