# Sunrise Examples - Arduino

/**
  @page Senseair Sunrise Sensor with Arduino Examples
 
  @verbatim
  ******************************************************************************
  * @file    sunrise-examples-arduino/readme.txt
  * @author  Senseair FW team 
  * @brief   Senseair Sunrise Sensor with Arduino examples using I2C and Modbus communication
  ******************************************************************************
  @endverbatim

@par Application Description

How to implement communication with Senseair Sunrise sensor with Arduino using I2C or Modbus.

These applications demonstrates easy ways to use Arduino to communicate with the Senseair Sunrise sensor using either the I2C or 
Modbus communication protocols.

@note The I2C and Modbus examples uses the DSS Circuits Arduino I2C Master(http://dsscircuits.com/articles/arduino-i2c-master-library) 
	  and SoftwareSerial libraries, where the first is only used in the I2C examples and the latter is used in all but one of the I2C 
	  examples.




@par Directory contents
    - sunrise-examples-arduino/examlpes/modbus/sunrise_modbus_continuous/sunrise_modbus_continuous.ino	Example for Modbus in continuous mode
    - sunrise-examples-arduino/examlpes/modbus/sunrise_modbus_single/sunrise_modbus_single.ino			Example for Modbus in single mode
    - sunrise-examples-arduino/examlpes/i2c/sunrise_i2c_continuous/sunrise_i2c_continuous.ino			Example for I2C in continuous mode
    - sunrise-examples-arduino/examlpes/i2c/sunrise_i2c_single/sunrise_i2c_single.ino					Example for I2C in single mode
	- sunrise-examples-arduino/libraries/I2C															The DSS Circuits Arduino I2C Master Library
	- sunrise-examples-arduino/wiring/sunrise_i2c_continuous.pdf                                        Wiring for the I2C in continuous mode example
	- sunrise-examples-arduino/wiring/sunrise_i2c_single.pdf                                            Wiring for the I2C in single mode example
	- sunrise-examples-arduino/wiring/sunrise_modbus_continuous.pdf                                     Wiring for the Modbus in continuous mode example
	- sunrise-examples-arduino/wiring/sunrise_modbus_single.pdf                                         Wiring for the Modbus in single mode example

@par Hardware and Software environment

  - This application runs on Arduino Mega 2560.
    

@par How to use it ?

In order to make the program work, you must do the following:
 - Place the DSS Circuits Arduino I2C Master library in your Arduino IDE's library folder 
 - Open one of the examlpes in the Arduino IDE
 - Upload the file into the target memory
 - Run the example

 */