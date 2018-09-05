Custom firmware for the MakeBlock Me High Power Motor Driver board (https://store.makeblock.com/me-high-power-encoder-motor-driver) with the purpose of removing the high pitched noise caused by the PWM frequency.

**Makes use of following libaries:**

PID by Brett Beauregard
https://github.com/br3ttb/Arduino-PID-Library

Encoder by Paul Stoffregen
https://www.pjrc.com/teensy/td_libs_Encoder.html


## Details
This firmware is for the Atmega328P (Arduino) that is found on the actual driver board. **Not** the Orion or any other of the arduino-versions from Makeblock. The motor driver board is shipped with a firmware on the atmega328P which is made by Makeblock. We were dissatisfied with some of the properties of this firmware, and decided to make our own. Makeblock has open sourced their firmware and we used that as a starting point for this code base.

## Important notes
- This firmware doesn't implement all the functionality of the original firmware made by the people from makeblock. Only the parts needed for our specific project is implemented (feel free to contribute and extend if you want to).
Because of this, this repository can't be seen as a replacement of the original firmware. Look at the source code and see if the functionality is what you need. This firmware only support I2C (not serial) and only the command for setting speed.
- The firmware will retrieve it's I2C address from EEPROM on every startup. If there isn't any stored in EEPROM the comunication with the ORION board will fail.

## Install
You need a USB-to-serial converter. There are pinouts on the motor board for programming the Atmega328P. You need to connect RX, TX, 5V and GND. Then upload the code to the motor board with the arduino software.

## Based on

Based on the original MakeBlock firmware, but not much is left of it.
https://github.com/Makeblock-official/Me_EncodeDriver/tree/master/EncodeDriver-HighPower/slave
