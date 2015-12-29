README
^^^^^

This is the README file for the port of NuttX to the Arduino Mega 2560 Rev3.

https://www.arduino.cc/en/Main/ArduinoBoardMega2560

The board is based on ATMega2560 chip from Atmel

http://www.atmel.com/devices/atmega2560.aspx


Toolchain
^^^^^^^^^

Right now only Atmel's AVR8 Toolchain is supported. You can get it from

http://www.atmel.com/tools/atmelavrtoolchainforwindows.aspx

It is basically WinAVR compatible so sub-projects may define WinAVR as a
tool-chain but specify path to the Atmel AVR8 in path. See
arduino-mega2560/hello for example.

