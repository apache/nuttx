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

MEMX
^^^^

  If you use the GCC AVR toolchain from the Atmel Studio, then you can
  enable suppport for the MEMX storage:

    CONFIG_AVR_HAS_MEMX_PTR=y

  If this support is enabled, then all strings will be saved in FLASH and
  standard string-oriented interfaces such printf() will change so that
  they accept memx pointers.

  This means that (1) ALL strings must lie in FLASH, and (2) since the
  strings are moved from SRAM to FLASH, you will save a LOT of SRAM usage
  in some configurations that use a lot of string memory (such as the
  ostest and nsh configurations).
