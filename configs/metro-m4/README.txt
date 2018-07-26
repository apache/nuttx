README
======

  This directory contains the port of NuttX to the Adafruit Metro M4.  The
  Metro M4 uses a Arduino form factor and and pinout.  It's powered with an
  ATSAMD51J19:

  o Cortex M4 core running at 120 MHz
  o Hardware DSP and floating point support
  o 512 KB flash, 192 KB RAM
  o 32-bit, 3.3V logic and power
  o Dual 1 MSPS DAC (A0 and A1)
  o Dual 1 MSPS ADC (8 analog pins)
  o 6 x hardware SERCOM (I2C, SPI or UART)
  o 16 x PWM outputs
  o Stereo I2S input/output with MCK pin
  o 10-bit Parallel capture controller (for camera/video in)
  o Built in crypto engines with AES (256 bit), true RNG, Pubkey controller
  o 64 QFN

Serial Console
==============

  An Arduino compatible serial Shield is assumed (or equivalently, and
  external RS-232 or serial-to-USB adapter connected on Arduino pins D0 and
  D1):

    ------ ----------------- -----------
    SHIELD SAMD5E5           FUNCTION
    ------ ----------------- -----------
    D0     PA23 SERCOM3 PAD2 RXD
    D1     PA22 SERCOM3 PAD0 TXD

LEDs
====

  The Adafruit Metro M4 has four LEDs, but only two are controllable by software:

    1. The red LED on the Arduino D13 pin, and
    2. A NeoPixel RGB LED.

  Currently, only the red LED is supported.

    ------ ----------------- -----------
    SHIELD SAMD5E5           FUNCTION
    ------ ----------------- -----------
    D13    PA16              GPIO output
