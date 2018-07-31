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

STATUS
======

  2018-07-26:  The basic port was merged into master.  It is still
    incomplete and untested.  It is missing the clock configuration logic.
    There is a placeholder from the SAML21, but it is currently stubbed out
    in the Make.defs file.  Configuration options in the board.h header
    file are bogus and also just cloned from the SAML21.
  2018-07-29:  Clock configuration logic now complete.  board.h
    configuration options still need to be verified.  Unverified SERCOM
    USART, SPI, I2C, Port configuration, and DMA support have been added.
    I still have no hardware in hand to test.
  2018-07-20:  Brought in the USB driver from the SAML21.  It is the same
    USB IP with only small differences.  There a a few, small open issues
    still to be resolved.

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

Configurations
==============

  Each Adafruit Metro M4 configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] metro-m4/<subdir>

  Do 'tools/configure.sh -h' for the list of options.  If you are building
  under Windows with Cygwin, you would need the -c option, for example.

  Before building, make sure that the PATH environmental variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of configurations listed in the following paragraph.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output of on SERCOM3 which is available on a Arduino Serial
     Shield (see the section "Serial Console" above).

  3. Unless otherwise stated, the configurations are setup build under
     Linux with a generic ARM EABI toolchain:

Configuration sub-directories
-----------------------------

  nsh:
    This configuration directory will built the NuttShell.  See NOTES above.
