README
======

This README discusses issues unique to NuttX configurations for the
Arduino M0.  I used a compatible board called Wemos SAMD21 M0 board,
but there are other equivalent boards, like the RobotDyn SAMD21 M0.

Unfortunately because the Arduino.cc vs Arduino.org conflict in the past,
we have three types of boards: Arduino Zero, Arduino M0 and Arduino M0 Pro.

The Wemos SAMD21 M0 is compatible with Arduino M0, but not exactly a clone.

You have two options to program it: using the SWD (EDBG) connector that
comes in the board or the Arduino M0 bootloader that comes flashed on it.
Currently only SWD programming is supported.  Bootloader skip area should be
implemented to avoid overwriting the bootloader area.

The board uses the ATSAMD21G18A MCU and can work over the Native USB Port.

Contents
========

  - STATUS/ISSUES
  - LEDs
  - Serial Consoles
  - Configurations

STATUS/ISSUES
=============

  Because the Arduino M0 doesn't have a 12MHz crystal, it uses the internal
  RC oscillator.

LEDs
====

  There is one yellow LED available on the Arduino M0 and it can be turned
  on and off. The LED can be activated by driving the connected
  PA17 I/O line to high level.

  When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
  control the LED as follows:

    SYMBOL              Meaning                 LED0
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus is LED is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If LED is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Serial Consoles
===============

  SERCOM5
  ------

  SERCOM5 is available on pins PB22 (TXD) and PB23 (RXD). You will need to
  solder a two pins header to RXD and TXD labels, near to ICSP pin header.

    PIN   GPIO Function
    ----  ---- ------------------
     37   PB22 SERCOM5 / USART RX
     38   PB23 SERCOM5 / USART TX

  If you have a 3.3V USB/Serial adapter then this is the most convenient
  serial console to use (because you don't lose the console device each time
  you lose the USB connection).  It is the default in all of these
  configurations.  An option is to use the virtual COM port.

  Native USB Port
  ---------------

  You can access the NSH shell directly using the USB connector. All you need
  to do is use the "usbnsh" board profile.

Configurations
==============

  Each Arduino M0 configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh arduino-m0:<subdir>

  Before building, make sure the PATH environment variable include the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTE:  These configurations use the mconf-based configuration tool.  To
  change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

Configuration sub-directories
-----------------------------

  nsh:
    This configuration directory will built the NuttShell.  See NOTES above
    and below:

    NOTES:

    1. This configuration is set up to build on Windows using the Cygwin
       environment using the ARM EABI toolchain.  This can be easily
       changed as described above under "Configurations."

    2. By default, this configuration provides a serial console on SERCOM5
       at 115200 8N1 via RXD/TXD pads:

       PIN   EXT3 GPIO Function
       ----  ---- ------------------
        37   PB22 SERCOM5 / USART RX
        38   PB23 SERCOM5 / USART TX

  usbnsh:
    This configuration directory will build the NuttShell to work over USB.
    It uses the internal SAMD21 USB port working as CDC/ACM Serial/Modem.

    Using the configuration you don't need to solder the header pins RXD/TXD
    to get access the NSH terminal.
