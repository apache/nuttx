README
^^^^^^

This port conributed by jeditekunum.

This is the README file for the port of NuttX to the MoteinoMEGA from
LowPowerLab (http://www.lowpowerlab.com).  The MoteinoMEGA is based
on an Atmel ATMega1284P.  As of this writing, documentation for the
MoteinoMEGA board is available here:

https://lowpowerlab.com/shop/index.php?_route_=Moteino/moteinomega

Contents
^^^^^^^^

  o MoteinoMEGA Features
  o Pin Connections
  o DualOptiboot Bootloader
  o Toolchains
  o MoteinoMEGA Configuration Options
  o Configurations

MoteinoMEGA Features
^^^^^^^^^^^^^^^^^^^^

   o  16MHz ATmega1284P Atmel 8bit AVR RISC Processor
   o  128Kbyte Flash
   o  16Kbyte RAM
   o  4Kbyte EEPROM
   o  2 High Speed Serial Ports
   o  8Ch 10bit Analog Input port

Pin Connections
^^^^^^^^^^^^^^^

  -------------------- -----------------------------
  ATMega1284P Pinout   MoteinoMEGA board connection
  -------------------- -----------------------------
  (left)
   1 AREF              AR
   2 PA7               A7
   3 PA6               A6
   4 PA5               A5
   5 PA4               A4
   6 PA3               A3
   7 PA2               A2
   8 PA1               A1
   9 PA0               A0
  10 PB0               0
  11 PB1               1
  12 PB2 (INT2)        2/i2  (used by optional radio)
  13 PB3 (PWM0)        3
  14 PB4 (PWM1/SS)     4/SS  (used by optional radio)
  15 PB5 (MOSI)        5/MO  (used by optional radio/flash)
  16 PB6 (MISO/PWM2)   6/MI  (used by optional radio/flash)
  17 PB7 (SCK/PWM3)    7/SCK (used by optional radio/flash)
  18 VOUT              3v3
  19 VIN               VIN
  20 GND               GND
  (bottom)
  21 DTR/RTS           DTR
  22 TX0               v
  23 RX0               ^
  24 VIN
  25
  26 GND               GND
  (right)
  27 GND               GND
  28 VIN               VIN
  29 VOUT              3v3
  30 RESET             RST
  31 PD0 (RX0)         8/Serial 0 ^
  32 PD1 (TX0)         9/Serial 0 v
  33 PD2 (RX1/INT0)    10/Serial 0 ^/i0
  34 PD3 (TX1/INT1)    11/Serial 1 v/i1
  35 PD4 (PWM4)        12
  36 PD5 (PWM5)        13
  37 PD6 (PWM6)        14
  38 PD7 (PWM7)        15/LED
  39 PC0 (SCL)         16/SCL
  40 PC1 (SDA)         17/SDA
  41 PC2 (TCK)         18
  42 PC3 (TMS)         19
  43 PC4 (TDO)         20
  44 PC5 (TDI)         21
  45 PC6               22
  46 PC7               23 (used by optional flash)


DualOptiboot Bootloader
^^^^^^^^^^^^^^^^^^^^^^^

o FTDI (or similar) USB-To-Serial converter with compatible connector
  configured for DTR (AdaFruit, SparkFun, etc)
o Obtain ard-reset-arduino Python script
  (one source: https://github.com/mikaelpatel/Cosa/blob/master/build/Arduino-Makefile/bin/ard-reset-arduino)
  This script triggers the DTR pin to enter bootloader mode.
o Obtain avrdude for your platform.

Bootloader operates at 115200 baud.  It would be useful to create a short script
that invokes ard-reset-arduino and then avrdude to load program.  This script
could then also, optionally, invoke miniterm.py or some other serial interface
program for console.

Example:

APP=nuttx
CPU=atmega1284p
BAUD=115200
PORT=/dev/tty.usbserial-A703X8PQ
avr-size --mcu=$CPU -C --format=avr $APP
ard-reset-arduino --verbose $PORT
avrdude -q -V -p $CPU -C {location-of-avrdude.conf} -D -c arduino -b $BAUD \
  -P $PORT -U flash:w:${APP}.hex:i
miniterm.py --port=$PORT --baud=$BAUD -q --lf

Toolchains
^^^^^^^^^^

The toolchain may be selected using the kconfig-mconf tool (via 'make menuconfig'),
by editing the existing configuration file (defconfig), or by overriding
the toolchain on the make commandline with CONFIG_AVR_TOOLCHAIN=<toolchain>.

The valid values for <toolchain> are BUILDROOT, CROSSPACK, LINUXGCC and WINAVR.

This port was tested using the OS X / CROSSPACK tool chain, GCC version 4.8.1.
Please see other NuttX documentation for toolchain details.

MoteinoMEGA Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=avr

    CONFIG_ARCH_FAMILY - For use in C code:

       CONFIG_ARCH_FAMILY=avr

    CONFIG_ARCH_FAMILY_family - For use in C code:

       CONFIG_ARCH_FAMILY_AVR=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=atmega

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_ATMEGA1284P=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=moteino-mega

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_MOTEINO_MEGA=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_RAM_SIZE - Describes the installed DRAM.  One of:

       CONFIG_RAM_SIZE=(16*1024) - (16Kb)

    CONFIG_RAM_START - The start address of installed SRAM

       CONFIG_RAM_START=0x800100

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    Individual subsystems can be enabled:

       CONFIG_AVR_INT0=n
       CONFIG_AVR_INT1=n
       CONFIG_AVR_INT2=n
       CONFIG_AVR_INT3=n
       CONFIG_AVR_INT4=n
       CONFIG_AVR_INT5=n
       CONFIG_AVR_INT6=n
       CONFIG_AVR_INT7=n
       CONFIG_AVR_WDT=n
       CONFIG_AVR_TIMER0=n
       CONFIG_AVR_TIMER1=n
       CONFIG_AVR_TIMER2=n
       CONFIG_AVR_TIMER3=n
       CONFIG_AVR_SPI=n
       CONFIG_AVR_USART0=y
       CONFIG_AVR_USART1=n
       CONFIG_AVR_ADC=n
       CONFIG_AVR_ANACOMP=n
       CONFIG_AVR_TWI=n

   If the watchdog is enabled, this specifies the initial timeout.  Default
  is maximum supported value.

      CONFIG_WDTO_15MS
      CONFIG_WDTO_30MS
      CONFIG_WDTO_60MS
      CONFIG_WDTO_120MS
      CONFIG_WDTO_1250MS
      CONFIG_WDTO_500MS
      CONFIG_WDTO_1S
      CONFIG_WDTO_2S
      CONFIG_WDTO_4S
      CONFIG_WDTO_8S

 ATMEGA specific device driver settings

    CONFIG_USARTn_SERIAL_CONSOLE - selects the USARTn for the
       console and ttys0 (default is the USART0).
    CONFIG_USARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_USARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_USARTn_BAUD - The configure BAUD of the USART.  Must be
    CONFIG_USARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_USARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_USARTn_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

Common Configuration Notes
--------------------------

  1. Each MoteinoMEGA configuration is maintained in a sub-directory and
     can be selected as follow:

       tools/configure.sh moteino-mega/<subdir>

     Where <subdir> is one of the configuration sub-directories described in
     the following paragraph.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

Configuration Sub-Directories
-----------------------------

  hello:
    The simple apps/examples/hello "Hello, World!" example.

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interfaces.
