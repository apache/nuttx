===================
ST STM32H747I-DISCO
===================

This page discusses issues unique to NuttX configurations for the
STMicro STM32H747I-DISCO development board featuring the STM32H747I
MCU. The STM32H747I is a Cortex-M7 and -M4 dual core with 2MBytes Flash
memory and 1MByte SRAM. The board features:

- On-board ST-Link v3E for programming and debugging,
- 4 color user LEDs
- One pushbuttons (user and reset)
- Four-way joystick controller with select key
- 32.768 kHz crystal oscillator
- USB OTG HS with Micro-AB connectors
- Ethernet connector compliant with IEEE-802.3-2002
- Board connectors:
  - Camera
  - USB with Micro-AB
  - SWD
  - Ethernet RJ45
  - Arduino Uno V3
  - Pmod
  - STMod+

Refer to the http://www.st.com website for further information about this
board (search keyword: STM32H747I-DISCO)

Serial Console
==============

1. Virtual Console.

   The virtual console uses Serial Port 1 (USART1).

      ================= ===
      VCOM Signal       Pin
      ================= ===
      SERIAL_RX         PA10
      SERIAL_TX         PA9
      ================= ===

      These signals are internally connected to the on board ST-Link.

   The virtual console is the default serial console in all
   configurations unless otherwise stated in the description of the
   configuration.

Configurations
==============

Information Common to All Configurations
----------------------------------------

Each STM32H747I-DISCO configuration is maintained in a sub-directory and
can be selected as follow::

    tools/configure.sh [options] stm32h747i-disco:<subdir>

Where options should specify the host build platform (-l for Linux, -c for
Cygwin under Windows, etc.).  Try 'tools/configure.sh -h' for the complete
list of options.

Before starting the build, make sure that (1) your PATH environment variable
includes the correct path to your toolchain, and (2) you have the correct
toolchain selected in the configuration.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.::

    make

The <subdir> that is provided above as an argument to the tools/configure.sh
must be is one of the following.

NOTES:

1. These configurations use the mconf-based configuration tool.  To
   change any of these configurations using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute 'make menuconfig' in nuttx/ in order to start the
      reconfiguration process.

2. Unless stated otherwise, all configurations generate console
     output on the ST-Link VCOM, USART1.

3. Unless otherwise stated, the configurations are setup for Linux by
   default:

     Build Setup:
       CONFIG_HOST_LINUX=y                     : Linux host operating system

4. All of these configurations use the general arm-none-eabi toolchain for
   Linux  That toolchain selection can easily be reconfigured using 'make
   menuconfig'.

5. These configurations all assume that you are loading code using
   something like the ST-Link v3E JTAG.  None of these configurations are
   setup to use the DFU bootloader but should be easily reconfigured to
   use that bootloader if so desired.

Configuration Sub-directories
-----------------------------

nsh:
----

This configuration provides a basic NuttShell configuration (NSH)
for the board.  The default console is the VCOM on USART1.
