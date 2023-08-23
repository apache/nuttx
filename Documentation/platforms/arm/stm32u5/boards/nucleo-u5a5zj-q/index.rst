===============
NUCLEO-U5A5ZJ-Q
===============

This README file discusses the port of NuttX to the STMicroelectronics
NUCLEO-U5A5ZJ-Q board. That board features the STM32U5A5ZJT6Q MCU with 4MiB
of Flash and 2500KiB of SRAM.
  
Status
======

2023-07-16: Initial port - works all ram memory and i2c(not extensively tested)
The i2c driver is based on stm32l4 one

Clock Source
============

Only the low speed external (LSE) 32.768kHz crystal (X2) is installed in
default configurations.

    ========= ====
    FUNC      GPIO
    ========= ====
    OSC32_IN  PC14
    OSC32_OUT PC15
    ========= ====

Serial Consoles
===============

Virtual COM Port on USART1
--------------------------

Default board is configured to use USART1 as console.  USART1 is connected
to the ST-LINKV3E Virtual COM port as well as made available on connector
CN10 (need some rework on PCB).

Pins and Connectors:

    ==== ====   ======= ========
    FUNC GPIO   Pin     NAME
    ==== ====   ======= ========
    TXD: PA9    CN10 14 T.VCP_TX
    RXD: PA10   CN9 13  T.VCP_RX
    ==== ====   ======= ========

Configurations
==============

Information Common to All Configurations
----------------------------------------

There is only one configuration which can be selected as follow::

    tools/configure.sh nucleo-u5a5zj-q:nsh

Before building, make sure the PATH environment variable includes the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.::

    make menuconfig 
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

2. All of these configurations are set up to build under Linux using the
   "GNU Tools for ARM Embedded Processors" that is maintained by ARM
   (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

