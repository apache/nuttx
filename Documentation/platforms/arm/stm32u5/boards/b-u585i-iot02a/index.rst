=================
ST B-U585I-IOT02A
=================

This page discusses the port of NuttX to the STMicroelectronics
B-U585I-IOT02A board.  That board features the STM32U585AII6QU MCU with 2MiB
of Flash and 768KiB of SRAM.

Status
======

2022-02-13: With TrustedFirmware-M from STM32CubeU5 and signing the Apache
NuttX binary image to get a tfm_ns_init.bin, the board now boots and the
basic NSH configuration works with Apache NuttX as the OS running in the
non-secure world.

2022-04-03: The dependency on TrustedFirmware-M was dropped.  I.e. the
b-u585i-iot02a:nsh configuration now runs standalone.

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

Arduino Connector
=================

CN13 / SPI1 / D10 - D13

    ========= ====
    FUNC      GPIO
    ========= ====
    SPI1_NSS  PE12
    SPI1_SCK  PE13
    SPI1_MISO PE14
    SPI1_MOSI PE15
    ========= ====

Serial Consoles
===============

Virtual COM Port on USART1
--------------------------

Default board is configured to use USART1 as console.  USART1 is connected
to the ST-LINKV3E Virtual COM port as well as made available on connector
CN9.

Pins and Connectors:

    ==== ====   ====== ========
    FUNC GPIO   Pin    NAME
    ==== ====   ====== ========
    TXD: PA9    CN9 14 T.VCP_TX
    RXD: PA10   CN9 13 T.VCP_RX
    ==== ====   ====== ========

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each configuration is maintained in a sub-directory and can be
selected as follow::

    tools/configure.sh b-u585i-iot02a:<subdir>

Before building, make sure the PATH environment variable includes the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.::

    make oldconfig
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
   output on USART3, as described above under "Serial Console".  The
   elevant configuration settings are listed below::

         CONFIG_STM32L5_USART3=y
         CONFIG_STM32L5_USART3_SERIALDRIVER=y
         CONFIG_STM32L5_USART=y

         CONFIG_USART3_SERIALDRIVER=y
         CONFIG_USART3_SERIAL_CONSOLE=y

         CONFIG_USART3_RXBUFSIZE=256
         CONFIG_USART3_TXBUFSIZE=256
         CONFIG_USART3_BAUD=115200
         CONFIG_USART3_BITS=8
         CONFIG_USART3_PARITY=0
         CONFIG_USART3_2STOP=0

3. All of these configurations are set up to build under Linux using the
   "GNU Tools for ARM Embedded Processors" that is maintained by ARM
   (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

   That toolchain selection can easily be reconfigured using
   'make menuconfig'.  Here are the relevant current settings:

   Build Setup::
       CONFIG_HOST_LINUX=y                 : Linux environment

   System Type -> Toolchain::
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

nsh:
----

Configures the NuttShell (nsh) located at examples/nsh.  This
configuration is focused on low level, command-line driver testing.
