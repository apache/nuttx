================
ST Nucleo H723ZG
================

.. tags:: chip:stm32, chip:stm32h7, chip:stm32h723

This page discusses issues unique to NuttX configurations for the
STMicro Nucleo-H723ZG development board featuring the STM32H723ZG
MCU. The STM32H723ZG is a up-to 550MHz Cortex-M7 operation with 1MByte Flash
memory and 564KBytes of SRAM. The board features:

- On-board ST-LINK/V2-1 for programming and debugging,
- 3 user LEDs
- Two pushbuttons (user and reset)
- 32.768 kHz crystal oscillator
- USB OTG HS with Micro-AB connector
- Ethernet connector compliant with IEEE-802.3-2002
- Board connectors:
  - USB with Micro-AB
  - SWD
  - Ethernet RJ45
  - ST Zio connector including Arduino Uno V3
  - ST morpho

Refer to the http://www.st.com website for further information about this
board (search keyword: Nucleo-H723ZG)

Serial Console
==============

Many options are available for a serial console via the Morpho connector.
Here two common serial console options are suggested:

1. Arduino Serial Shield.

   If you are using a standard Arduino RS-232 shield with the serial
   interface with RX on pin D0 and TX on pin D1 from USART6:

      ======== ========= =====
      ARDUINO  FUNCTION  GPIO
      ======== ========= =====
      DO RX    USART6_RX PG9
      D1 TX    USART6_TX PG14
      ======== ========= =====

2. Nucleo Virtual Console.

   The virtual console uses Serial Port 3 (USART3) with TX on PD8 and RX on
   PD9.

      ================= ===
      VCOM Signal       Pin
      ================= ===
      SERIAL_RX         PD9
      SERIAL_TX         PD8
      ================= ===

   These signals are internally connected to the on board ST-Link.

   The Nucleo virtual console is the default serial console in all
   configurations unless otherwise stated in the description of the
   configuration.

Configurations
==============

Information Common to All Configurations
----------------------------------------

Each Nucleo-H723ZG configuration is maintained in a sub-directory and
can be selected as follow::

    tools/configure.sh [options] nucleo-h723zg:<subdir>

Where options should specify the host build platform (-l for Linux, -c for
Cygwin under Windows, etc.).  Try ``tools/configure.sh -h`` for the complete
list of options.

Before starting the build, make sure that (1) your PATH environment variable
includes the correct path to your toolchain, and (2) you have the correct
toolchain selected in the configuration.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.::

    make

Configuration Sub-directories
=============================

nsh
---

This configuration provides a basic NuttShell configuration (NSH)
for the Nucleo-H723ZG.  The default console is the VCOM on USART3.

netnsh
------

This configuration enables support for the Ethernet and a set
of networking functionalities.

.. NOTE::
    The initialization logic waits for Ethernet auto negotiation to complete
    up to a timeout. The timeout is a number of times the MII status register
    is read from the PHY. First to verify the link status is up, subsequently
    to verify the auto negotiation is complete. The timeout is set to a fixed
    value of ``PHY_RETRY_TIMEOUT=0x1998``.
    In each cycle the program waits for ``nxsched_usleep(100)``. By default
    this gets rounded up to the value of the tick time which is 10 ms!
    Therefore the time you will wait for the nsh console if the Ethernet
    is not plugged in is ``2 * 10 ms * 0x1998 = 130s``.

oa_tc6
------
  This configuration features the network driver for 10BASE-T1S and 10BASE-T1L SPI MAC-PHYs
  that follow the `OPEN Alliance 10BASE-T1x MAC-PHY Serial Interface` specification (OA-TC6).

  Among such MAC-PHYs are e.g. Microchip LAN865x, Onsemi NCV7410 (NCN26010), Analog Devices ADIN1110.
  See the build configuration utility (e.g. ``make menuconfig``) to find out which ones are currently supported.

  The OA-TC6 defines a 5 signal connection between the MAC-PHY and the host MCU. These are 4 lines for the standard SPI and 1 line for the interrupt signal from the MAC-PHY to the MCU.

  **Default pinout**

  ============ ========== =========================================
  Nucleo Pin   Function   Description
  ============ ========== =========================================
  PF3          INT        MAC-PHY interrupt signal
  PB4          MISO       SPI Master In Slave Out
  PB5          MOSI       SPI Master Out Slave In
  PB3          CLK        SPI Clock
  PD14         CS         SPI Chip Select
  ============ ========== =========================================

  The ``oa_tc6`` configuration is additionally equipped with the ``plcatool`` utility. This allows configuration of the Physical Layer Collision Avoidance (PLCA) functionality
  in 10BASE-T1S PHYs.
