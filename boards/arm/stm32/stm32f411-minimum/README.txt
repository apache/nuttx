README
======

This README discusses issues unique to NuttX configurations for the
WeAct Studio MiniF4 minimum system development board.

Contents
========

  - Board information
  - LEDs
  - UARTs
  - USB
  - SPI NOR Flash
  - Configurations

Board information
=================

It is sometimes referred to as "black pill", although there are several clone
boards with f103-like chips and even an official f103 blackpill from Robotdyn.
Both WeAct Studio (via Taobao and Aliexpress) and Adafruit sell the boards as
"BlackPill Core Board" with STM32F411CEU6 or STM32F401CCU6 or
"STM32F411 BlackPill Development board", so we'll assume that
WeAct Studio is the original manufacturer and F401/F411 is the chip.

Board documentation:
https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1

Summary pages from STM32-base
https://stm32-base.org/boards/STM32F401CCU6-WeAct-Black-Pill-V1.2.html
https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html
https://stm32-base.org/boards/STM32F401CEU6-WeAct-Black-Pill-V3.0.html

The board features:

  - On-board 64 Mbits (8 MBytes) External SPI-NOR Flash (optional),
  - nRST reset button and BOOT0 ST BootROM entry button,
  - One user LED and one user push-button,
  - HSE 25 Mhz and LSE 32.768 kHz,
  - USB OTG FS with micro-AB connector,
  - Around 30 remappable GPIOs on 2.54mm headers (after excluding 7 power pins,
    two LSE pins, the LED pin, NRST, BOOT1 and the SWD header),
  - Serial Wire Debug header for use with an external SWD/JTAG adapter.

As F4 series have a USB DFuSe-capable BootROM [AN2606], the board can be flashed
via `dfu-util` over USB, or via `stm32flash` over UART without any debuggers.

LEDs
====

  The STM32F411 Minimum board has only one software controllable LED on PC13.
  This LED can be used by the board port when CONFIG_ARCH_LEDS option is
  enabled.

  If enabled the LED is simply turned on when the board boots
  successfully, and is blinking on panic / assertion failed.

UARTs
=====

  UART/USART PINS
  ---------------

  USART1
    TX      PA9
    RX      PA10
  USART2
    CTS     PA0
    RTS     PA1
    TX      PA2
    RX      PA3
    CK      PA4

Default USART/UART Configuration
--------------------------------

  USART1 (RX & TX only) is available through pins PA9 (TX) and PA10 (RX).

USB
===

The board routes F401/F411's built-in FS PHY to a USB-C (non-dual-role) port.
The chips still have a DWC2 USBOTG IP core, but only device mode is used.
Note that only V3.1 PCB got a DNI solder bridge to short 5V and USB VBUS.
Previous revisions had no support for USB host because of lack of port power
as well as 5k pull-downs on USB-C CC1/CC2 pins (not affecting USB-A adapters).
Because of this we are not considering USBOTG-FS Host configurations and
do not map OTG_VBUS, OTG_ID pins to FS PHY, also no OVER/PWRON GPIOs.

SPI NOR Flash
=============

The seller may send boards with soldered on-board SPI NOR Flash, usually
Winbond W25Q64JVSIQ (QuadSPI) or similar. F401/F411 lack QuadSPI support.
NuttX supports such MTD storage via special drivers and filesystems.
For example, external flash can be formatted with SmartFS (or NXFFS, LittleFS)
and mounted on boot by board init code or interactively from NSH.

Configurations
==============

Each stm32f411-minimum configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh stm32f411-minimum:<subdir>

  Where <subdir> is one of the following:


  Configuration Directories
  -------------------------

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh. This
    configuration enables a serial console on UART1.
