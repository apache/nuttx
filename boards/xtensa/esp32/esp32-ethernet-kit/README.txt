README for the Espressif ESP32 Ethernet Kit
==============================================

  The ESP32 is a dual-core system from Espressif with two Harvard
  architecture Xtensa LX6 CPUs. All embedded memory, external memory and
  peripherals are located on the data bus and/or the instruction bus of
  these CPUs. With some minor exceptions, the address mapping of two CPUs
  is symmetric, meaning they use the same addresses to access the same
  memory. Multiple peripherals in the system can access embedded memory via
  DMA.

  The ESP32-Ethernet-Kit is an Ethernet-to-Wi-Fi development board that enables
  Ethernet devices to be interconnected over Wi-Fi. At the same time, to provide
  more flexible power supply options, the ESP32-Ethernet-Kit also supports power
  over Ethernet (PoE).

Buttons and LEDs
================

  Buttons
  -------
  There are two buttons labeled Boot and EN.  The EN button is not available
  to software.  It pulls the chip enable line that doubles as a reset line.

  The BOOT button is connected to IO0.  On reset it is used as a strapping
  pin to determine whether the chip boots normally or into the serial
  bootloader.  After reset, however, the BOOT button can be used for software
  input.

  LEDs
  ----
  There are several on-board LEDs for that indicate the presence of power
  and USB activity.  None of these are available for use by software.

Ethernet
========

  ESP32 has a 802.11 hardware MAC, so just connects to external PHY chip.
  Due to the limited number of GPIOs in ESP32, it's recommended to use RMII to
  connect to an external PHY chip. Current driver also only supports RMII option.

  The RMII GPIO pins are fixed, but the SMI and functional GPIO pins are optional.

  RMII GPIO pins are as following:

      ESP32 GPIO          PHY Chip GPIO
        IO25       <-->       RXD[0]
        IO26       <-->       RXD[1]
        IO27       <-->       CRS_DV
        IO0        <-->       REF_CLK
        IO19       <-->       TXD[0]
        IO21       <-->       TX_EN
        IO22       <-->       TXD[1]

  SMI GPIO pins (default option) are as following:

      ESP32 GPIO          PHY Chip GPIO
        IO18       <-->       MDIO
        IO23       <-->       MDC

  Functional GPIO pins(default option) are as following:

      ESP32 GPIO          PHY Chip GPIO
        IO5        <-->      Reset_N

Espressif has an offcial Ethernet development board:

  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-ethernet-kit.html

