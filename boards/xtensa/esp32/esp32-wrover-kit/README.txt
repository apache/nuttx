README for the Espressif ESP Wrover Kit
==============================================

  The ESP32 is a dual-core system from Espressif with two Harvard
  architecture Xtensa LX6 CPUs. All embedded memory, external memory and
  peripherals are located on the data bus and/or the instruction bus of
  these CPUs. With some minor exceptions, the address mapping of two CPUs
  is symmetric, meaning they use the same addresses to access the same
  memory. Multiple peripherals in the system can access embedded memory via
  DMA.

ESP-WROVER-KIT is an ESP32-based development board produced by Espressif.

ESP-WROVER-KIT features the following integrated components:
  ESP32-WROVER-B module
  LCD screen
  MicroSD card slot

Its another distinguishing feature is the embedded FTDI FT2232HL chip,
an advanced multi-interface USB bridge. This chip enables to use JTAG
for direct debugging of ESP32 through the USB interface without a separate
JTAG debugger. ESP-WROVER-KIT makes development convenient, easy, and
cost-effective.

Most of the ESP32 I/O pins are broken out to the board’s pin headers for easy access.

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
  and USB activity.
  There is an RGB LED available for software.

Configurations
==============

  nsh
  ---

  Basic configuration to run the NuttShell (nsh).

  wapi
  ___

  This is a congiuration to test the Wifi driver using WAPI.
  The Wifi passphrase and SSID can be configured from menuconfig, then once
  booted you can check if an IP address was assigned:
    nsh>  ifconfig

  If not configured at startup, you can connect to a network with the following:

    wapi psk wlan0 mypassword 1
    wapi essid wlan0 myssid 1

  gpio
  ---
  
  This is a test for the GPIO driver.  It includes the 3 LEDs and one,
  arbitrary, GPIO.  For this example, GPIO22 was used.
  At the nsh, we can turn LEDs on and off with the following.
    nsh> gpio -o 1 /dev/gpout0
    nsh> gpio -o 0 /dev/gpout1

  We can use the interrupt pin to send a signal when the interrupt fires:
    nsh> gpio -w 14 /dev/gpint3
  The pin is configured to as a rising edge interrupt, so after issuing the
  above command, connect it to 3.3V.

Using QEMU:
==========

First follow the instructions at https://github.com/espressif/qemu/wiki to build QEMU.
Enable the ESP32_QEMU_IMAGE config found in "Board Selection -> ESP32 binary image for QEMU".
Download the bootloader and the partition table from https://github.com/espressif/esp-nuttx-bootloader/releases
and place them in a directory, say ../esp-bins.
Build and generate the QEMU image: `make ESPTOOL_BINDIR=../esp-bins`
A new image "esp32_qemu_image.bin" will be created.  It can be run as:

 ~/PATH_TO_QEMU/qemu/build/xtensa-softmmu/qemu-system-xtensa -nographic \
    -machine esp32 \
    -drive file=esp32_qemu_image.bin,if=mtd,format=raw
 
External devices:
=================

  BMP180
  ------

  When using BMP180 (enabling CONFIG_SENSORS_BMP180), it's expected this device is wired to I2C0 bus.
