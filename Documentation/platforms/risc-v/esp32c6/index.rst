==================
Espressif ESP32-C6
==================

The ESP32-C6 is an ultra-low-power and highly integrated SoC with a RISC-V
core and supports 2.4 GHz Wi-Fi 6, Bluetooth 5 (LE) and the 802.15.4 protocol.

* Address Space
  - 800 KB of internal memory address space accessed from the instruction bus
  - 560 KB of internal memory address space accessed from the data bus
  - 1016 KB of peripheral address space
  - 8 MB of external memory virtual address space accessed from the instruction bus
  - 8 MB of external memory virtual address space accessed from the data bus
  - 480 KB of internal DMA address space
* Internal Memory
  - 320 KB ROM
  - 512 KB SRAM (16 KB can be configured as Cache)
  - 16 KB of SRAM in RTC
* External Memory
  - Up to 16 MB of external flash
* Peripherals
  - 35 peripherals
* GDMA
  - 7 modules are capable of DMA operations.

ESP32-C6 Toolchain
==================

A generic RISC-V toolchain can be used to build ESP32-C6 projects.
SiFive's toolchain can be downloaded from: https://github.com/sifive/freedom-tools/releases

Second stage bootloader and partition table
===========================================

The NuttX port for now relies on IDF's second stage bootloader to carry on some hardware
initializations.  The binaries for the bootloader and the partition table can be found in
this repository: https://github.com/espressif/esp-nuttx-bootloader
That repository contains a dummy IDF project that's used to build the bootloader and
partition table, these are then presented as Github assets and can be downloaded
from: https://github.com/espressif/esp-nuttx-bootloader/releases
Download ``bootloader-esp32c6.bin`` and ``partition-table-esp32c6.bin`` and place them
in a folder, the path to this folder will be used later to program them. This
can be: ``../esp-bins``

Building and flashing
=====================

First make sure that ``esptool.py`` is installed.  This tool is used to convert
the ELF to a compatible ESP32-C6 image and to flash the image into the board.
It can be installed with: ``pip install esptool``.

Configure the NUttX project: ``./tools/configure.sh esp32c6-devkit:nsh``
Run ``make`` to build the project.  Note that the conversion mentioned above is
included in the build process.  
The `esptool.py` command to flash all the binaries is::

     esptool.py --chip esp32c6 --port /dev/ttyUSBXX --baud 921600 write_flash 0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 nuttx.bin

However, this is also included in the build process and we can build and flash with::

   make flash ESPTOOL_PORT=<port> ESPTOOL_BINDIR=../esp-bins

Where ``<port>`` is typically ``/dev/ttyUSB0`` or similar and ``../esp-bins`` is 
the path to the folder containing the bootloader and the partition table
for the ESP32-C6 as explained above.
Note that this step is required only one time.  Once the bootloader and partition
table are flashed, we don't need to flash them again.  So subsequent builds
would just require: ``make flash ESPTOOL_PORT=/dev/ttyUSBXX``
