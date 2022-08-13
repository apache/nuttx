========
rp2040
========

The rp2040 is a dual core chip produced by the RaspberryPi Foundation that
is based on ARM Cortex-M0+.

Peripheral Support
==================

The following list indicates peripherals currently supported in NuttX:

============== =====
Peripheral     Notes
============== =====
GPIO           See Supported Boards documentation for available pins.
UART           GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX) are used for the console.
I2C            
SPI         
DMAC        
PWM         
USB         
PIO            RP2040 Programmable I/O
IRQs        
DMA         
ws2812         Smart pixels (e.g. Neopixel)
Flash ROM Boot
SRAM Boot      If Pico SDK is available a nuttx.uf2 file will be created
BMP180         Requires I2C0
INA219         Requires I2C0
============== =====

The Pico Display Pack (ST7789 LCD) and Pico Audio Pack (PCM5100A I2S DAC) are 
also available.

Installation
============

1. Download Raspberry Pi Pico SDK::

    git clone -b 1.1.2 https://github.com/raspberrypi/pico-sdk.git

2. Set PICO_SDK_PATH environment variable::

    export PICO_SDK_PATH=<absolute_path_to_pico-sdk_directory>

3. Download NuttX and the companion applications.  These must both be
   contained in the same directory::
  
    git clone https://github.com/apache/incubator-nuttx.git nuttx
    git clone https://github.com/apache/incubator-nuttx-apps.git apps

Building NuttX
==============

1. Change to NuttX directory::

    cd nuttx

2. Select a configuration. The available configurations
   can be listed with the command::

    ./tools/configure.sh -L

3. Load the selected configuration.::

    make distclean
    ./tools/configure.sh <selected_configuration>

4. Modify the configuration as needed (optional)::

    make menuconfig

5. Build NuttX::
 
    make

Programming
============

Programming using BOOTSEL
-------------------------

Connect  board to USB port while pressing BOOTSEL.
The board will be detected as USB Mass Storage Device.
Then copy "nuttx.uf2" into the device.
(Same manner as the standard Pico SDK applications installation.)

Programming using SDB
---------------------

Most (but no all) RP2040 boards provide a serial (SDB) debug port.
The "nuttx" ELF file can be uploaded with an appropriate SDB programmer
module and companion software.

Running NuttX
=============

Most builds provide access to the console via UART0.  To access this
GPIO 0 and 1 pins must be connected to the device such as USB-serial converter.

The `usbnsh` configuration provides the console access by USB CDC/ACM serial
devcice.  The console is available by using a terminal software on the USB host.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

