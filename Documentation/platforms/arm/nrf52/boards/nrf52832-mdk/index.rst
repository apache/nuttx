=======================
MakerDiary nRF52832-MDK
=======================

The nRF52832-MDK is a development board for the nRF52832 SoC from Nordic. It features 24 I/Os
an on-board RGB led and a chip antenna. It also includes an embedded DAPlink debugger which
allows to flash/debug and monitor UART from the USB port.

More information about this board can be found at `MakerDiary wiki <https://wiki.makerdiary.co/nrf52832-mdk>`_.

.. figure:: pinout.webp
   :align: center

   Pinout diagram

.. tip:: Pins P0.19 and P0.20 correspond to UART RX/TX (from nRF52 perspective) which are connected
   to the embedded debugger.

Resources
=========

The nRF52832 chip has 512K of FLASH and 64K of RAM.

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via USB connection, at 115200 bps).

sdc
---

Enables Nordic's SoftDevice controller and uses nimBLE for the host-layer.
The ``nimble`` test application can be used to enable a simple GATT server.

Flash & Debug
=============

Both flashing and debugging are done using the embedded DAPlink debugger. OpenOCD can be invoked
in the following way to flash::

    openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg -c "program nuttx/nuttx.bin 0x0000000 verify reset; shutdown"

