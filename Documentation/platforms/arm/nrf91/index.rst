==============
 Nordic nRF91
==============

The nRF91 series of chips from Nordic Semiconductor are based around an ARM Cortex-M33 core
with integrated LTE-M/NB-IoT modem and GNSS.

Modem Support
=============

Modem is supported in the nRF91 using Nordic's `Modem library <https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrfxlib/nrf_modem/README.html>`_.

Tool Issues
===========

OpenOCD
-------------
There is no official support for Nordic Cortex M33 chips (nRF9160 or nRF5340).

Segger J-Link
-------------
To start the GDB servers for the application core, use these commands::

    JLinkGDBServer -device nRF9160 -if SWD -speed 4000 -port 2331

Then you can connect GDB to targets::

    (gdb) target remote localhost:2331

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
