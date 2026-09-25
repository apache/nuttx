================
TI/Hercules RM57
================

.. tags:: chip:rm57, arch:armv7-r

The RM57 is part of TI's Hercules family of ARM Cortex-R5F safety
microcontrollers, aimed at applications that need lockstep execution and
built-in self-test (industrial, medical, and automotive safety systems).
NuttX currently supports the **RM57L843** part.

.. warning::

   Support for this chip family is new and experimental. Only the SCI
   (serial) and GIO (LED) peripherals have been brought up so far; the
   PLL/clock configuration values, JTAG IDCODE, and LED polarity have not
   all been independently confirmed against hardware/schematics. See the
   :doc:`board documentation <boards/rm57l843-launchxl2/index>` for
   details.

Peripheral Support
==================

The following list indicates RM57 peripherals currently supported in NuttX:

============== =====
Peripheral     Notes
============== =====
SCI            Serial Communication Interface, used for the console
GIO            General purpose I/O, used for LEDs
============== =====

.. todo::

   Many peripherals available on the RM57L843 (N2HET, MibSPI, CAN, ADC,
   the ESM diagnostic module, PBIST/STC self-test, etc.) are not yet
   implemented. Contributions are welcome.

Supported Boards
=================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
