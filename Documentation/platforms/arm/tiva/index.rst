===================================================
Texas Instruments Tiva C / Luminary Micro Stellaris
===================================================

The Tiva architecture is a combination of the Luminary Micro Stellaris chips
and the TI Tiva variants.  There are both M3 and M4 Arm cores in this family.

Supported MCUs
=============

The following list includes MCUs from Tiva C series supported in NuttX

=============   ==============  ======= =================
MCU             Core            Radio    Frequency
=============   ==============  ======= =================
LM3S6918        Cortex-M3       No      50 MHz
LM3S9B92        Cortex-M3       No      50 MHz
LM3S9B96        Cortex-M3       No      50 MHz
LM3S6432        Cortex-M3       No      50 MHz
LM3S6965        Cortex-M3       No      50 MHz
LM3S8962        Cortex-M3       No      50 MHz
LM4F120         Cortex-M4       No      80 MHz
TM4C123AH6PM    Cortex-M4       No      120 MHz
TM4C123GH6ZRB   Cortex-M4       No      120 MHz
TM4C123GH6PM    Cortex-M4       No      120 MHz
TM4C123GH6PZ    Cortex-M4       No      120 MHz
TM4C123GH6PGE   Cortex-M4       No      120 MHz
TM4C1294NCPDT   Cortex-M4       No      120 MHz
TM4C129ENCPDT   Cortex-M4       No      120 MHz
TM4C129ENCZAD   Cortex-M4       No      120 MHz
TM4C129XNCZAD   Cortex-M4       No      120 MHz
CC1310          Cortex-M3       Yes     24 MHz
CC1312R1        Cortex-M4       Yes     48 MHz
CC1352R1        Cortex-M4       Yes     48 MHz
=============   ==============  ======= =================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========
Peripheral
==========
GPIO
ADC
EEPROM
FLASH
HCIUART
I2C
MPU
PWM
QENCODER
SERIAL
SSI
TIMER
ETHERNET
WDT
I2S
USB
==========

GPIO
-----------


Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
