================
Nordic Thingy:53
================

The `Thingy:53 (PCA20053) <https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-53>`_
is a prototyping platform build around the nRF5340 from Nordic.

Peripheral Support
==================

================================== ======= =============
Peripheral                         Support NOTES
================================== ======= =============
UART                               Yes
USB                                Yes
Buttons                            Yes
LEDs                               No
NFC                                No
MX25R6435F                         No      Dual line SPI
PMIC                               No
Battery monitoring                 No
Buzzer                             No
PDM microphone (VM3011)            No
Front End Module (nRF21540)        No
Low power accelerometer (ADXL362)  Yes     SPI
IMU (BMI270)                       Yes     SPI or I2C 0x68
Magnetometer (BMM150)              Yes     I2C 0x10
Color sensor (BH1749NUC)           Yes     I2C 0x38
Air quality sensor (BME688)        No      I2C 0x76
================================== ======= =============

Serial Console
==============

Serial console for the application core:

===== ============ =============
Pin   Signal       Notes
===== ============ =============
P0.11 APP UART0 TX virtual COM 0
P0.12 APP UART0 RX virtual COM 0
===== ============ =============

Serial console for the network core:

===== ============ =============
Pin   Signal       Notes
===== ============ =============
P0.09 NET UART0 TX virtual COM 1
P0.10 NET UART0 RX virtual COM 1
===== ============ =============

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh thingy53:<subdir>

Where <subdir> is one of the following:

composite_cpuapp
----------------
NuttShell configuration with support for CDC/ACM with RNDIS composite driver.

nsh_cpuapp
----------

Basic NuttShell configuration for the application core (console enabled in UART0,
exposed via J-Link VCOM0, at 115200 bps).

nsh_cpunet
----------

Basic NuttShell configuration for the network core (console enabled in UART0,
exposed via J-Link VCOM1, at 115200 bps).

Flash & Debug
=============

Both flashing and debuing is possible only with an external debug probe.
