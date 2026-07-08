=================
Nordic Thingy:91
=================

.. tags:: chip:nrf91, chip:nrf52, chip:nrf9160, chip:nrf52840

The `Thingy:91 (PCA0035) <https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-91>`_
is a development board based on the nRF9160 and nRF52840 from Nordic.

Peripheral Support
==================

================================== ======= =============
Peripheral                         Support NOTES
================================== ======= =============
UART                               Yes
Modem                              Yes
GPS                                No
Buttons                            Yes
LEDs                               No
COEX                               No
PMIC (ADP5360)                     No      I2C 0x46
Battery monitoring                 No
Buzzer                             No
EEPROM (24CW160)                   No      I2C 0x50
Low power accelerometer (ADXL362)  No      SPI
Hi G accelerometer (ADXL372)       No      SPI
Air quality sensor (BME680)        No      I2C 0x76
Color sensor (BH1749NUC)           No      I2C 0x38
================================== ======= =============

Serial Console
==============

1. Console over RTT UART
2. Access to UART0 console over USB connected to nRF52840.
   MCU_IF0 and MCU_IF1 pins are used to communicate between MCUs.

Configurations
==============

thingy91_rtt
------------

Configuration with a console over RTT, enabling available peripherals
on the board (WIP).

Flash & Debug
=============

Both flashing and debuing is possible only with an external debug probe.
