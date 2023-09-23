=================
Nordic Thingy:91
=================

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
PMIC                               No
Battery monitoring                 No
Buzzer                             No
EEPROM (24CW160)                   No
Low power accelerometer (ADXL362)  No
Hi G accelerometer (ADXL372)       No
Air quality sensor (HBME680)       No
Color sensor (BH1749NUC)           No
================================== ======= =============

Serial Console
==============

1. Console over RTT UART
2. Access to UART0 console over USB connected to nRF52840.
   MCU_IF0 and MCU_IF1 pins are used to communicate between MCUs.

Configurations
==============

TODO

Flash & Debug
=============

Both flashing and debuing is possible only with an external debug proble.
