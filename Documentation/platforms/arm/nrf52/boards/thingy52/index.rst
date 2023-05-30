=================
Nordic Thingy:52
=================

The `Thingy:52 (PCA20020) <https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-52>`_
is a prototyping platform build around the nRF52832 from Nordic.

Peripheral Support
==================

================================== ======= =============
Peripheral                         Support NOTES
================================== ======= =============
UART                               Yes
Buttons                            No
LEDs                               No
NFC                                No
IMU (MPU-9250)                     No
Low power accelerometer (LIS2DH12) No
Pressure and temperature (LPS22HB) No
Humidity and temperature (HTS221)  No
Color sensor (BH1745NUC)           No
Gas sensor (CCS811)                No
Microphone (MP34DB02)              No
Speaker                            No
Battery monitoring                 No
================================== ======= =============

Serial Console
==============

===== ========== ==========
Pin   Signal     Notes
===== ========== ==========
P0.02 UART TX    P4 header
P0.03 UART RX    P4 header
===== ========== ==========

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh thingy52:<subdir>

Where <subdir> is one of the following:

nsh
----

Basic NuttShell configuration (console enabled in UART0, exposed via P4 header, at 115200 bps).

Flash & Debug
=============

Both flashing and debuing is possible only with an external debug proble.
