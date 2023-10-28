===============
nRF52840-dongle
===============

The `nRF52840-dongle (PCA10059) <https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dongle>`_
is a a small, low-cost USB dongle based on nRF52840 from Nordic.

Serial Console
==============

===== ========== =====================
Pin   Signal     Notes
===== ========== =====================
P0.15 UART TX    edge soldering points
P0.13 UART RX    edge soldering points
===== ========== =====================

LEDs and Buttons
================

LEDs
----

====  =======
LED   MCU
====  =======
LED1  P0.06
====  =======

Pushbuttons
-----------

=======  =======
BUTTON   MCU
=======  =======
BUTTON1  P1.06
=======  =======

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh nrf52840-dongle:<subdir>

Where <subdir> is one of the following:

nsh
----

Basic NuttShell configuration (console enabled in UART0, exposed via soldering points, at 115200 bps).

usbnsh
------

Basic NuttShell configuration (CDCACM console enabled in USB Port, at 115200 bps).

Flash & Debug
=============

Both flashing and debuing is possible only with an external debug probe.

