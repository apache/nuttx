===========
nRF52840-DK
===========

The `NRF52840-DK (PCA10056) <https://www.nordicsemi.com/Products/Development-hardware/nRF52840-DK>`_
is a development board for the nRF52840 SoC from Nordic.

Serial Console
==============

The PCA10056 default console is the UART0.

The PCA10056 does not have RS-232 drivers or serial connectors on board.
UART0 is connected to the virtual COM port:

========  =====
Signal    PIN
========  =====
UART0-RX  P0.08
UART0-TX  P0.06
========  =====

LEDs and Buttons
================

LEDs
----
The PCA10056 has 4 user-controllable LEDs:

====  =======
LED   MCU
====  =======
LED1  P0.13
LED2  P0.14
LED3  P0.15
LED4  P0.26
====  =======

A low output illuminates the LED.

Pushbuttons
-----------

=======  =======
BUTTON   MCU
=======  =======
BUTTON1  P0.11
BUTTON2  P0.12
BUTTON3  P0.24
BUTTON4  P0.25
=======  =======

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh nrf52840-dk:<subdir>

Where <subdir> is one of the following:

adc
----

This configuration shows the use of the ADC peripheral.

buttons
-------

This configuration shows the use of the buttons subsystem.

cdcacm
-------

NuttShell configuration with support for CDC/ACM USB device driver.

composite
---------

NuttShell configuration with support for CDC/ACM with RNDIS composite driver.

highpri
-------

This application demonstrates high priority interrupt feature of the NuttX.

nsh
----

Basic NuttShell configuration (console enabled in UART0, exposed via J-Link VCOM connection,
at 115200 bps).

ostest_tickless
---------------

This is a NSH configuration that includes ``apps/testing/ostest`` as a builtin
and enable support for the tick-less OS.

pwm
---

This configuration shows the use of the PWM peripheral.

qspi
----

NuttShell configuration with enabled support for on-board MX25R QSPI memory.

rndis
-----

NuttShell configuration with support for RNDIS USB device driver.

sdc
---

Enables Nordic's SoftDevice controller and uses NuttX BLE stack for the host-layer.
The ``btsak`` application is included as a builtin.

sdc_nimble
----------

Enables Nordic's SoftDevice controller and uses nimBLE for the host-layer.
The ``nimble`` test application can be used to enable a simple GATT server.

sx127x
------

NuttShell configuration with support for sx127x chip.

timer
------

This configuration shows the use of the TIMER peripheral.

usbnsh
------

Basic NuttShell configuration (CDCACM console enabled in USB Port, at 115200 bps).

ieee802154_6lowpan
------------------

Cheat Sheet.  Here is a concise summary of all all the steps needed to
run the UDP test (C=Coordinator; E=Endpoint)::

         C: nsh> i8 wpan0 startpan cd:ab
         C: nsh> i8 set saddr 0A:00
         C: nsh> i8 set ep_saddr 0B:00
         C: nsh> i8 acceptassoc
         E: nsh> i8 wpan0 assoc
         C: nsh> ifup wpan0
         C: nsh> ifconfig          <-- To get the <server-ip>
         E: nsh> ifup wpan0
         C: nsh> udpserver &
         E: nsh> udpclient <server-ip> &
