===========
nRF52832-DK
===========

The `NRF52832-DK (PCA10040) <https://www.nordicsemi.com/Products/Development-hardware/nrf52-dk>`_
is a development board for the nRF52832 SoC from Nordic.

Serial Console
==============

The PCA10040 default console is the UART0.

The PCA10040 does not have RS-232 drivers or serial connectors on board.
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
The PCA10040 has 4 user-controllable LEDs:

====  =======
LED   MCU
====  =======
LED1  P0.17
LED2  P0.18
LED3  P0.19
LED4  P0.20
====  =======

A low output illuminates the LED.

CONFIG_ARCH_LEDS
----------------

If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
control of the application.  The following interfaces are then available
for application control of the LEDs::

  uint32_t board_userled_initialize(void);
  void board_userled(int led, bool ledon);
  void board_userled_all(uint32_t ledset);

Pushbuttons
-----------

=======  =======
BUTTON   MCU
=======  =======
BUTTON1  P0.13
BUTTON2  P0.14
BUTTON3  P0.15
BUTTON4  P0.16
=======  =======

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh nrf52832-dk:<subdir>

Where <subdir> is one of the following:

buttons
-------

This configuration shows the use of the buttons subsystem.

nsh
----

Basic NuttShell configuration (console enabled in UART0, exposed via J-Link VCOM connection,
at 115200 bps).

ostest_tickless
---------------

This is a NSH configuration that includes ``apps/testing/ostest`` as a builtin and
enable support for the tick-less OS.

sdc
----

Enables Nordic's SoftDevice controller and uses NuttX BLE stack for the host-layer.
The ``btsak`` application is included as a builtin.

sdc_nimble
----------

Enables Nordic's SoftDevice controller and uses nimBLE for the host-layer.
The ``nimble`` test application can be used to enable a simple GATT server.

wdog
----
This configuration is a simple NSH-based test of the nRF52 watchdog
timer driver using the test at ``apps/examples/watchdog``.
