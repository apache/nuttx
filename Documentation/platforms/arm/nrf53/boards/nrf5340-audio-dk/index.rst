=======================
Nordic nRF5340-Audio-DK
=======================

The `nRF5340-Audio-DK <https://www.nordicsemi.com/Products/Development-hardware/nRF5340-Audio-DK>`_
is a development kit dedicated for Bluetooth LE Audio application based on the nRF5340 from Nordic.

Peripheral Support
==================

======================== ======= =====
Peripheral               Support NOTES
======================== ======= =====
UART                     Yes
QSPI                     No
Buttons                  No
LEDs                     No
USB                      No
SD Card                  No
NFC                      No
PDM Microphone (VM3011)  No
Audio DSP (CS47L63)      No
======================== ======= =====

Serial Console
==============

Serial console for the application core:

===== ============ =============
Pin   Signal       Notes
===== ============ =============
P1.05 APP UART0 TX virtual COM 0
P1.04 APP UART0 RX virtual COM 0
===== ============ =============

Serial console for the network core:

===== ============ =============
Pin   Signal       Notes
===== ============ =============
P1.09 NET UART0 TX virtual COM 1
P1.08 NET UART0 RX virtual COM 1
===== ============ =============

LEDs and Buttons
================

LEDs
----
The nRF5340-Audio-DK has 4 user-controllable LEDs:

====  =======
LED   MCU
====  =======
LED1  P0.31
LED2  P1.01
LED3  P1.02
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
BUTTON1  P0.02
BUTTON2  P0.03
BUTTON3  P0.04
BUTTON4  P0.05
BUTTON5  P0.06
=======  =======

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh nrf5340-audio-dk:<subdir>

Where <subdir> is one of the following:

nsh_cpuapp
----------

Basic NuttShell configuration for the application core (console enabled in UART0,
exposed via J-Link VCOM0, at 115200 bps).
