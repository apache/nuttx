======================
Adafruit NRF52 Feather
======================

The `Adafruit NRF52 Feather <https://www.adafruit.com/product/3406>`_
is a development board for the nRF52832 SoC from Nordic.

Serial Console
==============

The Feather nRF52 default console is the UART0.

The Feather nRF52 have USB serial bridge chip on board and UART0 is
connected to micro USB connector through the bridge chip.

LEDs and Buttons
================

LEDs
----

The Feather has 2 user-controllable LEDs:

====  =======
LED   MCU
====  =======
LED1  PIN0-17
LED2  PIN0-19
====  =======

A high output illuminates the LED.

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

The Feather nRF52 does not have user-controllable buttons. The reset button
on the board is connected to nRF52832 reset pin directly.

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow::

  tools/configure.sh nrf52-feather:<subdir>

Where <subdir> is one of the following:

nsh
---

This configuration is the NuttShell (NSH) example at examples/nsh/.

NOTES:

  1. This configuration uses the mconf-based configuration tool.  To
     change this configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.
