README
======

README for NuttX port to generic NRF52832 boards.

Contents
========

  - Status
  - PCA10040 development board
  - Configurations

Status
======

  This is the current status of the NRF52 port:

  - The basic OS test configuration and the basic NSH configurations
    are present and fully verified.  This includes:  SYSTICK system time,
    pin and GPIO configuration, and serial console support.

PCA10040 board
==============

  Console
  -------

  The PCA10040 default console is the UART0.

  The PCA10040 does not have RS-232 drivers or serial connectors on board.
  UART0, is available on P4 as follows:

    --------  -----
    Signal    PIN
    --------  -----
    UART0-TX  P0.24
    UART0-RX  P0.23

  LEDs
  ----
  The PCA10040 has 4 user-controllable LEDs

    LED   MCU
    LED1  PIN-17
    LED2  PIN-18
    LED3  PIN-19
    LED4  PIN-20

  A low output illuminates the LED.

  Pushbuttons
  -----------
  To be provided

Feather nRF52 board
===================

  https://www.adafruit.com/product/3406

  Console
  -------
  The Feather nRF52 default console is the UART0.

  The Feather nRF52 have USB serial bridge chip on board and UART0 is
  connected to micro USB connector through the bridge chip.

  LEDs
  ----
  The Feather has 2 user-controllable LEDs

    LED   MCU
    LED1  PIN-17
    LED2  PIN-19

  A high output illuminates the LED.

  Pushbuttons
  -----------
  The Feather nRF52 does not have user-controllable buttons. The reset button
  on the board is connected to nRF52832 reset pin directly.

Memory Map
==========

  Block                 Start      Length
  Name                  Address
  --------------------- ---------- ------
  FLASH                 0x00000000   512K
  RAM                   0x20000000    64K

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follow:

    tools/configure.sh nrf52-generic:<subdir>

Where <subdir> is one of the following:

  <board>-nsh:
  -----------
    This configuration is the NuttShell (NSH) example at examples/nsh/.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

  <board>-wdog:
  ------------
    This configuration is a simple NSH-based test of the nRF52 watchdog
    timer driver using the test at apps/examples/watchdog.

  CONFIG_ARCH_LEDS
  ----------------
  If CONFIG_ARCH_LEDS is defined, the LED will be controlled as follows
  for NuttX debug functionality (where NC means "No Change").

    TBD!

  If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
  control of the application.  The following interfaces are then available
  for application control of the LEDs:

    void board_userled_initialize(void);
    void board_userled(int led, bool ledon);
    void board_userled_all(uint8_t ledset);

