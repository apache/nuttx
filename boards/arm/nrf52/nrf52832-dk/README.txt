README
======

README for NuttX port to NRF52832-DK (PCA10040) boards.

Contents
========

  - Status
  - NRF52832 development kit (PCA10040)
  - Configurations

Status
======

  This is the current status of the NRF52 port:

  - The basic OS test configuration and the basic NSH configurations
    are present and fully verified.  This includes:  SYSTICK system time,
    pin and GPIO configuration, and serial console support.

NRF52832 development kit (PCA10040)
===================================

  Console
  -------

  The PCA10040 default console is the UART0.

  The PCA10040 does not have RS-232 drivers or serial connectors on board.
  UART0 is connected to the virtual COM port:

    --------  -----
    Signal    PIN
    --------  -----
    UART0-TX  P0.06
    UART0-RX  P0.08

  LEDs
  ----
  The PCA10040 has 4 user-controllable LEDs

    LED   MCU
    LED1  PIN0-17
    LED2  PIN0-18
    LED3  PIN0-19
    LED4  PIN0-20

  A low output illuminates the LED.

  Pushbuttons
  -----------
  To be provided

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

    tools/configure.sh nrf52832-dk:<subdir>

Where <subdir> is one of the following:

  nsh:
  -----------
    This configuration is the NuttShell (NSH) example at examples/nsh/.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

  wdog:
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

    uint32_t board_userled_initialize(void);
    void board_userled(int led, bool ledon);
    void board_userled_all(uint32_t ledset);
