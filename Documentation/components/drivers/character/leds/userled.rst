===============
USERLED Drivers
===============

The USERLED is a NuttX subsystem to control LEDs on the user board.
Using it is possible for applications to control each LED individually
or as a group.

There is an application called "leds" that will test all LEDs in your board (counting in binary, turning ON and OFF each LED).

.. code-block:: bash

  NuttShell (NSH)
  nsh> leds
  nsh>
  leds_main: Starting the led_daemon
  leds main: led daemon started
  led_daemon: Running
  led_daemon: Opening /dev/userleds
  led_daemon: Supported LEDs 0xff
  led daemon: LED set 0x01
  led daemon: LED set 0x02
  led daemon: LED set 0x03
  led daemon: LED set 0x04
  led daemon: LED set 0x05

Also is possible for users to control the LEDs from "nsh>" using the "printf" command to send data to it in hexa code:

.. code-block:: bash

  NuttShell (NSH)
  nsh> printf \\x000000a5 > /dev/userleds

This command will turn ON the LEDs mapped to bits 0, 2, 5 and 7.

It is important to note that USERLED and ARCH_LEDS will not work together,
so in order to use USERLED please disable CONFIG_ARCH_LEDS.

The NuttX USERLED driver is split into two parts:

#. An "upper half" (userled_upper.c), generic driver that provides the
   common interface to application level code, and
#. A "lower half" (userled_lower.c), that calls the platform-specific board
   functions (board_userled_initialize(), board_userled(), board_userled_all(), etc) that implements the low-level control of the LEDs.

Files supporting USERLED can be found in the following locations:

-  **Interface Definition**. The header file for the NuttX USERLED
   driver resides at ``include/nuttx/leds/userled.h``. This header
   file includes both the application level interface to the USERLED
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The USERLED module uses a standard character
   driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" USERLED driver
   resides at ``drivers/leds/userled_upper.c``.
-  **"Lower Half" Drivers**. Lower Half of the USERLED driver resides
   in ``drivers/leds/userled_lower.c`` and the directory for the board
   specific functions will be at ``boards/<arch>/<family>/<boardname>/src/<arch>_userleds.c``.

Something important to note is that your board initialization code (normally named ``<arch>_bringup.c`` should call the function to register the driver.

For stm32f4discovery board this initialization code is placed at ``boards/arm/stm32/stm32f4discovery/src/stm32_bringup.c`` and this is the block responsible to initialize the subsystem:

.. code-block:: C

  #ifdef CONFIG_USERLED
    /* Register the LED driver */

    ret = userled_lower_initialize("/dev/userleds");
    if (ret < 0)
      {
        syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      }
  #endif


