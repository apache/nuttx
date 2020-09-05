===========
LCD Drivers
===========

-  ``include/nuttx/lcd/lcd.h``. Structures and APIs needed to
   work with LCD drivers are provided in this header file. This
   header file also depends on some of the same definitions used
   for the frame buffer driver as provided in
   ``include/nuttx/video/fb.h``.

-  ``struct lcd_dev_s``. Each LCD device driver must implement
   an instance of ``struct lcd_dev_s``. That structure defines a
   call table with the following methods:

   Get information about the LCD video controller configuration
   and the configuration of each LCD color plane.

   The following are provided only if the video hardware supports
   RGB color mapping:

   The following are provided only if the video hardware supports
   a hardware cursor:

   Get the LCD panel power status (0: full off -
   ``CONFIG_LCD_MAXPOWER``: full on). On backlit LCDs, this
   setting may correspond to the backlight setting.

   Enable/disable LCD panel power (0: full off -
   ``CONFIG_LCD_MAXPOWER``: full on). On backlit LCDs, this
   setting may correspond to the backlight setting.

   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST) \*/

   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST)

-  **Binding LCD Drivers**. LCD drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. In general, the binding sequence is:

   #. Get an instance of ``struct lcd_dev_s`` from the
      hardware-specific LCD device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``drivers/lcd/p14201.c``,
   ``boards/arm/sam34/sam3u-ek/src/up_lcd.c.`` See also the usage
   of the LCD driver in the ``graphics/`` directory.
