.. note:: See also the usage of the LCD driver in the graphics/ directory.

=====================
LCD Character Drivers
=====================

The LCD driver exposes the LCD interface to userspace via ``ioctl()`` commands.

The LCD driver is intended to be used in the following scenarios:

* On memory-constrained devices, as it doesn't require a buffer to represent
   the whole display:

   * Hence, it's an alternative to the :doc:`framebuffer`

* For graphics libraries that draw specific areas of the displays, like ``LVGL``

Binding
=======


LCD drivers usually are not directly accessed by user code, but are usually
bound to another, higher-level device driver. In general, the binding sequence is:

#. Get an instance of ``struct lcd_dev_s`` from the hardware-specific LCD screen driver, and
#. Provide that instance to the initialization method of the higher-level character driver.

.. _genericlcdlcd:

Generic LCD Character Driver
----------------------------

This example will walk through the path from userspace to hardware-specific
details on how an LCD screen is bound to an LCD character driver.

* ``include/nuttx/lcd/lcd.h`` provides all structures and APIs needed to work
   with LCD screens drivers:

   * This header file also depends on some of the same definitions used for the
     frame buffer driver as provided in ``include/nuttx/video/fb.h``

* ``drivers/lcd/lcd_dev.c`` is the higher-level device driver. An instance of
   ``struct lcd_dev_s`` will be provided to it:

   * ``include/nuttx/lcd/lcd_dev.h`` prototypes public structures and functions;
   * ``lcddev_register`` registers the LCD character driver as ``/dev/lcdN``
     where N is the display number and,
   * calls the ``board_lcd_getdev``, an LCD-specific function usually defined in
     ``boards/<arch>/<chip>/<board>/src`` and prototyped in ``include/nuttx/board.h``

* Finally, the LCD screen drivers are usually available at ``drivers/lcd/`` and
  implement the callbacks defined at ``include/nuttx/lcd/lcd.h``:

    * ``include/nuttx/lcd/lcd.h`` provides structures and APIs needed to work
      with LCD screens, whether using the framebuffer adapter or the :doc:`lcd`;

Supported devices
=================

Re-usable LCD drivers reside in the drivers/lcd directory:

LCDs
----

- ``mio283qt2.c``

  This is a driver for the MI0283QT-2 LCD from Multi-Inno
  Technology Co., Ltd.  This LCD is based on the Himax HX8347-D LCD
  controller.

- ``mio283qt9a.c``

  This is a driver for the MI0283QT-9A LCD from Multi-Inno
  Technology Co., Ltd.  This LCD is based on the Ilitek ILI9341 LCD
  controller.

- ``ssd12989.c``

  Generic LCD driver for LCDs based on the Solomon Systech
  SSD1289 LCD controller. Think of this as a template for an LCD driver
  that you will probably have to customize for any particular LCD
  hardware. (See also boards/arm/stm32/hymini-stm32v/src/ssd1289.c below).

- ``st7567.c``

  LCD Display Module, ST7567, Univision Technology Inc.
  Used with the LPCXpresso and Embedded Artists base board.

- ``memlcd.c``

  Sharp Memory LCD Suite, LS013B7DH01, LS013B7DH03, etc.
  There are some more different models, they are basically controlled
  by similar logics, thus this driver can be extended.

- ``ra8875.c``
  
  RAiO Technologies RA8875 LCD controller.  Contributed by Marten Svanfeldt.

OLEDs
-----

- ``p14201.c``

  Driver for RiT P14201 series display with SD1329 IC controller.
  Based on the SD1329 controller.  This OLED is used with
  older versions of the TI/Luminary LM3S8962 Evaluation Kit.  Example
  usage::

      boards/arm/tiva/lm3s6965-ek/src
      boards/arm/tiva/lm3s8962-ek/src

- ``ug-2864ambag01.c``

  OLED Display Module, UUG-2864AMBAG01, Univision Technology Inc.
  Based on the SH1101A controller.  Example usage::

      boards/arm/stm32/stm32f4discovery
      boards/arm/lpc214x/zp214xpa

- ``ug-9664hswag01.c``

  OLED Display Module, UG-9664HSWAG01, Univision Technology Inc.
  Based on the SSD1305 controller.  Used with the
  LPC Xpresso and Embedded Artists base board.  Example usage::

      boards/arm/lpc71xx_40xx/lpcxpresso-lpc1768

- ``ssd1306.c``

  OLED Display Modules based on the SSD1306 controllers.
  This includes the UG-2864HSWEG01 and UG2832HSWEG04, both from Univision
  Technology Inc.  The latter is used with the OLED1 module that comes
  with the Atmel SAM4l Xplained Pro board.  This driver also supports
  Densitron Technologies DD-12864WO-4A which is based on SSD1309 LCD
  controller. Example usage::

      boards/arm/stm32/stm32f4discovery
      boards/arm/sam34/sam4l-xplained

Segment LCDS (SLCDs)
--------------------

- ``pcf8574_lcd_backpack.c``

  See pcf8574_lcd_backpack_readme.txt.


Examples
========

Examples apply to specific cases of the :ref:`genericlcdlcd`:

.. _ttgotdisplayesp32_lcd:

TTGO T-Display ESP32 board
---------------------------

This board contains an ST7789 TFT Display (135x240).
By selecting the ``ttgo_t_display_esp32:lvgl_lcd`` config, the ``lvgldemo``
example will be built with the LCD character interface.

* ``boards/xtensa/esp32/ttgo_t_display_esp32/src/esp32_bringup.c`` registers the
  LCD character driver:

.. code-block:: c

   #ifdef CONFIG_LCD_DEV
    ret = board_lcd_initialize();
    if (ret < 0)
      {
        syslog(LOG_ERR, "ERROR: board_lcd_initialize() failed: %d\n", ret);
      }

    ret = lcddev_register(0);
    if (ret < 0)
      {
        syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
      }
   #endif

* ``board_lcd_initialize`` and ``board_lcd_getdev`` are defined at
  ``boards/xtensa/esp32/common/src/esp32_st7789.c``;

   * ``board_lcd_initialize`` initializes the LCD hardware on the board by
     defining the SPI interface which is connected to the display controller;

* ``lcddev_register`` then calls ``board_lcd_getdev``:

   * ``board_lcd_getdev`` calls the ``st7789_lcdinitialize`` and returns a
     reference to the LCD object for the specified LCD;
   * ``st7789_lcdinitialize`` is part of the LCD screen driver at
     ``drivers/lcd/st7789.c``;

* The LVGL demo application (``lvgldemo``) makes use of the ``ioctl`` system
  call to trigger an ``LCDDEVIO_PUTAREA`` request to the higher-level device
  driver to refresh the LCD screen with data:

.. code-block:: c

   ioctl(state.fd, LCDDEVIO_PUTAREA, (unsigned long)((uintptr_t)&lcd_area));;

NuttX Simulator
----------------

:doc:`NuttX Simulator </platforms/sim/sim/index>` provides a X11-based LCD
     character driver to simulate the LCD character displat usage into a
     X11-compatible host.

By selecting the ``sim:lvgl_lcd`` config, the ``lvgldemo`` example will be
built with the LCD character interface.

* ``boards/sim/sim/sim/src/sim_bringup.c`` registers the lcd driver the
  same way :ref:`ttgotdisplayesp32_lcd`;
* ``arch/sim/src/sim/up_lcd.c`` and ``arch/sim/src/sim/up_x11framebuffer.c``
  will be built as ``CONFIG_SIM_LCDDRIVER = y`` and ``CONFIG_SIM_X11FB = y``
  are set, respectively;

   * ``up_lcd.c`` provides ``board_lcd_initialize`` and ``board_lcd_getdev``:

      * ``board_lcd_initialize`` calls ``up_x11initialize`` from
        ``up_x11framebuffer.c`` that initializes a X11-based window as an LCD
        character device. This is the underlying "driver".

* The LVGL demo application (``lvgldemo``) makes use of the ``ioctl`` system
  call to trigger an ``LCDDEVIO_PUTAREA`` request to the higher-level device
  driver to refresh the LCD screen with data as usual;

LCD Header files
================

``include/nuttx/lcd/lcd.h``

  Structures and APIs needed to work with LCD drivers are provided in
  this header file.  This header file also depends on some of the same
  definitions used for the frame buffer driver as provided in
  include/nuttx/video/fb.h.

``struct lcd_dev_s``

  Each LCD device driver must implement an instance of struct lcd_dev_s.
  That structure defines a call table with the following methods:

  - Get information about the LCD video controller configuration and the
    configuration of each LCD color plane.

    .. code-block:: C

        int (*getvideoinfo)(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo);
        int (*getplaneinfo)(FAR struct lcd_dev_s *dev, unsigned int planeno,
                            FAR struct lcd_planeinfo_s *pinfo);


  - The following are provided only if the video hardware supports RGB
    color mapping:

     .. code-block:: C

        int (*getcmap)(FAR struct lcd_dev_s *dev,
                       FAR struct fb_cmap_s *cmap);
        int (*putcmap)(FAR struct lcd_dev_s *dev,
                       FAR const struct fb_cmap_s *cmap);


  - The following are provided only if the video hardware supports a
    hardware cursor:

     .. code-block:: C

        int (*getcursor)(FAR struct lcd_dev_s *dev,
                         FAR struct fb_cursorattrib_s *attrib);
        int (*setcursor)(FAR struct lcd_dev_s *dev,
                         FAR struct fb_setcursor_s *settings);


  - Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER:
    full on). On backlit LCDs, this setting may correspond to the
    backlight setting:

     .. code-block:: C

        int (*getpower)(struct lcd_dev_s *dev);


  - Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
    full on). On backlit LCDs, this setting may correspond to the
    backlight setting:

     .. code-block:: C

        int (*setpower)(struct lcd_dev_s *dev, int power);


  - Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST):

     .. code-block:: C

        int (*getcontrast)(struct lcd_dev_s *dev);


  - Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST):

     .. code-block:: C
                
        int (*setcontrast)(struct lcd_dev_s *dev, unsigned int contrast);
