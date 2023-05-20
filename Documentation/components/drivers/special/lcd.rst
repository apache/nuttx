=====================
LCD Character Drivers
=====================

The LCD driver exposes the LCD interface to userspace via ``ioctl()`` commands.

The LCD driver is intended to be used in the following scenarios:

#. On memory-constrained devices, as it doesn't require a buffer to represent the whole display:

   #. Hence, it's an alternative to the :doc:`framebuffer`;

#. For graphics libraries that draw specific areas of the displays, like ``LVGL``;

Binding
========
LCD drivers usually are not directly accessed by user code, but are usually bound to another, higher-level device driver.
In general, the binding sequence is:

#. Get an instance of ``struct lcd_dev_s`` from the hardware-specific LCD screen driver, and
#. Provide that instance to the initialization method of the higher-level character driver.

.. _genericlcdlcd:

Generic LCD Character Driver
----------------------------

This example will walk through the path from userspace to hardware-specific details on how an LCD screen is bound to an LCD character driver.

#. ``include/nuttx/lcd/lcd.h`` provides all structures and APIs needed to work with LCD screens drivers:

   #. This header file also depends on some of the same definitions used for the frame buffer driver as provided in ``include/nuttx/video/fb.h``;
#. ``drivers/lcd/lcd_dev.c`` is the higher-level device driver. An instance of ``struct lcd_dev_s`` will be provided to it:

   #. ``include/nuttx/lcd/lcd_dev.h`` prototypes public structures and functions;
   #. ``lcddev_register`` registers the LCD character driver as ``/dev/lcdN`` where N is the display number and,
   #. calls the ``board_lcd_getdev``, an LCD-specific function usually defined in ``boards/<arch>/<chip>/<board>/src`` and prototyped in ``include/nuttx/board.h``;

#. Finally, the LCD screen drivers are usually available at ``drivers/lcd/`` and implement the callbacks defined at ``include/nuttx/lcd/lcd.h``:

    #. ``include/nuttx/lcd/lcd.h`` provides structures and APIs needed to work with LCD screens, whether using the framebuffer adapter or the :doc:`lcd`;


Examples
========

Examples apply to specific cases of the :ref:`genericlcdlcd`:

.. _ttgotdisplayesp32_lcd:

TTGO T-Display ESP32 board
---------------------------

This board contains an ST7789 TFT Display (135x240).
By selecting the ``ttgo_t_display_esp32:lvgl_lcd`` config, the ``lvgldemo`` example will be built with the LCD character interface.

* ``boards/xtensa/esp32/ttgo_t_display_esp32/src/esp32_bringup.c`` registers the LCD character driver:

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

* ``board_lcd_initialize`` and ``board_lcd_getdev`` are defined at ``boards/xtensa/esp32/common/src/esp32_st7789.c``;

   * ``board_lcd_initialize`` initializes the LCD hardware on the board by defining the SPI interface which is connected to the display controller;

* ``lcddev_register`` then calls ``board_lcd_getdev``:

   * ``board_lcd_getdev`` calls the ``st7789_lcdinitialize`` and returns a reference to the LCD object for the specified LCD;
   * ``st7789_lcdinitialize`` is part of the LCD screen driver at ``drivers/lcd/st7789.c``;

* The LVGL demo application (``lvgldemo``) makes use of the ``ioctl`` system call to trigger an ``LCDDEVIO_PUTAREA`` request to the higher-level device driver to refresh the LCD screen with data:

.. code-block:: c

   ioctl(state.fd, LCDDEVIO_PUTAREA, (unsigned long)((uintptr_t)&lcd_area));;

NuttX Simulator
----------------

:doc:`NuttX Simulator </platforms/sim/sim/index>` provides a X11-based LCD character driver to simulate the LCD character displat usage into a X11-compatible host.

By selecting the ``sim:lvgl_lcd`` config, the ``lvgldemo`` example will be built with the LCD character interface.

* ``boards/sim/sim/sim/src/sim_bringup.c`` registers the lcd driver the same way :ref:`ttgotdisplayesp32_lcd`;
* ``arch/sim/src/sim/up_lcd.c`` and ``arch/sim/src/sim/up_x11framebuffer.c`` will be built as ``CONFIG_SIM_LCDDRIVER = y`` and ``CONFIG_SIM_X11FB = y`` are set, respectively;

   * ``up_lcd.c`` provides ``board_lcd_initialize`` and ``board_lcd_getdev``:

      * ``board_lcd_initialize`` calls ``up_x11initialize`` from ``up_x11framebuffer.c`` that initializes a X11-based window as an LCD character device. This is the underlying "driver".

* The LVGL demo application (``lvgldemo``) makes use of the ``ioctl`` system call to trigger an ``LCDDEVIO_PUTAREA`` request to the higher-level device driver to refresh the LCD screen with data as usual;
