====================
Frame Buffer Drivers
====================

A framebuffer is a memory-mapped buffer that represents all the pixels necessary to drive a video display.

The Frame Buffer driver is intended to be used in the following scenarios:

#. Whenever it is necessary to hold all the pixels that would be used to drive a video display. This includes:

   #. Graphics libraries that directly access the underlying framebuffer;
   #. Advanced UIs (e.g. alpha blending) that need to read back the image data;

#. Applications that expect the framebuffer to exist;

Binding
========
LCD and frame buffer drivers usually are not directly accessed by user code, but are usually bound to another, higher-level device driver.
In general, the binding sequence is:

#. Get an instance of ``struct fb_vtable_s`` from the hardware-specific frame buffer device driver, and
#. Provide that instance to the initialization method of the higher-level device driver.

.. _genericlcdfb:

Generic LCD Frame Buffer
------------------------

This example will walk through the path from userspace to hardware-specific details on how an LCD screen is bound to a framebuffer.

#. ``include/nuttx/video/fb.h`` provides all structures and APIs needed to work with frame buffer drivers:

   #. ``drivers/video/fb.c`` is the higher-level device driver. An instance of ``struct fb_vtable_s`` will be provided to it;
   #. ``fb_register`` registers the framebuffer character device at ``/dev/fbN`` where N is the display number;
   #. It also provides the prototype of ``up_fbinitialize``, which may be defined by:

      #. An specific device into ``arch/<arch>/src/<chip>`` directory;
      #. By the LCD framebuffer adapter in ``drivers/lcd/lcd_framebuffer.c``, which provides an intermediary interface between the Frame Buffer Driver and the LCD screen drivers;

#. Let's consider we are using the LCD framebuffer (``CONFIG_LCD_FRAMEBUFFER = y``):

   #. This interface implements the ``up_fbinitialize`` which:

      #. Provides the instance of ``struct fb_vtable_s`` (a member of ``struct lcdfb_dev_s``);
      #. Calls ``board_lcd_initialize`` and ``board_lcd_getdev`` LCD-specific functions. These functions are defined in ``boards/<arch>/<chip>/<board>/src`` and prototyped in ``include/nuttx/board.h``;

#. Finally, the LCD screen drivers are usually available at ``drivers/lcd/`` and implement the callbacks defined at ``include/nuttx/lcd/lcd.h``:

    #. ``include/nuttx/lcd/lcd.h`` provides structures and APIs needed to work with LCD screens, whereas using the framebuffer adapter or the :doc:`lcd`;


Examples
========

Examples apply to specific cases of the :ref:`genericlcdfb`:

.. _ttgotdisplayesp32:

TTGO T-Display ESP32 board
---------------------------

This board contains an ST7789 TFT Display (135x240).
By selecting the ``ttgo_t_display_esp32:lvgl_fb`` config, the ``lvgldemo`` example will be built with the framebuffer interface.

* ``boards/xtensa/esp32/ttgo_t_display_esp32/src/esp32_bringup.c`` registers the framebuffer driver:

.. code-block:: c

   #ifdef CONFIG_VIDEO_FB
     ret = fb_register(0, 0);
     if (ret < 0)
       {
         syslog(LOG_ERR, "ERROR: Failed to initialize Frame Buffer Driver.\n");
       }
   #endif

* ``up_fbinitialize`` from the frame buffer adapter will then be called as ``CONFIG_LCD_FRAMEBUFFER = y``:

   * ``board_lcd_initialize`` and ``board_lcd_getdev`` are defined at ``boards/xtensa/esp32/common/src/esp32_st7789.c``:

       *  ``board_lcd_initialize`` initializes the LCD hardware on the board by defining the SPI interface which is connected to the display controller;
       * ``board_lcd_getdev`` calls the ``st7789_lcdinitialize`` and returns a reference to the LCD object for the specified LCD;
       * ``st7789_lcdinitialize`` is part of the LCD screen driver at ``drivers/lcd/st7789.c``;

* The LVGL demo application (``lvgldemo``) makes use of the ``ioctl`` system call to trigger a ``FBIO_UPDATE`` request to the higher-level device driver to refresh the LCD screen with framebuffer data:

.. code-block:: c

   ioctl(state.fd, FBIO_UPDATE, (unsigned long)((uintptr_t)&fb_area));

NuttX Simulator
----------------

:doc:`NuttX Simulator </platforms/sim/sim/index>` provides a X11-based framebuffer driver to simulate the framebuffer usage into a X11-compatible host.

By selecting the ``sim:lvgl_fb`` config, the ``lvgldemo`` example will be built with the framebuffer driver.

* ``boards/sim/sim/sim/src/sim_bringup.c`` registers the framebuffer driver the same way :ref:`ttgotdisplayesp32`;
* ``arch/sim/src/sim/up_framebuffer.c`` and ``arch/sim/src/sim/up_x11framebuffer.c`` will be built as ``CONFIG_SIM_FRAMEBUFFER = y`` and ``CONFIG_SIM_X11FB = y`` are set, respectively;

   * ``up_framebuffer.c`` provides ``up_fbinitialize`` and,
   * calls ``up_x11initialize`` from ``up_x11framebuffer.c`` that initializes a X11-based window as a framebuffer. This is the underlying "driver".

* The LVGL demo application (``lvgldemo``) makes use of the ``ioctl`` system call to trigger a ``FBIO_UPDATE`` request to the higher-level device driver in order to refresh the LCD screen with framebuffer data as usual;

.. warning::

   One must consider that framebuffer requires that the entire display's pixels to be represented.
   Considering a 320x480 @RGB565 LCD screen, that would be 300KiB, which it'd be too much for a memory-constrained device.

   However, when memory is not a constraint, framebuffer may offer applications a faster way to update display contents once writing to the RAM-mapped buffer is faster than doing multiple SPI transfers.

   For memory-constrained devices, consider using :doc:`lcd`.
