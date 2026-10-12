===============================
``nvmpg-dro`` NVMPG pendant DRO
===============================

Digital readout (DRO) firmware for the Digital Dream NVMPG CNC pendant
(:doc:`nvmpg-stm32f1 </platforms/arm/stm32f1/boards/nvmpg-stm32f1/index>`
board). It replaces the original firmware of the pendant and speaks the same
serial protocol with the NVUM/NVEM/EC300/EC500 motion controllers.

The application:

* Receives the status frames from the controller (``5A 5A`` followed by 51
  bytes) and answers the identify request (``5A 5B``) with ``01 13``.
* Draws the axis positions (XYZ or ABC page, depending on the selected axis),
  the FRO/SJR/SRO overrides, the MPG step and the RESET/COMM lamps. Only the
  fields that changed since the last frame are redrawn.
* Polls the keys every 10 ms, debounces them and sends the key code
  (key index + 2) when a key is pressed and ``code | 0x80`` when it is
  released.
* Draws the axis positions in grey when no frame was received for
  ``CONFIG_INDUSTRY_CNC_NVMPG_DRO_LINK_TIMEOUT`` milliseconds.
* Optionally shows the NuttX logo at startup.

The frame layout is described in the board documentation.

Drawing is done with ``LCDDEVIO_PUTAREA`` on the LCD character device and a
built-in 5x7 font. No framebuffer is used: a 220x176 RGB565 framebuffer would
need more RAM than the STM32F103RC has.

Configuration
=============

* ``CONFIG_INDUSTRY_CNC_NVMPG_DRO``: enable the application. Depends on
  ``CONFIG_LCD_DEV``, ``CONFIG_DEV_GPIO`` and pthread support.
* ``CONFIG_INDUSTRY_CNC_NVMPG_DRO_TTY``: serial port connected to the
  controller (default ``/dev/ttyS0``).
* ``CONFIG_INDUSTRY_CNC_NVMPG_DRO_LCD``: LCD character device (default
  ``/dev/lcd0``).
* ``CONFIG_INDUSTRY_CNC_NVMPG_DRO_NKEYS``: number of keys, read from
  ``/dev/gpio0`` .. ``/dev/gpioN-1`` (default 10).
* ``CONFIG_INDUSTRY_CNC_NVMPG_DRO_SPLASH``: show the NuttX logo at startup.
* ``CONFIG_INDUSTRY_CNC_NVMPG_DRO_SPLASH_MS``: how long the logo is shown
  (default 1000 ms).
* ``CONFIG_INDUSTRY_CNC_NVMPG_DRO_LINK_TIMEOUT``: link timeout in
  milliseconds (default 1000).

The NuttX logo image is the RGB565 run-length encoded logo from
``apps/examples/nximage``.
