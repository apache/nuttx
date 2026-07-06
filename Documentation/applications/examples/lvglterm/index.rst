==============================================
``lvglterm`` LVGL Terminal for NuttShell (NSH)
==============================================

LVGL application that runs an interactive NuttShell (NSH) on the display.  NSH
is started with its standard streams redirected through pipes, its output is
rendered in an LVGL text area, and the input comes from one of three sources
selected at build time.

The shared code lives in ``lvglterm.c`` (NSH startup, output rendering, main
loop); ``lvglterm_touch.c`` and ``lvglterm_kbd.c`` implement the input
variants.

Input variants
==============

The input source is chosen with the *LVGL Terminal input source* Kconfig
choice (only one is built at a time).  The physical-keyboard options differ in
the data the keyboard device returns on ``read()``, so the one that matches the
hardware must be selected.

On-screen keyboard (touch)
    ``CONFIG_EXAMPLES_LVGLTERM_INPUT_TOUCH`` (default).  An LVGL keyboard
    widget operated by touch; a command line is typed and submitted with
    Enter.  Suits touchscreen boards and keeps the original behaviour.

    .. figure:: lvglterm-touch.png
       :align: center
       :width: 500px
       :alt: LVGL Terminal with the on-screen touch keyboard

       On-screen keyboard (touch) variant

Matrix / upper-half keyboard
    ``CONFIG_EXAMPLES_LVGLTERM_INPUT_KBD_MATRIX``.  ``struct
    keyboard_event_s`` events are read from a keyboard registered through the
    ``CONFIG_INPUT_KEYBOARD`` upper half (for example the M5Stack Cardputer
    matrix keyboard on ``/dev/kbd0``) and streamed to the shell; the output
    fills the whole screen.  The Fn Up/Down cursor keys scroll the output.

    .. figure:: lvglterm-kbd.png
       :align: center
       :width: 500px
       :alt: LVGL Terminal driven by a matrix keyboard

       Matrix / upper-half keyboard variant

USB HID keyboard
    ``CONFIG_EXAMPLES_LVGLTERM_INPUT_KBD_USB``.  A USB HID keyboard (for
    example on ``/dev/kbda`` with ``CONFIG_USBHOST_HIDKBD``) delivers a byte
    stream that is decoded with the keyboard codec and streamed to the shell.
    When the driver is built with ``CONFIG_HIDKBD_ENCODED`` the Up/Down cursor
    keys scroll the terminal.

    .. figure:: lvglterm-usb.png
       :align: center
       :width: 500px
       :alt: LVGL Terminal driven by a USB HID keyboard

       USB HID keyboard variant

Both keyboard variants default the device to
``CONFIG_EXAMPLES_LVGLTERM_KBD_DEV`` (``/dev/kbd0`` for the matrix keyboard,
``/dev/kbda`` for USB) and can be overridden at run time by passing the path
as the first argument (``lvglterm /dev/kbd1``) when more than one keyboard is
present.

Font
====

The monospaced terminal font is chosen with the *LVGL Terminal font* choice:
``CONFIG_EXAMPLES_LVGLTERM_FONT_UNSCII_16`` (default), or the smaller
``CONFIG_EXAMPLES_LVGLTERM_FONT_UNSCII_8`` for low-resolution displays where
UNSCII 16 shows too few columns.

Configuration
=============

- ``CONFIG_LIBC_EXECFUNCS=y`` -- ``posix_spawn()`` must be enabled.
- ``CONFIG_PIPES=y`` -- Pipes must be enabled.
- ``CONFIG_SYSTEM_NSH=y`` -- the NSH library must be enabled.
- ``CONFIG_GRAPHICS_LVGL=y`` and ``CONFIG_LV_USE_NUTTX=y`` -- LVGL with its
  NuttX integration.
- A display (``CONFIG_LV_USE_NUTTX_LCD`` or a framebuffer).  Each input variant
  also needs its own driver: a touchscreen
  (``CONFIG_LV_USE_NUTTX_TOUCHSCREEN``) for the touch variant,
  ``CONFIG_INPUT_KEYBOARD`` for the matrix variant, or
  ``CONFIG_USBHOST_HIDKBD`` for the USB variant.  The USB variant selects
  ``CONFIG_LIBC_KBDCODEC`` automatically and, for the cursor keys, the driver
  should be built with ``CONFIG_HIDKBD_ENCODED``.
- The selected font (``CONFIG_LV_FONT_UNSCII_8`` or
  ``CONFIG_LV_FONT_UNSCII_16``) is enabled automatically by the font choice.

Reference: `NuttX RTOS for PinePhone: LVGL Terminal for NSH Shell
<https://lupyuen.github.io/articles/terminal>`_.
