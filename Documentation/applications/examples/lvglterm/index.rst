==============================================
``lvglterm`` LVGL Terminal for NuttShell (NSH)
==============================================

LVGL application that runs an interactive NuttShell (NSH) on the display.  NSH
is started with its standard streams redirected through pipes, its output is
rendered in an LVGL text area, and the input comes from one of two sources
selected at build time.

The shared code lives in ``lvglterm.c`` (NSH startup, output rendering, main
loop); ``lvglterm_touch.c`` and ``lvglterm_kbd.c`` implement the two input
variants.

Input variants
==============

The input source is chosen with the *LVGL Terminal input source* Kconfig
choice (only one is built at a time):

On-screen keyboard (touch)
    ``CONFIG_EXAMPLES_LVGLTERM_INPUT_TOUCH`` (default).  An LVGL keyboard
    widget operated by touch; a command line is typed and submitted with
    Enter.  Suits touchscreen boards and keeps the original behaviour.

    .. figure:: lvglterm-touch.png
       :align: center
       :width: 500px
       :alt: LVGL Terminal with the on-screen touch keyboard

       On-screen keyboard (touch) variant

Physical keyboard
    ``CONFIG_EXAMPLES_LVGLTERM_INPUT_KBD``.  Key events are read from a
    ``/dev/kbdN`` keyboard device and streamed to the shell; the output fills
    the whole screen.  Requires ``CONFIG_INPUT_KEYBOARD``.

    The device defaults to ``CONFIG_EXAMPLES_LVGLTERM_KBD_DEV``
    (``/dev/kbd0``) and can be overridden at run time by passing the path as
    the first argument (``lvglterm /dev/kbd1``) when more than one keyboard is
    present.  Keyboards that report the Fn navigation cluster as cursor keys
    can scroll the output with Up/Down.

    .. figure:: lvglterm-kbd.png
       :align: center
       :width: 500px
       :alt: LVGL Terminal driven by a physical keyboard

       Physical keyboard variant

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
- A display (``CONFIG_LV_USE_NUTTX_LCD`` or a framebuffer).  The touch variant
  also needs a touchscreen input (``CONFIG_LV_USE_NUTTX_TOUCHSCREEN``); the
  keyboard variant needs a keyboard driver (``CONFIG_INPUT_KEYBOARD``).
- The selected font (``CONFIG_LV_FONT_UNSCII_8`` or
  ``CONFIG_LV_FONT_UNSCII_16``) is enabled automatically by the font choice.

Reference: `NuttX RTOS for PinePhone: LVGL Terminal for NSH Shell
<https://lupyuen.github.io/articles/terminal>`_.
