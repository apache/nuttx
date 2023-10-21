``lvglterm`` LVGL Terminal for NuttShell (NSH)
==============================================

LVGL application that executes NuttShell (NSH) commands entered with a
Touchscreen Keyboard and displays the NSH output. Prerequisite configuration
settings:

- ``CONFIG_NSH_ARCHINIT=n`` – NSH architecture initialization must be disabled.
- ``CONFIG_NSH_CONSOLE=y`` – NSH must be configured to use a console.
- ``CONFIG_LIBC_EXECFUNCS=y`` – posix_spawn() must be enabled.
- ``CONFIG_PIPES=y`` – Pipes must be enabled.
- ``CONFIG_GRAPHICS_LVGL=y`` – LVGL graphics must be enabled.
- ``CONFIG_LV_FONT_UNSCII_16=y`` – LVGL font UNSCII 16 must be enabled.
