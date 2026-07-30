================================
``hidkbd`` USB Host HID keyboard
================================

This is a simple test to ``debug/verify`` the USB host HID keyboard class driver.

- ``CONFIG_EXAMPLES_HIDKBD_DEFPRIO`` – Priority of waiter thread. Default: ``50``.
- ``CONFIG_EXAMPLES_HIDKBD_STACKSIZE`` – Stacksize of waiter thread. Default
  ``1024``.
- ``CONFIG_EXAMPLES_HIDKBD_DEVNAME`` – Name of keyboard device to be used.
  Default: ``/dev/kbda``.
- ``CONFIG_EXAMPLES_HIDKBD_ENCODED`` –  Decode special key press events in the
  user buffer. In this case, the example coded will use the interfaces defined
  in ``include/nuttx/input/kbd_codec.h`` to decode the returned keyboard data.
  These special keys include such things as up/down arrows, home and end keys,
  etc. If this not defined, only 7-bit printable and control ASCII characters
  will be provided to the user. Requires
  ``CONFIG_INPUT_KEYBOARD_BYTESTREAM`` and ``CONFIG_LIBC_KBDCODEC``.

.. note::

   This example reads the byte stream that the USB HID keyboard driver used
   to deliver on its own, which now comes from
   ``CONFIG_INPUT_KEYBOARD_BYTESTREAM``. New work should use ``kbd``
   instead: it reads any keyboard, in either format, and reports key
   releases as well.
