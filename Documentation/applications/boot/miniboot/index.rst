========
miniboot
========

Minimal bootloader based on NuttX.

Configuration options:

- ``CONFIG_MINIBOOT_SLOT_PATH`` - The path to the application firmware image
  slot character device driver. Default: ``/dev/ota0``
- ``CONFIG_MINIBOOT_HEADER_SIZE`` - Application firmware image header size
