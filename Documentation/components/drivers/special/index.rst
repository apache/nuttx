==========================
Specialized Device Drivers
==========================

All device drivers that are accessible to application logic are
either: (1) Character device drivers that can be accessed via the
standard driver operations (``open()``, ``close()``, ``read()``,
``write()``, etc.), or (2) block drivers that can be accessing
only as part of mounting a file system or other special use cases
as described in the preceding paragraph.

In addition to this, there are also specialized "drivers" that can
be used only within the OS logic itself and are not accessible to
application logic. These specialized drivers are discussed in the
following section.

.. note::
  While special drivers are *internal*, in some cases there are also
  character/block drivers that sit on top of these special drivers
  and thus expose them to applications.

.. toctree::
  :caption: Supported Drivers

  syslog.rst
  spi.rst
  i2c.rst
  ethernet.rst
  socketcan.rst
  framebuffer.rst
  lcd.rst
  mtd.rst
  sdio.rst
  usbhost.rst
  usbdev.rst
  rwbuffer.rst
  regmap.rst
