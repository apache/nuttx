.. _chardev:

========================
Character Device Drivers
========================

Character device drivers have these properties:

-  ``include/nuttx/fs/fs.h``. All structures and APIs needed
   to work with character drivers are provided in this header
   file.

-  ``struct file_operations``. Each character device driver
   must implement an instance of ``struct file_operations``. That
   structure defines a call table with the following methods:

-  ``int register_driver(const char *path, const struct file_operations *fops, mode_t mode, void *priv);``.
   Each character driver registers itself by calling
   ``register_driver()``, passing it the ``path`` where it will
   appear in the :ref:`pseudo file system <file_system_overview>` and it's
   initialized instance of ``struct file_operations``.

-  **User Access**. After it has been registered, the character
   driver can be accessed by user code using the standard driver
   operations including
   ``open()``, ``close()``, ``read()``, ``write()``, etc.

-  **Specialized Character Drivers**. Within the common character
   driver framework, there are different specific varieties of
   *specialized* character drivers. The unique requirements of the
   underlying device hardware often mandates some customization of
   the character driver. These customizations tend to take the
   form of:

   -  Device-specific ``ioctl()`` commands used to performed
      specialized operations on the device. These ``ioctl()`` will
      be documented in header files under ``include/nuttx`` that
      detail the specific device interface.
   -  Specialized I/O formats. Some devices will require that
      ``read()`` and/or ``write()`` operations use data conforming
      to a specific format, rather than a plain stream of bytes.
      These specialized I/O formats will be documented in header
      files under ``include/nuttx`` that detail the specific
      device interface. The typical representation of the I/O
      format will be a C structure definition.

   The specialized character drivers support by NuttX are
   documented in the following paragraphs.

-  **Examples**: ``drivers/dev_null.c``, ``drivers/fifo.c``,
   ``drivers/serial.c``, etc.

.. toctree::
  :caption: Supported Drivers

  serial.rst
  touchscreen.rst
  analog.rst
  pwm.rst
  can.rst
  quadrature.rst
  timer.rst
  rtc.rst
  watchdog.rst
  keypad.rst
  note.rst
  foc.rst
  ws2812.rst

