=====================
Serial Device Drivers
=====================

-  ``include/nuttx/serial/serial.h``. All structures and APIs
   needed to work with serial drivers are provided in this header
   file.

-  ``struct uart_ops_s``. Each serial device driver must
   implement an instance of ``struct uart_ops_s``. That structure
   defines a call table with the following methods:

-  ``int uart_register(FAR const char *path, FAR uart_dev_t *dev);``.
   A serial driver may register itself by calling
   ``uart_register()``, passing it the ``path`` where it will
   appear in the :ref:`pseudo file system <file_system_overview>` and it's
   initialized instance of ``struct uart_ops_s``. By convention,
   serial device drivers are registered at paths like
   ``/dev/ttyS0``, ``/dev/ttyS1``, etc. See the
   ``uart_register()`` implementation in ``drivers/serial.c``.

-  **User Access**. Serial drivers are, ultimately, normal
   `character drivers <#chardrivers>`__ and are accessed as other
   character drivers.

-  **Examples**: ``arch/arm/src/stm32/stm32_serial.c``,
   ``arch/arm/src/lpc214x/lpc214x_serial.c``,
   ``arch/z16/src/z16f/z16f_serial.c``, etc.

