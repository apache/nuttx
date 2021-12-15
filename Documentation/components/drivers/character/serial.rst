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

-  **TTY_LAUNCH** this depends on ``CONFIG_TTY_LAUNCH``, this feature
   allow user launch a new program with a special char input.

   e.g. use ctrl+R to start a nuttx shell.
   e.g. use ctrl+E to start user entry.

   You can use ``TTY_LAUNCH_CHAR`` to customize which special char.

   You can choose launch method:
   ``TTY_LAUNCH_ENTRY`` or ``TTY_LAUNCH_FILE``,
   If``TTY_LAUNCH_ENTRY`` you can set program entery by ``TTY_LAUNCH_ENTRYPOINT``.
   If``TTY_LAUNCH_FILE`` you can set file path by ``TTY_LAUNCH_FILEPATH``.

   Also, you can customize:
   ``TTY_LAUNCH_ARGS`` ``TTY_LAUNCH_PRIORITY`` ``TTY_LAUNCH_STACKSIZE``

-  **User Access**. Serial drivers are, ultimately, normal
   `character drivers <#chardrivers>`__ and are accessed as other
   character drivers.

-  **Examples**: ``arch/arm/src/stm32/stm32_serial.c``,
   ``arch/arm/src/lpc214x/lpc214x_serial.c``,
   ``arch/z16/src/z16f/z16f_serial.c``, etc.

