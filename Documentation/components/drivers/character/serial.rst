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
   If``TTY_LAUNCH_ENTRY`` you can set program entry by ``TTY_LAUNCH_ENTRYPOINT``.
   If``TTY_LAUNCH_FILE`` you can set file path by ``TTY_LAUNCH_FILEPATH``.

   Also, you can customize:
   ``TTY_LAUNCH_ARGS`` ``TTY_LAUNCH_PRIORITY`` ``TTY_LAUNCH_STACKSIZE``

-  **User Access**. Serial drivers are, ultimately, normal
   `character drivers <#chardrivers>`__ and are accessed as other
   character drivers.

-  **Examples**: ``arch/arm/src/stm32/stm32_serial.c``,
   ``arch/arm/src/lpc214x/lpc214x_serial.c``,
   ``arch/z16/src/z16f/z16f_serial.c``, etc.

Serial Error Reporting
----------------------

It is possible to check if there are some frame, parity, overrun, break, or
other error using the ioctl TIOCGICOUNT just like on Linux.

Serial Debug Structure (TIOCSERGSTRUCT)
---------------------------------------

.. note::
   This is a **debug-only** ioctl. The internal structures it exposes are
   driver-specific, may change without notice, and must not be relied upon
   as a stable ABI.

The ``TIOCSERGSTRUCT`` ioctl allows a developer to retrieve a copy of the
serial driver's internal state structure for diagnostic and debugging purposes.
It is defined in ``include/nuttx/serial/tioctl.h``::

   #define TIOCSERGSTRUCT  _TIOC(0x0032)  /* Get device TTY structure */

Enabling ``TIOCSERGSTRUCT``
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Support is controlled by the Kconfig option ``CONFIG_SERIAL_TIOCSERGSTRUCT``.
To enable it:

1. ``CONFIG_DEBUG_FEATURES`` must be enabled (the option depends on it).
2. Either ``CONFIG_MCU_SERIAL`` or ``CONFIG_16550_UART`` must be active (i.e.,
   the board must use an MCU serial driver or the generic 16550 UART driver).
3. The specific low-level serial driver for your hardware must implement the
   ``TIOCSERGSTRUCT`` case in its ``ioctl`` method. Most serial drivers in
   the tree already do (63+ drivers across ARM, ARM64, RISC-V, Xtensa, and
   MIPS architectures).

Via ``menuconfig``, navigate to:

.. code-block:: text

   Device Drivers  --->
     Serial Driver Support  --->
       [*] Support TIOCSERGSTRUCT

If the option is not visible, ensure that ``CONFIG_DEBUG_FEATURES`` is enabled
first.

How It Works
~~~~~~~~~~~~

Because the exact layout depends on the serial driver selected for your board,
there is no single portable structure definition. The caller must consult the
driver source for the struct definition and size the buffer accordingly.

If ``arg`` is ``NULL``, the ioctl returns ``-EINVAL``.

Example Usage
~~~~~~~~~~~~~

The following example shows how an application might use ``TIOCSERGSTRUCT``
with the 16550 UART driver to inspect internal state. Adapt the structure type
and header to match the serial driver used on your board.

.. code-block:: c

   #include <stdio.h>
   #include <fcntl.h>
   #include <unistd.h>
   #include <sys/ioctl.h>
   #include <nuttx/serial/tioctl.h>

   /* Include the driver-specific header for the struct definition.
    * This example uses the 16550 UART; replace with the header that
    * defines your board's serial driver state structure.
    */

   #include <nuttx/serial/uart_16550.h>

   int main(int argc, char *argv[])
   {
     struct u16550_s devstate;
     int fd;
     int ret;

     fd = open("/dev/ttyS0", O_RDONLY);
     if (fd < 0)
       {
         perror("open");
         return 1;
       }

     ret = ioctl(fd, TIOCSERGSTRUCT, (unsigned long)&devstate);
     if (ret < 0)
       {
         perror("ioctl TIOCSERGSTRUCT");
         close(fd);
         return 1;
       }

     /* Inspect driver-internal fields for debugging.  Field names
      * are specific to the driver; consult the driver source for
      * the struct definition.
      */

     printf("UART base address: 0x%08lx\n",
            (unsigned long)devstate.uartbase);

     close(fd);
     return 0;
   }

.. warning::
   The structure layout and field names are internal to each driver
   implementation and **may change between NuttX releases**. Use this ioctl
   for interactive debugging and diagnostics only — never in production
   application logic.
