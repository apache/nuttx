===========================
``/dev/null`` and friends
===========================

The small pseudo-devices that behave like files but are not backed by
anything.  They exist because a program should be able to write output
nowhere, or read an endless supply of zeros, without that being a special
case in the program.

The code is in ``drivers/misc/``.

.. list-table::
   :header-rows: 1
   :widths: 18 22 60

   * - Device
     - Enabled by
     - Behaviour
   * - ``/dev/null``
     - ``CONFIG_DEV_NULL``
     - Reads return end of file immediately; writes succeed and discard
       everything.  The usual way to silence output that you cannot turn off
       at the source.
   * - ``/dev/zero``
     - ``CONFIG_DEV_ZERO``
     - Reads return as many zero bytes as asked for and never end; writes are
       discarded.  Useful for filling a buffer or a file with a known value,
       and for measuring how fast something can read.
   * - ``/dev/mem``
     - ``CONFIG_DEV_MEM``
     - Physical memory as a file: the file position *is* the address, and a
       read or write is a ``memcpy()`` to or from it.  It also supports
       ``mmap()``, which is the usual way to reach a register block.
       Powerful and unguarded: it is a debugging tool, not a production
       interface.
   * - ``/dev/ascii``
     - ``CONFIG_DEV_ASCII``
     - Reads return the printable ASCII characters, ``!`` through ``~``,
       cycling forever, with a newline in the place where the space would
       fall.  That is what makes it a test pattern you can read as text
       rather than one endless line.

``/dev/null`` and ``/dev/zero`` cost almost nothing to enable, and both
default to on unless ``CONFIG_DEFAULT_SMALL`` is set.  ``/dev/mem`` and
``/dev/ascii`` default to off.  On a system that is genuinely tight, the
first two are among the first things to turn off, because a program that
really needs them is rare.

Each is registered by a call the board makes during bring-up --
``devnull_register()``, ``devzero_register()``, ``devmem_register()`` and
``devascii_register()`` -- declared in ``include/nuttx/drivers/drivers.h``.
