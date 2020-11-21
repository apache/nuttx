.. include:: /substitutions.rst
.. _debugging:

Debugging
=========

Finding and fixing bugs is an important part of the hardware and software development process. Sometimes you also need
to use debugging techniques to understand how the system works. Two tools that are helpful are debug logging and
debugging using the GNU Debugger (gdb).

Debug Logging
-------------

NuttX has a powerful system logging facility (syslog) with ``info``, ``warn``, and ``error`` levels. You can enable
debugging for your build for the subsystem or feature by using the ``menuconfig`` system.

The debug options are available under :menuselection:`Build Setup --> Debug Options`. You will most likely have to enable the
following options:

* ``Enable Debug Features`` — selecting this will turn on subsystem-level debugging options, they will become visible
  on the page below. You can then select the ones you want.
* ``Enable Error Output`` — this will only log errors.
* ``Enable Warnings Output`` — this will log warnings and errors.
* ``Enable Informational Debug Output`` — this will produce informational output, warnings, and errors.

You can then select from the subsystems that are available, Network, Scheduler, USB, etc. Note that you will need to
separately enable the subsystem elsewhere in the ``menuconfig`` system. To see the ``CONFIG`` define that is set,
use the arrow keys to highlight the subsystem (for instance, ``Network Debug Features``) and type '?'. This will show
you that the C macro that is set is called ``CONFIG_DEBUG_NET``. ``debug.h`` defines the ``netinfo()`` logging
function that will log output if this macro is set. You can search the source code for ``netinfo`` to see how it is
used.

.. image:: ../_static/images/menuconfig-debug.png
    :width: 800px
    :align: center
    :alt: Screenshot of menuconfig system main screen

Note that enabling all these will produce an incredible amount of logging output. Enable the level you want and
the area you're interested in, and leave the rest disabled, save the config, and then recompile. You can see the full
list of debug feature logging functions in the file
`debug.h <https://github.com/apache/incubator-nuttx/blob/master/include/debug.h>`__.

Syslog timestamps can be enabled in the configuration in :menuselection:`Device Drivers --> System Logging --> Prepend
timestamp to syslog message` (``CONFIG_SYSLOG_TIMESTAMP``).

You may need to do a little bit of experimenting to find the combination of logging settings that work for the problem
you're trying to solve. See the file `debug.h <https://github.com/apache/incubator-nuttx/blob/master/include/debug.h>`_
for available debug settings that are available.

There are also subsystems that enable USB trace debugging, and you can log to memory too, if you need the logging to be
faster than what the console can output.

Debugging with ``openocd`` and ``gdb``
--------------------------------------

To debug our Nucleo board using its embedded SWD debug adapter,
start ``openocd`` with the following command:

.. code-block:: console

  $ openocd -f interface/st-link-v2.cfg -f target/stm32f1x.cfg

This will start a ``gdb`` server. Then, start ``gdb`` with:

.. code-block:: console

  $ cd nuttx/
  $ gdb-multiarch nuttx/nuttx

Inside ``gdb`` console, connect to the ``gdb`` server with:

.. code-block::

  (gdb) target extended-remote :3333

You can now use standard ``gdb`` commands.

Debugging with an external JTAG adapter
---------------------------------------

.. todo::
  Explain this with openocd. It gives the impression that JTAG requires
  a specific tool. Also, some of the example commands apply to both cases.
  This repeats some of the above.

If your board does not have an embedded programmer and uses
`JTAG <https://en.wikipedia.org/wiki/JTAG>`_ connector instead,
things are a bit different. This guide assumes you have a JTAG hardware debugger like a
`Segger J-Link <https://www.segger.com/products/debug-probes/j-link/>`_.
JTAG is a set of standards that let you
attach a hardware device to your embedded board, and then remotely control the CPU.
You can load code, start, stop, step through the program, and examine variables and memory.

#. Attach the Debugger Cables

#. Start the Debugger

   Refer to your JTAG debugger's documentation for information on how to start a GDB Server process that gdb can
   communicate with to load code and start, stop, and step the embedded board's CPU. Your command line may be
   different from this one.

    .. code-block:: console

       $ JLinkGDBServer -device ATSAMA5D27 -if JTAG -speed 1000 -JTAGConf -1,-1

#. Launch the GNU Debugger

   In another terminal window, launch the GDB. In the case of this guide, this came with the
   ARM Embedded GNU Toolchain we downloaded in the Install step.

    .. code-block:: console

       $ cd nuttx/
       $ gdb-multiarch nuttx/nuttx

#. Set gdb to talk with the J-Link

    ::

       (gdb) target extended-remote :2331

#. Reset the board

    ::

       (gdb) mon reset

#. You may need to switch to the serial console to hit a key to stop the board from booting from its boot monitor
   (U-Boot, in the case of the SAMA5 boards from Microchip).

#. Halt the board

    ::

       (gdb) mon halt

#. Load nuttx

    ::

       (gdb) load nuttx
       `/home/adamf/src/nuttx-sama5d36-xplained/nuttx/nuttx' has changed; re-reading symbols.
       Loading section .text, size 0x9eae4 lma 0x20008000
       Loading section .ARM.exidx, size 0x8 lma 0x200a6ae4
       Loading section .data, size 0x125c lma 0x200a6aec
       Start address 0x20008040, load size 654664
       Transfer rate: 75 KB/sec, 15587 bytes/write.
       (gdb)

#. Set a breakpoint

    ::

       (gdb) breakpoint nsh_main

#. Start nuttx

    ::

       (gdb) continue
       Continuing.

       Breakpoint 1, nsh_main (argc=1, argv=0x200ddfac) at nsh_main.c:208
       208	  sched_getparam(0, &param);
       (gdb) continue
       Continuing.

Debugging Shortcuts
-------------------

Note that you can abbreviate ``gdb`` commands, ``info b`` is a shortcut for
``information breakpoints``; ``c`` works the same as ``continue``, etc.

See this article for more info:
`Debugging a Apache NuttX target with GDB and OpenOCD <https://micro-ros.github.io/docs/tutorials/advanced/nuttx/debugging/>`_.

----

Next up is :ref:`organization`.
