.. include:: /substitutions.rst
.. _running:

Running
=======

Embedded boards have different ways to get your program onto them and get them running. This guide assumes your board
has a JTAG connector, and you have a JTAG hardware debugger like a
`Segger J-Link <https://www.segger.com/products/debug-probes/j-link/>`_. `JTAG <https://en.wikipedia.org/wiki/JTAG>`_
is a set of standards that let you attach a hardware device to your embedded board, and then remotely control the CPU.
You can load code, start, stop, step through the program, and examine variables and memory.

#. Attach the Debugger Cables

#. Start the Debugger

   Refer to your JTAG debugger's documentation for information on how to start a GDB Server process that gdb can
   communicate with to load code and start, stop, and step the embedded board's CPU. Your command line may be
   different from this one.

    .. code-block:: bash

       $ JLinkGDBServer -device ATSAMA5D27 -if JTAG -speed 1000 -JTAGConf -1,-1

#. Launch the GNU Debugger

   In another terminal window, launch the GDB for your platform. In the case of this guide, this came with the
   ARM Embedded GNU Toolchain we downloaded in the Install step.

    .. code-block:: bash

       $ arm-none-eabi-gdb

#. Connect to the board's serial console

   Usually you connect a USB-to-serial adapter to the board's serial console so you can see debug logging or
   execute Apache NuttX Shell (nsh) commands. You can access the serial console from Linux with the ``picocom`` terminal
   program. From another terminal, do this:

    .. code-block:: bash

       $ picocom -b 115200 /dev/ttyUSB0

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

       (gdb) file nuttx
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

----

Next up is :ref:`configuring`.
