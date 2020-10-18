.. include:: /substitutions.rst
.. _running:

=======
Running
=======

In order to finally run NuttX on your board, you first have to flash the NuttX
binary. As an easy start, it is recommended that you choose a well supported board
which also integrates the debugger/programmer in the board itself exposed via USB
connector.

A good choice is a Nucleo or Discovery board from ST Microelectronics,
as there is a wide choice of suported boards for the STM32 architecture in NuttX.
Also, these boards expose an UART port over the USB connection which allows you
to interact with NuttX via the interactive console without any extra hardware.
For the purposes of this guide, we will use the Nucleo F103RB board.

Flashing
========

There are various tools you can use to flash the NuttX binary to your Nucleo
board. One common option is to use `openocd` which supports a large number
of programmers and target microcontrollers.

To install the stable version of openocd you can do:

.. code-block:: console

  $ apt install openocd

.. todo:: add instructions for other platforms

You should note that openocd project has not made stable releases for long
time and support for newer hardware will probably be only available in the
latest Git version. To install it you should:

.. code-block:: console

  $ git clone git://git.code.sf.net/p/openocd/code openocd
  $ cd openocd
  $ ./bootstrap
  $ ./configure --prefix install/
  $ make install

The resulting installation will be under ``openocd/install``. You can add
``openocd/install/bin`` to your ``PATH``.

Now, to flash the binary to your board, connect the USB cable and do:

.. code-block:: console

  $ cd nuttx/
  $ openocd -f interface/st-link-v2.cfg -f target/stm32f1x.cfg -c 'init' \
    -c 'program nuttx/nuttx.bin verify reset' -c 'shutdown'

Access NuttShell
================

Once you flash your board, it will reset and offer a prompt over the serial
console. With the Nucleo board, you can simply open the terminal program
of your choice where you will see the ``nsh>`` prompt:

.. tabs::

  .. code-tab:: console picocom (CLI)

    $ picocom -b 115200 /dev/ttyUSB0

  .. code-tab:: console gtkterm (GUI)

    $ gtkterm -s 115200 -p /dev/ttyUSB0


Debugging
=========

Using ``openocd`` you can also debug NuttX. To do so, first run:

.. code-block:: console

  $ openocd -f interface/st-link-v2.cfg -f target/stm32f1x.cfg

which will start a GDB server. Then, start ``gdb`` as:

.. code-block:: console

  $ cd nuttx/
  $ gdb-multiarch nuttx/nuttx

Inside ``gdb`` console, connect to the ``openocd`` server with:

.. code-block::

  (gdb) target extended-remote :3333

You can debug using standard ``gdb`` commands.

Advanced Debugging with JTAG
----------------------------

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

----

Next up is :ref:`configuring`.
