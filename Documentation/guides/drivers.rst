.. include:: /substitutions.rst
.. _drivers:

Drivers
=======

Some NuttX boards don't have full support for all the on-chip peripherals. If you need support for this hardware,
you will either need to port a driver from another chip, or write one yourself. This section discusses how to do that.

.. _drivers-porting:

Porting a Driver
----------------

Often support for on-chip peripherals exists in a closely related chip, or even a different family or from a different
manufacturer. Many chips are made up of different Intellectual Property (IP) blocks that are licensed from vendors like
Cadence, Synopsys, and others. The IP blocks may be similar enough to use another chip's driver with little
modification.

* Find a similar driver in NuttX source code:

  * Look at register names listed in the datasheet for the peripheral.
  * Search the NuttX codebase for the register names (try several different ones).
  * Note that you'll have to compare the datasheet to the header and code files to see if there are differences; there
    will usually be some differences between architectures, and they can be significant.

* Find a similar driver in U-Boot, Zephyr or BSD Unix (OpenBSD, FreeBSD, NetBSD) source code:

  * Only for inspiration, you can't copy code because of license incompatibility and Apache Foundation restrictions.
    (Apache License 2.0 and BSD code can come in with a software grant agreement from the original authors; this can
    sometimes be hard to get. Ask on the mailing list if you're unsure.)
  * But you can debug to see how the driver works.
  * `U-Boot <https://www.denx.de/wiki/U-Boot>`_ drivers are often easier to understand than BSD Unix drivers because
    U-Boot is simpler.

* Understanding how the driver works

  Here are a couple of techniques that helped me.

    * printf debugging

      * Sprinkle ``custinfo()`` logging statements through your code to see execution paths and look at variables
        while the code is running. The reason to use ``custinfo()`` as opposed to the other logging shortcuts
        (``mcinfo()``, etc.) is that you can turn on and off other other logging and still see your custom debug
        logging. Sometimes it's useful to quiet the flood of logging that comes from a particular debug logging
        shortcut.
      * Note that printing info to the console will affect timing.
      * Keep a file with just your debug settings in it, like this (``debugsettings``):

        .. code-block:: c

           CONFIG_DEBUG_CUSTOM_INFO=y
           (etc..)

      * Add the settings to the end of your ``.config`` file after running ``make menuconfig`` (that will reorder
        the file, making it harder to see and change the debug settings if you need to).

        .. code-block:: bash

           $ cat .config debugsettings > .config1 ; mv .config1 .config

      * If you are using interrupts and threads (many things in NuttX run in different threads as a response to interrupts),
        you can use printf debugging to see overlapping execution.

        * Interrupts - here's how to inspect the C stack frame to see what execution environment is currently running:

          .. code-block:: c

            uint32_t frame = 0;  /* MUST be the very first thing in the function */
            uint32_t p_frame;
            frame++;             /* make sure that frame doesn't get optimized out */
            p_frame = (uint32_t)(&frame);
            custinfo("p_frame: %08x\n", p_frame);

        * Threads - here's how to get the thread identifier to see which thread is currently executing:

          .. code-block:: c

            pthread_t thread_id = pthread_self();
            custinfo("pthread_id: %08x\n", thread_id);

      * stack frame printf
      * thread printf

    * `GDB — the GNU Debugger <https://www.gnu.org/software/gdb/>`_

      GDB is a great tool. In this guide we've already used it to load our program and run it. But it can do a lot
      more. You can single-step through code, examine variables and memory, set breakpoints, and more. I generally use
      it from the commandline, but have also used it from an IDE like JetBrains' Clion, where it's easier to see the
      code context.

      One add-on that I found to be essential is the ability to examine blocks of memory, like buffers that NuttX uses
      for reading and writing to storage media or network adapters. This `Stack Overflow question on using GDB to
      examine memory <https://stackoverflow.com/a/54784260/431222>`_ includes a GDB command that is very handy. Add
      this to your ``.gdbinit`` file, and then use the ``xxd`` command to dump memory in an easy-to-read format:

      .. code-block::

         define xxd
           if $argc < 2
             set $size = sizeof(*$arg0)
           else
             set $size = $arg1
           end
           dump binary memory dump.bin $arg0 ((void *)$arg0)+$size
           eval "shell xxd -o %d dump.bin; rm dump.bin", ((void *)$arg0)
         end
         document xxd
           Dump memory with xxd command (keep the address as offset)

           xxd addr [size]
             addr -- expression resolvable as an address
             size -- size (in byte) of memory to dump
                     sizeof(*addr) is used by default end

      Here's a short GDB session that shows what this looks like in practice. Note that the memory location being
      examined (``0x200aa9eo`` here) is a buffer being passed to ``mmcsd_readsingle``:

      .. code-block::

        Program received signal SIGTRAP, Trace/breakpoint trap.
        0x200166e8 in up_idle () at common/arm_idle.c:78
        78	}
        (gdb) b mmcsd_readsingle
        Breakpoint 1 at 0x2006ea70: file mmcsd/mmcsd_sdio.c, line 1371.
        (gdb) c
        Continuing.

        Breakpoint 1, mmcsd_readsingle (priv=0x200aa8c0, buffer=0x200aa9e0 "WRTEST  TXT \030", startblock=2432) at mmcsd/mmcsd_sdio.c:1371
        1371	  finfo("startblock=%d\n", startblock);
        (gdb) xxd 0x200aa9e0 200
        200aa9e0: 5752 5445 5354 2020 5458 5420 1800 0000  WRTEST  TXT ....
        200aa9f0: 0000 0000 0000 0000 0000 5500 1100 0000  ..........U.....
        200aaa00: 5752 5445 5354 3520 5458 5420 1800 0000  WRTEST5 TXT ....
        200aaa10: 0000 0000 0000 0000 0000 5800 1500 0000  ..........X.....
        200aaa20: e552 5445 5854 3620 5458 5420 1800 0000  .RTEXT6 TXT ....
        200aaa30: 0000 0000 0000 0000 0000 5600 1200 0000  ..........V.....
        200aaa40: 5752 5445 5354 3620 5458 5420 1800 0000  WRTEST6 TXT ....
        200aaa50: 0000 0000 0000 0000 0000 5600 1200 0000  ..........V.....
        200aaa60: 0000 0000 0000 0000 0000 0000 0000 0000  ................
        200aaa70: 0000 0000 0000 0000 0000 0000 0000 0000  ................
        200aaa80: 0000 0000 0000 0000 0000 0000 0000 0000  ................
        200aaa90: 0000 0000 0000 0000 0000 0000 0000 0000  ................
        200aaaa0: 0000 0000 0000 0000                      ........


NuttX Drivers as a Reference
----------------------------

If you're not porting a NuttX driver from another architecture, it still helps to look at other similar NuttX
drivers, if there are any. For instance, when implementing an Ethernet driver, look at other NuttX Ethernet drivers;
for an SD Card driver, look at other NuttX Ethernet drivers. Even if the chip-specific code won't be the same, the
structure to interface with NuttX can be used.

Using Chip Datasheets
---------------------

To port or write a driver, you'll have to be familiar with the information in the chip datasheet. Definitely find
the datasheet for your chip, and read the sections relevant to the peripheral you're working with. Doing so ahead
of time will save a lot of time later.

Another thing that's often helpful is to refer to sample code provided by the manufacturer, or driver code from
another operating system (like U-Boot, Zephyr, or FreeBSD) while referring to the datasheet — seeing how working
code implements the necessary algorithms often helps one understand how the driver needs to work.

* How to use a datasheet

  Key pieces of information in System-on-a-Chip (SoC) datasheets are usually:

  * Chip Architecture Diagram — shows how the subsections of the chip (CPU, system bus, peripherals, I/O, etc.) connect
    to each other.
  * Memory Map — showing the location of peripheral registers in memory. This info usually goes into a header file.
  * DMA Engine — if Direct Memory Access (DMA) is used, this may have info on how to use it.
  * Peripheral — the datasheet usually has a section on how the peripheral works. Key parts of this include:

    * Registers List — name and offset from the base memory address of the peripheral. This needs to go into a header
      file.
    * Register Map — what is the size of each register, and what do the bits mean? You will need to create ``#defines``
      in a header file that your code will use to operate on the registers. Refer to other driver header files for
      examples.

Logic Analyzers
---------------

For drivers that involve input and output (I/O), especially that involve complex protocols like SD Cards, SPI, I2C,
etc., actually seeing the waveform that goes in and out the chip's pins is extremely helpful. `Logic Analyzers <https://en.wikipedia.org/wiki/Logic_analyzer>`_
can capture that information and display it graphically, allowing you to see if the driver is doing the right thing
on the wire.

DMA Debugging
-------------

* Dump registers before, during, and after transfer. Some NuttX drivers (``sam_sdmmc.c`` or ``imxrt_sdmmc.c`` for
  example) have built-in code for debugging register states, and can sample registers before, during, and
  immediately after a DMA transfer, as well as code that can dump the peripheral registers in a nicely-formatted
  way onto the console device (which can be a serial console, a network console, or memory). Consider using something
  like this to see what's happening inside the chip if you're trying to debug DMA transfer code.
* Compare register settings to expected settings determined from the datasheet or from dumping registers from working
  code in another operating system (U-Boot, Zephyr, FreeBSD, etc.).
* Use the ``xxd`` GDB tool mentioned above to dump NuttX memory buffers before, during, and after a transfer to see if
  data is being transferred correctly, if there are over- or under-runs, or to diagnose data being stored in incorrect
  locations.
* printf debugging register states can also help here.
* Remember that logging can change the timing of any algorithms you might be using, so things may start or stop
  working when logging is added or removed. Definitely test with logging disabled.

