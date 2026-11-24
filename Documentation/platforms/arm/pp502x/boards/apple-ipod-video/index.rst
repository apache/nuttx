================
Apple iPod Video
================

.. tags:: arch:arm, chip:pp502x, vendor:apple

The Apple iPod Video, 5th generation (5G) and 5.5G (A1136), has a PortalPlayer
PP502x, 32 MiB or 64 MiB of SDRAM and a hard drive. The 5.5G is model MA446LL/A.
One board covers both generations. The port was developed on a 5.5G
(MA446LL/A); ``ostest`` passes on that unit in a build that reports over
USB in place of the UART. The 5G is expected to work and has not been run.

The start-up code finds the size of the SDRAM and gives the heap all of it. The
image is linked in the first 32 MiB.

The PP502x is described in :doc:`../../index`.

Serial Console
==============

The console is UART 0, which the port routes to the dock connector, at the
default of the 16550 driver (115200 baud, 8N1). The UART takes the 24 MHz
crystal as its clock. The console on a unit has not been tested as it requires
a specialized iPod 30-pin dock connector that I do not have on-hand.

Installing
==========

The image is a flat binary that is linked at address 0 and starts with the
exception vector table. The boot ROM loads the OS image of the firmware
partition to the start of SDRAM and enters it at its first byte. The start-up
code copies the image to the start of SDRAM if it was loaded elsewhere.

Writing the image over the OS image of the firmware partition replaces the
installed OS. Save the installed image first and keep it outside the iPod, and
use an iPod that holds nothing you need. ``ipodpatcher`` from the Rockbox
utilities reads and writes the firmware partition; the
`Rockbox wiki <https://www.rockbox.org/wiki/IpodPatcher>`__ has the binaries
for Linux, macOS and Windows and points at the source. It needs raw disk
access, so run it as root, and give it the device node if it finds more than
one disk or the iPod is formatted for macOS.

1. Put the iPod in disk mode and connect it to the host.
2. Check the layout. Continue only if the tool reports the iPod Video::

     ipodpatcher --scan
     ipodpatcher -l

3. Save the installed OS image and the firmware partition::

     ipodpatcher -rf original.ipod
     ipodpatcher -r firmware-partition.bin

4. Build ``nuttx.bin`` with ``make`` and pack it into the file format of
   ``ipodpatcher``. The file is a 4 byte checksum, the 4 byte model tag
   ``ipvd`` and the image. The checksum is big-endian and is 5 plus the sum of
   all bytes of the image, modulo 2^32. For example::

     python3 - <<'PY'
     import struct
     d = open("nuttx.bin", "rb").read()
     d += b"\0" * (-len(d) % 4)
     open("nuttx.ipod", "wb").write(
         struct.pack(">I", (5 + sum(d)) & 0xffffffff) + b"ipvd" + d)
     PY

5. Write the image over the OS entry. ``-wf`` checks the tag and the
   checksum::

     ipodpatcher -wf nuttx.ipod

6. Restart the iPod. To return to the saved OS, write it back with
   ``ipodpatcher -wf original.ipod``.
   With the replaced entry the disk mode gesture of the installed OS is not
   available, so use the one of the boot ROM: slide Hold on and off, hold Menu
   and Select to reset the iPod, then hold Select and Play/Pause until the
   disk mode screen shows.

Configurations
==============

nsh
---

Configures the NuttShell (NSH) on the console. The configuration has no
board-specific start-up commands.

ostest
------

Configures the OS test of ``apps/testing/ostest`` as the init program. On
completion the console shows ``ostest_main: Exiting with status 0``.
