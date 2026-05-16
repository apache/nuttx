.. _device-nodes:

============
Device Nodes
============

Linux Device Nodes
==================

I used to have good Linux expertise a decade or so ago.
But my current Linux knowledge is dated and rusty.
I don't know anything about udev, SystemD, devtmpfs, sysfs, or any of that.
So this is my simplified understanding.

Device files work quite a bit differently in Linux and NuttX.
A device node in Unix/Linux only really contains the only type of the device
and its major and minor device numbers, i.e., it just holds data.
So creating the device node does not install or create the driver;
it simply writes a tiny file containing some special data.

Nothing happens until you try to open the device.
If something in the operating system has not initialized and registered
a driver for that type and major/minor numbers,
then you fail to open the device.

So the device nodes and the device drivers are decoupled in Linux/Unix
and there is a rendezvous that must occur later for the device node
to actually refer to the device.

  "(..) Linux maps the device special file passed in system calls
  (say to mount a file system on a block device) to the device's
  device driver using the major device number and a number of system
  tables, ...The major number is actually the offset into the
  kernel's device driver table, which tells the kernel what
  kind of device it is (whether it is a hard disk or a serial
  terminal) (..)"

  -- Source:
     www.linux-tutorial.info/modules.php?name=MContent&pageid=94.

Normally, when you create a Linux file system, you also create all of the
standard device nodes. But most of these do not map to real devices.
If you try to access most of the devices under ``/dev`` in Linux, they will
fail because the underlying driver that maps to that major/minor number
has not been initialized.


NuttX Device Nodes
==================

NuttX does not use major/minor device numbers and there are no device
"system tables" to associate major/minor numbers to a driver implementation.
NuttX simplifies this be removing the "man in the middle": When you register
the driver, you also create the device node.

.. important:: The device node IS the driver registry.
               This is a tremendous simplification and one of the things that
               makes NuttX usable in the constrained MCU environment.

In NuttX, device nodes are not really files at all.
They are special entries in the NuttX root pseudo-filesystem.
See :ref:`NuttX Pseudo File System <nuttx-pseudofs>` for more details.

Usage Differences
=================

.. important:: Only devices drivers can create device nodes and the existence
               of the device node means that the device has been initialized,
               registered, and is ready for use (with the exception of some
               removable devices that may not actually be ready).

               You cannot create device nodes from applications!

You could argue that this simplification is a deviation from my Unix/Linux
roadmap and would have to agree that you are right.
But it is also the kind of enabling simplification that makes a tiny
Unix-like operating system feasible on these lower end MCUs.

In Linux standard device drivers are initialized and registered as with NuttX.
A (privileged) application can create a device node, but cannot initialize
or register a device driver directly (as far as I know).
I believe that if you want to instantiate an uninitialized, unregistered
device driver you would have to install a kernel module containing
the driver (which would probably also create the device nodes corresponding
to the driver).

boardctl()
==========

NuttX does support a sneak interface to support interactions with board-level
OS logic. That sneak interface is ``boardctl()`` (see :ref:`board-ioctl` and
:ref:`nuttx-initialization-sequence` for more details).
That interface could potentially be used to force initialization of device
drivers by application code. That discussion is to be provided.
