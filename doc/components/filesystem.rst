=================
NuttX File System
=================

**Overview**. NuttX includes an optional, scalable file system.
This file-system may be omitted altogether; NuttX does not depend
on the presence of any file system.

**Pseudo Root File System**. A simple *in-memory*, *pseudo* file
system can be enabled by default. This is an *in-memory* file
system because it does not require any storage medium or block
driver support. Rather, file system contents are generated
on-the-fly as referenced via standard file system operations
(open, close, read, write, etc.). In this sense, the file system
is *pseudo* file system (in the same sense that the Linux
``/proc`` file system is also referred to as a pseudo file
system).

Any user supplied data or logic can be accessed via the
pseudo-file system. Built in support is provided for character and
block `drivers <#DeviceDrivers>`__ in the ``/dev`` pseudo file
system directory.

**Mounted File Systems** The simple in-memory file system can be
extended my mounting block devices that provide access to true
file systems backed up via some mass storage device. NuttX
supports the standard ``mount()`` command that allows a block
driver to be bound to a mountpoint within the pseudo file system
and to a file system. At present, NuttX supports the standard VFAT
and ROMFS file systems, a special, wear-leveling NuttX FLASH File
System (NXFFS), as well as a Network File System client (NFS
version 3, UDP).

**Comparison to Linux** From a programming perspective, the NuttX
file system appears very similar to a Linux file system. However,
there is a fundamental difference: The NuttX root file system is a
pseudo file system and true file systems may be mounted in the
pseudo file system. In the typical Linux installation by
comparison, the Linux root file system is a true file system and
pseudo file systems may be mounted in the true, root file system.
The approach selected by NuttX is intended to support greater
scalability from the very tiny platform to the moderate platform.

