==================
Pseudo File System
==================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Pseudo+File+System

Overview
========

Pseudo Root File System
-----------------------

NuttX includes an optional, scalable file system. As a minimum, this may 
be a simple in-memory, pseudo file system. This is an in-memory file 
system because it does not require any storage medium or block driver 
support. Rather, file system contents are generated on-the-fly as 
referenced via standard file system operations (open, close, read, 
write, etc.). In this sense, the file system is a pseudo file system 
(in the same sense that the Linux ``/proc`` file system is also referred 
to as a pseudo file system).

Any user supplied data or logic can be accessed via the pseudo-file 
system. Built in support is provided for character, block, and MTD 
(Memory Technology Device) drivers in the ``/dev`` pseudo file system 
directory.

Special Files
-------------

NuttX does not support special files in the way that, say, 
Linux does. In fact, it is more correct to say that NuttX 
file systems do not support special files at all.

NuttX does, however, support Linux-like special `device node`, 
character driver, and block driver files (as well as NuttX-specific 
mountpoint, named semaphore, message queue, and shared memory 
special files). However, these are not special files in sense 
that the term special files is used in a POSIX environment: In 
NuttX these special files may `only` be created in the root 
pseudo-file system. For the case of device nodes, see `Device 
Nodes <https://cwiki.apache.org/confluence/display/NUTTX/Device+Nodes>`_ 
for further information.

In NuttX, the underlying principle is that all `named resources` 
appear as special files in the root pseudo-file system and are 
managed by the VFS.

Mounted Volumes
---------------

The simple in-memory file system can be extended by mounting block 
devices that provide access to true file systems backed up via 
some mass storage device. NuttX supports the standard mount() 
command that allows a block driver to be bound to a mount point 
within the pseudo file system and to a file system. At present, 
NuttX supports the standard VFAT and ROMFS file systems, a 
special, wear-levelling NuttX FLASH File System (NXFFS), as well 
as a Network File System client (NFS version 3, UDP).

Comparison to Linux
-------------------

From a programming perspective, the NuttX file system appears 
very similar to a Linux file system. However, there is a 
fundamental difference: The NuttX root file system is a pseudo 
file system and true file systems may be mounted in the pseudo 
file system. In the typical Linux installation by comparison, 
the Linux root file system is a true file system and pseudo 
file systems may be mounted in the true, root file system. 
The approach selected by NuttX is intended to support greater 
scalability from the very tiny platform to the moderate platform.

FAQ
===

**Question**: I'm wondering why I can't create a directory. If 
I try to create a dir.

.. code-block:: bash

    mkdir /mnt

I get this,

.. code-block:: bash

    nsh: mkdir: mkdir failed: 2

although if I do this it creates both directories, mnt and sda

.. code-block:: bash

    mount -t vfat /dev/mmcsd0 /mnt/sda

**Answer**: This is because the top level directories are part of a 
`pseudo-filesystem` – like the Linux ``proc/`` or ``sys/`` file systems. 
But the NuttX pseudo-file system begins at the top level ``/``.

What that really means is that you do must have 
``CONFIG_DISABLE_PSEUDOFS_OPERATIONS`` selected. Because you 
can normally create directories in the pseudo-filesystem 
with not problem:

.. code-block:: bash

    NuttShell (NSH) NuttX-9.0.0
    nsh> mkdir /mnt
    nsh> ls
    /:
    dev/
    etc/
    mnt/
    proc/
    tmp/
    nsh> ls mnt
    /mnt:
    nsh>

But lets assume that you do have operations on the pseudo-file 
system disabled. Why doesn't it work? There is no `real` media 
there so you cannot create a file there or create any directories 
there. The ``mount`` command is special, it knows how to create mount 
points in the pseudo-file system.

The pseudo-file system is just a tree structure in RAM. 
It serves two purposes: (1) you don't have to have a real 
file system to use NuttX.
It comes up out-of-the-box with usable (but limited) 
pseudo-file system. That allows a little more civilized 
programming environment on even very resource limited MCUs. 
And (2) this pseudo-file system is a place where all special 
NuttX files are retained: Character drivers, block drivers, 
and mount points.

The NuttX top-level pseudo-filesystem creates the `illusion` of 
directories and provides a consistent, seamless semantic for 
interacting with mounted file systems. If there is a file 
called ``hello.txt`` in your volume mounted at ``/mnt/sda``, then:

``/mnt`` - is a `node` in the pseudo-filesystem that does 
nothing but contain the name mnt and provide links 
to things `under` ``mnt``.

``/mnt/sda`` - This refers to a node that contains the name 
sda that can be found `under` the node with the name mnt.
This node is a special `mountpoint node` in the pseudo-filesystem.
It contains the methods needed to interact will real file system.
Everything `below` ``/mnt/sda`` is in the physical media.

``/mnt/sda/hello.txt`` - This, then refers to the file 
``hello.txt`` at the relative path ``hello.txt`` on the mounted media.
The transition from the pseudo-filesystem to the 
real media is seamless.

This is a little different from Linux: Linux always 
has to boot up with a `real` file system – even if it 
is only a initrd RAM disk.
In Linux, these special files (links, drivers, pipes, 
etc.) reside on real media and can reside in any 
Linux-compatible filesystem.

Normal ``mkdir`` can only work if there is a `real` filesystem 
at the location. There are no real directories in the 
pseudo-filesystem. The pseudo-filesystem does support 
`nodes` that look like directories and have some of the 
properties of directories (like the node ``/mnt`` mentioned 
above). But this is really an illusion.

If ``CONFIG_DISABLE_PSEUDOFS_OPERATIONS`` is not enabled, 
then NuttX adds the capability to create new, empty `nodes` 
in the pseudo-filesystem using ``mkdir``, completing the illusion.

[On the other hand, all directories are really an 
`illusion` in a way and I suppose that in that sense 
these nodes the pseudo-filesystem are just as `real` 
as any other directory.]

After you mount the SD card at ``/mnt/sda``, then you can do:

.. code-block:: bash

    mkdir /mnt/sda/newdir

That should work fine and should create a directory at 
the relative path ``newdir`` in the mounted volume.

There are a few other special NSH commands like mount that 
can change the pseudo-filesystem. Like ``losetup``, ``mkfifo``, 
``mkrd``, ``umount``, etc.
In fact, these commands `only` work in the pseudo-filesystem. 
Try them in ``/mnt/sda``... they won't work.

But none of the `normal` commands that modify files or directories 
will work in the pseudo-filesystem: ``mkdir``, ``mv``, ``rm``, ``rmdir``. 
These all require real media. They will not work in the 
pseudo-filesystem, but will work in ``/mnt/sda``.

And trying to pipe to something in the pseudo-filesystem 
will also fail. You cannot do this, for example:

.. code-block:: bash

    NuttShell (NSH) NuttX-6.20
    nsh> cat "Hello, World!" >/hello.text
    nsh: cat: open failed: 22
    nsh>

See also NxFileSystem in 
`Porting Guide <https://cwiki.apache.org/confluence/display/NUTTX/Porting+Guide>`_