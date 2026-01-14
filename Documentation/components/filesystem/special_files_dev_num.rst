================================
Special Files and Device Numbers
================================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Special+Files+and+Device+Numbers

Special Files in Unix-like File Systems
=======================================

In Unix-like operating systems, a `special file` is an interface for 
a device driver that appears in a file system as if it were an 
ordinary file. Special files are a feature built into Unix-like 
file systems such as EXT2.

See also `Wikipedia article <https://en.wikipedia.org/wiki/Device_file>`_.


Special Files in Other File Systems
===================================

Some other file systems systems, such as NTFS, also have some more 
limited and incompatible notion of special files. Others, such as 
FAT, do not have any such concept. Unix-like environments such as 
Cygwin that run on these other file systems have to do things a 
little differently: The have to emulate special files using the 
resources of the file system that they operate on. This may mean 
creating `regular` files with special naming conventions or with 
special content that can be used to emulate the behavior of 
special files.

Special Files and NuttX
=======================

NuttX has done things in a very different way. There are no 
special files supported in any file system. Rather, special 
files can exist only in the NuttX :doc:`pseudo file system <pseudofs>`. This 
was a decision that was made in the initial design to simplify 
things for resource limited platforms yet still provide a mostly 
standard Unix-like/POSIX programming environment.

What are the advantages of the special files in the NuttX 
pseudo-file system? Reduce resource usage, reduced bring-up 
requirements. What are the disadvantages? In NuttX, special 
files can only reside in the pseudo-file system.

The only other consequence that I aware of is that NuttX cannot 
support the POSIX requirement for the ``st_dev`` field in the 
``struct stat`` structure.


Device Files and Device Numbers
===============================

In a Unix-like system, devices are access special device files. 
In a Unix-like system, device files can reside in any compatible 
file system but, by convention, are always placed in the ``/dev`` 
directory. NuttX achieves programming compatibility with this 
convention because the top-level, `root` file system is the 
pseudo-file system and ``/dev`` is part of the pseudo-file system.

But there is a bigger difference that this. The bigger 
difference is how device drivers are registered and how 
the are accessed. The primary content of the Unix-like 
device file is simply a number, a device number, usually 
represented as type ``dev_t``. The device number an encoded 
that consists of a `major` device number and a `minor` device 
number. The major device number identifies the type of 
driver and the minor number identifies an instance of a 
driver of that type.

There is nothing special about these number from the sense of 
a file system. Just because device file exists with a 
certain major and minor number, that does not mean that there 
is actually any real driver instance backing that device file 
up. For example, you can create a device file from the Linux 
command line like this, knowing nothing other than major and 
minor device number:

.. code-block:: C

    dev_t makedev(unsigned int maj, unsigned int min);

In a Unix-like system, when you try to open a device driver 
several things must happen: The system must open the device 
file, obtain the device major and minor number, and then 
look up the driver instance in some internal `registry` of 
registered device drivers. If one is found, then the system 
can complete the open operation.

So, the device number is then a `key` of some kind into a 
`registry` of device drivers. NuttX does this very differently: 
There are no device numbers, rather the pseudo-file system `is` 
the device registry! In NuttX, device files cannot be created 
by users; they can only be created by device drivers by calling 
the following, internal interface:

.. code-block:: c 

    int register_driver(FAR const char *path, FAR const struct file_operations *fops, mode_t mode, FAR void *priv);

The ``path`` argument determines where in the pseudo-file system the 
device file should be placed. ``mode`` provides device file privileges. 
The ``fops`` and ``priv`` provide the internal information for the ``registry``.

So the when you open a device driver in NuttX, many fewer steps are 
involved: The system must still open the device file, but then 
since the pseudo-file system `is` the device registry, all of the 
device driver information is available and ``open`` operation 
completes with no further actions.

Named Resources
===============

This use of the pseudo-file system in NuttX to manage device 
files is consistent with a core NuttX device philosophy: The 
NuttX VFS and the pseudo-file system in particular, are used 
to manage all named OS resources. That applies not only to 
device files and other special files but also to such things 
named message queues and named semaphores (which can be found 
in the pseudo-file system in the ``/var`` directory).