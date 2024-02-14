=====
TMPFS
=====

NuttX TMPFS file system is a tiny dyamic RAM based file system. 

It can be enabled by adding ``CONFIG_FS_TMPFS=y`` to the configuration at build time. 

At runtime, simply use ``mount -t tmpfs /tmp`` to have a ``/tmp`` folder backed by TMPFS, then files and folders can be created under that folder. 

Be aware that TMPFS is backed by kernel memory thus don't expect to store big files on it and its size is limited by free kernel memory.

We can watch the size of TMPFS with ``df -h`` command, especially you can see the ``Size`` column of TMPFS changes when files are added or removed in the TMPFS folder. Changes in TMPFS size is always reflected by reverse changes of free kernel memory size.
