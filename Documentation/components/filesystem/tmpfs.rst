=====
TMPFS
=====

NuttX TMPFS file system is a tiny dynamic RAM based file system.

It can be enabled by adding ``CONFIG_FS_TMPFS=y`` to the configuration at build time. 

At runtime, simply use ``mount -t tmpfs /tmp`` to have a ``/tmp`` folder backed by TMPFS, then files and folders can be created under that folder. 

Be aware that TMPFS is backed by kernel memory thus don't expect to store big files on it and its size is limited by free kernel memory.

We can watch the size of TMPFS with ``df -h`` command, especially you can see the ``Size`` column of TMPFS changes when files are added or removed in the TMPFS folder. Changes in TMPFS size is always reflected by reverse changes of free kernel memory size.

Permissions
===========

When ``CONFIG_FS_PERMISSION`` is enabled, tmpfs stores per-object owner, group,
and mode and enforces them on path operations inside the volume.  It also
implements the optional ``mountpt_operations.permission`` hook.

Access into the mount from the pseudoFS is gated by the VFS (parent and
mountpoint ``X_OK``).  See :ref:`file-permission`.
