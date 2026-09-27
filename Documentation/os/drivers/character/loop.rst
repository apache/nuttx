=================
Loop device
=================

Makes a **file** look like a **block device**, so that something which
expects a disk can be given a file instead.  The usual reason is to mount a
file system image without having a partition to put it on.  The Kconfig help
names character devices too, but whatever is wrapped has to report a size
through ``stat()``: the driver divides that size by the sector size to get
the number of sectors, and refuses the setup with ``-ERANGE`` if there is
not room for even one.

Enabled with ``CONFIG_DEV_LOOP``.  The code is in ``drivers/loop/``, and the
control device is ``/dev/loop``.

How it is used
==============

Two ioctls on ``/dev/loop``, defined in ``include/nuttx/fs/loop.h``:

``LOOPIOC_SETUP``
   Takes a ``struct losetup_s``.  After this, the node it names can be
   mounted.

``LOOPIOC_TEARDOWN``
   Takes the path of a node created by ``LOOPIOC_SETUP`` and removes it.

The structure carries more than the two paths:

.. code-block:: c

   struct losetup_s
   {
     FAR const char *devname;   /* The loop block device to be created */
     FAR const char *filename;  /* The file or character device to use */
     uint16_t sectsize;         /* The sector size to use with the block device */
     off_t offset;              /* An offset that may be applied to the device */
     bool readonly;             /* True: Read access will be supported only */
   };

``offset`` is the one worth knowing about.  Because the device starts that
many bytes into the file, a single image holding several partitions can be
mounted one partition at a time, without cutting it up first.

An application has to use the ioctls.  The driver does export ``losetup()``
and ``loteardown()`` as C functions, but they sit behind ``#ifdef
__KERNEL__`` in the header, under the note that they are internal OS
interfaces and not available to applications.

From the shell, NSH wraps the same two ioctls::

   losetup [-d <dev-path>] | [[-o <offset>] [-r] [-b <sect-size>] <dev-path> <file-path>]

``-d`` tears down, ``-o`` is the offset, ``-r`` makes it read-only and
``-b`` sets the sector size, which defaults to 512.

Why it is useful on an embedded system
======================================

It is easy to dismiss as a desktop convenience, but it earns its place:

* A file system image built on the host can be dropped onto whatever storage
  the board has -- even a ROMFS in flash -- and mounted from there, without
  the image having to own a partition.
* It makes file system code testable in the simulator, where there is no
  block device at all but there are plenty of files.

What to watch for
=================

The loop device adds a layer: every block access becomes a file access,
which becomes whatever the file's own file system does.  Two file systems
are now in the path, and so are two sets of buffers.  Fine for images and
for testing; a poor idea for anything on a hot path.
