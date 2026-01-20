====================
Block Device Drivers
====================

.. toctree::
  :maxdepth: 1

  ramdisk.rst


Block device drivers have these properties:

-  ``include/nuttx/fs/fs.h``. All structures and APIs needed
   to work with block drivers are provided in this header file.

-  ``struct block_operations``. Each block device driver must
   implement an instance of ``struct block_operations``. That
   structure defines a call table with the following methods:

-  ``int register_blockdriver(const char *path, const struct block_operations *bops, mode_t mode, void *priv);``.
   Each block driver registers itself by calling
   ``register_blockdriver()``, passing it the ``path`` where it
   will appear in the :ref:`pseudo file system <file_system_overview>` and
   it's initialized instance of ``struct block_operations``.

-  **User Access**. Users do not normally access block drivers
   directly, rather, they access block drivers indirectly through
   the ``mount()`` API. The ``mount()`` API binds a block driver
   instance with a file system and with a mountpoint. Then the
   user may use the block driver to access the file system on the
   underlying media. *Example*: See the ``cmd_mount()``
   implementation in ``apps/nshlib/nsh_fscmds.c``.

-  **Accessing a Character Driver as a Block Device**. See the
   loop device at ``drivers/loop.c``. *Example*: See the
   ``cmd_losetup()`` implementation in
   ``apps/nshlib/nsh_fscmds.c``.

-  **Accessing a Block Driver as Character Device**. See the
   Block-to-Character (BCH) conversion logic in ``drivers/bch/``.
   *Example*: See the ``cmd_dd()`` implementation in
   ``apps/nshlib/nsh_ddcmd.c``.

-  **Caching Block Driver** Any instantiated block device can
   be encapsulated in secondary device which uses a predefined
   amount of available RAM to provide read-ahead and write buffering.
   The secondary block driver then becomes the mountpoint for
   the target filesystem. Beware that any data not flushed to the
   underlying device, through ``umount`` or other filesystem
   mechanisms will be lost if the device is removed unexpectedly.

   See ``boards/risc-v/litex/arty_a7/src/litex_sdio.c`` and
   ``nuttx/nuttx/boards/risc-v/litex/arty_a7/Kconfig`` for example
   usage and configuration.

-  **Examples**. ``drivers/loop.c``,
   ``drivers/mmcsd/mmcsd_spi.c``, ``drivers/ramdisk.c``, etc.
