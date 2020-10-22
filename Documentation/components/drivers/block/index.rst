====================
Block Device Drivers
====================

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

-  **Examples**. ``drivers/loop.c``,
   ``drivers/mmcsd/mmcsd_spi.c``, ``drivers/ramdisk.c``, etc.


