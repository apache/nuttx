========
LITTLEFS
========

A little fail-safe filesystem designed for microcontrollers from
https://github.com/littlefs-project/littlefs.

In NuttX, littlefs can be interacted with through the virtual file system. This
means that it can be used normally with ``read()``, ``write()``, etc. calls, as
well as file operations (``fopen()``, ``fclose()``, etc).

.. note::

   Since littlefs is power-fail safe and must commit writes before they are
   permanently stored (corruption prevention mechanism), it is required to
   periodically call ``fsync`` on the file to commit your writes. The exact
   semantics of when littlefs files are committed are discussed in `this issue
   <https://github.com/apache/nuttx/issues/15840>`_.


.. note::

  If your littlefs setup is experiencing crashes when you boot, try
  troubleshooting by tweaking the ``BLOCK_SIZE_FACTOR`` options in Kconfig. A
  factor of 4 works well for SD cards.

.. warning::

   The littlefs support on NuttX only works with mtd drivers, for storage
   devices such as flash chips, SD cards and eMMC. Performance on SD cards and
   eMMC devices is worse than flash.

User identity and permissions
=============================

When both ``CONFIG_FS_PERMISSION`` and ``CONFIG_FS_LITTLEFS_ATTR_UPDATE`` are
enabled, littlefs stores owner, group, and mode in per-file custom attributes.
This requires :ref:`user-identity` (``CONFIG_SCHED_USER_IDENTITY``).

**Metadata.** ``chmod``/``chown`` (via ``chstat``) update the stored mode,
UID, and GID. ``stat`` and ``ls -l`` read them back.

**Creation.** New files and directories are created with the effective UID and
GID of the creating task.

**Open checks.** ``open()`` enforces POSIX permission bits using the caller's
effective identity:

* Existing files: each path component must grant search (``X_OK``) permission;
  the final component must grant the access requested by ``oflags``.
* ``O_CREAT`` on a non-existent file: the parent directory must grant write and
  search permission.

**Directory reads.** ``opendir()`` requires read and search permission on the
target directory.

Files created before permission support was enabled, or without stored
attributes, default to mode ``0777`` until ``chmod``/``chown`` sets explicit
metadata.
