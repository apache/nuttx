.. _file-permission:

========================================
Filesystem Permission Interface
========================================

When ``CONFIG_FS_PERMISSION`` is enabled, the VFS applies POSIX-style
discretionary access control (DAC) using the caller's effective credentials
(``tg_euid`` / ``tg_egid``).  This page describes the common inode helpers,
how mountpoints participate, and how access across a mount is gated by
pseudoFS directory modes.

Prerequisite reading: :ref:`user-identity`.

Configuration
=============

=============================== =============================================
Option                          Role
=============================== =============================================
``CONFIG_SCHED_USER_IDENTITY``  Per-task-group UID/GID credentials
``CONFIG_PSEUDOFS_ATTRIBUTES``  Store ``i_mode`` / ``i_owner`` / ``i_group``
                                on pseudoFS inodes
``CONFIG_FS_PERMISSION``        Enable DAC helpers and VFS enforcement
                                (depends on the two options above)
=============================== =============================================

Without ``CONFIG_FS_PERMISSION``, the helpers described here return success
and no mode-based checks are performed.

Helpers
=======

``inode_checkperm``
  Check ``amode`` (``R_OK`` / ``W_OK`` / ``X_OK``) against an inode's
  ``i_owner`` / ``i_group`` / ``i_mode``.  Empty macro returning ``0``
  when ``CONFIG_FS_PERMISSION`` is disabled.

``inode_checkpathperm``
  Require ``X_OK`` on every ancestor of an inode, and on the inode itself
  when it is a pseudo directory or a mountpoint (directory search /
  traverse).  If ``amode`` is non-zero, also require that access on the
  inode.  Takes the inode tree read lock unless ``INODE_CHECK_LOCKED`` is
  set (caller already holds ``inode_lock`` / ``inode_rlock``).  Empty
  macro returning ``0`` when ``CONFIG_FS_PERMISSION`` is disabled.

``inode_checkopenperm``
  Validate that the inode supports the requested open access, then apply
  mode checks for non-mountpoint inodes.

``fs_checkmode``
  Core owner/group/other test used by the helpers above and by filesystems
  such as tmpfs and littlefs.

Optional mountpoint hook
------------------------

``struct mountpt_operations`` may provide a ``permission`` method for
in-volume DAC.  The field is at the **end** of the structure so existing
positional initialisers remain valid.

* **tmpfs** implements ``tmpfs_permission``.
* Filesystems without Unix ownership on disk (for example FAT and ROMFS)
  leave the method ``NULL``.

The VFS mount-crossing gate does **not** depend on this hook.  Entry into a
volume is enforced with ``inode_checkpathperm`` against the mountpoint
inode's stored ``i_mode``.  In-volume checks remain the filesystem's job
(tmpfs and littlefs enforce DAC inside their own open/mkdir/path helpers.
``mops->permission`` is an optional common entry point for the same policy;
the VFS does not invoke it for mount-crossing).

Open vs traverse
================

Mountpoint inodes are **not** open-mode-checked by ``inode_checkopenperm``.
Applying the mount directory's mode bits as file open modes would require
read/write on the mount directory merely to open a file beneath it.

Traverse is separate: callers use ``inode_checkpathperm`` so parent
directories and the mountpoint itself still require ``X_OK``.

Typical order after a successful ``inode_find``:

1. ``inode_checkpathperm(inode, 0, 0)`` — search permission on the path
   prefix (and on the mountpoint when entering a volume).  Call sites that
   already hold the tree lock pass ``INODE_CHECK_LOCKED``; mount and
   pseudoFS create/remove may pass a non-zero ``amode`` (for example
   ``W_OK``) to combine traverse and target checks in one call.
2. Operation-specific checks — ``inode_checkopenperm``, or the
   filesystem's own methods for paths inside a mount.

Mount-crossing
==============

Path walk stops at a mountpoint and returns that inode plus a relative path
into the volume.  Without traverse checks, a restrictive mode on a pseudoFS
parent would not protect objects under a filesystem mounted beneath it.

Example::

  /secure_dir          # pseudoFS directory, mode 0700, owner root
  /secure_dir/mnt      # mounted volume (tmpfs, FAT, ...)
  /secure_dir/mnt/a    # object inside the volume

``inode_checkpathperm`` requires ``X_OK`` on ``secure_dir`` and on the
mountpoint ``mnt``.  A non-root open of ``/secure_dir/mnt/a`` therefore
returns ``EACCES``, even if the mounted filesystem itself has no Unix DAC.

Where the checks run
--------------------

* After ``inode_find`` in open, unlink, mkdir, rmdir, rename, stat, chstat,
  statfs, readlink, mount, and umount.  ``inode_checkpathperm`` takes
  ``inode_rlock`` unless the caller already holds the tree lock
  (``INODE_CHECK_LOCKED``).
* Inside ``inode_reserve`` / ``inode_remove`` for pseudoFS create and remove
  (ancestor ``X_OK`` and parent ``W_OK`` in one call, under ``inode_lock``).

Who enforces what
-----------------

* **PseudoFS parent dirs** — ``X_OK`` (and ``W_OK`` when creating/removing)
* **Mountpoint inode** — ``X_OK`` to enter the volume (stored ``i_mode``)
* **Inside the volume** — Filesystem methods; optional ``mops->permission``
* **FAT / ROMFS** — No Unix ownership on disk; entry still gated by the
  mountpoint ``i_mode``

References
==========

* ``fs/inode/fs_inode.c`` — ``inode_checkperm``, ``inode_checkpathperm``
* ``include/nuttx/fs/fs.h`` — ``struct mountpt_operations``
* ``fs/tmpfs/fs_tmpfs.c`` — ``tmpfs_permission``
* :ref:`user-identity` — credential model
* :doc:`/components/filesystem/littlefs` — littlefs in-volume DAC
* :doc:`/components/filesystem/tmpfs` — tmpfs overview
