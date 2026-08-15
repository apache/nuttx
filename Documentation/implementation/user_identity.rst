.. _user-identity:

=======================
User and Group Identity
=======================

When ``CONFIG_SCHED_USER_IDENTITY`` is enabled, each task group maintains POSIX
process credentials. All threads within a task group share the same credentials
(see :ref:`tasks-vs-threads`).

Credentials
===========

The full POSIX three-field credential model is stored in ``struct task_group_s``
(``include/nuttx/sched.h``):

* ``tg_uid`` / ``tg_gid`` — real user and group IDs.
* ``tg_euid`` / ``tg_egid`` — effective IDs used for permission checks.
* ``tg_suid`` / ``tg_sgid`` — saved set-IDs that allow a non-root process to
  restore a previously held effective ID.
* ``tg_groups`` / ``tg_ngroups`` — supplementary group IDs (when
  ``CONFIG_SCHED_NGROUPS`` is greater than zero).

All six primary credential fields are zero-initialized at task creation, so
the initial task runs as root (UID/GID 0) unless explicitly changed.  The
supplementary list starts empty.

Supplementary Groups
====================

When ``CONFIG_SCHED_NGROUPS`` is greater than zero:

* ``setgroups()`` replaces the calling task group's supplementary list
  (requires effective UID 0).
* ``getgroups()`` returns that list as stored (may be empty after
  ``setgroups(0, NULL)``).  The effective GID is not invented into an
  empty list; use ``getegid()`` for the effective GID.
* ``initgroups()`` builds a membership list with ``getgrouplist()`` (from
  ``/etc/group`` when ``CONFIG_LIBC_GROUP_FILE`` is enabled) and installs it
  with ``setgroups()``.
* ``NGROUPS_MAX`` equals ``CONFIG_SCHED_NGROUPS``.

Filesystem DAC (``fs_checkmode()``) grants the group-class mode bits when the
file's group matches ``tg_egid`` **or** any entry in ``tg_groups``.

Inheritance
===========

When a new task is created, ``group_inherit_identity()`` in
``sched/group/group_create.c`` copies all credential fields — including the
supplementary group list — from the parent task group to the child.

Privilege Transitions
=====================

``setuid()`` and ``setgid()``
-----------------------------

When the effective ID is zero (root):

* ``setuid(uid)`` sets ``tg_uid``, ``tg_euid``, and ``tg_suid`` to ``uid``.
* ``setgid(gid)`` sets ``tg_gid``, ``tg_egid``, and ``tg_sgid`` to ``gid``.

When the effective ID is non-zero:

* The caller may only set the effective ID to the current real or saved value.
* Any other value causes the function to return ``-1`` with ``errno`` set to
  ``EPERM``.

``seteuid()`` and ``setegid()``
-------------------------------

When the effective ID is zero, any value may be assigned as the new effective
ID.

When the effective ID is non-zero, the requested value must equal the real or
the saved ID. Otherwise the function returns ``-1`` with ``errno`` set to
``EPERM``.

This implements temporary privilege drop with ``seteuid()`` /
``setegid()`` and later restore from the saved ID.

``setreuid()`` and ``setregid()``
---------------------------------

These functions set the real and/or effective IDs in a single call. When the
effective ID is zero, any requested real and effective values may be assigned
and the saved set-ID is updated accordingly. When the effective ID is
non-zero, each requested value must equal the current effective ID, saved
set-ID, or (for the effective argument only) the real ID; otherwise the call
returns ``-1`` with ``errno`` set to ``EPERM``. When the real ID is changed,
or the effective ID is changed to a value not equal to the real ID, the saved
set-ID is set to the new effective ID.

``getresuid()`` and ``getresgid()``
-----------------------------------

These functions return the real, effective, and saved set-IDs for the calling
task group. Any output pointer may be ``NULL`` if that ID is not needed.

``setresuid()`` and ``setresgid()``
-----------------------------------

These functions set the real, effective, and saved set-IDs in one call.
Pass ``(uid_t)-1`` / ``(gid_t)-1`` to leave an ID unchanged.  When the
effective UID is zero, any values may be assigned.  When the effective
UID is non-zero, each new ID must equal the current real, effective, or
saved ID.

Soft drop (keep saved-root)::

  setresgid(gid, gid, 0);
  setresuid(uid, uid, 0);

Hard drop (clear saved-root)::

  setresgid(gid, gid, gid);
  setresuid(uid, uid, uid);

``setresgid()`` requires effective UID zero to assign arbitrary GIDs.
Change group IDs before dropping the effective UID.

Setuid-on-exec
================

When ``binfmt`` loads an executable with the set-user-ID bit set
(``S_ISUID`` in ``nx_mode``), the new task group's effective and saved
UIDs become the file owner's UID while the real UID remains the caller's.
This is the mechanism used by the setuid-root ``sudo`` helper
(``CONFIG_SYSTEM_SUDO``); see :ref:`cmdsudo`.  A valid test logs in as
root, drops to an unprivileged sudoers account (``su user``), confirms
that account cannot exec a root-only binary, then runs that binary
through ``sudo``.

Configuration
=============

``CONFIG_SCHED_USER_IDENTITY``
  Enables per-task-group credential tracking. Without this option, stub
  root-only versions of all credential interfaces are provided.

``CONFIG_SCHED_NGROUPS``
  Maximum supplementary group IDs per task group (default 8).  Visible only
  when ``CONFIG_SCHED_USER_IDENTITY`` is enabled.  Becomes ``NGROUPS_MAX``.
  ``getgrouplist()`` / ``initgroups()`` return failure (they do **not**
  silently truncate) when membership exceeds this limit; ``initgroups()``
  also logs a warning.  Increase ``CONFIG_SCHED_NGROUPS`` if needed.

``CONFIG_FS_PERMISSION``
  Enables filesystem ownership and permission enforcement. Requires
  ``CONFIG_SCHED_USER_IDENTITY`` and ``CONFIG_PSEUDOFS_ATTRIBUTES``.
  See :ref:`file-permission` for the VFS helpers, mount-crossing
  traverse rules, and testing notes.

Pseudo-Filesystem Ownership
===========================

When ``CONFIG_PSEUDOFS_ATTRIBUTES`` and ``CONFIG_SCHED_USER_IDENTITY`` are both
enabled, ``inode_alloc()`` assigns ``i_owner`` and ``i_group`` from the
caller's effective credentials. This covers
message queues (``mq_open()``), named semaphores (``sem_open()``), shared
memory objects (``shm_open()``), FIFOs (``mkfifo()``), and pseudo-files
created through the same inode reservation path.

Path resolution requires directory search permission (``X_OK``) on ancestors
via ``inode_checkpathperm()``.  Open-time checks on the final node use
``inode_checkopenperm()`` (or ``inode_checkperm()`` for named IPC
objects).  Full details, including mounts under private pseudoFS parents,
are in :ref:`file-permission`.
