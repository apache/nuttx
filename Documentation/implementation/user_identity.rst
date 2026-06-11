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

All six fields are zero-initialized at task creation, so the initial task runs
as root (UID/GID 0) unless explicitly changed.

Inheritance
===========

When a new task is created, ``group_inherit_identity()`` in
``sched/group/group_create.c`` copies all six credential fields from the parent
task group to the child task group.

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

This implements the standard POSIX pattern of temporarily dropping privileges
with ``seteuid()`` or ``setegid()`` and later restoring them to the saved value.

Configuration
=============

``CONFIG_SCHED_USER_IDENTITY``
  Enables per-task-group credential tracking. Without this option, stub
  root-only versions of all credential interfaces are provided.

``CONFIG_FS_PERMISSION``
  Enables filesystem ownership and permission enforcement. Requires
  ``CONFIG_SCHED_USER_IDENTITY``.
