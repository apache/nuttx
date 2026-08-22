.. _chroot:

======
chroot
======

``chroot()`` is a kernel-enforced filesystem jail.  When
``CONFIG_FS_CHROOT`` is enabled, each task group may pin a directory as
its root.  Absolute path lookup starts there, so the group cannot see
files outside that tree.

This is **not** a container.  NuttX does not provide PID, mount, or
network namespaces.  ``chroot()`` only changes where pathname lookup
begins.

Configuration
=============

Enable ``CONFIG_FS_CHROOT`` in the filesystem configuration.  The
syscall is then available from ``unistd.h``.

When ``CONFIG_SCHED_USER_IDENTITY`` is also enabled, ``chroot()``
requires effective UID 0 and returns ``-1`` with ``errno`` set to
``EPERM`` otherwise.  Without user identity every task is treated as
root, so ``chroot()`` remains allowed.  On ``CONFIG_BUILD_FLAT`` this
``euid == 0`` gate is the same class of check as credential DAC, not a
process-isolation boundary; see the trust-boundary note in
:ref:`user-identity`.

Semantics
=========

* ``chroot(path)`` resolves ``path`` relative to the caller's current
  root (so a nested ``chroot()`` cannot walk back to the host tree).
* ``path`` must name a directory (``ENOTDIR`` otherwise).
* ``chroot("/")`` from the global root is a no-op.  From inside a jail,
  ``/`` is the jail root, so it cannot be used to escape.
* The jail is stored on the task group (``tg_root``, and ``tg_rootrel``
  when the jail is a subdirectory of a mount such as tmpfs).  Child
  tasks inherit it.  Kernel threads do not.
* After a successful ``chroot()``, ``PWD`` is rewritten so relative
  lookups stay inside the jail:

  * If ``PWD`` is exactly the new root, it becomes ``/``.
  * If ``PWD`` is under the new root, the prefix is stripped.
  * Otherwise ``PWD`` is set to ``/``.  Unlike Linux, NuttX does not
    keep a current directory outside the new tree.

NSH
===

The NSH ``chroot`` command performs the usual Unix dance::

  chdir(newroot);
  chroot(".");
  chdir("/");

With no extra arguments the current NSH session stays jailed (``pwd``
shows ``/``, ``ls /`` lists the jail tree).  An optional command is
executed with ``execvp()`` after the jail is in place; NSH closes
non-stdio, non-``O_CLOEXEC`` descriptors first (see below).

When ``CONFIG_SCHED_USER_IDENTITY`` is enabled, drop extra privilege
after the jail is in place (for example ``setuid()`` to a non-root
user) so a later ``chroot()`` cannot be used to escape.

Open file descriptors
=====================

File descriptors opened before ``chroot()`` are not retroactively
contained.  POSIX allows this; NuttX does not close them.  A jailed
task that inherits a host descriptor can read and write that file
without going through pathname lookup, so the jail does not apply.
This is the most common way ``chroot()`` is misused as a security
tool.  Do not treat it as a sandbox against a process that already
holds host file descriptors.

The NSH ``chroot <newroot> <command>`` form closes every open
descriptor above stderr that is not already marked ``O_CLOEXEC``
before ``execvp()``.  Stdio (fds 0--2) is left intact.  The
no-command form leaves the current NSH session jailed with its
existing descriptors, including any that point outside the tree.

Out of scope
============

Bind-mounts or unionfs to populate ``/dev`` inside a jail, mount/PID/
network namespaces, and ``pivot_root()`` are not provided.
