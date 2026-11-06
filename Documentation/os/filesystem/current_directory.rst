.. _current-directory:

=====================
The Current Directory
=====================

Each task group (a process) has a current working directory: the start of
each relative path.  The current directory is kernel state of the task
group (``tg_cwd`` in ``struct task_group_s``).  The environment variable
``PWD`` is not part of it.

Interfaces
==========

``chdir(path)``
  Makes ``path`` the current directory.  The path must name a directory
  with search permission (``ENOENT``, ``ENOTDIR``, ``EACCES``).  The kernel
  keeps the canonical absolute path: no ``.``, ``..`` or symbolic links.
  ``chdir()`` does not change ``PWD`` or ``OLDPWD``.

``fchdir(fd)``
  The same with an open directory (``EBADF``, ``ENOTDIR``).  The kernel
  finds the directory by the path that it had when it was opened
  (``F_GETPATH``).  ``fchdir()`` needs ``CONFIG_FS_IOCTL_FILEPATH``;
  without it ``fchdir()`` fails with ``ENOSYS``.

``getcwd(buf, size)``
  Copies the absolute path of the current directory.  ``getcwd(NULL, size)``
  allocates the buffer (``size`` bytes, or ``PATH_MAX`` when ``size`` is
  zero).  It fails with ``ERANGE`` if the path does not fit, and with
  ``ENOENT`` if the directory was removed or is outside the ``chroot()``
  root of the caller.

``chdir()``, ``fchdir()`` and ``getcwd()`` are system calls in protected
and kernel builds.  ``getcwd(NULL, size)`` allocates the buffer in the
heap of the caller.  In a flat build these are direct calls.

The relative paths of all other calls, and the ``*at()`` functions with
``AT_FDCWD``, start at the current directory of the task group.

Inheritance
===========

A new task group gets the current directory of the task group that
creates it: ``fork()``, ``vfork()``, ``posix_spawn()``, ``task_create()``
and the ``exec`` functions.  The threads of a task group share the
directory: a ``chdir()`` in one thread changes it for all.  The kernel
threads share one directory of their own.

The Shell and ``PWD``
=====================

``PWD`` and ``OLDPWD`` are information for the shell and the programs that
it starts.  A shell sets them after its own ``chdir()``: the NSH ``cd``
command sets ``PWD`` to the new directory and ``OLDPWD`` to the old
``PWD`` (``cd -`` uses it).  A program that sets or unsets ``PWD`` does
not move its current directory, and ``env -i`` starts a program in the
same directory.

Changes to the Directory
========================

The kernel keeps a path, so it updates the current directories when the
VFS changes a directory:

* ``rename()`` of the current directory or of a directory above it: the
  current directory gets the new name.
* ``rmdir()`` of the current directory: ``getcwd()`` fails with ``ENOENT``
  and each relative path gives ``ENOENT``.  A new directory of the same
  name is another directory.  Use an absolute path, for example
  ``chdir("/")``, to leave this state.
* ``umount()`` of a file system that holds the current directory of a task
  group fails with ``EBUSY``.  ``umount2()`` with ``MNT_FORCE`` or
  ``MNT_DETACH`` unmounts it, and then the current directory is removed,
  as after ``rmdir()``.  A current directory in another file system that
  is mounted below the mount point and stays mounted does not change.

There is no list of current directories: ``rename()``, ``rmdir()`` and
``umount()`` walk the threads of all task groups (``nxsched_foreach()``)
under one kernel mutex.  The walk does not allocate memory, so
``rename()`` gives the new name to one task group after the other.

Limits:

* A change that does not go through the VFS of this system is not seen,
  for example another host that renames a directory of a shared file
  system.
* A ``mount()`` on the current directory, or above it, is used by the next
  relative path.
* ``fchdir()`` and the ``*at()`` functions with a directory descriptor use
  the path that the directory had when it was opened.
* A ``chdir()`` at the same time as the ``rename()``, ``rmdir()`` or
  ``umount()`` of its directory can keep the old name.
* When the kernel has no memory for the update (no temporary buffer free
  and no ``CONFIG_LIBC_TEMPBUFFER_MALLOC``, or a full kernel heap), the
  current directory keeps its old name after ``rename()``, and
  ``getcwd()`` still gives the name after ``rmdir()``.
* Only a directory set with ``chdir()`` or ``fchdir()`` makes ``umount()``
  busy: the default, ``CONFIG_LIBC_HOMEDIR``, does not.
* After ``rmdir()`` of the current directory, ``chdir("..")`` fails with
  ``ENOENT`` too (other systems go to the parent).

chroot()
========

The current directory stays where it is when the task group calls
``chroot()``, as on other systems.  A current directory outside the new
root is not the start of a lookup: relative paths give ``ENOENT`` and
``getcwd()`` fails with ``ENOENT`` until the program calls ``chdir()``.
So relative names never leave the root.  A task group that never called
``chdir()`` has ``CONFIG_LIBC_HOMEDIR`` inside its root.  See
:ref:`chroot`.

Memory
======

A task group that never calls ``chdir()`` has only the pointer
``tg_cwd``.  ``chdir()`` allocates the path as a string in the kernel
heap.  Each task group that it creates gets its own copy.  An empty string
is a removed directory.
