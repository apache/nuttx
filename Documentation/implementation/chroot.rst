.. _chroot:

======
chroot
======

``chroot()`` is a kernel-enforced filesystem jail.  When
``CONFIG_FS_CHROOT`` is enabled, each task group may pin a directory as
its root.  Absolute path lookup starts there, so the group cannot see
files outside that tree.

Limitations: this is **not** a container.  The current implementation
only changes where pathname lookup begins; it does not provide PID,
mount, or network namespaces, and it does not populate the new root
with ``/dev`` or ``/proc``.  See `TODO`_ for what each of these would
require.

Design and implementation
==========================

The Kconfig option and the syscall are the easy part; ``chroot()`` on
NuttX has no MMU-backed process isolation to lean on, so the whole
feature has to be built on top of the single, global pseudo-filesystem
inode tree that every task already shares. This section walks through
why that made the implementation harder than it looks, in the order
the pieces had to be worked out.

Where does the jail live?
--------------------------

The first question is what a "jail" even is in a system with one
shared filesystem tree: it cannot be a separate tree, so it has to be
a *starting point* that path lookups are not allowed to walk above.
That starting point needs to be remembered somewhere per-caller, and
it needs to survive ``fork()``-style child creation the same way an
open file table or a working directory does.

NuttX already keeps exactly that kind of shared, inheritable state on
the task group (``struct task_group_s``), not on the individual task,
because every thread in a task group is supposed to see the same
filesystem view. The jail is stored as a single absolute path::

  struct task_group_s
  {
    ...
  #ifdef CONFIG_FS_CHROOT
    FAR char *tg_root;     /* Absolute jail path, or NULL */
  #endif
  };

A path is used instead of a cached inode so a later unmount/remount
at that location is picked up on the next lookup.  ``tg_root`` is
``NULL`` when the group has not called ``chroot()``.

``group_inherit_chroot()`` (``sched/group/group_create.c``) copies the
string to child task groups (kernel threads are skipped):

.. code-block:: c

  if (rgroup->tg_root == NULL)
    {
      return OK;
    }

  group->tg_root = strdup(rgroup->tg_root);
  if (group->tg_root == NULL)
    {
      return -ENOMEM;
    }

That is what makes a jail apply to a whole subtree of children, not
just the one task that called ``chroot()``.

How lookups stay inside the jail
---------------------------------

Every absolute path goes through ``inode_search_setup()`` and then
the original walk from ``g_root_inode``:

1. Prepend ``tg_root`` to the incoming path (``/tmp/jail`` + ``/foo``
   becomes ``/tmp/jail/foo``).
2. Canonicalize the combined string with ``_inode_canonicalize()``:
   drop empty and ``.`` segments and collapse ``..``.  The jail prefix
   is the floor for that walk, so ``..`` cannot pop above ``tg_root``.
3. ``/../etc`` inside the jail therefore becomes ``/tmp/jail/etc``,
   not host ``/etc``.
4. Continue the original inode-tree walk on that host path.

Without a jail the same canonicalize step still runs, so
``chroot(".")`` under a mount (``$PWD/.``) does not pass a leftover
``.`` to the filesystem as ``relpath``.

There is no separate jailed walk, and no extra ``..`` handling inside
the tree traversal.

Why ``chroot()`` does not touch ``PWD``
-----------------------------------------

Relative lookups go through ``inode_search()``, which prepends
``$PWD`` to the path and then calls the exact same absolute-path
logic described above. That raises an obvious question: what happens
to a task's current directory when its whole notion of "root" just
moved?

An earlier version of this change rewrote ``PWD`` inside ``chroot()``
itself to keep it consistent with the new jail. That turned out to be
both the wrong layer and unnecessary:

* ``chroot()`` is a filesystem primitive; ``PWD`` is environ state.
  POSIX ``chroot()`` does not touch the current directory either --
  the well-known Unix idiom is that the *caller* must ``chdir()``
  immediately after ``chroot()``, precisely so that no stale
  reference to the old tree is left lying around.
* It is not needed for containment. A stale ``PWD`` used in a
  relative lookup after ``chroot()`` is still rewritten by
  ``inode_search_setup()`` (prepend the jail path, canonicalize, clamp
  ``..``) before the tree walk. The lookup can fail or land on a
  path inside the jail that was not intended, but it cannot resolve
  to a node outside the jail.

So ``chroot()`` leaves ``PWD`` alone, and the caller is responsible
for calling ``chdir()`` afterward if a sane current directory inside
the jail is needed -- exactly as the NSH ``chroot`` command already
does with its trailing ``chdir("/")`` (see `NSH`_ below), which sets
``PWD`` correctly via the ordinary ``chdir()`` path, with no special
jail-aware logic required.

Why the privilege gate lives in ``chroot()`` itself
-----------------------------------------------------

The last design question was who is allowed to call ``chroot()`` at
all. When ``CONFIG_SCHED_USER_IDENTITY`` is enabled, the syscall
checks ``tg_euid`` directly and returns ``EPERM`` for anything but
effective UID 0::

  #ifdef CONFIG_SCHED_USER_IDENTITY
    if (group->tg_euid != 0)
      {
        set_errno(EPERM);
        return ERROR;
      }
  #endif

This is intentionally the same *class* of check as the credential DAC
checks described in :ref:`user-identity`, and it inherits the same
caveat: on ``CONFIG_BUILD_FLAT``, kernel and application code share
one address space, so this is a userspace-visible gate rather than a
hardware-enforced boundary -- other code in that address space can
write ``tg_euid`` or ``tg_root`` directly. Protected and kernel builds
close that gap by enforcing the check at the syscall boundary, which
untrusted code cannot bypass. Without ``CONFIG_SCHED_USER_IDENTITY``
at all, every task is effectively root, so ``chroot()`` stays
available to everyone and is a pure path-containment mechanism with no
privilege check gating it.

Configuration
=============

Enable ``CONFIG_FS_CHROOT`` in the filesystem configuration.  The
syscall is then available from ``unistd.h``.

Semantics
=========

* ``chroot(path)`` resolves ``path`` relative to the caller's current
  root (so a nested ``chroot()`` cannot walk back to the host tree).
* ``path`` must name a directory (``ENOTDIR`` otherwise).
* ``chroot("/")`` resolves to the host root and clears the jail
  (``tg_root = NULL``).  From inside a jail, ``/`` is the jail root, so
  it cannot be used to escape.
* The jail is stored on the task group as the absolute path ``tg_root``.
  Child tasks inherit it.  Kernel threads do not.
* ``chroot()`` does not modify ``PWD`` or any other environ state; the
  caller is responsible for calling ``chdir()`` afterward if a
  specific current directory inside the jail is needed.  See
  `Why chroot() does not touch PWD`_.

NSH
===

The NSH ``chroot`` command performs the usual Unix dance::

  chdir(newroot);
  chroot(".");
  chdir("/");

With no extra arguments the current NSH session stays jailed (``pwd``
shows ``/``, ``ls /`` lists the jail tree) because of the trailing
``chdir("/")`` in the sequence above, not because ``chroot()`` itself
touches ``PWD``.  An optional command is executed with ``execvp()``
after the jail is in place; NSH closes non-stdio, non-``O_CLOEXEC``
descriptors first (see below).

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

TODO
====

The following are deliberately out of scope for this initial
implementation, and are listed with what each would require, since
that scoping was itself a large part of the design work:

* **Populating ``/dev``, ``/proc``, etc. inside the jail.**  Nothing
  bind-mounts or otherwise recreates these pseudo-filesystems under
  the new root, so a jailed task cannot open devices or read process
  info unless the jail directory tree already contains them.  Adding
  this needs either a bind-mount primitive (mount an existing inode
  subtree at a second path) or a per-jail selective mount step run at
  ``chroot()`` time; neither existed in the VFS before this change,
  and both are a materially larger change than pathname jailing.
  This is the specific gap raised for using ``chroot()`` to sandbox
  remote logins (telnet/ssh): without a minimal ``/dev``, a jailed
  shell cannot even do much I/O.
* **PID namespaces.**  NuttX has one flat, global task/PID table.
  Isolating it per jail would mean making scheduler and IPC lookups
  (``kill()``, ``/proc``-style listings, signal delivery) aware of a
  namespace boundary, which touches the scheduler core, not just the
  VFS. This implementation does not attempt that.
* **Mount namespaces.**  The mount table (``g_root_inode`` and its
  mounted filesystems) is process-global. A jailed task group can be
  confined to a subtree of the existing mount table, but it cannot
  have a private view where mounts made outside the jail are hidden,
  or where the jailed task can mount/unmount without affecting the
  rest of the system.  That requires per-task-group mount tables.
* **Network namespaces.**  Sockets and network interfaces are global
  to the OS instance; nothing in this change touches the network
  stack.
* **``pivot_root()``.**  Swapping the process root while keeping the
  old root reachable is not implemented; ``chroot()`` only changes
  where lookups begin.

None of these are ruled out architecturally -- they are simply not
part of this change, which is scoped to pathname-lookup containment.
