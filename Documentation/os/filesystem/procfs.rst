======
PROCFS
======

This is a tiny procfs file system that allows read-only access to a few
attributes of a task or thread.  This tiny procfs fs file system can be
built into the system by enabling::

    CONFIG_FS_PROCFS=y

It can then be mounted from the NSH command like like::

    nsh> mount -t procfs /proc

Example::

  NuttShell (NSH) NuttX-6.31
  nsh> mount -t procfs /proc

  nsh> ls /proc
  /proc:
   0/
   1/

  nsh> ls /proc/1
  /proc/1:
   status
   cmdline

  nsh> cat /proc/1/status
  Name:       init
  Type:       Task
  State:      Running
  Priority:   100
  Scheduler:  SCHED_FIFO
  SigMask:    00000000

  nsh> cat /proc/1/cmdline
  init

  nsh> sleep 100 &
  sleep [2:100]
  nsh> ls /proc
  ls /proc
  /proc:
   0/
   1/
   2/

  nsh> cat /proc/2/cmdline
  <pthread> 0x527420

Process user and group
======================

With ``CONFIG_SCHED_USER_IDENTITY=y``, ``/proc/<pid>/status`` has two more
lines.  ``Uid:`` gives the real and the effective user ID of the task group
of ``<pid>``.  ``Gid:`` gives the real and the effective group ID::

  Uid:        1000 1000
  Gid:        1000 1000

``stat()`` of ``/proc/<pid>`` and of the files in it gives the effective
user and group ID of the task group in ``st_uid`` and ``st_gid``.
