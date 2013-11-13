fs/procfs README
================

  This is a tiny procfs file system that allows read-only access to a few
  attributes of a task or thread.  This tiny procfs fs file system can be
  built into the system by enabling:

    CONFIG_FS_PROCFS=y

  It can then be mounted from the NSH command like like:

    nsh> mount -t procfs /proc
