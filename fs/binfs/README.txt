fs/binfs README
================

  This is the binfs file system that allows "fake" execution of NSH built-
  in applications via the file system.  The binfs fs file system can be
  built into the system by enabling:

    CONFIG_BUILTIN=y
    CONFIG_FS_BINFS=y

  It can then be mounted from the NSH command like like:

   mount -t binfs /bin

Example
=======

  NuttShell (NSH) NuttX-6.31
  nsh> hello
  nsh: hello: command not found

  nsh> mount -t binfs /bin
  nsh> ls /bin
  ls /bin
  /bin:
   hello

  nsh> /bin/hello
  Hello, World!!
