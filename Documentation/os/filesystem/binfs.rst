=====
BINFS
=====

This is the binfs file system that allows "fake" execution of NSH built-
in applications via the file system.  The binfs fs file system can be
built into the system by enabling::

    CONFIG_BUILTIN=y
    CONFIG_FS_BINFS=y

It can then be mounted from the NSH command like like::

   mount -t binfs /bin

Example::

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

By default, ``exec()`` and ``posix_spawn()`` look up only the last part of a
path in the builtin applications: ``/nonexistent/hello`` also starts ``hello``.
With ``CONFIG_BINFMT_BUILTIN_STRICT_PATH``, a name with a ``/`` starts a builtin
application only if it names a file in a mounted binfs.  A bare name still
starts it.  The option breaks code that uses a path with no file there, such as
``/bin/sh`` with no binfs at ``/bin``: see its help text.

