====================
popen() and dpopen()
====================

libc has ``popen()``, ``pclose()``, ``dpopen()`` and ``dpclose()`` when
``CONFIG_PIPES``, ``CONFIG_LIBC_EXECFUNCS`` and ``CONFIG_SCHED_WAITPID`` are
enabled.

``popen()`` starts ``sh -c command``.  With ``CONFIG_LIBC_ENVPATH``, ``PATH``
gives the location of ``sh``; else it is ``/bin/sh``.  If there is no shell,
the command is split at blanks and the program is started directly, without
shell syntax.  The command starts with the ``posix_spawn()`` defaults for
priority and stack size.  ``pclose()`` returns the termination status of the
shell, as from ``waitpid()``.

``dpopen()`` and ``dpclose()`` are NuttX extensions in ``unistd.h``.  They
use a file descriptor instead of a stream.  ``dpopen()`` with ``O_RDWR``
connects both the standard input and output of the command through a socket
pair (needs ``CONFIG_NET_LOCAL`` and ``CONFIG_NET_LOCAL_STREAM``).
