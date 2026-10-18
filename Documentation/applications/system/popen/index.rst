====================================
``popen`` popen()/pclose() Functions
====================================

This library provides ``popen()``, ``pclose()``, ``dpopen()`` and
``dpclose()``.  Without a shell (no ``CONFIG_NSH_LIBRARY``), it starts the
command directly with ``posix_spawnp()``.

libc also has these functions when ``CONFIG_PIPES``,
``CONFIG_LIBC_EXECFUNCS`` and ``CONFIG_SCHED_WAITPID`` are enabled.  The
libc version starts ``sh -c command``.  If there is no shell, it splits
the command at blanks and starts the program directly, without shell
syntax.  When ``CONFIG_SYSTEM_POPEN`` is selected, libc does not build its
copy and this library is used.
