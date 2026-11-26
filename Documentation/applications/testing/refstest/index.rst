====================================================
``refstest`` File and inode reference counting tests
====================================================

The ``testing/fs/refstest`` application checks NuttX file and inode reference
counts using cmocka. Run it as ``cmocka_fs_refstest``. A zero exit status means
that all enabled tests and their setup/teardown functions passed.

Configuration
=============

Enable ``CONFIG_TESTING_CMOCKA`` and ``CONFIG_TESTING_FS_REFSTEST``. The suite
requires a flat build, pthreads, full descriptor inheritance,
``CONFIG_SCHED_WAITPID`` and ``CONFIG_SCHED_CHILD_STATUS``. cmocka also requires
``CONFIG_LIBC_REGEX`` and its ``CONFIG_ALLOW_MIT_COMPONENTS`` dependency.

Set ``CONFIG_TESTING_FS_REFSTEST_MOUNTDIR`` to an existing writable directory.
The filesystem must support unlinking open files. tmpfs, hostfs and writable
pseudofiles are suitable. Allow at least eight unused file descriptors.

Each test creates its own temporary directory and removes its files when it
finishes, including after assertion failures. Existing files in the parent
directory are retained. The task priority and stack size are controlled by
``CONFIG_TESTING_FS_REFSTEST_PRIORITY`` and
``CONFIG_TESTING_FS_REFSTEST_STACKSIZE`` (4096 bytes by default).

For example, with tmpfs and the NSH mount command enabled, set the test
directory to ``/refsdata`` and run::

  nsh> mount -t tmpfs /refsdata
  nsh> cmocka_fs_refstest
  nsh> echo $?
  0

Use an unused mount path in this example, or use an already mounted writable
filesystem without issuing the mount command.

Coverage
========

The suite checks the following operations:

* Separate ``open`` calls versus a file object shared by ``dup``.
* ``dup2`` onto itself, an occupied descriptor and a released descriptor slot.
* Continued access through open descriptors after ``unlink``.
* References held by epoll ADD and released by DEL or closing epoll.
* ``dup3`` replacement, CLOEXEC flags and rejection of identical descriptors.
* ``task_spawn`` inheritance and CLOEXEC exclusion. Semaphores synchronize the
  parent and child before the parent inspects the counts.
* RAMMAP references acquired by ``mmap`` and released by ``munmap``.
* Invalid descriptor errors without changing live reference counts.
* Four pthreads, each completing 100 ``dup``/``close`` pairs.

The RAMMAP case is registered only with ``CONFIG_FS_RAMMAP=y``. It creates a
temporary character device under ``/dev`` with neither mmap nor XIP support.
This forces the RAMMAP fallback rather than a filesystem-specific direct
mapping. Mapping failure fails the test; it is not treated as a skipped case.
The other eight cases run when RAMMAP is disabled.

The suite reads internal NuttX counters, so it is limited to flat builds.
Descriptor exhaustion is reported as a test failure. The concurrent test does
not establish race freedom on every SMP target.
