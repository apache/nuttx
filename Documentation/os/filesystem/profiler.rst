==============================
VFS Performance Profiler
==============================

The Virtual File System (VFS) Performance Profiler provides a simple, in-kernel 
mechanism to track execution times and invocation counts for core VFS operations 
(read, write, open, close) seamlessly. This is highly suitable for 
CI/CD automated regression testing and performance bottleneck identification.

Configuration
=============

To enable the profiler, select ``CONFIG_FS_PROFILER`` in your Kconfig.
To expose the metrics dynamically via procfs, ensure ``CONFIG_FS_PROCFS`` is enabled, and 
the profiler node is included via ``CONFIG_FS_PROCFS_PROFILER``.

Usage
=====

When enabled, the profiler automatically intercepts calls to the underlying 
inode operations and records the execution elapsed times using ``perf_gettime()``.
Since no blocking mutexes are used during updates (fast ``atomic.h`` operations 
are utilized instead), the overhead is extremely minimal and safely scales on SMP.

To view the current statistics collectively from the NuttShell (NSH), simply 
read the node:

.. code-block:: bash

    nsh> cat /proc/fs/profile
    VFS Performance Profile:
      Reads:          12 (Total time: 4500120 ns)
      Writes:          3 (Total time:   95050 ns)
      Opens:          15 (Total time: 1005000 ns)
      Closes:         15 (Total time:   45000 ns)

The reported times are in the raw ticks/units provided by ``perf_gettime()`` on 
your specific architecture.
