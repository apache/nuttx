======================================
``exitmem`` Memory of an ended process
======================================

This test checks that the memory of a process comes back when the process
ends.  It needs ``fork()`` (``CONFIG_ARCH_HAVE_FORK``: kernel builds).

``exitmem`` forks children one after the other.  Each child takes a part of
the free memory with ``malloc()`` (less when its heap is smaller), writes
every page and ends.  After each ``waitpid()`` the free memory
(``sysconf(_SC_AVPHYS_PAGES)``) must come back within 3 seconds.

With SMP, each child runs on another CPU than the parent, and a thread of
the child runs on a third CPU first.  Both CPUs then run only the idle task,
and must not keep the address environment of the ended child.

Usage::

   exitmem [rounds [percent]]

The defaults are 8 rounds and 40 percent of the free memory.  Example on
``qemu-armv8a:knsh`` with 4 CPUs:

.. code:: console

   nsh> exitmem
   exitmem: 4 CPUs, 8 rounds, 40% of the free memory
   exitmem: round 1 CPU 1: 3600 KiB, free 9052 KiB before, 9032 KiB after
   exitmem: round 2 CPU 2: 3600 KiB, free 9032 KiB before, 9024 KiB after
   ...
   exitmem: round 8 CPU 2: 3600 KiB, free 9016 KiB before, 9016 KiB after
   exitmem: PASS
