========================================
``dumpstack`` Task Call Stack Backtrace
========================================

Description
-----------

The ``dumpstack`` application is a debugging utility that displays the call stack
backtrace information for one or more tasks (threads) in the NuttX system. It helps
developers understand the execution path and function call hierarchy of tasks, which
is particularly useful for debugging crashes, deadlocks, or abnormal behavior.

The application leverages the ``sched_dumpstack()`` function from the NuttX kernel
to retrieve and display backtrace information. Depending on the configuration, the
backtrace can display either raw memory addresses or symbolic function names.

Features
--------

* Dump backtrace for a single task by PID
* Dump backtrace for a range of tasks by PID range
* Support for symbolic backtrace display (requires ``CONFIG_ALLSYMS``)

Usage
-----

Basic Syntax
^^^^^^^^^^^^

.. code-block:: bash

   dumpstack [start_pid] [end_pid]

**No arguments** - Dump backtrace of the current task:

.. code-block:: bash

   nsh> dumpstack

**Single PID** - Dump backtrace of a specific task:

.. code-block:: bash

   nsh> dumpstack 5

**PID range** - Dump backtrace of all tasks from start_pid to end_pid (inclusive):

.. code-block:: bash

   nsh> dumpstack 3 10

Examples
^^^^^^^^

1. **Dump backtrace for tasks PID 4 to 6 (without CONFIG_ALLSYMS):**

   .. code-block:: bash

      nsh> dumpstack 4 6
      sched_dumpstack: backtrace| 4: 0x0000000010024fe8 0x000000001000cccc 0x000000001002504c 0x000000001002759c 0x000000001002a870
      sched_dumpstack: backtrace| 5: 0x0000000010024fe8 0x000000001000cccc 0x000000001002504c 0x000000001002759c 0x000000001002a870
      sched_dumpstack: backtrace| 6: 0x000000001002bbb4 0x000000001002bd70 0x00000000100d3890 0x00000000100d3908 0x000000001005193c 0x000000001004fc74 0x0000000010051514 0x000000001005160c
      sched_dumpstack: backtrace| 6: 0x0000000010051870 0x0000000010047fa8 0x000000001002cd50 0x000000001003d5ec 0x000000001002a888

2. **Dump backtrace for task PID 4 (with CONFIG_ALLSYMS enabled):**

   .. code-block:: bash

      nsh> dumpstack 4
      sched_dumpstack: backtrace:
      sched_dumpstack: [ 4] [<0x10025174>] nxsem_wait_slow+0x158/0x1ac
      sched_dumpstack: [ 4] [<0x1000cccc>] nxsem_wait+0x94/0xb0
      sched_dumpstack: [ 4] [<0x100251d8>] nxsem_wait_uninterruptible+0x10/0x2c
      sched_dumpstack: [ 4] [<0x10027728>] work_thread+0x1b4/0x238
      sched_dumpstack: [ 4] [<0x1002a9fc>] nxtask_start+0x7c/0xa4

Configuration
-------------

**CONFIG_SYSTEM_DUMPSTACK**

Dependencies
^^^^^^^^^^^^

The dumpstack application requires the following kernel configurations:

* **CONFIG_SCHED_BACKTRACE** - Must be enabled to provide the ``sched_backtrace()`` function
* **CONFIG_ALLSYMS** - Optional, enables symbolic function names in backtrace output

Limitations
-----------

* Maximum backtrace depth is 16 frames per iteration
* Some architectures may have limited or no backtrace support (known issue: sim environment cannot view stack of other threads)
