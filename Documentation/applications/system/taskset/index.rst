===========================
``taskset`` Taskset Command
===========================

Overview
========

The ``taskset`` command displays or changes the CPU affinity mask for a
running task, or starts a command after applying an affinity mask to the
``taskset`` process.  It is intended for SMP configurations where tasks can
be constrained to a subset of CPUs.

The mask argument is a decimal CPU affinity bit mask.  Bit 0 selects CPU 0,
bit 1 selects CPU 1, and so on.  The command accepts masks greater than zero
and less than ``1 << CONFIG_SMP_NCPUS``.

Configuration
=============

Enable the application with ``CONFIG_SYSTEM_TASKSET``.  This option depends
on ``CONFIG_SMP`` and ``CONFIG_SYSTEM_SYSTEM``.

The program name, task priority, and stack size are controlled by:

- ``CONFIG_SYSTEM_TASKSET_PROGNAME``
- ``CONFIG_SYSTEM_TASKSET_PRIORITY``
- ``CONFIG_SYSTEM_TASKSET_STACKSIZE``

Usage
=====

.. code-block:: console

   taskset <mask> <command> [args ...]
   taskset -p [mask] <pid>
   taskset -h

Forms
=====

``taskset <mask> <command> [args ...]``
  Applies ``mask`` to the ``taskset`` process and then runs ``command`` with
  any remaining arguments through ``system()``.

``taskset -p <pid>``
  Prints the current CPU affinity mask for ``pid``.

``taskset -p <mask> <pid>``
  Sets the CPU affinity mask for ``pid`` and then prints the task's current
  mask.

``taskset -h``
  Prints command usage.

Examples
========

Show the current affinity mask for task ID 4:

.. code-block:: console

   nsh> taskset -p 4
   pid 4's current affinity mask: 0x2

Set task ID 4 to affinity mask 3 and print the resulting mask:

.. code-block:: console

   nsh> taskset -p 3 4
   pid 4's current affinity mask: 0x3

Start ``busyloop`` after applying affinity mask 1 to the command process:

.. code-block:: console

   nsh> taskset 1 busyloop &

Notes
=====

The command reports scheduler affinity errors on standard error.  The exact
set of valid mask values depends on ``CONFIG_SMP_NCPUS`` for the target
configuration.
