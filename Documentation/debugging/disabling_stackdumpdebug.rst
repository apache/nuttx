=========================================
Disabling the Stack Dump During Debugging
=========================================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Disabling+the+Stack+Dump+During+Debugging

The stack dump routine can clutter the output of GDB during debugging. 
To disable it, set this configuration option in the defconfig file of
the board configuration:

.. code-block:: c

    CONFIG_ARCH_STACKDUMP=n

Reducing the Crash Dump Verbosity
=================================

In addition to ``CONFIG_ARCH_STACKDUMP``, which controls whether the
architecture-specific stack dump is produced at all, two finer-grained
options are available to tune how verbose the assertion / crash output
from ``sched/misc/assert.c`` is. Disabling them is useful to reduce
flash usage on small targets and to keep the crash log short.

Both options default to ``y`` unless ``CONFIG_DEFAULT_SMALL`` is
selected, in which case they default to ``n``.

CONFIG_SCHED_DUMP_TASKS
-----------------------

Controls whether the per-task information table (task list with PID,
priority, scheduling policy, stack usage, state, etc.) is printed as
part of the crash dump.

.. code-block:: c

    CONFIG_SCHED_DUMP_TASKS=n   /* Omit the task table from the crash dump */

CONFIG_SCHED_DUMP_STACK
-----------------------

Controls whether the contents of each stack (IRQ / kernel / user) are
dumped as hex words. When disabled, only the base address and size of
each stack are printed; the stack memory itself is not dumped.

.. code-block:: c

    CONFIG_SCHED_DUMP_STACK=n   /* Omit the hex stack contents from the dump */
