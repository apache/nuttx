====================================
``critmon`` critical section monitor
====================================

The ``critmon`` command displays critical section and pre-emption timing
information for all tasks and threads in the system. It reads from the
procfs ``critmon`` and ``status`` files for each process/thread.

Three commands are provided:

- ``critmon``: One-shot dump of critical section timing data.
- ``critmon_start``: Start the background monitoring daemon.
- ``critmon_stop``: Stop the background monitoring daemon.

The daemon periodically samples critical section usage at a configurable
interval (default: 2 seconds) and prints the timing information.

Configuration
=============

- ``CONFIG_SCHED_CRITMONITOR``: Enable the critical section monitor
  in the kernel (required dependency).
- ``CONFIG_FS_PROCFS``: Enable procfs filesystem support.
- ``CONFIG_SYSTEM_CRITMONITOR``: Enable the critmon system tool.

The following additional options are available:

- ``CONFIG_SYSTEM_CRITMONITOR_DAEMON_STACKSIZE`` - Stack size for the
  daemon task (default: 2048).
- ``CONFIG_SYSTEM_CRITMONITOR_DAEMON_PRIORITY`` - Priority for the
  daemon task (default: 50).
- ``CONFIG_SYSTEM_CRITMONITOR_INTERVAL`` - Sampling interval in
  seconds (default: 2).
- ``CONFIG_SYSTEM_CRITMONITOR_MOUNTPOINT`` - procfs mount point
  (default: ``/proc``).

Usage
=====

.. code-block:: text

   critmon
   critmon_start
   critmon_stop

Output Format
=============

The output shows a table with the following columns:

- **PRE-EMPTION CALLER**: Maximum pre-emption-disabled time and its
  associated caller.
- **CSECTION CALLER**: Maximum critical-section time and its associated
  caller.
- **RUN**: Maximum uninterrupted execution time for the thread.
- **TIME**: Total accumulated execution time for the thread.
- **PID**: Process/thread ID.
- **DESCRIPTION**: Task name (if ``CONFIG_TASK_NAME_SIZE > 0``).

Examples
========

One-shot dump of critical section timing:

.. code-block:: text

   nsh> critmon
   PRE-EMPTION CALLER            CSECTION CALLER               RUN              TIME             PID   DESCRIPTION
   nxtask_exit                   nxtask_init                   0.000010000      0.000010000      0     Idle_Task
   nxtask_init                   up_irq_save                   0.000005000      0.000015000      1     hpwork 0x10001000

Start the monitoring daemon (prints every 2 seconds):

.. code-block:: text

   nsh> critmon_start
   Csection Monitor: Started: 5

Stop the monitoring daemon:

.. code-block:: text

   nsh> critmon_stop
   Csection Monitor: Stopping: 5
   Csection Monitor: Stopped: 5

See Also
========

- :doc:`/implementation/critical_sections`
- :doc:`../stackmonitor/index`
- :doc:`../gprof/index`
