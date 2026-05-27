==============================
``stackmonitor`` Stack Monitor
==============================

Overview
========

The ``stackmonitor`` command starts a background daemon that periodically
monitors stack usage for all tasks and threads in the system. It reads the
procfs filesystem to collect stack size and usage information, helping
developers identify potential stack overflow issues and optimize memory
allocation.

The daemon runs as a separate task and outputs a table showing the PID,
stack size, stack used, and thread name (if available) for each task.
The monitoring continues until explicitly stopped with the
``stackmonitor_stop`` command.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_STACKMONITOR``. This option depends
on:

- ``CONFIG_FS_PROCFS`` - procfs filesystem support
- ``CONFIG_STACK_COLORATION`` - stack coloration feature for usage tracking

The following configuration options are available:

``CONFIG_SYSTEM_STACKMONITOR_STACKSIZE``
  Stack size for the monitor daemon task. Default: 2048 bytes.

``CONFIG_SYSTEM_STACKMONITOR_PRIORITY``
  Priority for the monitor daemon task. Default: 50.

``CONFIG_SYSTEM_STACKMONITOR_INTERVAL``
  Monitoring interval in seconds. Default: 2 seconds.

``CONFIG_SYSTEM_STACKMONITOR_MOUNTPOINT``
  procfs mount point path. Default: ``/proc``.

Usage
=====

.. code-block:: console

   stackmonitor

Start the stack monitoring daemon. If the daemon is already running, display
its current status.

.. code-block:: console

   stackmonitor_stop

Stop the running stack monitoring daemon.

Options
=======

The ``stackmonitor`` and ``stackmonitor_stop`` commands accept no options.

Examples
========

Start the stack monitor and observe the output:

.. code-block:: console

   nsh> stackmonitor
   Stack Monitor: Started: 3

The daemon will periodically output stack usage information:

.. code-block:: console

   PID   SIZE   USED   THREAD NAME
   0     2048   1024   Idle_Task
   1     2048   512    hpwork
   2     2048   768    nsh_main
   3     2048   256    Stack Monitor

Check the daemon status:

.. code-block:: console

   nsh> stackmonitor
   Stack Monitor: Running: 3

Stop the daemon:

.. code-block:: console

   nsh> stackmonitor_stop
   Stack Monitor: Stopping: 3
   Stack Monitor: Stopped: 3

Notes
=====

- The daemon requires the procfs filesystem to be mounted at the configured
  mount point (default ``/proc``).
- Stack usage tracking depends on the ``CONFIG_STACK_COLORATION`` feature,
  which fills unused stack space with a known pattern to detect usage.
- The daemon will automatically stop after 100 consecutive errors reading
  the procfs directory.
- The monitoring interval can be adjusted at compile time using
  ``CONFIG_SYSTEM_STACKMONITOR_INTERVAL``.
- The output format includes the thread name only when
  ``CONFIG_TASK_NAME_SIZE`` is greater than 0.
