===========================
``gprof`` profiling command
===========================

The ``gprof`` command controls the NuttX function-call profiling subsystem.
It can start and stop sampling at runtime and dump the collected profiling
data to a file in ``gmon.out`` format.  The resulting file can be processed
with the host ``gprof`` tool (or compatible analysers) to produce a flat
profile and a call-graph.

Profiling relies on compiler instrumentation (``-finstrument-functions``
or equivalent).  Each instrumented function call records the caller and
callee addresses; the dump subcommand writes these records together with a
histogram of sampled program-counter values.

Configuration
=============

Enable the application with ``CONFIG_SYSTEM_GPROF`` (tristate; can be
built-in or an NSH command).

``CONFIG_SYSTEM_GPROF`` depends on profiling being enabled.  Set
``CONFIG_PROFILE_NONE`` to ``n`` (or enable ``CONFIG_SIM_GPROF`` for the
simulator).

Related profiling options:

- ``CONFIG_PROFILE`` -- enable the NuttX profiling framework
- ``CONFIG_PROFILE_DUMP_ON_EXIT`` -- automatically dump profiling data when
  the process exits

Task tuning:

- ``CONFIG_SYSTEM_GPROF_PRIORITY`` -- task priority (default ``100``)
- ``CONFIG_SYSTEM_GPROF_STACKSIZE`` -- stack size (default
  ``DEFAULT_TASK_STACKSIZE``)

Usage
=====

.. code-block:: console

   gprof start
   gprof stop
   gprof dump [output]
   gprof help

Subcommands
===========

start
-----

Begin profiling.  Calls ``monstartup()`` with the text-section boundaries
(``_stext`` to ``_etext``) and then enables sampling via ``moncontrol(1)``.

stop
----

Disable sampling by calling ``moncontrol(0)``.  Profiling data remains in
memory and can be dumped later.

dump [output]
-------------

Write collected profiling data to a file and release the associated
resources.  Calls ``_mcleanup()`` internally.

By default the output file is ``gmon.out`` in the current working
directory.  When *output* is specified it is used as the file-name prefix
via the ``GMON_OUT_PREFIX`` environment variable; the actual file will be
named ``<output>.0`` (the suffix is appended by the gmon writer).

.. note::

   The ``dump`` subcommand requires ``CONFIG_DISABLE_ENVIRON`` to be
   ``n`` so that ``setenv()`` is available.  On configurations that
   disable the environment, ``dump`` prints an error message and does
   not write a file.

help
----

Print a short usage summary and exit.

Examples
========

Profile a section of code interactively:

.. code-block:: console

   nsh> gprof start
   nsh> <run the workload to be profiled>
   nsh> gprof stop
   nsh> gprof dump

Dump to a custom file name:

.. code-block:: console

   nsh> gprof dump /tmp/my_profile

The resulting ``/tmp/my_profile.0`` can be copied to the host and analysed:

.. code-block:: console

   $ arm-none-eabi-objcopy --update-section .gmon/data=my_profile.0 nuttx
   $ arm-none-eabi-gprof nuttx

Automatic dump on exit
======================

When ``CONFIG_PROFILE_DUMP_ON_EXIT`` is enabled, profiling data is written
automatically when the profiled process exits.  In that case there is no
need to call ``gprof dump`` explicitly.

Notes
=====

- Profiling data structures are allocated by ``monstartup()``.  Starting
  profiling a second time without dumping first will silently discard the
  previous data.

- The ``gprof`` command itself must also be built with profiling
  instrumentation if you want to profile it; normally it is only used as a
  controller for profiling other code.
