==================================================
``smf`` State Machine Framework HSM PSiCC2 Example
==================================================

This example implements an event-driven hierarchical state machine using the
State Machine Framework (SMF). It reproduces the statechart shown in Figure 2.11
of Practical UML Statecharts in C/C++, 2nd Edition, by Miro Samek (PSiCC2).
The ebook is available at https://www.state-machine.com/psicc2.

For each state, the entry, run, and exit actions are logged to the console, as
well as logging when a state handles an event or explicitly ignores it and
passes it up to the parent state.

It should be possible to build and run this demo on most boards or emulators
that support NSH, SMF, and message queues.

Configuration
=============

- ``CONFIG_EXAMPLES_SMF`` – Enables the SMF PSiCC2 demo.
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the demo as an NSH built-in application.
- ``CONFIG_SYSTEM_SMF`` – Enable the State Machine Framework support.
- ``CONFIG_SYSTEM_SMF_ANCESTOR_SUPPORT`` – Enable ancestor/parent state support.
- ``CONFIG_SYSTEM_SMF_INITIAL_TRANSITION`` – Enable initial transition support.
- ``CONFIG_DISABLE_MQUEUE=n`` – Message queue support must be available.
- ``CONFIG_EXAMPLES_SMF_PROGNAME`` – Program name, default ``hsm_psicc2``.
- ``CONFIG_EXAMPLES_SMF_PRIORITY`` – Priority of the SMF task, default ``100``.
- ``CONFIG_EXAMPLES_SMF_STACKSIZE`` – Stack size of the SMF task, default
  ``2048``.
- ``CONFIG_EXAMPLES_SMF_QUEUE_SIZE`` – Size of the message queue, default ``10``.
- ``CONFIG_EXAMPLES_SMF_MQ_NAME`` – Name of the message queue, default
  ``/hsm_psicc2_mq``.

Usage
=====

The demo registers the ``hsm_psicc2`` NSH command (configurable via
``CONFIG_EXAMPLES_SMF_PROGNAME``):

.. code-block:: bash

   hsm_psicc2 start
   hsm_psicc2 event <A..I>
   hsm_psicc2 terminate

- ``start`` spawns the state machine thread and initializes the SMF context.
- ``event <A..I>`` sends events A through I to the state machine (PSiCC2
  Figure 2.11).
- ``terminate`` stops the state machine thread; there is no way to restart it
  and further events are not processed.

Comparison to PSiCC2 Output
===========================

Not all transitions modeled in UML may be supported by the State Machine
Framework. Unsupported transitions may lead to results different from the
example run in PSiCC2 Section 2.3.15. The differences are not listed here since
it is hoped SMF will support these transitions in the future and the list would
become outdated.
