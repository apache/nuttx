.. _signal-handlers:

===============
Signal Handlers
===============

Signals are used to exchange information between sending and receiving thread.

 
Sending Thread
==============

Posting Signals
---------------

These are the actions that run on the thread that posts the signal.
Signals are initiated in these ways:

* ``signal/sig_kill.c: nxsig_kill()``. The standard function ``kill()`` is a
  simple wrapper around ``nxsig_kill()``. ``nxsig_kill()`` can be used
  to send a signal to any task group. It simply sets up the call
  to ``nxsig_dispatch()``.
* ``signal/sig_queue.c: nxsig_queue()``. The standard function ``sigqueue()``
  is a simple wrapper around ``nxsig_queue()``. ``nxsig_kill()`` can be used
  to send a signal to any task group, passing more information than
  is possible with ``kill()``.
  It again simply sets up the call to ``nxsig_dispatch()``.
* ``signal/sig_notification.c: nxsig_notification()``. This logic can also
  generate signals via a call to ``nxsig_dispatch()``.
  But this is part of the internal, NuttX signal notification system.
  It sends signals to tasks via the work queue.

signal/sig_dispatch.c: ``nxsig_dispatch()``
-------------------------------------------

.. code-block:: c

  int nxsig_dispatch(pid_t pid, FAR siginfo_t *info);

This is the front-end for ``nxsig_tcbdispatch()`` that should be typically
be used to dispatch a signal. If ``HAVE_GROUP_MEMBERS`` is defined,
then this function will follow the group signal delivery algorithms:

This front-end does the following things before calling
``nxsig_tcbdispatch()``:

1. With ``HAVE_GROUP_MEMBERS`` defined:

   1. Get the TCB associated with the ``pid``.
   2. If the TCB was found, get the group from the TCB.
   3. If the PID has already exited, lookup the group that that was
      started by this task.
   4. Use the group to pick the TCB to receive the signal.
   5.  Call ``nxsig_tcbdispatch()`` with the TCB.

2. With ``HAVE_GROUP_MEMBERS`` not defined:

   1. Get the TCB associated with the ``pid``.
   2. Call ``nxsig_tcbdispatch()`` with the TCB

group/group_signal.c: ``group_signal()``
----------------------------------------

.. code-block:: c

  int group_signal(FAR struct task_group_s *group, FAR siginfo_t *siginfo);

Send a signal to the appropriate member(s) of the group.
This is typically called from ``nxsig_dispatch()`` as described above
but may also be called from ``task/task_exithook.c`` to handle
Death-of-Child (``SIGCHLD``) signals.

group/group_signal.c: ``group_signal_handler()``
------------------------------------------------

.. code-block:: c

  static int group_signal_handler(pid_t pid, FAR void *arg)

Callback from ``group_foreachchild()`` that handles one member of the group.

signal/sig_dispatch.c: ``nxsig_tcbdispatch()``
----------------------------------------------

.. code-block:: c

  int nxsig_tcbdispatch(FAR struct tcb_s *stcb, siginfo_t *info)

All signals received the task (whatever the source) go through this function
to be processed. This function is responsible for:

1. Determining if the signal is blocked.
2. Queuing and dispatching signal actions
3. Unblocking tasks that are waiting for signals
4. Queuing pending signals.

This function will deliver the signal to the specific task associated
with the specified TCB.

This function is also called when the OS needs to deliver a signal
to a specific task.
It is normally only called via ``group_signal_handler()`` so that is follows
the rules of signal deliver in multi-threaded tasks.
But it is also called from a few other places:

1. ``pthread/pthread_condtimedwait.c: pthread_condtimedout().``
   Used to wake-up a specific task waiting on a condition.
2. ``task/task_exithook.c: task_sigchild()``
   Used to send the Death-of-Child signal, ``SIGCHLD``, to the parent task.

For unmasked signals that have a signal handler attached,
``nxsig_tcbdispatch()`` will call the architecture-specific interface,
``up_schedule_sigaction()``.

arch/xxx/src/xxx/up_schedsigaction.c: ``up_schedule_sigaction()``
-----------------------------------------------------------------

.. code-block:: c

  void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)

Where sigdeliver is always the OS function ``nxsig_deliver()``.

This function is called by the OS when one or more signal handling actions
have been queued for execution for a specific task.

The architecture specific-code must configure things so that the sigdeliver
callback (i.e., ``nxsig_deliver()``) is executed on the thread specified
by tcb as soon as possible. This amounts to saving the signal handler state
information in the TCB so that when the receiving task next executes,
it will be the signal handler that runs, not the normal, uninterrupted thread.

One of the fixups performed by ``up_schedule_sigaction()`` is to force
the address of the signal handler to point to the trampoline function
``up_sigdeliver()``.

There are other special cases when the signal is generated from interrupt
handler or when the a task signals itself for some reason.
Those variations are not addressed here.


Receiving Thread
================

signal/sig_deliver.c: ``nxsig_deliver()``
-----------------------------------------

.. code-block:: c

  void nxsig_deliver(FAR struct tcb_s *stcb)

This function is called on the thread of execution of the signal receiving task
when that task next runs again. It processes all queued signals then returns.

The mechanism by which a signal is deliver depends on the build configuration;
in PROTECTED and KERNEL build modes, it must go through a trampoline,
``up_signal_dispatch()`` to handle user-space signal actions.
``up_signal_dispatch()`` will drop from kernel mode to user mode,
then call the signal handler.

Otherwise, the signal handler is called directly from ``nxsig_deliver()``.
When the signal handler returns, the action is over and there is only
clean up to be done.

arch/xxx/src/xxx/up_sigdeliver.c: ``up_sigdeliver()``
-----------------------------------------------------

.. code-block:: c

  void up_sigdeliver(void)

This is the a signal handling trampoline.
Logic in ``up_schedule_sigaction()`` forced the signal action to be
delivered to ``up_sigdeliver()``.

``up_sigdeliver()`` will do such things as:

1. Set up the state to return to the normal, uninterrupted thread
   when the signal handler exits.
2. Make sure that the signal handler runs with interrupts enabled.
3. Invoke the signal handler.

When the signal handler returns this function will:

1. Free up resources committed by ``up_schedule_sigaction()``.
2. Restore the interrupt state.
3. And perform a context switch to return to the normal, uninterrupted thread.

When the uninterrupted thread is next suspended, the return will go back to
``nxsig_deliver()`` which will continue delivering signals.
(Hmmm.. shouldn't any other queued signal actions be handled first
before returning to the normal, uninterrupted thread?)
