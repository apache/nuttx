==============
Events
==============

Events groups are synchronization primitives that allow tasks to wait
for multiple conditions to be met before proceeding. They are particularly
useful in scenarios where a task needs to wait for several events to occur
simultaneously.
This concept can be particularly powerful in real-time operating systems (RTOS).

Overview
=========================

An event group consists of a set of binary flags, each representing a
specific event. Tasks can set, clear, and wait on these flags. When a
task waits on an event group, it can specify which flags it is interested
in and whether it wants to wait for all specified flags to be set or just
any one of them.

Configuration Options
=====================

``CONFIG_SCHED_EVENTS``
	 This option enables event objects. Threads may wait on event
	 objects for specific events, but both threads and ISRs may deliver
	 events to event objects.

Common Events Interfaces
================================

Events Types
--------------------

-  ``nxevent_t``. Defines one event group entry.
-  ``nxevent_mask_t``. Defines one events mask value.

Notifier Chain Interfaces
-------------------------

.. c:function:: int nxevent_init(FAR nxevent_t *event, nxevent_mask_t events)

  Initializes an event object, Set of default events to post
  to event.

  :param event: Address of the event object
  :param events: Set of events to post to event

.. c:function:: int nxevent_destroy(FAR nxevent_t *event)

  This function is used to destroy the event.

  :param event: Address of the event object

.. c:function:: int nxevent_reset(FAR nxevent_t *event, nxevent_mask_t events)

  Reset events mask to a specific value.

  :param event: Address of the event object
  :param events: Set of events to post to event

.. c:function:: int nxevent_post(FAR nxevent_t *event, nxevent_mask_t events, nxevent_flags_t eflags)

  Post one or more events to an event object.

  This routine posts one or more events to an event object. All tasks
  waiting on the event object event whose waiting conditions become
  met by this posting immediately unpend.

  Posting differs from setting in that posted events are merged together
  with the current set of events tracked by the event object.

  :param event: Address of the event object
  :param events: Set of events to post to event
                 Set events to 0 will be considered as any,
                 waking up the waiting thread immediately.
  :param eflags: Events flags

.. c:function:: nxevent_mask_t nxevent_wait(FAR nxevent_t *event, nxevent_mask_t events, nxevent_flags_t eflags)

  Wait for all of the specified events.

  This routine waits on event object event until all of the specified
  events have been delivered to the event object. A thread may wait on
  up to 32 distinctly numbered events that are expressed as bits in a
  single 32-bit word.

  :param event: Address of the event object
  :param events: Set of events to wait, 0 will indicate wait from any events
  :param eflags: Events flags

.. c:function:: nxevent_mask_t nxevent_tickwait(FAR nxevent_t *event, nxevent_mask_t events, nxevent_flags_t eflags, uint32_t delay)

  Wait for all of the specified events for the specified tick time.

  This routine waits on event object event until all of the specified
  events have been delivered to the event object, or the maximum wait time
  timeout has expired. A thread may wait on up to 32 distinctly numbered
  events that are expressed as bits in a single 32-bit word.

  :param event: Address of the event object.
  :param events: Set of events to wait, 0 will indicate wait from any events
  :param eflags: Events flags
  :param delay: Ticks to wait from the start time until the event is posted,
                If ticks is zero, then this function is equivalent to nxevent_trywait().

.. c:function:: nxevent_mask_t nxevent_trywait(FAR nxevent_t *event, nxevent_mask_t events, nxevent_flags_t eflags)

  Try wait for all of the specified events.

  This routine try to waits on event object event if any of the specified
  events have been delivered to the event object. A thread may wait on
  up to 32 distinctly numbered events that are expressed as bits in a
  single 32-bit word.

  :param event: Address of the event object
  :param events: Set of events to wait, 0 will indicate wait from any events
  :param eflags: Events flags
