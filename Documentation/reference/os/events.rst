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

.. c:function:: nxevent_mask_t nxevent_clear(FAR nxevent_t *event, nxevent_mask_t mask)

  This function is used to clear specific bits from the event mask of the given event object.

  :param event: Address of the event object
  :param mask: Bit mask specifying which event flags should be cleared

.. c:function:: nxevent_mask_t nxevent_tickwait_wait(FAR nxevent_t *event, FAR nxevent_wait_t *wait, nxevent_mask_t events, nxevent_flags_t eflags, uint32_t delay);

  Wait for all of the specified events for the specified tick time.

  This routine is functionally identical to nxevent_tickwait, except that the wait object is explicitly provided by the user.
  In nxevent_tickwait, the wait object is allocated as a temporary variable on the event-wait thread's stack, and the user has no visibility or control over it.
  Because this object is accessed by both the event-wait thread and the event-post thread, such an arrangement may lead to safety concerns.
  To address this, the new function allows the user to supply the wait object directly, providing greater control and improving safety.

  :param event: Address of the event object
  :param wait:  Address of the event wait object
  :param events: Set of events to wait, 0 will indicate wait from any events
  :param eflags: Events flags
  :param delay: Ticks to wait from the start time until the event is posted,
                If ticks is zero, then this function is equivalent to nxevent_trywait().

.. c:function:: nxevent_mask_t nxevent_getmask(FAR nxevent_t *event)

  This function returns the event mask value of the current event object.

  :param event: Location to return the event group reference.

  :return: The event mask value of the current event object.

.. c:function:: int nxevent_open(FAR nxevent_t **event, FAR const char *name, int oflags, ...)

  This function establishes a connection between named event groups and a
  task. the task may reference the event group associated with name using
  the address returned by this call. The event group may be used in a
  subsequent calls to nxevent_wait(), or nxevent_post(). And the event
  group remains usable until the event group is closed by a successful
  call to nxevent_close().

  If a task makes multiple calls to event_open() with the same name, then
  the same event group address is returned.

  :param event: Location to return the event group reference.
  :param name: Event group name.
  :param oflags: Event group creation options. This may the following settings:

    -  ``oflags`` = 0: Connect to the event group only if it already exists.
    -  ``oflags`` = O_CREAT: Connect to the event group if it exists, otherwise create the event group.
    -  ``oflags`` = O_CREAT|O_EXCL: Create a new event group unless already exists.

  :param ...: Optional parameters. When the O_CREAT flag is specified,
              the two optional parameters are expected:

    -  ``mode``: The mode parameter is of type mode_t. This parameter is
        required but not used in the present implementation.
    -  ``events``: The events parameter is type unsigned. The event group
        is created with an initial value of ``events``.

  :return: 0 (OK), or negated errno if unsuccessful.

.. c:function:: int nxevent_close(FAR nxevent_t *event)

    This function is called to indicate that the calling task is finished
    with the specified named event group. The event_close() deallocates
    any system resources allocated by the system for this named event group.

  :param event: event descriptor

  :return: 0 (OK), or negated errno if unsuccessful.
