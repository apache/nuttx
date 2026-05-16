===============================
Naming of OS Internal Functions
===============================

.. note:: Most of the naming examples used in this section no longer exist.
          They have been converted to the naming conventions described
          in this section. While they are good examples, they no longer
          reflect the current state of the code base.


Legacy smooshed-together names
==============================

There has been some controvery lately about the consistency of naming of OS
internal functions. We should come to a consensus on the desired naming
of conventions for internal OS functions before we start arbitrarily
changing names.

NuttX function names are all lower case.
That is required by the naming standard.
Historically, has used a subsystem name, an underscore, then all of the
renaming words smooshed together. For example:

.. code-block:: c

  void sched_mergeprioritized(FAR dq_queue_t *list1, FAR dq_queue_t *list2,
                              uint8_t task_state);

I would represent that form as::

  <subsystem>_<everythingelsesmooshedaltogether>

Many of these names are impossible to read unless you stop and parse
the letters to find the word boundaries.


System-Object-Verb Naming
=========================

Most recent naming has broken out one more of the smooshed-together-words
and separated them with undersore characters like:

.. code-block:: c

  unsigned int sched_timer_cancel(void);
  void sched_timer_resume(void);
  void sched_timer_reassess(void);

Which I would describe as::

  <subsystem>_<object>_<verb>


System-Verb-Object Naming
=========================

Another form, which we have been using lately, is to switch the
``<object>`` and the ``<verb>`` like::

  <subsystem>_<verb>_<object>

For example:

.. code-block:: c

  int sched_get_stackinfo(pid_t pid, FAR struct stackinfo_s *stackinfo);

By the ``<subsystem>_<object>_<verb>`` naming this would have been:

.. code-block:: c

 int sched_stackinfo_get(pid_t pid, FAR struct stackinfo_s *stackinfo);

And the timer interfaces would become the following under the
``<subsystem>_<verb>_<object>`` rule:

.. code-block:: c

  unsigned int sched_cancel_timerl(void);
  void sched_resume_timer(void);
  void sched_reassess_timer(void);

And the "smooshed" name ``sched_mergeprioritized()`` would become:

.. code-block:: c

  void sched_merge_prioritized(FAR dq_queue_t *list1, FAR dq_queue_t *list2,
                               uint8_t task_state);

We won't even consider the ``<subsystem>_<everythingelsesmooshedaltogether>``.
It has a history but is pretty much odious.
We should not consider, for example, the smooshed-together form:

.. code-block:: c

  int sched_getstackinfo(pid_t pid, FAR struct stackinfo_s *stackinfo);

What an ugly mess! :-)


Unit Suffixes
=============

Often, you will have the functions that return the same value,
but in different units.
For example, these two functions return the system time:

.. code-block:: c

  clock_t clock_systimer(void);
  int clock_systimespec(FAR struct timespec *ts);

The first returns the time in system clock ticks.
The second returns the time as a struct timespec.

I have seen a naming practice The keeps the naming of the functions the same,
but appends the units, preceded by an underscore character, at the end
of the function name. For example, these could be:

.. code-block:: c

  clock_t clock_systime_ticks(void);
  int clock_systime_timespec(FAR struct timespec *ts);

Or should they include a ``<verb>``:

.. code-block:: c

  clock_t clock_get_systime_ticks(void);
  int clock_get_systime_timespec(FAR struct timespec *ts);

That form would then be::

  <subsystem>_<verb>_<object>[_<units>]

Where the square bracket represent and optional unit suffix.


Implicit get
============

In many cases, I think that the ``_get_`` is implicit and need not appear
in the function name. But if there are multiple operations
on the ``<object>``, then the ``_get_`` would be necessary.
Hence, I think the naming:

.. code-block:: c

  clock_t clock_systime_ticks(void);
  int clock_systime_timespec(FAR struct timespec *ts);

is perfectly accepable without the ``_get_``.
And since ``_get_`` is the only operation on ``stackinfo``,
the following would be a perfectly acceptable renaming:

.. code-block:: c

  int sched_stackinfo(pid_t pid, FAR struct stackinfo_s *stackinfo);


Other Naming
============

There is other naming that uses long names with many underscore characters
separating each word in the long name. We are not a fan of long names.


Conclusion
==========

Our preference would be the ``<subsystem>_<verb>_<object>`` form.
We have been using this form in creating all new internal interfaces.
Does anyone else have a thought? or a preference?

We should come to an understanding and stop the arbitrary name changes.
We hope that we can put an end to the smooshed-together naming
(except wheree required) in any case.

.. note:: This applies only to the internal naming of functions within the OS.
          Other names are imposed on us by standards for application
          interfaces to the OS which may follow other rules.
