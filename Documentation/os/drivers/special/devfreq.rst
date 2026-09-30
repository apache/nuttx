========================
Device Frequency Scaling
========================

The device frequency framework (devfreq) lets several unrelated parts of the
system have an opinion about how fast a device should run, and resolves those
opinions into one frequency. A platform supplies a lower half: a table of the
frequencies its hardware supports and a way to move between them. A governor
decides, from moment to moment, where inside the arbitrated window the device
should sit. Everything else is arbitration.

Unlike the CPU frequency framework, which manages a single system-wide CPU
policy, devfreq manages any number of independent devices, each registered by
name. A GPU, a memory bus, and a DSP can each have their own devfreq instance,
table and governor.

It is enabled with ``CONFIG_DEVFREQ``.

Design
======

A devfreq instance is created by a driver calling ``devfreq_register()`` with
a name, a governor and a lower half. From then on two independent forces act
on the frequency:

- **QoS requests** narrow the allowed window. Each requester installs a
  ``[min, max]`` window it can live with, and the framework aggregates every
  window into a single ``[min, max]`` clamp on the device.
- **The governor** picks a target inside that clamp. The ``performance``
  governor always asks for the top of the window, ``powersave`` always asks
  for the bottom, and ``ondemand`` moves between them according to load.

Whenever the set of requests changes, the framework recomputes the aggregate
window and lets the governor re-pick. The chosen frequency is then snapped to
a real table entry and applied through the lower half.

Frequencies are expressed in kHz throughout.

Resolving Requests
------------------

Each QoS request carries a ``min`` and a ``max``. The aggregate window is the
intersection of all of them: the highest ``min`` across every request, and
the lowest ``max``. A floor is honoured here, so a requester that needs a
device to run *at least* some speed can guarantee it, and a ceiling caps it.

When the requests do not intersect (the aggregate ``min`` ends up above the
aggregate ``max``) the driver's ``conflict_policy`` decides who wins:

- ``DEVFREQ_CONFLICT_PREFER_HIGH`` clamps to the floor and chooses the higher
  frequency. A device that would rather waste power than stall picks this.
- ``DEVFREQ_CONFLICT_PREFER_LOW`` clamps to the ceiling and chooses the lower
  frequency. A device protecting a thermal or power budget picks this.

The thermal framework is one such requester. With
``CONFIG_THERMAL_CDEV_DEVFREQ`` it registers a cooling device over the
devfreq device named by ``CONFIG_THERMAL_CDEV_DEVFREQ_NAME``, and each
cooling state installs a ceiling one entry further down the table. Drivers
that expect to be throttled should carry ``DEVFREQ_CONFLICT_PREFER_LOW``, so
that the ceiling wins against anything asking for more.

A table of *n* usable frequencies gives the cooling device a maximum state of
*n* - 1: state zero caps at the highest frequency, which leaves the device
unthrottled, and the maximum state caps at the lowest. Entries of
``DEVFREQ_ENTRY_INVALID`` are not frequencies the device can be held at, so
they earn no state, and the ``low`` and ``high`` bounds of a thermal zone's
cooling map therefore count usable frequencies rather than table positions.

The resolved ``[min, max]`` is then snapped to the table: ``min`` rounds up to
the lowest entry at or above it, ``max`` rounds down to the highest entry at
or below it. The governor picks within that snapped range, and the lower half
is told only "go to table entry N".

The Lower Half
==============

A platform provides a ``struct devfreq_driver_s``. ``get_table`` and
``target_index`` are mandatory; the rest may be NULL:

.. code-block:: c

  struct devfreq_driver_s
  {
    int conflict_policy;
    CODE FAR const uint32_t *
             (*get_table)(FAR struct devfreq_s *devfreq);
    CODE int (*target_index)(FAR struct devfreq_s *devfreq,
                             size_t index);
    CODE uint32_t (*get_frequency)(FAR struct devfreq_s *devfreq);
    CODE int (*suspend)(FAR struct devfreq_s *devfreq);
    CODE int (*resume)(FAR struct devfreq_s *devfreq);
  };

``conflict_policy``
  ``DEVFREQ_CONFLICT_PREFER_HIGH`` or ``DEVFREQ_CONFLICT_PREFER_LOW``, applied
  when QoS windows do not intersect, as described above.

``get_table``
  Returns the frequency table, an array of ``uint32_t`` in kHz. It must
  ascend, and it must end with an entry equal to ``DEVFREQ_ENTRY_END``. An
  entry of ``DEVFREQ_ENTRY_INVALID`` is skipped, which lets a driver punch a
  hole in an otherwise fixed table.

``target_index``
  Moves the hardware to the table entry at ``index``. This is the only call
  that changes the frequency.

``get_frequency``
  Reports where the hardware actually is, in kHz. The framework consults it
  rather than trusting a cached value, so an external change is noticed.

``suspend`` and ``resume``
  Called from ``devfreq_suspend()`` and ``devfreq_resume()``.

Register the device once its hardware is ready:

.. code-block:: c

  static const struct devfreq_driver_s g_mydev_devfreq =
  {
    .conflict_policy = DEVFREQ_CONFLICT_PREFER_LOW,
    .get_table       = mydev_get_table,
    .target_index    = mydev_target_index,
    .get_frequency   = mydev_get_frequency,
  };

  devfreq_register("gpu", devfreq_performance(),
                   &g_mydev_devfreq, priv);

``devfreq_register()`` returns a handle, or NULL on failure, including when a
device of the same name is already registered. Pass the governor you want the
device to start with; ``devfreq_performance()`` and ``devfreq_powersave()``
return the two built-in governors, and the ondemand governor is available when
``CONFIG_DEVFREQ_GOV_ONDEMAND`` is built in.

.. code-block:: c

  int devfreq_unregister(FAR struct devfreq_s *devfreq);

``devfreq_unregister()`` stops the governor, tears the instance down and frees
it.

Governors
=========

A governor is a small ``struct devfreq_governor_s`` with lifecycle callbacks
and a ``limit`` that returns the frequency the governor currently wants. The
framework clamps that want to the QoS window before applying it.

``performance``
  Always wants the maximum of the window. Built in.

``powersave``
  Always wants the minimum of the window. Built in.

``ondemand``
  Samples CPU load periodically and scales between the window's bounds. When
  load crosses ``CONFIG_DEVFREQ_LOAD_THRESHOLD`` it asks for the top;
  otherwise it scales proportionally. The sampling interval defaults to
  ``CONFIG_DEVFREQ_SAMPLE_RATE`` microseconds. Enabled with
  ``CONFIG_DEVFREQ_GOV_ONDEMAND``.

A driver may also supply its own governor to ``devfreq_register()`` instead of
a built-in one.

In-kernel Requests
==================

Kernel code constrains a device's frequency through three calls:

.. code-block:: c

  FAR struct qos_request_s *qos;

  qos = devfreq_qos_add_request(devfreq,
                                200000,    /* min kHz */
                                800000);   /* max kHz */

  devfreq_qos_update_request(devfreq, qos, 400000, 800000);

  devfreq_qos_remove_request(devfreq, qos);

Each call re-resolves the window and lets the governor re-pick before
returning. ``devfreq_qos_remove_request()`` frees the request.

The current frequency can be read at any time:

.. code-block:: c

  uint32_t khz = devfreq_get_frequency(devfreq);

A device is looked up by name or by index when its handle is not already held:

.. code-block:: c

  FAR struct devfreq_s *devfreq = devfreq_find_by_name("gpu");

Change Notifications
====================

Interested code can register a notifier block to hear about every frequency
transition. The chain is called with ``DEVFREQ_PRECHANGE`` before the change
and ``DEVFREQ_POSTCHANGE`` after, each carrying a ``struct devfreq_notifier_s``
with the old and new frequencies. If the lower half's ``target_index`` fails,
a compensating pair is sent so listeners always end on the hardware's true
state.

.. code-block:: c

  devfreq_register_notifier(devfreq, &nb);
  devfreq_unregister_notifier(devfreq, &nb);

procfs
======

With ``CONFIG_DEVFREQ_PROCFS`` each registered device appears under
``/proc/devfreq/<name>``. Reading it reports the device name, its current
governor, the current frequency, whether it is suspended, and the frequency
table:

.. code-block:: text

  nsh> cat /proc/devfreq/gpu
   devfreq:     gpu
   governor:    ondemand
   cur_freq:    400000
   suspended:   False
   freq_table:  200000 400000 600000 800000

Writing to the entry installs a frequency QoS constraint from user space, so
an application can cap or floor a device without kernel code.

With ``CONFIG_DEVFREQ_PROCFS_QOS`` the read also lists every outstanding QoS
request as ``min, max`` pairs. When ``CONFIG_LIBC_BACKTRACE_DEPTH`` is greater
than zero, each request is annotated with the call stack that installed it,
which turns "who is holding this device down?" into a question with an answer.

Suspend and Resume
==================

.. code-block:: c

  devfreq_suspend(devfreq);
  devfreq_resume(devfreq);

These pass through to the lower half's ``suspend`` and ``resume`` and stop or
restart the governor. While suspended the governor does not touch the
hardware; requests are still accepted and recorded, and whatever they resolve
to takes effect on resume.

Configuration
=============

``CONFIG_DEVFREQ``
  Enables the framework.

``CONFIG_DEVFREQ_PROCFS``
  Exposes each device under ``/proc/devfreq``. Requires ``CONFIG_FS_PROCFS``.

``CONFIG_DEVFREQ_PROCFS_QOS``
  Lists outstanding QoS requests, with call stacks when backtrace is
  available, in the procfs output. Requires ``CONFIG_DEVFREQ_PROCFS``.

``CONFIG_DEVFREQ_GOV_ONDEMAND``
  Builds the ondemand governor. Requires CPU-load sampling
  (``!CONFIG_SCHED_CPULOAD_NONE``).

``CONFIG_DEVFREQ_SAMPLE_RATE``
  The ondemand governor's sampling interval, in microseconds.

``CONFIG_DEVFREQ_LOAD_THRESHOLD``
  The load percentage at which ondemand jumps to the maximum frequency.
