=====================
CPU Frequency Scaling
=====================

The CPU frequency framework lets several unrelated parts of the system have
an opinion about how fast the CPU should run, and resolves those opinions
into one frequency. A platform supplies a lower half: a table of the
frequencies its hardware supports and a way to move between them. Everything
above that is arbitration.

It is enabled with ``CONFIG_CPUFREQ``. There is one policy per system.

Design
======

Each requester installs a ``[min, max]`` window that it can live with. A
thermal cooling device installs one as a zone heats up, an application
holding ``/dev/cpufreq`` installs one, a power manager installs one. The
framework keeps every window and, whenever the set changes, recomputes a
single answer:

1. Take the lowest ``max`` across all installed requests. A request that
   passes ``CPUFREQ_NO_LIMIT`` for a bound leaves that side unconstrained
   and does not participate.
#. Choose the highest table entry at or below that ceiling.
#. Apply it through the lower half, if it differs from the entry currently
   applied.

Speed is therefore the default: with no requests installed, the top table
entry is selected, and any single requester can pull the system down.

Two consequences of this are worth stating plainly, because they are design
decisions rather than oversights:

- **A floor never raises the frequency.** Because the resolver already picks
  the highest permitted entry, any ``min`` that can be satisfied already is.
  The ``min`` field is accepted and stored so that a request expresses a
  complete window, but it does not participate in the calculation.
- **When windows do not intersect, the lowest ceiling wins.** A requester
  asking for less speed is presumed to be protecting something, so its
  ceiling is honoured and a conflicting floor is not.

The lower half is never told who asked for what. It only ever hears "go to
table entry N".

The Lower Half
==============

A platform provides a ``struct cpufreq_driver``. ``get_table`` and
``target_index`` are mandatory; the rest may be NULL:

.. code-block:: c

  struct cpufreq_driver
  {
    CODE FAR const struct cpufreq_frequency_table *
                   (*get_table)(FAR struct cpufreq_policy *policy);
    CODE int (*target_index)(FAR struct cpufreq_policy *policy,
                             unsigned int index);
    CODE int (*get_frequency)(FAR struct cpufreq_policy *policy);
    CODE int (*suspend)(FAR struct cpufreq_policy *policy);
    CODE int (*resume)(FAR struct cpufreq_policy *policy);
  };

``get_table``
  Returns the frequency table. It must ascend, and it must end with an entry
  whose ``frequency`` is ``CPUFREQ_TABLE_END``.

``target_index``
  Moves the hardware to the table entry at ``index``. This is the only call
  that changes the frequency.

``get_frequency``
  Reports where the hardware actually is, in table units. Called once during
  initialisation so the framework can start from the truth rather than an
  assumption. If NULL, the framework assumes the lowest entry.

``suspend`` and ``resume``
  Called from ``cpufreq_suspend()`` and ``cpufreq_resume()``.

The frequency unit is the lower half's choice. kHz is the Linux convention
and a reasonable default, but the framework does not care as long as every
consumer of the same policy agrees.

Bring the framework up once, after the hardware it drives is ready:

.. code-block:: c

  static struct cpufreq_driver g_mychip_cpufreq =
  {
    .get_table     = mychip_get_table,
    .target_index  = mychip_target_index,
    .get_frequency = mychip_get_frequency,
  };

  cpufreq_init(&g_mychip_cpufreq);

``cpufreq_init()`` returns ``-EBUSY`` if called twice, and registers
``/dev/cpufreq`` when ``CONFIG_CPUFREQ_CHARDEV`` is set.

.. note::
  ``driver`` must remain the first member of ``struct cpufreq_policy``.
  Existing consumers reach the lower half by casting a policy pointer.

In-kernel Requests
==================

Kernel code constrains the frequency through three calls:

.. code-block:: c

  FAR struct cpufreq_qos *qos;

  qos = cpufreq_qos_add_request(cpufreq_policy_get(),
                                CPUFREQ_NO_LIMIT,   /* min */
                                800000);            /* max */

  cpufreq_qos_update_request(qos, CPUFREQ_NO_LIMIT, 1200000);

  cpufreq_qos_remove_request(qos);

Each call re-resolves the frequency before returning.
``cpufreq_qos_remove_request()`` frees the request.

``cpufreq_policy_get()`` returns NULL before ``cpufreq_init()`` has run.

/dev/cpufreq
============

With ``CONFIG_CPUFREQ_CHARDEV`` the policy is also a character device, so an
application can read the frequency and install a request of its own. Each
open descriptor owns at most one request, which is released when the
descriptor closes, including on task exit.

``CPUFREQIOC_GET_FREQUENCY``
  Arg: ``FAR unsigned int *``. Receives the current frequency.

``CPUFREQIOC_SET_REQUEST``
  Arg: ``FAR const struct cpufreq_request_s *``. Installs or updates this
  descriptor's request. Either bound may be ``CPUFREQ_NO_LIMIT``.

``CPUFREQIOC_CLEAR_REQUEST``
  No argument. Removes this descriptor's request.

``CPUFREQIOC_GET_TABLE``
  Arg: ``FAR struct cpufreq_table_query_s *``. Copies out the frequency
  table. Set ``frequencies`` and ``maxlen``; ``nentries`` is returned as the
  number the table really has, which may exceed ``maxlen``.

.. code-block:: c

  struct cpufreq_request_s req =
    {
      .min = CPUFREQ_NO_LIMIT,
      .max = 800000,
    };

  int fd = open("/dev/cpufreq", O_RDONLY);
  ioctl(fd, CPUFREQIOC_SET_REQUEST, (unsigned long)&req);

  /* The cap holds for as long as this descriptor is open */

  close(fd);

Thermal Integration
===================

``CONFIG_THERMAL_CDEV_CPUFREQ`` registers CPU frequency as a thermal cooling
device, so a thermal zone can throttle the CPU with no board code in
between. The cooling device counts states downward from the top of the
frequency table: state 0 is unthrottled, and each step installs a tighter
window through the same QoS interface described above.

See :doc:`../thermal/index` for how zones, trip points and cooling devices
fit together.

Suspend and Resume
==================

.. code-block:: c

  cpufreq_suspend();
  cpufreq_resume();

These pass through to the lower half's ``suspend`` and ``resume``. While
suspended, the resolver leaves the hardware alone: requests are still
accepted and still recorded, and whatever they resolve to is applied on
resume.

Configuration
=============

``CONFIG_CPUFREQ``
  Enables the framework.

``CONFIG_CPUFREQ_CHARDEV``
  Registers ``/dev/cpufreq``. Default on.

``CONFIG_THERMAL_CDEV_CPUFREQ``
  Registers the thermal cooling device. Requires ``CONFIG_THERMAL``.
