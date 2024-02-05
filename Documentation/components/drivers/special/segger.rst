==================
Segger RTT drivers
==================

.. note:: Segger drivers works only with J-Link debug probes.
          Sometimes it's possible to replace vendor-specific debug interface
          with J-Link OB firmware. For details look at
          `Segger website <https://www.segger.com/downloads/jlink>`_

Supported Segger drivers:

* Serial over RTT - ``CONFIG_SERIAL_RTTx``,
* Console over RTT - ``CONFIG_SERIAL_RTT_CONSOLE_CHANNEL``
* Segger SystemView - ``CONFIG_SEGGER_SYSVIEW``
* Note RTT - ``CONFIG_NOTE_RTT``

Segger SystemView
=================

Steps to enable SystemView support:

#. Make sure your architecture supports a high-performance counter.
   In most cases it will be:

   :menuselection:`CONFIG_ARCH_PERF_EVENTS=y`

   In that case, the the architecture logic must initialize the perf counter
   with ``up_perf_init()``.

#. Enable instrumentation support:

   :menuselection:`CONFIG_SCHED_INSTRUMENTATION=y`

#. Configure instrumentation support. Available options for SystemView are:

   :menuselection:`CONFIG_SCHED_INSTRUMENTATION_SWITCH=y`

   :menuselection:`CONFIG_SCHED_INSTRUMENTATION_SYSCALL=y`

   :menuselection:`CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER=y`

#. Make sure that ``CONFIG_TASK_NAME_SIZE > 0``, otherwise task/thread
   names will not be displayed correctly

#. Enable Note Driver support and disable Note RAM driver:

   :menuselection:`CONFIG_DRIVERS_NOTE=y`

   :menuselection:`CONFIG_DRIVERS_NOTERAM=n`

#. Enable Note RTT and Segger SystemView support:

   :menuselection:`CONFIG_NOTE_RTT=y`

   :menuselection:`CONFIG_SEGGER_SYSVIEW=y`

#. Configure RTT channel and RTT buffer size for SystemView:

   :menuselection:`CONFIG_SEGGER_SYSVIEW_RTT_CHANNEL=0`

   :menuselection:`CONFIG_SEGGER_SYSVIEW_RTT_BUFFER_SIZE=1024`

   In case SystemView returns buffer overflow errors, you should increase
   ``CONFIG_NOTE_RTT_BUFFER_SIZE_UP``.
