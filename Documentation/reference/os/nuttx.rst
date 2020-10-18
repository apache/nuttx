=====================================================
APIs Exported by NuttX to Architecture-Specific Logic
=====================================================

These are standard interfaces that are exported by the OS for use
by the architecture specific logic.

.. c:function:: void nx_start(void)

  **To be provided**

OS List Management APIs
=======================

**To be provided**

.. c:function:: void nxsched_process_timer(void)

  This function handles system timer events. The
  timer interrupt logic itself is implemented in the architecture
  specific code, but must call the following OS function
  periodically -- the calling interval must be
  ``CONFIG_USEC_PER_TICK``.

.. c:function:: void nxsched_timer_expiration(void)

  Description: if ``CONFIG_SCHED_TICKLESS`` is defined, then this
  function is provided by the RTOS base code and called from
  platform-specific code when the interval timer used to implemented
  the tick-less OS expires.

  **Assumptions**: Base code implementation assumes that this
  function is called from interrupt handling logic with interrupts disabled.

.. c:function:: void nxsched_alarm_expiration(void);

  If ``CONFIG_SCHED_TICKLESS`` is defined, then this
  function is provided by the RTOS base code and called from
  platform-specific code when the interval timer used to implemented
  the tick-less OS expires.

  **Assumptions**: Base code implementation assumes that this
  function is called from interrupt handling logic with interrupts disabled.

.. c:function:: void irq_dispatch(int irq, FAR void *context)

  This function must be called from the
  architecture- specific logic in order to display an interrupt to
  the appropriate, registered handling logic.

