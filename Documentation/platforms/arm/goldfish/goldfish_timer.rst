===============
GOLDFISH TIMER
===============

Introduction
============
This device is used to return the current host time as a high-precision signed 64-bit nanosecond value to the kernel, with the starting point being an arbitrary loose reference point. The value should correspond to the QEMU "vm_clock"; that is, it should not be updated when the emulated system is not running, hence it cannot be directly based on the host clock.

Timer Registers
===============

::

    #define GOLDFISH_TIMER_TIME_LOW         0x0  /* Get current time, then return low-order 32-bits. */
    #define GOLDFISH_TIMER_TIME_HIGH        0x4  /* Return high 32-bits from previous TIME_LOW read. */
    #define GOLDFISH_TIMER_ALARM_LOW        0x8  /* Set low 32-bit value of alarm, then arm it. */
    #define GOLDFISH_TIMER_ALARM_HIGH       0xc  /* Set high 32-bit value of alarm. */
    #define GOLDFISH_TIMER_IRQ_ENABLED      0x10 /* Enable interrupts. */
    #define GOLDFISH_TIMER_CLEAR_ALARM      0x14 /* Clear alarm. */
    #define GOLDFISH_TIMER_ALARM_STATUS     0x18 /* Return 1 if alarm is armed, 0 if not. */
    #define GOLDFISH_TIMER_CLEAR_INTERRUPT  0x1c /* Clear interrupt. */

Timer Read
==========
To read the current time, the kernel must execute an IO_READ(TIME_LOW), which returns an unsigned 32-bit value, followed by an IO_READ(TIME_HIGH), which returns a signed 32-bit value corresponding to the high half of the complete value.

::

    static int goldfish_timer_current(struct oneshot_lowerhalf_s *lower_,
                                      struct timespec *ts)
    {
      struct goldfish_timer_lowerhalf_s *lower =
        (struct goldfish_timer_lowerhalf_s *)lower_;
      irqstate_t flags;
      uint32_t l32;
      uint32_t h32;
      uint64_t nsec;

      DEBUGASSERT(lower != NULL);

      flags = spin_lock_irqsave(&lower->lock);

      l32 = getreg32(lower->base + GOLDFISH_TIMER_TIME_LOW);
      h32 = getreg32(lower->base + GOLDFISH_TIMER_TIME_HIGH);
      nsec = ((uint64_t)h32 << 32) | l32;

      ts->tv_sec  = nsec / NSEC_PER_SEC;
      ts->tv_nsec = nsec % NSEC_PER_SEC;

      spin_unlock_irqrestore(&lower->lock, flags);

      return 0;
    }

Timer Set Alarm
===============
This device can also be used to set an alarm, as follows:

::

    IO_WRITE(ALARM_HIGH, <high-value>)  // Must be executed first.
    IO_WRITE(ALARM_LOW, <low-value>)    // Must be executed second.

When the corresponding value is reached, the device will raise its IRQ. Note that if the alarm value is already older than the current time, the IRQ will be triggered immediately after the second IO_WRITE().

::

    static int goldfish_timer_start(struct oneshot_lowerhalf_s *lower_,
                                    oneshot_callback_t callback,
                                    void *arg,
                                    const struct timespec *ts)
    {
      struct goldfish_timer_lowerhalf_s *lower =
        (struct goldfish_timer_lowerhalf_s *)lower_;
      irqstate_t flags;
      uint64_t nsec;
      uint32_t l32;
      uint32_t h32;

      DEBUGASSERT(lower != NULL);

      flags = spin_lock_irqsave(&lower->lock);

      lower->callback = callback;
      lower->arg      = arg;

      nsec  = ts->tv_sec * 1000000000 + ts->tv_nsec;
      l32   = getreg32(lower->base + GOLDFISH_TIMER_TIME_LOW);
      h32   = getreg32(lower->base + GOLDFISH_TIMER_TIME_HIGH);
      nsec += ((uint64_t)h32 << 32) | l32;

      putreg32(1, lower->base + GOLDFISH_TIMER_IRQ_ENABLED);
      putreg32(nsec >> 32, lower->base + GOLDFISH_TIMER_ALARM_HIGH);
      putreg32(nsec, lower->base + GOLDFISH_TIMER_ALARM_LOW);

      spin_unlock_irqrestore(&lower->lock, flags);

      return 0;
    }

Timer Interrupt
===============
IO_WRITE(CLEAR_INTERRUPT, <any>) can be used to lower the IRQ level after the kernel has processed the alarm.
IO_WRITE(CLEAR_ALARM, <any>) can be used to disarm the current alarm (if one exists).

Note: Currently, the alarm is used only on ARM-based systems. On MIPS-based systems, only TIME_LOW / TIME_HIGH are used.

::

    static int goldfish_timer_interrupt(int irq,
                                        void *context,
                                        void *arg)
    {
      struct goldfish_timer_lowerhalf_s *lower = arg;
      oneshot_callback_t callback = NULL;
      irqstate_t flags;
      void *cbarg;

      DEBUGASSERT(lower != NULL);

      flags = spin_lock_irqsave(&lower->lock);

      putreg32(1, lower->base + GOLDFISH_TIMER_CLEAR_ALARM);

      if (lower->callback != NULL)
        {
          callback        = lower->callback;
          cbarg           = lower->arg;
          lower->callback = NULL;
          lower->arg      = NULL;
        }

      spin_unlock_irqrestore(&lower->lock, flags);

      /* Then perform the callback */

      if (callback)
        {
          callback(&lower->lh, cbarg);
        }

      return 0;
    }
