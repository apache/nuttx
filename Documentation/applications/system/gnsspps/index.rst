=========================================
``gnsspps`` GNSS PPS clock discipline
=========================================

Overview
========

``gnsspps`` synchronizes NuttX boards to GNSS time using the PPS (pulse per
second) output of a GNSS receiver. Each board has its own receiver (or they
share one PPS signal), and each board disciplines a local PTP clock
(``/dev/ptpX``) to the pulses. All the boards then share the UTC time scale,
to within the accuracy of the receivers' PPS, without any network.

It complements the Ethernet PTP support (``ptpd``): PTP needs a network and a
master clock, while GNSS PPS gives every board an absolute reference directly.

The chain is made of three parts:

#. A PTP clock with an *external time stamp* (EXTTS) input. The PPS edge
   latches the clock counter in hardware and the driver queues the time stamp
   as a ``struct ptp_extts_event`` (Linux compatible ``PTP_EXTTS_REQUEST``
   API). ``CONFIG_PTP_PPS`` provides such a clock on top of any 32-bit timer
   with an input capture channel, see :doc:`/os/drivers/special/ptp`.
#. ``gnsspps`` reads the time stamps. Their fractional second is the phase
   error of the clock; a PI servo steers the clock frequency with
   ``clock_adjtime(ADJ_FREQUENCY)`` to drive it to zero.
#. The whole seconds come from the NMEA ``RMC`` or ``ZDA`` sentences that the
   receiver sends after each pulse. Without NMEA, ``gnsspps`` still locks the
   phase (all boards agree on the second boundaries) and takes the seconds
   from ``CLOCK_REALTIME``.

Applications read the disciplined time with ``clock_gettime()`` on the PTP
clock. Optionally (``-r``) ``CLOCK_REALTIME`` follows the clock as well.

.. code-block:: c

   int fd = open("/dev/ptp0", O_RDONLY);
   clockid_t clk = (fd << CLOCK_SHIFT) | CLOCK_FD;
   struct timespec ts;

   clock_gettime(clk, &ts);   /* GNSS time, ~12 ns resolution on STM32F4 */

Design: how to time stamp the PPS
=================================

The accuracy of the whole system is set by how the PPS edge is time stamped.
The possible methods on a microcontroller are, from worst to best:

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Method
     - Error sources
     - Typical result (STM32F4)
   * - GPIO interrupt, then ``clock_gettime(CLOCK_REALTIME)``
     - NuttX interrupt dispatch (about 2 us), critical sections and higher
       priority interrupts (jitter of tens of us), and the resolution of
       ``CLOCK_REALTIME``, which is one system tick (10 ms by default) unless
       tickless mode is used
     - 10 us to 10 ms
   * - Zero latency interrupt (``CONFIG_ARCH_HIPRI_INTERRUPT``) that reads a
       free running counter (e.g. DWT ``CYCCNT``) on entry
     - Not masked by critical sections, but the hardware entry latency still
       varies: completion of multi-cycle instructions, flash wait states and
       cache misses, bus contention, tail chaining, other zero latency
       interrupts. Plus the GPIO/EXTI synchronizer. The handler cannot call
       the OS, so the value must be handed over through PendSV or a software
       interrupt
     - 50 to 200 ns jitter, more complex code
   * - **Timer input capture** (``CONFIG_PTP_PPS``)
     - The timer copies its counter into ``CCRx`` on the edge, in hardware.
       Only quantization (one timer clock) and a constant input synchronizer
       delay, which is identical on identical boards
     - **11.9 ns resolution at 84 MHz, no software jitter**

With input capture the interrupt latency does not matter at all: the edge
was already time stamped by the hardware, and the handler only has to read
``CCRx`` before the next edge, one second later. This is why the STM32 driver
uses an ordinary interrupt at the default priority. A zero latency interrupt
would add complexity (no OS calls, a PendSV hand-over) and would still be
less accurate. It is only worth considering as a fallback on a chip or board
where the PPS cannot reach a timer capture input; see
`Fallback without input capture`_.

The same reasoning applies to the clock itself. ``CLOCK_REALTIME`` is driven
by the system tick: it is read with the resolution of one tick (10 ms by
default) unless tickless mode is used, and ``adjtime()`` only changes the
tick period (``up_adjtime()`` changes the SysTick reload, a step of about
0.6 ppm with a 10 ms tick at 168 MHz) for a limited time. The PPS clock
instead converts the 32-bit counter to nanoseconds with a 64-bit fixed point
factor, giving a frequency resolution better than 1 ppb and a read resolution
of one counter clock. That is why the GNSS time is kept in a
dedicated PTP clock and ``CLOCK_REALTIME`` only follows it.

The servo
=========

#. **Acquire.** Two consecutive pulses give the frequency error of the local
   oscillator (the phase change in one second, in ns, is the error in ppb).
   The frequency is corrected and the phase is removed with one step
   (``ADJ_SETOFFSET``).
#. **Locked.** A PI controller runs on every pulse:
   ``freq = I - (Kp + Ki) * offset`` and ``I -= Ki * offset``, with
   ``Kp = 0.7`` and ``Ki = 0.3`` by default (the values used by linuxptp for
   hardware time stamps at 1 Hz).
#. **Outliers.** When locked, an offset larger than the step threshold
   (``-S``, 20 us by default) is ignored. After 4 consecutive outliers the
   servo acquires again.
#. **Holdover.** If the PPS disappears the last frequency is kept, so the
   clock keeps running at the GNSS-calibrated rate. When the PPS returns the
   servo acquires again.
#. **Seconds.** When an NMEA time arrives less than one second after a pulse,
   it is the UTC time of that pulse. If it differs from the second given by
   the clock in two consecutive sentences, the clock is stepped by whole
   seconds.

Hardware
========

* Connect the PPS output of the receiver to a timer input capture pin (3.3 V
  logic, use a level shifter for a 5 V PPS). Keep the wire short or
  terminated; use ``CONFIG_STM32_PTP_PPS_FILTER`` against noise.
* Connect the receiver TX (NMEA) to a UART RX.
* For the best agreement between boards, use the same receiver model and
  firmware, and compensate the antenna cable delay (about 5 ns per metre of
  coax, ``-D``). The PPS of the receiver itself is typically accurate to
  20-60 ns RMS (see its datasheet); timing receivers are better.

STM32F4Discovery
----------------

The ``stm32f4discovery:gnsspps`` configuration uses TIM5 (32-bit, 84 MHz)
with the PPS on **PA1** (TIM5_CH2) and the NMEA on **USART3 RX, PB11**
(``/dev/ttyS1``, 9600 baud). The console is on USART2 (PA2/PA3).

.. code-block:: console

   $ ./tools/configure.sh stm32f4discovery:gnsspps
   $ make

Other STM32 boards need ``CONFIG_STM32_TIM2`` or ``CONFIG_STM32_TIM5``,
``CONFIG_STM32_PTP_PPS``, a ``GPIO_TIMx_CHyIN`` definition in ``board.h`` and
a call to ``stm32_ptp_pps_initialize()`` in the board bring-up.

Usage
=====

Example on the STM32F4Discovery, just after boot:

.. code-block:: console

   nsh> gnsspps -s /dev/ttyS1 &
   gnsspps [4:200]
   gnsspps: /dev/ptp0 channel 0, NMEA from /dev/ttyS1
   gnsspps: locked, offset +292956130 ns stepped, freq -71107 ppb
   gnsspps: stepping +27368958 s to NMEA time
   gnsspps: rms 203 ns max 521 ns freq -71185 ppb
   gnsspps: rms 95 ns max 147 ns freq -71549 ppb

Every 16 pulses the RMS and maximum phase error and the frequency correction
are printed. ``-v`` prints every sample.

Options:

``-d <dev>``
  PTP clock with the PPS input (default ``CONFIG_SYSTEM_GNSSPPS_DEVPATH``,
  ``/dev/ptp0``).
``-c <n>``
  External time stamp channel that receives the PPS (default 0).
``-s <tty>``
  NMEA serial port of the receiver, for the time of day.
``-b <baud>``
  Set the baud rate of the NMEA port (needs ``CONFIG_SERIAL_TERMIOS``).
``-D <ns>``
  Delay of the PPS to compensate: antenna cable, receiver, level shifter.
``-P <kp>``, ``-I <ki>``
  Servo gains. Lower gains filter the PPS jitter more and make the clock
  smoother, at the cost of a slower response to temperature changes.
``-S <ns>``
  Outlier and re-acquisition threshold when locked (default 20000).
``-r``
  Keep ``CLOCK_REALTIME`` synchronized to the clock: it is stepped when the
  error exceeds ``-R`` (default 1 ms) and slewed with ``adjtime()``
  otherwise. Its accuracy is limited by the resolution of
  ``CLOCK_REALTIME`` (one tick unless tickless).
``-R <ns>``
  ``CLOCK_REALTIME`` step threshold.
``-n <n>``
  Exit after ``n`` pulses.
``-v``
  Print every sample.

NSH accepts 7 arguments by default; the ``stm32f4discovery:gnsspps``
configuration raises ``CONFIG_NSH_MAXARGUMENTS`` to 12.

Testing on the simulator
========================

``CONFIG_PTP_PPS_DUMMY`` simulates a counter whose oscillator has a frequency
error (``CONFIG_PTP_PPS_DUMMY_ERROR_PPB``) and a PPS with a jitter
(``CONFIG_PTP_PPS_DUMMY_JITTER_NS``), using the host time as the reference:

.. code-block:: console

   $ ./tools/configure.sh sim:nsh
   $ kconfig-tweak --enable PTP_CLOCK --enable PTP_PPS --enable PTP_PPS_DUMMY \
                   --enable CLOCK_ADJTIME --enable SYSTEM_GNSSPPS
   $ make olddefconfig && make
   $ ./nuttx
   nsh> gnsspps -v -n 40

With a +37 ppm oscillator error and 20 ns of PPS jitter the servo locks in
about 10 s, measures -37.00 ppm and keeps an RMS phase error of about 13 ns,
which is the jitter of the simulated PPS. With -123 ppm and 50 ns of jitter it
measures +123.45 ppm and keeps 38 ns RMS.

Fallback without input capture
==============================

If the PPS can only reach a plain GPIO, a lower half of ``CONFIG_PTP_PPS`` can
still be written with a zero latency interrupt:

#. Use a free running counter for ``read()`` (for example DWT ``CYCCNT`` or a
   32-bit timer).
#. Attach the GPIO interrupt as a zero latency interrupt
   (``CONFIG_ARCH_HIPRI_INTERRUPT``, ``CONFIG_ARCH_RAMVECTORS``,
   ``arm_ramvec_attach()``, highest priority) whose first instruction reads the
   counter and stores it in a variable, then pends a normal interrupt
   (PendSV or an unused IRQ with ``up_trigger_irq()``).
#. In that normal interrupt call ``ptp_pps_capture()``, which is not allowed
   from the zero latency handler.

Expect 50 to 200 ns of jitter instead of one timer clock. See
:doc:`/guides/concurrency/zerolatencyinterrupts`.

Possible improvements
=====================

* A periodic output (``PTP_PEROUT_REQUEST``) on a timer compare channel, to
  produce a PPS from the disciplined clock and measure the skew between
  boards with an oscilloscope.
* Quantization error correction: u-blox receivers report the error of the
  next pulse (``qErr`` in ``UBX-TIM-TP``), which removes their own
  ``sawtooth`` error of a few ns to some tens of ns.
* Use the GNSS disciplined clock as the grandmaster of ``ptpd`` (clock class
  6, time source GPS) to distribute GNSS time over Ethernet to boards without
  a receiver.
* External time stamp inputs on Ethernet MAC PTP clocks that have an
  auxiliary snapshot input, so the PPS disciplines the MAC clock directly.
