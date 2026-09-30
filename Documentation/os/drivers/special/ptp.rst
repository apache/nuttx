===========================
PTP Clock Driver Framework
===========================

Overview
========

The PTP (Precision Time Protocol) Clock driver framework provides support for
IEEE 1588 compliant hardware clocks in NuttX. This framework enables precise
time synchronization across networked systems, achieving accuracy within
microseconds or even nanoseconds with hardware timestamping support.

The PTP clock framework follows a layered architecture with upper-half driver
logic in the kernel and lower-half hardware-specific implementations, similar
to other NuttX device drivers.

Architecture
============

The PTP clock framework consists of the following components:

Upper-Half Driver
-----------------

The upper-half driver (``drivers/timers/ptp_clock.c``) provides:

- Character device interface (``/dev/ptpN``)
- Common ioctl command handling
- Frequency and time adjustment logic
- Cross-timestamp support
- Interface to POSIX clock APIs through the ``CLOCK_FD`` clock type

Lower-Half Driver
-----------------

Hardware-specific drivers embed a ``struct ptp_lowerhalf_s`` and point its
``ops`` field to a ``struct ptp_ops_s`` with the following operations:

.. code-block:: c

   struct ptp_lowerhalf_s
   {
     FAR const struct ptp_ops_s *ops;  /* Lower half driver operations */
     FAR void *upper;                  /* The upper handle */
   };

   struct ptp_ops_s
   {
     CODE int (*adjfine)(FAR struct ptp_lowerhalf_s *lower, long ppb);
     CODE int (*adjphase)(FAR struct ptp_lowerhalf_s *lower, int32_t phase);
     CODE int (*adjtime)(FAR struct ptp_lowerhalf_s *lower, int64_t delta);
     CODE int (*gettime)(FAR struct ptp_lowerhalf_s *lower,
                         FAR struct timespec *ts,
                         FAR struct ptp_system_timestamp *sts);
     CODE int (*getcrosststamp)(FAR struct ptp_lowerhalf_s *lower,
                                FAR struct system_device_crosststamp *cts);
     CODE int (*settime)(FAR struct ptp_lowerhalf_s *lower,
                         FAR const struct timespec *ts);
     CODE int (*getres)(FAR struct ptp_lowerhalf_s *lower,
                        FAR struct timespec *res);
     CODE int (*control)(FAR struct ptp_lowerhalf_s *lower,
                         int cmd, unsigned long arg);
   };

The units are:

- ``adjfine``: the frequency offset from the nominal frequency, in parts per
  billion. The upper half converts the ``freq`` field of ``struct timex``
  (parts per million with 16 fractional bits) and rejects values beyond the
  ``max_adj`` given at registration before calling it.
- ``adjphase``: the change to apply to the phase, in nanoseconds.
- ``adjtime``: the change to apply to the time, in nanoseconds.

The upper half calls ``adjtime`` unconditionally, so every driver has to
provide it. The other operations are optional: ``gettime``, ``settime`` and
``getres`` make the matching ioctl return ``-ENOTSUP`` when they are missing,
``adjfine`` and ``adjphase`` make the corresponding ``clock_adjtime()`` mode
fail the same way, and ``getcrosststamp`` is reported through the
``cross_timestamping`` capability. The capabilities are built by the upper
half from which operations exist and from ``max_adj``, so there is no
operation to report them. Ioctl commands that the upper half does not know
are passed to ``control``.

Configuration Options
=====================

The PTP clock framework can be enabled with the following Kconfig options:

``CONFIG_PTP_CLOCK``
  Enable PTP clock driver framework support. This provides the upper-half
  driver infrastructure and POSIX clock API integration.

``CONFIG_PTP_CLOCK_DUMMY``
  Enable a software-based dummy PTP clock driver for testing and development.
  This driver provides a PTP clock implementation without hardware support,
  using ``CLOCK_REALTIME`` as the time base.

``CONFIG_CLOCK_ADJTIME``
  Enable the ``clock_adjtime()`` system call, required for frequency and
  phase adjustments of PTP clocks.

Device Interface
================

Character Device
----------------

PTP clocks are exposed as character devices with names like ``/dev/ptp0``,
``/dev/ptp1``, etc. Applications can open these devices and perform operations
using ioctl commands.

IOCTL Commands
--------------

The following ioctl commands are supported:

``PTP_CLOCK_GETTIME``
  Get the current time from the PTP clock.

  .. code-block:: c

     struct timespec ts;
     ioctl(fd, PTP_CLOCK_GETTIME, &ts);

``PTP_CLOCK_SETTIME``
  Set the time of the PTP clock.

  .. code-block:: c

     struct timespec ts;
     ts.tv_sec  = 1234567890;
     ts.tv_nsec = 123456789;
     ioctl(fd, PTP_CLOCK_SETTIME, &ts);

``PTP_CLOCK_GETRES``
  Get the resolution of the PTP clock.

  .. code-block:: c

     struct timespec res;
     ioctl(fd, PTP_CLOCK_GETRES, &res);

``PTP_CLOCK_ADJTIME``
  Adjust the time or frequency of the PTP clock.

  .. code-block:: c

     struct timex tx;
     memset(&tx, 0, sizeof(tx));
     tx.modes = ADJ_FREQUENCY;
     tx.freq = 655360;  /* +10 ppm, in ppm scaled by 65536 */
     ioctl(fd, PTP_CLOCK_ADJTIME, &tx);

``PTP_CLOCK_GETCAPS``
  Get the capabilities of the PTP clock.

  .. code-block:: c

     struct ptp_clock_caps caps;
     ioctl(fd, PTP_CLOCK_GETCAPS, &caps);
     printf("Max adjustment: %d PPB\n", caps.max_adj);

``PTP_SYS_OFFSET``
  Measure the offset between the PTP clock and system time.

  .. code-block:: c

     struct ptp_sys_offset offset;
     offset.n_samples = 10;
     ioctl(fd, PTP_SYS_OFFSET, &offset);

``PTP_SYS_OFFSET_PRECISE``
  Get precise system-device cross-timestamp.

  .. code-block:: c

     struct ptp_sys_offset_precise precise;
     ioctl(fd, PTP_SYS_OFFSET_PRECISE, &precise);

``PTP_SYS_OFFSET_EXTENDED``
  Like ``PTP_SYS_OFFSET``, but returns for each sample the system time before
  and after reading the device time (``struct ptp_sys_offset_extended``).

``PTP_CLOCK_GETSTATS`` and ``PTP_CLOCK_SETSTATS``
  Read or set the drift statistics kept by the upper half
  (``struct ptp_statistics_s``).

The ``PTP_CLOCK_GETCAPS``, ``PTP_SYS_OFFSET``, ``PTP_SYS_OFFSET_PRECISE`` and
``PTP_SYS_OFFSET_EXTENDED`` commands also exist with a ``2`` suffix
(``PTP_CLOCK_GETCAPS2``, ``PTP_SYS_OFFSET2`` and so on), which the upper half
handles the same way. The time values of ``PTP_CLOCK_GETTIME``,
``PTP_CLOCK_SETTIME`` and ``PTP_CLOCK_GETRES`` are ``struct timespec``, while
``struct ptp_clock_time`` is used by the ``PTP_SYS_OFFSET`` family.

POSIX Clock API (CLOCK_FD)
==========================

NuttX allows PTP clocks to be accessed through the standard POSIX clock APIs.
This provides a more familiar interface for applications already using
``clock_gettime()``, ``clock_settime()``, ``clock_getres()`` and
``clock_adjtime()``.

The clock identifier is built from the file descriptor of the opened device
with the ``CLOCK_SHIFT`` and ``CLOCK_FD`` definitions of
``<nuttx/clock.h>``:

.. code-block:: c

   #include <time.h>
   #include <fcntl.h>
   #include <sys/timex.h>
   #include <nuttx/clock.h>

   int fd = open("/dev/ptp0", O_RDWR);
   clockid_t clockid = (fd << CLOCK_SHIFT) | CLOCK_FD;
   struct timespec ts;

   /* Get PTP clock time using POSIX API */
   clock_gettime(clockid, &ts);

   /* Set PTP clock time */
   clock_settime(clockid, &ts);

   /* Get PTP clock resolution */
   struct timespec res;
   clock_getres(clockid, &res);

   /* Adjust PTP clock frequency */
   struct timex tx = {0};
   tx.modes = ADJ_FREQUENCY;
   tx.freq = -327680;  /* -5 ppm, in ppm scaled by 65536 */
   clock_adjtime(clockid, &tx);

   close(fd);

The clock identifier is only valid while the file descriptor is open.

Supported Adjustment Modes
---------------------------

For a PTP clock, ``clock_adjtime()`` handles one of the following modes per
call, checked in this order:

- ``ADJ_SETOFFSET``: step the clock by the offset in ``tx.time``. The
  ``tv_usec`` field holds microseconds, or nanoseconds if ``ADJ_NANO`` is also
  set. A value of one second or more in that field is rejected with
  ``-EINVAL``.
- ``ADJ_FREQUENCY``: set the frequency offset from ``tx.freq``, in parts per
  million with 16 fractional bits (ppm multiplied by 65536, so +10 ppm is
  655360). A value beyond the maximum adjustment of the clock returns
  ``-ERANGE``. This needs the ``adjfine`` operation of the driver.
- ``ADJ_OFFSET``: adjust the **phase** by ``tx.offset``, in microseconds, or in
  nanoseconds if ``ADJ_NANO`` is also set. This needs the ``adjphase``
  operation of the driver.
- No mode (``tx.modes`` equal to zero): read back the last frequency set in
  ``tx.freq``.

``ADJ_NANO`` only selects the unit of the two time based modes above.
``ADJ_MAXERROR``, ``ADJ_ESTERROR``, ``ADJ_STATUS`` and ``ADJ_TIMECONST`` are not
handled for a PTP clock and make the call return ``-ENOTSUP``.

Dummy PTP Clock Driver
=======================

NuttX provides a software-based dummy PTP clock driver for testing and
development purposes. This driver can be used on platforms without hardware
PTP support.

Features
--------

- Software-based PTP clock using system monotonic clock
- Supports all standard PTP clock operations
- Frequency adjustment simulation
- Time offset adjustment
- Suitable for testing PTP applications without hardware

Initialization
--------------

The dummy driver is automatically initialized when ``CONFIG_PTP_CLOCK_DUMMY``
is enabled. It creates a ``/dev/ptp0`` device node on system startup.

Example Usage
=============

Basic Time Operations
---------------------

.. code-block:: c

   #include <stdio.h>
   #include <fcntl.h>
   #include <time.h>
   #include <nuttx/clock.h>

   int main(void)
   {
     int fd;
     clockid_t clockid;
     struct timespec ts;
     struct timespec res;

     /* Open PTP clock device */
     fd = open("/dev/ptp0", O_RDWR);
     if (fd < 0)
       {
         perror("Failed to open PTP clock");
         return -1;
       }

     clockid = (fd << CLOCK_SHIFT) | CLOCK_FD;

     /* Get current PTP clock time */
     if (clock_gettime(clockid, &ts) == 0)
       {
         printf("PTP time: %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
       }

     /* Get PTP clock resolution */
     if (clock_getres(clockid, &res) == 0)
       {
         printf("PTP resolution: %ld.%09ld\n", res.tv_sec, res.tv_nsec);
       }

     close(fd);
     return 0;
   }

Frequency Adjustment
--------------------

.. code-block:: c

   #include <stdio.h>
   #include <fcntl.h>
   #include <string.h>
   #include <sys/timex.h>
   #include <nuttx/clock.h>

   int main(void)
   {
     int fd;
     clockid_t clockid;
     struct timex tx;

     fd = open("/dev/ptp0", O_RDWR);
     if (fd < 0)
       {
         perror("Failed to open PTP clock");
         return -1;
       }

     clockid = (fd << CLOCK_SHIFT) | CLOCK_FD;

     /* Adjust frequency by +10 ppm */
     memset(&tx, 0, sizeof(tx));
     tx.modes = ADJ_FREQUENCY;
     tx.freq = 655360;  /* 10 ppm in scaled ppm (65536 * ppm) */

     if (clock_adjtime(clockid, &tx) == 0)
       {
         printf("Frequency adjusted successfully\n");
       }
     else
       {
         perror("Failed to adjust frequency");
       }

     close(fd);
     return 0;
   }

Time Offset Adjustment
----------------------

.. code-block:: c

   #include <stdio.h>
   #include <fcntl.h>
   #include <string.h>
   #include <sys/timex.h>
   #include <nuttx/clock.h>

   int main(void)
   {
     int fd;
     clockid_t clockid;
     struct timex tx;

     fd = open("/dev/ptp0", O_RDWR);
     if (fd < 0)
       {
         perror("Failed to open PTP clock");
         return -1;
       }

     clockid = (fd << CLOCK_SHIFT) | CLOCK_FD;

     /* Apply time offset: +1 second */
     memset(&tx, 0, sizeof(tx));
     tx.modes = ADJ_SETOFFSET | ADJ_NANO;
     tx.time.tv_sec = 1;
     tx.time.tv_usec = 0;  /* tv_usec holds nanoseconds when ADJ_NANO is set */

     if (clock_adjtime(clockid, &tx) == 0)
       {
         printf("Time offset applied successfully\n");
       }
     else
       {
         perror("Failed to apply time offset");
       }

     close(fd);
     return 0;
   }

Implementing a Lower-Half Driver
=================================

To implement a hardware-specific PTP clock driver, create a lower-half driver
that embeds a ``struct ptp_lowerhalf_s`` and provides the operations of
``struct ptp_ops_s`` described above:

.. code-block:: c

   #include <nuttx/timers/ptp_clock.h>

   /* Hardware-specific state */
   struct my_ptp_lowerhalf_s
   {
     struct ptp_lowerhalf_s base;  /* Must be first */
     /* Hardware-specific fields */
     uint32_t hw_base_addr;
     /* ... */
   };

   /* Implement the operations */
   static int my_ptp_adjfine(FAR struct ptp_lowerhalf_s *lower, long ppb)
   {
     FAR struct my_ptp_lowerhalf_s *priv =
       (FAR struct my_ptp_lowerhalf_s *)lower;

     /* Adjust hardware clock frequency by ppb parts per billion */
     /* ... hardware-specific code ... */

     return OK;
   }

   static int my_ptp_gettime(FAR struct ptp_lowerhalf_s *lower,
                             FAR struct timespec *ts,
                             FAR struct ptp_system_timestamp *sts)
   {
     FAR struct my_ptp_lowerhalf_s *priv =
       (FAR struct my_ptp_lowerhalf_s *)lower;

     /* Read time from hardware */
     /* ... hardware-specific code ... */

     return OK;
   }

   /* Define operations structure. Operations that the hardware does not
    * support are left out (NULL), except adjtime.
    */
   static const struct ptp_ops_s g_my_ptp_ops =
   {
     .adjfine = my_ptp_adjfine,
     .adjtime = my_ptp_adjtime,
     .gettime = my_ptp_gettime,
     .settime = my_ptp_settime,
     .getres  = my_ptp_getres,
   };

   /* Registration function */
   int my_ptp_register(void)
   {
     FAR struct my_ptp_lowerhalf_s *priv;

     priv = kmm_zalloc(sizeof(struct my_ptp_lowerhalf_s));
     if (priv == NULL)
       {
         return -ENOMEM;
       }

     priv->base.ops = &g_my_ptp_ops;

     /* Initialize hardware */
     /* ... */

     /* Register as /dev/ptp0, with a maximum adjustment of 500 ppm
      * (500000 ppb).
      */

     return ptp_clock_register(&priv->base, 500000, 0);
   }

Existing Implementations
========================

The following lower-half drivers are available in the tree:

- ``drivers/timers/ptp_clock_dummy.c`` - The dummy driver described above,
  enabled with ``CONFIG_PTP_CLOCK_DUMMY``. It is registered as ``/dev/ptp0``
  at start-up.
- ``arch/arm/src/common/stm32/stm32_eth_m3m4_v1.c`` - The Ethernet MAC of the
  STM32 (the legacy driver), registered with ``CONFIG_PTP_CLOCK`` and
  ``CONFIG_STM32_ETH_PTP`` as ``/dev/ptpN``, where N is the number of the
  Ethernet interface. The clock is the PTP counter of the MAC. It is also the
  time base of the hardware timestamps of the received packets, so those are
  not values of ``CLOCK_REALTIME``.

Both use the device number 0 by default, so they cannot be registered at the
same time.

Integration with PTP Daemon
===========================

The PTP clock framework mirrors the PTP hardware clock interface of Linux
(``/dev/ptpN`` and ``clock_adjtime()``), so software written for that
interface is easier to port.

The PTP daemon of NuttX is ``ptpd`` (``apps/netutils/ptpd``, see
:doc:`/applications/system/ptpd/index`). It uses a PTP clock through the
clock identifier described above, when it is started with the device path, for
example ``ptpd -p /dev/ptp0``. The ``ptp4l`` and ``timemaster`` programs of
the linuxptp project are Linux programs and are not part of NuttX.

Performance Considerations
==========================

Hardware Timestamping
---------------------

For best synchronization accuracy (sub-microsecond), PTP clocks should support
hardware timestamping of network packets. This requires coordination between
the PTP clock driver and network interface driver.

Cross-Timestamping
------------------

The ``getcrosststamp()`` operation provides synchronized capture of both the
PTP clock and system time, which is essential for:

- Accurate offset measurements
- System time synchronization from PTP clock
- Minimizing measurement errors

Frequency Adjustment Resolution
--------------------------------

The frequency adjustment resolution depends on hardware capabilities. Most
hardware supports adjustments in the range of:

- Maximum: ±500 to ±1000 parts per million (PPM)
- Resolution: Better than 1 part per billion (PPB)

Debugging
=========

Debug output can be enabled with ``CONFIG_DEBUG_PTP`` and the following
options, which send the messages to the SYSLOG:

- ``CONFIG_DEBUG_PTP_ERROR``: Error messages
- ``CONFIG_DEBUG_PTP_WARN``: Warning messages
- ``CONFIG_DEBUG_PTP_INFO``: Informational messages

References
==========

- IEEE 1588-2008: IEEE Standard for a Precision Clock Synchronization Protocol
  for Networked Measurement and Control Systems

- `Linux PTP Project <https://linuxptp.sourceforge.net/>`_

- ``include/nuttx/timers/ptp_clock.h`` - PTP clock header file
- ``include/nuttx/clock.h`` - ``CLOCK_FD`` and ``CLOCK_SHIFT`` definitions
- ``drivers/timers/ptp_clock.c`` - Upper-half driver implementation
- ``drivers/timers/ptp_clock_dummy.c`` - Dummy driver implementation
