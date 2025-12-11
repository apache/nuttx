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
- Interface to POSIX clock APIs via CLOCKFD mechanism

Lower-Half Driver
-----------------

Hardware-specific drivers implement the ``struct ptp_lowerhalf_s`` interface
with the following operations:

.. code-block:: c

   struct ptp_clock_ops_s
   {
     CODE int (*adjfine)(FAR struct ptp_lowerhalf_s *lower, long scaled_ppm);
     CODE int (*adjtime)(FAR struct ptp_lowerhalf_s *lower, int64_t delta);
     CODE int (*gettime)(FAR struct ptp_lowerhalf_s *lower,
                         FAR struct timespec *ts);
     CODE int (*settime)(FAR struct ptp_lowerhalf_s *lower,
                         FAR const struct timespec *ts);
     CODE int (*getcaps)(FAR struct ptp_lowerhalf_s *lower,
                         FAR struct ptp_clock_caps *caps);
     CODE int (*getcrosststamp)(FAR struct ptp_lowerhalf_s *lower,
                               FAR struct system_device_crosststamp *xts);
   };

Configuration Options
=====================

The PTP clock framework can be enabled with the following Kconfig options:

``CONFIG_PTP_CLOCK``
  Enable PTP clock driver framework support. This provides the upper-half
  driver infrastructure and POSIX clock API integration.

``CONFIG_PTP_CLOCK_DUMMY``
  Enable a software-based dummy PTP clock driver for testing and development.
  This driver provides a PTP clock implementation without hardware support,
  using the system monotonic clock as the time base.

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

     struct ptp_clock_time time;
     ioctl(fd, PTP_CLOCK_GETTIME, &time);

``PTP_CLOCK_SETTIME``
  Set the time of the PTP clock.

  .. code-block:: c

     struct ptp_clock_time time;
     time.sec = 1234567890;
     time.nsec = 123456789;
     ioctl(fd, PTP_CLOCK_SETTIME, &time);

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
     tx.freq = 10000000;  /* +10 PPM */
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

POSIX Clock API (CLOCKFD)
==========================

NuttX implements the CLOCKFD mechanism, allowing PTP clocks to be accessed
through standard POSIX clock APIs. This provides a more familiar interface
for applications already using ``clock_gettime()``, ``clock_settime()``,
``clock_getres()``, and ``clock_adjtime()``.

The CLOCKFD mechanism works by encoding a file descriptor into a clockid_t
value using the ``CLOCKFD()`` macro:

.. code-block:: c

   #include <time.h>
   #include <fcntl.h>
   #include <nuttx/clock.h>

   int fd = open("/dev/ptp0", O_RDONLY);
   struct timespec ts;
   
   /* Get PTP clock time using POSIX API */
   clock_gettime(CLOCKFD(fd), &ts);
   
   /* Set PTP clock time */
   clock_settime(CLOCKFD(fd), &ts);
   
   /* Get PTP clock resolution */
   struct timespec res;
   clock_getres(CLOCKFD(fd), &res);
   
   /* Adjust PTP clock frequency */
   struct timex tx = {0};
   tx.modes = ADJ_FREQUENCY;
   tx.freq = -5000000;  /* -5 PPM */
   clock_adjtime(CLOCKFD(fd), &tx);
   
   close(fd);

Supported Adjustment Modes
---------------------------

The ``clock_adjtime()`` function supports the following adjustment modes
via ``struct timex``:

- ``ADJ_OFFSET``: Apply time offset adjustment
- ``ADJ_FREQUENCY``: Adjust clock frequency in scaled PPM
- ``ADJ_MAXERROR``: Set maximum time error estimate
- ``ADJ_ESTERROR``: Set estimated time error
- ``ADJ_STATUS``: Modify clock status bits
- ``ADJ_TIMECONST``: Set PLL time constant
- ``ADJ_SETOFFSET``: Set absolute time offset (with ``ADJ_NANO`` flag)
- ``ADJ_MICRO``: Interpret time values as microseconds
- ``ADJ_NANO``: Interpret time values as nanoseconds

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
     struct timespec ts;
     struct timespec res;
     
     /* Open PTP clock device */
     fd = open("/dev/ptp0", O_RDWR);
     if (fd < 0)
       {
         perror("Failed to open PTP clock");
         return -1;
       }
     
     /* Get current PTP clock time */
     if (clock_gettime(CLOCKFD(fd), &ts) == 0)
       {
         printf("PTP time: %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
       }
     
     /* Get PTP clock resolution */
     if (clock_getres(CLOCKFD(fd), &res) == 0)
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
     struct timex tx;
     
     fd = open("/dev/ptp0", O_RDWR);
     if (fd < 0)
       {
         perror("Failed to open PTP clock");
         return -1;
       }
     
     /* Adjust frequency by +10 PPM */
     memset(&tx, 0, sizeof(tx));
     tx.modes = ADJ_FREQUENCY;
     tx.freq = 10000000;  /* 10 PPM in scaled PPM (65536 * PPM) */
     
     if (clock_adjtime(CLOCKFD(fd), &tx) == 0)
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
     struct timex tx;
     
     fd = open("/dev/ptp0", O_RDWR);
     if (fd < 0)
       {
         perror("Failed to open PTP clock");
         return -1;
       }
     
     /* Apply time offset: +1 second */
     memset(&tx, 0, sizeof(tx));
     tx.modes = ADJ_SETOFFSET | ADJ_NANO;
     tx.time.tv_sec = 1;
     tx.time.tv_usec = 0;  /* tv_usec holds nanoseconds when ADJ_NANO is set */
     
     if (clock_adjtime(CLOCKFD(fd), &tx) == 0)
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
that implements the ``struct ptp_lowerhalf_s`` interface:

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

   /* Implement required operations */
   static int my_ptp_adjfine(FAR struct ptp_lowerhalf_s *lower,
                            long scaled_ppm)
   {
     FAR struct my_ptp_lowerhalf_s *priv =
       (FAR struct my_ptp_lowerhalf_s *)lower;
     
     /* Adjust hardware clock frequency */
     /* ... hardware-specific code ... */
     
     return OK;
   }

   static int my_ptp_gettime(FAR struct ptp_lowerhalf_s *lower,
                            FAR struct timespec *ts)
   {
     FAR struct my_ptp_lowerhalf_s *priv =
       (FAR struct my_ptp_lowerhalf_s *)lower;
     
     /* Read time from hardware */
     /* ... hardware-specific code ... */
     
     return OK;
   }

   /* Define operations structure */
   static const struct ptp_clock_ops_s g_my_ptp_ops =
   {
     .adjfine       = my_ptp_adjfine,
     .adjtime       = my_ptp_adjtime,
     .gettime       = my_ptp_gettime,
     .settime       = my_ptp_settime,
     .getcaps       = my_ptp_getcaps,
     .getcrosststamp = my_ptp_getcrosststamp,
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
     
     return ptp_clock_register(&priv->base, 0);  /* Register as /dev/ptp0 */
   }

Integration with PTP Daemon
===========================

The PTP clock framework is designed to work with standard PTP synchronization
daemons such as:

- **ptp4l**: IEEE 1588 PTP daemon from the linuxptp project
- **timemaster**: Synchronization manager combining PTP and NTP
- **ptpd**: PTP daemon (IEEE 1588-2008 implementation)

These daemons can use the PTP clock devices through the standard POSIX clock
APIs via the CLOCKFD mechanism, making porting straightforward.

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

Debug output can be enabled using the ``CONFIG_DEBUG_PTPCLK_*`` configuration
options:

- ``CONFIG_DEBUG_PTPCLK_ERROR``: Error messages
- ``CONFIG_DEBUG_PTPCLK_WARN``: Warning messages
- ``CONFIG_DEBUG_PTPCLK_INFO``: Informational messages

Example debug output:

.. code-block:: text

   ptpclk: PTP clock registered as /dev/ptp0
   ptpclk: adjfine: scaled_ppm=656360 (10 PPM)
   ptpclk: gettime: ts=1234567890.123456789

References
==========

- IEEE 1588-2008: IEEE Standard for a Precision Clock Synchronization Protocol
  for Networked Measurement and Control Systems

- `Linux PTP Project <https://linuxptp.sourceforge.net/>`_

- ``include/nuttx/timers/ptp_clock.h`` - PTP clock header file
- ``drivers/timers/ptp_clock.c`` - Upper-half driver implementation
- ``drivers/timers/ptp_clock_dummy.c`` - Dummy driver implementation
