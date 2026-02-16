=======
Capture
=======

The **capture driver** is a character device driver that allows capturing
timer values on specific events. This is useful for tasks such as
measuring the frequency, duty cycle, or pulse count of an input signal.

This documentation is based on the STM32H7 timer capture driver.

Usage
=====

The capture driver is accessed via a device file (e.g., ``/dev/capture0``).
You can use standard file operations along with ``ioctl()`` calls to
retrieve captured values or configure the driver.

Supported ``ioctl`` Commands
----------------------------

.. c:macro:: CAPIOC_DUTYCYCLE

   Get the PWM duty cycle from the capture unit.

   **Argument:** ``int8_t *`` (pointer to duty cycle percentage).

.. c:macro:: CAPIOC_FREQUENCY

   Get the pulse frequency from the capture unit.

   **Argument:** ``int32_t *`` (pointer to frequency in Hz).

.. c:macro:: CAPIOC_EDGES

   Get the number of PWM edges detected.

   **Argument:** ``int32_t *`` (pointer to edge count).

.. c:macro:: CAPIOC_ALL

   Get duty cycle, pulse frequency, and edge count in a single call.

   **Argument:** ``struct cap_all_s *`` (structure containing all values).

.. c:macro:: CAPIOC_PULSES

   Read the current pulse count value.

   **Argument:** ``int *`` (pointer to pulse count).

.. c:macro:: CAPIOC_CLR_CNT

   Clear the pulse count value.

   **Argument:** None.

.. c:macro:: CAPIOC_FILTER

   Configure the glitch filter.

   **Argument:** ``uint32_t`` (filter value in nanoseconds, ``0`` to disable).

.. c:macro:: CAPIOC_HANDLER

   Set a user callback function for capture events.

   **Argument:** ``xcpt_t`` (function pointer, ``NULL`` to disable).

.. c:macro:: CAPIOC_ADD_WP

   Add a watchpoint to the capture unit.

   **Argument:** ``int`` (value to watch for).

.. c:macro:: CAPIOC_REGISTER

   Register for capture edge event notifications. This allows applications
   to receive asynchronous signal notifications when capture edge events
   occur, instead of polling for events.

   **Argument:** ``struct cap_notify_s *`` (pointer to notification structure).

   The ``struct cap_notify_s`` contains:

   * ``event`` - The signal event configuration (``struct sigevent``)
   * ``chan`` - Capture channel number
   * ``type`` - Edge type (``CAP_TYPE_RISING``, ``CAP_TYPE_FALLING``, or ``CAP_TYPE_BOTH``)
   * ``ptr`` - User data pointer

   **Returns:**

   * ``OK`` on success
   * ``-EINVAL`` for invalid channel
   * ``-EBUSY`` if channel already registered by another task

.. c:macro:: CAPIOC_UNREGISTER

   Unregister capture edge event notifications.

   **Argument:** ``int`` (channel number).

   **Returns:** ``OK`` on success.

Configuration
-------------

To enable the capture driver, enable the following configuration options:

* ``CONFIG_CAPTURE`` - Enable the capture driver framework
* ``CONFIG_CAPTURE_NOTIFY`` - Enable signal notification support for edge events
* ``CONFIG_FAKE_CAPTURE`` - Enable fake capture driver for testing (generates 10Hz signal with 50% duty cycle)
* ``CONFIG_STM32H7_TIM4_CAP`` (for STM32H7 Timer 4, platform-specific)

The ``CONFIG_CAPTURE`` option enables the lower-half driver and registers
the ``/dev/capture`` device.

The ``CONFIG_CAPTURE_NOTIFY`` option enables the signal notification feature,
allowing applications to receive asynchronous notifications when capture
edge events occur. This requires hardware support for edge interrupts and
depends on ``CONFIG_CAPTURE``.

The ``CONFIG_FAKE_CAPTURE`` option enables a software-based fake capture
driver that simulates a 10Hz square wave with 50% duty cycle. This is
useful for development and testing without requiring actual hardware.
It depends on ``CONFIG_CAPTURE`` and ``CONFIG_CAPTURE_NSIGNALS > 0``.

Without it, capture is still possible manually by including the appropriate
header (e.g., ``arch/arm/src/stm32h7/stm32_capture.h``) and performing a
manual initialization.

Example
-------

Here is a simple example of using the capture driver to read a signal's
frequency:

.. code-block:: c

    #include <stdio.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <nuttx/timers/capture.h>

    int main(int argc, char *argv[])
    {
      int fd;
      uint32_t frequency;

      fd = open("/dev/capture0", O_RDONLY);
      if (fd < 0)
        {
          perror("Failed to open capture device");
          return 1;
        }

      if (ioctl(fd, CAPIOC_FREQUENCY, (unsigned long)&frequency) < 0)
        {
          perror("Failed to get frequency");
          close(fd);
          return 1;
        }

      printf("Frequency: %u Hz\n", frequency);

      close(fd);
      return 0;
    }

Signal Notification Example
----------------------------

Here is an example using signal notifications for event-driven capture
(requires ``CONFIG_CAPTURE_NOTIFY``):

.. code-block:: c

    #include <stdio.h>
    #include <signal.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <nuttx/timers/capture.h>

    static volatile int edge_count = 0;

    static void capture_handler(int signo, siginfo_t *info, void *context)
    {
      edge_count++;
    }

    int main(int argc, char *argv[])
    {
      int fd;
      struct cap_notify_s notify;
      struct sigaction sa;
      uint32_t frequency;
      uint8_t duty;

      /* Set up signal handler */
      sa.sa_sigaction = capture_handler;
      sa.sa_flags = SA_SIGINFO;
      sigemptyset(&sa.sa_mask);
      sigaction(SIGUSR1, &sa, NULL);

      /* Open capture device */
      fd = open("/dev/capture0", O_RDONLY);
      if (fd < 0)
        {
          perror("Failed to open capture device");
          return 1;
        }

      /* Configure notification for both edges on channel 0 */
      notify.chan = 0;
      notify.type = CAP_TYPE_BOTH;
      notify.event.sigev_notify = SIGEV_SIGNAL;
      notify.event.sigev_signo = SIGUSR1;
      notify.event.sigev_value.sival_ptr = NULL;

      if (ioctl(fd, CAPIOC_REGISTER, (unsigned long)&notify) < 0)
        {
          perror("Failed to register notification");
          close(fd);
          return 1;
        }

      printf("Waiting for capture events...\n");

      /* Wait for some events */
      sleep(2);

      /* Get frequency and duty cycle */
      ioctl(fd, CAPIOC_FREQUENCY, (unsigned long)&frequency);
      ioctl(fd, CAPIOC_DUTYCYCLE, (unsigned long)&duty);

      printf("Captured %d edges\n", edge_count);
      printf("Frequency: %u Hz, Duty: %u%%\n", frequency, duty);

      /* Unregister notification */
      ioctl(fd, CAPIOC_UNREGISTER, 0);

      close(fd);
      return 0;
    }

Fake Capture Testing Example
-----------------------------

The fake capture driver can be used for testing without hardware
(requires ``CONFIG_FAKE_CAPTURE``):

.. code-block:: c

    #include <stdio.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <nuttx/timers/capture.h>

    int main(int argc, char *argv[])
    {
      int fd;
      uint32_t frequency;
      uint8_t duty;

      /* Open fake capture device */
      fd = open("/dev/fake_capture0", O_RDONLY);
      if (fd < 0)
        {
          perror("Failed to open fake capture device");
          return 1;
        }

      /* Start capture */
      ioctl(fd, CAPIOC_START, 0);

      /* Wait for capture to stabilize */
      sleep(1);

      /* Read values (should be 10Hz, 50% duty) */
      ioctl(fd, CAPIOC_FREQUENCY, (unsigned long)&frequency);
      ioctl(fd, CAPIOC_DUTYCYCLE, (unsigned long)&duty);

      printf("Fake Capture - Frequency: %u Hz, Duty: %u%%\n",
             frequency, duty);

      /* Stop capture */
      ioctl(fd, CAPIOC_STOP, 0);

      close(fd);
      return 0;
    }

Notes
-----

* The actual set of supported ``ioctl`` commands may vary depending on
  the hardware and driver implementation.
* The ``CAPIOC_FREQUENCY`` macro name is preserved for compatibility,
  even though "frequency" is the correct English spelling.
* Always check return values from ``ioctl()`` calls for error handling.
* **Important:** In debug builds of NuttX, calling an unsupported
  ``ioctl`` command will trigger a ``DEBUGASSERT`` in the driver,
  which will halt or crash the system.

Signal Notification Features
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When ``CONFIG_CAPTURE_NOTIFY`` is enabled:

* Applications can register for asynchronous edge event notifications
* Supports per-channel registration with independent configurations
* Edge types supported: rising edge (``CAP_TYPE_RISING``), falling edge
  (``CAP_TYPE_FALLING``), or both (``CAP_TYPE_BOTH``)
* Only one task can register per channel at a time
* Signal notifications use standard POSIX ``sigevent`` mechanism
* Lower-half drivers must implement ``bind()`` and ``unbind()`` operations
* Ideal for event-driven applications like tachometers, encoders, and
  frequency counters

Fake Capture Driver
~~~~~~~~~~~~~~~~~~~

The fake capture driver (``CONFIG_FAKE_CAPTURE``) provides:

* Software simulation of capture events using watchdog timers
* Fixed 10Hz frequency with 50% duty cycle
* Edge toggles every 50ms (rising and falling)
* Supports all standard capture operations including notifications
* Available at ``/dev/fake_capture0``, ``/dev/fake_capture1``, etc.
* Useful for development, testing, and CI/CD without hardware
* Platform-independent implementation
* Automatically initialized at boot (2 channels by default)

Limitations:

* Fixed timing parameters (not configurable at runtime)
* Software timing accuracy (not hardware-precise)
* Suitable for functional testing, not timing precision validation
