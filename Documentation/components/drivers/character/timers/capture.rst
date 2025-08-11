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

.. c:macro:: CAPIOC_FREQUENCE

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

Configuration
-------------

To enable the capture driver, enable the following configuration options:

* ``CONFIG_CAPTURE``
* ``CONFIG_STM32H7_TIM4_CAP`` (for STM32H7 Timer 4)

The ``CONFIG_CAPTURE`` option enables the lower-half driver and registers
the ``/dev/capture`` device.

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

      if (ioctl(fd, CAPIOC_FREQUENCE, (unsigned long)&frequency) < 0)
        {
          perror("Failed to get frequency");
          close(fd);
          return 1;
        }

      printf("Frequency: %u Hz\n", frequency);

      close(fd);
      return 0;
    }

Notes
-----

* The actual set of supported ``ioctl`` commands may vary depending on
  the hardware and driver implementation.
* The ``CAPIOC_FREQUENCE`` macro name is preserved for compatibility,
  even though "frequency" is the correct English spelling.
* Always check return values from ``ioctl()`` calls for error handling.
* **Important:** In debug builds of NuttX, calling an unsupported
  ``ioctl`` command will trigger a ``DEBUGASSERT`` in the driver,
  which will halt or crash the system.
