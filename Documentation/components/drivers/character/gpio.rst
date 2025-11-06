============
GPIO Drivers
============

-  ``include/nuttx/ioexpander/gpio.h``. All structures and APIs needed
   to work with GPIO pins as drivers are provided in this header file. This
   header file includes:

   #. Structures and interface descriptions needed to develop a
      low-level, board-specific, GPIO driver.
   #. To register the GPIO driver with a common GPIO character
      driver.
   #. Interfaces needed for interfacing user programs with the
      common GPIO character driver.

-  ``drivers/ioexpander/gpio.c``. The implementation of the common GPIO
   character driver.

Application Programming Interface
=================================

The Application Programming Interface is included in the application with
the following header file.


.. code-block:: c

  #include <nuttx/ioexpander/gpio.h>

A GPIO pin is either an input pin or an output pin.

One GPIO pin is registered as a POSIX character device file into ``/dev``
namespace. It is necessary to open the device to get a file descriptor for
further operations. This can be done with standard POSIX ``open()`` call.
Only one pir per driver is supported by the peripheral.

Standard POSIX ``read()`` and ``write()`` operations are not allowed for GPIO
based drivers. All interface is routed through IOCTL calls. Following commands
are supported:

 * :c:macro:`GPIOC_WRITE`
 * :c:macro:`GPIOC_READ`
 * :c:macro:`GPIOC_PINTYPE`
 * :c:macro:`GPIOC_REGISTER`
 * :c:macro:`GPIOC_UNREGISTER`
 * :c:macro:`GPIOC_SETPINTYPE`


.. c:macro:: GPIOC_WRITE

The ``GPIOC_WRITE`` command sets the value of an output GPIO. The argument
is either 0 (set low value) or 1 (set high value). It is possible to write
to output pins only. Typical use case is:

.. code-block:: c

  bool value = true;
  int ret = ioctl(fd, GPIOC_WRITE, value);


.. c:macro:: GPIOC_READ

The ``GPIOC_READ`` command reads the value of a GPIO. The argument
is a pointer to a bool value to receive the result. The result is either 0
(low value) or 1 (high value). It is possible to read from both input and
output pins. The currently set value is returned in case the pin is output
pin. Typical use case is:

.. code-block:: c

  bool value;
  int ret = ioctl(fd, GPIOC_READ, (unsigned long)(uintptr_t)&value);

.. c:macro:: GPIOC_PINTYPE

The ``GPIOC_PINTYPE`` command gets the type of GPIO pin. The argument
is a pointer to an instance of type :c:enum:`gpio_pintype_e`.

.. code-block:: c

  enum gpio_pintype_e
  {
    GPIO_INPUT_PIN = 0, /* float */
    GPIO_INPUT_PIN_PULLUP,
    GPIO_INPUT_PIN_PULLDOWN,
    GPIO_OUTPUT_PIN, /* push-pull */
    GPIO_OUTPUT_PIN_OPENDRAIN,
    GPIO_INTERRUPT_PIN,
    GPIO_INTERRUPT_HIGH_PIN,
    GPIO_INTERRUPT_LOW_PIN,
    GPIO_INTERRUPT_RISING_PIN,
    GPIO_INTERRUPT_FALLING_PIN,
    GPIO_INTERRUPT_BOTH_PIN,
    GPIO_INTERRUPT_PIN_WAKEUP,
    GPIO_INTERRUPT_HIGH_PIN_WAKEUP,
    GPIO_INTERRUPT_LOW_PIN_WAKEUP,
    GPIO_INTERRUPT_RISING_PIN_WAKEUP,
    GPIO_INTERRUPT_FALLING_PIN_WAKEUP,
    GPIO_INTERRUPT_BOTH_PIN_WAKEUP,
    GPIO_NPINTYPES
  };

.. c:macro:: GPIOC_REGISTER

The ``GPIOC_REGISTER`` command registers a pin to receive a signal whenever
there is an interrupt received on an input GPIO pin. This feature, of course,
depends upon interript GPIO support in the platform specific code. Please
refer to the documentation describing your target platform for futher
information. The argument is the pointer to :c:type:`sigevent` value, a signal
to be generated when the interrupt occurs.

Typical use case is following:

.. code-block:: c

  struct sigevent notify;

  notify.sigev_notify = SIGEV_SIGNAL;
  notify.sigev_signo = SIGUSR1;

  int ret = ioctl(fd, GPIOC_REGISTER, (unsigned long)&notify);

.. c:macro:: GPIOC_UNREGISTER

The ``GPIOC_UNREGISTER`` command unresigters a pin and stop receiving signals
for pin interrupt.

.. c:macro:: GPIOC_SETPINTYPE

The ``GPIOC_SETPINTYPE`` command can be used to change the GPIO pin type
(from input pin to output pin, changing interrupt edges and similar). The
types to set are listed in :c:enum:`gpio_pintype_e`.

Application Example
~~~~~~~~~~~~~~~~~~~

An example application can be found in ``nuttx-apps`` repository under
path ``examples/gpio``. It is an example application that allows you
to read, write or configure GPIO pins.

Configuration
=============

This section describes GPIO driver configuration in ``Kconfig``. The reader
should refer to target documentation for target specific configuration.

GPIO peripheral is enabled by ``CONFIG_DEV_GPIO``. Option
``CONFIG_DEV_NPOLLWAITERS`` is used to specify the maximum number of threads
that can be waiting on poll with default set to one. It is also possible
to register signals with the GPIO driver. The number of allowed signals
is configured with ``CONFIG_DEV_NSIGNALS``.

IO Expander Device Drivers
==========================

IO Expander device drivers are chips that provides more GPIO pins, usually
connected to the MCU with SPI or I2C bus. It is possible to register
individual GPIO pins of the expander as a separate pins if needed. This
option is enabled by ``CONFIG_GPIO_LOWER_HALF`` option. Please refer
to `ioexpander documentation <https://nuttx.apache.org/docs/latest/components/drivers/special/ioexpander.html>`_
for more description.
