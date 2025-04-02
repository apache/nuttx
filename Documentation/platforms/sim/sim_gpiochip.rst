======================================
Sim GPIO Chip Driver (Linux Host GPIO)
======================================

Overview
========

The Sim GPIO Chip driver provides a mechanism for NuttX simulation (sim) to access
the Linux host's GPIO chip devices (``/dev/gpiochipN``). This allows NuttX applications
running in simulation mode to interact with real hardware GPIO pins connected to the
Linux host system.

This driver is particularly useful for:

- Testing GPIO-based applications in a simulated environment with real hardware
- Interfacing with USB-to-GPIO adapters (e.g., CH341A) from NuttX simulation
- Developing and debugging GPIO drivers without dedicated embedded hardware

Host Prepare
============

Preparation required on the host side:

- Hardware module required: USB-CH341A module
- Refer to https://github.com/frank-zago/ch341-i2c-spi-gpio, and install the driver
- Verify existence of /dev/gpiochipN device file

Architecture
============

The driver consists of two layers:

1. **NuttX Layer** (``sim_gpiochip.c``): Implements the NuttX ``ioexpander_dev_s``
   interface, providing standard GPIO operations to upper-layer NuttX drivers.

2. **Host Layer** (``sim_linux_gpiochip.c``): Interfaces directly with Linux kernel's
   GPIO character device (``/dev/gpiochipN``) using the GPIO v2 ABI.

::

    +---------------------+
    |  NuttX Application  |
    +---------------------+
            |
            v
    +---------------------+
    |  GPIO Lower Half    |
    |  (gpio_lower_half)  |
    +---------------------+
            |
            v
    +---------------------+
    |   sim_gpiochip.c    |  <-- NuttX ioexpander interface
    | (ioexpander_dev_s)  |
    +---------------------+
            |
            v
    +---------------------+
    | sim_linux_gpiochip.c|  <-- Linux host GPIO interface
    |  (GPIO v2 ABI)      |
    +---------------------+
            |
            v
    +---------------------+
    |  /dev/gpiochipN     |  <-- Linux GPIO character device
    +---------------------+

Header Files
============

-  ``arch/sim/src/sim/sim_hostgpiochip.h``: Host GPIO chip interface definitions
   and function prototypes.

-  ``include/nuttx/ioexpander/ioexpander.h``: Standard NuttX IO expander interface.

-  ``include/nuttx/ioexpander/gpio.h``: NuttX GPIO interface definitions.

Configuration Options
=====================

The following configuration options are relevant to this driver:

- ``CONFIG_SIM_GPIOCHIP``: Enable the sim GPIO chip driver.
- ``CONFIG_IOEXPANDER_NPINS``: Maximum number of GPIO pins supported (default: 64).
- ``CONFIG_IOEXPANDER_INT_ENABLE``: Enable interrupt support for GPIO pins.

Supported Operations
====================

The driver supports the following GPIO operations:

Direction Control
-----------------

.. code-block:: c

   int sim_gpiochip_direction(struct ioexpander_dev_s *dev,
                              uint8_t pin, int direction);

Set GPIO pin direction. Supported directions:

- ``IOEXPANDER_DIRECTION_IN``: Configure as input
- ``IOEXPANDER_DIRECTION_OUT``: Configure as output
- ``IOEXPANDER_DIRECTION_OUT_OPENDRAIN``: Configure as open-drain output

Read/Write Pin
--------------

.. code-block:: c

   int sim_gpiochip_readpin(struct ioexpander_dev_s *dev, uint8_t pin,
                            bool *value);
   int sim_gpiochip_writepin(struct ioexpander_dev_s *dev, uint8_t pin,
                             bool value);

Read or write the value of a GPIO pin.

Interrupt Configuration
-----------------------

.. code-block:: c

   int sim_gpiochip_option(struct ioexpander_dev_s *dev, uint8_t pin,
                           int option, void *val);

Configure GPIO pin options. Supported interrupt edge configurations:

- ``IOEXPANDER_VAL_RISING``: Trigger on rising edge
- ``IOEXPANDER_VAL_FALLING``: Trigger on falling edge
- ``IOEXPANDER_VAL_BOTH``: Trigger on both edges
- ``IOEXPANDER_VAL_DISABLE``: Disable interrupt

Interrupt Callback
------------------

.. code-block:: c

   void *sim_gpiochip_attach(struct ioexpander_dev_s *dev,
                             ioe_pinset_t pinset,
                             ioe_callback_t callback,
                             void *arg);
   int sim_gpiochip_detach(struct ioexpander_dev_s *dev, void *handle);

Attach or detach an interrupt callback function for GPIO pins.

Host Layer API
==============

The host layer (``sim_linux_gpiochip.c``) provides the following functions:

.. code-block:: c

   /* Allocate and initialize a host GPIO chip device */
   struct host_gpiochip_dev *host_gpiochip_alloc(const char *filename);

   /* Free a host GPIO chip device */
   void host_gpiochip_free(struct host_gpiochip_dev *dev);

   /* Set GPIO pin direction */
   int host_gpiochip_direction(struct host_gpiochip_dev *dev,
                               uint8_t pin, bool input);

   /* Read GPIO pin value */
   int host_gpiochip_readpin(struct host_gpiochip_dev *dev,
                             uint8_t pin, bool *value);

   /* Write GPIO pin value */
   int host_gpiochip_writepin(struct host_gpiochip_dev *dev,
                              uint8_t pin, bool value);

   /* Request GPIO interrupt */
   int host_gpiochip_irq_request(struct host_gpiochip_dev *dev,
                                 uint8_t pin, uint16_t cfgset);

   /* Check if GPIO interrupt is active */
   bool host_gpiochip_irq_active(struct host_gpiochip_dev *dev, uint8_t pin);

   /* Get GPIO line information */
   int host_gpiochip_get_line(struct host_gpiochip_dev *priv,
                              uint8_t pin, bool *input);

Linux Kernel Version Requirements
=================================

The driver uses Linux GPIO v2 ABI, which requires:

- **Linux kernel >= 6.8.0**: Full functionality with GPIO v2 API support.
- **Linux kernel < 6.8.0**: The driver compiles but provides stub implementations
  that return 0 or NULL.

Usage Example
=============

Initialization
--------------

.. code-block:: c

   #include <nuttx/ioexpander/gpio.h>
   #include "sim_internal.h"

   int board_gpio_initialize(void)
   {
     struct ioexpander_dev_s *ioe;
     int ret;

     /* Initialize the GPIO chip device */
     ioe = sim_gpiochip_initialize("/dev/gpiochip0");
     if (ioe == NULL)
       {
         return -ENODEV;
       }

     /* Register GPIO pins using gpio_lower_half */
     ret = gpio_lower_half(ioe, 0, GPIO_INPUT_PIN, 60);  /* Pin 0 as input, minor 60 */
     if (ret < 0)
       {
         return ret;
       }

     ret = gpio_lower_half(ioe, 1, GPIO_OUTPUT_PIN, 61); /* Pin 1 as output, minor 61 */
     if (ret < 0)
       {
         return ret;
       }

     return OK;
   }

Application Usage
-----------------

After initialization, GPIO pins can be accessed through standard NuttX GPIO interface:

.. code-block:: c

   #include <fcntl.h>
   #include <sys/ioctl.h>
   #include <nuttx/ioexpander/gpio.h>

   int main(void)
   {
     int fd;
     bool value;

     /* Open GPIO device */
     fd = open("/dev/gpio60", O_RDWR);
     if (fd < 0)
       {
         return -1;
       }

     /* Read GPIO value */
     ioctl(fd, GPIOC_READ, &value);
     printf("GPIO value: %d\n", value);

     close(fd);
     return 0;
   }

Interrupt Handling
==================

The driver uses a work queue to poll for GPIO events. The polling interval is
defined by ``SIM_GPIOCHIP_WORK_DELAY`` (default: 500 microseconds).

When an interrupt event is detected on a GPIO pin, the registered callback
function is invoked with the pin number and user-provided argument.

.. code-block:: c

   static int gpio_interrupt_handler(struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, void *arg)
   {
     printf("GPIO interrupt on pin %d\n", pinset);
     return OK;
   }

   /* Attach interrupt handler */
   void *handle = IOEP_ATTACH(ioe, (1 << pin), gpio_interrupt_handler, NULL);

   /* Configure interrupt edge */
   IOEP_SETOPTION(ioe, pin, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_RISING);

Files
=====

-  ``arch/sim/src/sim/sim_gpiochip.c``: NuttX IO expander implementation
-  ``arch/sim/src/sim/posix/sim_linux_gpiochip.c``: Linux host GPIO interface
-  ``arch/sim/src/sim/sim_hostgpiochip.h``: Host GPIO chip header file

Limitations
===========

1. **Polling-based interrupts**: Due to simulation constraints, interrupts are
   implemented using polling rather than true hardware interrupts.

2. **Linux kernel version**: Full functionality requires Linux kernel >= 6.8.0.

3. **Pin count**: Limited by ``CONFIG_IOEXPANDER_NPINS`` configuration.

4. **Invert option**: The ``IOEXPANDER_OPTION_INVERT`` option is not yet implemented.

See Also
========

-  Linux GPIO documentation: https://www.kernel.org/doc/html/latest/driver-api/gpio/
