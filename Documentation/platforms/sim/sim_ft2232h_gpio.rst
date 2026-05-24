=====================================
Sim FTDI GPIO Driver (Using libftdi1)
=====================================

Overview
========

The NuttX simulation (sim) already has a GPIO Chip driver that provides a mechanism use GPIO to control external devices. However that solution depends on having a driver to Linux side and that adds more overhead.

This FTDI driver allows a direct use of libftdi1 to control up to 8 GPIOs.

This driver is particularly useful for:

- Testing GPIO-based applications in a simulated environment with real hardware
- Interfacing with USB-to-GPIO adapters from NuttX simulation
- Developing and debugging GPIO drivers without dedicated embedded hardware

Host Prepare
============

Preparation required on the host side:

- Hardware module required: FT2232H module like the CJMCU-2232HL module
- A udev rule on Linux side to avoid using the FT2232H as USB/Serial:

.. code-block:: console

   $ cat /etc/udev/rules.d/99-ft2232h-d2xx.rules 
   ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", ATTRS{product}=="FTDI Device", MODE="0666"

- The libftdi1-dev installed on your system.

Architecture
============

The driver consists of two layers:

1. **NuttX Layer** (``sim_gpiochip.c``): Implements the NuttX ``ioexpander_dev_s``
   interface, providing standard GPIO operations to upper-layer NuttX drivers.

2. **Host Layer** (``sim_ftdi_gpiochip.c``): Interfaces directly with libftdio1 to initialize and to provide the functions to control the GPIOs.

.. uml::

   @startuml
   skinparam componentStyle rectangle
   skinparam defaultFontName Monospaced

   [NuttX Application] as app
   [GPIO Lower Half\n(gpio_lower_half)] as gpio
   [sim_gpiochip.c\n(ioexpander_dev_s)] as ioex
   note right of ioex : NuttX ioexpander interface
   [sim_ftdi_gpiochip.c\n(host interface)] as sim
   note right of sim : Linux host GPIO interface
   [libftdi1] as ftdi
   note right of ftdi : Linux library to control FT2232H

   app --> gpio
   gpio --> ioex
   ioex --> sim
   sim --> ftdi
   @enduml

Header Files
============

-  ``arch/sim/src/sim/sim_gpiochip.h``: Host GPIO chip interface definitions and function prototypes.

-  ``include/nuttx/ioexpander/ioexpander.h``: Standard NuttX IO expander interface.

-  ``include/nuttx/ioexpander/gpio.h``: NuttX GPIO interface definitions.

Configuration Options
=====================

The following configuration options are relevant to this driver:

- ``CONFIG_SIM_GPIOCHIP_FTDI``: Enable the FTDI/FT2232H driver support.

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

Host Layer API
==============

The host layer (``sim_ftdi_gpiochip.c``) provides the following functions:

.. code-block:: c

   /* Allocate and initialize a host GPIO chip device */
   struct host_gpiochip_dev *host_gpiochip_alloc(uint8_t pins_dir);

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
     fd = open("/dev/gpio20", O_RD);
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

Files
=====

-  ``arch/sim/src/sim/sim_gpiochip.c``: NuttX IO expander implementation
-  ``arch/sim/src/sim/posix/sim_ftdi_gpiochip.c``: Linux host GPIO interface
-  ``arch/sim/src/sim/sim_gpiochip.h``: Host GPIO chip header file

Limitations
===========

1. **Polling-based interrupts**: Due to simulation constraints, interrupts are
   implemented using polling rather than true hardware interrupts.

3. **Pin count**: Limited to 8 pins of SYNBB AD0-7.

4. **Invert option**: The ``IOEXPANDER_OPTION_INVERT`` option is not yet implemented.

