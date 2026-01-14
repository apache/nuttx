==================
I2C Device Drivers
==================

-  ``include/nuttx/i2c/i2c_master.h``
   All structures and APIs needed to work with I2C drivers are provided in
   this header file.

-  ``struct i2c_ops_s``. Each I2C device driver must implement
   an instance of ``struct i2c_ops_s``. That structure defines a
   call table with the following methods:

-  **Binding I2C Drivers**. I2C drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. In general, the binding sequence is:

   #. Get an instance of ``struct i2c_master_s`` from the
      hardware-specific I2C device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``arch/z80/src/ez80/ez80_i2c.c``,
   ``arch/z80/src/z8/z8_i2c.c``, etc.


=======================
I2C Bit-Bang Driver
=======================

The I2C bit-bang driver provides a software implementation of the I2C
protocol using GPIO pins. This is useful when hardware I2C peripherals
are not available or when additional I2C buses are needed.

Overview
--------

-  ``include/nuttx/i2c/i2c_bitbang.h``
   Generic upper-half I2C bit-bang driver interface.

-  ``drivers/i2c/i2c_bitbang.c``
   Generic upper-half implementation that handles the I2C protocol timing.

-  Platform-specific lower-half drivers control the GPIO pins (SDA and SCL).

IO Expander-Based I2C Bit-Bang
-------------------------------

A generic lower-half implementation is provided for systems using IO expanders
to control GPIO pins. This eliminates the need for platform-specific code when
implementing I2C bit-bang via IO expander pins.

Configuration
~~~~~~~~~~~~~

-  ``CONFIG_I2C_BITBANG`` - Enable I2C bit-bang driver framework
-  ``CONFIG_I2C_BITBANG_IOEXPANDER`` - Enable IO expander-based lower-half
   (depends on ``CONFIG_IOEXPANDER``)

Header Files
~~~~~~~~~~~~

-  ``include/nuttx/i2c/i2c_bitbang_ioexpander.h``
   IO expander-based lower-half driver interface.

API
~~~

.. c:function:: FAR struct i2c_master_s *i2c_bitbang_ioexpander_initialize(FAR struct ioexpander_dev_s *ioe, int scl_pin, int sda_pin, int busnum);

   Initialize an I2C bit-bang driver using IO expander pins.

   :param ioe: Pointer to the IO expander device
   :param scl_pin: IO expander pin number for SCL (clock line)
   :param sda_pin: IO expander pin number for SDA (data line)
   :param busnum: I2C bus number to register (use negative value to skip registration)
   
   :return: Pointer to ``struct i2c_master_s`` on success, NULL on failure

   The pins will be configured as open-drain outputs, which is required
   for proper I2C operation. If busnum >= 0, the I2C bus is automatically
   registered and accessible via standard I2C APIs.

Usage Example
~~~~~~~~~~~~~

.. code-block:: c

   #include <nuttx/ioexpander/ioexpander.h>
   #include <nuttx/i2c/i2c_bitbang_ioexpander.h>
   #include <nuttx/i2c/i2c_master.h>

   /* Assume we have an IO expander device */
   FAR struct ioexpander_dev_s *ioe = /* ... get IO expander ... */;
   FAR struct i2c_master_s *i2c;

   /* Initialize I2C bit-bang using IO expander pins 10 (SCL) and 11 (SDA) */
   /* Register as I2C bus 0 */
   i2c = i2c_bitbang_ioexpander_initialize(ioe, 10, 11, 0);
   if (i2c == NULL)
     {
       /* Initialization failed */
       return -1;
     }

   /* Now use the I2C master device normally */
   /* For example, with I2C character driver or directly */

   /* If registered (busnum >= 0), can also access via /dev/i2c0 */

Use Cases
~~~~~~~~~

-  **GPIO Expansion**: When using I2C or SPI IO expanders for GPIO expansion,
   and need to implement additional I2C buses using those expanded pins.

-  **Multi-Master Scenarios**: Software bit-bang can be useful in multi-master
   configurations where hardware I2C has limitations.

-  **Pin Flexibility**: Implement I2C on any GPIO pins, not limited to
   hardware I2C peripheral pins.

-  **Testing and Debugging**: Use IO expander pins for I2C communication
   during prototyping and debugging.

-  **Hardware I2C Unavailable**: When hardware I2C peripherals are exhausted
   or not available on specific pins.

Features
~~~~~~~~

-  Uses standard IO expander API (``IOEXP_WRITEPIN``, ``IOEXP_READPIN``)
-  Automatic open-drain configuration
-  Supports clock stretching (via pin reading)
-  Platform-independent implementation
-  Works with any IO expander that implements the standard interface
-  Automatic I2C bus registration

Limitations
~~~~~~~~~~~

-  Software timing (slower than hardware I2C)
-  Timing accuracy depends on system load and IO expander response time
-  Limited to standard I2C speeds (fast-mode and high-speed may not be reliable)

Implementation Details
~~~~~~~~~~~~~~~~~~~~~~

The IO expander-based implementation provides these callbacks:

-  ``initialize``: Configures SCL and SDA pins as open-drain outputs
-  ``set_scl/set_sda``: Controls pin output values
-  ``get_scl/get_sda``: Reads current pin states (for clock stretching detection)

The driver automatically manages the IO expander pin states and handles
the bit-bang protocol timing according to I2C specifications.


========================
I2C Slave Device Drivers
========================

-  ``include/nuttx/i2c/i2c_slave.h``
   Declaration of all macros and ops and ``int i2c_slave_register``,
   used to bind the lowerhalf driver to the upper one and register an
   I2C slave device.

-  **Binding I2C Slave Drivers**. Use ``int i2c_slave_register`` in your BSP
   to register your slave device. Before that, you need to get the instance
   of ``struct i2c_slave_s`` from the hardware-specific I2C Slave driver.

-  **Using I2C Slave in your Application**. I2C slave drivers are normally directly
   accessed by user code, We can read and write to device nodes using posix
   interfaces. The device is registered as ``/dev/i2cslv%d``, where ``%d``
   is a number provided in the BSP initialization phase.

-  **BSP initialization example (STM32 I2C Slave)**:

.. code-block:: c

   int stm32_i2cs_setup(void)
   {
      i2cs = stm32_i2cbus_slaveinitialize(1);
      if (i2cs != NULL)
        {
          return -ENOENT;
        }
      return i2c_slave_register(i2cs, 1, 0x01, 7);
   }

I2C Slave Driver API
--------------------

.. c:function:: int i2c_slave_register(FAR struct i2c_slave_s *dev, int bus, int addr, int nbit);

   Bind the lowerhalf ``dev`` driver to the upperhalf driver and register
   a device. The name of the device is ``/dev/i2cslv%d``, where ``%d`` is
   bus. The address the I2C slave is specified in ``addr``.
   ``nbit`` is either 7 or 10 and it specifies the address format.


-  ``struct i2c_slaveops_s``. Each I2C slave lowerhalf driver must implement
   an instance of ``struct i2c_slaveops_s``. That structure defines a call
   table with the following methods:

   - ``setownaddress``: the address the slave responds to,
   - ``write``: sets the tx buffer pointer,
   - ``read``: sets the rx buffer pointer,
   - ``registercallback``: registers the callback function which should
     be called from a service routine. Signals the received or fully transferred
     I2C packet.
   - ``setup``: initializes the peripheral,
   - ``shutdown``: shutdowns the peripheral.
