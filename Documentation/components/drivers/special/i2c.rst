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
