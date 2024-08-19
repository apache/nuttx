==================
I2C Device Drivers
==================

-  ``include/nuttx/i2c/i2c_master.h`` and ``include/nuttx/i2c/i2c_slave.h``.
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

-  ``struct i2c_slaveops_s``. Each I2C slave device driver must implement
   an instance of ``struct i2c_slaveops_s``. That structure defines a call
   table with the following methods:

-  **Binding I2C Slave Drivers**. I2C slave drivers are normally directly
   accessed by user code, We can read and write to device nodes using posix
   interfaces.
