==========================
IO Expander Device Drivers
==========================

-  ``include/nuttx/ioexpander/ioexpander.h`` and ``include/nuttx/ioexpander/gpio.h``.
   All structures and APIs needed to work with ioexpander drivers are provided in
   this header file.

-  ``struct ioexpander_ops_s``. Each ioexpand device driver must implement
   an instance of ``struct ioexpander_ops_s``. That structure defines a
   call table with the methods, and we also provide macros to help access methods.

-  we also provide method ``gpio_lower_half`` to make ioexpander compatible with normal gpio.

-  **Binding ioexpander Drivers**. ioexpander drivers are not normally directly
   accessed by user code, we should always get lower level drivers, for example I2C,
   and map extended gpio feature same asa normal gpio. See for example,
   ``int nrf52_sx1509_initialize(void)``
   in ``boards/arm/nrf52/thingy52/src/nrf52_sx1509.c``. In general, the binding
   sequence is:

   #. Get an instance of ``struct i2c_master_s`` from the
      hardware-specific I2C device driver, and
   #. Provide that instance and configurations to the ioexpander initialization method
      to get the ``struct ioexpander_dev_s`` ioe device instance.
   #. Then use ioe device instance to do ioexpander operations, or use ``gpio_lower_half``
      to make ioexpand compatible with normal gpio.


-  **Examples**: ``drivers/ioexpander/pca9555.c``,
   ``drivers/input/aw86225.c``,
   ``drivers/analog/lmp92001.c``,
   ``drivers/ioexpander/ioe_rpmsg.c``,
   ``boards/sim/sim/sim/src/sim_ioexpander.c``,
   ``boards/arm/nrf52/thingy52/src/nrf52_sx1509.c`` etc.
