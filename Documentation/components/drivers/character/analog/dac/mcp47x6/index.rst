===========================
Microchip MCP4706/4716/4726
===========================

Microchip MCP4706/4716/4726 DAC.

The digital-analog-converter operates over I2C.

-  ``include/nuttx/analog/mcp47x6.h``. All structures and APIs needed
   to work with DAC drivers are provided in this header file.

The following features are configurable via the ``ioctl`` interface of
the device:

- gain
- power down
- voltage reference


Usage Example
-------------

.. code-block:: c

   #include <nuttx/analog/dac.h>
   #include <nuttx/analog/mcp47x6.h>

   struct dac_dev_s *dac;
   unsigned int const i2c_bus = 0;
   unsigned int const i2c_address = 0x63;

   /* create and register device */

   dac = mcp47x6_initialize(i2c_bus, i2c_address);
   dac_register("/dev/dac0", dac);

   /* configure the DAC */

   int fd = open("/dev/dac0", O_WRONLY | O_NONBLOCK);
   ioctl(fd, ANIOC_MCP47X6_DAC_SET_REFERENCE, MCP47X6_REFERENCE_VREF_BUFFERED);

   /* set DAC output value */

   struct dac_msg_s dac_message = {
     .am_channel = 0,
     .am_data = 1234
   };
   write(fd, &dac_message, sizeof(dac_message));

   /* clean up */

   close(fd);
