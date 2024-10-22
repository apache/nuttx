=======
MCP3008
=======

Contributed by Matteo Golin

The MCP3008 is a 10-bit, 8-channel ADC made by Microchip which operates over
SPI.

There is the option to operate in single-ended mode, which measures the voltage
on each channel individually, or differential mode which measures the voltage
difference between pairs of channels.

When operating in differential mode, the channel numbers below correspond to the
listed differential pairs:

.. list-table:: Differential pair channel numbers
   :widths: auto

   * - Channel number
     - Sources
   * - 0                 
     - CH0+, CH1- 
   * - 1                 
     - CH0-, CH1+ 
   * - 2                 
     - CH2+, CH3- 
   * - 3                 
     - CH2-, CH3+ 
   * - 4                 
     - CH4+, CH5- 
   * - 5                 
     - CH4-, CH5+ 
   * - 6                 
     - CH6+, CH7- 
   * - 7                 
     - CH6-, CH7+ 

Driver Interface
---------------------

To register the MCP3008 device driver as a standard NuttX analog device on your
board, you can use something similar to the below code for the RP2040.

.. code-block:: c

  #include <nuttx/analog/mcp3008.h>
  #include <nuttx/analog/adc.h>

  /* Register MCP3008 ADC */

  struct spi_dev_s *spi = rp2040_spibus_initialize(0);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize SPI bus 0\n");
    }

  struct adc_dev_s *mcp3008 = mcp3008_initialize(spi);
  if (mcp3008 == NULL)
  {
    syslog(LOG_ERR, "Failed to initialize MCP3008\n");
  }

  int ret = adc_register("/dev/adc1", mcp3008);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Failed to register MCP3008 device driver: %d\n", ret);
  }

Once registered, this driver can be interacted with using the ADC example
(:ref:`adc-example`). Be sure to enable the software trigger, since the MCP3008
driver does not support hardware triggers (interrupts). You can also change the
number of samples per group up to 8 for all 8 channels of the ADC.

You may need to increase the `CONFIG_ADC_FIFOSIZE` value to something larger
than 8 in order to be able to store all the ADC measurements after a measurement
trigger (i.e 9).

You can configure the driver in differential mode by default using the
`CONFIG_ADC_MCP3008_DIFFERENTIAL` configuration option.

You can also configure the speed of SPI communications to the MCP3008 using the
`CONFIG_ADC_MCP3008_SPI_FREQUENCY` configuration option. This speed should be
selected based on the supply voltage used to power the MCP3008:

.. list-table:: SPI frequencies for supply voltage
   :widths: auto
   :header-rows: 1

   * - Supply Voltage
     - Frequency
   * - VDD >= 4V
     - 3.6MHz
   * - VDD >= 3.3V
     - 2.34MHz
   * - VDD = 2.7V
     - 1.35MHz

If you have a measurement from the MCP3008, you can convert it into a voltage
like so:

.. code-block:: c

   #define VREF (3.3) /* Whatever voltage is used on the VREF pin */

   struct adc_msg_s msg;

   /* Some code here to read the ADC device, you can read the ADC driver docs */

   double voltage = ((double)msg.am_data * VREF) / (1023.0);

There is also an additional `ioctl()` command supported for the MCP3008 that
permits you to switch from differential to single ended mode at runtime:

.. c:macro:: ANIOC_MCP3008_DIFF

This command changes the mode of the MCP3008 driver. The argument passed should
be 0 to disable differential mode (and thus use single-ended mode), and 1 to
enable differential mode. No other values are allowed.
