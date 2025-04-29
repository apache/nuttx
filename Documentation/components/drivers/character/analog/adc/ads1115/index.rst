==========
TI ADS1115
==========

Contributed by Jia Lin

The ADS1115 is a 16-bit, 4-channel ADC made by Texas Instruments which operates over
I2C. It can measure voltages from each channel individually, or between different 
pairs of channels. The ADS1115 also supports a programmable gain amplifier (PGA) and 
a digital comparator. 


.. list-table:: Channel Numbers and Corresponding Sources
   :widths: auto

   * - Channel number
     - AINP
     - AINN
   * - 0                 
     - AIN0
     - AIN1
   * - 1                 
     - AIN0
     - AIN3
   * - 2                 
     - AIN1
     - AIN3
   * - 3                 
     - AIN2
     - AIN3
   * - 4                 
     - AIN0
     - GND
   * - 5                 
     - AIN1
     - GND
   * - 6                 
     - AIN2
     - GND
   * - 7                 
     - AIN3
     - GND

Driver Interface
---------------------

To register the ADS1115 device driver as a standard NuttX analog device on your
board, you can use something similar to the below code for the RP2040.

.. code-block:: c
  
  #include <nuttx/analog/ads1115.h>
  #include <nuttx/analog/adc.h>

  /* Register ADS1115 ADC. */

  struct adc_dev_s *ads1115 = ads1115_initialize(rp2040_i2cbus_initialize(0),
                                                 CONFIG_ADC_ADS1115_ADDR);
  if (ads1115 == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize ADS1115\n");
    }

  ret = adc_register("/dev/adc1", ads1115);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register ADS1115 device driver: %d\n", ret);
    }

If you have a measurement from the ADS1115, you can convert it into a voltage
like so:

.. code-block:: c

   #define FSR (2.048)

   struct adc_msg_s msg;

   /* Some code here to read the ADC device, you can read the ADC driver docs */

   double voltage = ((double)msg.am_data  * FSR) / (32768.0);

Once registered, this driver can be interacted with using the ADC example
(:ref:`adc-example`). Be sure to enable the software trigger, since the ADS1115
driver does not support hardware triggers (interrupts). You can also change the
number of samples per group up to 8 for all 8 channels of the ADC.

You may need to increase the `CONFIG_ADC_FIFOSIZE` value to something larger
than 8 in order to be able to store all the ADC measurements after a measurement
trigger (i.e 9).

Configuration
---------------------

.. list-table:: Configuration Options
   :widths: auto

   * - Name
     - Description
   * - CONFING_ADC_ADS1115_I2C_FREQUENCY
     - I2C frequency of the ADS1115
   * - CONFIG_ADC_ADS1115_ADDR
     - I2C address of the ADS1115
   * - CONFIG_ADC_ADS1115_CHANNEL 
     - Default ADC channel to read
   * - CONFIG_ADC_ADS1115_PGA
     - Gain of the ADS1115
   * - CONFIG_ADC_ADS1115_CONTINOUS
     - Continuous mode of the ADS1115
   * - CONFIG_ADC_ADS1115_DR
     - Data rate of the ADS1115
   * - CONFIG_ADC_ADS1115_COMP_MODE
     - Mode of the ADS1115 comparator, traditional or window  
   * - CONFIG_ADC_ADS1115_COMP_POL
     - Polarity of the ADS1115 comparator, active high or active low
   * - CONFIG_ADC_ADS1115_COMP_LAT
     - Latching mode of the ADS1115 comparator, traditional or latching
   * - CONFIG_ADC_ADS1115_COMP_QUE
     - Comparator queue of the ADS1115, which changes when the ALRT/RDY pin is asserted.
   * - CONFIG_ADC_ADS1115_HI_THRESH
     - HIGH_THRESH register of the ADS1115
   * - CONFIG_ADC_ADS1115_LO_THRESH
     - LOW_THRESH register of the ADS1115

.. list-table:: Data Rates
   :widths: auto

   * - Value in Kconfig
     - Data Rate
   * - 0
     - 8 SPS
   * - 1
     - 16 SPS
   * - 2
     - 32 SPS
   * - 3
     - 64 SPS
   * - 4
     - 128 SPS
   * - 5
     - 250 SPS
   * - 6
     - 475 SPS
   * - 7
     - 860 SPS


.. list-table:: PGA Values
   :widths: auto

   * - Value in Kconfig
     - Full Scale Range (FSR) 
   * - 0
     - ±6.144V
   * - 1
     - ±4.096V
   * - 2
     - ±2.048V
   * - 3
     - ±1.024V
   * - 4
     - ±0.512V
   * - 5
     - ±0.256V
   * - 6
     - ±0.256V
   * - 7
     - ±0.256V


.. list-table:: Comparator Queue Values
   :widths: auto

   * - Value in Kconfig
     - Comparator Queue
   * - 0
     - Assert after one conversion
   * - 1
     - Assert after two conversions
   * - 2
     - Assert after four conversions
   * - 3
     - Disable comparator



Additional ioctl Commands
--------------------------------

There are various additional ioctl() commands that can be used with the ADS1115 driver. 
These mostly allow for changes of configuration in runtime.

.. c:macro:: ANIOC_ADS1115_SET_PGA

This command changes the gain of the ADS1115 driver. The argument passed should
be of type ads1115_pga_e, which corresponds to the gain seen above.

.. c:macro:: ANIOC_ADS1115_SET_MODE

This command changes the ADS1115 to operate in continuous or single-shot mode.

.. c:macro:: ANIOC_ADS1115_SET_DR

This command changes the data rate of the ADS1115 driver. The argument passed should
be of type ads1115_dr_e, which corresponds to the data rate seen above. 

.. c:macro:: ANIOC_ADS1115_SET_COMP_MODE

This command changes the ADS1115 to operate in either traditional or window comparator mode.

.. c:macro:: ANIOC_ADS1115_SET_COMP_POL

This command changes the ADS1115 to operate in either active high or active low mode.

.. c:macro:: ANIOC_ADS1115_SET_COMP_LAT

This command changes the ADS1115 to operate in either traditional or latching comparator mode.

.. c:macro:: ANOIC_ADS1115_SET_COMP_QUEUE

This command changes the comparator queue feature of the ADS1115. The argument passed should 
be of type ads1115_comp_queue_e, which corresponds to the comparator queue seen above.

.. c:macro:: ANOIC_ADS1115_SET_HI_THRESH

This command changes the HIGH_THRESH register of the ADS1115. The argument passed should be 
of type uint16_t, which corresponds to the HIGH_THRESH register value.

.. c:macro:: ANOIC_ADS1115_SET_LO_THRESH

This command changes the LOW_THRESH register of the ADS1115. The argument passed should be 
of type uint16_t, which corresponds to the LOW_THRESH register value.
