========================
Analog (ADC/DAC) Drivers
========================

The NuttX analog drivers are split into two parts:

#. An "upper half", generic driver that provides the common analog
   interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level controls to implement the analog functionality.

-  General header files for the NuttX analog drivers reside in
   ``include/nuttx/analog/``. These header files includes both the
   application level interface to the analog driver as well as the
   interface between the "upper half" and "lower half" drivers.
-  Common analog logic and share-able analog drivers reside in the
   ``drivers/analog/``.
-  Platform-specific drivers reside in
   ``arch/<architecture>//src/<hardware>`` directory
   for the specific processor ``<architecture>`` and for the
   specific ``<chip>`` analog peripheral devices.

ADC Drivers
-----------

-  ``include/nuttx/analog/adc.h``. All structures and APIs needed
   to work with ADC drivers are provided in this header file. This
   header file includes:

   #. Structures and interface descriptions needed to develop a
      low-level, architecture-specific, ADC driver.
   #. To register the ADC driver with a common ADC character
      driver.
   #. Interfaces needed for interfacing user programs with the
      common ADC character driver.

-  ``drivers/analog/adc.c``. The implementation of the common ADC
   character driver.

DAC Drivers
-----------

-  ``include/nuttx/analog/dac.h``. All structures and APIs needed
   to work with DAC drivers are provided in this header file. This
   header file includes:

   #. Structures and interface descriptions needed to develop a
      low-level, architecture-specific, DAC driver.
   #. To register the DAC driver with a common DAC character
      driver.
   #. Interfaces needed for interfacing user programs with the
      common DAC character driver.

-  ``drivers/analog/dac.c``. The implementation of the common DAC
   character driver.
