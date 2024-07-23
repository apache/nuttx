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
   ``arch/<architecture>/src/<hardware>`` directory
   for the specific processor ``<architecture>`` and for the
   specific ``<chip>`` analog peripheral devices.

.. toctree::
  :caption: Supported Drivers
  :maxdepth: 1

  adc/index.rst
  dac/index.rst

