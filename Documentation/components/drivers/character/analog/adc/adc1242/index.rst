==========
TI ADS1242
==========

ADS1242 24-Bit SPI powered ADC. This driver supports reading the ADC
conversionresult as well as configuring the ADC, setting the input channel,
etc. is implemented via ioctl calls. However, it does not yet implement
the standard ADC interface.

-  ``include/nuttx/analog/ads1242.h``. All structures and APIs needed
   to work with DAtC drivers are provided in this header file.
