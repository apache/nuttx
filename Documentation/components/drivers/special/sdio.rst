===================
SDIO Device Drivers
===================

-  ``include/nuttx/sdio.h``. All structures and APIs needed to
   work with SDIO drivers are provided in this header file.

-  ``struct sdio_dev_s``. Each SDIO device driver must
   implement an instance of ``struct sdio_dev_s``. That structure
   defines a call table with the following methods:

   Mutual exclusion:

   Initialization/setup:

   Command/Status/Data Transfer:

   Event/Callback support:

   DMA support:

-  **Binding SDIO Drivers**. SDIO drivers are not normally
   directly accessed by user code, but are usually bound to
   another, higher level device driver. In general, the binding
   sequence is:

   #. Get an instance of ``struct sdio_dev_s`` from the
      hardware-specific SDIO device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``arch/arm/src/stm32/stm32_sdio.c`` and
   ``drivers/mmcsd/mmcsd_sdio.c``
