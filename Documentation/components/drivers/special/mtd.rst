================================
Memory Technology Device Drivers
================================

-  ``include/nuttx/mtd/mtd.h``. All structures and APIs needed
   to work with MTD drivers are provided in this header file.

-  ``struct mtd_dev_s``. Each MTD device driver must implement
   an instance of ``struct mtd_dev_s``. That structure defines a
   call table with the following methods:

   Erase the specified erase blocks (units are erase blocks):

   Read/write from the specified read/write blocks:

   Some devices may support byte oriented reads (optional). Most
   MTD devices are inherently block oriented so byte-oriented
   accesses are not supported. It is recommended that low-level
   drivers not support read() if it requires buffering.

   Some devices may also support byte oriented writes (optional).
   Most MTD devices are inherently block oriented so byte-oriented
   accesses are not supported. It is recommended that low-level
   drivers not support read() if it requires buffering. This
   interface is only available if ``CONFIG_MTD_BYTE_WRITE`` is
   defined.

   Support other, less frequently used commands:

   -  ``MTDIOC_GEOMETRY``: Get MTD geometry
   -  ``MTDIOC_XIPBASE:``: Convert block to physical address for
      eXecute-In-Place
   -  ``MTDIOC_BULKERASE``: Erase the entire device

   is provided via a single ``ioctl`` method (see
   ``include/nuttx/fs/ioctl.h``):

-  **Binding MTD Drivers**. MTD drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. In general, the binding sequence is:

   #. Get an instance of ``struct mtd_dev_s`` from the
      hardware-specific MTD device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``drivers/mtd/m25px.c`` and ``drivers/mtd/ftl.c``
