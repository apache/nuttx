==================
SPI Device Drivers
==================

-  ``include/nuttx/spi/spi.h``. All structures and APIs needed
   to work with SPI drivers are provided in this header file.

-  ``struct spi_ops_s``. Each SPI device driver must implement
   an instance of ``struct spi_ops_s``. That structure defines a
   call table with the following methods:

-  **Binding SPI Drivers**. SPI drivers are not normally directly
   accessed by user code, but are usually bound to another, higher
   level device driver. See for example,
   ``int mmcsd_spislotinitialize(int minor, int slotno, FAR struct spi_dev_s *spi)``
   in ``drivers/mmcsd/mmcsd_spi.c``. In general, the binding
   sequence is:

   #. Get an instance of ``struct spi_dev_s`` from the
      hardware-specific SPI device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``drivers/loop.c``,
   ``drivers/mmcsd/mmcsd_spi.c``, ``drivers/ramdisk.c``, etc.

=================
Native VirtIO SPI
=================

An SPI controller can be exposed over a VirtIO transport.
``CONFIG_DRIVERS_VIRTIO_SPI`` enables the frontend (client), which probes a
VirtIO SPI device (device id 45), reads its configuration space and exposes a
``struct spi_dev_s`` retrievable with ``virtio_spi_get_master(index)``.
Board code may then pass it to device drivers or register it with
``spi_register()`` as ``/dev/spiN``. ``CONFIG_SPI_EXCHANGE`` is required.

The wire format in ``include/nuttx/virtio/virtio-spi.h`` follows the
Linux VirtIO SPI definitions. An ``spi_transfer()`` sequence executes as one
bus-locked run of requests, with CS held between transfers that do not
de-select. The frontend can be validated against a host vhost-user backend.
