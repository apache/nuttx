===========
DAC Drivers
===========

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

Application Programming Interface
=================================

The first necessary thing to be done in order to use the DAC driver from an
application is to include the correct header filer. It contains the
Application Programming Interface to the PWM driver. To do so, include

.. code-block:: c

  #include <nuttx/analog/dac.h>

DAC driver is registered as a POSIX character device driver into ``/dev``
namespace. It is necessary to open the device to get a file descriptor for
further operations. This can be done with standard POSIX ``open()`` call.

Standard POSIX ``write()`` call is used to send data from an application to
a controller. Structure  ``dac_msg_s`` is used to pass the data/samples.

.. c:struct:: dac_msg_s
.. code-block:: c

  begin_packed_struct struct dac_msg_s
  {
    /* The 8-bit DAC Channel */
    uint8_t      am_channel;
    /* DAC convert result (4 bytes) */
    int32_t      am_data;
  } end_packed_struct;

Application Example
~~~~~~~~~~~~~~~~~~~

An example application can be found in ``nuttx-apps`` repository under
path ``examples/dac``. It provides command line interface to write data
to DAC channels.

Configuration
=============

This section describes DAC driver configuration in ``Kconfig``. The reader
should refer to target documentation for target specific configuration.

The peripheral is enabled by ``CONFIG_ANALOG``  and ``CONFIG_DAC`` options,
respectively. The FIFO queue size is configurable with ``CONFIG_DAC_FIFOSIZE``.
This size is limited to ``255`` to fit into ``uint8_t``.

Supported External DACs (I2C/SPI)
=================================

NuttX also provides support for various external DAC devices. These usually
communicate with the MCU via I2C or SPI interfaces.

I2C-based DACs:

.. toctree::
  :maxdepth: 1

  dac7571/index.rst
  mcp47x6/index.rst

SPI-based DACs:

.. toctree::
  :maxdepth: 1

  dac7554/index.rst
  mcp48xx/index.rst
