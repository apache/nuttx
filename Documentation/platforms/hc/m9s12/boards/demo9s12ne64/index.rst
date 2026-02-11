============
demo9s12ne64
============

.. tags:: vendor:freescale

This README discusses issues unique to NuttX configurations for the Freescale
DEMO9S12NE64 development board.

See :doc:` </platforms/hc/m9s12/index>` for more information about the chip this
board uses.

Installation
============

.. note::

   Either Linux or Cygwin on Windows can be used for the development
   environment. The source has been built only using the GNU toolchain (see
   below).  Other toolchains will likely cause problems.

A GNU GCC-based toolchain is assumed. The PATH environment variable should be
modified to point to the correct path to the HC12 GCC toolchain (if different
from the default in your PATH variable).

If you have no HC12 toolchain, one can be downloaded from the NuttX
Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
This GNU toolchain builds and executes in the Linux or Cygwin
environments.

1. You must have already configured NuttX in ``<some-dir>/nuttx``.

   .. code:: console

      $ tools/configure.sh demo9s12nec64:<sub-dir>

2. Download the latest buildroot package into ``<some-dir>``

3. Unpack the buildroot tarball. The resulting directory may have versioning
   information on it like ``buildroot-x.y.z``. If so, rename
   ``<some-dir>/buildroot-x.y.z`` to ``<some-dir>/buildroot``.

   .. code:: console

      $ cd <some-dir>/buildroot
      $ cp boards/m9s12x-defconfig-3.3.6 .config
      $ make oldconfig
      $ make

   If the make fails because it can't find the file to download, you may have to
   locate the file on the internet and download it into the ``archives/``
   directory manually. For example, ``binutils-2.18`` can be found here:
   http://ftp.gnu.org/gnu/binutils/

4. Make sure that the ``PATH`` variable includes the path to the newly built
   binaries.

Configuration
=============

You can configure this board using the following command, where ``<config>`` is
one of the configurations below.

.. code:: console

   $ tools/configure.sh demo9s12nec64:<subdir>

**GPIO Interrupts**

* ``CONFIG_HCS12_GPIOIRQ``: Enable general support for GPIO IRQs
* ``CONFIG_HCS12_PORTG_INTS``: Enable PortG IRQs
* ``CONFIG_HCS12_PORTH_INTS``: Enable PortH IRQs
* ``CONFIG_HCS12_PORTJ_INTS``: Enable PortJ IRQs

**HCS12 build options**

* ``CONFIG_HCS12_SERIALMON``: Indicates that the target systems uses the
  Freescale serial bootloader.

* ``CONFIG_HCS12_NONBANKED``: Indicates that the target systems does not support
  banking. Only short calls are made; one fixed page is presented in the paging
  window. Only 48Kb of FLASH is usable in this configuration: pages 3e, 3d, then
  3f will appear as a contiguous address space in memory.

**HCS12 Sub-system support**

* ``CONFIG_HCS12_SCI0``
* ``CONFIG_HCS12_SCI1``

**HCS12 specific device driver settings**

* ``CONFIG_SCIn_SERIAL_CONSOLE``: selects SCIn for the console and ttys0
  (default is the SCI0).

* ``CONFIG_SCIn_RXBUFSIZE``: Characters are buffered as received. This specific
  the size of the receive buffer

* ``CONFIG_SCIn_TXBUFSIZE``: Characters are buffered before being sent.  This
  specific the size of the transmit buffer

* ``CONFIG_SCIn_BAUD``: The configure BAUD of the UART.

* ``CONFIG_SCIn_BITS``: The number of bits.  Must be either 7 or 8.

* ``CONFIG_SCIn_PARTIY``: 0=no parity, 1=odd parity, 2=even parity, 3=mark 1,
  4=space 0

* ``CONFIG_SCIn_2STOP``: Two stop bits

ostest
------

This configuration directory, performs a simple OS test using
``examples/ostest``.
