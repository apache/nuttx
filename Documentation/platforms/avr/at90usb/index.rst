=================
Microchip AT90USB
=================

AVR AT90USB64x and AT90USB6128x
-------------------------------

**Micropendous 3 AT90USB64x** and **AT90USB6128x**. This port of NuttX
to the Opendous Micropendous 3 board. The Micropendous3 is may be
populated with an AT90USB646, 647, 1286, or 1287. I have only the
AT90USB647 version for testing. This version have very limited memory
resources: 64K of FLASH and 4K of SRAM.

**PJRC Teensy++ 2.0 AT90USB1286**. This is a port of NuttX to the PJRC
Teensy++ 2.0 board. This board was developed by
`PJRC <http://pjrc.com/teensy/>`__. The Teensy++ 2.0 is based on an
Microchip AT90USB1286 MCU.

**AVR-Specific Issues**. The basic AVR port is solid. The biggest issue
for using AVR is its tiny SRAM memory and its Harvard architecture.
Because of the Harvard architecture, constant data that resides to flash
is inaccessible using "normal" memory reads and writes (only SRAM data
can be accessed "normally"). Special AVR instructions are available for
accessing data in FLASH, but these have not been integrated into the
normal, general purpose OS.

Most NuttX test applications are console-oriented with lots of strings
used for ``printf()`` and debug output. These strings are all stored in
SRAM now due to these data accessing issues and even the smallest
console-oriented applications can quickly fill a 4-8K memory. So, in
order for the AVR port to be useful, one of two things would need to be
done:

#. Don't use console applications that required lots of strings. The
   basic AVR port is solid and your typical deeply embedded application
   should work fine. Or,
#. Create a special version of printf that knows how to access strings
   that reside in FLASH (or EEPROM).

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU toolchain, 3) Cygwin/MSYS with Windows
native toolchain, or 4) Native Windows. All testing, however, has been
performed using the NuttX DIY toolchain for Linux or Cygwin is provided
by the NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
package. As a result, that toolchain is recommended.

Toolchains
==========

There are several toolchain options. However, testing has been performed using
*only* the NuttX buildroot toolchain described below. Therefore, the NuttX
buildroot toolchain is the recommended choice.

The toolchain may be selected using the kconfig-mconf tool (via ``make
menuconfig``), by editing the existing configuration file (``defconfig``), or by
overriding the toolchain on the make commandline with
``CONFIG_AVR_TOOLCHAIN=<toolchain>``.

The valid values for ``<toolchain>`` are ``BUILDROOT``, ``CROSSPACK``,
``LINUXGCC`` and ``WINAVR``.

Buildroot
---------

There is a DIY buildroot version for the AVR boards here:
http://bitbucket.org/nuttx/buildroot/downloads/. See the following section for
details on building this toolchain.

Before building, make sure that the path to the new toolchain is included in
your ``PATH`` environment variable.

After configuring NuttX, make sure that ``CONFIG_AVR_BUILDROOT_TOOLCHAIN=y`` is
set in your ``.config`` file.

WinAVR
------

For Cygwin development environment on Windows machines, you can use WinAVR:
http://sourceforge.net/projects/winavr/files/

Before building, make sure that the path to the new toolchain is included in
your ``PATH`` environment variable.

After configuring NuttX, make sure that ``CONFIG_AVR_WINAVR_TOOLCHAIN=y`` is set
in your ``.config`` file.

.. warning::

   There is an incompatible version of ``cygwin.dll`` in the ``WinAVR/bin``
   directory! Make sure that the path to the correct ``cygwin.dll`` file
   precedes the path to the WinAVR binaries!

Linux
-----

For Linux, there are widely available avr-gcc packages.  On Ubuntu, use:
sudo apt-get install gcc-avr gdb-avr avr-libc

After configuring NuttX, make sure that CONFIG_AVR_LINUXGCC_TOOLCHAIN=y is set in your
.config file.

macOS
-----

For macOS, the CrossPack for AVR toolchain is available from:
http://www.obdev.at/products/crosspack/index.html

This toolchain is functionally equivalent to the Linux GCC toolchain.

Windows Native Toolchains
=========================

The WinAVR toolchain is a Windows native toolchain. There are several
limitations to using a Windows native toolchain in a Cygwin environment. The
three biggest are:

1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
   performed automatically in the Cygwin makefiles using the ``cygpath`` utility
   but you might easily find some new path problems.  If so, check out ``cygpath
   -w``

2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
   are used in NuttX (e.g., ``include/arch``).  The make system works around
   these problems for the Windows tools by copying directories instead of
   linking them.  But this can also cause some confusion for you:  For example,
   you may edit a file in a "linked" directory and find that your changes had no
   effect. That is because you are building the copy of the file in the "fake"
   symbolic directory.  If you use a Windows toolchain, you should get in the
   habit of making like this:

   .. code:: console

      $ make clean_context all

   An alias in your ``.bashrc`` file might make that less painful.

An additional issue with the WinAVR toolchain, in particular, is that it
contains an incompatible version of the Cygwin DLL in its ``bin/`` directory.
You must take care that the correct Cygwin DLL is used.

NuttX buildroot toolchain
=========================

If NuttX buildroot toolchain source tarball cne can be downloaded from the NuttX
Bitbucket download site (https://bitbucket.org/nuttx/nuttx/downloads/). This GNU
toolchain builds and executes in the Linux or Cygwin environment.

1. You must have already configured NuttX in ``<some-dir>/nuttx``.

   .. code:: console

      $ tools/configure.sh micropendous3:<sub-dir>

   .. note::

      You also must copy avr-libc header files into the NuttX include directory
      with a command perhaps like:

      .. code:: console

         $ cp -a /cygdrive/c/WinAVR/include/avr include/.

2. Download the latest buildroot package into ``<some-dir>``

3. Unpack the buildroot tarball.  The resulting directory may have versioning
   information on it like ``buildroot-x.y.z``.  If so, rename
   ``<some-dir>/buildroot-x.y.z`` to ``<some-dir>/buildroot``.

   .. code:: console

      $ cd <some-dir>/buildroot
      $ cp boards/avr-defconfig-4.5.2 .config
      $ make oldconfig
      $ make

4. Make sure that the ``PATH`` variable includes the path to the newly built
   binaries.

See the file ``boards/README.txt`` in the buildroot source tree.That has more
detailed PLUS some special instructions that you will need to follow if you are
building a toolchain for Cygwin under Windows.

avr-libc
========

Header Files
------------

In any case, header files from avr-libc are required:
http://www.nongnu.org/avr-libc/. A snapshot of avr-lib is included in the WinAVR
installation. For Linux development platforms, avr-libc package is readily
available (and would be installed in the apt-get command shown above). But if
you are using the NuttX buildroot configuration on Cygwin, then you will have to
build get avr-libc from binaries.

Header File Installation
------------------------

The NuttX build will required that the AVR header files be available via the
NuttX include directory. This can be accomplished by either copying the
avr-libc header files into the NuttX include directory:

.. code:: console

   $ cp -a <avr-libc-path>/include/avr <nuttx-path>/include/.

Or simply using a symbolic link:

.. code:: console

   $ ln -s <avr-libc-path>/include/avr <nuttx-path>/include/.

Build Notes
-----------

It may not be necessary to have a built version of avr-lib; only header files
are required. But if you choose to use the optimized library functions of the
floating point library, then you may have to build avr-lib from sources. Below
are instructions for building avr-lib from fresh sources:

1. Download the avr-libc package from
   http://savannah.nongnu.org/projects/avr-libc/. I am using
   avr-lib-1.7.1.tar.bz2

2. Unpack the tarball and ``cd`` into it:

   .. code:: console

      $ tar jxf avr-lib-1.7.1.tar.bz2
      $ cd avr-lib-1.7.1

3. Configure avr-lib. Assuming that WinAVR is installed at the following
   location:

   .. code:: console

      $ export PATH=/cygdrive/c/WinAVR/bin:$PATH
      $ ./configure --build=`./config.guess` --host=avr

   This takes a *long* time.

4. Make avr-lib.

   .. code:: console

      $ make

   This also takes a long time because it generates variants for nearly
   all AVR chips.

5. Install avr-lib.

   .. code:: console

      $ make install


Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
