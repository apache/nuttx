.. include:: /substitutions.rst
.. _install:

Installing
==========

To start developing on Apache NuttX, we need to get the source code, configure it, compile it, and get it uploaded onto an
embedded computing board. These instructions are for `Ubuntu <https://ubuntu.com/>`_ Linux and macOS Catalina. If you're using a different
version, you may need to change some of the commands.

Prerequisites
-------------

Install prerequisites for building using Linux
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#. Install system packages

    .. code-block:: bash

        $ sudo apt install \
        bison flex gettext texinfo libncurses5-dev libncursesw5-dev \
        gperf automake libtool pkg-config build-essential gperf \
        libgmp-dev libmpc-dev libmpfr-dev libisl-dev binutils-dev libelf-dev \
        libexpat-dev gcc-multilib g++-multilib picocom u-boot-tools util-linux

#. Give yourself access to the serial console device

   This is done by adding your Linux user to the ``dialout`` group:

    .. code-block:: console

       $ sudo usermod -a -G dialout $USER
       $ # now get a login shell that knows we're in the dialout group:
       $ su - $USER

Install prerequisites for building and using Apache NuttX (macOS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 .. code-block:: bash

    $ brew tap discoteq/discoteq
    $ brew install flock
    $ brew install x86_64-elf-gcc  # Used by simulator
    $ brew install u-boot-tools  # Some platform integrate with u-boot

Install prerequisites for building and using Apache NuttX (Windows -- WSL)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you are are building Apache NuttX on windows and using WSL follow
that installation guide for Linux.  This has been verified against the
Ubunutu 18.04 version.

There may be complications interacting with
programming tools over USB.  Recently support for USBIP was added to WSL 2
which has been used with the STM32 platform, but it is not trivial to configure:
https://github.com/rpasek/usbip-wsl2-instructions

Install prerequisites for building and using Apache NuttX (Windows -- Cygwin)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Download and install `Cygwin <https://www.cygwin.com/>`_ using the minimal
installation in addition to these packages::

    make              bison             libmpc-devel
    gcc-core          byacc             automake-1.15
    gcc-g++           gperf             libncurses-devel
    flex              gdb               libmpfr-devel
    git               unzip             zlib-devel

Install Required Tools (All Platforms)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There are a collection of required tools that need to be built to build most Apache NuttX configurations:

 .. code-block:: bash

    $ mkdir buildtools
    $ export NUTTXTOOLS=`pwd`/buildtools
    $ export NUTTXTOOLS_PATH=$NUTTXTOOLS/bin:$NUTTXTOOLS/usr/bin
    $ git clone https://bitbucket.org/nuttx/tools.git tools

*NOTE:* You will need to add ``NUTTXTOOLS_PATH`` to your environment ``PATH``

#. Install gperf tool

   This required to build ``kconfig-frontends``.

   .. code-block:: console

      $ cd tools/
      $ wget http://ftp.gnu.org/pub/gnu/gperf/gperf-3.1.tar.gz
      $ tar zxf gperf-3.1.tar.gz
      $ cd gperf-3.1
      $ ./configure --prefix=$NUTTXTOOLS
      $ make
      $ make install

#. Install kconfig-frontends tool

   This is necessary to run the ``./nuttx/tools/configure.sh`` script as well as using the ncurses-based menuconfig system.

   .. code-block:: console

      $ cd tools/
      $ cd kconfig-frontends
      $ # on MacOS do the following:
      $ patch < ../kconfig-macos.diff -p 1
      $ ./configure --prefix=$NUTTXTOOLS --enable-mconf --disable-shared --enable-static --disable-gconf --disable-qconf --disable-nconf
      $ # on Linux do the following:
      $ ./configure --prefix=$NUTTXTOOLS --enable-mconf --disable-gconf --disable-qconf
      $ touch aclocal.m4 Makefile.in
      $ make
      $ make install

#. Install gen-romfs (optional)


   This is required if you want to build ROM-based file systems.

   .. code-block:: console

      $ cd tools/
      $ tar zxf genromfs-0.5.2.tar.gz
      $ cd genromfs-0.5.2
      $ make install PREFIX=$NUTTXTOOLS

Get Source Code (Stable)
------------------------
Apache NuttX releases are published on the project `Downloads <https://nuttx.apache.org/download/>`_ page and distributed
by the Apache mirrors.  Be sure to download both the nuttx and apps tarballs.


Get Source Code (Development)
------------------------------
Apache NuttX is `actively developed on GitHub <https://github.com/apache/incubator-nuttx/>`_. If you want
to use it, modify it or help develop it, you'll need the source code.

You can either clone the public repositories:

.. code-block:: console

   $ mkdir nuttx
   $ cd nuttx
   $ git clone https://github.com/apache/incubator-nuttx.git nuttx
   $ git clone https://github.com/apache/incubator-nuttx-apps apps

Or, download the `tarball <https://github.com/apache/incubator-nuttx/tarball/master>`_:

.. code-block:: console

   $ curl -OL https://github.com/apache/incubator-nuttx/tarball/master
   $ curl -OL https://github.com/apache/incubator-nuttx-apps/tarball/master
   # optionally, zipball is also available (for Windows users).

Later if we want to make modifications (for instance, to improve NuttX and save them in our own branch,
or submit them back to the project), we can do that easily using git to track our changes and submit them
upstream to the NuttX project.

Install a Cross-Compiler Toolchain
----------------------------------

With Apache NuttX, you compile the operating system and your application on your desktop or laptop computer, then install the
binary file on your embedded computer. This guide assumes your computer is an
`ARM <https://en.wikipedia.org/wiki/ARM_architecture>`_ CPU. If it isn't, you'll need a different tool chain.

   There are hints on how to get the latest tool chains for most supported architectures in the Apache NuttX CI helper
   `script <https://github.com/apache/incubator-nuttx-testing/blob/master/cibuild.sh>`_ and Docker `container <https://github.com/apache/incubator-nuttx-testing/blob/master/docker/linux/Dockerfile>`_

Download the right flavor of the
`ARM Embedded GNU Toolchain <https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm>`_
for your embedded processor's CPU.

Unpack it into ``/opt/gcc`` and add the bin directory to your path. For instance:

.. code-block:: console

   $ usermod -a -G users $USER
   $ # get a login shell that knows we're in this group:
   $ su - $USER
   $ sudo mkdir /opt/gcc
   $ sudo chgrp -R users /opt/gcc
   $ sudo chmod -R u+rw /opt/gcc
   $ cd /opt/gcc
   $ HOST_PLATFORM=x86_64-linux   # use "mac" for macOS.
   $ # For windows there is a zip instead (gcc-arm-none-eabi-9-2019-q4-major-win32.zip)
   $ curl -L -o gcc-arm-none-eabi-9-2019-q4-major-${HOST_PLATFORM}.tar.bz2 https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-${HOST_PLATFORM}.tar.bz2
   $ tar xf gcc-arm-none-eabi-9-2019-q4-major-${HOST_PLATFORM}.tar.bz2
   $ # add the toolchain bin/ dir to your path...
   $ # you can edit your shell's rc files if you don't use bash
   $ echo "export PATH=/opt/gcc/gcc-arm-none-eabi-9-2019-q4-major/bin:$PATH" >> ~/.bashrc

----

Next up is :ref:`compiling`.
