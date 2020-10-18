.. include:: /substitutions.rst
.. _install:

Installing
==========

To start developing on Apache NuttX, we need to get the source code, configure it, compile it, and get it uploaded onto an
embedded computing board. These instructions are for `Ubuntu <https://ubuntu.com/>`_ Linux and macOS Catalina. If you're using a different
version, you may need to change some of the commands.

Prerequisites
-------------

.. tabs::

  .. tab:: Linux

    #. Install system packages

      .. code-block:: console

        $ sudo apt install \
        bison flex gettext texinfo libncurses5-dev libncursesw5-dev \
        gperf automake libtool pkg-config build-essential gperf genromfs \
        libgmp-dev libmpc-dev libmpfr-dev libisl-dev binutils-dev libelf-dev \
        libexpat-dev gcc-multilib g++-multilib picocom u-boot-tools util-linux

    #. Give yourself access to the serial console device

      This is done by adding your Linux user to the ``dialout`` group:

      .. code-block:: console

        $ sudo usermod -a -G dialout $USER
        $ # now get a login shell that knows we're in the dialout group:
        $ su - $USER

  .. tab:: macOS

    .. code-block:: console

      $ brew install x86_64-elf-gcc  # Used by simulator
      $ brew install u-boot-tools  # Some platform integrate with u-boot

  .. tab:: Windows / WSL

    If you are are building Apache NuttX on windows and using WSL follow
    that installation guide for Linux.  This has been verified against the
    Ubunutu 18.04 version.

    There may be complications interacting with
    programming tools over USB.  Recently support for USBIP was added to WSL 2
    which has been used with the STM32 platform, but it is not trivial to configure:
    https://github.com/rpasek/usbip-wsl2-instructions

  .. tab:: Windows/Cygwin

    Download and install `Cygwin <https://www.cygwin.com/>`_ using the minimal
    installation in addition to these packages::

        make              bison             libmpc-devel
        gcc-core          byacc             automake-1.15
        gcc-g++           gperf             libncurses-devel
        flex              gdb               libmpfr-devel
        git               unzip             zlib-devel

To complete the installation of prerequisites, you need to install `kconfig-frontends`
as instructed in the :doc:`quickstart` guide.

Install a Cross-Compiler Toolchain
----------------------------------

To build Apache NuttX you need the appropriate toolchain
according to your target platform. Some Operating Systems
such as Linux distribute toolchains for various architectures.
This is usually an easy choice however you should be aware
that in some cases the version offered by your OS may have
problems and it may better to use a widely used build from
another source.

The following example shows how to install a toolchain for
ARM architecture:

.. tabs::

  .. code-tab:: console Ubuntu (deb)

    $ apt install gcc-arm-none-eabi binutils-arm-none-eabi

  .. tab:: From arm.com

    First, create a directory to hold the toolchain:

    .. code-block:: console

      $ usermod -a -G users $USER
      $ # get a login shell that knows we're in this group:
      $ su - $USER
      $ sudo mkdir /opt/gcc
      $ sudo chgrp -R users /opt/gcc
      $ sudo chmod -R u+rw /opt/gcc
      $ cd /opt/gcc

    Download and extract toolchain:

    .. code-block:: console

      $ HOST_PLATFORM=x86_64-linux   # use "mac" for macOS.
      $ # For windows there is a zip instead (gcc-arm-none-eabi-9-2019-q4-major-win32.zip)
      $ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-${HOST_PLATFORM}.tar.bz2
      $ tar xf gcc-arm-none-eabi-9-2019-q4-major-${HOST_PLATFORM}.tar.bz2

    Add the toolchain to your `PATH`:

    .. code-block:: console

      $ echo "export PATH=/opt/gcc/gcc-arm-none-eabi-9-2019-q4-major/bin:$PATH" >> ~/.bashrc

    You can edit your shell's rc files if you don't use bash.

.. tip::
  There are hints on how to get the latest tool chains for most supported
  architectures in the Apache NuttX CI helper
  `script <https://github.com/apache/incubator-nuttx-testing/blob/master/cibuild.sh>`_
  and Docker `container <https://github.com/apache/incubator-nuttx-testing/blob/master/docker/linux/Dockerfile>`_

Get Source Code
---------------

Now that all required tools are installed, you need to download NuttX source-code.

.. tabs::

  .. tab:: Development (Git)

    Apache NuttX is `actively developed on GitHub <https://github.com/apache/incubator-nuttx/>`_.
    If you intend to contribute changes or you simply need the absolute latest version,
    you should clone the Git repositories:

    .. code-block:: console

       $ mkdir nuttx
       $ cd nuttx
       $ git clone https://github.com/apache/incubator-nuttx.git nuttx
       $ git clone https://github.com/apache/incubator-nuttx-apps apps

    The development source code is also available as a compressed archive, should you need it:

    .. code-block:: console

       $ curl -OL https://github.com/apache/incubator-nuttx/tarball/master
       $ curl -OL https://github.com/apache/incubator-nuttx-apps/tarball/master
       # optionally, zipball is also available (for Windows users).

  .. tab:: Stable Release

    Apache NuttX releases are published on the project `Downloads <https://nuttx.apache.org/download/>`_
    page and distributed by the Apache mirrors.  Be sure to download both the `nuttx` and `apps` tarballs.

----

Next up is :ref:`compiling`.
