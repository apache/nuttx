.. include:: /substitutions.rst
.. _quickstart:

Quickstart
==========

Here's the quick version of getting started with NuttX. This is a bare-bones outline for experienced developers– if it's
going too quickly, dive into the following sections. This Quickstart guide assumes you're on a Linux
computer, you're using an ARM microcontroller on your embedded board, and you're familiar with using the command line.

#. Install a Cross-Compiler Toolchain

   With NuttX, you compile the operating system and your application on your desktop or laptop computer, then install the
   binary file on your embedded computer. This guide assumes your computer is an
   `ARM <https://en.wikipedia.org/wiki/ARM_architecture>`_ CPU. If it isn't, you'll need a different tool chain.

   You can download a toolchain from
   `ARM Embedded GNU Toolchain <https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm>`_
   for your embedded processor's CPU. You can also use a toolchain shipped with your OS for the `none-eabi` target, such as `gcc-arm-none-eabi` in Linux.

   In the following example, we download ``gcc-arm-none-eabi`` version 9.0 and unpack it into ``/opt/gcc``:

   .. code-block:: console

      $ sudo mkdir /opt/gcc
      $ sudo chgrp -R users /opt/gcc
      $ cd /opt/gcc
      $ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2
      $ tar xf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2

   Then, add the toolchain ``bin/`` directory to your path:

   .. code-block:: console

      $ echo "export PATH=/opt/gcc/gcc-arm-none-eabi-9-2019-q4-major/bin:$PATH" >> ~/.bashrc

   If you are using any other shell, the procedure is similar by editing the corresponding rc file.

#. Download Apache NuttX

   The next step is to download NuttX main repository along the application repository. The latter is technically optional in a very minimal configurations but should be included in normal configuration since it includes the NuttX shell.

   .. code-block:: console

      $ mkdir nuttx
      $ cd nuttx
      $ git clone https://github.com/apache/incubator-nuttx.git nuttx
      $ git clone https://github.com/apache/incubator-nuttx-apps apps
      $ git clone https://bitbucket.org/nuttx/tools.git tools

#. Install the ``kconfig-frontends`` package

   NuttX is configured using ``kconfig`` system via an interactive menu system (``menuconfig``). It also includes the ``kconfig-tweak`` utility that can be used to quickly change debug settings without going into the menu system.

   .. tabs::

      .. code-tab:: console Ubuntu 20.04 LTS and later

         $ apt install kconfig-frontends

      .. code-tab:: console MacOS, Ubuntu 18.04 LTS and earlier

         $ cd tools/kconfig-frontends
         $ # on MacOS do the following:
         $ patch < ../kconfig-macos.diff -p 1
         $ ./configure --enable-mconf --disable-shared --enable-static --disable-gconf --disable-qconf --disable-nconf
         $ # on Linux do the following:
         $  ./configure --enable-mconf --disable-nconf --disable-gconf --disable-qconf
         $ make
         $ make install

#. List Possible Apache NuttX Base Configurations

   Find your hardware and a good starting application in the list of base configurations. In the application list,
   ``nsh`` is the Apache NuttX Shell, an interactive commandline that's a good starting place if you're new.

   .. code-block:: bash

      $ cd nuttx
      $ ./tools/configure.sh -L | less

#. Initialize Configuration

   Pick one of the board:application base configuration pairs from the list, and feed it to
   the configuration script. The ``-l`` tells us that we're on Linux. macOS and Windows builds
   are possible, this guide doesn't cover them yet.

   .. code-block:: bash

      $ cd nuttx
      $ ./tools/configure.sh -l <board-name>:<config-dir>
      # for instance:
      $ ./tools/configure.sh -l sama5d27-xult:nsh

#. Customize Your Configuration (Optional)

   This step is optional. Right now, this is mainly to get familiar with how it works– you don't need to change
   any of the options now, but knowing how to do this will come in handy later.

   There are a lot of options. We'll cover a few of them here. Don't worry about the complexity– you don't have to use most of the options.

   .. code-block:: bash

      $ make menuconfig

   Use your arrows to navigate the menu and :kbd:`Enter` key to enable/disable options. To exit and save your configuration, go back to the main menu, choose ``<Exit>`` and select "yes" when asked if you want to save.

#. Compile Apache NuttX

   .. code-block:: bash

      $ make

#. Install the Executable Program on Your Board

   This step is a bit more complicated, depending on your board. It's covered in the section
   :ref:`Running Apache NuttX <running>`.

----

.. rubric:: More Details

If you want more details, start at :ref:`install`.

