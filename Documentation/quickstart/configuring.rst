.. include:: /substitutions.rst
.. _configuring:

===========
Configuring
===========

Apache NuttX is a very configurable: nearly all features can be configured in or
out of the system. This makes it possible to compile a build tailored for your
hardware and application.

The Apache NuttX configuration system uses Linux's
`kconfig system <https://www.kernel.org/doc/Documentation/kbuild/kconfig-language.txt>`_ which
includes various frontends that allow you to modify configuration easily. Usually, the ``menuconfig``
frontend is used, which is a console based menu system (more info `here <https://en.wikipedia.org/wiki/Menuconfig>`_).

As previously explained in :doc:`compiling_make`, the first step is to load a premade configuration for
your board. Then, you can modify this configuration to your liking. In this example, we will show
how you modify the default configuration of the ``sim`` build, a build of NuttX which runs on your own
computer.

#. Initialize Board Configuration

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh -l sim:nsh
         Copy files
         Select CONFIG_HOST_LINUX=y
         Refreshing...
         
#. Build & run

    .. code-block:: console

       $ make clean
       $ make -j
       $ ./nuttx
       User Logged-in!
       nsh>

   You can explore the nsh typing ``help`` or ``?``. Then to leave you can run:

    .. code-block:: console

       nsh> quit

#. Modify configuration

   In this case we will remove the login feature (which will boot straight to the prompt). To
   do so, we use the ``menuconfig`` frontend.

    .. code-block:: console

       $ make menuconfig

   Here's what you should see:

   .. image:: ../_static/images/menuconfig.png
       :width: 800px
       :align: center
       :alt: Screenshot of menuconfig system main screen

   |br|

   The NSH Login setting is under :menuselection:`Application Configuration --> NSH Library`. You
   can use :kbd:`🢁` and :kbd:`🢃` keys to navigate and :kbd:`↵` to enter a submenu.
   To disable the corresponding setting go to :menuselection:`Console Login` and press :kbd:`spacebar` to
   it (so that it has a blank space instead of a star in it).

   Now you need to exit ``menuconfig`` and save the modified configuration. Use the :kbd:`🡸` and
   :kbd:`🡺` arrow keys to navigate the lower menu. If you select :menuselection:`Exit` you will be
   prompted to save the config.

#. Build with the new Configuration

    .. code-block:: console

       $ make

#. Run

    .. code-block:: console

       $ ./nuttx
       NuttShell (NSH) NuttX-12.10.0

   Success!

.. tip::
   If you find that message of the day (MOTD) annoying and want to turn that off, it's
   configured in :menuselection:`Application Configuration --> NSH Library --> Message of the Day (MOTD)`.
   
Fast configuration changes
==========================

If you know exactly which configuration symbol you want to change, you can use the ``kconfig-tweak`` tool (comes with the ``kconfig-frontends`` package) to quickly change a setting without going into the configuration frontend. This is useful to change settings such as debug options:

.. code-block:: console

   $ kconfig-tweak --disable CONFIG_DEBUG_NET
   $ make olddefconfig  # needed to have the kconfig system check the config
   $ kconfig-tweak --enable CONFIG_DEBUG_NET
   $ make olddefconfig

This is also useful to script configuration changes that you perform often:

.. code-block:: bash

   #!/bin/bash

   kconfig-tweak --disable CONFIG_DEBUG_ALERT
   kconfig-tweak --disable CONFIG_DEBUG_FEATURES
   kconfig-tweak --disable CONFIG_DEBUG_ERROR
   kconfig-tweak --disable CONFIG_DEBUG_WARN
   kconfig-tweak --disable CONFIG_DEBUG_INFO
   kconfig-tweak --disable CONFIG_DEBUG_ASSERTIONS
   kconfig-tweak --disable CONFIG_DEBUG_NET
   kconfig-tweak --disable CONFIG_DEBUG_NET_ERROR
   kconfig-tweak --disable CONFIG_DEBUG_NET_WARN
   kconfig-tweak --disable CONFIG_DEBUG_NET_INFO
   kconfig-tweak --disable CONFIG_DEBUG_SYMBOLS
   kconfig-tweak --disable CONFIG_DEBUG_NOOPT
   kconfig-tweak --disable CONFIG_SYSLOG_TIMESTAMP
   make oldconfig

Reference configuration
=======================

Defconfig supports the use of ``#include`` statements to reference other configuration files:

.. code-block::

   CONFIG_XXX1=y
   CONFIG_XXX2=y
   #include "configs/system.config"
   #include "configs/net.config"

The default header file search path includes:

* Current directory;
* ``${boards}/configs/common``;
* ``${boards}/common/configs``;

Merge configuration
===================

Multiple config fragments can be merged manually using the tools/merge_config.py script.

.. code-block:: console

   $ cd nuttx
   $ ./tools/merge_config.py -o defconfig .config1 .config2

License Setup
=============

NuttX includes components with various open source licenses. To use these components,
you must explicitly enable the corresponding license configuration option in the
:menuselection:`License Setup` menu:

* ``CONFIG_ALLOW_BSD_COMPONENTS`` - BSD licensed components (NFS, SPIFFS, Bluetooth LE, etc.)
* ``CONFIG_ALLOW_GPL_COMPONENTS`` - GPL/LGPL licensed components
* ``CONFIG_ALLOW_MIT_COMPONENTS`` - MIT licensed components
* ``CONFIG_ALLOW_BSDNORDIC_COMPONENTS`` - 5-Clause Nordic licensed components (NRF chips only)
* ``CONFIG_ALLOW_ECLIPSE_COMPONENTS`` - Eclipse Public License components
* ``CONFIG_ALLOW_ICS_COMPONENTS`` - ICS licensed components
* ``CONFIG_ALLOW_CUSTOM_PERMISSIVE_COMPONENTS`` - Custom permissive licensed components

.. warning::
   Please carefully review the license terms for each enabled component to ensure
   compliance with your project's licensing requirements.
