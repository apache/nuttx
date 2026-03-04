==============
``sethost.sh``
==============

Saved configurations may run on Linux, Cygwin (32- or 64-bit), or other
platforms.  The platform characteristics can be changed use 'make menuconfig'.
Sometimes this can be confusing due to the differences between the platforms.
Enter sethost.sh

sethost.sh is a simple script that changes a configuration to your host
platform. This can greatly simplify life if you use many different
configurations. For example, if you are running on Linux and you configure like
this:

.. code:: console

   $ tools/configure.sh board:configuration

The you can use the following command to both (1) make sure that the
configuration is up to date, AND (2) the configuration is set up
correctly for Linux:

.. code:: console

   $ tools/sethost.sh -l

Or, if you are on a Windows/Cygwin 64-bit platform:

.. code:: console

   $ tools/sethost.sh -c

Other options are available:

.. code:: console

   $ ./sethost.sh -h

   USAGE: ./sethost.sh [-l|m|c|g|n] [make-opts]
          ./sethost.sh -h

   Where:
     -l|m|c|g|n selects Linux (l), macOS (m), Cygwin (c),
        MSYS/MSYS2 (g) or Windows native (n). Default Linux
     make-opts directly pass to make
     -h will show this help test and terminate
