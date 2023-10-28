====================
Compiling with CMake
====================

Initialize Configuration with CMake
===================================

The first step is to initialize NuttX configuration for a given board, based on
a pre-existing configuration. To list all supported configurations you can do:

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh -L | less

The output is in the format ``<board name>:<board configuration>``. You will see that
generally all boards support the ``nsh`` configuration which is a good starting point
since it enables booting into the interactive command line
:doc:`/applications/nsh/index`.

To choose a configuration you pass the ``<board name>:<board configuration>`` such as:

    .. code-block:: console

       $ cd nuttx
       $ cmake -B build -DBOARD_CONFIG=stm32f4discovery:nsh -GNinja

The ``-B build`` tells what is the build directory.

You can then customize this configuration by using the menu based
configuration system with:

.. code-block:: console

   $ cd nuttx
   $ cmake --build build -t menuconfig 

Modifying the configuration is covered in :doc:`configuring`.

Build NuttX with CMake
======================

We can now build NuttX. To do so, you can simply run:

  .. code-block:: console

     $ cd nuttx
     $ cmake --build build -t menuconfig 

The build will complete by generating the binary outputs
inside ``build/nuttx`` directory. Typically this includes the ``nuttx``
ELF file (suitable for debugging using ``gdb``) and a ``nuttx.bin``
file that can be flashed to the board.

To clean the build, you can do:

  .. code-block:: console

     $ cmake --build build -t clean
