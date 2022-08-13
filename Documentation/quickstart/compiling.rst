.. include:: /substitutions.rst
.. _compiling:

=========
Compiling
=========

Now that we've installed Apache NuttX prerequisites and downloaded the source code,
we are ready to compile the source code into an executable binary file that can
be run on the embedded board.

Initialize Configuration
========================

The first step is to initialize NuttX configuration for a given board, based from
a pre-existing configuration. To list all supported configurations you can do:

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh -L | less

The output is in the format ``<board name>:<board configuration>``. You will see that
generally all boards support the ``nsh`` configuration which is a good starting point
since it enables booting into the interactive command line
:doc:`/applications/nsh/index`.

To choose a configuration you pass the ``<board name>:<board configuration>`` option
to ``configure.sh`` and indicate your host platform, such as:

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh -l stm32f4discovery:nsh

The ``-l`` tells use that we're on Linux (macOS and Windows builds are
possible). Use the ``-h`` argument to see all available options.

You can then customize this configuration by using the menu based
configuration system with:

.. code-block:: console

   $ cd nuttx
   $ make menuconfig

Modifying the configuration is covered in :doc:`configuring`.

Build NuttX
===========

We can now build NuttX. To do so, you can simply run:

  .. code-block:: console

     $ cd nuttx
     $ make

The build will complete by generating the binary outputs
inside ``nuttx`` directory. Typically this includes the ``nuttx``
ELF file (suitable for debugging using ``gdb``) and a ``nuttx.bin``
file that can be flashed to the board.

To clean the build, you can do:

  .. code-block:: console

     $ make clean

.. tip::

  To increase build speed (or of any other target such as ``clean``), you can
  pass the ``-jN`` flag to ``make``, where ``N`` is the number of parallel jobs
  to start (usually, the number of processors on your machine).

