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
generally all boards support the ``nsh`` configuration which is a good sarting point
since it enables booting into the interactive command line
:doc:`/applications/nsh/index`.

To choose a configuration you pass the ``<board name>:<board configuration>`` option
to ``configure.sh`` and indicate your host platform, such as:

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh -l stm32f4discovery:nsh

The ``-l`` tells use that we're on Linux (macOS and Windows builds are
possible). Use the ``-h`` argument to see all available options.

Customize Your Configuration (Optional)
=======================================

This step is optional. Right now, this is mainly to get familiar with how it
works– you don't need to change any of the options now, but knowing how
to do this will come in handy later.

There are a lot of options. We'll cover a few of them here.
Don't worry about the complexity– you don't have to use most of the options.

.. code-block:: console

   $ cd nuttx/
   $ make menuconfig

.. todo::
  Explain some useful options.

Build NuttX
===========

We can now build NuttX. To do so, you can simply run:

  .. code-block:: console

     $ cd nuttx/
     $ make make

The build will complete by generating the binary outputs
inside `nuttx` directory. Typically this includes the `nuttx`
ELF file (suitable for debugging using `gdb`) and a `nuttx.bin`
file that can be flashed to the board.

To clean the build, you can do:

  .. code-block:: console

     $ make clean

.. warning::
  At the moment it is recommended that after modifying the
  configuration you first clean before building again. This
  is currently worked on.

----

Next up is :ref:`running`.

