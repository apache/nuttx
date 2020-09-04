.. include:: /substitutions.rst
.. _compiling:

Compiling
=========

Now that we've installed Apache NuttX prerequisites and downloaded the source code, we are ready to compile the source code
into an executable binary file that can be run on the embedded board.

#. List Possible Apache NuttX Base Configurations

   Find your hardware and a good starting application in the list of base configurations. In the application list,
   ``nsh`` is the Apache NuttX Shell, an interactive commandline that's a good starting place if you're new.

    .. code-block:: bash

       $ cd nuttx
       $ ./tools/configure.sh -L | less

#. Initialize Configuration

   Pick one of the board:application base configuration pairs from the list, and feed it to the
   configuration script. The ``-l`` tells use that we're on Linux. macOS and Windows builds are
   possible, this guide doesn't cover them yet.

    .. code-block:: bash

       $ cd nuttx
       $ # this is the basic layout of the command:
       $ # ./tools/configure.sh -l <board-name>:<config-dir>
       $ # for example:
       $ ./tools/configure.sh -l sama5d2-xult:nsh

#. Customize Your Configuration (Optional)

   This step is optional. Right now, this is mainly to get familiar with how it works– you don't need to change
   any of the options now, but knowing how to do this will come in handy later.

   There are a lot of options. We'll cover a few of them here. Don't worry about the complexity– you don't have to use most of the options.

    .. code-block:: bash

       $ make menuconfig

#. Compile NuttX

    .. code-block:: bash

       $ make clean; make

#. Install the Executable Program on Your Board

   This step is a bit more complicated, depending on your board. It's covered in the section
   :ref:`Running Apache NuttX <running>`.

----

Next up is :ref:`running`.

