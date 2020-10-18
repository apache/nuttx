.. include:: /substitutions.rst
.. _configuring:

Configuring
===========

Apache NuttX is a very configurable operating system. Nearly all features can be configured in or
out of the system. This makes it possible to compile a build tailored for your hardware and
application. It also makes configuring the system complex at times.

There is a configuration system that can be used on the commandline or in a GUI. I've found
the easiest way to configured Apache NuttX is to use the ``menuconfig`` system. This is used
via a terminal program and allows quick access to all of Apache NuttX's features via a system of
menus.

The Apache NuttX configuration system uses Linux's
`kconfig system <https://www.kernel.org/doc/Documentation/kbuild/kconfig-language.txt>`_ adapted for use with Apache
NuttX. Here's info on Linux's kconfig `menuconfig <https://en.wikipedia.org/wiki/Menuconfig>`_ system.

After you've configured your board (see :ref:`compiling`), you can use the menuconfig system
to change the configuration. Once you've configured, you can compile to make a build that
has your configuration options selected.

#. Initialize Board Configuration

   Here we'll use the simulator since that's the simplest to explain. You can do this with
   any board and base configuration.  Note here you should be supplying `configure.sh` the correct flag
   for your build environment:

    .. code-block:: bash

       -l selects the Linux (l) host environment.
       -m selects the macOS (m) host environment.
       -c selects the Windows host and Cygwin (c) environment.
       -g selects the Windows host and MinGW/MSYS environment.
       -n selects the Windows host and Windows native (n) environment.

   Select the simulator configuration for a Linux host:

    .. code-block:: bash

       $ cd nuttx
       $ make distclean  # make a clean start, clearing out old configurations
       $ ./tools/configure.sh -l sim:nsh
         Copy files
         Select CONFIG_HOST_LINUX=y
         Refreshing...
#. Make

    .. code-block:: bash

       $ make clean; make
       $ ./nuttx
       login:

   From another terminal window, kill the simulator:

    .. code-block:: bash

       $ pkill nuttx

#. Menu Configuration

   Showing that ``login:`` is annyoing. Let's use the ``menuconfig`` system to turn it off.

    .. code-block:: bash

       $ make menuconfig

   Here's what you should see:

   .. image:: ../_static/images/menuconfig.png
       :width: 800px
       :align: center
       :alt: Screenshot of menuconfig system main screen

   |br|

#. Application Configuration

   The NSH Login setting is under ``Application Configuration > NSH Library``. Use
   the up and down arrows to navigate to ``Application Configuration``; hit ``<return>`` to
   select it. Now you're in the ``Application Configuration`` menu. Use the arrows to go
   down to ``NSH Library`` and select that. Now navigate down to ``Console Login`` and use
   the spacebar to uncheck that setting (so that it has a blank space instead of a star in it).

   Now let's save. Use the right and left arrow keys to select the ``Exit`` menu item at the
   bottom of the screen. Hit ``<return>`` to select it, hit ``<return>`` again, and again, finally
   hitting ``<return>`` in the ``Save Configuration`` dialog box.

#. Make the New Configuration

    .. code-block:: bash

       $ make clean; make

#. Run

    .. code-block:: bash

       $ ./nuttx
       NuttShell (NSH) NuttX-8.2
       MOTD: username=admin password=Administrator

   Success!

   If you find that message of the day (MOTD) annoying and want to turn that off, it's
   configured in ``Application Configuration > NSH Library >> Message of the Day (MOTD)``.

----

Next up is :ref:`debugging`.
