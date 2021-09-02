*************************
Customizing the NuttShell
*************************

**Overview.** The NuttShell (NSH) is a simple shell application that may
be used with NuttX. It supports a variety of commands and is (very)
loosely based on the Bash shell and the common utilities used with Bash
shell programming. The paragraphs in this appendix will focus on
customizing NSH: Adding new commands, changing the initialization
sequence, etc.

The NSH Library and NSH Initialization
**************************************

**Overview.** NSH is implemented as a library that can be found at
``apps/nshlib``. As a library, it can be custom built into any
application that follows the NSH initialization sequence described
below. As an example, the code at ``apps/examples/nsh/nsh_main.c``
illustrates how to start NSH and the logic there was intended to be
incorporated into your own custom code. Although code was generated
simply as an example, in the end most people just use this example code
as their application ``main()`` function. That initialization performed
by that example is discussed in the following paragraphs.

NSH Initialization sequence
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The NSH start-up sequence is very simple. As an example, the code at
``apps/system/nsh/nsh_main.c`` illustrates how to start NSH. It simple
does the following:

  #. This function calls ``nsh_initialize()`` which initializes the NSH
     library. ``nsh_initialize()`` is described in more detail below.

  #. If the Telnetconsole is enabled, it calls ``nsh_telnetstart()`` which
     resides in the NSH library. ``nsh_telnetstart()`` will start the
     Telnet daemon that will listen for Telnet connections and start
     remote NSH sessions.

  #. If a local console is enabled (probably on a serial port), then
     ``nsh_consolemain()`` is called. ``nsh_consolemain()`` also resides
     in the NSH library. ``nsh_consolemain()`` does not return so that
     finished the entire NSH initialization sequence.

``nsh_initialize()``
~~~~~~~~~~~~~~~~~~~~

The NSH initialization function, ``nsh_initialize()``, be found in
``apps/nshlib/nsh_init.c``. It does only four things:

  #. ``nsh_romfsetc()``: If so configured, it executes NSH system init and
     start-up script that can be found at ``/etc/init.d/rc.sysinit`` and
     ``/etc/init.d/rcS`` in the target file system. The ``nsh_romfsetc()``
     function can be found in ``apps/nshlib/nsh_romfsetc.c``.
     This function will (1) register a ROMFS file system, then (2) mount
     the ROMFS file system. ``/etc`` is the default location where a
     read-only, ROMFS file system is mounted by ``nsh_romfsetc()``.

     The ROMFS image is, itself, just built into the firmware. By default,
     this ``rc.sysinit`` system init script contains the following logic::

        # Create a RAMDISK and mount it at XXXRDMOUNTPOINTXXX

        mkrd -m XXXMKRDMINORXXX -s XXMKRDSECTORSIZEXXX XXMKRDBLOCKSXXX
        mkfatfs /dev/ramXXXMKRDMINORXXX
        mount -t vfat /dev/ramXXXMKRDMINORXXX XXXRDMOUNTPOINTXXX

     Where the ``XXXX*XXXX`` strings get replaced in the template when the
     ROMFS image is created:

     -  ``XXXMKRDMINORXXX`` will become the RAM device minor number.
        Default: 0

     -  ``XXMKRDSECTORSIZEXXX`` will become the RAM device sector size

     -  ``XXMKRDBLOCKSXXX`` will become the number of sectors in the
        device.

     -  ``XXXRDMOUNTPOINTXXX`` will become the configured mount point.
        Default: ``/etc``

     By default, the substituted values would yield an ``rc.sysinit`` file like::

        # Create a RAMDISK and mount it at /tmp

        mkrd -m 1 -s 512 1024
        mkfatfs /dev/ram1
        mount -t vfat /dev/ram1 /tmp

     This script will, then:

     -  Create a RAMDISK of size 512*1024 bytes at ``/dev/ram1``,

     -  Format a FAT file system on the RAM disk at ``/dev/ram1``, and
        then

     -  Mount the FAT file system at a configured mountpoint, ``/tmp``.

     This ``rc.sysinit.template`` template file can be found at
     ``apps/nshlib/rc.sysinit.template``. The resulting ROMFS file system can be
     found in ``apps/nshlib/nsh_romfsimg.h``.

  #. ``board_app_initialize()``: Next any architecture-specific NSH
     initialization will be performed (if any). For the STM3240G-EVAL,
     this architecture specific initialization can be found at
     ``boards/arm/stm32/stm3240g-eval/src/stm32_appinit.c``. This it does
     things like: (1) Initialize SPI devices, (2) Initialize SDIO, and (3)
     mount any SD cards that may be inserted.

  #. ``nsh_netinit()``: The ``nsh_netinit()`` function can be found in
     ``apps/nshlib/nsh_netinit.c``.

  #. The start-up script ``rcS`` is executed after the system-init script
     to startup some application and other system service.

     This ``rcS`` template file can be found at
     ``apps/nshlib/rcS.template``. The resulting ROMFS file system can be
     found in ``apps/nshlib/nsh_romfsimg.h``.

NSH Commands
************

**Overview.** NSH supports a variety of commands as part of the NSH
program. All of the NSH commands are listed in the NSH documentation
`above <#cmdoverview>`__. Not all of these commands may be available at
any time, however. Many commands depend upon certain NuttX configuration
options. You can enter the help command at the NSH prompt to see the
commands actual available:

For example, if network support is disabled, then all network-related
commands will be missing from the list of commands presented by
'``nsh> help``'. You can see the specific command dependencies in the
table `above <#cmddependencies>`__.

Adding New NSH Commands
~~~~~~~~~~~~~~~~~~~~~~~

New commands can be added to the NSH very easily. You simply need to add
two things:

  #. The implementation of your command, and

  #. A new entry in the NSH command table

**Implementation of Your Command.** For example, if you want to add a
new a new command called ``mycmd`` to NSH, you would first implement the
``mycmd`` code in a function with this prototype:

.. code-block:: c

  int cmd_mycmd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

The ``argc`` and ``argv`` are used to pass command line arguments to the
NSH command. Command line parameters are passed in a very standard way:
``argv[0]`` will be the name of the command, and ``argv[1]`` through
``argv[argc-1]`` are the additional arguments provided on the NSH
command line.

The first parameter, ``vtbl``, is special. This is a pointer to
session-specific state information. You don't need to know the contents
of the state information, but you do need to pass this ``vtbl`` argument
when you interact with the NSH logic. The only use you will need to make
of the ``vtbl`` argument will be for outputting data to the console. You
don't use ``printf()`` within NSH commands. Instead you would use:

.. code-block:: c

  void nsh_output(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);

So if you only wanted to output "Hello, World!" on the console, then
your whole command implementation might be:

.. code-block:: c

  int cmd_mycmd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
  {
    nsh_output(vtbl, "Hello, World!");
    return 0;
  }

The prototype for the new command should be placed in
``apps/examples/nshlib/nsh.h``.

**Adding You Command to the NSH Command Table**. All of the commands
support by NSH appear in a single table called:

.. code-block:: c

  const struct cmdmap_s g_cmdmap[]

That table can be found in the file
``apps/examples/nshlib/nsh_parse.c``. The structure ``cmdmap_s`` is also
defined in ``apps/nshlib/nsh_parse.c``:

.. code-block:: c

  struct cmdmap_s
  {
    const char *cmd;        /* Name of the command */
    cmd_t       handler;    /* Function that handles the command */
    uint8_t     minargs;    /* Minimum number of arguments (including command) */
    uint8_t     maxargs;    /* Maximum number of arguments (including command) */
    const char *usage;      /* Usage instructions for 'help' command */
  };

This structure provides everything that you need to describe your
command: Its name (``cmd``), the function that handles the command
(``cmd_mycmd()``), the minimum and maximum number of arguments needed by
the command, and a string describing the command line arguments. That
last string is what is printed when enter "``nsh> help``".

So, for you sample command, you would add the following the to the
``g_cmdmap[]`` table:

.. code-block:: c

  { "mycmd", cmd_mycmd, 1, 1, NULL },

This entry is particularly simply because ``mycmd`` is so simple. Look
at the other commands in ``g_cmdmap[]`` for more complex examples.
