.. include:: /substitutions.rst
.. _nsh:

********
Overview
********

**The NSH Library**. The ``apps/nshlib`` sub-directory contains the
NuttShell (NSH) library. This library can easily to linked to
produce a NSH application (See as an example
``apps/examples/nsh``). The NSH Library provides a simple shell
application for NuttX.

Console/NSH Front End
*********************

**NSH Consoles**. Using settings in the configuration file, NSH may be
configured to use (1) the serial stdin/out, (2) a USB serial
device (such as CDC/ACM), or (3) a telnet connection as the
console. Or, perhaps even all at once since or BOTH. An indefinite
number of telnet sessions are supported.

**Start-Up prompt**. When NSH is started, you will see the a welcome
message such the following on the selected console:

.. code-block::

  NuttShell (NSH)
  nsh>

The greeting may also include NuttX versioning information if you
are using a versioned copy of NuttX. ``nsh>`` is the NSH prompt
and indicates that you may enter a command from the console.

**USB console startup**. When using a USB console, the start-up
sequence differs a little: In this case, you are required to press
*ENTER* three times. Then NSH prompt will appear as described
above. This is required for the following reasons:

  #. This assures that the USB connection is stable. The USB
     connection may be made, broken, and re-established a few times
     if the USB cable is not yet fully seated. Waiting for *ENTER*
     to be pressed three times assures that the connection is
     stable.
  #. The establishment of the connection is two step process: First,
     the USB serial connection is made with the host PC. Then the
     application that uses the serial interface is started on the
     host. When the serial connection is established on the host,
     the host operating system may send several *AT* modem commands
     to the host depending upon how the host serial port is
     configured. By waiting for *ENTER* to be pressed three
     consecutive times, all of these modem commands will go to the
     bit-bucket and will not be interpreted as NSH command input.
  #. Similarly, in the second step when the applications is started,
     there may be additional *AT* modem commands sent out the serial
     port. Most serial terminal programs will do this unless they
     are specifically configured to suppress the modem command
     output. Waiting for the *ENTER* input eliminates the invalid
     command errors from both (2) and (3).
  #. Finally, if NSH did not wait for some positive indication that
     the serial terminal program is up and running, then the output
     of the NSH greeting and initial NSH prompt would be lost.

**Extended Command Line Editing**. By default, NuttX uses a simple
command line editor that allows command entry after the ``nsh>``
and supports only the *backspace* key for editing. However, a more
complete command line editor can be selected by setting
``CONFIG_NSH_CLE=y`` in the NuttX configuration file. When that
option is selected, the following EMACS-like line editing commands
are supported:

=====================  ================================================
Key Binding            Editor Action
=====================  ================================================
``^A``                 Move cursor to start of the line
``^B``                 Move left one character
``^D`` or *Del*        Delete a single character at the cursor position
``^E``                 Move cursor to end of current line
``^F``                 Move right one character
``^H`` or *Backspace*  Delete character, left (backspace)
``^K``                 Delete to the end of the line
``^U``                 Delete the entire line
=====================  ================================================

Command Overview
****************

**Simple, Re-directed, and Background Commands**. The NuttShell
(NSH) is a simple shell application. NSH supports the following
commands forms:

===============================   ======================================
Simple command                    ``<cmd>``
Command with re-directed output   ``<cmd> > <file> <cmd> >> <file>``
Background command                ``<cmd> &``
Re-directed background command    ``<cmd> > <file> & <cmd> >> <file> &``
===============================   ======================================

Where:

  * ``<cmd>`` is any one of the simple commands listed later.
  * ``<file>`` is the full or relative path to any writable object in the file system name space (file or character driver). Such objects will be referred to simply as files throughout this document.

``nice`` **'d Background Commands**. NSH executes at the
mid-priority (128). Backgrounded commands can be made to execute
at higher or lower priorities using ``nice``::

  [nice [-d <niceness>>]] <cmd> [> <file>|>> <file>] [&]

Where ``<niceness>`` is any value between -20 and 19 where lower
(more negative values) correspond to higher priorities. The
default niceness is 10.

**Multiple commands per line**. NSH will accept multiple commands
per command line with each command separated with the semi-colon
character (;).

**Optional Syntax Extensions** Because these features commit
significant resources, they are disabled by default.

  -  ``CONFIG_NSH_CMDPARMS``: If selected, then the output from
     commands, from file applications, and from NSH built-in
     commands can be used as arguments to other commands. The entity
     to be executed is identified by enclosing the command line in
     back quotes. For example,

     .. code-block:: bash

       set FOO myprogram $BAR

     Will execute the program named ``myprogram`` passing it the
     value of the environment variable ``BAR``. The value of the
     environment variable ``FOO`` is then set output of
     ``myprogram`` on ``stdout``.

  -  ``CONFIG_NSH_ARGCAT``: Support concatenation of strings
     with environment variables or command output. For example:

     .. code-block:: bash

       set FOO XYZ
       set BAR 123
       set FOOBAR ABC_${FOO}_${BAR}

     would set the environment variable ``FOO`` to ``XYZ``, ``BAR``
     to ``123`` and ``FOOBAR`` to ``ABC_XYZ_123``. If
     ``CONFIG_NSH_ARGCAT`` is not selected, then a slightly smaller
     FLASH footprint results but then also only simple environment
     variables like ``$FOO`` can be used on the command line.

  -  ``CONFIG_NSH_QUOTE``: Enables back-slash quoting of certain
     characters within the command. This option is useful for the
     case where an NSH script is used to dynamically generate a new
     NSH script. In that case, commands must be treated as simple
     text strings without interpretation of any special characters.
     Special characters such as ``$``, :literal:`\``, ``"``, and
     others must be retained intact as part of the test string. This
     option is currently only available is ``CONFIG_NSH_ARGCAT`` is
     also selected.

Conditional Command Execution
*****************************

An ``if-then[-else]-fi`` construct is also supported in order to
support conditional execution of commands. This works from the
command line but is primarily intended for use within NSH scripts
(see the ```sh`` <#cmdsh>`__ command). The syntax is as follows:

.. code-block:: bash

  if [!] <cmd>
  then
    [sequence of <cmd>]
  else
    [sequence of <cmd>]
  fi

Where ``<cmd>`` is a `simple command <#cmdoverview>`__. The
command success value of zero is treated true; a non-zero command
failure value is treated false. The ```test`` <#cmdtest>`__
command is frequently used for comparisons.

Looping
*******

**Looping Constructs**. ``while-do-done`` and ``until-do-done``
looping constructs are also supported. These work from the command
line but are primarily intended for use within NSH scripts (see
the ```sh`` <#cmdsh>`__ command).

  - ``while-do-done``: Execute ``[sequence of <cmd>]`` as long
    as ``<cmd>`` has an exit status of zero. The syntax is as
    follows:

    .. code-block:: bash

      while <cmd>
      do
        [sequence of <cmd>]
      done

  - ``until-do-done``: Execute ``[sequence of <cmd>]`` as long
    as ``<cmd>`` has a non-zero exit status. The syntax is as
    follows:

    .. code-block::

      until <cmd>
      do
        [sequence of <cmd>]
      done

Where ``<cmd>`` is a `simple command <#cmdoverview>`__. The
command success value of zero is treated true; a non-zero command
failure value is treated false. The ```test`` <#cmdtest>`__
command is frequently used for comparisons.

**The** ``break`` **Command**. A ``break`` command is also supported.
The ``break`` command is only meaningful within the body of the a
``while`` or ``until`` loop, between the ``do`` and ``done`` tokens.
If the ``break`` command is executed within the body of a loop,
the loop will immediately terminate and execution will continue
with the next command immediately following the ``done`` token.

Built-In Variables
******************

======  ====================================================
``$?`` 	The result of the last simple command execution. |br|
        On backgrounded commands, this variable holds only |br|
        the result of spawning the background command.
======  ====================================================

Current Working Directory
*************************

``cd`` **and** ``pwd``. All path arguments to commands may be
either an absolute path or a path relative to the current working
directory. The current working directory is set using the
``cd`` command and can be queried either by using the
``pwd`` command or by using the ``echo $PWD`` command.

Environment Variables
*********************

==========  ================================================
``PATH``    The default path in the file systems to look |br|
            for executable, binary programs working directory
``PWD``	    The current working directory
``OLDPWD``  The previous working directory
==========  ================================================

NSH Start-Up Script
*******************

**NSH Start-Up Script**. NSH supports options to provide a start
up script for NSH. In general this capability is enabled with
``CONFIG_NSH_ROMFSETC``, but has several other related
configuration options as described with the `NSH-specific
configuration settings <#nshconfiguration>`__. This capability
also depends on:

  -  ``CONFIG_DISABLE_MOUNTPOINT`` not set
  -  ``CONFIG_NFILE_DESCRIPTORS`` > 4
  -  ``CONFIG_FS_ROMFS`` enabled

**Default Start-Up Behavior**. The implementation that is provided
is intended to provide great flexibility for the use of Start-Up
files. This paragraph will discuss the general behavior when all
of the configuration options are set to the default values.

In this default case, enabling ``CONFIG_NSH_ROMFSETC`` will cause
NSH to behave as follows at NSH startup time:

  -  NSH will create a read-only RAM disk (a ROM disk), containing a
     tiny ROMFS file system containing the following::

       `--init.d/
            `-- rcS

     Where rcS is the NSH start-up script.

  -  NSH will then mount the ROMFS file system at ``/etc``,
     resulting in::

       |--dev/
       |   `-- ram0
       `--etc/
           `--init.d/
               `-- rcS

  -  By default, the contents of rcS script are::

       # Create a RAMDISK and mount it at XXXRDMOUNTPOINTXXX

       mkrd -m 1 -s 512 1024
       mkfatfs /dev/ram1
       mount -t vfat /dev/ram1 /tmp

  -  NSH will execute the script at ``/etc/init.d/rcS`` at start-up
     (before the first NSH prompt). After execution of the script,
     the root FS will look like::

        |--dev/
        |   |-- ram0
        |   `-- ram1
        |--etc/
        |   `--init.d/
        |       `-- rcS
        `--tmp/

**Modifying the ROMFS Image**. The contents of the ``/etc``
directory are retained in the file ``apps/nshlib/nsh_romfsimg.h``
OR, if ``CONFIG_NSH_ARCHROMFS`` is defined,
``include/arch/board/rcs.template``). In order to modify the
start-up behavior, there are three things to study:

  #. **Configuration Options.** The additional
     ``CONFIG_NSH_ROMFSETC`` configuration options discussed with
     the other `NSH-specific configuration
     settings <#nshconfiguration>`__.
  #. `tools/mkromfsimg.sh`` **Script**. The script
     ``tools/mkromfsimg.sh`` creates ``nsh_romfsimg.h``. It is not
     automatically executed. If you want to change the configuration
     settings associated with creating and mounting the ``/tmp``
     directory, then it will be necessary to re-generate this header
     file using the ``tools/mkromfsimg.sh`` script.

     The behavior of this script depends upon three things:

     -  The configuration settings then installed configuration.
     -  The ``genromfs`` tool (available from
        http://romfs.sourceforge.net).
     -  The file ``apps/nshlib/rcS.template`` (OR, if
        ``CONFIG_NSH_ARCHROMFS`` is defined
        ``include/arch/board/rcs.template``.

  #. ``rcS.template``: The file ``apps/nshlib/rcS.template``
     contains the general form of the ``rcS`` file; configured
     values are plugged into this template file to produce the final
     ``rcS`` file.

.. note::
  ``apps/nshlib/rcS.template`` generates the standard,
  default ``nsh_romfsimg.h`` file. If ``CONFIG_NSH_ARCHROMFS`` is
  defined in the NuttX configuration file, then a custom,
  board-specific ``nsh_romfsimg.h`` file residing in the
  ``boards/<arch>/<chip>/<board>/include`` directory will be used.
  NOTE when the OS is configured, ``include/arch/board`` will be
  linked to ``boards/<arch>/<chip>/<board>/include``.

All of the startup-behavior is contained in ``rcS.template``. The
role of ``mkromfsimg.sh`` is to (1) apply the specific
configuration settings to ``rcS.template`` to create the final
``rcS``, and (2) to generate the header file ``nsh_romfsimg.h``
containing the ROMFS file system image.

**Further Information**. See the section on `Customizing the
NuttShell <#customizingnsh>`__ for additional, more detailed
information about the NSH start-up script and how to modify it.


