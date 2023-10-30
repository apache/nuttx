==============================
Customizing NSH Initialization
==============================

**Ways to Customize NSH Initialization**. There are three ways to
customize the NSH start-up behavior. Here they are presented in order of
increasing difficulty:

  #. You can extend the initialization logic in
     ``boards/arm/stm32/stm3240g-eval/src/stm32_appinit.c``. The logic
     there is called each time that NSH is started and is good place in
     particular for any device-related initialization.

  #. You replace the sample code at ``apps/examples/nsh/nsh_main.c`` with
     whatever start-up logic that you want. NSH is a library at
     ``apps/nshlib``. ``apps.examples/nsh`` is just a tiny, example
     start-up function (``CONFIG_INIT_ENTRYPOINT``\ ()) that runs
     immediately and illustrates how to start NSH If you want something
     else to run immediately then you can write your write your own custom
     ``CONFIG_INIT_ENTRYPOINT``\ () function and then start other tasks
     from your custom ``CONFIG_INIT_ENTRYPOINT``\ ().

  #. NSH also supports a start-up script that executed when NSH first
     runs. This mechanism has the advantage that the start-up script can
     contain any NSH commands and so can do a lot of work with very little
     coding. The disadvantage is that is is considerably more complex to
     create the start-up script. It is sufficiently complex that is
     deserves its own paragraph

NuttShell Start up Scripts
--------------------------

First of all you should look at `NSH Start-Up Script <#startupscript>`__
paragraph. Most everything you need to know can be found there. That
information will be repeated and extended here for completeness.

**NSH Start-Up Script**. NSH supports options to provide a start up
script for NSH. The start-up script contains any command support by NSH
(i.e., that you see when you enter 'nsh> help'). In general this
capability is enabled with ``CONFIG_NSH_ROMFSETC=y``, but has several
other related configuration options as described with the `NSH-specific
configuration settings <#nshconfiguration>`__ paragraph. This capability
also depends on:

  -  ``CONFIG_DISABLE_MOUNTPOINT=n``. If mount point support is disabled,
     then you cannot mount *any* file systems.

  -  ``CONFIG_FS_ROMFS`` enabled. This option enables ROMFS file system
     support.

**Default Start-Up Behavior**. The implementation that is provided is
intended to provide great flexibility for the use of Start-Up files.
This paragraph will discuss the general behavior when all of the
configuration options are set to the default values.

In this default case, enabling ``CONFIG_NSH_ROMFSETC`` will cause NSH to
behave as follows at NSH start-up time:

  -  NSH will create a read-only RAM disk (a ROM disk), containing a tiny
     ROMFS file system containing the following::

      `--init.d/
          `-- rcS
          `-- rc.sysinit

     Where ``rcS`` is the NSH start-up script.
     Where ``rc.sysinit`` is the NSH system-init script.

  -  NSH will then mount the ROMFS file system at ``/etc``, resulting in::

      |--dev/
      |   `-- ram0
      `--etc/
          `--init.d/
              `-- rcS
              `-- rc.sysinit

  -  By default, the contents of ``rc.sysinit`` script are::

      # Create a RAMDISK and mount it at /tmp

      mkrd -m 1 -s 512 1024
      mkfatfs /dev/ram1
      mount -t vfat /dev/ram1 /tmp

  -  NSH will execute the script at ``/etc/init.d/rc.sysinit`` at system
     init (before the first NSH prompt). After execution of the script,
     the root FS will look like::

      |--dev/
      |   |-- ram0
      |   `-- ram1
      |--etc/
      |   `--init.d/
      |       `-- rcS
      |       `-- rc.sysinit
      `--tmp/

**Example Configurations**. Here are some configurations that have
``CONFIG_NSH_ROMFSETC=y`` in the NuttX configuration file. They might
provide useful examples:

  -  ``boards/arm/stm32/hymini-stm32v/nsh2``
  -  ``boards/arm/dm320/ntosd-dm320/nsh``
  -  ``boards/sim/sim/sim/nsh``
  -  ``boards/sim/sim/sim/nsh2``
  -  ``boards/sim/sim/sim/nx``
  -  ``boards/sim/sim/sim/nx11``
  -  ``boards/sim/sim/sim/touchscreen``

In most of these cases, the configuration sets up the *default*
``/etc/init.d/rc.sysinit`` and ``/etc/init.d/rcS`` script. The default
script is here: ``apps/nshlib/rc.sysinit.template`` and
``apps/nshlib/rcS.template``. (The funny values in the rc.sysinit.template
like ``XXXMKRDMINORXXX`` get replaced via ``sed`` at build time). This
default configuration creates a ramdisk and mounts it at ``/tmp`` as
discussed above.

If that default behavior is not what you want, then you can provide your
own custom ``rc.sysinit`` and ``rcS`` script by defining
``CONFIG_NSH_ARCHROMFS=y`` in the configuration file.

**Modifying the ROMFS Image**. The contents of the ``/etc`` directory
are retained in the file ``apps/nshlib/nsh_romfsimg.h`` OR, if
``CONFIG_NSH_ARCHROMFS`` is defined,
``include/arch/board/nsh_romfsimg.h``. In order to modify the start-up
behavior, there are three things to study:

  #. **Configuration Options.** The additional ``CONFIG_NSH_ROMFSETC``
     configuration options discussed with the other `NSH-specific
     configuration settings <#nshconfiguration>`__.

  #. ``tools/mkromfsimg.sh`` **Script**. The script
     ``tools/mkromfsimg.sh`` creates ``nsh_romfsimg.h``. It is not
     automatically executed. If you want to change the configuration
     settings associated with creating and mounting the ``/tmp``
     directory, then it will be necessary to re-generate this header file
     using the ``tools/mkromfsimg.sh`` script.

     The behavior of this script depends upon several things:

     #. The configuration settings then installed configuration.

     #. The ``genromfs`` tool(available from
        `http://romfs.sourceforge.net <http://romfs.sourceforge.net/>`__)
        or included within the NuttX buildroot toolchain. There is also a
        snapshot available in the NuttX tools repository
        `here <https://bitbucket.org/nuttx/tools/src/master/genromfs-0.5.2.tar.gz>`__.

     #. The ``xxd`` tool that is used to generate the C header files (xxd
        is a normal part of a complete Linux or Cygwin installation,
        usually as part of the ``vi`` package).

     #. The file ``apps/nshlib/rc.sysinit.template`` (OR, if
        ``CONFIG_NSH_ARCHROMFS`` is defined
        ``include/arch/board/rc.sysinit.template``.

        The file ``apps/nshlib/rcS.template`` (OR, if
        ``CONFIG_NSH_ARCHROMFS`` is defined
        ``include/arch/board/rcs.template``.

  #. ``rc.sysinit.template``. The file ``apps/nshlib/rc.sysinit.template``
     contains the general form of the ``rc.sysinit`` file; configured values
     are plugged into this template file to produce the final ``rc.sysinit`` file.

     ``rcS.template``. The file ``apps/nshlib/rcS.template`` contains the
     general form of the ``rcS`` file; configured values are plugged into
     this template file to produce the final ``rcS`` file.

     To generate a custom ``rc.sysinit`` and ``rcS`` file a copy of
     ``rc.sysinit.template`` and ``rcS.template`` needs to
     be placed at ``tools/`` and changed according to the desired start-up
     behaviour. Running ``tools/mkromfsimg.h`` creates ``nsh_romfsimg.h``
     which needs to be copied to ``apps/nshlib`` OR if
     ``CONFIG_NSH_ARCHROMFS`` is defined to
     ``boards/<arch>/<chip>/<board>/include``.

``rc.sysinit.template``. The default ``rc.sysinit.template``,
``apps/nshlib/rc.sysinit.template``, generates the standard, default
``apps/nshlib/nsh_romfsimg.h`` file.

``rcS.template``. The default ``rcS.template``,
``apps/nshlib/rcS.template``, generates the standard, default
``apps/nshlib/nsh_romfsimg.h`` file.

If ``CONFIG_NSH_ARCHROMFS`` is defined in the NuttX configuration file,
then a custom, board-specific ``nsh_romfsimg.h`` file residing in
``boards/<arch>/<chip>/<board>/include``\ will be used. NOTE when the OS
is configured, ``include/arch/board`` will be linked to
``boards/<arch>/<chip>/<board>/include``.

All of the startup-behavior is contained in ``rc.sysinit.template`` and
``rcS.template``. The role of ``mkromfsimg.sh`` script is to (1) apply
the specific configuration settings to ``rc.sysinit.template`` to create
the final ``rc.sysinit``, and ``rcS.template`` to create the final ``rcS``,
and (2) to generate the header file ``nsh_romfsimg.h`` containing the ROMFS file
system image. To do this, ``mkromfsimg.sh`` uses two tools that must be
installed in your system:

  #. The ``genromfs`` tool that is used to generate the ROMFS file system
     image.

  #. The ``xxd`` tool that is used to create the C header file.
