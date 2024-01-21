=========================
etc romfs
=========================
The ROMFS image is the contents of the ``/etc`` directory, including the start-up script
contains any command support by Nuttx, and other customized contents needed.

Configuration
=============

.. code-block:: console

  CONFIG_NSH_ROMFS          /* Mount a ROMFS file system at "/etc" and provide a system init script at
                               "/etc/init.d.rc.sysinit" and a startup script at "etc/init.d/rcS". */
  CONFIG_ETC_ROMFSMOUNTPT   /* The default mountpoint for the ROMFS volume is "/etc", but that can be
                               changed with this setting.  This must be a absolute path beginning with '/'
                               and enclosed in quotes. */
  CONFIG_ETC_ROMFSDEVNO     /* This is the minor number of the ROMFS block device. The default is '0'
                               corresponding to "/dev/ram0". */
  CONFIG_ETC_ROMFSSECTSIZE  /* This is the sector size to use with the ROMFS volume. Since the default volume
                               is very small, this defaults to 64 but should be increased if the ROMFS volume
                               were to be become large. Any value selected must be a power of 2. */

This capability also depends on:

.. code-block:: console

  CONFIG_DISABLE_MOUNTPOINT  /* If mount point support is disabled, then you cannot mount any file systems. */
  CONFIG_FS_ROMFS            /* This option enables ROMFS file system support. */

Start up Scripts
================

**Start-Up Script**. The start-up script contains any command support by Nuttx
(i.e., that you see when you enter 'nsh> help'). The implementation that is provided is
intended to provide great flexibility for the use of Start-Up files.
This paragraph will discuss the general behavior when all of the
configuration options are set to the default values.

In this default case, enabling ``CONFIG_ETC_ROMFS`` will cause Nuttx to
behave as follows at Nuttx start-up time:

  -  Nuttx will create a read-only RAM disk (a ROM disk), containing a tiny
     ROMFS file system containing the following::

      `--init.d/
          `-- rcS
          `-- rc.sysinit

     Where ``rcS`` is the start-up script.
     Where ``rc.sysinit`` is the system-init script.

  -  Nuttx will then mount the ROMFS file system at ``/etc``, resulting in::

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
``CONFIG_ETC_ROMFS=y`` in the NuttX configuration file. They might
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

Customizing Start up Scripts
============================

In order to modify the start-up behavior, there are three things to study:

  #. **Configuration Options.** The additional ``CONFIG_ETC_ROMFS``
     configuration options discussed with `Configuration`

  #. ``tools/mkromfsimg.sh`` **Script**. The script
     ``tools/mkromfsimg.sh`` creates ``etc_romfs.c``. It is not
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

     #. The file ``include/arch/board/rc.sysinit.template`` and
        the file ``include/arch/board/rcs.template``

  #. ``rc.sysinit.template``. The file ``apps/nshlib/rc.sysinit.template``
     contains the general form of the ``rc.sysinit`` file; configured values
     are plugged into this template file to produce the final ``rc.sysinit`` file.

     ``rcS.template``. The file ``apps/nshlib/rcS.template`` contains the
     general form of the ``rcS`` file; configured values are plugged into
     this template file to produce the final ``rcS`` file.

     To generate a custom ``rc.sysinit`` and ``rcS`` file a copy of
     ``rc.sysinit.template`` and ``rcS.template`` needs to
     be placed at ``tools/`` and changed according to the desired start-up
     behaviour. Running ``tools/mkromfsimg.h`` creates ``etc_romfs.c``
     which needs to be copied to ``arch/board/src`` and compiled in Makefile

All of the startup-behavior is contained in ``rc.sysinit.template`` and
``rcS.template``. The role of ``mkromfsimg.sh`` script is to (1) apply
the specific configuration settings to ``rc.sysinit.template`` to create
the final ``rc.sysinit``, and ``rcS.template`` to create the final ``rcS``,
and (2) to generate the source file ``etc_romfs.c`` containing the ROMFS file
system image. To do this, ``mkromfsimg.sh`` uses two tools that must be
installed in your system:

  #. The ``genromfs`` tool that is used to generate the ROMFS file system
     image.

  #. The ``xxd`` tool that is used to create the C header file.

Customizing ROMFS Image
=======================

The ROMFS image can be generated from the content in the corresponding
``board/arch/board/board/src/etc`` directory, and added by Makefile.

**Example Configurations**. Here are some configurations that have
``CONFIG_ETC_ROMFS=y`` in the NuttX configuration file. They might
provide useful examples:

  -  ``boards/risc-v/bl808/ox64/src/etc``
  -  ``boards/risc-v/qemu-rv/rv-virt/src/etc``
  -  ``boards/risc-v/esp32c3/esp32c3-devkit/src/etc``
  -  ``boards/risc-v/k230/canmv230/src/etc``
  -  ``boards/risc-v/jh7110/star64/src/etc``
  -  ``boards/arm64/rk3399/nanopi_m4/src/etc``
  -  ``boards/sim/sim/sim/src/etc``
