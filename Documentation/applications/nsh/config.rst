.. |br| raw:: html

   <br/>

======================
Configuration Settings
======================

The availability of the above commands depends upon features that may or
may not be enabled in the NuttX configuration file. The following
:ref:`cmdtable <cmddependencies>` indicates the dependency of each command on
NuttX configuration settings. General configuration settings are
discussed in the NuttX Porting Guide.
Configuration settings specific to NSH as discussed at the
:ref:`cmdbottom <nshconfiguration>` of this document.

Note that in addition to general NuttX configuration settings, each NSH
command can be individually disabled via the settings in the rightmost
column. All of these settings make the configuration of NSH potentially
complex but also allow it to squeeze into very small memory footprints.

.. _cmddependencies:

Command Dependencies on Configuration Settings
==============================================

====================== =========================================== ======================
Command                Depends on Configuration                    Can Be Disabled with
====================== =========================================== ======================
``[``                  ! ``CONFIG_NSH_DISABLESCRIPT``              ``CONFIG_NSH_DISABLE_TEST``
:ref:`cmdaddroute`     ``CONFIG_NET`` && ``CONFIG_NET_ROUTE``      ``CONFIG_NSH_DISABLE_ADDROUTE``
:ref:`cmdarp`          ``CONFIG_NET`` && ``CONFIG_NET_ARP``        ``CONFIG_NSH_DISABLE_ARP``
:ref:`cmdbase64dec`    ``CONFIG_NETUTILS_CODECS`` &&               ``CONFIG_NSH_DISABLE_BASE64DEC``
                       ``CONFIG_CODECS_BASE64``
:ref:`cmdbase64enc`    ``CONFIG_NETUTILS_CODECS`` &&               ``CONFIG_NSH_DISABLE_BASE64ENC``
                       ``CONFIG_CODECS_BASE64``
:ref:`cmdbasename`     .                                           ``CONFIG_NSH_DISABLE_BASENAME``
:ref:`cmdbreak`        ! ``CONFIG_NSH_DISABLESCRIPT`` &&           .
                       ! ``CONFIG_NSH_DISABLE_LOOPS``  
:ref:`cmdcat`          ``CONFIG_NSH_DISABLE_CAT``                  .
:ref:`cmdcd`           ! ``CONFIG_DISABLE_ENVIRON``                ``CONFIG_NSH_DISABLE_CD``
:ref:`cmdcmp`          ``CONFIG_NSH_DISABLE_CMP``                  .
:ref:`cmdcp`           ``CONFIG_NSH_DISABLE_CP``                   .
:ref:`cmddate`         ``CONFIG_NSH_DISABLE_DATE``                 .
:ref:`cmddd`           ``CONFIG_NSH_DISABLE_DD``                   .
:ref:`cmddelroute`     ``CONFIG_NET`` && ``CONFIG_NET_ROUTE``      ``CONFIG_NSH_DISABLE_DELROUTE``
:ref:`cmddf`           ! ``CONFIG_DISABLE_MOUNTPOINT``             ``CONFIG_NSH_DISABLE_DF``
:ref:`cmddirname`      ``CONFIG_NSH_DISABLE_DIRNAME``              .
:ref:`cmddmesg`        ``CONFIG_RAMLOG_SYSLOG``                    ``CONFIG_NSH_DISABLE_DMESG``
:ref:`cmdecho`         ``CONFIG_NSH_DISABLE_ECHO``                 .
:ref:`cmdenv`          ``CONFIG_FS_PROCFS`` &&                     ``CONFIG_NSH_DISABLE_ENV``
                       ! ``CONFIG_DISABLE_ENVIRON`` && |br|
                       ! ``CONFIG_PROCFS_EXCLUDE_ENVIRON``
:ref:`cmdexec`         ``CONFIG_NSH_DISABLE_EXEC``                 .
:ref:`cmdexit`         ``CONFIG_NSH_DISABLE_EXIT``                 .
:ref:`cmdexport`       ``CONFIG_NSH_VARS`` &&
                       ! ``CONFIG_DISABLE_ENVIRON``                ``CONFIG_NSH_DISABLE_EXPORT``
:ref:`cmdfree`         ``CONFIG_NSH_DISABLE_FREE``                 .
:ref:`cmdget`          ``CONFIG_NET`` && ``CONFIG_NET_UDP`` &&      ``CONFIG_NSH_DISABLE_GET``
                       *MTU* >= 58\ [#1]_
:ref:`cmdhelp`  [#3]_  ``CONFIG_NSH_DISABLE_HELP``                 .
:ref:`cmdhexdump`      ``CONFIG_NSH_DISABLE_HEXDUMP``              .
:ref:`cmdifconfig`     ``CONFIG_NET`` && ``CONFIG_FS_PROCFS`` &&    ``CONFIG_NSH_DISABLE_IFCONFIG``
                       ! ``CONFIG_FS_PROCFS_EXCLUDE_NET``
:ref:`cmdifdown`       ``CONFIG_NET`` && ``CONFIG_FS_PROCFS`` &&   ``CONFIG_NSH_DISABLE_IFUPDOWN``
                       ! ``CONFIG_FS_PROCFS_EXCLUDE_NET``
:ref:`cmdifup`         ``CONFIG_NET`` && ``CONFIG_FS_PROCFS`` &&
                       ! ``CONFIG_FS_PROCFS_EXCLUDE_NET``          ``CONFIG_NSH_DISABLE_IFUPDOWN``
:ref:`cmdinsmod`       ``CONFIG_MODULE``                           ``CONFIG_NSH_DISABLE_MODCMDS``
:ref:`cmdirqinfo`      ! ``CONFIG_DISABLE_MOUNTPOINT`` &&          .
                       ``CONFIG_FS_PROCFS`` && |br|
                       ``CONFIG_SCHED_IRQMONITOR``
:ref:`cmdkill`         ``CONFIG_NSH_DISABLE_KILL``                 .
:ref:`cmdlosetup`      ! ``CONFIG_DISABLE_MOUNTPOINT`` &&          ``CONFIG_NSH_DISABLE_LOSETUP``
                       ``CONFIG_DEV_LOOP``
:ref:`cmdln`           ``CONFIG_PSEUDOFS_SOFTLINKS``               ``CONFIG_NSH_DISABLE_LN``
:ref:`cmdls`           ``CONFIG_NSH_DISABLE_LS``                   .
:ref:`cmdlsmod`        ``CONFIG_MODULE`` && ``CONFIG_FS_PROCFS``   ``CONFIG_NSH_DISABLE_MODCMDS``
                       && |br|
                       ! ``CONFIG_FS_PROCFS_EXCLUDE_MODULE``
:ref:`cmdmd5`          ``CONFIG_NETUTILS_CODECS`` &&               ``CONFIG_NSH_DISABLE_MD5``
                       ``CONFIG_CODECS_HASH_MD5``
:ref:`cmdmx`           .                                           ``CONFIG_NSH_DISABLE_MB``, |br|
                                                                   ``CONFIG_NSH_DISABLE_MH``, |br|
                                                                   ``CONFIG_NSH_DISABLE_MW``
:ref:`cmdmkdir`        (! ``CONFIG_DISABLE_MOUNTPOINT`` \|\|       ``CONFIG_NSH_DISABLE_MKDIR``
                       ! ``CONFIG_DISABLE_PSEUDOFS_OPERATIONS``)
:ref:`cmdmkfatfs`      ! ``CONFIG_DISABLE_MOUNTPOINT`` &&          ``CONFIG_NSH_DISABLE_MKFATFS``
                       ``CONFIG_FSUTILS_MKFATFS``
:ref:`cmdmkfifo`       ``CONFIG_PIPES`` &&                         ``CONFIG_NSH_DISABLE_MKFIFO``
                       ``CONFIG_DEV_FIFO_SIZE`` > 0
:ref:`cmdmkrd`         ! ``CONFIG_DISABLE_MOUNTPOINT``             ``CONFIG_NSH_DISABLE_MKRD``
:ref:`cmdmount`        ! ``CONFIG_DISABLE_MOUNTPOINT``             ``CONFIG_NSH_DISABLE_MOUNT``
:ref:`cmdmv`           ! ``CONFIG_DISABLE_MOUNTPOINT`` \|\|        ``CONFIG_NSH_DISABLE_MV``
                       ! ``CONFIG_DISABLE_PSEUDOFS_OPERATIONS``
:ref:`cmdnfsmount`     ! ``CONFIG_DISABLE_MOUNTPOINT`` &&          ``CONFIG_NSH_DISABLE_NFSMOUNT``
                       ``CONFIG_NET`` && ``CONFIG_NFS``
:ref:`cmdnslookup`     ``CONFIG_LIBC_NETDB`` &&                    ``CONFIG_NSH_DISABLE_NSLOOKUP``
                       ``CONFIG_NETDB_DNSCLIENT``
:ref:`cmdpasswd`       ! ``CONFIG_DISABLE_MOUNTPOINT`` &&          ``CONFIG_NSH_DISABLE_PASSWD``
                       ``CONFIG_NSH_LOGIN_PASSWD``
:ref:`cmdpmconfig`     ``CONFIG_PM``                               ``CONFIG_NSH_DISABLE_PMCONFIG``
:ref:`cmdpoweroff`     ``CONFIG_BOARDCTL_POWEROFF``                ``CONFIG_NSH_DISABLE_POWEROFF``
:ref:`cmdps`           ``CONFIG_FS_PROCFS`` &&                     ``CONFIG_NSH_DISABLE_PS``
                       ! ``CONFIG_FS_PROCFS_EXCLUDE_PROC``
:ref:`cmdput`          ``CONFIG_NET`` && ``CONFIG_NET_UDP`` &&     ``CONFIG_NSH_DISABLE_PUT``
                       ``MTU >= 558`` [#1]_, [#2]_
:ref:`cmdpwd`          !  ``CONFIG_DISABLE_ENVIRON``               ``CONFIG_NSH_DISABLE_PWD``
:ref:`cmdreadlink`     ``CONFIG_PSEUDOFS_SOFTLINKS``               ``CONFIG_NSH_DISABLE_READLINK``
:ref:`cmdreboot`       ``CONFIG_BOARD_RESET``                      ``CONFIG_NSH_DISABLE_REBOOT``
:ref:`cmdrm`           ! ``CONFIG_DISABLE_MOUNTPOINT`` \|\|        ``CONFIG_NSH_DISABLE_RM``
                       ! ``CONFIG_DISABLE_PSEUDOFS_OPERATIONS``
:ref:`cmdrmdir`        ! ``CONFIG_DISABLE_MOUNTPOINT`` \|\
                       ! ``CONFIG_DISABLE_PSEUDOFS_OPERATIONS``    ``CONFIG_NSH_DISABLE_RMDIR``
:ref:`cmdrmmod`        ``CONFIG_MODULE``                           ``CONFIG_NSH_DISABLE_MODCMDS``
:ref:`cmdroute`        ``CONFIG_FS_PROCFS`` &&                     ``CONFIG_NSH_DISABLE_ROUTE``
                       ``CONFIG_FS_PROCFS_EXCLUDE_NET`` && |br|
                       ! ``CONFIG_FS_PROCFS_EXCLUDE_ROUTE`` &&
                       ``CONFIG_NET_ROUTE`` && |br|
                       ! ``CONFIG_NSH_DISABLE_ROUTE`` && |br|
                       (``CONFIG_NET_IPv4`` \|\
                       ``CONFIG_NET_IPv6``)
:ref:`cmdrptun`        ``CONFIG_RPTUN``                            ``CONFIG_NSH_DISABLE_RPTUN``
:ref:`cmdset`          ``CONFIG_NSH_VARS`` \|\|                    ``CONFIG_NSH_DISABLE_SET``
                       ! ``CONFIG_DISABLE_ENVIRON``
:ref:`cmdshutdown`     ``CONFIG_BOARDCTL_POWEROFF`` \|\|           ``CONFIG_NSH_DISABLE_SHUTDOWN``
                       ``CONFIG_BOARD_RESET``
:ref:`cmdsleep`        .                                           ``CONFIG_NSH_DISABLE_SLEEP``
``cmdsource``          ``CONFIG_FILE_STREAM`` &&                   ``CONFIG_NSH_DISABLE_SOURCE``
                       ! ``CONFIG_NSH_DISABLESCRIPT``
:ref:`cmdtelnetd`      ``CONFIG_NSH_TELNET``                       ``CONFIG_NSH_DISABLE_TELNETD``
:ref:`cmdtest`         !  ``CONFIG_NSH_DISABLESCRIPT``             ``CONFIG_NSH_DISABLE_TEST``
:ref:`cmdtime`         .                                           ``CONFIG_NSH_DISABLE_TIME``
:ref:`cmdtruncate`     ! ``CONFIG_DISABLE_MOUNTPOINT``             ``CONFIG_NSH_DISABLE_TRUNCATE``
:ref:`cmdumount`       !  ``CONFIG_DISABLE_MOUNTPOINT``            ``CONFIG_NSH_DISABLE_UMOUNT``
:ref:`cmduname`        .                                           ``CONFIG_NSH_DISABLE_UNAME``
:ref:`cmdunset`        ``CONFIG_NSH_VARS`` \|\|                    ``CONFIG_NSH_DISABLE_UNSET``
                       !  ``CONFIG_DISABLE_ENVIRON``
:ref:`cmdurldecode`    ! ``CONFIG_NETUTILS_CODECS`` &&             ``CONFIG_NSH_DISABLE_URLDECODE``
                       ``CONFIG_CODECS_URLCODE``
:ref:`cmdurlencode`    ! ``CONFIG_NETUTILS_CODECS`` &&             ``CONFIG_NSH_DISABLE_URLENCODE``
                       ``CONFIG_CODECS_URLCODE``
:ref:`cmduseradd`      ! ``CONFIG_DISABLE_MOUNTPOINT`` &&          ``CONFIG_NSH_DISABLE_USERADD``
                       ``CONFIG_NSH_LOGIN_PASSWD``
:ref:`cmduserdel`      ! ``CONFIG_DISABLE_MOUNTPOINT`` &&          ``CONFIG_NSH_DISABLE_USERDEL``
                       ``CONFIG_NSH_LOGIN_PASSWD``
:ref:`cmdusleep`       .                                           ``CONFIG_NSH_DISABLE_USLEEP``
:ref:`cmdwget`         ``CONFIG_NET`` && ``CONFIG_NET_TCP``        ``CONFIG_NSH_DISABLE_WGET``
:ref:`cmdxd`           .                                           ``CONFIG_NSH_DISABLE_XD``
====================== =========================================== ======================

.. [#1] Because of hardware padding, the actual required packet size may be larger
.. [#2] Special TFTP server start-up options will probably be required to permit creation of files for the correct operation of the ``put`` command.
.. [#3] Verbose help output can be suppressed by defining ``CONFIG_NSH_HELP_TERSE``. In that case, the help command is still available but will be slightly smaller.

Built-In Command Dependencies on Configuration Settings
=======================================================

All built-in applications require that support for NSH built-in
applications has been enabled. This support is enabled with
``CONFIG_BUILTIN=y`` and ``CONFIG_NSH_BUILTIN_APPS=y``.

=============  ==================================================================================================
Command        Depends on Configuration
=============  ==================================================================================================
``ping``       ``CONFIG_NET`` && ``CONFIG_NET_ICMP`` && ``CONFIG_NET_ICMP_SOCKET`` && ``CONFIG_SYSTEM_PING``
``ping6``      ``CONFIG_NET`` && ``CONFIG_NET_ICMPv6`` && ``CONFIG_NET_ICMPv6_SOCKET`` && ``CONFIG_SYSTEM_PING6``
=============  ==================================================================================================

.. _nshconfiguration:

NSH-Specific Configuration Settings
===================================

The behavior of NSH can be modified with the following settings in the
``boards/<arch>/<chip>/<board>/defconfig`` file:

===================================  ==================================
Configuration                        Description
===================================  ==================================
 ``CONFIG_NSH_READLINE``             Selects the minimal implementation of ``readline()``.
                                     This minimal implementation provides on backspace for command
                                     line editing. It expects some minimal VT100 command support from the terminal.

 ``CONFIG_NSH_CLE``                  Selects the more extensive, EMACS-like command line editor.
                                     Select this option only if (1) you don't mind a modest increase
                                     in the FLASH footprint, and (2) you work with a terminal that
                                     supports extensive VT100 editing commands. Selecting this option
                                     will add probably 1.5-2KB to the FLASH footprint.

 ``CONFIG_NSH_BUILTIN_APPS``         Support external registered, "builtin" applications that can
                                     be executed from the NSH command line (see apps/README.txt for
                                     more information). This required ``CONFIG_BUILTIN`` to enable
                                     NuttX support for "builtin" applications.

 ``CONFIG_NSH_FILEIOSIZE``           Size of a static I/O buffer used for file access (ignored if there
                                     is no file system). Default is 1024.

 ``CONFIG_NSH_STRERROR``             ``strerror(errno)`` makes more readable output but
                                     ``strerror()`` is very large and will not be used unless this
                                     setting is *y*. This setting depends upon the ``strerror()``
                                     having been enabled with ``CONFIG_LIBC_STRERROR``.

 ``CONFIG_NSH_LINELEN``              The maximum length of one command line and of one output line.
                                     Default: 80

 ``CONFIG_NSH_DISABLE_SEMICOLON``    By default, you can enter multiple NSH commands on a line
                                     with each command separated by a semicolon. You can disable this
                                     feature to save a little memory on FLASH challenged platforms.
                                     Default: n

 ``CONFIG_NSH_CMDPARMS``             If selected, then the output from commands, from file applications,
                                     and from NSH built-in commands can be used as arguments to other
                                     commands. The entity to be executed is identified by
                                     enclosing the command line in back quotes. For example::

                                       set FOO `myprogram $BAR`

                                     will execute the program named ``myprogram`` passing it the
                                     value of the environment variable ``BAR``. The value of the
                                     environment variable ``FOO`` is then set output of ``myprogram``
                                     on ``stdout``. Because this feature commits significant
                                     resources, it is disabled by default. The ``CONFIG_NSH_CMDPARMS`` interim
                                     output will be retained in a temporary file. Full path to a
                                     directory where temporary files can be created is taken from
                                     ``CONFIG_LIBC_TMPDIR`` and it defaults to ``/tmp`` if
                                     ``CONFIG_LIBC_TMPDIR`` is not set.

 ``CONFIG_NSH_MAXARGUMENTS``         The maximum number of NSH command arguments. Default: 6

 ``CONFIG_NSH_ARGCAT``               Support concatenation of strings with environment variables or
                                     command output. For example::

                                       set FOO XYZ
                                       set BAR 123
                                       set FOOBAR ABC_${FOO}_${BAR}

                                     would set the environment variable ``FOO`` to ``XYZ``,
                                     ``BAR`` to ``123`` and ``FOOBAR`` to ``ABC_XYZ_123``. If
                                     ``CONFIG_NSH_ARGCAT`` is not selected, then a slightly small
                                     FLASH footprint results but then also only simple environment
                                     variables like ``$FOO`` can be used on the command line.

 ``CONFIG_NSH_VARS``                 By default, there are no internal NSH variables. NSH will use OS
                                     environment variables for all variable storage. If this option,
                                     NSH will also support local NSH variables. These variables are,
                                     for the most part, transparent and work just like the OS
                                     environment variables. The difference is that when you
                                     create new tasks, all of environment variables are
                                     inherited by the created tasks. NSH local variables are not.
                                     If this option is enabled (and ``CONFIG_DISABLE_ENVIRON`` is not),
                                     then a new command called 'export' is enabled. The export
                                     command works very must like the set command except that is
                                     operates on environment variables. When CONFIG_NSH_VARS
                                     is enabled, there are changes in the behavior of certain commands.
                                     See following :ref:`cmdtable <nsh_vars_table>`.

 ``CONFIG_NSH_QUOTE``                Enables back-slash quoting of certain characters within the
                                     command. This option is useful for the case where an NSH script
                                     is used to dynamically generate a new NSH script. In that case,
                                     commands must be treated as simple text strings without
                                     interpretation of any special characters. Special characters
                                     such as ``$``, :literal:`\``, ``"``, and others must be
                                     retained intact as part of the test string. This option is
                                     currently only available is ``CONFIG_NSH_ARGCAT`` is also
                                     selected.

 ``CONFIG_NSH_NESTDEPTH``            The maximum number of nested ``if-then[-else]-fi`` <#conditional>`__
                                     sequences that are permissible. Default: 3

 ``CONFIG_NSH_DISABLESCRIPT``        This can be set to *y* to suppress support for scripting.
                                     This setting disables the ```sh`` <#cmdsh>`__,
                                     ```test`` <#cmdtest>`__, and ```[`` <#cmtest>`__ commands and
                                     the ```if-then[-else]-fi`` <#conditional>`__
                                     construct. This would only be set on systems where a minimal
                                     footprint is a necessity and scripting is not.

 ``CONFIG_NSH_DISABLE_ITEF``         If scripting is enabled, then then this option can be selected
                                     to suppress support for ``if-then-else-fi`` sequences in
                                     scripts. This would only be set on systems where some minimal
                                     scripting is required but ``if-then-else-fi`` is not.

 ``CONFIG_NSH_DISABLE_LOOPS``        If scripting is enabled, then then this option can be selected
                                     suppress support ``for while-do-done`` and
                                     ``until-do-done`` sequences in scripts. This would only be set
                                     on systems where some minimal scripting is required but looping
                                     is not.

 ``CONFIG_NSH_DISABLEBG``            This can be set to *y* to suppress support for background
                                     commands. This setting disables the ```nice`` <#cmdoverview>`__
                                     command prefix and the ```&`` <#cmdoverview>`__ command
                                     suffix. This would only be set on systems where a minimal footprint
                                     is a necessity and background command execution is not.

 ``CONFIG_NSH_MMCSDMINOR``           If the architecture supports an MMC/SD slot and if the NSH
                                     architecture specific logic is present, this option will provide
                                     the MMC/SD minor number, i.e., the MMC/SD block driver will be
                                     registered as ``/dev/mmcsd``\ *N* where *N* is the minor number.
                                     Default is zero.

 ``CONFIG_NSH_ROMFSETC``             Mount a ROMFS file system at ``/etc`` and provide a system init
                                     script at ``/etc/init.d.rc.sysinit`` and a startup script at
                                     ``/etc/init.d/rcS``.
                                     The default system init script will mount a FAT FS RAMDISK at
                                     ``/tmp`` but the logic is `easily extensible <#startupscript>`__.

 ``CONFIG_NSH_CONSOLE``              If ``CONFIG_NSH_CONSOLE`` is set to *y*, then a serial console
                                     front-end is selected.

                                     Normally, the serial console device is a UART and RS-232
                                     interface. However, if ``CONFIG_USBDEV`` is defined,
                                     then a USB serial device may, instead, be used if the one of
                                     the following are defined:

                                     -  ``CONFIG_PL2303`` and ``CONFIG_PL2303_CONSOLE``.
                                        Sets up the Prolifics PL2303 emulation as a console device
                                        at ``/dev/console``.
                                     -  ``CONFIG_CDCACM`` and ``CONFIG_CDCACM_CONSOLE``.
                                        Sets up the CDC/ACM serial device as a console device at
                                        ``/dev/console``.
                                     -  ``CONFIG_NSH_USBCONSOLE``. If defined, then an arbitrary USB
                                        device may be used to as the NSH console. In this case,
                                        ``CONFIG_NSH_USBCONDEV`` must be defined to indicate which
                                        USB device to use as the console. The advantage of
                                        using a device other that ``/dev/console`` is that
                                        normal debug output can then use ``/dev/console`` while NSH
                                        uses ``CONFIG_NSH_USBCONDEV``.

                                        ``CONFIG_NSH_USBCONDEV``. If ``CONFIG_NSH_USBCONSOLE`` is
                                        set to 'y', then ``CONFIG_NSH_USBCONDEV`` must also be set to select the USB
                                        device used to support the NSH console. This should be set to
                                        the quoted name of a readable/write-able USB driver
                                        such as: ``CONFIG_NSH_USBCONDEV="/dev/ttyACM0"``.

                                     If there are more than one USB slots, then a USB device minor
                                     number may also need to be provided:

                                     -  ``CONFIG_NSH_UBSDEV_MINOR``: The minor device number of the USB device. Default: 0

                                     If USB tracing is enabled (``CONFIG_USBDEV_TRACE``), then
                                     NSH will initialize USB tracing as requested by the following.
                                     Default: Only USB errors are traced.

                                     - ``CONFIG_NSH_USBDEV_TRACEINIT``: Show initialization events
                                     -  ``CONFIG_NSH_USBDEV_TRACECLASS``: Show class driver events
                                     -  ``CONFIG_NSH_USBDEV_TRACETRANSFERS``: Show data transfer events
                                     -  ``CONFIG_NSH_USBDEV_TRACECONTROLLER``: Show controller events
                                     -  ``CONFIG_NSH_USBDEV_TRACEINTERRUPTS``: Show interrupt-related events.

 ``CONFIG_NSH_ALTCONDEV`` and        If ``CONFIG_NSH_CONSOLE`` is set ``CONFIG_NSH_CONDEV``
                                     to *y*, then ``CONFIG_NSH_ALTCONDEV`` may also
                                     be selected to enable use of an alternate character device to
                                     support the NSH console. If ``CONFIG_NSH_ALTCONDEV`` is
                                     selected, then ``CONFIG_NSH_CONDEV`` holds the
                                     quoted name of a readable/write-able character
                                     driver such as: ``CONFIG_NSH_CONDEV="/dev/ttyS1"``.
                                     This is useful, for example, to separate the NSH command line
                                     from the system console when the system console is used to provide
                                     debug output. Default: ``stdin`` and ``stdout`` (probably
                                     "``/dev/console``")

                                     -  **NOTE 1:** When any other device other than
                                        ``/dev/console`` is used for a user interface, (1) linefeeds
                                        (``\n``) will not be expanded to carriage return / linefeeds
                                        (``\r\n``). You will need to configure your terminal
                                        program to account for this.
                                        And (2) input is not automatically echoed so you
                                        will have to turn local echo on.
                                     -  **NOTE 2:** This option forces the console of all sessions to
                                        use NSH_CONDEV. Hence, this option only makes sense for a
                                        system that supports only a single session. This option
                                        is, in particular, incompatible with Telnet
                                        sessions because each Telnet session must use a different
                                        console device.

 ``CONFIG_NSH_TELNET``               If ``CONFIG_NSH_TELNET`` is set to *y*, then a TELNET server
                                     front-end is selected. When this option is provided, you may log
                                     into NuttX remotely using telnet in order to access NSH.

 ``CONFIG_NSH_ARCHINIT``             Set ``CONFIG_NSH_ARCHINIT`` if your board provides architecture
                                     specific initialization via the board-specific function
                                     ``board_app_initialize()``. This function will be called early in
                                     NSH initialization to allow board logic to do such things as
                                     configure MMC/SD slots.
===================================  ==================================

.. _nsh_vars_table:

==================  ===================================   =============================================
CMD                 w/o ``CONFIG_NSH_VARS``               w/``CONFIG_NSH_VARS``
==================  ===================================   =============================================
``set <a> <b>``     Set environment variable <a> to <b>   Set NSH variable <a> to <b> (Unless the NSH variable has been *promoted* via
                                                          ``export``, in which case the env ironment variable of the same name is set to <b>).
``set``             Causes an error.                      Lists all NSH variables.
``unset <a>``       Unsets environment variable <a>       Unsets both environment variable *and* NSH variable with and name <a>
``export <a> <b>``  Causes an error,                      Unsets NSH variable <a>. Sets environment variable <a> to <b>.
``export <a>``      Causes an error.                      Sets environment variable <a> to the value of NSH variable <a> (or "" if the
                                                          NSH variable has not been set). Unsets NSH local variable <a>.
``env``             Lists all environment variables       Lists all environment variables (*only*)
==================  ===================================   =============================================

If Telnet is selected for the NSH console, then we must configure the
resources used by the Telnet daemon and by the Telnet clients.

======================================  ================================
Configuration                           Description
======================================  ================================
``CONFIG_NSH_TELNETD_PORT``             The telnet daemon will listen on this TCP port number for connections. Default: 23
``CONFIG_NSH_TELNETD_DAEMONPRIO``       Priority of the Telnet daemon. Default: ``SCHED_PRIORITY_DEFAULT``
``CONFIG_NSH_TELNETD_DAEMONSTACKSIZE``  Stack size allocated for the Telnet daemon. Default: 2048
``CONFIG_NSH_TELNETD_CLIENTPRIO``       Priority of the Telnet client. Default: ``SCHED_PRIORITY_DEFAULT``
``CONFIG_NSH_TELNETD_CLIENTSTACKSIZE``  Stack size allocated for the Telnet client. Default: 2048
======================================  ================================

One or both of ``CONFIG_NSH_CONSOLE`` and ``CONFIG_NSH_TELNET`` must be
defined. If ``CONFIG_NSH_TELNET`` is selected, then there some other
configuration settings that apply:

======================================  ================================
Configuration                           Description
======================================  ================================
``CONFIG_NET=y``                        Of course, networking must be enabled.
``CONFIG_NET_TCP=y``                    TCP/IP support is required for telnet (as well as various other
                                        TCP-related configuration settings).
``CONFIG_NSH_IOBUFFER_SIZE``            Determines the size of the I/O buffer to use for sending/ receiving
                                        TELNET commands/responses
``CONFIG_NSH_DHCPC``                    Obtain the IP address via DHCP.
``CONFIG_NSH_IPADDR``                   If ``CONFIG_NSH_DHCPC`` is NOT set, then the static IP address must be
                                        provided.
``CONFIG_NSH_DRIPADDR``                 Default router IP address
``CONFIG_NSH_NETMASK``                  Network mask
``CONFIG_NSH_NOMAC``                    Set if your Ethernet hardware has no built-in MAC address. If set, a
                                        bogus MAC will be assigned.
``CONFIG_NSH_MAX_ROUNDTRIP``            This is the maximum round trip for a response to a ICMP ECHO request. It
                                        is in units of deciseconds. The default is 20 (2 seconds).
======================================  ================================

If you use DHCPC, then some special configuration network options are
required. These include:

============================================== ============================================================
Configuration                                  Description
============================================== ============================================================
``CONFIG_NET=y``                               Of course, networking must be enabled.
``CONFIG_NET_UDP=y``                           UDP support is required for DHCP (as well as various other
                                               UDP-related configuration settings).
``CONFIG_NET_BROADCAST=y``                     UDP broadcast support is needed.
``CONFIG_NET_ETH_PKTSIZE=650`` (or larger)     Per RFC2131 (p. 9), the DHCP client must be prepared to receive
                                               DHCP messages of up to 576 bytes (excluding Ethernet, IP, or
                                               UDP headers and FCS). NOTE: Note that the actual MTU setting
                                               will depend upon the specific link protocol. Here Ethernet
                                               is indicated.
============================================== ============================================================

If ``CONFIG_NSH_ROMFSETC`` is selected, then the following additional
configuration setting apply:

============================== ==============================================================
Configuration                  Description
============================== ==============================================================
``CONFIG_NSH_ARCHROMFS``       May be defined to specify an alternative ROMFS image
                               that can be found at ``boards/<arch>/<chip>/<board>/include/nsh_romfsimg.h``.
``CONFIG_NSH_ROMFSMOUNTPT``    The default mountpoint for the ROMFS volume is ``"/etc"``,
                               but that can be changed with this setting. This must be a
                               absolute path beginning with '``/``' and enclosed in quotes.
``CONFIG_NSH_SYSINITSCRIPT``   This is the relative path to the system init script within the
                               mountpoint. The default is ``"init.d/rc.sysinit"``. This is a relative
                               path and must not start with '``/``' but must be enclosed in quotes.
``CONFIG_NSH_INITSCRIPT``      This is the relative path to the startup script within the
                               mountpoint. The default is ``"init.d/rcS"``. This is a relative
                               path and must not start with '``/``' but must be enclosed in quotes.
``CONFIG_NSH_ROMFSDEVNO``      This is the minor number of the ROMFS block device.
                               The default is '``0``' corresponding to ``/dev/ram0``.
``CONFIG_NSH_ROMFSSECTSIZE``   This is the sector size to use with the ROMFS volume. Since the
                               default volume is very small, this defaults to 64 but should
                               be increased if the ROMFS volume were to be become large.
                               Any value selected must be a power of 2.
============================== ==============================================================

When the default ``rc.sysinit`` file used when ``CONFIG_NSH_ROMFSETC`` is
selected, it will mount a FAT FS under ``/tmp``. The following
selections describe that FAT FS.

============================== =======================================================
Configuration                  Description
============================== =======================================================
``CONFIG_NSH_FATDEVNO``        This is the minor number of the FAT FS block device.
                               The default is '``1``' corresponding to ``/dev/ram1``.
``CONFIG_NSH_FATSECTSIZE``     This is the sector size use with the FAT FS. Default is 512.
============================== =======================================================

