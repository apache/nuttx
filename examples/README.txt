examples
^^^^^^^^

  The examples directory contains several sample applications that
  can be linked with nuttx.  The specific example is selected in the
  configs/<board-name>/defconfig file via the CONFIG_EXAMPLE setting.
  For example,

    CONFIG_EXAMPLE=ostest

  Selects the examples/ostest example.

examples/ostest
^^^^^^^^^^^^^^^

  This is the NuttX 'qualification' suite.  It attempts to exercise
  a broad set of OS functionality.  Its coverage is not very extensive
  as of this writing, but it is used to qualify each NuttX release.

  The behavior of the ostest can be modified with the following
  settings in the configs/<board-name>/defconfig file:

  * CONFIG_EXAMPLES_OSTEST_LOOPS
      Used to control the number of executions of the test.  If
      undefined, the test executes one time.  If defined to be
      zero, the test runs forever.
  * CONFIG_EXAMPLES_OSTEST_STACKSIZE
      Used to create the ostest task.  Default is 8192.

examples/nsh
^^^^^^^^^^^^

  This directory contains the NuttShell (NSH).  This is a simple
  shell-like application.  With some additional development, NSH will
  someday be a great NuttX application debugger.  At present, NSH
  supports the following commands:

  Command    Depends on Configuration
  ---------- --------------------------
  [          !CONFIG_EXAMPLES_NSH_DISABLESCRIPT
  cat        CONFIG_NFILE_DESCRIPTORS > 0
  cd         !CONFIG_DISABLE_ENVIRON && CONFIG_NFILE_DESCRIPTORS > 0
  cp         CONFIG_NFILE_DESCRIPTORS > 0
  echo       --
  exec       --
  exit       --
  help       --
  ifconfig   CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS > 0
  ls         CONFIG_NFILE_DESCRIPTORS > 0
  mb,mh,mw   ---
  mem        ---
  mkdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  mkfatfs    !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  mkfifo     !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  mount      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  ps         --
  pwd        !CONFIG_DISABLE_ENVIRON && CONFIG_NFILE_DESCRIPTORS > 0
  rm         !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  rmdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  set        !CONFIG_DISABLE_ENVIRON
  sh         CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0 && !CONFIG_EXAMPLES_NSH_DISABLESCRIPT
  sleep      !CONFIG_DISABLE_SIGNALS
  test       !CONFIG_EXAMPLES_NSH_DISABLESCRIPT
  umount     !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  unset      !CONFIG_DISABLE_ENVIRON
  usleep     !CONFIG_DISABLE_SIGNALS

  Test syntax:

    expression = simple-expression | !expression |
                 expression -o expression | expression -a expression

    simple-expression = unary-expression | binary-expression

    unary-expression = string-unary | file-unary

    string-unary = -n string | -z string

    file-unary = -b file | -c file | -d file | -e file | -f file |
                 -r file | -s file | -w file

    binary-expression = string-binary | numeric-binary

    string-binary = string = string | string == string | string != string

    numeric-binary = integer -eq integer | integer -ge integer |
                     integer -gt integer | integer -le integer |
                     integer -lt integer | integer -ne integer

  Other behavior of NSH can be modified with the following settings in
  the configs/<board-name>/defconfig file:

  * CONFIG_EXAMPLES_NSH_FILEIOSIZE
      Size of a static I/O buffer used for file access (ignored if
      there is no filesystem).

  * CONFIG_EXAMPLES_NSH_STRERROR
      strerror(errno) makes more readable output but strerror() is
      very large and will not be used unless this setting is 'y'

  * CONFIG_EXAMPLES_NSH_LINELEN
      The maximum length of one command line and of one output line.
      Default: 80

  * CONFIG_EXAMPLES_NSH_STACKSIZE
      The stack size to use when spawning new threads or tasks.  Such
      new threads are generated when a command is executed in background
      or as new TELNET connections are established.

  * CONFIG_EXAMPLES_NSH_NESTDEPTH
      The maximum number of nested if-then[-else]-fi sequences that
      are permissable.  Default: 3

  * CONFIG_EXAMPLES_NSH_DISABLESCRIPT
      This can be set to 'y' to suppress support for scripting.  This
      setting disables the 'sh', 'test', and '[' commands and the
      if-then[-else]-fi construct.  This would only be set on systems
      where a minimal footprint is a necessity and scripting is not.

  * CONFIG_EXAMPLES_NSH_DISABLEBG
      This can be set to 'y' to suppress support for background
      commands.  This setting disables the 'nice' command prefix and
      the '&' command suffix.  This would only be set on systems
      where a minimal footprint is a necessity and background command
      execution is not.

  * CONFIG_EXAMPLES_NSH_CONSOLE
      If CONFIG_EXAMPLES_NSH_CONSOLE is set to 'y', then a serial
      console front-end is selected.

  * CONFIG_EXAMPLES_NSH_TELNET
      If CONFIG_EXAMPLES_NSH_TELNET is set to 'y', then a TELENET
      server front-end is selected.  When this option is provided,
      you may log into NuttX remotely using telnet in order to
      access NSH.

  One or both of CONFIG_EXAMPLES_NSH_CONSOLE and CONFIG_EXAMPLES_NSH_TELNET
  must be defined.  If CONFIG_EXAMPLES_NSH_TELNET is selected, then there some
  other configuration settings that apply:

  * CONFIG_EXAMPLES_NSH_IOBUFFER_SIZE
      Determines the size of the I/O buffer to use for sending/
      receiving TELNET commands/reponses

  * CONFIG_EXAMPLES_NSH_DHCPC
      Obtain the the IP address via DHCP.

  * CONFIG_EXAMPLES_NSH_IPADDR
      If CONFIG_EXAMPLES_NSH_DHCPC is NOT set, then the static IP
      address must be provided.

  * CONFIG_EXAMPLES_NSH_DRIPADDR
      Default router IP address

  * CONFIG_EXAMPLES_NSH_NETMASK
      Network mask

  * CONFIG_EXAMPLES_NSH_NOMAC
      Set if your ethernet hardware has no built-in MAC address.
      If set, a bogus MAC will be assigned.

examples/mount
^^^^^^^^^^^^^^

  This contains a simple test of filesystem mountpoints.
  
  * CONFIG_EXAMPLES_MOUNT_DEVNAME
      The name of the user-provided block device to mount.
      If CONFIG_EXAMPLES_MOUNT_DEVNAME is not provided, then
      a RAM disk will be configured.

  * CONFIG_EXAMPLES_MOUNT_NSECTORS
      The number of "sectors" in the RAM disk used when
      CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.

  * CONFIG_EXAMPLES_MOUNT_SECTORSIZE
      The size of each sectors in the RAM disk used when
      CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.

  * CONFIG_EXAMPLES_MOUNT_RAMDEVNO
      The RAM device minor number used to mount the RAM disk used
      when CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.  The
      default is zero (meaning that "/dev/ram0" will be used).

examples/null
^^^^^^^^^^^^^

  This is the do nothing application.  It is only used for bringing
  up new NuttX architectures.

examples/uip
^^^^^^^^^^^^

  This is a port of uIP example application.  It includes
  conditionally compiled logic to exercise the uIP webserver,
  webclient, telnet, smtp, dncpc, and resolver.

examples/netttest
^^^^^^^^^^^^^^^^^

  This is a simple network test for verifying client- and server-
  functionality in a TCP/IP connection.

examples/udp
^^^^^^^^^^^^

  This is a simple network test for verifying client- and server-
  functionality over UDP.

examples/pashello
^^^^^^^^^^^^^^^^^

  This is "Hello, World" implemented via the Pascal P-Code interpreter

examples/pipe
^^^^^^^^^^^^^

  A test of the mkfifo() and pipe() APIs.

 * CONFIG_EXAMPLES_PIPE_STACKSIZE
     Sets the size of the stack to use when creating the child tasks.
     The default size is 1024.
