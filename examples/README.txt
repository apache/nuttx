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

  * CONFIG_OSTEST_STACKSIZE
      Used to create the ostest task.  Default is 8192.

examples/nsh
^^^^^^^^^^^^

  This directory contains the NuttShell (NSH).  This is a simple
  shell-like application.  With some additional development, NSH will
  someday be a great NuttX application debugger.  At present, NSH
  supports the following commands:

  Command    Depends on Configuration
  ---------- --------------------------
  cat        CONFIG_NFILE_DESCRIPTORS > 0
  cp         CONFIG_NFILE_DESCRIPTORS > 0
  echo       --
  exec       --
  exit       --
  help       --
  ifconfig   CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS > 0
  ls         CONFIG_NFILE_DESCRIPTORS > 0
  mkdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  mount      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  ps         --
  set        !CONFIG_DISABLE_ENVIRON
  rm         !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  rmdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  umount     !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  unset      !CONFIG_DISABLE_ENVIRON

  Other behavior of NSH can be modified with the following settings in
  the configs/<board-name>/defconfig file:

  * CONFIG_NSH_IOBUFFERSIZE
      Size of a static I/O buffer used for file access (ignored if
      there is no filesystem).

  * CONFIG_NSH_STRERROR
      strerror(errno) makes more readable output but strerror() is
      very large and will not be used unless this setting is 'y'

  * CONFIG_EXAMPLES_NSH_TELNET
      By default, NSH is configured to use the serial console.
      If CONFIG_EXAMPLES_NSH_TELNET is set to 'y', then a TELENET
      server front-end is selected.  When this option is provided,
      you must log into NuttX remotely using telnet in order to
      access NSH.

  If CONFIG_EXAMPLES_NSH_TELNET is selected, then there some other
  configuration settings that apply:

  * CONFIG_EXAMPLES_NSH_IOBUFFER_SIZE
      Determines the size of the I/O buffer to use for sending/
      receiving TELNET commands/reponses

  * CONFIG_EXAMPLES_NSH_CMD_SIZE
      The size of one parsed NSH command

  * CONFIG_EXAMPLES_NSH_STACKSIZE
      The stack size to use when spawning new threads as new TELNET
      connections are established.

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


