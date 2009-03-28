examples
^^^^^^^^

  The examples directory contains several sample applications that
  can be linked with nuttx.  The specific example is selected in the
  configs/<board-name>/defconfig file via the CONFIG_EXAMPLE setting.
  For example,

    CONFIG_EXAMPLE=ostest

  Selects the examples/ostest example.

examples/dhcpd
^^^^^^^^^^^^^^

  This examples builds a tiny DCHP server for the target system.
  
  NOTE: For test purposes, this example can be built as a
  host-based DHCPD server.  This can be built as follows:

    cd examples/dhcpd
    make -f Makefile.host

  Kernel configuratin settings:

    CONFIG_EXAMPLE_DHCPD_NOMAC     - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLE_DHCPD_IPADDR    - Target IP address
    CONFIG_EXAMPLE_DHCPD_DRIPADDR  - Default router IP addess
    CONFIG_EXAMPLE_DHCPD_NETMASK   - Network mask

  See also CONFIG_NETUTILS_DHCPD_* settings described elsewhere
  and used in netutils/dhcpd/dhcpd.c. These settings are required
  to described the behavior of the daemon.

examples/hello
^^^^^^^^^^^^^^

  This is the mandatory, "Hello, World!!" example.  It is little more
  than examples/null with a single printf statement.  Again useful only
  for bringing up new NuttX architectures.

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

examples/netttest
^^^^^^^^^^^^^^^^^

  This is a simple network test for verifying client- and server-
  functionality in a TCP/IP connection.

examples/nsh
^^^^^^^^^^^^

  This directory contains the NuttShell (NSH).  This is a simple
  shell application.  With some additional development, NSH will
  someday be a great NuttX application debugger.  NSH is described
  in its own README located at examples/nsh/README.txt

examples/nx
^^^^^^^^^^^

  This directory contains a simple test of a subset of the NX APIs
  defined in include/nuttx/nx.h.  The following configuration options
  can be selected:

    CONFIG_EXAMPLES_NX_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NX_BGCOLOR -- The color of the background.  Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_COLOR1 -- The color of window 1. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_COLOR2 -- The color of window 2. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_TBCOLOR -- The color of the toolbar. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_FONTCOLOR -- The color of the toolbar. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_BPP -- Pixels per pixel to use.  Valid options
      include 2, 4, 8, 16, 24, and 32.  Default is 32.
    CONFIG_EXAMPLES_NX_RAWWINDOWS -- Use raw windows;  Default is to
      use pretty, framed NXTK windows with toolbars.

  This test can be performed with either the single-user version of
  NX or with the multiple user version of NX selected with CONFIG_NX_MULTIUSER.
  If CONFIG_NX_MULTIUSER is defined, then the following configuration
  options also apply:

    CONFIG_EXAMPLES_NX_STACKSIZE -- The stacksize to use when creating
      the NX server.  Default 2048
    CONFIG_EXAMPLES_NX_CLIENTPRIO -- The client priority.  Default: 100
    CONFIG_EXAMPLES_NX_SERVERPRIO -- The server priority.  Default: 120
    CONFIG_EXAMPLES_NX_LISTENERPRIO -- The priority of the event listener
      thread. Default 80.
    CONFIG_EXAMPLES_NX_NOTIFYSIGNO -- The signal number to use with
      nx_eventnotify().  Default: 4

  If CONFIG_NX_MULTIUSER is defined, then the example also expects the
  following settings and will generate an error if they are not as expected:

    CONFIG_DISABLE_MQUEUE=n
    CONFIG_DISABLE_SIGNALS=n
    CONFIG_DISABLE_PTHREAD=n
    CONFIG_NX_BLOCKING=y

examples/null
^^^^^^^^^^^^^

  This is the do nothing application.  It is only used for bringing
  up new NuttX architectures in the most minimal of environments.

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
  * CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS
      Specifies the number of threads to create in the barrier
      test.  The default is 8 but a smaller number may be needed on
      systems without sufficient memory to start so many threads.

examples/pashello
^^^^^^^^^^^^^^^^^

  This is "Hello, World" implemented via the Pascal P-Code interpreter

examples/pipe
^^^^^^^^^^^^^

  A test of the mkfifo() and pipe() APIs.

 * CONFIG_EXAMPLES_PIPE_STACKSIZE
     Sets the size of the stack to use when creating the child tasks.
     The default size is 1024.

examples/poll
^^^^^^^^^^^^^

  A test of the poll() and select() APIs using FIFOs and, if available,
  stdin, and a TCP/IP socket.  In order to build this test, you must the
  following selected in your NuttX configuration file:

  CONFIG_NFILE_DESCRIPTORS          - Defined to be greater than 0
  CONFIG_DISABLE_POLL               - NOT defined

  In order to use the TCP/IP select test, you have also the following
  additional things selected in your NuttX configuration file:

  CONFIG_NET                        - Defined for general network support
  CONFIG_NET_TCP                    - Defined for TCP/IP support
  CONFIG_NSOCKET_DESCRIPTORS        - Defined to be greater than 0
  CONFIG_NET_NTCP_READAHEAD_BUFFERS - Defined to be greater than zero

  CONFIG_EXAMPLE_POLL_NOMAC         - (May be defined to use software assigned MAC)
  CONFIG_EXAMPLE_POLL_IPADDR        - Target IP address
  CONFIG_EXAMPLE_POLL_DRIPADDR      - Default router IP addess
  CONFIG_EXAMPLE_POLL_NETMASK       - Network mask

  In order to for select to work with incoming connections, you
  must also select:

  CONFIG_NET_TCPBACKLOG             - Incoming connections pend in a backlog until accept() is called.

  In additional to the target device-side example, there is also
  a host-side application in this directory.  It can be compiled under
  Linux or Cygwin as follows:

    cd examples/usbserial
    make -f Makefile.host TOPDIR=../../. TARGETIP=<target-ip>

  Where <target-ip> is the IP address of your target board.

  This will generate a small program called 'host'.  Usage:

  1. Build the examples/poll target program with TCP/IP poll support
     and start the target.

  3. Then start the host application:

       ./host

  The host and target will exchange are variety of small messages. Each
  message sent from the host should cause the select to return in target.
  The target example should read the small message and send it back to
  the host.  The host should then receive the echo'ed message.

examples/romfs
^^^^^^^^^^^^^^

  This example exercises the romfs filesystem.  Configuration options
  include:

  * CONFIG_EXAMPLES_ROMFS_RAMDEVNO
      The minor device number to use for the ROM disk.  The default is
      1 (meaning /dev/ram1)

  * CONFIG_EXAMPLES_ROMFS_SECTORSIZE
      The ROM disk sector size to use.  Default is 64.

  * CONFIG_EXAMPLES_ROMFS_MOUNTPOINT
      The location to mount the ROM disk.  Deafault: "/usr/local/share"

examples/serloop
^^^^^^^^^^^^^^^^

  This is a mindlessly simple loopback test on the console.  Useful
  for testing new serial drivers.  Configuration options include:

  * CONFIG_EXAMPLES_SERLOOP_BUFIO
      Use C buffered I/O (getchar/putchar) vs. raw console I/O
      (read/read).

examples/udp
^^^^^^^^^^^^

  This is a simple network test for verifying client- and server-
  functionality over UDP.

examples/uip
^^^^^^^^^^^^

  This is a port of uIP example application.  It includes conditionally
  compiled logic to exercise the uIP webserver, telnet, smtp, dhcpc,
  and resolver.  

  Other configuratin items apply also to the selected network utility.
  For example, the additional relevant settings for the uIP webserver
  are:

    CONFIG_NETUTILS_HTTPDSTACKSIZE
    CONFIG_NETUTILS_HTTPDFILESTATS
    CONFIG_NETUTILS_HTTPDNETSTATS

examples/usbserial
^^^^^^^^^^^^^^^^^^

  This is another implementation of "Hello, World" but this one uses
  a USB serial driver.  Configuration options can be used to simply
  the test. These options include:

  CONFIG_EXAMPLES_USBSERIAL_INONLY
     Only verify IN (device-to-host) data transfers.  Default: both
  CONFIG_EXAMPLES_USBSERIAL_OUTONLY
     Only verify OUT (host-to-device) data transfers.  Default: both
  CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL
     Send only small, single packet messages.  Default: Send large and small.
  CONFIG_EXAMPLES_USBSERIAL_ONLYBIG
     Send only large, multi-packet messages.  Default: Send large and small.

  In additional to the target device-side example, there is also
  a host-side application in this directory.  It can be compiled under
  Linux or Cygwin as follows:

    cd examples/usbserial
    make -f Makefile.host TOPDIR=../../.

  This will generate a small program called 'host'.  Usage:

  1. Build the examples/usbserial target program and start the target.

  2. Wait a bit, then do enter:

     dmesg

     At the end of the dmesg output, you should see the the seria
     device was successfully idenfied and assigned to a tty device,
     probably /dev/ttyUSB0.

  3. Then start the host application:

       ./host [<tty-dev>]

     Where:

       <tty-dev> is the USB TTY device to use.  The default is /dev/ttyUSB0.

  The host and target will exchange are variety of very small and very large
  serial messages.

examples/usbstorage
^^^^^^^^^^^^^^^^^^^

  This example registers a block device driver, then exports the block
  the device using the USB storage class driver.  Configuration options:

  CONFIG_EXAMPLES_USBSTRG_NLUNS
    Defines the number of logical units (LUNs) exported by the USB storage
    driver.  Each LUN corresponds to one exported block driver (or partition
    of a block driver).  May be 1, 2, or 3.  Default is 1.
  CONFIG_EXAMPLES_USBSTRG_DEVMINOR1
    The minor device number of the block driver for the first LUN. For
    example, N in /dev/mmcsdN.  Used for registering the block driver. Default
    is zero.
  CONFIG_EXAMPLES_USBSTRG_DEVPATH1
    The full path to the registered block driver.  Default is "/dev/mmcsd0"
  CONFIG_EXAMPLES_USBSTRG_DEVMINOR2 and CONFIG_EXAMPLES_USBSTRG_DEVPATH2
    Similar parameters that would have to be provided if CONFIG_EXAMPLES_USBSTRG_NLUNS
    is 2 or 3.  No defaults.
  CONFIG_EXAMPLES_USBSTRG_DEVMINOR3 and CONFIG_EXAMPLES_USBSTRG_DEVPATH3
    Similar parameters that would have to be provided if CONFIG_EXAMPLES_USBSTRG_NLUNS
    is 3.  No defaults.

  If CONFIG_USBDEV_TRACE is enabled (or CONFIG_DEBUG and CONFIG_DEBUG_USB), then
  the example code will also manage the USB trace output.  The amount of trace output
  can be controlled using:

  CONFIG_EXAMPLES_USBSTRG_TRACEINIT
    Show initialization events
  CONFIG_EXAMPLES_USBSTRG_TRACECLASS
    Show class driver events
  CONFIG_EXAMPLES_USBSTRG_TRACETRANSFERS
    Show data transfer events
  CONFIG_EXAMPLES_USBSTRG_TRACECONTROLLER
    Show controller events
  CONFIG_EXAMPLES_USBSTRG_TRACEINTERRUPTS
    Show interrupt-related events.

  Error results are always shown in the trace output

examples/wget
^^^^^^^^^^^^^

  A simple web client example.  It will obtain a file from a server using the HTTP
  protocol.  Settings unique to this example include:

    CONFIG_EXAMPLE_WGET_URL       - The URL of the file to get
    CONFIG_EXAMPLE_WGET_NOMAC     - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLE_WGET_IPADDR    - Target IP address
    CONFIG_EXAMPLE_WGET_DRIPADDR  - Default router IP addess
    CONFIG_EXAMPLE_WGET_NETMASK   - Network mask

  This example uses netutils/webclient.  Additional configuration settings apply
  to that code as follows (but built-in defaults are probably OK):

    CONFIG_WEBCLIENT_GETMIMETYPE, CONFIG_WEBCLIENT_MAXHTTPLINE,
    CONFIG_WEBCLIENT_MAXMIMESIZE, CONFIG_WEBCLIENT_MAXHOSTNAME,
    CONFIG_WEBCLIENT_MAXFILENAME

  Of course, the example also requires other settings including CONFIG_NET and
  CONFIG_NET_TCP.  The example also uses the uIP resolver which requires CONFIG_UDP.

  WARNNG: As of this writing, wget is untested on the target platform.  At present
  it has been tested only in the host-based configuration described in the following
  note.  The primary difference is that the target version will rely on the also
  untested uIP name resolver.

  NOTE: For test purposes, this example can be built as a host-based wget function.
  This can be built as follows:

    cd examples/wget
    make -f Makefile.host

