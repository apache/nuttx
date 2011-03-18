examples
^^^^^^^^

  The examples directory contains several sample applications that
  can be linked with NuttX.  The specific example is selected in the
  configs/<board-name>/defconfig file via the CONFIG_APP_DIR setting.
  This setting provides the path to the directory containing the
  application Makefile (this path is a relative to the NuttX top-
  level directory).  For example, 

    CONFIG_APP_DIR=examples/ostest

  Selects the examples/ostest example.

examples/dhcpd
^^^^^^^^^^^^^^

  This examples builds a tiny DCHP server for the target system.
  
  NOTE: For test purposes, this example can be built as a
  host-based DHCPD server.  This can be built as follows:

    cd examples/dhcpd
    make -f Makefile.host

  NuttX configuration settings:
  
    CONFIG_NET=y                   - Of course
    CONFIG_NSOCKET_DESCRIPTORS     - And, of course, you must allocate some
                                     socket descriptors.
    CONFIG_NET_UDP=y               - UDP support is required for DHCP
                                     (as well as various other UDP-related
                                     configuration settings)
    CONFIG_NET_BROADCAST=y         - UDP broadcast support is needed.

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

  NuttX configuration settings:

    CONFIG_EXAMPLE_HELLOXX_NOSTATICCONST - Set if system does not support
      static constructors.
    CONFIG_EXAMPLE_HELLOXX_NOSTACKCONST - Set if the systgem does not
      support constructionof objects on the stack.

examples/helloxx
^^^^^^^^^^^^^^^^

  This is C++ version of the "Hello, World!!" example.  It is intended
  only to verify that the C++ compiler is function, that basic C++
  library suupport is available, and that class are instantiated
  correctly.

examples/hidkbd
^^^^^^^^^^^^^^^^

  This is a simple test to debug/verify the USB host HID keyboard class
  driver.

    CONFIG_EXAMPLES_HIDKBD_DEFPRIO - Priority of "waiter" thread.
    CONFIG_EXAMPLES_HIDKBD_STACKSIZE - Stacksize of "waiter" thread.

examples/igmp
^^^^^^^^^^^^^

  This is a trivial test of the NuttX IGMP capability.  It present it
  does not do much of value -- Much more is needed in order to verify
  the IGMP features!

  * CONFIG_EXAMPLE_IGMP_NOMAC
      Set if the hardware has no MAC address; one will be assigned
  * CONFIG_EXAMPLE_IGMP_IPADDR
      Target board IP address
  * CONFIG_EXAMPLE_IGMP_DRIPADDR
      Default router address
  * CONFIG_EXAMPLE_IGMP_NETMASK
      Network mask
  * CONFIG_EXAMPLE_IGMP_GRPADDR
      Multicast group address

examples/mm
^^^^^^^^^^^

  This is a simplified version of the "built-in" memory manager test of
  mm/mm_test.c.  It is simplified because it does not have access to the
  internals of the memory manager as does mm/mm_test.c, but it has the
  advantage that it runs in the actual NuttX tasking environment (the
  mm/mm_test.c only runs in a PC simulation environment).

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

  This directory provides an example of how to configure and use
  the NuttShell (NSH) application.  NSH is a simple shell
  application.  NSH is described in its own README located at
  apps/nshlib/README.txt

examples/nx
^^^^^^^^^^^

  This directory contains a simple test of a subset of the NX APIs
  defined in include/nuttx/nx.h.  The following configuration options
  can be selected:

    CONFIG_EXAMPLES_NX_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NX_DEVNO - The LCD device to select from the LCD
      driver for use in the test: Default: 0
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
    CONFIG_EXAMPLES_NX_EXTERNINIT - The driver for the graphics device on
      this platform requires some unusual initialization.  This is the
      for, for example, SPI LCD/OLED devices.  If this configuration is
      selected, then the platform code must provide an LCD initialization
      function with a prototype like:

      #ifdef CONFIG_NX_LCDDRIVER
      FAR struct lcd_dev_s *up_nxdrvinit(unsigned int devno);
      #else
      FAR struct fb_vtable_s *up_nxdrvinit(unsigned int devno);
      #endif

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

examples/nxflat
^^^^^^^^^^^^^^^

  This example builds a small NXFLAT test case.  This includes several
  test programs under examples/nxflat tests.  These tests are build using
  the NXFLAT format and installed in a ROMFS file system.  At run time,
  each program in the ROMFS file system is executed.  Requires CONFIG_NXFLAT.

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

examples/sendmail
^^^^^^^^^^^^^^^^^

  This examples exercises the uIP SMTP logic by sending a test message
  to a selected recipient.  This test can also be built to execute on
  the Cygwin/Linux host environment:

    cd examples/sendmail
    make -f Makefile.host

 Settings unique to this example include:

    CONFIG_EXAMPLE_SENDMAIL_NOMAC     - May be defined to use software assigned MAC (optional)
    CONFIG_EXAMPLE_SENDMAIL_IPADDR    - Target IP address (required)
    CONFIG_EXAMPLE_SENDMAIL_DRIPADDR  - Default router IP addess (required)
    CONFIG_EXAMPLE_SENDMAILT_NETMASK  - Network mask (required)
    CONFIG_EXAMPLE_SENDMAIL_RECIPIENT - The recipient of the email (required)
    CONFIG_EXAMPLE_SENDMAIL_SENDER    - Optional. Default: "nuttx-testing@example.com"
    CONFIG_EXAMPLE_SENDMAIL_SUBJECT   - Optional. Default: "Testing SMTP from NuttX"
    CONFIG_EXAMPLE_SENDMAIL_BODY   -    Optional. Default: "Test message sent by NuttX"

  NOTE: This test has not been verified on the NuttX target environment.
  As of this writing, unit-tested in the Cygwin/Linux host environment.

  NOTE 2: This sendmail example only works for the simplest of 
  environments.  Virus protection software on your host may have
  to be disabled to allow you to send messages.  Only very open,
  unprotected recipients can be used.  Most will protect themselves
  from this test email because it looks like SPAM.

examples/serloop
^^^^^^^^^^^^^^^^

  This is a mindlessly simple loopback test on the console.  Useful
  for testing new serial drivers.  Configuration options include:

  * CONFIG_EXAMPLES_SERLOOP_BUFIO
      Use C buffered I/O (getchar/putchar) vs. raw console I/O
      (read/read).

examples/thttpd
^^^^^^^^^^^^^^^

  An example that builds netutils/thttpd with some simple NXFLAT
  CGI programs.  see configs/README.txt for most THTTPD settings.
  In addition to those, this example accepts:

    CONFIG_EXAMPLE_THTTPD_NOMAC    - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLE_THTTPD_DRIPADDR - Default router IP addess
    CONFIG_EXAMPLE_THTTPD_NETMASK  - Network mask

examples/udp
^^^^^^^^^^^^

  This is a simple network test for verifying client- and server-
  functionality over UDP.

examples/uip
^^^^^^^^^^^^

  This is a port of uIP tiny webserver example application.  Settings
  specific to this example include:

    CONFIG_EXAMPLE_UIP_NOMAC     - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLE_UIP_IPADDR    - Target IP address
    CONFIG_EXAMPLE_UIP_DRIPADDR  - Default router IP addess
    CONFIG_EXAMPLE_UIP_NETMASK   - Network mask
    CONFIG_EXAMPLE_UIP_DHCPC     - Select to get IP address via DHCP

  If you use DHCPC, then some special configuration network options are
  required.  These include:

    CONFIG_NET=y                 - Of course
    CONFIG_NSOCKET_DESCRIPTORS   - And, of course, you must allocate some
                                   socket descriptors.
    CONFIG_NET_UDP=y             - UDP support is required for DHCP
                                   (as well as various other UDP-related
                                   configuration settings).
    CONFIG_NET_BROADCAST=y       - UDP broadcast support is needed.
    CONFIG_NET_BUFSIZE=650       - Per RFC2131 (p. 9), the DHCP client must be
    (or larger)                    prepared to receive DHCP messages of up to
                                   576 bytes (excluding Ethernet, IP, or UDP
                                   headers and FCS).

  Other configuration items apply also to the selected webserver net utility.
  Additional relevant settings for the uIP webserver net utility are:

    CONFIG_NETUTILS_HTTPDSTACKSIZE
    CONFIG_NETUTILS_HTTPDFILESTATS
    CONFIG_NETUTILS_HTTPDNETSTATS

examples/usbserial
^^^^^^^^^^^^^^^^^^

  TARGET CONFIGURATION:

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

    If CONFIG_USBDEV_TRACE is enabled (or CONFIG_DEBUG and CONFIG_DEBUG_USB), then
    the example code will also manage the USB trace output.  The amount of trace output
    can be controlled using:

    CONFIG_EXAMPLES_USBSERIAL_TRACEINIT
      Show initialization events
    CONFIG_EXAMPLES_USBSERIAL_TRACECLASS
      Show class driver events
    CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS
      Show data transfer events
    CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER
      Show controller events
    CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS
      Show interrupt-related events.

    Error results are always shown in the trace output

  HOST-SIDE TEST PROGRAM

    In additional to the target device-side example, there is also a
    host-side application in this directory.  This host side application
    must be executed on a Linux host in order to perform the USBSERIAL
    test.  The host application can be compiled under Linux (or Cygwin?)
    as follows:

      cd examples/usbserial
      make -f Makefile.host TOPDIR=../../.

  RUNNING THE TEST

    This will generate a small program called 'host'.  Usage:

    1. Build the examples/usbserial target program and start the target.

    2. Wait a bit, then do enter:

       dmesg

       At the end of the dmesg output, you should see the serial
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
  the device using the USB storage class driver.  In order to use this
  example, your board-specific logic must provide the function:

    void usbstrg_archinitialize(void);

  This function will be called by the example/usbstorage in order to
  do the actual registration of the block device drivers.  For examples
  of the implementation of usbstrg_archinitialize() see
  configs/mcu123-lpc124x/src/up_usbstrg.c or
  configs/stm3210e-eval/src/usbstrg.c

  Configuration options:

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

