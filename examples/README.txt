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
  * CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS
      Specifies the number of threads to create in the barrier
      test.  The default is 8 but a smaller number may be needed on
      systems without sufficient memory to start so many threads.

examples/nsh
^^^^^^^^^^^^

  This directory contains the NuttShell (NSH).  This is a simple
  shell application.  With some additional development, NSH will
  someday be a great NuttX application debugger.  NSH is described
  in its own README located at examples/nsh/README.txt

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

examples/null
^^^^^^^^^^^^^

  This is the do nothing application.  It is only used for bringing
  up new NuttX architectures.

examples/hello
^^^^^^^^^^^^^^

  This is the mandatory, "Hello, World!!" example.  It is little more
  than examples/null with a single printf statement.  Again useful only
  for bringing up new NuttX architectures.

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

examples/usbstorage
^^^^^^^^^^^^^^^^^^

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

