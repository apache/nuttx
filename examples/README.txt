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

  This directory containst the NuttShell (NSH).  This is a primitive
  shell-like application.  With some additional development, NSH will
  someday be a great NuttX application debugger.

  The behavior of NSH can be modified with the following settings in
  the configs/<board-name>/defconfig file:

  * CONFIG_NSH_IOBUFFERSIZE
      Size of a static I/O buffer used for file access (ignored if
      there is no filesystem).

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

