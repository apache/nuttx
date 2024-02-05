================================
``telnetd`` Telnet server daemon
================================

This is the Telnet logic adapted from uIP and generalized for use as the front
end to any shell. The telnet daemon creates sessions that are "wrapped" as
character devices and mapped to ``stdin``, ``stdout`` and ``stderr``.
Now the telnet session can be inherited by spawned tasks.

Tips for Using Telnetd
----------------------

Telnetd is set up to be the front end for a shell. The primary use of Telnetd in
NuttX is to support the NuttShell (NSH) Telnet front end. See
``apps/include/netutils/telnetd.h`` for information about how to incorporate
Telnetd into your custom applications.

To enable and link the Telnetd daemon, you need to include the following in in
your defconfig file::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETUTILS_TELNETD=y

Also if the Telnet console is enabled, make sure that you have the following set
in the NuttX configuration file or else the performance will be very bad
(because there will be only one character per TCP transfer):

- ``CONFIG_STDIO_BUFFER_SIZE`` – Some value ``>= 64``.
- ``CONFIG_STDIO_LINEBUFFER=y`` – Since Telnetd is line oriented, line buffering
  is optimal.
