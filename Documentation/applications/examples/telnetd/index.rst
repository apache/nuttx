``telnetd`` Simple Telnet Shell
===============================

This directory contains a functional port of the tiny uIP shell. In the NuttX
environment, the NuttShell (at ``apps/nshlib``) supersedes this tiny shell and
also supports ``telnetd``.

- ``CONFIG_EXAMPLES_TELNETD`` – Enable the Telnetd example.
- ``CONFIG_NETUTILS_NETLIB``, ``CONFIG_NETUTILS_TELNETD`` – Enable netutils libraries
  needed by the Telnetd example.
- ``CONFIG_EXAMPLES_TELNETD_DAEMONPRIO`` – Priority of the Telnet daemon. Default:
  ``SCHED_PRIORITY_DEFAULT``.
- ``CONFIG_EXAMPLES_TELNETD_DAEMONSTACKSIZE`` – Stack size allocated for the
  Telnet daemon. Default: ``2048``.
- ``CONFIG_EXAMPLES_TELNETD_CLIENTPRIO`` – Priority of the Telnet client. Default:
  ``SCHED_PRIORITY_DEFAULT``.
- ``CONFIG_EXAMPLES_TELNETD_CLIENTSTACKSIZE`` – Stack size allocated for the
  Telnet client. Default: ``2048``.
- ``CONFIG_EXAMPLES_TELNETD_NOMAC`` – If the hardware has no MAC address of its
  own, define this ``=y`` to provide a bogus address for testing.
- ``CONFIG_EXAMPLES_TELNETD_IPADDR`` – The target IP address. Default ``10.0.0.2``.
- ``CONFIG_EXAMPLES_TELNETD_DRIPADDR`` – The default router address. Default
  ``10.0.0.1``.
- ``CONFIG_EXAMPLES_TELNETD_NETMASK`` – The network mask. Default:
  ``255.255.255.0``.

Also, make sure that you have the following set in the NuttX configuration file
or else the performance will be very bad (because there will be only one
character per TCP transfer):

- ``CONFIG_STDIO_BUFFER_SIZE`` – Some value ``>= 64``
- ``CONFIG_STDIO_LINEBUFFER=y``
