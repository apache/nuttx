===============================
``nsh`` NuttShell (NSH) example
===============================

Basic Configuration
-------------------

This directory provides an example of how to configure and use the NuttShell
(NSH) application. NSH is a simple shell application. NSH is described in its
own README located at ``apps/nshlib/README.md``. This function is enabled with::

  CONFIG_SYSTEM_NSH=y

Applications using this example will need to provide an ``defconfig`` file in the
configuration directory with instruction to build the NSH library like::

  CONFIG_NSH_LIBRARY=y

Other Configuration Requirements
--------------------------------

**Note**: If the NSH serial console is used, then following is also required to
build the ``readline()`` library::

  CONFIG_SYSTEM_READLINE=y

And if networking is included::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETUTILS_DHCPC=y
  CONFIG_NETDB_DNSCLIENT=y
  CONFIG_NETUTILS_TFTPC=y
  CONFIG_NETUTILS_WEBCLIENT=y

If the Telnet console is enabled, then the defconfig file should also include::

  CONFIG_NETUTILS_TELNETD=y

Also if the Telnet console is enabled, make sure that you have the following set
in the NuttX configuration file or else the performance will be very bad
(because there will be only one character per TCP transfer):

- ``CONFIG_STDIO_BUFFER_SIZE`` - Some value ``>= 64``
- ``CONFIG_STDIO_LINEBUFFER=y``
