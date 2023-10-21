``sendmail`` SMTP Client
========================

This examples exercises the uIP SMTP logic by sending a test message to a
selected recipient. This test can also be built to execute on the Cygwin/Linux
host environment::

  cd examples/sendmail
  make -f Makefile.host TOPDIR=<nuttx-directory>

Settings unique to this example include:

- ``CONFIG_EXAMPLES_SENDMAIL_NOMAC``     – May be defined to use software assigned
  MAC (optional)
- ``CONFIG_EXAMPLES_SENDMAIL_IPADDR``    – Target IP address (required)
- ``CONFIG_EXAMPLES_SENDMAIL_DRIPADDR``  – Default router IP address (required)
- ``CONFIG_EXAMPLES_SENDMAILT_NETMASK``  – Network mask (required)
- ``CONFIG_EXAMPLES_SENDMAIL_RECIPIENT`` – The recipient of the email (required)
- ``CONFIG_EXAMPLES_SENDMAIL_SENDER``    – Optional. Default:
  ``nuttx-testing@example.com``
- ``CONFIG_EXAMPLES_SENDMAIL_SUBJECT``   – Optional. Default: ``Testing SMTP from
  NuttX``
- ``CONFIG_EXAMPLES_SENDMAIL_BODY``      – Optional. Default: ``Test message sent
  by NuttX``

**Note 1**: This test has not been verified on the NuttX target environment. As
of this writing, unit-tested in the Cygwin/Linux host environment.

**Note 2**: This sendmail example only works for the simplest of environments.
Virus protection software on your host may have to be disabled to allow you to
send messages. Only very open, unprotected recipients can be used. Most will
protect themselves from this test email because it looks like SPAM.

Applications using this example will need to enable the following netutils
libraries in their defconfig file: ::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETUTILS_SMTP=y
