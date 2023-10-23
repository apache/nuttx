===================
``ftpd`` FTP daemon
===================

This example exercises the FTPD daemon at ``apps/netutils/ftpd``. Below are
configurations specific to the FTPD example (the FTPD daemon itself may require
other configuration options as well).

- ``CONFIG_EXAMPLES_FTPD`` – Enable the FTPD example.
- ``CONFIG_EXAMPLES_FTPD_PRIO`` – Priority of the FTP daemon. Default:
  ``SCHED_PRIORITY_DEFAULT``.
- ``CONFIG_EXAMPLES_FTPD_STACKSIZE`` – Stack size allocated for the FTP daemon.
  Default: ``2048``.
- ``CONFIG_EXAMPLES_FTPD_NONETINIT`` – Define to suppress configuration of the
  network by ``apps/examples/ftpd``. You would need to suppress network
  configuration if the network is configuration prior to running the example.

NSH always initializes the network so if ``CONFIG_NSH_NETINIT`` is defined, so is
``CONFIG_EXAMPLES_FTPD_NONETINIT`` (se it does not explicitly need to be defined
in that case):

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the FTPD daemon example test as an NSH
  built-in function. By default the FTPD daemon will be built as a standalone
  application.

If ``CONFIG_EXAMPLES_FTPD_NONETINIT`` is not defined, then the following may be
specified to customized the network configuration:

- ``CONFIG_EXAMPLES_FTPD_NOMAC`` – If the hardware has no MAC address of its own,
  define this ``=y`` to provide a bogus address for testing.
- ``CONFIG_EXAMPLES_FTPD_IPADDR`` – The target IP address. Default ``10.0.0.2``.
- ``CONFIG_EXAMPLES_FTPD_DRIPADDR`` – The default router address. Default:
  ``10.0.0.1``.
- ``CONFIG_EXAMPLES_FTPD_NETMASK`` – The network mask. Default: ``255.255.255.0``.

TCP networking support is required. So are pthreads so this must be set to 'n':

- ``CONFIG_DISABLE_PTHREAD`` – ``pthread`` support is required.

Other FTPD configuration options they may be of interest:

- ``CONFIG_FTPD_VENDORID`` – The vendor name to use in FTP communications.
  Default: ``NuttX``.
- ``CONFIG_FTPD_SERVERID`` – The server name to use in FTP communications.
  Default: ``NuttX FTP Server``.
- ``CONFIG_FTPD_CMDBUFFERSIZE`` – The maximum size of one command. Default: ``512``
  bytes.
- ``CONFIG_FTPD_DATABUFFERSIZE`` – The size of the I/O buffer for data transfers.
  Default: ``2048`` bytes.
- ``CONFIG_FTPD_WORKERSTACKSIZE`` – The stacksize to allocate for each FTP daemon
  worker thread. Default: ``2048`` bytes.

The following netutils libraries should be enabled in your ``defconfig`` file: ::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETUTILS_FTPD=y
