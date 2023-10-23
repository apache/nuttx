========================
``thttpd`` THTTPD server
========================

An example that builds ``netutils/thttpd`` with some simple NXFLAT CGI programs.
See ``boards/README.txt`` for most THTTPD settings. In addition to those, this
example accepts:

- ``CONFIG_EXAMPLES_THTTPD_NOMAC``    – (May be defined to use software assigned
  MAC)
- ``CONFIG_EXAMPLES_THTTPD_DRIPADDR`` – Default router IP address.
- ``CONFIG_EXAMPLES_THTTPD_NETMASK``  – Network mask.

Applications using this example will need to enable the following ``netutils``
libraries in the ``defconfig`` file: ::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETUTILS_THTTPD=y
