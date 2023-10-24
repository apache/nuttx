===========================
``thttpd`` THTTPD webserver
===========================

This is a port of Jef Poskanzer's THTTPD HTPPD server. See
http://acme.com/software/thttpd/ for general THTTPD information. See
``apps/include/netutils/thttpd.h`` for interface information. Applications using
this ``thttpd`` will need to provide the following definitions in the
``defconfig`` file to select the appropriate ``netutils`` libraries::

  CONFIG_NETUTILS_NETLIB=y
  CONFIG_NETUTILS_THTTPD=y
