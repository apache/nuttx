README
^^^^^^

defconfig
^^^^^^^^^
The default configuration file, defconfig, performs a
simple OS test using examples/ostest.  This can be
configuration as follows:

	cd tools
	./configure.sh ntosd-dm320
	cd -
	. ./setenv.sh

netconfig
^^^^^^^^^
This alternative configuration file, netconfig, may be used
instead of the default configuration (defconfig). This
configuration enables networking using the OSDs DM9000A
Ethernet interface.  It uses examples/nettest to excercise
the TCP/IP network.

uipconfig
^^^^^^^^^
This configuration file demonstrates the tiny webserver
at examples/uip.

These alternative configurations can be selected by

	(Seleted the default configuration as show above)
	cp config/ntosd-dm320/uiponfig .config

udpconfig
^^^^^^^^^
This alternative configuration file, is similar to netconfig
except that is use examples/upd to exercise UDP.

