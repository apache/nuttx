README
^^^^^^

defconfig
^^^^^^^^^
The default configuration file, defconfig, performs a
simple OS test using examples/ostest.  This can be
configuration as follows:

	cd tools
	./configure.sh c5471evm
	cd -
	. ./setenv.sh

netconfig
^^^^^^^^^
This alternative configuration file, netconfig, may be used
instead of the default configuration (defconfig). This
configuration enables networking using the c5471's built-in
Ethernet interface.  It uses examples/nettest to excercise
the TCP/IP network.

uipconfig
^^^^^^^^^
This configuration file demonstrates the tiny webserver
at examples/uip.

udpconfig
^^^^^^^^^
This alternative configuration file, is similar to netconfig
except that is use examples/upd to exercise UDP.

TThese alternative configurations can be selected by (using
uipconfig as example):

	(Seleted the default configuration as show above)
	cp config/c5471evm/uiponfig .config

