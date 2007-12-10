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

nshconfig
^^^^^^^^^
This configuration file builds NSH (examples/nsh) using the
TELNET server front end

These alternative configurations can be selected by (using
uipconfig as example):

	(Seleted the default configuration as show above)
	cp config/c5471evm/uiponfig .config

