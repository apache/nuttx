README
^^^^^^

Each Neuros OSD configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh ntosd-dm320/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest
^^^^^^
This configuration directory, performs a simple OS test using
examples/ostest.

nsh
^^^
Configures the NuttShell (nsh) located at examples/nsh.  The
Configuration enables both the serial and telnetd NSH interfaces.

nettest
^^^^^^^
This alternative configuration directory may be used to
enable networking using the OSDs DM9000A Ethernet interface.
It uses examples/nettest to excercise the TCP/IP network.

uip
^^^
This configuration file demonstrates the tiny webserver
at examples/uip.

udp
^^^
This alternative configuration directory is similar to nettest
except that is use examples/upd to exercise UDP.

