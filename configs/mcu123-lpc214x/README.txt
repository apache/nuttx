README
^^^^^^

Tools
^^^^^

I use the lpc21isp tool to load NuttX into FLASH.  That tool is available
in the files section at http://tech.groups.yahoo.com/group/lpc21isp/.  In
order version 1.60 of lpc21isp for Linux, I had to make several changes.
This changes are shown in lpc21ips-1.60.diff.

I use the script lpc21isp.sh to perform the actual download.  You will
probably have to make some changes to this script in order to use it.
For example, the path to the built lpc21isp binary will most likely
have to have change.  Then move this script to the top level NuttX
directory and simply execute it to load NuttX onto the board.

Configurations
^^^^^^^^^^^^^^

Each NXP LPC214x configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh mcu123-lpc214x/<subdir>
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
Configuration enables only the serial NSH interfaces.

