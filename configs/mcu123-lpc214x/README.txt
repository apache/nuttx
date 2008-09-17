README
^^^^^^

Each Phillips LPC214x configuration is maintained in a sudirectory and
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
