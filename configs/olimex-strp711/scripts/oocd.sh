#!/bin/sh

# Set up pathes to binaries, scripts, configuration files

installdir=/usr/local/bin
openocd=$installdir/openocd

# The root to the top-level NuttX directory should be in an environment variable

if [ -z $STR41XCFGS ]; then
	echo "Environment variable $STR41XCFGS is not defined"
	echo "Has NuttX been configured?"
	echo "If so, try sourcing the setenv.sh script in the top-level directory"
	exit 1
fi

# Check that at least one configuration file exists at that point
if [ ! -f $STR41XCFGS/oocd_wiggler.cfg ]; then
	echo "No configuration files found at $STR41XCFGS"
	echo "Path to configuration files unknown"
	exit 1
fi

# Parse command line inputs

usage="USAGE: $0 [-h] [-d]  [-pp] [-ftdi]"

debug=no
interface=pp
oocdcfg=$STR41XCFGS/oocd_wiggler.cfg
while [ ! -z "$1" ]; do
	case $1 in
	-d )
		debug=yes
		set -x
		;;
	-pp )
		interface=pp
		oocdcfg=$STR41XCFGS/oocd_wiggler.cfg
		;;
	-ftdi )
		interface=ftdi
		oocdcfg=$STR41XCFGS/oocd_ftdi.cfg
		;;
	-h )
		echo $usage
		exit 0
		;;
	* )
		echo "Unrecognized option: $1"
		echo $usage
		exit 1
		;;
	esac
	shift
done

# Setup debug options

export options="-d 1"

# Run OpenOCD -- here it is assumed (1) that you must have root priveleges to 
# execute OpenOCD and (2) that your user is listed in the /etc/sudoers file.

sudo $openocd $options -f $oocdcfg
