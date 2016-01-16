#!/bin/sh
############################################################################
# tools/indent.sh
#
#   Copyright (C) 2008, 2010, 2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
#
# This script uses the Linux 'indent' utility to re-format C source files
# to match the coding style that I use.  It differs from my coding style in that
#
# - I normally put the trailing */ of a multi-line comment on a separate line,
# - I usually align things vertically (like '=' in assignments),
# - indent puts a bogus blank line at the top of the file,
# - I don't like the way it handles nested conditional compilation intermixed with code.
# - I also indent brackets differently on structures than does this script.
#

# Constants

options="-nbad -bap -bbb -nbbo -nbc -bl -bl2 -bls -nbs -cbi2 -ncdw -nce -ci2 -cli0 -cp40 -ncs -nbfda -nbfde -di1 -nfc1 -fca -i2 -l80 -lp -ppi2 -lps -npcs -pmt -nprs -npsl -saf -sai -sbi2 -saw -sc -sob -nss -nut"

advice="Try '$0 -h' for more information"

# Parse inputs

unset filelist
unset outfile
files=none
mode=inplace

while [ ! -z "${1}" ]; do
	case ${1} in
	-d )
		set -x
		;;
	-o )
		shift
		outfile=${1}
		mode=copy
		;;
	-h )
		echo "$0 is a tool for generation of proper version files for the NuttX build"
		echo ""
		echo "USAGE:"
		echo "	$0 [-d] -o <out-file> <in-file>"
		echo "	$0 [-d] <in-file-list>"
		echo "	$0 [-d] -h"
		echo ""
		echo "Where:"
		echo "	-<in-file>"
		echo "		A single, unformatted input file"
		echo "	-<in-file-list>"
		echo "		A list of unformatted input files that will be reformatted in place."
		echo "	-o <out-file>"
		echo "		Write the single, reformatted <in-file> to <out-file>.  <in-file>"
		echo "		will not be modified."
		echo "	-d"
		echo "		Enable script debug"
		echo "	-h"
		echo "		Show this help message and exit"
		exit 0
		;;
	* )
		if [ ! -r ${1} ]; then
			echo "Readable ${1} does not exist"
			echo ${advice}
			exit 1
		fi
		if [ -z "${filelist}" ]; then
			filelist="${1}"
			files=single
		else
			filelist="${filelist} ${1}"
			files=multiple
		fi
		;;
	esac
	shift
done

# Verify that at least one input file was provided

if [ "X${files}" == "Xnone" ]; then
	echo "ERROR: Neither <in-file> nor <in-file-list> provided"
	echo ${advice}
	exit 1
fi

# Perform the indentation

if [ "X${mode}" == "Xcopy" ]; then
	if [ "X${files}" == "Xmultiple" ]; then
		echo "ERROR: Only a single <in-file> can be used with the -o option"
		echo ${advice}
		exit 1
	fi
	if [ -f $outfile ]; then
		echo "Removing old $outfile"
		rm $outfile || { echo "Failed to remove $outfile" ; exit 1 ; }
	fi
	indent $options $filelist -o $outfile
else
	indent $options $filelist
fi
