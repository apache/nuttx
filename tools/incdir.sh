#!/bin/bash
# tools/incdir.sh
#
#   Copyright (C) 2008 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

# Input parameters:
#  $1 : Compiler name as it appears in  config/*/*/Make.defs
#  $2, $3, ...: Include file paths

compiler=$1
shift
dirlist=$@

usage="USAGE: $0 <compiler-path> <dir1> [<dir2> [<dir3> ...]]"

if [ -z "$compiler" ]; then
	echo "Missing compiler path"
	echo $usage
	exit 1
fi

if [ -z "$dirlist" ]; then
	echo "Missing include directory list"
	echo $usage
	exit 1
fi

#
# Most compilers support CFLAG options like '-I<dir>' to add include
# file header paths.  Some (like the Zilog tools), do not.  This script
# makes the select of header file paths compiler independent.
#
# Below are all known compiler names (as found in the config/*/*/Make.defs
# files).  If a new compiler is used that has some unusual syntax, then
# additional logic needs to be added to this file.
#
#   NAME                        Syntax
#   $(CROSSDEV)gcc              -I<dir1> -I<dir2> -I<dir3> ...
#   sdcc                        -I<dir2> -I<dir2> -I<dir3> ...
#   $(ZDSBINDIR)/ez8cc.exe      -usrinc:'<dir1>:<dir2:<dir3>:...`
#   $(ZDSBINDIR)/zneocc.exe     -usrinc:'<dir1>:<dir2:<dir3>:...`
#   $(ZDSBINDIR)/eZ80cc.exe     -usrinc:'<dir1>:<dir2:<dir3>:...`
#
# Furthermore, just to make matters more difficult, with Windows based
# toolchains, we have to use the full windows-style paths to the header
# files.

fmt=std
windows=no

exefile=`basename $compiler`
if [ "X$exefile" = "Xez8cc.exe" -o "X$exefile" = "Xzneocc.exe" -o "X$exefile" = "XeZ80cc.exe" ]; then
	fmt=userinc
	windows=yes
fi

# Now process each directory in the directory list

unset response
for dir in $dirlist; do

	# Verify that the include directory exists

	if [ ! -d $dir ]; then
		echo "Include path '$dir' does not exist"
		echo $showusage
		exit 1
	fi

	# Check if the path needs to be extended for Windows-based tools under Cygwin

	if [ "X$windows" = "Xyes" ]; then
		path=`cygpath -w $dir`
	else
		path=$dir
	fi

	# Handle the output using the selected format

	if [ "X$fmt" = "Xuserinc" ]; then
		# Treat the first directory differently

		if [ -z "$response" ]; then
			response="-usrinc:'"$path
		else
			response=$response":$path"
		fi
	else
		# Treat the first directory differently

		if [ -z "$response" ]; then
			response=-I$path
		else
			response=$response" -I$path"
		fi
	fi
done

if [ "X$fmt" = "Xuserinc" ]; then
	response=$response"'"
else
	response=\"$response\"
fi

echo $response


