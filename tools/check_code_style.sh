#!/usr/bin/env bash
#
# Copyright (c) 2012 - 2019, PX4 Development Team
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# An example hook script to verify what is about to be committed.
# Called by "git commit" with no arguments.  The hook should
# # exit with non-zero status after issuing an appropriate message if
# it wants to stop the commit.
#

if [ ! "$(git rev-parse --is-inside-work-tree 2>/dev/null)" ]; then
  echo "$0 can only be run with git installed and inside of a git repository"
  exit 1
fi

if [ -z "$1" ]; then
	FILES=$(git diff master --name-only);
else
	FILES=$1
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
for FILE in $FILES; do
  FILENAME=$(basename $FILE)
  FILE_EXT="${FILENAME#*.}"
  if [ -f "$FILE" ]; then
    if [ "$FILE_EXT" = "c" ] || [ "$FILE_EXT" = "h" ]; then
	  CHECK_FAILED=$(${DIR}/nxstyle $FILE)
	#	if [ -n "$CHECK_FAILED" ]; then
	#		${DIR}/fix_code_style.sh --quiet < $FILE > $FILE.pretty
	#
	#		echo
	#		git --no-pager diff --no-index --minimal --histogram --color=always $FILE $FILE.pretty
	#		rm -f $FILE.pretty
	#		echo
	#
	#		if [[ $NXSTYLE_FIX -eq 1 ]]; then
	#			${DIR}/fix_code_style.sh $FILE
	#		else
	#			echo $FILE 'bad formatting, please run "make format" or "./Tools/some tool when we get one/fix_code_style.sh' $FILE'"'
	#			exit 1
	#		fi
	#	fi
  	fi
  fi
done
