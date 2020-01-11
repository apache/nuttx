#!/bin/bash
############################################################################
# tools/listcommonpercentage.sh
#
#   Copyright (C) 2020 Pieter du Preez. All rights reserved.
#   Author: Pieter du Preez	(https://github.com/wingunder)
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

############################################################################
#
# Usage: listcommonpercentage.sh file0 file1 [...]
#
# This script uses 'wdiff -123 -s' in order to get the relative percentage
# of common lines of text of each supplied file, relative to all other
# supplied files. It can be used to identify copies or semi-copies of files.
#
# The default delimiter of the output is a space. This can be changed, by
# setting the DELIM environment variable to the desired delimiter string.
# eg. For a CSV-type output, do the following:
#   DELIM=";" listcommonpercentage.sh file0 file1 file3 file4
#
# The output of this script is 4 delimited terms:
#
#   1. The first of the 2 files, being diffed.
#   2. The common text percentage, relative to the first file.
#   3. The second of the 2 files, being diffed.
#   4. The common text percentage, relative to the second file.
#
# Please note that:
#
#   - all supplied files are diffed with each other which can quickly
#     escalate into many diffs, as N supplied files there will be (N-1)!
#     results (that is Factorial(N-1)).
#
#   - this script requires the following programs to run:
#
#     - bash, wdiff, awk, cut and tr
#     - optionally (see examples): find and sort
#
# Examples:
#
# Here follows a practical example that will recursively find all files
# called suspectedCopiedFile.txt in the current directory and use it
# as input. The delimiter is set to ";", for a CSV-type output. The
# percentage sign gets omitted as PERCENT_SIGN is set to an empty string.
#
#   find . -type f -name suspectedCopiedFile.txt | \
#   PERCENT_SIGN="" DELIM=";" xargs ./listcommonpercentage.sh
#
# Sorting the output by the common percentage, to identify copied files,
# is now pretty straight forward:
#
#   find . -type f -name suspectedCopiedFile.txt | \
#   PERCENT_SIGN="" xargs ./listcommonpercentage.sh |sort -nk2 -nk4
#
# And the sorting can also easily be reversed:
#
#   find . -type f -name suspectedCopiedFile.txt | \
#   PERCENT_SIGN="" xargs ./listcommonpercentage.sh |sort -rnk2 -rnk4
#
############################################################################

DELIM="${DELIM:- }"

PERCENT_SIGN="${PERCENT_SIGN-%}"

args=("$@")
for i in $(seq 0 $(($#-2)))
do
  for j in $(seq $(($i+1)) $(($#-1)))
  do
	changed=`wdiff -123 -s ${args[$i]} ${args[$j]} \
	  |sed -e 's/://' |awk '{print  $5}' |tr -d '%' |xargs`
	iChanged=`echo $changed |cut -d' ' -f1`
	jChanged=`echo $changed |cut -d' ' -f2`
	printf '%s%s%d%s%s%s%s%d%s\n' ${args[$i]} \
	  "$DELIM" $iChanged "$PERCENT_SIGN" \
	  "$DELIM" ${args[$j]} \
	  "$DELIM" $jChanged "$PERCENT_SIGN"
  done
done
