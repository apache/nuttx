#!/usr/bin/env bash

############################################################################
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# We define a replacement for the quote character (") 
# since we cannot escape quote characters found inside 
# the commit message

Q='^@^'

function getlog
{
  (echo -n '['
   git --no-pager log --follow --simplify-merges \
     --pretty=format:'{ '$Q'commit'$Q': '$Q'%H'$Q', '$Q'author'$Q': '$Q'%aN'$Q', '$Q'author-email'$Q': '$Q'%aE'$Q', '$Q'date'$Q': '$Q'%ad'$Q', '$Q'committer'$Q': '$Q'%cn'$Q', '$Q'committer-email'$Q': '$Q'%ce'$Q', '$Q'message'$Q': '$Q'%s'$Q', '$Q'body'$Q': '$Q'%b'$Q', '$Q'signed'$Q': '$Q'%G?'$Q', '$Q'signer'$Q': '$Q'%GS'$Q', '$Q'key'$Q': '$Q'%GK'$Q' },' -- "$1"
  echo -n ']') | 
    sed -r 's|\\|\\\\|g' |      # escape backquotes
    sed -r 's|"|\\"|g' |        # replace quotes with escaped quotes
    tr '\r\n' ' ' |             # replace newlines with spaces (otherwise strings) are broken
    sed -r 's|\}, \{|},\n{|g' | # add newlines between entries, for readability
    sed 's|\},\]|}]|g' |        # remove dangling comma at the final array entry
    sed -r "s|\\^@\\^|\"|g"     # replace $Q with "
}

function getblobs
{
  echo -n '[';
  git --no-pager log --pretty='' --no-abbrev --raw --follow --simplify-merges -- "$1" |
    tr '\t' ' ' |
    cut -f 4,6 -d ' ' |
    sed -r 's|^(\S+) (\S+)$|{ "blob": "\1", "path": "\2" },|g' |
    sed -r '$s|,$||g'  # remove dangling comma at the final array entry
  echo -n ']'
}

if [ "$1" = "" ]; then
	echo "usage: $(basename $0) <file>"
	exit 1
fi

out1=$(mktemp)
out2=$(mktemp)

getlog "$1" > $out1
getblobs "$1" > $out2

if [ "$(jq '. | length' $out1)" != "$(jq '. | length' $out2)" ]; then
	# TODO: handle this case, we get more than one blob when the file is moved
	# to/from submodule and it difficulties parsing. Also, the blob we get
	# for when they file is in the submodule is not usable since it is possibly
	# from the pointed repo.
	echo "Log and blob list differ in size, probably a file which lived in a submodule"
	echo "Log output is at: $out1"
	echo "Blob output is at: $out2"
	exit 1
fi

jq -s 'transpose | map(.[0] + .[1])' $out1 $out2

rm -f $out1 $out2
