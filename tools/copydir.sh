#!/usr/bin/env bash
############################################################################
# tools/copydir.sh
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
#
# NuttX uses symbolic links to configure platform-specific directories into
# the build system.  This works great except for when a Windows native
# toolchain is used in a Cygwin environment.  In that case, symbolic
# links do not work correctly when accessed from the Windows native toolchain;
# rather, just look link files with the extension .lnk
#
# In this environment, the build system will work around this using this script
# as a replacement for the 'ln' command.  This scrpt will simply copy the
# directory into the expected positiion.
#
#set -x

src=$1
dest=$2

# Verify that arguments were provided

if [ -z "${src}" -o -z "${dest}" ]; then
  echo "Missing src and/or dest arguments"
  exit 1
fi

# Check if something already exists at the destination path replace it with
# the new link (which might be different).  Note that we check for the
# the link (-h) before we check for existence (-e) because a bad link will
# report that it does not exist.

if [ -h "${dest}" ]; then
  rm -f "${dest}"
else

  # If the path exists and is a directory that contains the "fake link"
  # mark, then treat it like a soft link (i.e., remove the directory)

  if [ -d "${dest}" -a -f "${dest}/.fakelnk" ]; then
    rm -rf "${dest}"
  else

    # Does anything exist at the destination path?

    if [ -e "${dest}" ]; then

      # It is something else (like a file) or directory that does
      # not contain the "fake link" mark

      echo "${dest} already exists but is not a symbolic link"
      exit 1
    fi
  fi
fi

# Verify that a directory exists at the source path

if [ ! -d "${src}" ]; then
  echo "No directory at ${src}"
  exit 1
fi

# Copy the directory

cp -a "${src}" "${dest}" || \
  { echo "Failed to create link: $dest" ; rm -rf ${dest} ; exit 1 ; }
touch "${dest}/.fakelnk" || \
  { echo "Failed to touch ${dest}/.fakelnk" ; rm -rf ${dest} ; exit 1 ; }
