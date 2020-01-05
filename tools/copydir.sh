#!/usr/bin/env bash
############################################################################
# tools/copydir.sh
#
#   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
