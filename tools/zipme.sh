#!/bin/bash
# zipme.sh
#
#   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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

#set -x

WD=`pwd`
VERSION=$1

TAR="tar cvf"
ZIP=gzip

# Make sure we know what is going on

if [ -z ${VERSION} ] ; then
   echo "You must supply a version like xx.yy as a parameter"
   exit 1;
fi

# Find the directory we were executed from and were we expect to
# see the directories to tar up

MYNAME=`basename $0`

if [ -x ${WD}/${MYNAME} ] ; then
   PROJECTS="${WD}/../../.."
else
   if [ -x ${WD}/tools/${MYNAME} ] ; then
     PROJECTS="${WD}/../.."
   else
     if [ -x ${WD}/nuttx/tools/${MYNAME} ] ; then
       PROJECTS="${WD}/.."
     else
       echo "You must cd into the NUTTX directory to execute this script."
       exit 1
     fi
   fi
fi

# Get the NuttX directory name and the path to the parent directory

NUTTXDIR=${PROJECTS}/nuttx-${VERSION}
NUTTX=${NUTTXDIR}/nuttx
APPS=${NUTTXDIR}/apps

# Make sure that the versioned directory exists

if [ ! -d ${NUTTXDIR} ]; then
   echo "Directory ${NUTTXDIR} does not exist"
   exit 1
fi

cd ${PROJECTS} || \
   { echo "Failed to cd to ${PROJECTS}" ; exit 1 ; }

if [ ! -d nuttx-${VERSION} ] ; then
   echo "Directory ${PROJECTS}/nuttx-${VERSION} does not exist!"
   exit 1
fi

if [ ! -d ${APPS} ] ; then
   echo "Directory ${APPS} does not exist!"
   exit 1
fi

TAR_NAME=nuttx-${VERSION}.tar
ZIP_NAME=${TAR_NAME}.gz

# Prepare the nuttx directory -- Remove editor garbage

find ${NUTTXDIR} -name '*~' -exec rm -f '{}' ';' || \
      { echo "Removal of emacs garbage failed!" ; exit 1 ; }
find ${NUTTXDIR} -name '*.swp' -exec rm -f '{}' ';' || \
      { echo "Removal of VI garbage failed!" ; exit 1 ; }

# Make sure that all of the necessary soft links are in place

cd ${NUTTX}/Documentation || \
   { echo "Failed to cd to ${NUTTX}/Documentation" ; exit 1 ; }

ln -sf ../TODO TODO.txt
ln -sf ../ChangeLog ChangeLog.txt

# Perform a full clean for the distribution

cd ${PROJECTS} || \
   { echo "Failed to cd to ${PROJECTS}" ; exit 1 ; }

make -C ${NUTTX} distclean

# Remove any previous tarballs

if [ -f ${TAR_NAME} ] ; then
   echo "Removing ${PROJECTS}/${TAR_NAME}"
   rm -f ${TAR_NAME} || \
      { echo "rm ${TAR_NAME} failed!" ; exit 1 ; }
fi

if [ -f ${ZIP_NAME} ] ; then
   echo "Removing ${PROJECTS}/${ZIP_NAME}"
   rm -f ${ZIP_NAME} || \
      { echo "rm ${ZIP_NAME} failed!" ; exit 1 ; }
fi

# Then zip it

${TAR} ${TAR_NAME} nuttx-${VERSION}/nuttx nuttx-${VERSION}/apps || \
      { echo "tar of ${TAR_NAME} failed!" ; exit 1 ; }
${ZIP} ${TAR_NAME} || \
      { echo "zip of ${TAR_NAME} failed!" ; exit 1 ; }

cd ${NUTTX}
