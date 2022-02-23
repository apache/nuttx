#!/usr/bin/env bash
# tools/mkexport.sh
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

# Get the input parameter list

USAGE="USAGE: $0 [-d] [-z] [-u] [-t <top-dir> [-x <lib-ext>] [-a <apps-dir>] [-m <make-exe>] -l \"lib1 [lib2 [lib3 ...]]\""
unset TOPDIR
unset LIBLIST
unset TGZ
unset APPDIR
unset BOARDDIR

USRONLY=n
LIBEXT=.a

while [ ! -z "$1" ]; do
  case $1 in
  -a )
    shift
    APPDIR="$1"
    ;;
  -b )
    shift
    BOARDDIR="$1"
    ;;
  -d )
    set -x
    ;;
  -l )
    shift
    LIBLIST=$1
    ;;
  -m )
    shift
    MAKE="$1"
    ;;
  -t )
    shift
    TOPDIR=$1
    ;;
  -u )
    USRONLY=y
    ;;
  -x )
    shift
    LIBEXT=$1
    ;;
  -z )
    TGZ=y
    ;;
  -h )
    echo $USAGE
    exit 0
    ;;
  * )
    echo "Unrecognized argument: $1"
    echo $USAGE
    exit 1
    ;;
  esac
  shift
done

# Check arguments

if [ -z "${TOPDIR}" -o -z "${LIBLIST}" ]; then
  echo "MK: Missing required arguments"
  echo $USAGE
  exit 1
fi

if [ ! -d "${TOPDIR}" ]; then
  echo "MK: Directory ${TOPDIR} does not exist"
  exit 1
fi

# Check configuration
# Verify that we have Make.defs, .config, and .version files.

if [ ! -f "${TOPDIR}/Make.defs" ]; then
  echo "MK: Directory ${TOPDIR}/Make.defs does not exist"
  exit 1
fi

if [ ! -f "${TOPDIR}/.config" ]; then
  echo "MK: Directory ${TOPDIR}/.config does not exist"
  exit 1
fi

if [ ! -f "${TOPDIR}/.version" ]; then
  echo "MK: File ${TOPDIR}/.version does not exist"
  exit 1
fi

# Check if the make environment variable has been defined

if [ -z "${MAKE}" ]; then
  MAKE=`which make`
fi

# Get the version string

source "${TOPDIR}/.version"
if [ ! -z "${CONFIG_VERSION_STRING}" -a "${CONFIG_VERSION_STRING}" != "0.0" ]; then
  VERSION="-${CONFIG_VERSION_STRING}"
fi

# Create the export directory

EXPORTSUBDIR="nuttx-export${VERSION}"
EXPORTDIR="${TOPDIR}/${EXPORTSUBDIR}"

# If the export directory already exists, then remove it and create a new one

if [ -d "${EXPORTDIR}" ]; then
  echo "MK: Removing old export directory"
  rm -rf "${EXPORTDIR}"
fi

# Remove any possible previous results

rm -f "${EXPORTDIR}.tar"
rm -f "${EXPORTDIR}.zip"
rm -f "${EXPORTDIR}.tar.gz"

# Create the export directory and some of its subdirectories

mkdir "${EXPORTDIR}" || { echo "MK: 'mkdir ${EXPORTDIR}' failed"; exit 1; }
mkdir "${EXPORTDIR}/startup" || { echo "MK: 'mkdir ${EXPORTDIR}/startup' failed"; exit 1; }
mkdir "${EXPORTDIR}/libs" || { echo "MK: 'mkdir ${EXPORTDIR}/libs' failed"; exit 1; }
mkdir "${EXPORTDIR}/scripts" || { echo "MK: 'mkdir ${EXPORTDIR}/scripts' failed"; exit 1; }
mkdir "${EXPORTDIR}/tools" || { echo "MK: 'mkdir ${EXPORTDIR}/tools' failed"; exit 1; }

if [ "X${USRONLY}" != "Xy" ]; then
  mkdir "${EXPORTDIR}/arch" || { echo "MK: 'mkdir ${EXPORTDIR}/arch' failed"; exit 1; }
fi

# Copy the .config file

cp -a "${TOPDIR}/.config" "${EXPORTDIR}/.config" ||
  { echo "MK: Failed to copy ${TOPDIR}/.config to ${EXPORTDIR}/.config"; exit 1; }

# Copy the Make.defs files

cp -a "${TOPDIR}/Make.defs" "${EXPORTDIR}/Make.defs" ||
  { echo "MK: Failed to copy ${TOPDIR}/Make.defs to ${EXPORTDIR}/Make.defs"; exit 1; }

# Extract information from the Make.defs file.  A Makefile can do this best

${MAKE} -C "${TOPDIR}/tools" -f Export.mk TOPDIR="${TOPDIR}" EXPORTDIR="${EXPORTDIR}"
source "${EXPORTDIR}/makeinfo.sh"
rm -f "${EXPORTDIR}/makeinfo.sh"
rm -f "${EXPORTDIR}/Make.defs"

# Verify the build info that we got from makeinfo.sh

if [ ! -d "${ARCHDIR}" ]; then
  echo "MK: Directory ${ARCHDIR} does not exist"
  exit 1
fi

# Copy the depends script

cp "${TOPDIR}/tools/mkdeps.c" "${EXPORTDIR}/tools/."
cp "${TOPDIR}/tools/incdir.c" "${EXPORTDIR}/tools/."

# Copy the default linker script

cp -f "${TOPDIR}/binfmt/libelf/gnu-elf.ld" "${EXPORTDIR}/scripts/."

# Copy the board config script

if [ -f "${BOARDDIR}/scripts/Config.mk" ]; then
  cp -f "${BOARDDIR}/scripts/Config.mk" "${EXPORTDIR}/scripts/."
fi

# Is there a linker script in this configuration?

if [ "X${USRONLY}" != "Xy" ]; then

  # LDPATH can contain multiple files.
  # The "Copy additional ld scripts" step might copy a file multiple times.

  for LDSCRIPT in ${LDPATH}; do

    # Apparently so.  Verify that the script exists

    if [ ! -f "${LDSCRIPT}" ]; then
      echo "MK: File ${LDSCRIPT} does not exist"
      exit 1
    fi

    # Copy the linker script

    cp -p "${LDSCRIPT}" "${EXPORTDIR}/scripts/." || \
      { echo "MK: cp ${LDSCRIPT} failed"; exit 1; }

    # Copy additional ld scripts

    LDDIR="$(dirname "${LDSCRIPT}")"
    for f in "${LDDIR}"/*.ld ; do
      [ -f "${f}" ] && cp -f "${f}" "${EXPORTDIR}/scripts/."
    done
  done
fi

# Save the compilation options

echo "ARCHCFLAGS       = ${ARCHCFLAGS}" >"${EXPORTDIR}/scripts/Make.defs"
echo "ARCHCPUFLAGS     = ${ARCHCPUFLAGS}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "ARCHCXXFLAGS     = ${ARCHCXXFLAGS}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "ARCHPICFLAGS     = ${ARCHPICFLAGS}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "ARCHWARNINGS     = ${ARCHWARNINGS}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "ARCHWARNINGSXX   = ${ARCHWARNINGSXX}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "ARCHOPTIMIZATION = ${ARCHOPTIMIZATION}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "CROSSDEV         = ${CROSSDEV}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "CC               = ${CC}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "CXX              = ${CXX}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "CPP              = ${CPP}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "LD               = ${LD}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "AR               = ${AR}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "NM               = ${NM}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "STRIP            = ${STRIP}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "OBJCOPY          = ${OBJCOPY}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "OBJDUMP          = ${OBJDUMP}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "NXFLATLDFLAGS1   = ${NXFLATLDFLAGS1}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "NXFLATLDFLAGS2   = ${NXFLATLDFLAGS2}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "OBJEXT           = ${OBJEXT}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "LIBEXT           = ${LIBEXT}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "EXEEXT           = ${EXEEXT}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "HOSTCC           = ${HOSTCC}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "HOSTINCLUDES     = ${HOSTINCLUDES}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "HOSTCFLAGS       = ${HOSTCFLAGS}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "HOSTLDFLAGS      = ${HOSTLDFLAGS}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "HOSTEXEEXT       = ${HOSTEXEEXT}" >>"${EXPORTDIR}/scripts/Make.defs"
echo "LDNAME           = ${LDNAME}" >>"${EXPORTDIR}/scripts/Make.defs"

# Additional compilation options when the kernel is built

if [ "X${USRONLY}" != "Xy" ]; then
  echo "EXTRA_LIBS       = ${EXTRA_LIBS}" >>"${EXPORTDIR}/scripts/Make.defs"
  echo "EXTRA_OBJS       = ${EXTRA_OBJS}" >>"${EXPORTDIR}/scripts/Make.defs"
  echo "HEAD_OBJ         = ${HEAD_OBJ}" >>"${EXPORTDIR}/scripts/Make.defs"
  echo "LDENDGROUP       = ${LDENDGROUP}" >>"${EXPORTDIR}/scripts/Make.defs"
  echo "LDFLAGS          = ${LDFLAGS}" >>"${EXPORTDIR}/scripts/Make.defs"
  echo "LDSTARTGROUP     = ${LDSTARTGROUP}" >>"${EXPORTDIR}/scripts/Make.defs"
fi

# Copy the system map file(s)

if [ -r ${TOPDIR}/System.map ]; then
  cp -a "${TOPDIR}/System.map" "${EXPORTDIR}/."
fi

if [ -r ${TOPDIR}/User.map ]; then
        cp -a "${TOPDIR}/User.map" "${EXPORTDIR}/."
fi

# Copy the NuttX include directory (retaining attributes and following symbolic links)

cp -LR -p "${TOPDIR}/include" "${EXPORTDIR}/." || \
  { echo "MK: 'cp ${TOPDIR}/include' failed"; exit 1; }

# Copy the startup object file(s)

${MAKE} -C ${ARCHDIR} export_startup TOPDIR=${TOPDIR} EXPORT_DIR="${EXPORTDIR}"

# Copy architecture-specific header files into the arch export sub-directory.
# This is tricky because each architecture does things in a little different
# way.
#
# First copy any header files in the architecture src/ sub-directory (some
# architectures keep all of the header files there, some a few, and others
# none

cp -f "${ARCHDIR}"/*.h "${EXPORTDIR}"/arch/. 2>/dev/null

# Then look a list of possible places where other architecture-specific
# header files might be found.  If those places exist (as directories or
# as symbolic links to directories, then copy the header files from
# those directories into the EXPORTDIR

if [ "X${USRONLY}" != "Xy" ]; then
  ARCH_HDRDIRS="arm armv7-m avr avr32 board common chip mips32"
  for hdir in $ARCH_HDRDIRS; do

    # Does the directory (or symbolic link) exist?

    if [ -d "${ARCHDIR}/${hdir}" -o -h "${ARCHDIR}/${hdir}" ]; then

      # Yes.. create a export sub-directory of the same name

      mkdir "${EXPORTDIR}/arch/${hdir}" || \
        { echo "MK: 'mkdir ${EXPORTDIR}/arch/${hdir}' failed"; exit 1; }

      # Then copy the header files (only) into the new directory

      cp -f "${ARCHDIR}"/${hdir}/*.h "${EXPORTDIR}"/arch/${hdir}/. 2>/dev/null

      # Most architectures have low directory called "hardware" that
      # holds the header files

      if [ -d "${ARCHDIR}/${hdir}/hardware" ]; then

        # Yes.. create a export sub-directory of the same name

        mkdir "${EXPORTDIR}/arch/${hdir}/hardware" || \
          { echo "MK: 'mkdir ${EXPORTDIR}/arch/${hdir}/hardware' failed"; exit 1; }

        # Then copy the header files (only) into the new directory

        cp -f "${ARCHDIR}"/${hdir}/hardware/*.h "${EXPORTDIR}"/arch/${hdir}/hardware/. 2>/dev/null
      fi
    fi
  done

  # Copy OS internal header files as well.  They are used by some architecture-
  # specific header files.

  mkdir "${EXPORTDIR}/arch/os" || \
    { echo "MK: 'mkdir ${EXPORTDIR}/arch/os' failed"; exit 1; }

  OSDIRS="clock environ errno group init irq mqueue paging pthread sched semaphore signal task timer wdog"

  for dir in ${OSDIRS}; do
    mkdir "${EXPORTDIR}/arch/os/${dir}" || \
      { echo "MK: 'mkdir ${EXPORTDIR}/arch/os/${dir}' failed"; exit 1; }
    cp -f "${TOPDIR}"/sched/${dir}/*.h "${EXPORTDIR}"/arch/os/${dir}/. 2>/dev/null
  done

  # Add the board library to the list of libraries

  if [ -f "${ARCHDIR}/board/libboard${LIBEXT}" ]; then
    LIBLIST="${LIBLIST} ${ARCHSUBDIR}/board/libboard${LIBEXT}"
  fi
fi

LDLIBS=`basename -a ${LIBLIST} | sed -e "s/lib/-l/g" -e "s/\.${LIBEXT:1}//g" | tr "\n" " "`

if [ "X${USRONLY}" != "Xy" ]; then
  echo "LDLIBS           = ${LDLIBS}" >>"${EXPORTDIR}/scripts/Make.defs"
fi

# Then process each library

for lib in ${LIBLIST}; do
  if [ ! -f "${TOPDIR}/${lib}" ]; then
    echo "MK: Library ${TOPDIR}/${lib} does not exist"
    exit 1
  fi

  cp ${TOPDIR}/${lib} ${EXPORTDIR}/libs
done

# Process extra librarys

for lib in ${EXTRA_LIBS}; do

  # Convert library name

  if [ ${lib:0:2} = "-l" ]; then
    lib=`echo "${lib}" | sed -e "s/-l/lib/" -e "s/$/${LIBEXT}/"`
  fi

  for path in ${EXTRA_LIBPATHS}; do

    # Skip the library path options

    if [ ${#path} == 2 ]; then continue; fi

    if [ ${path:0:2} = "-l" ] || [ ${path:0:2} = "-L" ]; then
      path=${path:2}
    fi

    # Export the extra librarys

    if [ -f "${path}/${lib}" ]; then
      cp -a ${path}/${lib} ${EXPORTDIR}/libs
      break
    fi

  done
done

# Copy the essential build script file(s)

cp -f "${TOPDIR}/tools/Config.mk" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/copydir.bat" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/copydir.sh" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/define.bat" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/define.sh" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/incdir.bat" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/incdir.sh" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/link.bat" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/link.sh" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/unlink.bat" "${EXPORTDIR}/tools/"
cp -f "${TOPDIR}/tools/unlink.sh" "${EXPORTDIR}/tools/"

# Now tar up the whole export directory

cd "${TOPDIR}" || \
  { echo "MK: 'cd ${TOPDIR}' failed"; exit 1; }

if [ -e "${APPDIR}/Makefile" ]; then
  "${MAKE}" -C "${APPDIR}" EXPORTDIR="$(cd "${EXPORTSUBDIR}" ; pwd )" TOPDIR="${TOPDIR}" export || \
      { echo "MK: call make export for APPDIR not supported"; }
fi

if [ "X${TGZ}" = "Xy" ]; then
  tar cvf "${EXPORTSUBDIR}.tar" "${EXPORTSUBDIR}" 1>/dev/null
  gzip -f "${EXPORTSUBDIR}.tar"
else
  zip -r "${EXPORTSUBDIR}.zip" "${EXPORTSUBDIR}" 1>/dev/null
fi

# Clean up after ourselves

rm -rf "${EXPORTSUBDIR}"
