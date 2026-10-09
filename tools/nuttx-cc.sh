#!/bin/sh
# tools/nuttx-cc.sh
#
# SPDX-License-Identifier: Apache-2.0
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

# Compiler wrapper for programs built outside the NuttX tree.
#
# tools/mkexport.sh installs this script as bin/nuttx-cc and bin/nuttx-c++
# in the export package, next to scripts/nuttx-cc.conf, which holds the
# flags of the exported configuration.  Use it like a normal compiler:
#
#   nuttx-cc -O2 -c hello.c
#   nuttx-cc -o hello hello.o
#   ./configure --host=aarch64-none-elf CC=/path/to/export/bin/nuttx-cc
#
# It compiles against the exported headers only (-nostdinc), so a header
# that NuttX does not have is reported missing instead of being taken from
# the toolchain's own C library.  When it links, it adds the startup
# object, the exported linker script and the exported libraries.
#
# NUTTX_CC and NUTTX_CXX override the compiler named in the package.

# Find the export package from the location of this script

case $0 in
  */*) bindir=${0%/*} ;;
  *)   bindir=$(dirname "$(command -v "$0")") ;;
esac

export_dir=$(cd "${bindir}/.." && pwd) || exit 1
conf=${export_dir}/scripts/nuttx-cc.conf

if [ ! -r "${conf}" ]; then
  echo "nuttx-cc: ${conf} not found" >&2
  exit 1
fi

. "${conf}"

# C or C++, from the name we were called by

case ${0##*/} in
  *++*)
    cc=${NUTTX_CXX:-${NUTTXCC_CXX}}
    cflags="${NUTTXCC_CXXFLAGS}"
    for dir in ${NUTTXCC_CXXINCDIRS}; do
      cflags="${cflags} -isystem ${export_dir}/include/${dir}"
    done
    ;;
  *)
    cc=${NUTTX_CC:-${NUTTXCC_CC}}
    cflags="${NUTTXCC_CFLAGS}"
    ;;
esac

# Use only the exported headers plus the compiler's own (stddef.h,
# arm_neon.h, ...).

builtin=$(${cc} ${NUTTXCC_CPUFLAGS} -print-file-name=include)
cflags="${cflags} -isystem ${export_dir}/include"
if [ -d "${builtin}" ]; then
  cflags="-nostdinc ${cflags} -isystem ${builtin}"
fi

# Do we link?  Not with -c, -S, -E or -M/-MM, and not when there is no
# input file at all (--version, -print-file-name=..., -dumpmachine).

link=n
skip=n
out=a.out
for arg in "$@"; do
  if [ ${skip} = o ]; then
    out=${arg}
    skip=n
    continue
  elif [ ${skip} = y ]; then
    skip=n
    continue
  fi

  case ${arg} in
    -c | -S | -E | -M | -MM)
      link=never
      ;;
    -o)
      skip=o
      ;;
    -o*)
      out=${arg#-o}
      ;;
    -x | -MF | -MT | -MQ | -I | -L | -D | -U | -T | -l | -u | -e | \
    -include | -imacros | -isystem | -idirafter | -iquote | -iprefix | \
    -Xlinker | -Xassembler | -Xpreprocessor | --param)
      skip=y
      ;;
    -*)
      ;;
    *)
      [ ${link} = n ] && link=y
      ;;
  esac
done

if [ ${link} != y ]; then
  exec ${cc} ${NUTTXCC_CPUFLAGS} ${cflags} "$@"
fi

# Link: crt0, the user's objects and libraries, then the NuttX libraries.
# The ld options in the package are passed one by one through -Xlinker.

ldflags=
for opt in ${NUTTXCC_LDFLAGS}; do
  ldflags="${ldflags} -Xlinker ${opt}"
done

if [ -n "${NUTTXCC_LDSCRIPT}" ]; then
  ldflags="${ldflags} -T ${export_dir}/scripts/${NUTTXCC_LDSCRIPT}"
fi

libs=
for lib in ${NUTTXCC_LIBS}; do
  libs="${libs} -l${lib}"
done

${cc} ${NUTTXCC_CPUFLAGS} ${cflags} -nostdlib -nostartfiles ${ldflags} \
  "${export_dir}/startup/crt0.o" "$@" -L "${export_dir}/libs" \
  -Wl,--start-group ${libs} -Wl,--end-group || exit

# A relocatable (-r) link never fails on an undefined symbol.  In a kernel
# build nothing is left to resolve it at load time, so fail here, as a
# normal link would.  Configure scripts depend on this to find out which
# functions exist.

if [ "${NUTTXCC_NOUNDEF}" = y ]; then
  syms=$("$(${cc} -print-prog-name=nm)" -u "${out}") || exit

  # Only a strong reference (U) must be resolved.  A weak one (w, v) may
  # stay undefined and then has the value 0, as in a normal link.

  undef=
  while read -r type sym; do
    if [ "${type}" = U ]; then
      undef="${undef}  ${sym}
"
    fi
  done <<EOF
${syms}
EOF

  if [ -n "${undef}" ]; then
    echo "nuttx-cc: ${out}: undefined references:" >&2
    printf '%s' "${undef}" >&2
    rm -f "${out}"
    exit 1
  fi
fi
