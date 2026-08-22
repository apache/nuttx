#!/bin/bash
############################################################################
# tools/fdpic/build-binutils.sh
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
############################################################################

# Build arm-uclinuxfdpiceabi binutils -- the only part of the module
# toolchain that has to be built from source.
#
# The compiling is done by the stock Arm bare-metal toolchain, which emits
# correct FDPIC objects for both C and C++.  What it cannot do is *link*
# them: arm-none-eabi-ld is configured with the `armelf` emulation alone, so
# it produces an object marked "UNIX - System V" that the loader refuses.
# The FDPIC linker carries armelf_linux_fdpiceabi, and that is the whole of
# the gap.
#
# So this builds binutils and nothing else: about a minute, roughly 23 MB.
# An FDPIC GCC is not needed for any of this.
#
# Usage: build-binutils.sh <workdir> [install-prefix]
#
# Then add <install-prefix>/bin to PATH.

set -e

WORK="${1:?usage: build-binutils.sh <workdir> [prefix]}"
PREFIX="${2:-$WORK/toolchain}"
TARGET=arm-uclinuxfdpiceabi
BINUTILS=binutils-2.43
J="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"

mkdir -p "$WORK/src" "$WORK/build"

cd "$WORK/src"
[ -d "$BINUTILS" ] || {
    curl -fL -O "https://ftp.gnu.org/gnu/binutils/$BINUTILS.tar.xz"
    tar xf "$BINUTILS.tar.xz"
}

rm -rf "$WORK/build/binutils"
mkdir -p "$WORK/build/binutils"
cd "$WORK/build/binutils"

# --with-system-zlib because the bundled copy does not compile against the
# macOS SDK headers.  Harmless elsewhere.

"$WORK/src/$BINUTILS/configure" \
    --target="$TARGET" \
    --prefix="$PREFIX" \
    --disable-nls \
    --disable-werror \
    --with-system-zlib

make -j"$J"
make install

echo
echo "Installed to $PREFIX/bin"
echo
"$PREFIX/bin/$TARGET-ld" -V | head -8
echo
echo "armelf_linux_fdpiceabi in the list above is the one that matters."
echo "Add to PATH:  export PATH=$PREFIX/bin:\$PATH"
