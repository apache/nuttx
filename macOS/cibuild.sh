#! /bin/sh

# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

set -x
set -e

ORIG_DIR=$(pwd)
MAKE=make
NTHREADS=$(sysctl -n machdep.cpu.thread_count)

# Install prerequisites

cd tools/kconfig-frontends
./configure --disable-kconfig --disable-nconf --disable-qconf \
    --disable-gconf --disable-mconf --disable-static \
    --disable-shared --disable-L10n --disable-utils
${MAKE}
sudo ${MAKE} install
cd ${ORIG_DIR}

brew tap PX4/homebrew-px4
brew install genromfs

brew tap discoteq/discoteq
brew install flock

brew install x86_64-elf-gcc

PATH=$PATH:${ORIG_DIR}/cache/xtensa-esp32-elf/bin

# Run builds

cd nuttx
./tools/testbuild.sh -j ${NTHREADS} $@
