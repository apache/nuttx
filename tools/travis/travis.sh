#! /bin/sh

# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

set -e
set -x

HOSTOS=$(uname -s)

case $HOSTOS in
	Linux)
		sudo apt-get update -q
		sudo apt-get install -y gperf
		if [ $TEST_M32 -ne 0 ]; then
			sudo apt-get install -y gcc-multilib
		fi
		MAKE=make
		CONFIGURE_HOSTOS=-l
		;;
	Darwin)
		MAKE=make
		CONFIGURE_HOSTOS=-m
		;;
	*)
		echo Unknown OS: $HOSTOS
		exit 1
		;;
esac

ORIG_DIR=$(pwd)

cd ..
git clone --depth=1 https://github.com/apache/incubator-nuttx-apps apps
git clone --depth=1 https://github.com/nuttx/tools

cd tools/kconfig-frontends
./configure --disable-kconfig --disable-nconf --disable-qconf \
	--disable-gconf --disable-mconf --disable-static --disable-shared \
	--disable-L10n --disable-utils
${MAKE}
sudo ${MAKE} install
cd ..

cd ${ORIG_DIR}
# git-based versioning doesn't always work because
# travis only clones the repo with shallow history.
mv .git .git.bak
cp tools/travis/version .version
./tools/configure.sh ${CONFIGURE_HOSTOS} sim:ostest
if [ $TEST_M32 -ne 0 ]; then
	echo CONFIG_SIM_M32=y >> .config
	${MAKE} oldconfig
fi
${MAKE}
./nuttx
