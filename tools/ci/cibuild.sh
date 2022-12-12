#!/usr/bin/env bash

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

# Prerequisites for macOS
#  - Xcode (cc, etc)
#  - homebrew
#  - autoconf
#  - wget

set -e
set -o xtrace

WD=$(cd "$(dirname "$0")" && pwd)
WORKSPACE=$(cd "${WD}"/../../../ && pwd -P)
nuttx=${WORKSPACE}/nuttx
apps=${WORKSPACE}/apps
tools=${WORKSPACE}/tools
os=$(uname -s)
EXTRA_PATH=

function add_path {
  PATH=$1:${PATH}
  EXTRA_PATH=$1:${EXTRA_PATH}
}

function arm-clang-toolchain {
  add_path "${tools}"/clang-arm-none-eabi/bin

  if [ ! -f "${tools}/clang-arm-none-eabi/bin/clang" ]; then
    local flavor
    case ${os} in
      Linux)
        flavor=linux
        ;;
    esac
    cd "${tools}"
    curl -O -L -s https://github.com/ARM-software/LLVM-embedded-toolchain-for-Arm/releases/download/release-14.0.0/LLVMEmbeddedToolchainForArm-14.0.0-${flavor}.tar.gz
    tar zxf LLVMEmbeddedToolchainForArm-14.0.0-${flavor}.tar.gz
    mv LLVMEmbeddedToolchainForArm-14.0.0 clang-arm-none-eabi
    cp /usr/bin/clang-extdef-mapping-10 clang-arm-none-eabi/bin/clang-extdef-mapping
    rm LLVMEmbeddedToolchainForArm-14.0.0-${flavor}.tar.gz
  fi

  clang --version
}

function arm-gcc-toolchain {
  add_path "${tools}"/gcc-arm-none-eabi/bin

  if [ ! -f "${tools}/gcc-arm-none-eabi/bin/arm-none-eabi-gcc" ]; then
    local flavor
    case ${os} in
      Darwin)
        flavor=-darwin
        ;;
      Linux)
        flavor=
        ;;
    esac
    cd "${tools}"
    wget --quiet https://developer.arm.com/-/media/Files/downloads/gnu/11.3.rel1/binrel/arm-gnu-toolchain-11.3.rel1${flavor}-x86_64-arm-none-eabi.tar.xz
    xz -d arm-gnu-toolchain-11.3.rel1${flavor}-x86_64-arm-none-eabi.tar.xz
    tar xf arm-gnu-toolchain-11.3.rel1${flavor}-x86_64-arm-none-eabi.tar
    mv arm-gnu-toolchain-11.3.rel1${flavor}-x86_64-arm-none-eabi gcc-arm-none-eabi
    patch -p0 < ${nuttx}/tools/ci/patch/arm-none-eabi-workaround-for-newlib-version-break.patch
    rm arm-gnu-toolchain-11.3.rel1${flavor}-x86_64-arm-none-eabi.tar
  fi

  arm-none-eabi-gcc --version
}

function arm64-gcc-toolchain {
  add_path "${tools}"/gcc-aarch64-none-elf/bin

  if [ ! -f "${tools}/gcc-aarch64-none-elf/bin/aarch64-none-elf-gcc" ]; then
    local flavor
    case ${os} in
      Darwin)
        flavor=darwin-x86_64
        ;;
      Linux)
        flavor=x86_64
        ;;
    esac
    cd "${tools}"
    wget --quiet https://developer.arm.com/-/media/Files/downloads/gnu/11.2-2022.02/binrel/gcc-arm-11.2-2022.02-${flavor}-aarch64-none-elf.tar.xz
    xz -d gcc-arm-11.2-2022.02-${flavor}-aarch64-none-elf.tar.xz
    tar xf gcc-arm-11.2-2022.02-${flavor}-aarch64-none-elf.tar
    mv gcc-arm-11.2-2022.02-${flavor}-aarch64-none-elf gcc-aarch64-none-elf
    rm gcc-arm-11.2-2022.02-${flavor}-aarch64-none-elf.tar
  fi

  aarch64-none-elf-gcc --version
}

function avr-gcc-toolchain {
  if ! type avr-gcc &> /dev/null; then
    case ${os} in
      Darwin)
        brew tap osx-cross/avr
        brew install avr-gcc
        ;;
      Linux)
        apt-get install -y avr-libc gcc-avr
        ;;
    esac
  fi

  avr-gcc --version
}

function binutils {
  mkdir -p "${tools}"/bintools/bin
  add_path "${tools}"/bintools/bin

  if ! type objcopy &> /dev/null; then
    case ${os} in
      Darwin)
        brew install binutils
        # It is possible we cached prebuilt but did brew install so recreate
        # symlink if it exists
        rm -f "${tools}"/bintools/bin/objcopy
        ln -s /usr/local/opt/binutils/bin/objcopy "${tools}"/bintools/bin/objcopy
        ;;
    esac
  fi

  objcopy --version
}

function bloaty {
  add_path "${tools}"/bloaty/bin

  if [ ! -f "${tools}/bloaty/bin/bloaty" ]; then
    git clone --branch main https://github.com/google/bloaty "${tools}"/bloaty-src
    cd "${tools}"/bloaty-src
    # Due to issues with latest MacOS versions use pinned commit.
    # https://github.com/google/bloaty/pull/326
    git checkout 52948c107c8f81045e7f9223ec02706b19cfa882
    mkdir -p "${tools}"/bloaty
    cmake -D BLOATY_PREFER_SYSTEM_CAPSTONE=NO -DCMAKE_SYSTEM_PREFIX_PATH="${tools}"/bloaty
    make install -j 6
    cd "${tools}"
    rm -rf bloaty-src
  fi

  command bloaty --version
}

function c-cache {
  add_path "${tools}"/ccache/bin

  if ! type ccache &> /dev/null; then
    case ${os} in
      Darwin)
        brew install ccache
        ;;
      Linux)
        cd "${tools}";
        wget https://github.com/ccache/ccache/releases/download/v3.7.7/ccache-3.7.7.tar.gz
        tar zxf ccache-3.7.7.tar.gz
        cd ccache-3.7.7; ./configure --prefix="${tools}"/ccache; make; make install
        cd "${tools}"; rm -rf ccache-3.7.7; rm ccache-3.7.7.tar.gz
        ;;
    esac
  fi

  ccache --version
}

function clang-tidy {
  if ! type clang-tidy &> /dev/null; then
    case ${os} in
      Linux)
        apt-get install -y clang clang-tidy
        ;;
    esac
  fi

  command clang-tidy --version
}

function elf-toolchain {
  if ! type x86_64-elf-gcc &> /dev/null; then
    case ${os} in
      Darwin)
        brew install x86_64-elf-gcc
        ;;
    esac
  fi

  x86_64-elf-gcc --version
}

function gen-romfs {
  add_path "${tools}"/genromfs/usr/bin

  if [ ! -f "${tools}/genromfs/usr/bin/genromfs" ]; then
    git clone https://bitbucket.org/nuttx/tools.git "${tools}"/nuttx-tools
    cd "${tools}"/nuttx-tools
    tar zxf genromfs-0.5.2.tar.gz -C "${tools}"
    cd "${tools}"/genromfs-0.5.2
    make install PREFIX="${tools}"/genromfs
    cd "${tools}"
    rm -rf genromfs-0.5.2
  fi
}

function gperf {
  add_path "${tools}"/gperf/bin

  if [ ! -f "${tools}/gperf/bin/gperf" ]; then
    cd "${tools}"
    wget --quiet http://ftp.gnu.org/pub/gnu/gperf/gperf-3.1.tar.gz
    tar zxf gperf-3.1.tar.gz
    cd "${tools}"/gperf-3.1
    ./configure --prefix="${tools}"/gperf; make; make install
    cd "${tools}"
    rm -rf gperf-3.1; rm gperf-3.1.tar.gz
  fi

  command gperf --version
}

function kconfig-frontends {
  add_path "${tools}"/kconfig-frontends/bin

  if [ ! -f "${tools}/kconfig-frontends/bin/kconfig-conf" ]; then
    cd "${tools}"/nuttx-tools/kconfig-frontends
    ./configure --prefix="${tools}"/kconfig-frontends \
      --disable-kconfig --disable-nconf --disable-qconf \
      --disable-gconf --disable-mconf --disable-static \
      --disable-shared --disable-L10n
    # Avoid "aclocal/automake missing" errors
    touch aclocal.m4 Makefile.in
    make install
    cd "${tools}"
    rm -rf nuttx-tools
  fi
}

function mips-gcc-toolchain {
  if [ ! -d "${tools}/pinguino-compilers" ]; then
    cd "${tools}"
    git clone https://github.com/PinguinoIDE/pinguino-compilers
  fi

  case ${os} in
    Darwin)
      add_path "${tools}"/pinguino-compilers/macosx/p32/bin
      mips-elf-gcc --version
      ;;
    Linux)
      add_path "${tools}"/pinguino-compilers/linux64/p32/bin
      p32-gcc --version
      ;;
  esac
}

function python-tools {
  # Python User Env
  export PIP_USER=yes
  export PYTHONUSERBASE=${tools}/pylocal
  add_path "${PYTHONUSERBASE}"/bin
  pip3 install \
    CodeChecker \
    cxxfilt \
    esptool==3.3.1 \
    pexpect==4.8.0 \
    pyelftools \
    pyserial==3.5 \
    pytest==6.2.5 \
    pytest-json==0.4.0 \
    pytest-ordering==0.6 \
    pytest-repeat==0.9.1 

  # MCUboot's tool for image signing and key management
  if ! command -v imgtool &> /dev/null; then
    pip3 install imgtool
  fi
}

function riscv-gcc-toolchain {
  add_path "${tools}"/riscv64-unknown-elf-gcc/bin

  if [ ! -f "${tools}/riscv64-unknown-elf-gcc/bin/riscv64-unknown-elf-gcc" ]; then
    local flavor
    case ${os} in
      Darwin)
        flavor=x86_64-apple-darwin
        ;;
      Linux)
        flavor=x86_64-linux-ubuntu14
        ;;
    esac
    cd "${tools}"
    wget --quiet https://static.dev.sifive.com/dev-tools/freedom-tools/v2020.12/riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-${flavor}.tar.gz
    tar zxf riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-${flavor}.tar.gz
    mv riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-${flavor} riscv64-unknown-elf-gcc
    rm riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-${flavor}.tar.gz
  fi

  riscv64-unknown-elf-gcc --version
}

function rust {
  mkdir -p "${tools}"/rust/bin
  add_path "${tools}"/rust/bin

  if ! type rustc &> /dev/null; then
    case ${os} in
      Darwin)
        brew install rust
        ;;
      Linux)
        # Currently Debian installed rustc doesn't support 2021 edition.
        export CARGO_HOME=${tools}/rust
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
        ;;
    esac
  fi

  rustc --version
}

function rx-gcc-toolchain {
  add_path "${tools}"/renesas-toolchain/rx-elf-gcc/bin

  if [ ! -f "${tools}/renesas-toolchain/rx-elf-gcc/bin/rx-elf-gcc" ]; then
    case ${os} in
      Linux)
        # Download toolchain source code
        # RX toolchain is built from source code. Once prebuilt RX toolchain is made available, the below code snippet can be removed.
        mkdir -p "${tools}"/renesas-tools/rx/source; cd "${tools}"/renesas-tools/rx/source
        wget --quiet https://gcc-renesas.com/downloads/d.php?f=rx/binutils/4.8.4.201803-gnurx/rx_binutils2.24_2018Q3.tar.gz \
          -O rx_binutils2.24_2018Q3.tar.gz
        tar zxf rx_binutils2.24_2018Q3.tar.gz
        wget --quiet https://gcc-renesas.com/downloads/d.php?f=rx/gcc/4.8.4.201803-gnurx/rx_gcc_4.8.4_2018Q3.tar.gz \
          -O rx_gcc_4.8.4_2018Q3.tar.gz
        tar zxf rx_gcc_4.8.4_2018Q3.tar.gz
        wget --quiet https://gcc-renesas.com/downloads/d.php?f=rx/newlib/4.8.4.201803-gnurx/rx_newlib2.2.0_2018Q3.tar.gz \
          -O rx_newlib2.2.0_2018Q3.tar.gz
        tar zxf rx_newlib2.2.0_2018Q3.tar.gz

        # Install binutils
        cd "${tools}"/renesas-tools/rx/source/binutils; chmod +x ./configure ./mkinstalldirs
        mkdir -p "${tools}"/renesas-tools/rx/build/binutils; cd "${tools}"/renesas-tools/rx/build/binutils
        "${tools}"/renesas-tools/rx/source/binutils/configure --target=rx-elf --prefix="${tools}"/renesas-toolchain/rx-elf-gcc \
          --disable-werror
        make; make install

        # Install gcc
        cd "${tools}"/renesas-tools/rx/source/gcc
        chmod +x ./contrib/download_prerequisites ./configure ./move-if-change ./libgcc/mkheader.sh
        ./contrib/download_prerequisites
        sed -i '1s/^/@documentencoding ISO-8859-1\n/' ./gcc/doc/gcc.texi
        sed -i 's/@tex/\n&/g' ./gcc/doc/gcc.texi && sed -i 's/@end tex/\n&/g' ./gcc/doc/gcc.texi
        mkdir -p "${tools}"/renesas-tools/rx/build/gcc; cd "${tools}"/renesas-tools/rx/build/gcc
        "${tools}"/renesas-tools/rx/source/gcc/configure --target=rx-elf --prefix="${tools}"/renesas-toolchain/rx-elf-gcc \
        --disable-shared --disable-multilib --disable-libssp --disable-libstdcxx-pch --disable-werror --enable-lto \
        --enable-gold --with-pkgversion=GCC_Build_1.02 --with-newlib --enable-languages=c
        make; make install

        # Install newlib
        cd "${tools}"/renesas-tools/rx/source/newlib; chmod +x ./configure
        mkdir -p "${tools}"/renesas-tools/rx/build/newlib; cd "${tools}"/renesas-tools/rx/build/newlib
        "${tools}"/renesas-tools/rx/source/newlib/configure --target=rx-elf --prefix="${tools}"/renesas-toolchain/rx-elf-gcc
        make; make install
        rm -rf "${tools}"/renesas-tools/
        ;;
    esac
  fi

  rx-elf-gcc --version
}

function sparc-gcc-toolchain {
  add_path "${tools}"/sparc-gaisler-elf-gcc/bin

  if [ ! -f "${tools}/sparc-gaisler-elf-gcc/bin/sparc-gaisler-elf-gcc" ]; then
    case ${os} in
      Linux)
        cd "${tools}"
        wget --quiet https://www.gaisler.com/anonftp/bcc2/bin/bcc-2.1.0-gcc-linux64.tar.xz
        xz -d bcc-2.1.0-gcc-linux64.tar.xz
        tar xf bcc-2.1.0-gcc-linux64.tar
        mv bcc-2.1.0-gcc sparc-gaisler-elf-gcc
        rm bcc-2.1.0-gcc-linux64.tar
        ;;
    esac
  fi

  sparc-gaisler-elf-gcc --version
}

function xtensa-esp32-gcc-toolchain {
  add_path "${tools}"/xtensa-esp32-elf/bin

  if [ ! -f "${tools}/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc" ]; then
    cd "${tools}"
    case ${os} in
      Darwin)
        wget --quiet https://dl.espressif.com/dl/xtensa-esp32-elf-gcc8_4_0-esp-2021r1-macos.tar.gz
        tar xzf xtensa-esp32-elf-gcc8_4_0-esp-2021r1-macos.tar.gz
        rm xtensa-esp32-elf-gcc8_4_0-esp-2021r1-macos.tar.gz
        ;;
      Linux)
        wget --quiet https://dl.espressif.com/dl/xtensa-esp32-elf-gcc8_4_0-esp32-2021r1-linux-amd64.tar.xz
        xz -d xtensa-esp32-elf-gcc8_4_0-esp32-2021r1-linux-amd64.tar.xz
        tar xf xtensa-esp32-elf-gcc8_4_0-esp32-2021r1-linux-amd64.tar
        rm xtensa-esp32-elf-gcc8_4_0-esp32-2021r1-linux-amd64.tar
        ;;
    esac
  fi

  xtensa-esp32-elf-gcc --version
}

function u-boot-tools {
  if ! type mkimage &> /dev/null; then
    case ${os} in
      Darwin)
        brew install u-boot-tools
        ;;
      Linux)
        apt-get install -y u-boot-tools
        ;;
    esac
  fi
}

function usage {
  echo ""
  echo "USAGE: $0 [-i] [-s] [-c] [-*] <testlist>"
  echo "       $0 -h"
  echo ""
  echo "Where:"
  echo "  -i install tools"
  echo "  -s setup repos"
  echo "  -c enable ccache"
  echo "  -* support all options in testbuild.sh"
  echo "  -h will show this help test and terminate"
  echo "  <testlist> select testlist file"
  echo ""
  exit 1
}

function enable_ccache {
  export CCACHE_DIR="${tools}"/ccache
}

function setup_links {
  mkdir -p "${tools}"/ccache/bin/
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/aarch64-none-elf-gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/aarch64-none-elf-g++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/arm-none-eabi-gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/arm-none-eabi-g++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/avr-gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/avr-g++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/cc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/c++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/clang
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/clang++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/g++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/p32-gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/riscv64-unknown-elf-gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/riscv64-unknown-elf-g++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/sparc-gaisler-elf-gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/sparc-gaisler-elf-g++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/x86_64-elf-gcc
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/x86_64-elf-g++
  ln -sf "$(which ccache)" "${tools}"/ccache/bin/xtensa-esp32-elf-gcc
}

function setup_repos {
  pushd .
  if [ -d "${nuttx}" ]; then
    cd "${nuttx}"; git pull
  else
    git clone https://github.com/apache/nuttx.git "${nuttx}"
    cd "${nuttx}"
  fi
  git log -1

  if [ -d "${apps}" ]; then
    cd "${apps}"; git pull
  else
    git clone https://github.com/apache/nuttx-apps.git "${apps}"
    cd "${apps}"
  fi
  git log -1
  popd
}

function install_tools {
  mkdir -p "${tools}"

case ${os} in
  Darwin)
    install="arm-gcc-toolchain arm64-gcc-toolchain avr-gcc-toolchain binutils bloaty elf-toolchain gen-romfs gperf kconfig-frontends mips-gcc-toolchain python-tools riscv-gcc-toolchain rust xtensa-esp32-gcc-toolchain u-boot-tools c-cache"
    mkdir -p "${tools}"/homebrew
    export HOMEBREW_CACHE=${tools}/homebrew
    # https://github.com/actions/virtual-environments/issues/2322#issuecomment-749211076
    rm -rf /usr/local/bin/2to3
    # https://github.com/osx-cross/homebrew-avr/issues/205#issuecomment-760637996
    brew update --quiet
    ;;
  Linux)
    install="arm-clang-toolchain arm-gcc-toolchain arm64-gcc-toolchain avr-gcc-toolchain binutils bloaty clang-tidy gen-romfs gperf kconfig-frontends mips-gcc-toolchain python-tools riscv-gcc-toolchain rust rx-gcc-toolchain sparc-gcc-toolchain xtensa-esp32-gcc-toolchain u-boot-tools c-cache"
    ;;
esac

  pushd .
  for func in ${install}; do
    ${func}
  done
  popd

  setup_links
  echo PATH="${EXTRA_PATH}"/"${PATH}" > "${tools}"/env.sh
}

function run_builds {
  local ncpus

  case ${os} in
    Darwin)
      ncpus=$(sysctl -n hw.ncpu)
      ;;
    Linux)
      ncpus=$(grep -c ^processor /proc/cpuinfo)
      ;;
  esac

  options+="-j ${ncpus}"

  for build in "${builds[@]}"; do
    "${nuttx}"/tools/testbuild.sh ${options} -e "-Wno-cpp -Werror" "${build}"
  done

  if [ -d "${CCACHE_DIR}" ]; then
    ccache -s
  fi
}

if [ -z "$1" ]; then
   usage
fi

while [ -n "$1" ]; do
  case "$1" in
  -h )
    usage
    ;;
  -i )
    install_tools
    ;;
  -c )
    enable_ccache
    ;;
  -s )
    setup_repos
    ;;
  -* )
    options+="$1 "
    ;;
  * )
    builds=( "$@" )
    break
    ;;
  esac
  shift
done

run_builds
