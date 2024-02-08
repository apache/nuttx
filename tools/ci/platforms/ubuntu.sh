#!/usr/bin/env sh
############################################################################
# tools/ci/platforms/ubuntu.sh
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

# Ubuntu

set -e
set -o xtrace

add_path() {
  PATH=$1:${PATH}
}

arm_clang_toolchain() {
  add_path "${NUTTXTOOLS}"/clang-arm-none-eabi/bin

  if [ ! -f "${NUTTXTOOLS}/clang-arm-none-eabi/bin/clang" ]; then
    local basefile
    basefile=LLVMEmbeddedToolchainForArm-17.0.1-Linux-x86_64
    cd "${NUTTXTOOLS}"
    # Download the latest ARM clang toolchain prebuilt by ARM
    curl -O -L -s https://github.com/ARM-software/LLVM-embedded-toolchain-for-Arm/releases/download/release-17.0.1/${basefile}.tar.xz
    xz -d ${basefile}.tar.xz
    tar xf ${basefile}.tar
    mv ${basefile} clang-arm-none-eabi
    rm ${basefile}.tar
  fi

  command clang --version
}

arm_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/gcc-arm-none-eabi/bin

  if [ ! -f "${NUTTXTOOLS}/gcc-arm-none-eabi/bin/arm-none-eabi-gcc" ]; then
    local basefile
    basefile=arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi
    cd "${NUTTXTOOLS}"
    # Download the latest ARM GCC toolchain prebuilt by ARM
    wget --quiet https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/${basefile}.tar.xz
    xz -d ${basefile}.tar.xz
    tar xf ${basefile}.tar
    mv ${basefile} gcc-arm-none-eabi
    rm ${basefile}.tar
  fi

  command arm-none-eabi-gcc --version
}

arm64_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/gcc-aarch64-none-elf/bin

  if [ ! -f "${NUTTXTOOLS}/gcc-aarch64-none-elf/bin/aarch64-none-elf-gcc" ]; then
    local basefile
    basefile=arm-gnu-toolchain-13.2.Rel1-x86_64-aarch64-none-elf
    cd "${NUTTXTOOLS}"
    # Download the latest ARM64 GCC toolchain prebuilt by ARM
    wget --quiet https://developer.arm.com/-/media/Files/downloads/gnu/13.2.Rel1/binrel/${basefile}.tar.xz
    xz -d ${basefile}.tar.xz
    tar xf ${basefile}.tar
    mv ${basefile} gcc-aarch64-none-elf
    rm ${basefile}.tar
  fi

  command aarch64-none-elf-gcc --version
}

avr_gcc_toolchain() {
  if ! type avr-gcc > /dev/null 2>&1; then
    sudo apt-get install -y binutils-avr gcc-avr avr-libc 
  fi

  command avr-gcc --version
}

binutils() {
  if ! type objcopy > /dev/null 2>&1; then
    sudo apt-get install -y binutils-dev
  fi

  command objcopy --version
}

bloaty() {
  add_path "${NUTTXTOOLS}"/bloaty/bin

  if [ ! -f "${NUTTXTOOLS}/bloaty/bin/bloaty" ]; then
    git clone --depth 1 --branch v1.1 https://github.com/google/bloaty "${NUTTXTOOLS}"/bloaty-src
    mkdir -p "${NUTTXTOOLS}"/bloaty
    cd "${NUTTXTOOLS}"/bloaty-src
    cmake -B build -DCMAKE_INSTALL_PREFIX="${NUTTXTOOLS}"/bloaty
    cmake --build build
    cmake --build build --target install
    cd "${NUTTXTOOLS}"
    rm -rf bloaty-src
    ls -a "${NUTTXTOOLS}"/bloaty
  fi

  command bloaty --version
}

c_cache() {
  if ! type ccache > /dev/null 2>&1; then
    sudo apt-get install -y ccache
  fi
  setup_links
  command ccache --version
}

clang_tidy() {
  if ! type clang-tidy > /dev/null 2>&1; then
    sudo apt-get install -y clang clang-tidy
  fi

  command clang-tidy --version
}

util_linux() {
  if ! type flock > /dev/null 2>&1; then
    sudo apt-get install -y util-linux
  fi

  command flock --version
}

gen_romfs() {
  if ! type genromfs > /dev/null 2>&1; then
    sudo apt-get install -y genromfs
  fi
}

gperf() {
 if ! type gperf > /dev/null 2>&1; then
    sudo apt-get install -y gperf
  fi

}

kconfig_frontends() {
  add_path "${NUTTXTOOLS}"/kconfig-frontends/bin

  if [ ! -f "${NUTTXTOOLS}/kconfig-frontends/bin/kconfig-conf" ]; then
    git clone https://bitbucket.org/nuttx/tools.git "${NUTTXTOOLS}"/nuttx-tools
    cd "${NUTTXTOOLS}"/nuttx-tools/kconfig-frontends
    ./configure --prefix="${NUTTXTOOLS}"/kconfig-frontends \
      --disable-kconfig --disable-nconf --disable-qconf \
      --disable-gconf --disable-mconf --disable-static \
      --disable-shared --disable-L10n
    # Avoid "aclocal/automake missing" errors
    touch aclocal.m4 Makefile.in
    make install
    cd "${NUTTXTOOLS}"
    rm -rf nuttx-tools
  fi
}

mips_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/pinguino-compilers/p32/bin

  if [ ! -d "${NUTTXTOOLS}/pinguino-compilers/p32/bin/p32-gcc" ]; then
    local basefile
    basefile=pinguino-linux64-p32
    mkdir -p "${NUTTXTOOLS}"/pinguino-compilers
    cd "${NUTTXTOOLS}"
    # Download the latest pinguino toolchain prebuilt by 32bit
    curl -O -L -s  https://github.com/PinguinoIDE/pinguino-compilers/releases/download/v20.10/${basefile}.zip
    unzip -qo ${basefile}.zip
    mv p32 "${NUTTXTOOLS}"/pinguino-compilers/p32
    rm ${basefile}.zip
  fi

  command p32-gcc --version
}

python_tools() {

  pip3 install \
    cmake-format \
    CodeChecker \
    cvt2utf \
    cxxfilt \
    esptool \
    imgtool \
    kconfiglib \
    pexpect==4.8.0 \
    pyelftools \
    pyserial==3.5 \
    pytest==6.2.5 \
    pytest-json==0.4.0 \
    pytest-ordering==0.6 \
    pytest-repeat==0.9.1
}

riscv_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/riscv-none-elf-gcc/bin

  if [ ! -f "${NUTTXTOOLS}/riscv-none-elf-gcc/bin/riscv-none-elf-gcc" ]; then
    local basefile
    basefile=xpack-riscv-none-elf-gcc-13.2.0-2-linux-x64
    cd "${NUTTXTOOLS}"
    # Download the latest RISCV GCC toolchain prebuilt by xPack
    wget --quiet https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/${basefile}.tar.gz
    tar zxf ${basefile}.tar.gz
    mv xpack-riscv-none-elf-gcc-13.2.0-2 riscv-none-elf-gcc
    rm ${basefile}.tar.gz
  fi
  command riscv-none-elf-gcc --version
}

rust() {
  if ! type rustc > /dev/null 2>&1; then
    sudo apt-get install rustc
    # Install targets supported from NuttX
    rustup target add thumbv6m-none-eabi
    rustup target add thumbv7m-none-eabi
  fi

  command rustc --version
}

rx_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/renesas-toolchain/rx-elf-gcc/bin

  if [ ! -f "${NUTTXTOOLS}/renesas-toolchain/rx-elf-gcc/bin/rx-elf-gcc" ]; then
    # Download toolchain source code
    # RX toolchain is built from source code. Once prebuilt RX toolchain is made available, the below code snippet can be removed.
    local basefilebinutils
    local basefilegcc
    local basefilenewlib
    basefilebinutils=binutils-2.36.1
    basefilegcc=gcc-8.3.0
    basefilenewlib=newlib-4.1.0

    mkdir -p "${NUTTXTOOLS}"/renesas-tools/source
    curl -L -s "https://llvm-gcc-renesas.com/downloads/d.php?f=rx/binutils/8.3.0.202305-gnurx/binutils-2.36.1.tar.gz" -o ${basefilebinutils}.tar.gz
    tar zxf ${basefilebinutils}.tar.gz
    mv ${basefilebinutils} "${NUTTXTOOLS}"/renesas-tools/source/binutils
    rm ${basefilebinutils}.tar.gz

    curl -L -s "https://llvm-gcc-renesas.com/downloads/d.php?f=rx/gcc/8.3.0.202305-gnurx/gcc-8.3.0.tar.gz" -o ${basefilegcc}.tar.gz
    tar zxf ${basefilegcc}.tar.gz
    mv ${basefilegcc} "${NUTTXTOOLS}"/renesas-tools/source/gcc
    rm ${basefilegcc}.tar.gz

    curl -L -s "https://llvm-gcc-renesas.com/downloads/d.php?f=rx/newlib/8.3.0.202305-gnurx/newlib-4.1.0.tar.gz" -o ${basefilenewlib}.tar.gz
    tar zxf ${basefilenewlib}.tar.gz
    mv ${basefilenewlib} "${NUTTXTOOLS}"/renesas-tools/source/newlib
    rm ${basefilenewlib}.tar.gz

    # Install binutils
    cd "${NUTTXTOOLS}"/renesas-tools/source/binutils; chmod +x ./configure ./mkinstalldirs
    mkdir -p "${NUTTXTOOLS}"/renesas-tools/build/binutils; cd "${NUTTXTOOLS}"/renesas-tools/build/binutils
    "${NUTTXTOOLS}"/renesas-tools/source/binutils/configure --target=rx-elf --prefix="${NUTTXTOOLS}"/renesas-toolchain/rx-elf-gcc \
      --disable-werror
    make; make install

    # Install gcc
    cd "${NUTTXTOOLS}"/renesas-tools/source/gcc
    chmod +x ./contrib/download_prerequisites ./configure ./move-if-change ./libgcc/mkheader.sh
    ./contrib/download_prerequisites
    sed -i '1s/^/@documentencoding ISO-8859-1\n/' ./gcc/doc/gcc.texi
    sed -i 's/@tex/\n&/g' ./gcc/doc/gcc.texi && sed -i 's/@end tex/\n&/g' ./gcc/doc/gcc.texi
    mkdir -p "${NUTTXTOOLS}"/renesas-tools/build/gcc; cd "${NUTTXTOOLS}"/renesas-tools/build/gcc
    "${NUTTXTOOLS}"/renesas-tools/source/gcc/configure --target=rx-elf --prefix="${NUTTXTOOLS}"/renesas-toolchain/rx-elf-gcc \
      --disable-shared --disable-multilib --disable-libssp --disable-libstdcxx-pch --disable-werror --enable-lto \
      --enable-gold --with-pkgversion=GCC_Build_1.02 --with-newlib --enable-languages=c
    make; make install

    # Install newlib
    cd "${NUTTXTOOLS}"/renesas-tools/source/newlib; chmod +x ./configure
    mkdir -p "${NUTTXTOOLS}"/renesas-tools/build/newlib; cd "${NUTTXTOOLS}"/renesas-tools/build/newlib
    "${NUTTXTOOLS}"/renesas-tools/source/newlib/configure --target=rx-elf --prefix="${NUTTXTOOLS}"/renesas-toolchain/rx-elf-gcc
    make; make install
    rm -rf "${NUTTXTOOLS}"/renesas-tools/
  fi

  command rx-elf-gcc --version
}

sparc_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/sparc-gaisler-elf-gcc/bin

  if [ ! -f "${NUTTXTOOLS}/sparc-gaisler-elf-gcc/bin/sparc-gaisler-elf-gcc" ]; then
    local basefile
    basefile=bcc-2.1.0-gcc-linux64
    cd "${NUTTXTOOLS}"
    # Download the SPARC GCC toolchain prebuilt by Gaisler
    wget --quiet https://www.gaisler.com/anonftp/bcc2/bin/${basefile}.tar.xz
    xz -d ${basefile}.tar.xz
    tar xf ${basefile}.tar
    mv bcc-2.1.0-gcc sparc-gaisler-elf-gcc
    rm ${basefile}.tar
  fi

  command sparc-gaisler-elf-gcc --version
}

xtensa_esp32_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/xtensa-esp32-elf/bin

  if [ ! -f "${NUTTXTOOLS}/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc" ]; then
    local basefile
    basefile=xtensa-esp32-elf-12.2.0_20230208-x86_64-linux-gnu
    cd "${NUTTXTOOLS}"
    # Download the latest ESP32 GCC toolchain prebuilt by Espressif
    wget --quiet https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/${basefile}.tar.xz
    xz -d ${basefile}.tar.xz
    tar xf ${basefile}.tar
    rm ${basefile}.tar
  fi

  command xtensa-esp32-elf-gcc --version
}

xtensa_esp32s2_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/xtensa-esp32s2-elf/bin

  if [ ! -f "${NUTTXTOOLS}/xtensa-esp32s2-elf/bin/xtensa-esp32s2-elf-gcc" ]; then
    local basefile
    basefile=xtensa-esp32s2-elf-12.2.0_20230208-x86_64-linux-gnu
    cd "${NUTTXTOOLS}"
    # Download the latest ESP32 S2 GCC toolchain prebuilt by Espressif
    wget --quiet https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/${basefile}.tar.xz
    xz -d ${basefile}.tar.xz
    tar xf ${basefile}.tar
    rm ${basefile}.tar
  fi

  command xtensa-esp32s2-elf-gcc --version
}

xtensa_esp32s3_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/xtensa-esp32s3-elf/bin

  if [ ! -f "${NUTTXTOOLS}/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc" ]; then
    local basefile
    basefile=xtensa-esp32s3-elf-12.2.0_20230208-x86_64-linux-gnu
    cd "${NUTTXTOOLS}"
    # Download the latest ESP32 S3 GCC toolchain prebuilt by Espressif
    wget --quiet https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/${basefile}.tar.xz
    xz -d ${basefile}.tar.xz
    tar xf ${basefile}.tar
    rm ${basefile}.tar
  fi

  command xtensa-esp32s3-elf-gcc --version
}

u_boot_tools() {
  if ! type mkimage > /dev/null 2>&1; then
    sudo apt-get install -y u-boot-tools
  fi
}

wasi_sdk() {
  add_path "${NUTTXTOOLS}"/wamrc

  if [ ! -f "${NUTTXTOOLS}/wasi-sdk/bin/clang" ]; then
    local wasibasefile
    local wasmbasefile
    wasibasefile=wasi-sdk-19.0-linux
    wasmbasefile=wamrc-1.1.2-x86_64-ubuntu-20.04
    cd "${NUTTXTOOLS}"
    mkdir wamrc

    # Download the latest WASI-enabled WebAssembly C/C++ toolchain prebuilt by WASM
    wget --quiet https://github.com/WebAssembly/wasi-sdk/releases/download/wasi-sdk-19/${wasibasefile}.tar.gz
    tar xzf ${wasibasefile}.tar.gz
    mv wasi-sdk-19.0 wasi-sdk
    rm ${wasibasefile}.tar.gz
    cd wamrc
    # Download the latest "wamrc" AOT compiler prebuilt by WAMR
    wget --quiet https://github.com/bytecodealliance/wasm-micro-runtime/releases/download/WAMR-1.1.2/${wasmbasefile}.tar.gz
    tar xzf ${wasmbasefile}.tar.gz
    rm ${wasmbasefile}.tar.gz

  fi

  export WASI_SDK_PATH="${NUTTXTOOLS}/wasi-sdk"
  echo "export WASI_SDK_PATH=${NUTTXTOOLS}/wasi-sdk" >> "${NUTTXTOOLS}"/env.sh

  command "${WASI_SDK_PATH}"/bin/clang --version
  command wamrc --version
}

setup_links() {
  # Configure ccache
  mkdir -p "${NUTTXTOOLS}"/ccache/bin/
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/aarch64-none-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/aarch64-none-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/arm-none-eabi-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/arm-none-eabi-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/avr-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/avr-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/cc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/c++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/clang
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/clang++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/p32-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/rx-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/riscv-none-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/riscv-none-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/sparc-gaisler-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/sparc-gaisler-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/x86_64-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/x86_64-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32s2-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32s2-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32s3-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32s3-elf-g++
}

install_build_tools() {
  mkdir -p "${NUTTXTOOLS}"
  echo "#!/usr/bin/env sh" > "${NUTTXTOOLS}"/env.sh

  install="arm_clang_toolchain arm_gcc_toolchain arm64_gcc_toolchain avr_gcc_toolchain binutils bloaty clang_tidy gen_romfs gperf kconfig_frontends mips_gcc_toolchain python_tools riscv_gcc_toolchain rust rx_gcc_toolchain sparc_gcc_toolchain xtensa_esp32_gcc_toolchain u_boot_tools util_linux wasi_sdk c_cache"

  oldpath=$(cd . && pwd -P)
  for func in ${install}; do
    ${func}
  done
  cd "${oldpath}"

  echo "PATH=${PATH}" >> "${NUTTXTOOLS}"/env.sh
  echo "export PATH" >> "${NUTTXTOOLS}"/env.sh
}

install_build_tools
