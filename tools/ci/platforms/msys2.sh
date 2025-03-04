#!/usr/bin/env sh
############################################################################
# tools/ci/platforms/msys2.sh
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

# MSYS2

set -e
set -o xtrace

add_path() {
  PATH=$1:${PATH}
}

arm_clang_toolchain() {
  add_path "${NUTTXTOOLS}"/clang-arm-none-eabi/bin

  if [ ! -f "${NUTTXTOOLS}/clang-arm-none-eabi/bin/clang" ]; then
    local basefile
    basefile=LLVMEmbeddedToolchainForArm-17.0.1-Windows-x86_64
    cd "${NUTTXTOOLS}"
    # Download the latest ARM clang toolchain prebuilt by ARM
    curl -O -L -s https://github.com/ARM-software/LLVM-embedded-toolchain-for-Arm/releases/download/release-17.0.1/${basefile}.zip
    unzip -qo ${basefile}.zip
    mv ${basefile} clang-arm-none-eabi
    rm ${basefile}.zip
  fi

  command clang --version
}

arm_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/gcc-arm-none-eabi/bin

  if [ ! -f "${NUTTXTOOLS}/gcc-arm-none-eabi/bin/arm-none-eabi-gcc" ]; then
    local basefile
    basefile=arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi
    cd "${NUTTXTOOLS}"
    curl -O -L -s https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/${basefile}.zip
    unzip -qo ${basefile}.zip
    mv ${basefile} gcc-arm-none-eabi
    rm ${basefile}.zip
  fi

  command arm-none-eabi-gcc --version
}

arm64_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/gcc-aarch64-none-elf/bin

  if [ ! -f "${NUTTXTOOLS}/gcc-aarch64-none-elf/bin/aarch64-none-elf-gcc" ]; then
    local basefile
    basefile=arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-aarch64-none-elf
    cd "${NUTTXTOOLS}"
    # Download the latest ARM64 GCC toolchain prebuilt by ARM
    curl -O -L -s https://developer.arm.com/-/media/Files/downloads/gnu/13.2.Rel1/binrel/${basefile}.zip
    unzip -qo ${basefile}.zip
    mv ${basefile} gcc-aarch64-none-elf
    rm ${basefile}.zip
  fi

  command aarch64-none-elf-gcc --version
}

c_cache() {
  add_path "${NUTTXTOOLS}"/ccache/bin

  if ! type ccache > /dev/null 2>&1; then
    pacman -S --noconfirm --needed ccache
  fi
  setup_links
  command ccache --version
}

esp_tool() {
  add_path "${NUTTXTOOLS}"/esp-tool

  if ! type esptool > /dev/null 2>&1; then
    local basefile
    basefile=esptool-v4.8.0-win64
    cd "${NUTTXTOOLS}"
    curl -O -L -s https://github.com/espressif/esptool/releases/download/v4.8.0/${basefile}.zip
    unzip -qo ${basefile}.zip
    mv esptool-win64 esp-tool
    rm ${basefile}.zip
  fi
  command esptool version
}

gen_romfs() {
  add_path "${NUTTXTOOLS}"/genromfs/usr/bin

  if ! type genromfs > /dev/null 2>&1; then
    git clone --depth 1 https://bitbucket.org/nuttx/tools.git "${NUTTXTOOLS}"/nuttx-tools
    cd "${NUTTXTOOLS}"/nuttx-tools
    tar zxf genromfs-0.5.2.tar.gz
    cd genromfs-0.5.2
    make install PREFIX="${NUTTXTOOLS}"/genromfs
    cd "${NUTTXTOOLS}"
    rm -rf nuttx-tools
  fi
}

kconfig_frontends() {
  add_path "${NUTTXTOOLS}"/kconfig-frontends/bin

  if [ ! -f "${NUTTXTOOLS}/kconfig-frontends/bin/kconfig-conf" ]; then
    git clone --depth 1 https://bitbucket.org/nuttx/tools.git "${NUTTXTOOLS}"/nuttx-tools
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

python_tools() {
  pip3 install \
    construct
}

mips_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/pinguino-compilers/windows64/p32/bin

  if [ ! -d "${NUTTXTOOLS}/pinguino-compilers" ]; then
    cd "${NUTTXTOOLS}"
    git clone https://github.com/PinguinoIDE/pinguino-compilers
  fi

  command p32-gcc --version
}

riscv_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/riscv-none-elf-gcc/bin

  if [ ! -f "${NUTTXTOOLS}/riscv-none-elf-gcc/bin/riscv-none-elf-gcc" ]; then
    local basefile
    basefile=xpack-riscv-none-elf-gcc-13.2.0-2-win32-x64
    cd "${NUTTXTOOLS}"
    # Download the latest RISCV GCC toolchain prebuilt by xPack
    curl -O -L -s https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/${basefile}.zip
    unzip -qo ${basefile}.zip
    mv xpack-riscv-none-elf-gcc-13.2.0-2 riscv-none-elf-gcc
    rm ${basefile}.zip
  fi
  command riscv-none-elf-gcc --version
}

rust() {
  add_path "${NUTTXTOOLS}"/rust/cargo/bin
  # Configuring the PATH environment variable
  export CARGO_HOME=${NUTTXTOOLS}/rust/cargo
  export RUSTUP_HOME=${NUTTXTOOLS}/rust/rustup
  echo "export CARGO_HOME=${NUTTXTOOLS}/rust/cargo" >> "${NUTTXTOOLS}"/env.sh
  echo "export RUSTUP_HOME=${NUTTXTOOLS}/rust/rustup" >> "${NUTTXTOOLS}"/env.sh
  if ! type rustc > /dev/null 2>&1; then
    local basefile
    basefile=x86_64-pc-windows-gnu
    mkdir -p "${NUTTXTOOLS}"/rust
    cd "${NUTTXTOOLS}"
    # Download tool rustup-init.exe
    curl -O -L -s https://static.rust-lang.org/rustup/dist/x86_64-pc-windows-gnu/rustup-init.exe
    # Install Rust target x86_64-pc-windows-gnu
    ./rustup-init.exe -y --default-host ${basefile} --no-modify-path
    # Install targets supported from NuttX
    "$CARGO_HOME"/bin/rustup target add thumbv6m-none-eabi
    "$CARGO_HOME"/bin/rustup target add thumbv7m-none-eabi
    "$CARGO_HOME"/bin/rustup target add riscv64gc-unknown-none-elf
    rm rustup-init.exe
  fi
  command rustc --version
}

sparc_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/sparc-gaisler-elf-gcc/bin

  if [ ! -f "${NUTTXTOOLS}/sparc-gaisler-elf-gcc/bin/sparc-gaisler-elf-gcc" ]; then
    local basefile
    basefile=bcc-2.1.0-gcc-mingw64
    cd "${NUTTXTOOLS}"
    # Download the SPARC GCC toolchain prebuilt by Gaisler
    curl -O -L -s https://www.gaisler.com/anonftp/bcc2/bin/${basefile}.zip
    unzip -qo ${basefile}.zip
    mv bcc-2.1.0-gcc sparc-gaisler-elf-gcc
    rm ${basefile}.zip
  fi

  command sparc-gaisler-elf-gcc --version
}

xtensa_esp32_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/xtensa-esp32-elf/bin

  if [ ! -f "${NUTTXTOOLS}/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc" ]; then
    local basefile
    basefile=xtensa-esp32-elf-12.2.0_20230208-x86_64-w64-mingw32
    cd "${NUTTXTOOLS}"
    # Download the latest ESP32 GCC toolchain prebuilt by Espressif
    curl -O -L -s https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/${basefile}.zip
    unzip -qo ${basefile}.zip
    rm ${basefile}.zip
  fi

  command xtensa-esp32-elf-gcc --version
}

xtensa_esp32s2_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/xtensa-esp32s2-elf/bin

  if [ ! -f "${NUTTXTOOLS}/xtensa-esp32s2-elf/bin/xtensa-esp32s2-elf-gcc" ]; then
    local basefile
    basefile=xtensa-esp32s2-elf-12.2.0_20230208-x86_64-w64-mingw32
    cd "${NUTTXTOOLS}"
    # Download the latest ESP32 S2 GCC toolchain prebuilt by Espressif
    curl -O -L -s https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/${basefile}.zip
    unzip -qo ${basefile}.zip
    rm ${basefile}.zip
  fi

  command xtensa-esp32s2-elf-gcc --version
}

xtensa_esp32s3_gcc_toolchain() {
  add_path "${NUTTXTOOLS}"/xtensa-esp32s3-elf/bin

  if [ ! -f "${NUTTXTOOLS}/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc" ]; then
    local basefile
    basefile=xtensa-esp32s3-elf-12.2.0_20230208-x86_64-w64-mingw32
    cd "${NUTTXTOOLS}"
    # Download the latest ESP32 S3 GCC toolchain prebuilt by Espressif
    curl -O -L -s https://github.com/espressif/crosstool-NG/releases/download/esp-12.2.0_20230208/${basefile}.zip
    unzip -qo ${basefile}.zip
    rm ${basefile}.zip
  fi

  command xtensa-esp32s3-elf-gcc --version
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
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/riscv64-unknown-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/riscv64-unknown-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/sparc-gaisler-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/sparc-gaisler-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/x86_64-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/x86_64-elf-g++
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32-elf-gcc
  ln -sf "$(which ccache)" "${NUTTXTOOLS}"/ccache/bin/xtensa-esp32-elf-g++
}

install_build_tools() {
  mkdir -p "${NUTTXTOOLS}"
  echo "#!/usr/bin/env sh" > "${NUTTXTOOLS}"/env.sh

  install="arm_clang_toolchain arm_gcc_toolchain arm64_gcc_toolchain kconfig_frontends riscv_gcc_toolchain rust python_tools"

  oldpath=$(cd . && pwd -P)
  for func in ${install}; do
    ${func}
  done
  cd "${oldpath}"

  echo "PATH=${PATH}" >> "${NUTTXTOOLS}"/env.sh
  echo "export PATH" >> "${NUTTXTOOLS}"/env.sh
}

install_build_tools
