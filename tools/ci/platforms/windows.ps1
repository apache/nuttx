#!/usr/bin/env pswd
############################################################################
# tools/ci/platforms/windows.ps1
# PowerShell script for CI on Windows Native
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

# Windows native

# Set-PSDebug -Trace 0

function add_path() {
  param (
    [string]$Path
  )
  if (!$Path) {
    Write-Host "Error: add_path path file not found" -ForegroundColor Red
    return
  }
  $envPaths = $env:Path -split ';'
  if ($envPaths -notcontains $Path) {
    Add-Content -Path "$NUTTXTOOLS\env.ps1" -Value "add_path $Path"
    $env:PATH = "$Path;" + $env:PATH
  }
}

function add_envpath {
  param(
    [string] $envfile
  )
  $head = @"
#!/usr/bin/env pswd
############################################################################
# env.ps1
# PowerShell script for CI on Windows Native
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

function add_path() {
  param (
       [string]`$Path
  )
  if (!`$Path) {
      Write-Host "Error: add_path path file not found" -ForegroundColor Red
      return
    }
    `$envPaths = `$env:Path -split ';'
    if (`$envPaths -notcontains `$Path) {
        `$env:PATH = "`$Path;" + `$env:PATH
    }
}
"@

  Add-Content -Path "$envfile" -Value "$head"
}

function arm_clang_toolchain {
  Write-Host "Check ARM clang toolchain ..." -ForegroundColor Green
  try {
    if (run_command("clang") -ne 0) {
      add_path "$NUTTXTOOLS\clang-arm-none-eabi\bin"
      if (-not (Test-Path -Path "$NUTTXTOOLS\clang-arm-none-eabi\bin\clang.exe")) {
        # Download the file
        Write-Host "Download: ARM clang toolchain" -ForegroundColor Green
        $basefile = "LLVMEmbeddedToolchainForArm-17.0.1-Windows-x86_64"
        Set-Location "$NUTTXTOOLS"
        # Download the latest ARM clang toolchain prebuilt by ARM
        Invoke-WebRequest -Uri "https://github.com/ARM-software/LLVM-embedded-toolchain-for-Arm/releases/download/release-17.0.1/$basefile.zip" -OutFile "$NUTTXTOOLS\$basefile.zip" -ErrorAction Stop
        Expand-Archive "$NUTTXTOOLS\$basefile.zip"
        Move-Item -Path "$basefile\$basefile" -Destination "clang-arm-none-eabi"
        Remove-Item "$basefile*" -Force
      }
    }
    clang --version
    Write-Host ""
  }
  catch {
    Write-Error "Failed to download the file: $_"
  }
}

function arm_gcc_toolchain() {
  Write-Host "Check ARM GCC toolchain toolchain ..." -ForegroundColor Green
  try {
    if (run_command("arm-none-eabi-gcc") -ne 0) {
      add_path "$NUTTXTOOLS\gcc-arm-none-eabi\bin"
      if (-not (Test-Path -Path "$NUTTXTOOLS\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe")) {
        # Download the file
        Write-Host "Download: ARM GCC toolchain" -ForegroundColor Green
        $basefile = "arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi"
        Set-Location "$NUTTXTOOLS"
        # Download the latest ARM GCC toolchain prebuilt by ARM
        Invoke-WebRequest -Uri "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/$basefile.zip" -OutFile "$NUTTXTOOLS\$basefile.zip" -ErrorAction Stop
        Expand-Archive "$NUTTXTOOLS\$basefile.zip"
        Move-Item -Path "$basefile\$basefile" -Destination "gcc-arm-none-eabi"
        Remove-Item "$basefile*" -Force
      }
    }
    arm-none-eabi-gcc --version
  }
  catch {
    Write-Error "Failed to download the file: $_"
  }
}

function arm64_gcc_toolchain() {
  Write-Host "Check ARM64 GCC toolchain toolchain ..." -ForegroundColor Green

  try {
    if (run_command("aarch64-none-elf-gcc") -ne 0) {
      add_path "$NUTTXTOOLS\gcc-aarch64-none-elf\bin"
      if (-not (Test-Path -Path "$NUTTXTOOLS\gcc-aarch64-none-elf\bin\aarch64-none-elf-gcc.exe")) {
        # Download the file
        Write-Host "Download: ARM64 GCC toolchain" -ForegroundColor Green
        $basefile = "arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-aarch64-none-elf"
        Set-Location "$NUTTXTOOLS"
        # Download the latest ARM64 GCC toolchain prebuilt by ARM
        Invoke-WebRequest -Uri "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.Rel1/binrel/$basefile.zip" -OutFile "$NUTTXTOOLS\$basefile.zip" -ErrorAction Stop
        Expand-Archive "$NUTTXTOOLS\$basefile.zip"
        Move-Item -Path "$basefile\$basefile" -Destination "gcc-aarch64-none-elf"
        Remove-Item "$basefile*" -Force
      }
    }
    aarch64-none-elf-gcc --version
  }
  catch {
    Write-Error "Failed to download the file: $_"
  }  
}

function cmake_tool {
  Write-Host "Check Cmake ..." -ForegroundColor Green
  if (run_command("cmake") -ne 0) {
    add_path "$NUTTXTOOLS\cmake\bin"
    if ($null -eq (Get-Command cmake -ErrorAction SilentlyContinue)) {
      Write-Host "Download: Ninja package" -ForegroundColor Green
      # Download the file
      $basefile = "cmake-3.31.6-windows-x86_64"
      Set-Location "$NUTTXTOOLS"
      # Download tool cmake
      Invoke-WebRequest -Uri "https://github.com/Kitware/CMake/releases/download/v3.31.6/$basefile.zip" -OutFile "$NUTTXTOOLS\$basefile.zip" -ErrorAction Stop
      Expand-Archive "$NUTTXTOOLS\$basefile.zip"
      Move-Item -Path "$basefile" -Destination "cmake"
      Remove-Item "$basefile*" -Force
    }
  }
  cmake --version
}

function kconfig_frontends() {
  Write-Host "Check kconfig-frontends ..." -ForegroundColor Green
  add_path "$NUTTXTOOLS\kconfig-frontends\bin"
  try {
    if (-not (Test-Path -Path "$NUTTXTOOLS\kconfig-frontends\bin\kconfig-conf.exe")) {
      # Download the file
      Write-Host "Download: kconfig-frontends package" -ForegroundColor Green
      $basefile = "kconfig-frontends-windows-mingw64"
      Set-Location "$NUTTXTOOLS"
      # Download the kconfig-frontends prebuilt
      Invoke-WebRequest -Uri "https://github.com/simbit18/kconfig-frontends-windows-mingw64/releases/download/kconfig-frontends-4.11.0/$basefile.zip" -OutFile "$NUTTXTOOLS\$basefile.zip" -ErrorAction Stop
      Expand-Archive "$NUTTXTOOLS\$basefile.zip"
      Move-Item -Path "$basefile\$basefile" -Destination "kconfig-frontends"
      Remove-Item "$basefile*" -Force
      Write-Host "File downloaded successfully to kconfig-frontends"
    }
  }
  catch {
    Write-Error "Failed to download the file: $_"
  }
}

function ninja_tool {
  Write-Host "Check Ninja ..." -ForegroundColor Green
  if (run_command("ninja") -ne 0) {
    add_path "$NUTTXTOOLS\ninja"
    if ($null -eq (Get-Command ninja -ErrorAction SilentlyContinue)) {
      Write-Host "Download: Ninja package" -ForegroundColor Green
      # Download the file
      $basefile = "ninja-win"
      Set-Location "$NUTTXTOOLS"
      # Download tool ninja
      Invoke-WebRequest -Uri "https://github.com/ninja-build/ninja/releases/download/v1.12.1/$basefile.zip" -OutFile "$NUTTXTOOLS\$basefile.zip" -ErrorAction Stop
      Expand-Archive "$NUTTXTOOLS\$basefile.zip" # -DestinationPath  "$basefile"
      Move-Item -Path "$basefile" -Destination "ninja"
      Remove-Item "$basefile*" -Force
    }
  }
  ninja --version
  Write-Host ""
}

function riscv_gcc_toolchain() {
  Write-Host "Check RISCV GCC toolchain ..." -ForegroundColor Green
  add_path "$NUTTXTOOLS\riscv-none-elf-gcc\bin"
  try {
    if (run_command("riscv-none-elf-gcc") -ne 0) {
      add_path "$NUTTXTOOLS\riscv-none-elf-gcc\bin"
      if (-not (Test-Path -Path "$NUTTXTOOLS\riscv-none-elf-gcc\bin\riscv-none-elf-gcc.exe")) {
        Write-Host "Download: RISCV GCC toolchain" -ForegroundColor Green
        $basefile = "xpack-riscv-none-elf-gcc-13.2.0-2-win32-x64"
        Set-Location "$NUTTXTOOLS"
        # Download the latest RISCV GCC toolchain prebuilt by xPack
        Invoke-WebRequest -Uri "https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v13.2.0-2/$basefile.zip" -OutFile "$NUTTXTOOLS\$basefile.zip" -ErrorAction Stop
        Expand-Archive "$NUTTXTOOLS\$basefile.zip"
        Move-Item -Path "$basefile\xpack-riscv-none-elf-gcc-13.2.0-2" -Destination "riscv-none-elf-gcc"
        Remove-Item "$basefile*" -Force
      }
    }
    riscv-none-elf-gcc --version
  }
  catch {
    Write-Error "Failed to download the file: $_"
  }
}

# GitHub Actions runners already have rustup installed
function rust() {
  Write-Host "Check Rust ..."
  if (run_command("rustc") -ne 0) {
    add_path "$NUTTXTOOLS\rust\cargo\bin"
    # Configuring the PATH environment variable
    $env:CARGO_HOME = "$NUTTXTOOLS\rust\cargo"
    $env:RUSTUP_HOME = "$NUTTXTOOLS\rust\rustup"

    Add-Content -Path "$NUTTXTOOLS\env.ps1" -Value "CARGO_HOME=$NUTTXTOOLS\rust\cargo"
    Add-Content -Path "$NUTTXTOOLS\env.ps1" -Value "RUSTUP_HOME=$NUTTXTOOLS\rust\rustup"

    if ($null -eq (Get-Command rustc -ErrorAction SilentlyContinue)) {
      Write-Host "Download: Rust package" -ForegroundColor Green
      # Download the file
      $basefile = "x86_64-pc-windows-gnu"
      New-Item -ItemType Directory -Path "$NUTTXTOOLS\rust" -Force
      Set-Location "$NUTTXTOOLS"
      # Download tool rustup-init.exe
      Invoke-WebRequest -Uri https://static.rust-lang.org/rustup/dist/x86_64-pc-windows-gnu/rustup-init.exe -OutFile "rustup-init.exe" -ErrorAction Stop
      # Install Rust target x86_64-pc-windows-gnu
      cmd /c start /wait rustup-init.exe -y --default-host $basefile --no-modify-path
      # Install targets supported from NuttX
      cmd /c start /wait rustup.exe target add thumbv6m-none-eabi
      cmd /c start /wait rustup.exe target add thumbv7m-none-eabi
      cmd /c start /wait rustup.exe target add riscv64gc-unknown-none-elf
      Remove-Item rustup-init.exe -Force
    }
  }
  rustc --version
}

function run_command ($command) {
  if ($null -eq (Get-Command "$command" -ErrorAction SilentlyContinue)) {
    return 1
  }
  else {
    return 0
  }
}

function install_build_tools {
  if (-not (Test-Path -Path "$NUTTXTOOLS\env.ps1")) {
    add_envpath "$NUTTXTOOLS\env.ps1"
  }
  $install = "arm_clang_toolchain arm_gcc_toolchain arm64_gcc_toolchain riscv_gcc_toolchain cmake_tool kconfig_frontends ninja_tool"

  $splitArray = $install.Split(" ")
  $oldpath = Get-Location

  foreach ( $node in $splitArray ) {
    & $node
  }
  
  Set-Location "$oldpath"
}

install_build_tools
