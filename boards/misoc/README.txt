Misoc README
============

  This README applies to a port to NuttX running on a Qemu LM32 system.  You
  can find the Qemu setup at https://bitbucket.org/key2/qemu

  This initial release supports two UARTs, but does not have a system timer
  or other peripherals.  More to come.

Buildroot Toolchain
===================

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the LM32 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no LM32 toolchain, one can be cloned from the NuttX
  Bitbucket GIT repository (https://bitbucket.org/nuttx/buildroot).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     tools/configure.sh misoc/<sub-dir>
     make oldconfig context

  2. Clone the latest buildroot package into <some-dir>/buildroot

       git clone git@bitbucket.org:nuttx/buildroot.git <some-dir>/buildroot

     or

       git clone https://patacongo@bitbucket.org/nuttx/buildroot.git <some-dir>/buildroot

  3. cd <some-dir>/buildroot

  4. cp lm32-elf-defconfig-6.1.0 .config

  5. make oldconfig

  6. make

  7. By default, the tools will be at the absolute path:

       <some-dir>/buildroot/build_lm32/staging_dir/bin

     Or the NuttX relative path:

       ../buildroot/build_lm32/staging_dir/bin

     Make sure that he PATH variable includes the path to the newly built
     binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you
  are building a LM32 toolchain for Cygwin under Windows.  Also included in
  that README file is a FAQ of frequent build issues that their work-arounds.

  In order to use the buildroot toolchain, you also must set the following
  in your .config file:

    CONFIG_LM3S_TOOLCHAIN_BUILDROOT=y

configs/misoc/include/generated
===============================

  In order to build this configuration, you must provide the
  configs/misoc/include/generated directory.  It contains the generated
  Misoc files and may be a symbolic link.  The base configurtion will NOT
  build without this directory!

  There is a sample generated directory at configs/misoc/include/generated-sample.
  If you want to do a test build without generating the architecture, then
  you can simply link this sample directory like:

    $ ln -s configs/misoc/include/generated-sample configs/misoc/include/generated

  That should permit a test build.

