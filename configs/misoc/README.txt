Misoc README
============

Buildroot Toolchain
===================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the LM32 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no LM32 toolchain, one can be cloned from the NuttX
  Bitbucket GIT repository (https://bitbucket.org/nuttx/buildroot).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh misoc/<sub-dir>

  2. Clone the latest buildroot package into <some-dir>

       git clone git@bitbucket.org:nuttx/buildroot.git <some-dir>

     or

       git clone https://patacongo@bitbucket.org/nuttx/buildroot.git <some-dir>

  3. cd <some-dir>

  4. cp lm32-elf-defconfig-6.1.0 .config

  5. make oldconfig

  6. make

  7. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

     By default, the tools will be at:

       <some-dir>/build_lm32/staging_dir/bin

     That location can be changed by reconfiguring the .config file.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you
  are building a LM32 toolchain for Cygwin under Windows.  Also included in
  that README file is a FAQ of frequent build issues that their work-arounds.

  In order to use the buildroot toolchain, you also must set the following
  in your .config file:

    CONFIG_LM3S_TOOLCHAIN_BUILDROOT=y

