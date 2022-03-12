z80sim README
^^^^^^^^^^^^^

This port uses a primitive, emulated Z80 and the SDCC toolchain.
This port uses an instruction set simulator called z80sim.

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using version 2.6.0 of the SDCC toolchain.
IMPORTANT: See notes in the SDCC section.

Contents
^^^^^^^^

  o Configuring NuttX
  o Reconfiguring NuttX
  o Reconfiguring for Windows Native, Cygwin, or macOS
  o SDCC
  o Building the SDCC toolchain

Configuring NuttX
^^^^^^^^^^^^^^^^^

  ostest

    This configuration performs a simple, minimal OS test using
    examples/ostest.  This can be configured as follows:

    1) From a POSIX window:

         tools/configure.sh [OPTIONS] z80sim:ostest

       where you need to select the right [OPTIONS] for your build
       environment.  Do:

         tools/configure.sh -h

       to see the options.

    2) Make sure that your PATH environment variable includes the path
       to the SDCC toolchain.

    3) Then build the binaries:

          make

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  See the
       "Reconfiguring" section below for information about changing this
       configuration.

    2. The default setup for this configuration builds under Linux.
       See the section entitled "Reconfiguring for Windows Native, Cygwin,
       or macOS" which will give you the steps you would need to do to convert
       this configuration to build in other environments.

    3. This configuration was last verified successfully prior to the
       the configure to Kconfig/mconf tool using SDCC 2.6.0 built to run
       natively under Cygwin.  The current build requires ca. 3.2.1 SDCC.

  nsh

    This configuration file builds NSH (examples/nsh).  This
    configuration is not functional due to issues with use of the
    simulated serial driver (see the TODO list).

    This configuration can be selected by:

    1) From a POSIX window:

         tools/configure.sh [OPTIONS] z80sim:nsh

       where you need to select the right [OPTIONS] for your build
       environment.  Do:

         tools/configure.sh -h

       to see the options.

    2) Set the PATH environment variable to include the path to the SDCC
       toolchain.

    3) Then build the binaries:

          make

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  See the
       "Reconfiguring" section below for information about changing this
       configuration.

    2. The default setup for this configuration uses a windows native build.
       See the section entitled "Reconfiguring for Windows Native, Cygwin,
       or macOS" which will give you the steps you would need to do to convert
       this configuration to build in other environments.

    3. This configuration was last verified successfully prior to the
       the configure to Kconfig/mconf tool using SDCC 2.6.0 built to run
       natively under Cygwin.nsh/defconfig:CONFIG_BOARD_LOOPSPERMSEC

Reconfiguring NuttX
^^^^^^^^^^^^^^^^^^^

These configurations all use the kconfig-frontends, mconf-based configuration
tool.  To change this configuration using that tool, you should:

  a. Build and install the kconfig-mconf tool.  See nuttx/README.txt and
     additional README.txt files in the NuttX tools repository.

  b. Execute 'make menuconfig' in nuttx/ in order to start the reconfiguration
     process.

Reconfiguring for Windows Native, Cygwin, or macOS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All of the z80 configurations in this directory are set up to build under
Linux.  That configuration can be converted to run natively in a Windows
CMD.exe shell.  That configuration requires the MinGW host compiler and
several GNUWin32 tools (see discussion in the top-level NuttX/README.txt file)
and the following changes to the configuration file:

  -CONFIG_HOST_LINUX=y
  +CONFIG_HOST_WINDOWS=y
  +CONFIG_WINDOWS_NATIVE=y

  -CONFIG_Z80_TOOLCHAIN_SDCCL=y
  +CONFIG_Z80_TOOLCHAIN_SDCCW=y

You may need to first manually change the CONFIG_APPS_DIR="../apps"
definition in the .config file because the forward slash may upset some
Windows-based tools.

This configuration will require a recent version of SDCC (ca. 3.2.1) for Linux
or custom built for Cygwin (see below).

SDCC
^^^^

IMPORTANT NOTE as of 2020-4-11:  Support for CONFIG_CAN_PASS_STRUCTS was
removed in NuttX-9.1.  This was necessary to enforce some POSIX interface
compliance but also means that ALL older SDCC versions will no long build
with NuttX.  I have been told that the newest SDCC compilers can indeed
pass structure and union parameters and return values.  If that is correct,
then perhaps the newer SDCC compilers will be used.  Otherwise, it will be
necessary to use some other, more compliant compiler.

These z80 configurations all use the SDCC toolchain (http://sdcc.sourceforge.net/).
Source and pre-built SDCC binaries can be downloaded from the SDCC SourceForge
site: http://sourceforge.net/projects/sdcc/files/ .  Pre-built binaries are
available for Linux, macOS, and for Win32.  Various SDCC options can be
selected with:

  CONFIG_Z80_TOOLCHAIN_SDCCL=y : SDCC for Linux, macOS or Cygwin (see below)
  CONFIG_Z80_TOOLCHAIN_SDCCW=y : SDCC for Win32

SDCC versions 3.2.0 or higher are recommended.

Building the SDCC toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^

You may also want to build your own SDCC toolchain.  You might want to do this,
for example, if you are running under Cygwin and want a Cygwin compatible
SDCC toolchain.

The SDCC toolchain is built with the standard configure/make/make install
sequence.  However, some special actions are required to generate libraries
compatible with this build.  First start with the usual steps

  download
  unpack
  cd sdcc
  ./configure

Note if you do not have the gputils packet installed, newer version of the
SDCC configure will fail.  You will have to either install the gputils
package or if you don't need PIC14 or PIC16 support:

  ./configure --disable-pic14-port --disable-pic16-port

Then make the SDCC binaries

  make

and install SDCC:

  sudo make install

Known compilation problems:

    CC:  stdlib/lib_strtof.c
    stdlib/lib_strtof.c:62:6: warning: #warning "Size of exponent is unknown"
    stdlib/lib_strtof.c:76: error 122: dividing by ZERO
    stdlib/lib_strtof.c:102: error 122: dividing by ZERO
    stdlib/lib_strtof.c:76: error 122: dividing by ZERO

  Workaround: Remove lib_strtof.c from libs/libc/stdlib/Make.defs

  In arch/z80/src/z180:  error 26: '_cbr' not a structure/union member
