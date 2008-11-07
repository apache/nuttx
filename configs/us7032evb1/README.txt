Toolchain
^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the SH toolchain (if
  different from the default).

  If you have no SH toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).

  1. You must have already configured Nuttx in <some-dir>nuttx.

     cd tools
     ./configure.sh us7032evb1/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp configs/sh-defconfig .config

  6. make oldconfig

  7. make

  8. Edit setenv.h so that the PATH variable includes the path to the
     newly built binaries.

shterm
^^^^^^

  The USB7032EVB1 supports CMON in PROM.  CMON requires special
  serial interactions in order to upload and download program files.
  Therefore, a standard terminal emulation program (such as minicom)
  cannot be used.

  The shterm subdirectory contains a small terminal emulation
  program that supports these special interactions for file transfers.

