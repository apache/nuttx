configs/skp16c26/README.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

BUILDING THE R8C/M16C/M32C GNU TOOLCHAIN
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. CD to the correct directory.

   Change to the directory just above the NuttX installation.  If <nuttx-dir> is
   where NuttX is installed, then cd to <nuttx-dir>/..

2. Get the buildroot Module

   Check out the misc/buildroot module (or download the appropriate buildroot package).
   CVS checkout instructions:

     cvs -d:pserver:anonymous@nuttx.cvs.sourceforge.net:/cvsroot/nuttx login
     cvs -z3 -d:pserver:anonymous@nuttx.cvs.sourceforge.net:/cvsroot/nuttx co -P nuttx

3. Move the buildroot Source Tree and create the archive directory

     mv misc/buildroot .
     mkdir archive

   The <nuttx>/../buildroot is where the toolchain is built;
   The <nuttx>/../archive directory is where toolchain sources will be downloaded.

4. Configure and Make the buildroot

     cd buildroot
     cp configs/m32c-defconfig-4.2.4 .config
     make oldconfig
     make

   This will download the large source packages for the toolchain and build the toolchain.
   The resulting binaries will be under buildroot/build_m32c.

Cygwin GCC BUILD NOTES
^^^^^^^^^^^^^^^^^^^^^^
   On Cygwin, the buildroot 'make' command will fail with an error like:

   "...
      build/genchecksum cc1-dummy > cc1-checksum.c
      opening cc1-dummy: No such file or directory
   ..."

   This is caused because on Cygwin, host executables will be generated with the extension .exe
   and, apparently, the make variable "exeext" is set incorrectly.  A work around after the
   above occurs is:

      cd toolchain_build_m32c/gcc-4.2.4-initial/gcc	# Go to the directory where error occurred
      mv cc1-dummy.exe cc1-dummy			# Rename the executable without .exe
      rm cc1-checksum.c					# Get rid of the bad generated file

   Then resume the buildroot make:

      cd -						# Back to the buildroot make directory
      make						# Restart the build

   GCC is built twice.  First a initial, "bootstap" GCC is produced in
   toolchain_build_m32c/gcc-4.2.4-initial, then the final GCC is produced in
   toolchain_build_m32c/gcc-4.2.4-final.  The above error will occur twice:  Once for
   the intial GCC build (see above) and once for the final GCC build. For the final GCC
   build, the workaround is the same except that the directory will be toolchain_build_m32c/gcc-4.2.4-final/gcc.

   