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
   