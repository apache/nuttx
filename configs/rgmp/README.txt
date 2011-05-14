RGMP README File
================

RGMP stands for RTOS and GPOS on Multi-Processor.  RGMP is a project for 
running GPOS and RTOS simultaneously on multi-processor platforms. You can
port your favorite RTOS to RGMP together with an unmodified Linux to form a
hybrid operating system. This makes your application able to use both RTOS
and GPOS features.

See http://rgmp.sourceforge.net/wiki/index.php/Main_Page for further
information about RGMP.

Build Instructions
------------------

1. Download RGMP from rgmp.svn.sourceforge.net:

   cd <rgmp-dir>
   
   svn co https://rgmp.svn.sourceforge.net/svnroot/rgmp/trunk rgmp

2. Configure NuttX.  For example, for the RGMP NSH configuration, do the
   following:

   cd <nuttx-dir>
   cd tools
   ./configure.sh rgmp/nsh
   cd ..

3. Edit Make.defs so that it can determine the include paths to the RGMP
   installation directory.  I made this change to <nuttx-dir>/Make.defs:

   --- configs/rgmp/nsh/Make.defs  2011-05-12 11:19:26.527273500 -0600
   +++ ./Make.defs 2011-05-12 12:52:42.847364700 -0600
   @@ -37,6 +37,7 @@
    
    include ${TOPDIR}/.config
    
   +RGMP_INST_DIR := /home/patacongo/projects/rgmp/rgmp/rtos
    RGMPLIBDIR := $(RGMP_INST_DIR)/lib
    RGMPINCDIR := $(RGMP_INST_DIR)/include
    RGMPLKSCPT := $(RGMP_INST_DIR)/etc/x86.ld

NOTE: This needs to be better integrated with the RGMP build process. If
you follow the above instructions, NuttX for RGMP will compile error-free
but will fail at link time:

  LD:  nuttx.exe
  ld: cannot open linker script file /home/patacongo/projects/rgmp/rgmp/rtos/etc/x86.ld: No such file or directory

