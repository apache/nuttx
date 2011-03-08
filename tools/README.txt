tools/README.txt
^^^^^^^^^^^^^^^^

This README file addresses the contents of the NuttX tools/ directory.

The tools/ directory contains miscellaneous scripts and host C programs
that are necessary parts of the the NuttX build system.  These files
include:

README.txt

  This file

configure.sh

  This is a bash script that is used to configure NuttX for a given
  target board.  See configs/README.txt or Documentation/NuttxPortingGuide.html
  for a description of how to configure NuttX with this script.

mkconfig.c

  This is C file that is used to build mkconfig program.  The mkconfig
  program is used during the intial NuttX build.

  When you configure NuttX, you will copy a configuration file called .config
  into the top level NuttX directory (See configs/README.txt or
  Documentation/NuttxPortingGuide.html).  The first time you make NuttX,
  the top-level makefile will build the mkconfig executable from mkconfig.c
  (using Makefile.mkconfig).  The top-level Makefile will then execute the
  mkconfig program to convert the .config file in the top level directory
  into include/nuttx/config.h.  config.h is a another version of the
  NuttX configuration that can be included by C files.

mkromfsimg.sh

  This script may be used to automate the generate of a ROMFS file system
  image.  It accepts an rcS script "template" and generates and image that
  may be mounted under /etc in the NuttX pseudo file system.

Makefile.mkconfig

  This is the makefile that is used to make the mkconfig program from
  the mkconfig.c C file.

mkdeps.sh
mknulldeps.sh

  NuttX uses the GCC compilers capabilities to create Makefile dependencies.
  The bash script mkdeps.sh is used to run GCC in order to create the
  dependencies.  If a NuttX configuration uses the GCC toolchain, its Make.defs
  file (see configs/README.txt) will include a line like:

    MKDEP = $(TOPDIR)/tools/mkdeps.sh

  If the NuttX configuration does not use a GCC compatible toolchain, then
  it cannot use the dependencies and instead it uses mknulldeps.sh:

    MKDEP = $(TOPDIR)/tools/mknulldeps.sh

  The mknulldeps.sh is a stub script that does essentially nothing.
  

incdir.sh

  Different compilers have different conventions for specifying lists
  of include file paths on the the compiler command line.  This bash
  script allows the build system to create include file paths without
  concern for the particular compiler in use.

link.sh
winlink.sh
unlink.sh

  Different file system have different capabilities for symbolic links.
  Some windows file systems have no native support for symbolic links.
  Cygwin running under windows has special links built in that work with
  all cygwin tools.  However, they do not work when Windows native tools
  are used with cygwin.  In that case something different must be done.

  If you are building under Linux or under cygwin with a cygwin tool
  chain, then your Make.defs file may have definitions like the
  following:

    DIRLINK = $(TOPDIR)/tools/link.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  The first definition is not always present because link.sh is the
  default.  link.sh is a bash script that performs a normal, Linux-style
  symbolic link;  unlink.sh is a do-it-all unlinking script.

  But if you are building under cygwin using a Windows native toolchain,
  then you will need something like the following in you Make.defs file:

    DIRLINK = $(TOPDIR)/tools/winlink.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  winlink.sh will copy the whole directory instead of linking it.

  NOTE:  I have been told that some NuttX users have been able to build
  successfully using the GnuWin32 tools and modifying the link.sh
  script so that it uses the NTFS mklink command.  But I have never
  tried that

mkimage.sh

  The creates a downloadable image as needed with the rrload bootloader.

indent.sh

  This script can be used to indent .c and .h files in a manner similar
  to my coding NuttX coding style.  It doesn't do a really good job,
  however (see the comments at the top of the indent.sh file).

zipme.sh

  I use this script to create the nutt-xx.yy.tar.gz tarballs for
  release on SourceForge.  It is handy because it also does the
  kind of clean that you need to do to make a clean code release.
