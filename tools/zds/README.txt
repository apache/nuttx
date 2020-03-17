README.txt
==========

tools/zds/zdsar.c:  This is a wrapper around the ZDS_II librarian.  It
  simplifies the build scripts by replacing large sequences of complex Bash
  script that were added to the build files.  Not only does this clean upi
  the build files but it also improves performance and, more importantly,i
  provides a common solution for the Windows native build case.

tools/zds/Config.mk:  This makefile fragment is include by ZDS-II Make.defs
  files after including tools/Config.mk.  The definitions in this file
  override some of the the definitions in tools/Config.mk to customize the
  build for use with the ZDS-II tools.

These tools should work with all ZDS-II based platforms including z8, zNeo,
and ez80.

