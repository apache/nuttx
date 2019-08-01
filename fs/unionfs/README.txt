fs/unionfs/README.txt
=====================

  Overview
  --------
  This directory contains the NuttX Union File System.  The Union file
  system is provides a mechanism to overlay two different, mounted file
  systems so that they appear as one.  In general this works like this:

  1) Mount file system 1 at some location, say /mnt/file1
  2) Mount file system 2 at some location, say /mnt/file2
  3) Call unionfs_mount() to combine and overly /mnt/file1 and mnt/file2
     as a new mount point, say /mnt/unionfs.

  /mnt/file1 and /mnt/file2 will disappear and be replaced by the single
  mountpoint /mnut/unionfs.  The previous contents under /mnt/file1 and
  /mnt/file2 will appear merged under /mnt/unionfs. Files at the same
  relative path in file system1 will take presence. If another file of the
  same name and same relative location exists 	in file system 2, it will
  not be visible because it will be occluded by the file in file system1.

  See include/nutts/unionfs.h for additional information.

  The Union File System is enabled by selecting the CONFIG_FS_UNIONFS option
  in the NuttX configuration file.

  Disclaimer:  This Union File System was certainly inspired by UnionFS
  (http://en.wikipedia.org/wiki/UnionFS) and the similarity in naming is
  unavoidable.  However, other than that, the NuttX Union File System
  has no relationship with the UnioinFS project in specification, usage,
  design, or implementation.

  Uses of the Union File System
  ------------------------------
  The original motivation for this file was for the use of the built-in
  function file system (BINFS) with a web server.  In that case, the built
  in functions provide CGI programs.  But the BINFS file system cannot hold
  content.  Fixed content would need to be retained in a more standard file
  system such as ROMFS.  With this Union File System, you can overly the
  BINFS mountpoint on the ROMFS mountpoint, providing a single directory
  that appears to contain the executables from the BINFS file system along
  with the web content from the ROMFS file system.

  Another possible use for the Union File System could be to augment or
  replace files in a FLASH file system.  For example, suppose that you have
  a product that ships with content in a ROMFS file system provided by the
  on-board FLASH.  Later, you overlay that ROMFS file system with additional
  files from an SD card by using the Union File System to overlay, and
  perhaps replace, the ROMFS files.

  Another use case might be to overlay a read-only file system like ROMFS
  with a writable file system (like a RAM disk).  This should then give
  to a readable/write-able file system with some fixed content.

  Prefixes
  --------

  And optional prefix may be provided with each of the file systems
  combined in by the Union File System.  For example, suppose that:

  o File system 1 is a ROMFS file system with prefix == NULL,
  o File system 2 is a BINFS file system with prefix == "cgin-bin", and
  o The union file system is mounted at /mnt/www.

  Then the content in the in the ROMFS file system would appear at
  /mnt/www and the content of the BINFS file system would appear at
  /mnt/www/cgi-gin.

  Example Configurations
  ----------------------

  o configs/sim/unionfs - This is a simulator configuration that uses the
    Union File System test at apps/examples/unionfs.  That test overlays two
    small ROMFS file systems with many conflicts in directories and file
    names.  This is a good platform for testing the Union file System and
    apps/examples/unionfs is a good example of how to configure the Union
    File System.

  o configs/lincoln60/thttpd-binfs - This is an example using the THTTPD
    web server.  It server up content from a Union File System with fixed
    content provided by a ROMFS file system and CGI content provided by a
    BINFS file system.

    You can see how the Union File System content directory is configured
    by logic in apps/example/thttpd/.

  o configs/olimex-lpc1766stk/thttpd-binfs - This is essentially the same
    as the lincoln60 configuration.  It does not work, however, because the
    LPC1766 has insufficient RAM to support the THTTPD application in this
    configuration.

  See the README.txt file in each of these board directories for additional
  information about these configurations.
