fs/unionfs/README.txt
=====================

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

  The Union File Sysem is enabled by selecting the CONFIG_FS_UNIONFS option
  in the NuttX configruation file.

  The original motivation for this file was for the use of the built-in
  function file system (BINFS) with a web server.  In that case, the built
  in functions provide CGI programs.  But the BINFS file system cannot hold
  content.  Fixed content would need to be retained in a more standard file
  system such as ROMFS.  With this Union File System, you can overly the
  BINFS mountpoint on the the ROMFS mountpoint, providing a single directory
  that appears to contain the executables from the BINFS file system along
  with the web content from the ROMFS file system.

  Another possible use for the Union File System could be to augment or
  replace files in a FLASH file system.  For example, suppose that you have
  a product that ships with content in a ROMFS file system provided by the
  on-board FLASH.  Later, you overlay that ROMFS file system with additional
  files from an SD card by using the Union File System to overlay, and
  perhaps replace, the ROMFS files.
