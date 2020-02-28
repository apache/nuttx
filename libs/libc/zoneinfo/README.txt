apps/system/zoneinfo/README.txt
Author: Gregory Nutt <gnutt@nuttx.org>

Directory Contents
==================

This directory contains logic to create a version of the TZ/Olson database.
This database is required if localtime() support is selected via
CONFIG_LIBC_LOCALTIME.  This logic in this directory does the following:

  - It downloads the current TZ database from the IANA website
  - It downloads the current timezone tools from the same location
  - It builds the tools and constructs the binary TZ database
  - It will then, optionally, build a ROMFS filesystem image containing
    the data base.

Creating and Mounting a ROMFS TZ Database
=========================================

The ROMFS filesystem image can that be mounted during the boot-up sequence
so that it is available for the localtime() logic.  There are two steps to
doing this:

  - First, a ROM disk device must be created.  This is done by calling
    the function romdisk_register() as described in
    nuttx/include/nuttx/drivers/ramdisk.h.  This is an OS level operation
    and must be done in the board-level logic before your application
    starts.

    romdisk_register() will create a block driver at /dev/ramN where N
    is the device minor number that was provided to romdisk_register.

  - The second step is to mount the file system.  This step can be
    performed either in your board configuration logic or by your
    application using the mount() interface described in
    nuttx/include/sys/mount.h.

    These steps, however, must be done very early in initialization,
    before there is any need for time-related services.

Both of these steps are shown together in the following code sample at the
end of this README file.

Example Configuration
=====================

I have tested this using the sim/nsh configuration.  Here are the
modifications to the configuration that I used for testing:

  CONFIG_BOARD_LATE_INITIALIZE=y

  CONFIG_LIBC_LOCALTIME=y
  CONFIG_LIBC_TZDIR="/share/zoneinfo"
  CONFIG_LIBC_TZ_MAX_TIMES=370
  CONFIG_LIBC_TZ_MAX_TYPES=20

  CONFIG_LIB_ZONEINFO=y
  CONFIG_LIB_ZONEINFO_ROMFS=y

NOTE:  The full TZ database is quite large.  To create a reasonable sized
ROMFS image, I had to trim some of the files like this:

  cd nuttx
  tools/configure.sh sim:nsh
  make menuconfig

Select the above localtime() and nuttx/zoneinfo configuration settings.
Then:

  make context
  cd ../nuttx/zoneinfo/tzbin/etc/zoneinfo

Remove as many timezone files as you can.  Do not remove the GMT, localtime,
or posixrules files.  Those might be needed in any event.  Then you can
force rebuilding of the ROMFS filesystem be removing some files:

  cd ../../..
  rm romfs_zoneinfo.*
  rm *.o
  cd ../../nuttx
  make

If you have problems building the simulator on your platform, check out
nuttx/boards/sim/sim/sim/README.txt.  You might find some help there.

Here is a sample run.  I have not seen any errors in single stepping through
the logic but neither am I certain that everything is working properly:

  NuttShell (NSH)
  nsh> date
  Jul 01 00:00:02 2008
  nsh> set TZ US/Mountain
  nsh> date -s "Apr 11 11:53:00 2015"
  nsh> date
  Apr 11 17:53:00 2015

NOTE: Because of daylight savings time, US/Mountain is GMT-6 on Apr 11.  The
above suggests that perhaps the NSH data command may be setting local time,
but printing GMT time?

Sample Code to Mount the ROMFS Filesystem
=========================================

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/drivers/ramdisk.h>
#include <nuttx/zoneinfo.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LIBC_TZDIR
#  error CONFIG_LIBC_TZDIR is not defined
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "Mountpoint support is disabled"
#endif

#if CONFIG_NFILE_DESCRIPTORS < 4
#  error "Not enough file descriptors"
#endif

#ifndef CONFIG_FS_ROMFS
#  error "ROMFS support not enabled"
#endif

#define SECTORSIZE  64
#define NSECTORS(b) (((b)+SECTORSIZE-1)/SECTORSIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mount_zoneinfo(int minor)
{
   char devname[32];
   int  ret;

  /* Create a RAM disk for the test */

  ret = romdisk_register(minor, romfs_zoneinfo_img,
                         NSECTORS(romfs_zoneinfo_img_len), SECTORSIZE);
  if (ret < 0)
    {
      printf("ERROR: Failed to create RAM disk\n");
      return ret;
    }

  /* Use the minor number to create a name for the ROM disk block device */

  snprintf(devname, 32, "/dev/ram%d", minor);

  /* Mount the ROMFS file system */

  printf("Mounting ROMFS filesystem at target=%s with source=%s\n",
         CONFIG_LIBC_TZDIR, devname);

  ret = mount(devname, CONFIG_LIBC_TZDIR, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      printf("ERROR: Mount failed: %d\n", errno);
      return ret;
    }

  printf("TZ database mounted at %s\n", CONFIG_LIBC_TZDIR);
  return OK;
}
