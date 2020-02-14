/****************************************************************************
 * boards/sim/sim/sim/src/sim.h
 *
 *   Copyright (C) 2015-2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_SIM_SIM_SIM_SRC_SIM_H
#define __BOARDS_SIM_SIM_SIM_SRC_SIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SIM_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SIM_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sim_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int sim_bringup(void);

/****************************************************************************
 * Name: sim_zoneinfo
 *
 * Description:
 *   Mount the TZ database.  The nuttx/zoneinfo directory contains
 *   logic to create a version of the TZ/Olson database.
 *   This database is required if localtime() support is selected via
 *   CONFIG_LIBC_LOCALTIME.  This logic in that directory does the following:
 *
 *   - It downloads the current TZ database from the IANA website
 *   - It downloads the current timezone tools from the same location
 *   - It builds the tools and constructs the binary TZ database
 *   - It will then, optionally, build a ROMFS filesystem image containing
 *     the data base.
 *
 *   The ROMFS filesystem image can that be mounted during the boot-up
 *   sequence so that it is available for the localtime logic.
 *   There are two steps todoing this:
 *
 *   - First, a ROM disk device must be created.  This is done by calling
 *     the function romdisk_register() as described in
 *     nuttx/include/nuttx/drivers/ramdisk.h.  This is an OS level operation
 *     and must be done in the board-level logic before your application
 *     starts.
 *
 *     romdisk_register() will create a block driver at /dev/ramN where N
 *     is the device minor number that was provided to romdisk_register.
 *
 *   - The second step is to mount the file system.  This step can be
 *     performed either in your board configuration logic or by your
 *     application using the mount() interface described in
 *     nuttx/include/sys/mount.h.
 *
 *     These steps, however, must be done very early in initialization,
 *     before there is any need for time-related services.
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_ZONEINFO_ROMFS
int sim_zoneinfo(int minor);
#endif

/****************************************************************************
 * Name: sim_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_GPIO
int sim_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: sim_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_SIM_X11FB) && defined(CONFIG_SIM_TOUCHSCREEN)
int sim_tsc_setup(int minor);
#endif

#endif /* __BOARDS_SIM_SIM_SIM_SRC_SIM_H */
