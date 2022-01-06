/****************************************************************************
 * boards/sim/sim/sim/src/sim.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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

#ifdef CONFIG_LIBC_ZONEINFO_ROMFS
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
 * Name: sim_foc_setup
 *
 * Description:
 *   Initialize the FOC controller driver.
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_MOTOR_FOC_DUMMY
int sim_foc_setup(void);
#endif

#endif /* __BOARDS_SIM_SIM_SIM_SRC_SIM_H */
