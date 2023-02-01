/****************************************************************************
 * boards/risc-v/litex/arty_a7/src/arty_a7.h
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

#ifndef __BOARDS_RISCV_LITEX_ARTY_A7_SRC_ARTY_A7_H
#define __BOARDS_RISCV_LITEX_ARTY_A7_SRC_ARTY_A7_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * configuration
 */

#define HAVE_SDMMC          1
#define HAVE_AUTOMOUNTER    1

/****************************************************************************
 * SDIO Configuration
 */

#define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#define SDIO_SLOTNO CONFIG_NSH_MMCSDSLOTNO

/* Can't support MMC/SD if the card interface(s) are not enable */

#if !defined(CONFIG_LITEX_SDIO) && !defined(CONFIG_LITEX_SDIO1)
#  undef HAVE_SDMMC
#endif

#if !defined(CONFIG_FS_AUTOMOUNTER)
#  undef HAVE_AUTOMOUNTER
#endif

/****************************************************************************
 * PROC File System Configuration
 */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define LITEX_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define LITEX_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: litex_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int litex_bringup(void);

/****************************************************************************
 * Name: litex_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#ifdef HAVE_SDMMC
int litex_sdio_initialize(void);
#endif

/****************************************************************************
 * Name:  litex_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enabled MikroBus MMCSD
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
int litex_automount_initialize(void);
#endif

/****************************************************************************
 * Name: litex_automount_event
 *
 * Description:
 *   The MMCSD card detection logic has detected an insertion or removal
 *   event.  It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the MB slot: MB1_MMCSD_SLOTNO or MB2_MMCSD_SLOTNO.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void litex_automount_event(int slotno, bool inserted);
#endif

/****************************************************************************
 * Name: litex_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

#ifdef HAVE_SDMMC
bool litex_cardinserted(int slotno);
#endif

/****************************************************************************
 * Name: litex_pwm_setup
 *
 * Description:
 *   Initialise all PWM channels enabled in gateware and map them to
 *   /dev/pwmX. Where X is the PMW channel number. From 0 ... LITEX_PWM_MAX.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ENODEV is return on the first PWM device initialise failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LITEX_PWM
int litex_pwm_setup(void);
#endif

#endif /* __BOARDS_RISCV_LITEX_ARTY_A7_SRC_ARTY_A7_H */
