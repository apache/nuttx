/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_wdog.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_WDOG_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Magic value for REFRESH register to reload watchdog countdown counter */

#define WDOG_REFRESH_RELOAD          (0xdeadc0de)

/* Magic value for FORCE register to perform immediate system reset */

#define WDOG_FORCE_IMMEDIATE_RESET   (0x0c)

/****************************************************************************
 * Register Offsets
 ****************************************************************************/

#define MPFS_WDOG_REFRESH_OFFSET            0x000 /* Write value 0xdeadc0de to reset wdog. Read to get current count value */
#define MPFS_WDOG_CONTROL_OFFSET            0x004 /* WDOG counter register */
#define MPFS_WDOG_STATUS_OFFSET             0x008 /* WDOG status register */
#define MPFS_WDOG_TIME_OFFSET               0x00C /* Set WDOG time value */
#define MPFS_WDOG_MSVP_OFFSET               0x010 /* Set MSVP int level */
#define MPFS_WDOG_TRIGGER_OFFSET            0x014 /* Set NMI int level */
#define MPFS_WDOG_FORCE_OFFSET              0x018 /* Force trigger WDOG NMI seq. Writing 0xc triggers immediate reset */

/****************************************************************************
 * Control register masks
 ****************************************************************************/

#define WDOG_CONTROL_INTEN_MSVP_MASK        (1 << 0) /* Bit 0:  Enable MVRP interrupt when MVRP level is passed */
#define WDOG_CONTROL_INTEN_TRIG_MASK        (1 << 1) /* Bit 1:  Enable NMI interrupt. This bit is permanently set */
#define WDOG_CONTROL_INTEN_SLEEP_MASK       (1 << 2) /* Bit 2:  Enable MVRP interrupt when MVRP level is passed and M3 is sleeping */
#define WDOG_CONTROL_ACTIVE_SLEEP_MASK      (1 << 3) /* Bit 3:  Set WDOG operational during CPU sleep */
#define WDOG_CONTROL_ENABLE_FORBITTEN_MASK  (1 << 4) /* Bit 4:  Enable trigger wdog from write during forbidden window */

/****************************************************************************
 * Status register masks
 ****************************************************************************/

#define WDOG_STATUS_MVRP_TRIPPED_MASK       (1 << 0) /* Bit 0:  MVRP level has passed. Write to clear interrupt */
#define WDOG_STATUS_WDOG_TRIPPED_MASK       (1 << 1) /* Bit 1:  TRIGGER level has passed and NMI is asserted. Write to clear interrupt */
#define WDOG_STATUS_FORBITTEN_MASK          (1 << 2) /* Bit 2:  Watchdog in forbidden window */
#define WDOG_STATUS_TRIGGERED_MASK          (1 << 3) /* Bit 3:  Watchdog has triggered */
#define WDOG_STATUS_LOCKED_MASK             (1 << 4) /* Bit 4:  Following registers are locked and cannot be changed */
#define WDOG_STATUS_DEVRST_MASK             (1 << 5) /* Bit 5:  DEVRST caused NMI */

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_WDOG_H */
