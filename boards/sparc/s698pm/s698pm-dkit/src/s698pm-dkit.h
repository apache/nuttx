/****************************************************************************
 * boards/sparc/s698pm/s698pm-dkit/src/s698pm-dkit.h
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

#ifndef __BOARDS_SPARC_S698PM_S698PM_DKIT_SRC_S698PM_DKIT_H
#define __BOARDS_SPARC_S698PM_S698PM_DKIT_SRC_S698PM_DKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define S698PM_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define S698PM_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Configuration ************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s698pm_ledinit
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void s698pm_led_initialize(void);
#endif

/****************************************************************************
 * Name: s698pm_dkit_watchdog_initialize
 *
 * Description:
 *   Initialize the s698pm watchdog.
 *
 ****************************************************************************/

#ifdef CONFIG_S698PM_DKIT_WDG
int s698pm_dkit_watchdog_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_SPARC_S698PM_S698PM_DKIT_SRC_S698PM_DKIT_H */
