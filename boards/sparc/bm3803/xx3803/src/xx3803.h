/****************************************************************************
 * boards/sparc/bm3803/xx3803/src/xx3803.h
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

#ifndef __BOARDS_SPARC_BM3803_XX3803_SRC_XX3803_H
#define __BOARDS_SPARC_BM3803_XX3803_SRC_XX3803_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
#define GPIO_PULSE_IRQ  0     /* GPIO 4*/
#define GPIO_1553B_IRQ  1     /* GPIO 5*/
#define GPIO_RS422_IRQ  2     /* GPIO 6*/
#define GPIO_SPACEWIRE_IRQ  3 /* GPIO 7*/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define BM3803_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define BM3803_PROCFS_MOUNTPOINT "/proc"
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
 * Name: bm3803_ledinit
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void bm3803_led_initialize(void);
#endif

/****************************************************************************
 * Name: bm3803_am29lv_initialize
 *
 * Description:
 *   Initialize and register the AM29LV FLASH file system.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AM29LV
int bm3803_am29lv_initialize(int minor);
#endif

#ifdef CONFIG_DRIVERS_1553B
int bm3803_1553b_initialize(void);
#endif

#ifdef CONFIG_DRIVERS_SPACEWIRE
int bm3803_spacewire_initialize(void);
#endif

#ifdef CONFIG_XX3803_WDG
int xx3803_watchdog_initialize(void);
#endif

#ifdef CONFIG_FPGA_UART
int bm3803_fpga_uart_initialize(void);
#endif

#ifdef CONFIG_SECOND_PULSE
int bm3803_secpulse_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_SPARC_BM3803_XX3803_SRC_XX3803_H */
