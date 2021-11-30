/****************************************************************************
 * boards/risc-v/mpfs/m100pfsevp/include/board.h
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

#ifndef __BOARDS_RISCV_M100PFSEVP_MPFS_INCLUDE_BOARD_H
#define __BOARDS_RISCV_M100PFSEVP_MPFS_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "mpfs_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MPFS_EMMCSD_MUX_GPIO
/* eMMC / SD-card GPIO selection signal */
#define MPFS_EMMCSD_GPIO (GPIO_BANK0 | GPIO_PIN12 | GPIO_OUTPUT | GPIO_BUFFER_ENABLE)
#endif

/* TODO: check Clocking */

#define MPFS_MSS_EXT_SGMII_REF_CLK (125000000UL)
#define MPFS_MSS_COREPLEX_CPU_CLK  (600000000UL)
#define MPFS_MSS_SYSTEM_CLK        (600000000UL)
#define MPFS_MSS_RTC_TOGGLE_CLK      (1000000UL)
#define MPFS_MSS_AXI_CLK           (300000000UL)
#define MPFS_MSS_APB_AHB_CLK       (150000000UL)
#define MPFS_FPGA_BCLK               (3000000UL)

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

/* Button definitions *******************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_boardinitialize
 ****************************************************************************/

void mpfs_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_M100PFSEVP_MPFS_INCLUDE_BOARD_H  */
