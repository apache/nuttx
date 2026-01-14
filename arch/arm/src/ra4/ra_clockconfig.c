/****************************************************************************
 * arch/arm/src/ra4/ra_clockconfig.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "ra_clockconfig.h"
#include "hardware/ra_flash.h"
#include "hardware/ra_system.h"
#include "hardware/ra_option_setting.h"

const uint32_t option_settings[] __attribute__((section(".rom_registers")))
__attribute__((__used__)) =
{
  /* Option Function Select Register 0 */

  (
  R_OFS0_RESERVED_31 | R_OFS0_WDTSTPCTL | R_OFS0_RESERVED_29 |
  R_OFS0_WDTRSTIRQS | R_OFS0_WDTRPSS_MASK | R_OFS0_WDTRPES_MASK |
  R_OFS0_WDTCKS_MASK | R_OFS0_WDTTOPS_MASK | R_OFS0_WDTSTRT |
  R_OFS0_RESERVED_16_15_MASK | R_OFS0_IWDTSTPCTL | R_OFS0_RESERVED_13 |
  R_OFS0_IWDTRSTIRQS | R_OFS0_IWDTRPSS_MASK | R_OFS0_IWDTRPES_MASK |
  R_OFS0_IWDTCKS_MASK | R_OFS0_IWDTTOPS_MASK | R_OFS0_IWDTSTRT |
  R_OFS0_RESERVED_0
  ),

  /* Option Function Select Register 1 */

  (
  R_OFS1_RESERVED_16_15_MASK | RA_HOCO_FREQUENCY |
  R_OFS1_RESERVED_11_9_MASK | RA_HOCOEN | R_OFS1_RESERVED_7_6_MASK |
  R_OFS1_VDSEL1_MASK | R_OFS1_LVDAS | R_OFS1_RESERVED_1_0_MASK),

  (uint32_t)0x00fffffc,         /* Security MPU Program Counter Start Address
                                 * Register (SECMPUPCS0) */
  (uint32_t)0x00ffffff,         /* Security MPU Program Counter End Address
                                 * Register (SECMPUPCE0)  */
  (uint32_t)0x00fffffc,         /* Security MPU Program Counter Start Address
                                 * Register (SECMPUPCS1) */
  (uint32_t)0x00ffffff,         /* Security MPU Program Counter End Address
                                 * Register (SECMPUPCE1)  */
  (uint32_t)0x00fffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS0) */
  (uint32_t)0x00ffffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE0) */
  (uint32_t)0x200ffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS1) */
  (uint32_t)0x200fffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE1) */
  (uint32_t)0x407ffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS2) */
  (uint32_t)0x407fffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE2) */
  (uint32_t)0x40dffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS3) */
  (uint32_t)0x40dfffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE3) */
  (uint32_t)0xffffffff,         /* Security MPU Access Control Register
                                 * (SECMPUAC) */
};

/** ID code definitions defined here. */

static const uint32_t g_bsp_id_codes[] __attribute__((section(".id_code")))
__attribute__((__used__)) =
{
  IDCODE1, IDCODE2, IDCODE3, IDCODE4
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Key code for writing PRCR register. */

#define BSP_PRV_PRCR_KEY            (0xA500U)
#define BSP_PRV_PRCR_PRC1_UNLOCK    ((BSP_PRV_PRCR_KEY) | 0x2U)
#define BSP_PRV_PRCR_UNLOCK         ((BSP_PRV_PRCR_KEY) | 0x3U)
#define BSP_PRV_PRCR_LOCK           ((BSP_PRV_PRCR_KEY) | 0x0U)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_clockconfig
 *
 * Description:
 *   Called to initialize the RA.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void ra_clockconfig(void)
{
  /* Unlock VBTCR1 register. */

  putreg16((BSP_PRV_PRCR_KEY | R_SYSTEM_PRCR_PRC0 | R_SYSTEM_PRCR_PRC1),
           R_SYSTEM_PRCR);

  /* The VBTCR1.BPWSWSTP must be set after reset on MCUs that have
   * VBTCR1.BPWSWSTP.
   * Reference section 11.2.1 "VBATT Control Register 1 (VBTCR1)" and Figure
   * 11.2
   * "Setting flow of the VBTCR1.BPWSWSTP bit" in the RA manual
   * R01UM0007EU0110. This must be done before bsp_clock_init because LOCOCR,
   * LOCOUTCR, SOSCCR, and SOMCR cannot be accessed until VBTSR.VBTRVLD is
   * set.
   * */

  modifyreg8(R_SYSTEM_VBTCR1, 0, R_SYSTEM_VBTCR1_BPWSWSTP);
  while ((getreg8(R_SYSTEM_VBTSR) & R_SYSTEM_VBTSR_VBTRVLD) == 0)
    {
    }

  /* Disable FCache. */

  modifyreg16(R_FCACHE_FCACHEE, R_FCACHE_FCACHEE_FCACHEEN, 0);

  modifyreg8(R_SYSTEM_SCKSCR, R_SYSTEM_SCKSCR_CKSEL_MASK, RA_CKSEL);

  /* lock VBTCR1 register. */

  putreg16(0, R_SYSTEM_PRCR);

#if (RA_ICLK_FREQUENCY > 32000000)
  modifyreg8(R_SYSTEM_MEMWAIT, 0, R_SYSTEM_MEMWAIT_MEMWAIT);
#endif

  modifyreg32(R_SYSTEM_SCKDIVCR,
              (R_SYSTEM_SCKDIVCR_FCK_MASK | R_SYSTEM_SCKDIVCR_ICK_MASK |
               R_SYSTEM_SCKDIVCR_PCKA_MASK | R_SYSTEM_SCKDIVCR_PCKB_MASK |
               R_SYSTEM_SCKDIVCR_PCKC_MASK | R_SYSTEM_SCKDIVCR_PCKD_MASK),
              (RA_FCK_DIV | RA_ICK_DIV | RA_PCKA_DIV | RA_PCKB_DIV |
               RA_PCKC_DIV |
               RA_PCKD_DIV));
}
