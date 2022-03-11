/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13x2_cc26x2_v1_rom.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of TI's setup_rom.c file which has a fully compatible BSD
 * license:
 *
 *    Copyright (c) 2015-2017, Texas Instruments Incorporated
 *    All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "arm_internal.h"
#include "hardware/tiva_adi2_refsys.h"
#include "hardware/tiva_adi3_refsys.h"
#include "hardware/tiva_adi4_aux.h"
#include "hardware/tiva_aon_batmon.h"
#include "hardware/tiva_aon_pmctl.h"
#include "hardware/tiva_aon_rtc.h"
#include "hardware/tiva_aux_sysif.h"
#include "hardware/tiva_ccfg.h"
#include "hardware/tiva_ddi0_osc.h"
#include "hardware/tiva_fcfg1.h"

#include "cc13xx/cc13x2_cc26x2_v1_rom.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rom_setup_stepvaddrtrimto
 ****************************************************************************/

void rom_setup_stepvaddrtrimto(uint32_t tocode)
{
  uint32_t pmctl_regsetctrl;
  int32_t target_trim;
  int32_t current_trim;

  target_trim =
    rom_signextend_vddrtrim(tocode & (ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_MASK >>
                                     ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT));
  current_trim =
    rom_signextend_vddrtrim((getreg16(TIVA_ADI3_REFSYS_DCDCCTL0) &
                                      ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_MASK) >>
                                      ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT);

  if (target_trim != current_trim)
    {
      pmctl_regsetctrl = (getreg32(TIVA_AON_PMCTL_RESETCTL) &
                         ~AON_PMCTL_RESETCTL_MCU_WARM_RESET);
      if (pmctl_regsetctrl & AON_PMCTL_RESETCTL_VDDR_LOSS_EN)
        {
          putreg32(pmctl_regsetctrl & ~AON_PMCTL_RESETCTL_VDDR_LOSS_EN,
                  TIVA_AON_PMCTL_RESETCTL);

          /* Wait for VDDR_LOSS_EN setting to propagate */

          getreg32(TIVA_AON_RTC_SYNC);
        }

      while (target_trim != current_trim)
        {
          /* Wait for next edge on SCLK_LF (positive or negative) */

          getreg32(TIVA_AON_RTC_SYNCLF);

          if (target_trim > current_trim)
            {
              current_trim++;
            }
          else
            {
              current_trim--;
            }

          putreg8(((getreg8(TIVA_ADI3_REFSYS_DCDCCTL0) &
                   ~ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_MASK) |
                   ((((uint32_t)current_trim) <<
                     ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT) &
                    ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_MASK)),
                   TIVA_ADI3_REFSYS_DCDCCTL0);
        }

      /* Wait for next edge on SCLK_LF (positive or negative) */

      getreg32(TIVA_AON_RTC_SYNCLF);

      if ((pmctl_regsetctrl & AON_PMCTL_RESETCTL_VDDR_LOSS_EN) != 0)
        {
          /* Wait for next edge on SCLK_LF (positive or negative) */

          getreg32(TIVA_AON_RTC_SYNCLF);

          /* Wait for next edge on SCLK_LF (positive or negative) */

          getreg32(TIVA_AON_RTC_SYNCLF);

          putreg32(pmctl_regsetctrl, TIVA_AON_PMCTL_RESETCTL);

          /* And finally wait for VDDR_LOSS_EN setting to propagate */

          getreg32(TIVA_AON_RTC_SYNC);
        }
    }
}

/****************************************************************************
 * Name: rom_setup_coldreset_from_shutdown_cfg1
 ****************************************************************************/

void rom_setup_coldreset_from_shutdown_cfg1(uint32_t ccfg_modeconf)
{
  uint32_t setbits;
  uint32_t clrbits;

  /* Check for CC1352 boost mode The combination VDDR_EXT_LOAD=0 and
   * VDDS_BOD_LEVEL=1 is defined to select boost mode
   */

  if ((ccfg_modeconf & CCFG_MODE_CONF_VDDR_EXT_LOAD) == 0 &&
      (ccfg_modeconf & CCFG_MODE_CONF_VDDS_BOD_LEVEL) != 0)
    {
      /* Set VDDS_BOD trim - using masked write {MASK8:DATA8} - TRIM_VDDS_BOD
       * is bits[7:3] of ADI3..REFSYSCTL1 - Needs a positive transition on
       * BOD_BG_TRIM_EN (bit[7] of REFSYSCTL3) to latch new VDDS BOD.
       * Set to 0 first to guarantee a positive transition.
       */

      putreg8(ADI3_REFSYS_REFSYSCTL3_BOD_BG_TRIM_EN,
              TIVA_ADI3_REFSYS_CLR + TIVA_ADI3_REFSYS_REFSYSCTL3_OFFSET);

      /* VDDS_BOD_LEVEL = 1 means that boost mode is selected
       * -Max out the VDDS_BOD trim( = VDDS_BOD_POS_31)
       */

      putreg16((ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_MASK << 8) |
               (ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_31),
               TIVA_ADI3_REFSYS_MASK8B +
               (TIVA_ADI3_REFSYS_REFSYSCTL1_OFFSET * 2));

      putreg8(ADI3_REFSYS_REFSYSCTL3_BOD_BG_TRIM_EN,
              TIVA_ADI3_REFSYS_SET + TIVA_ADI3_REFSYS_REFSYSCTL3_OFFSET);

      rom_setup_stepvaddrtrimto((getreg32(TIVA_FCFG1_VOLT_TRIM) &
                                FCFG1_VOLT_TRIM_VDDR_TRIM_HH_MASK) >>
                                FCFG1_VOLT_TRIM_VDDR_TRIM_HH_SHIFT);
    }

  /* 1. Do not allow DCDC to be enabled if in external regulator mode.
   *    Preventing this by setting both the RECHARGE and the ACTIVE bits bit
   *    in the CCFG_MODE_CONF copy register (ccfg_modeconf).
   * 2. Adjusted battery monitor low limit in internal regulator mode. This
   *    is done by setting AON_BATMON_FLASHPUMPP0_LOWLIM=0 in internal
   *    regulator mode.
   */

  if ((getreg32(TIVA_AON_PMCTL_PWRCTL) & AON_PMCTL_PWRCTL_EXT_REG_MODE) != 0)
    {
      ccfg_modeconf |= (CCFG_MODE_CONF_DCDC_RECHARGE |
                        CCFG_MODE_CONF_DCDC_ACTIVE);
    }
  else
    {
      modifyreg32(TIVA_AON_BATMON_FLASHPUMPP0,
                  AON_BATMON_FLASHPUMPP0_LOWLIM, 0);
    }

  /* Set the RECHARGE source based upon CCFG:MODE_CONF:DCDC_RECHARGE Note:
   * Inverse polarity
   */

  setbits = 0;
  clrbits = 0;

  if ((ccfg_modeconf & CCFG_MODE_CONF_DCDC_RECHARGE) != 0)
    {
      clrbits |= AON_PMCTL_PWRCTL_DCDC_EN;
    }
  else
    {
      setbits |= AON_PMCTL_PWRCTL_DCDC_EN;
    }

  /* Set the ACTIVE source based upon CCFG:MODE_CONF:DCDC_ACTIVE
   * Note: Inverse polarity
   */

  if ((ccfg_modeconf & CCFG_MODE_CONF_DCDC_ACTIVE) != 0)
    {
      clrbits |= AON_PMCTL_PWRCTL_DCDC_ACTIVE;
    }
  else
    {
      setbits |= AON_PMCTL_PWRCTL_DCDC_ACTIVE;
    }

  modifyreg32(TIVA_AON_PMCTL_PWRCTL, clrbits, setbits);
}
