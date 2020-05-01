/************************************************************************************
 * arch/arm/src/efm32/efm32_rmu.c
 *
 *   Copyright (C) 2015 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_arch.h"

#include "hardware/efm32_emu.h"
#include "hardware/efm32_rmu.h"

#include "efm32_rmu.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

#if defined(CONFIG_EFM32_RMU_DEBUG) && defined(CONFIG_DEBUG_WARN)
typedef struct
{
  const uint32_t val;
  const uint32_t mask;
  const char    *str;
} efm32_reset_cause_list_t;
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#if defined(CONFIG_EFM32_RMU_DEBUG) && defined(CONFIG_DEBUG_WARN)
static efm32_reset_cause_list_t efm32_reset_cause_list[] =
{
  {
    0x0001, /* 0bXXXX XXXX XXXX XXX1 */
    0x0001, /* 0bXXXX XXXX XXXX XXX1 */
    "A Power-on Reset has been performed. X bits are don't care."
  },
  {
    0x0002, /* 0bXXXX XXXX 0XXX XX10 */
    0x0003, /* 0bXXXX XXXX 1XXX XX11 */
    "A Brown-out has been detected on the unregulated power."
  },
  {
    0x0004, /* 0bXXXX XXXX XXX0 0100 */
    0x001F, /* 0bXXXX XXXX XXX1 1111 */
    "A Brown-out has been detected on the regulated power."
  },
  {
    0x0008, /* 0bXXXX XXXX XXXX 1X00 */
    0x000B, /* 0bXXXX XXXX XXXX 1X11 */
    "An external reset has been applied."
  },
  {
    0x0010, /* 0bXXXX XXXX XXX1 XX00 */
    0x0013, /* 0bXXXX XXXX XXX1 XX11 */
    "A watchdog reset has occurred."
  },
  {
    0x0020, /* 0bXXXX X000 0010 0000 */
    0x07FF, /* 0bXXXX X111 1111 1111 */
    "A lockup reset has occurred."
  },
  {
    0x0040, /* 0bXXXX X000 01X0 0000 */
    0x07DF, /* 0bXXXX X111 11X1 1111 */
    "A system request reset has occurred."
  },
  {
    0x0080, /* 0bXXXX X000 1XX0 0XX0 */
    0x0799, /* 0bXXXX X111 1XX1 1XX1 */
    "The system has woken up from EM4."
  },
  {
    0x0180, /* 0bXXXX X001 1XX0 0XX0 */
    0x0799, /* 0bXXXX X111 1XX1 1XX1 */
    "The system has woken up from EM4 on an EM4 wakeup reset request from pin."
  },
  {
    0x0200, /* 0bXXXX X01X XXX0 0000 */
    0x061F, /* 0bXXXX X11X XXX1 1111 */
    "A Brown-out has been detected on Analog Power Domain 0 (AVDD0)."
  },
  {
    0x0400, /* 0bXXXX X10X XXX0 0000 */
    0x061F, /* 0bXXXX X11X XXX1 1111 */
    "A Brown-out has been detected on Analog Power Domain 1 (AVDD1)."
  },
  {
    0x0800, /* 0bXXXX 1XXX XXXX 0XX0 */
    0x0809, /* 0bXXXX 1XXX XXXX 1XX1 */
    "A Brown-out has been detected by the Backup BOD on VDD_DREG."
  },
  {
    0x1000, /* 0bXXX1 XXXX XXXX 0XX0 */
    0x1009, /* 0bXXX1 XXXX XXXX 1XX1 */
    "A Brown-out has been detected by the Backup BOD on BU_VIN."
  },
  {
    0x2000, /* 0bXX1X XXXX XXXX 0XX0 */
    0x2009, /* 0bXX1X XXXX XXXX 1XX1 */
    "A Brown-out has been detected by the Backup BOD on unregulated power"
  },
  {
    0x4000, /* 0bX1XX XXXX XXXX 0XX0 */
    0x4009, /* 0bX1XX XXXX XXXX 1XX1 */
    "A Brown-out has been detected by the Backup BOD on regulated power."
  },
  {
    0x8000, /* 0b1XXX XXXX XXXX XXX0 */
    0x8001, /* 0b1XXX XXXX XXXX XXX1 */
    "The system has been in Backup mode."
  }
};
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* Variable old last reset cause of cpu. */

uint32_t g_efm32_rstcause;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_reset_cause_list_str
 *
 * Description:
 *    Return next reset cause string, NULL if no more reset cause.
 *
 * Input Parameters:
 *   reg: reset cause register to decode (like g_efm32_rstcause)
 *   idx: Use to keep in maind reset cause decoding position.
 *        set *idx to zero before first call.
 *
 ************************************************************************************/

#if defined(CONFIG_EFM32_RMU_DEBUG) && defined(CONFIG_DEBUG_WARN)
const char *efm32_reset_cause_list_str(uint32_t reg, unsigned int *idx)
{
  int len = sizeof(efm32_reset_cause_list)/sizeof(efm32_reset_cause_list[0]);
  efm32_reset_cause_list_t *ptr = NULL;

  do
    {
      if (*idx >= len)
        {
          return NULL;
        }

      ptr = &efm32_reset_cause_list[*idx];
       (*idx)++;
    }
  while ((ptr->mask & reg) != ptr->val);

  if (ptr != NULL)
    {
      return ptr->str;
    }

  return NULL;
}
#endif

/************************************************************************************
 * Name: efm32_rmu_initialize
 *
 * Description:
 *    Store reset cause into g_efm32_rstcause then clear reset cause register.
 *
 ************************************************************************************/

void efm32_rmu_initialize(void)
{
#ifdef CONFIG_EFM32_RMU_DEBUG
  unsigned int idx = 0;
#endif
  uint32_t locked;

  g_efm32_rstcause = getreg32(EFM32_RMU_RSTCAUSE);

  /* Now clear reset cause */

  putreg32(RMU_CMD_RCCLR, EFM32_RMU_CMD);

  /* Clear some reset causes not cleared with RMU CMD register
   * (If EMU registers locked, they must be unlocked first)
   */

  locked = getreg32(EFM32_EMU_LOCK) & EMU_LOCK_LOCKKEY_LOCKED;
  if (locked)
    {
      /* EMU unlock */

      putreg32(EMU_LOCK_LOCKKEY_LOCK, EMU_LOCK_LOCKKEY_UNLOCK);
    }

  modifyreg32(EFM32_EMU_AUXCTRL, 0, EMU_AUXCTRL_HRCCLR);
  modifyreg32(EFM32_EMU_AUXCTRL, EMU_AUXCTRL_HRCCLR, 0);

  if (locked)
    {
      /* EMU lock */

      putreg32(EMU_LOCK_LOCKKEY_LOCK, EMU_LOCK_LOCKKEY_LOCK);
    }

#if defined(CONFIG_EFM32_RMU_DEBUG) && defined(CONFIG_DEBUG_WARN)
  rmuwarn("RMU => reg = 0x%08X\n", g_efm32_rstcause);
  for (; ; )
    {
      const char *str;

      str = efm32_reset_cause_list_str(g_efm32_rstcause, &idx);
      if (str == NULL)
        {
          break;
        }

      rmuwarn("RMU => %s\n", str);
    }
#endif
}
