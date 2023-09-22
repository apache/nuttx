/****************************************************************************
 * arch/arm/src/at32/at32_pwr.c
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
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "at32_pwr.h"

#if defined(CONFIG_AT32_PWR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Wakeup Pin Definitions:  See chip/at32_pwr.h */

#undef HAVE_PWR_WKUP2
#undef HAVE_PWR_WKUP3

#if defined(CONFIG_AT32_AT32F43XXX)
#  define HAVE_PWR_WKUP2  1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_bkp_writable_counter = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t at32_pwr_getreg32(uint8_t offset)
{
  return getreg32(AT32_PWC_BASE + (uint32_t)offset);
}

static inline void at32_pwr_putreg32(uint8_t offset, uint32_t value)
{
  putreg32(value, AT32_PWC_BASE + (uint32_t)offset);
}

static inline void at32_pwr_modifyreg32(uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits)
{
  modifyreg32(AT32_PWC_BASE + (uint32_t)offset, clearbits, setbits);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_pwr_initbkp
 *
 * Description:
 *   Insures the referenced count access to the backup domain (RTC registers,
 *   RTC backup data registers and backup SRAM is consistent with the HW
 *   state without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void at32_pwr_initbkp(bool writable)
{
  uint16_t regval;

  /* Make the HW not writable */

  regval = at32_pwr_getreg32(AT32_PWC_CTRL_OFFSET);
  regval &= ~PWC_CTRL_BPWEN;
  at32_pwr_putreg32(AT32_PWC_CTRL_OFFSET, regval);

  /* Make the reference count agree */

  g_bkp_writable_counter =  0;
  at32_pwr_enablebkp(writable);
}

/****************************************************************************
 * Name: at32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 *   NOTE:
 *   Reference counting is used in order to supported nested calls to this
 *   function.  As a consequence, every call to at32_pwr_enablebkp(true)
 *   must be followed by a matching call to at32_pwr_enablebkp(false).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void at32_pwr_enablebkp(bool writable)
{
  irqstate_t flags;
  uint16_t regval;
  bool waswritable;
  bool wait = false;

  flags = enter_critical_section();

  /* Get the current state of the AT32 PWR control register */

  regval      = at32_pwr_getreg32(AT32_PWC_CTRL_OFFSET);
  waswritable = ((regval & PWC_CTRL_BPWEN) != 0);

  if (writable)
    {
      DEBUGASSERT(g_bkp_writable_counter < UINT16_MAX);
      g_bkp_writable_counter++;
    }
  else if (g_bkp_writable_counter > 0)
    {
      g_bkp_writable_counter--;
    }

  /* Enable or disable the ability to write */

  if (waswritable && g_bkp_writable_counter == 0)
    {
      /* Disable backup domain access */

      regval &= ~PWC_CTRL_BPWEN;
      at32_pwr_putreg32(AT32_PWC_CTRL_OFFSET, regval);
    }
  else if (!waswritable && g_bkp_writable_counter > 0)
    {
      /* Enable backup domain access */

      regval |= PWC_CTRL_BPWEN;
      at32_pwr_putreg32(AT32_PWC_CTRL_OFFSET, regval);

      wait = true;
    }

  leave_critical_section(flags);

  if (wait)
    {
      /* Enable does not happen right away */

      up_udelay(4);
    }
}

/****************************************************************************
 * Name: at32_pwr_enablewkup
 *
 * Description:
 *   Enables the WKUP pin.
 *
 * Input Parameters:
 *   wupin - Selects the WKUP pin to enable/disable
 *   wupon - state to set it to
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  The only cause of failure is if the selected MCU does not
 *   support the requested wakeup pin.
 *
 ****************************************************************************/

int at32_pwr_enablewkup(enum at32_pwr_wupin_e wupin, bool wupon)
{
  uint16_t pinmask;

  /* Select the PWR_CSR bit associated with the requested wakeup pin */

  switch (wupin)
    {
      case PWC_WUPIN_1:    /* Wake-up pin 1 (all parts) */
        pinmask = PWC_CTRLSTS_SWPEN1;
        break;

#ifdef HAVE_PWR_WKUP2
      case PWC_WUPIN_2:    /* Wake-up pin 2 */
        pinmask = PWC_CTRLSTS_SWPEN2;
        break;
#endif

#ifdef HAVE_PWR_WKUP3
      case PWC_WUPIN_3:     /* Wake-up pin 3 */
        pinmask = PWR_CSR_EWUP3;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Set/clear the the wakeup pin enable bit in the CSR.  This must be done
   * within a critical section because the CSR is shared with other functions
   * that may be running concurrently on another thread.
   */

  if (wupon)
    {
      /* Enable the wakeup pin by setting the bit in the CSR. */

      at32_pwr_modifyreg32(AT32_PWC_CTRLSTS_OFFSET, 0, pinmask);
    }
  else
    {
      /* Disable the wakeup pin by clearing the bit in the CSR. */

      at32_pwr_modifyreg32(AT32_PWC_CTRLSTS_OFFSET, pinmask, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: at32_pwr_getsbf
 *
 * Description:
 *   Return the standby flag.
 *
 ****************************************************************************/

bool at32_pwr_getsbf(void)
{
  return (at32_pwr_getreg32(AT32_PWC_CTRLSTS_OFFSET) & PWC_CTRLSTS_SEF) != 0;
}

/****************************************************************************
 * Name: at32_pwr_getwuf
 *
 * Description:
 *   Return the wakeup flag.
 *
 ****************************************************************************/

bool at32_pwr_getwuf(void)
{
  return (at32_pwr_getreg32(AT32_PWC_CTRLSTS_OFFSET) \
  & PWC_CTRLSTS_SWEF) != 0;
}

/****************************************************************************
 * Name: at32_pwr_setpvd
 *
 * Description:
 *   Sets power voltage detector
 *
 * Input Parameters:
 *   pls - PVD level
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.
 *   If used for any other purpose that protection to assure that its
 *   operation is atomic will be required.
 *
 ****************************************************************************/

void at32_pwr_setpvd(uint16_t pls)
{
  uint16_t regval;

  /* Set PLS */

  regval = at32_pwr_getreg32(AT32_PWC_CTRL_OFFSET);
  regval &= ~PWC_CTRL_PVMSEL_MASK;
  regval |= (pls & PWC_CTRL_PVMSEL_MASK);

  /* Write value to register */

  at32_pwr_putreg32(AT32_PWC_CTRL_OFFSET, regval);
}

/****************************************************************************
 * Name: at32_pwr_enablepvd
 *
 * Description:
 *   Enable the Programmable Voltage Detector
 *
 ****************************************************************************/

void at32_pwr_enablepvd(void)
{
  /* Enable PVD by setting the PVDE bit in PWR_CR register. */

  at32_pwr_modifyreg32(AT32_PWC_CTRL_OFFSET, 0, PWC_CTRL_PVMEN);
}

/****************************************************************************
 * Name: at32_pwr_disablepvd
 *
 * Description:
 *   Disable the Programmable Voltage Detector
 *
 ****************************************************************************/

void at32_pwr_disablepvd(void)
{
  /* Disable PVD by clearing the PVDE bit in PWR_CR register. */

  at32_pwr_modifyreg32(AT32_PWC_CTRL_OFFSET, PWC_CTRL_PVMEN, 0);
}

#endif /* CONFIG_AT32_PWR */
