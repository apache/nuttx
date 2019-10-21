/****************************************************************************
 * drivers/modem/altair/altmdm_pm_state.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <nuttx/irq.h>

#include <nuttx/modem/altmdm.h>
#include "altmdm_pm_state.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_stateofmodem;

#ifdef CONFIG_MODEM_PM_PUTSTATE
static char *g_putstring[MODEM_PM_INTERNAL_STATE_MAX] =
{
  "SLEEP",
  "GOING_TO_WAKE",
  "WAKE",
  "GOING_TO_SLEEP"
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_pm_getstate
 *
 * Description:
 *   Get current modem state.
 *
 ****************************************************************************/

uint32_t altmdm_pm_getstate(void)
{
  uint32_t get_state;
  uint32_t internal_state;

  internal_state = altmdm_pm_getinternalstate();

  switch (internal_state)
    {
    case MODEM_PM_INTERNAL_STATE_SLEEP:
    case MODEM_PM_INTERNAL_STATE_GOING_TO_WAKE:
      get_state = MODEM_PM_STATE_SLEEP;
      break;

    case MODEM_PM_INTERNAL_STATE_WAKE:
    case MODEM_PM_INTERNAL_STATE_GOING_TO_SLEEP:
      get_state = MODEM_PM_STATE_WAKE;
      break;

    default:
      get_state = MODEM_PM_STATE_WAKE;
      break;
    }

  return get_state;
}

/****************************************************************************
 * Name: altmdm_pm_getinternalstate
 *
 * Description:
 *   Get internal modem state.
 *
 ****************************************************************************/

uint32_t altmdm_pm_getinternalstate(void)
{
  uint32_t get_state;
  irqstate_t flags;

  flags = enter_critical_section();

  get_state = g_stateofmodem;

  leave_critical_section(flags);

  return get_state;
}

/****************************************************************************
 * Name: altmdm_pm_setinternalstate
 *
 * Description:
 *   Set internal modem state.
 *
 ****************************************************************************/

void altmdm_pm_setinternalstate(uint32_t state)
{
  irqstate_t flags;
#ifdef CONFIG_MODEM_PM_PUTSTATE
  uint32_t prev_state;
#endif

  if (state < MODEM_PM_INTERNAL_STATE_MAX)
    {
      flags = enter_critical_section();
#ifdef CONFIG_MODEM_PM_PUTSTATE
      prev_state = g_stateofmodem;
#endif
      g_stateofmodem = state;

      leave_critical_section(flags);
#ifdef CONFIG_MODEM_PM_PUTSTATE
      m_err("MODEM State [%d:%s]-->[%d:%s]\n",
            prev_state, g_putstring[prev_state], state, g_putstring[state]);
#endif
    }
}
#endif
