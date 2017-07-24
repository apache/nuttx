/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_timer.c
 * Configuration and management of SPIRIT timers.
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <assert.h>

#include "spirit_timer.h"
#include "spirit_radio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Returns the absolute value. */

#define S_ABS(a) ((a) > 0 ? (a) : -(a))

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name:
 *
 * Description:
 *
 * Parameters:
 *
 * Returned Value:
 *
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_timer_set_rxtimeout
 *
 * Description:
 *   Sets the RX timeout timer counter.  If 'counter' is equal to 0 the
 *   timeout is disabled.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   counter - value for the timer counter.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_rxtimeout(FAR struct spirit_library_s *spirit,
                               uint8_t counter)
{
  /* Writes the counter value for RX timeout in the corresponding register */

  return spirit_reg_write(spirit, TIMERS4_RX_TIMEOUT_COUNTER_BASE, &counter, 1);
}

/******************************************************************************
 * Name: spirit_timer_set_rxtimeout_stopcondition
 *
 * Description:
 *   Sets the RX timeout stop conditions.
 *
 * Input Parameters:
 *   spirit        - Reference to a Spirit library state structure instance
 *   stopcondition - New stop condition.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_rxtimeout_stopcondition(FAR struct spirit_library_s *spirit,
                                             enum spirit_rxtimeout_stopcondition_e
                                             stopcondition)
{
  uint8_t regval[2];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_RX_TIMEOUT_STOP_CONDITION(stopcondition));

  /* Reads value on the PKT_FLT_OPTIONS and PROTOCOL2 register */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, regval, 2);
  if (ret >= 0)
    {
      regval[0] &= 0xbf;
      regval[0] |= ((stopcondition & 0x08) << 3);

      regval[1] &= 0x1f;
      regval[1] |= (stopcondition << 5);

      /* Write value to the PKT_FLT_OPTIONS and PROTOCOL2 register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, regval, 2);
    }

  return ret;
}
