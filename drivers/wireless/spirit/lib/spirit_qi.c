/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_qi.c
 * Configuration and management of SPIRIT QI.
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

#include "spirit_qi.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

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
 * Name: spirit_qi_sqicheck
 *
 * Description:
 *   Enables/Disables the Synchronization Quality Indicator check. The
 *   running peak SQI is compared to a threshold value and the sync valid
 *   IRQ is asserted as soon as the threshold is passed.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - new state for SQI check.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_sqicheck(FAR struct spirit_library_s *spirit,
                       enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the QI register value */

  ret = spirit_reg_read(spirit, QI_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enables or disables the SQI Check bit on the QI_BASE register */

      if (newstate == S_ENABLE)
        {
          regval |= QI_SQI_MASK;
        }
      else
        {
          regval &= ~QI_SQI_MASK;
        }

      /* Write the value to the QI register */

      ret = spirit_reg_write(spirit, QI_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_set_sqithreshold
 *
 * Description:
 *   Sets the SQI threshold. The synchronization quality threshold is equal to
 *   8 * SYNC_LEN - 2 * SQI_TH with SQI_TH = 0..3. When SQI_TH is 0 perfect
 *   match is required; when SQI_TH = 1, 2, 3 then 1, 2, or 3 bit errors are
 *   respectively accepted. It is recommended that the SQI check is always
 *   enabled.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   sqithr - parameter of the formula above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_set_sqithreshold(FAR struct spirit_library_s *spirit,
                               enum spirit_sqi_threshold_e sqithr)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SQI_THR(sqithr));

  /* Reads the QI register value */

  ret = spirit_reg_read(spirit, QI_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the SQI threshold value to be written */

      regval &= 0x3f;
      regval |= (uint8_t)sqithr;

      /* Write the new value to the QI register */

      ret = spirit_reg_write(spirit, QI_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_get_sqithreshold
 *
 * Description:
 *   Returns the SQI threshold. The synchronization quality threshold is equal
 *   to 8 * SYNC_LEN - 2 * SQI_TH with SQI_TH = 0..3.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   SQI threshold (SQI_TH of the formula above).  Errors are not reported.
 *
 ******************************************************************************/

enum spirit_sqi_threshold_e
  spirit_qi_get_sqithreshold(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the QI register value */

  (void)spirit_reg_read(spirit, QI_BASE, &regval, 1);

  /* Return the SQI threshold value */

  return (enum spirit_sqi_threshold_e)(regval & 0xc0);
}

/******************************************************************************
 * Name: spirit_qi_set_rssithreshold
 *
 * Description:
 *   Sets the RSSI threshold from its dBm value according to the formula:
 *   (RSSI[Dbm] + 130)/0.5.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   dbmvalue - RSSI threshold reported in dBm.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_set_rssithreshold(FAR struct spirit_library_s *spirit,
                                int dbmvalue)
{
  uint8_t regval = 2 * (dbmvalue + 130);

  /* Check the parameters */

  DEBUGASSERT(IS_RSSI_THR_DBM(dbmvalue));

  /* Writes the new value on the RSSI_TH register */

  return spirit_reg_write(spirit, RSSI_TH_BASE, &regval, 1);
}
