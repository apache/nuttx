/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_pktcommon.c
 * Configuration and management of the common features of SPIRIT packets.
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

#include "spirit_pktcommon.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_pktcommon_set_controllen
 *
 * Description:
 *   Sets the CONTROL field length for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   ctrllen - Length of CONTROL field in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_controllen(FAR struct spirit_library_s *spirit,
                                    enum pkt_ctrllen_e ctrllen)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_CONTROL_LENGTH(ctrllen));

  /* Reads the PCKTCTRL4 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set the control length */

      regval &= ~PCKTCTRL4_CONTROL_LEN_MASK;
      regval |= (uint8_t)ctrllen;

      /* Write the new value on the PCKTCTRL4 register */

      ret = spirit_reg_write(spirit, PCKTCTRL4_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_pktcommon_get_controllen
 *
 * Description:
 *   Returns the CONTROL field length for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Control field length.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_controllen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PCKTCTRL4 register value */

  (void)spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);

  /* Rebuild and return value */

  return (regval & PCKTCTRL4_CONTROL_LEN_MASK);
}

/******************************************************************************
 * Name: spirit_pktcommon_enable_crcfilter
 *
 * Description:
 *   Enables or Disables the filtering on CRC.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for CRC_CHECK.  This parameter can be S_ENABLE or
 *              S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_crcfilter(FAR struct spirit_library_s *spirit,
                                      enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PCKT_FLT_OPTIONS register value */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Modify the register value: enable or disable the CRC filtering */

      if (newstate == S_ENABLE)
        {
          regval |= PCKT_FLT_OPTIONS_CRC_CHECK_MASK;
        }
      else
        {
          regval &= ~PCKT_FLT_OPTIONS_CRC_CHECK_MASK;
        }

      /* Writes the PCKT_FLT_OPTIONS register value */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
    }

  return ret;
}
