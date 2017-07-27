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
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_qi_enable_pqicheck
 *
 * Description:
 *   Enables/Disables the PQI Preamble Quality Indicator check. The running
 *   peak PQI is compared to a threshold value and the preamble valid IRQ is
 *   asserted as soon as the threshold is passed.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for PQI check.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_enable_pqicheck(FAR struct spirit_library_s *spirit,
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
      /* Enables or disables the PQI Check bit on the QI_BASE register */

      if (newstate == S_ENABLE)
        {
          regval |= QI_PQI_MASK;
        }
      else
        {
          regval &= ~QI_PQI_MASK;
        }

      /* Write the value on the QI register */

      ret = spirit_reg_write(spirit, QI_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_enable_sqicheck
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

int spirit_qi_enable_sqicheck(FAR struct spirit_library_s *spirit,
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
 * Name: spirit_qi_set_pqithreshold
 *
 * Description:
 *   Sets the PQI threshold. The preamble quality threshold is 4*PQI_TH
 *   (PQI_TH = 0..15).
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   pqithr - Parameter of the formula above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_set_pqithreshold(FAR struct spirit_library_s *spirit,
                                enum spirit_pqi_threshold_e pqithr)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PQI_THR(pqithr));

  /* Reads the QI register value */

  ret = spirit_reg_read(spirit, QI_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the PQI threshold value to be written */

      regval &= 0xc3;
      regval |= (uint8_t)pqithr;

     /* Write the value on the QI register */

      ret = spirit_reg_write(spirit, QI_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_get_pqithreshold
 *
 * Description:
 *   Returns the PQI threshold. The preamble quality threshold is 4*PQI_TH
 *   (PQI_TH = 0..15).
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   PQI threshold (PQI_TH of the formula above).
 *
 ******************************************************************************/

enum spirit_pqi_threshold_e
  spirit_qi_get_pqithreshold(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the QI register value */

  (void)spirit_reg_read(spirit, QI_BASE, &regval, 1);

  /* Rebuild and return the PQI threshold value */

  return (enum spirit_pqi_threshold_e)(regval & 0x3c);
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
 * Name: spirit_qi_get_pqi
 *
 * Description:
 *   Returns the PQI value.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   PQI value.
 *
 ******************************************************************************/

uint8_t spirit_qi_get_pqi(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the LINK_QUALIF2 register value */

  (void)spirit_reg_read(spirit, LINK_QUALIF2_BASE, &regval, 1);

  /* Returns the PQI value */

  return regval;
}

/******************************************************************************
 * Name: spirit_qi_get_sqi
 *
 * Description:
 *   Returns the SQI value.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   SQI value.
 *
 ******************************************************************************/

uint8_t spirit_qi_get_sqi(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register LINK_QUALIF1 value */

  (void)spirit_reg_read(spirit, LINK_QUALIF1_BASE, &regval, 1);

  /* Rebuild and return the SQI value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_qi_get_lqi
 *
 * Description:
 *   Returns the LQI value.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   LQI value.
 *
 ******************************************************************************/

uint8_t spirit_qi_get_lqi(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the LINK_QUALIF0 register value */

  (void)spirit_reg_read(spirit, LINK_QUALIF0_BASE, &regval, 1);

  /* Rebuild and return the LQI value */

  return ((regval & 0xf0) >> 4);
}

/******************************************************************************
 * Name: spirit_qi_get_cs
 *
 * Description:
 *   Returns the CS status.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CS value (S_SET or S_RESET).
 *
 ******************************************************************************/

enum spirit_flag_status_e spirit_qi_get_cs(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the LINK_QUALIF1 register value */

  (void)spirit_reg_read(spirit, LINK_QUALIF1_BASE, &regval, 1);

  /* Rebuild and returns the CS status value */

  if ((regval & 0x80) == 0)
    {
      return S_RESET;
    }
  else
    {
      return S_SET;
    }
}

/******************************************************************************
 * Name: spirit_qi_get_rssi
 *
 * Description:
 *   Returns the RSSI value. The measured power is reported in steps of half a
 *   dB from 0 to 255 and is offset in such a way that -120 dBm corresponds to
 *   20.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   RSSI value.
 *
 ******************************************************************************/

uint8_t spirit_qi_get_rssi(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the RSSI_LEVEL register value */

  (void)spirit_reg_read(spirit, RSSI_LEVEL_BASE, &regval, 1);

  /* Returns the RSSI value */

  return regval;
}

/******************************************************************************
 * Name: spirit_qi_set_rssithreshold
 *
 * Description:
 *   Sets the RSSI threshold.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   rssithr - RSSI threshold reported in steps of half a dBm with a -130
 *             dBm offset.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_set_rssithreshold(FAR struct spirit_library_s *spirit,
                                uint8_t rssithr)
{
  /* Writes the new value on the RSSI_TH register */

  return spirit_reg_write(spirit, RSSI_TH_BASE, &rssithr, 1);
}

/******************************************************************************
 * Name: spirit_qi_get_rssithreshold
 *
 * Description:
 *   Returns the RSSI threshold.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   RSSI threshold.
 *
 ******************************************************************************/

uint8_t spirit_qi_get_rssithreshold(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the RSSI_TH register value */

  (void)spirit_reg_read(spirit, RSSI_TH_BASE, &regval, 1);

  /* Returns RSSI threshold */

  return regval;
}

/******************************************************************************
 * Name: spirit_qi_calc_rssithreshold
 *
 * Description:
 *   Computes the RSSI threshold from its dBm value according to the formula:
 *   (RSSI[Dbm] + 130)/0.5
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   dbmvalue - RSSI threshold reported in dBm.
 *
 * Returned Value:
 *   RSSI threshold corresponding to dBm value.
 *
 ******************************************************************************/

uint8_t spirit_qi_calc_rssithreshold(FAR struct spirit_library_s *spirit,
                                    int dbmvalue)
{
  /* Check the parameters */

  DEBUGASSERT(IS_RSSI_THR_DBM(dbmvalue));

  /* Computes the RSSI threshold for register */

  return 2 * (dbmvalue + 130);
}

/******************************************************************************
 * Name: spirit_qi_set_rssithreshold_dbm
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

int spirit_qi_set_rssithreshold_dbm(FAR struct spirit_library_s *spirit,
                                    int dbmvalue)
{
  uint8_t regval = 2 * (dbmvalue + 130);

  /* Check the parameters */

  DEBUGASSERT(IS_RSSI_THR_DBM(dbmvalue));

  /* Writes the new value on the RSSI_TH register */

  return spirit_reg_write(spirit, RSSI_TH_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_qi_set_rssifiltergain
 *
 * Description:
 *   Sets the RSSI filter gain. This parameter sets the bandwidth of a low
 *   pass IIR filter (RSSI_FLT register, allowed values 0..15), a lower values
 *   gives a faster settling of the measurements but lower precision. The
 *   recommended value for such parameter is 14.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   rssfg  -  RSSI filter gain value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_set_rssifiltergain(FAR struct spirit_library_s *spirit,
                                 enum spirit_rssi_filtergain_e rssfg)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_RSSI_FILTER_GAIN(rssfg));

  /* Reads the RSSI_FLT register */

  ret = spirit_reg_read(spirit, RSSI_FLT_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Sets the specified filter gain */

      regval &= 0x0f;
      regval |= ((uint8_t) rssfg);

      /* Write the new value to the RSSI_FLT register */

      ret = spirit_reg_write(spirit, RSSI_FLT_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_get_rssifiltergain
 *
 * Description:
 *   Returns the RSSI filter gain.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   RSSI filter gain.
 *
 ******************************************************************************/

enum spirit_rssi_filtergain_e
  spirit_qi_get_rssifiltergain(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the RSSI_FLT register */

  (void)spirit_reg_read(spirit, RSSI_FLT_BASE, &regval, 1);

  /* Rebuild and returns the filter gain value */

  return (enum spirit_rssi_filtergain_e)(regval & 0xf0);
}

/******************************************************************************
 * Name: spirit_qi_set_csmode
 *
 * Description:
 *   Sets the CS Mode. When static carrier sensing is used (cs_mode = 0), the
 *   carrier sense signal is asserted when the measured RSSI is above the
 *   value specified in the RSSI_TH register and is deasserted when the RSSI
 *   falls 3 dB below the same threshold.  When dynamic carrier sense is used
 *   (cs_mode = 1, 2, 3), the carrier sense signal is asserted if the signal
 *   is above the threshold and a fast power increase of 6, 12 or 18 dB is
 *   detected; it is deasserted if a power fall of the same amplitude is
 *   detected.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   csmode - CS mode selector.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_set_csmode(FAR struct spirit_library_s *spirit,
                         enum spirit_csmode_e csmode)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_CS_MODE(csmode));

  /* Reads the RSSI_FLT register */

  ret = spirit_reg_read(spirit, RSSI_FLT_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Sets bit to select the CS mode */

      regval &= ~0x0c;
      regval |= (uint8_t)csmode;

      /* Writesthe new value to the RSSI_FLT register */

      ret = spirit_reg_write(spirit, RSSI_FLT_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_get_csmode
 *
 * Description:
 *   Returns the CS Mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CS mode.
 *
 ******************************************************************************/

enum spirit_csmode_e spirit_qi_get_csmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the RSSI_FLT register */

  (void)spirit_reg_read(spirit, RSSI_FLT_BASE, &regval, 1);

  /* Rebuild and returns the CS mode value */

  return (enum spirit_csmode_e) (regval & 0x0c);
}

/******************************************************************************
 * Name: spirit_qi_enable_cstimeout
 *
 * Description:
 *   Enables/Disables the CS Timeout Mask. If enabled CS value contributes to
 *   timeout disabling.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for CS Timeout Mask.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_enable_cstimeout(FAR struct spirit_library_s *spirit,
                               enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL2 register value */

  ret = spirit_reg_read(spirit, PROTOCOL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enables or disables the CS timeout mask */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL2_CS_TIMEOUT_MASK;
        }
      else
        {
          regval &= ~PROTOCOL2_CS_TIMEOUT_MASK;
        }

      /* Write the new value to the PROTOCOL2 register */

      ret = spirit_reg_write(spirit, PROTOCOL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_enable_pqitimeout
 *
 * Description:
 *   Enables/Disables the PQI Timeout Mask. If enabled PQI value contributes
 *   to timeout disabling.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for PQI Timeout Mask.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_enable_pqitimeout(FAR struct spirit_library_s *spirit,
                                enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL2 register */

  ret = spirit_reg_read(spirit, PROTOCOL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enables or disables the PQI timeout mask */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL2_PQI_TIMEOUT_MASK;
        }
      else
        {
          regval &= ~PROTOCOL2_PQI_TIMEOUT_MASK;
        }

      /* Write the new value to the PROTOCOL2 register */

      ret = spirit_reg_write(spirit, PROTOCOL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_qi_enable_sqitimeout
 *
 * Description:
 *   Enables/Disables the SQI Timeout Mask. If enabled SQI value contributes
 *   to timeout disabling.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for SQI Timeout Mask.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_qi_enable_sqitimeout(FAR struct spirit_library_s *spirit,
                                enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL2 register */

  ret = spirit_reg_read(spirit, PROTOCOL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enables or disables the SQI timeout mask */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL2_SQI_TIMEOUT_MASK;
        }
      else
        {
          regval &= ~PROTOCOL2_SQI_TIMEOUT_MASK;
        }

      /* Write the new value to the PROTOCOL2 register */

      ret = spirit_reg_write(spirit, PROTOCOL2_BASE, &regval, 1);
    }

  return ret;
}
