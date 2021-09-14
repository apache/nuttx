/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_csma.c
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include "spirit_csma.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_csma_initialize
 *
 * Description:
 *   Initializes the SPIRIT CSMA according to the specified parameters in the
 *   struct spirit_csma_init_s.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   csmainit - Reference to the Csma init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_initialize(FAR struct spirit_library_s *spirit,
                           FAR const struct spirit_csma_init_s *csmainit)
{
  uint8_t regval[5];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(csmainit->csmapersistent));
  DEBUGASSERT(IS_CCA_PERIOD(csmainit->multbit));
  DEBUGASSERT(IS_CSMA_LENGTH(csmainit->ccalen));
  DEBUGASSERT(IS_BU_COUNTER_SEED(csmainit->seed));
  DEBUGASSERT(IS_BU_PRESCALER(csmainit->prescaler));
  DEBUGASSERT(IS_CMAX_NB(csmainit->maxnb));

  /* CSMA BU counter seed (MSB) config */

  regval[0] = (uint8_t)(csmainit->seed >> 8);

  /* CSMA BU counter seed (LSB) config */

  regval[1] = (uint8_t)csmainit->seed;

  /* CSMA BU prescaler config and CCA period config */

  regval[2] = (csmainit->prescaler << 2) | csmainit->multbit;

  /* CSMA CCA length config and max number of back-off */

  regval[3] = (csmainit->ccalen | csmainit->maxnb);

  /* Reads the PROTOCOL1_BASE register value, to write the SEED_RELOAD and
   * CSMA_PERS_ON fields/
   */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval[4], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Writes the new value for persistent mode */

  if (csmainit->csmapersistent == S_ENABLE)
    {
      regval[4] |= PROTOCOL1_CSMA_PERS_ON_MASK;
    }
  else
    {
      regval[4] &= ~PROTOCOL1_CSMA_PERS_ON_MASK;
    }

  /* Writes PROTOCOL1_BASE register */

  ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval[4], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Writes CSMA_CONFIGx_BASE registers */

  ret = spirit_reg_write(spirit, CSMA_CONFIG3_BASE, regval, 4);
  return ret;
}

/******************************************************************************
 * Name: spirit_csma_getinfo
 *
 * Description:
 *   Returns the fitted structure struct spirit_csma_init_s starting from the
 *   registers values.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   csmainit - Csma structure to be fitted.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_getinfo(FAR struct spirit_library_s *spirit,
                        FAR struct spirit_csma_init_s *csmainit)
{
  uint8_t regval[5];
  int ret;

  /* Reads PROTOCOL1_BASE register */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval[4], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Reads CSMA_CONFIGx_BASE registers */

  ret = spirit_reg_read(spirit, CSMA_CONFIG3_BASE, regval, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Reads the bu counter seed */

  csmainit->seed = (uint16_t)regval[1] | (uint16_t)regval[0] << 8;

  /* Reads the bu prescaler */

  csmainit->prescaler = regval[2] >> 2;

  /* Reads the Cca period */

  csmainit->multbit = (enum spirit_cca_period_e) (regval[2] & 0x03);

  /* Reads the Cca length */

  csmainit->ccalen = (enum spirit_csmalen_e) (regval[3] & 0xf0);

  /* Reads the max number of back off */

  csmainit->maxnb = regval[3] & 0x07;

  /* Reads the persistent mode enable bit */

  csmainit->csmapersistent =
    (enum spirit_functional_state_e)((regval[4] >> 1) & 0x01);
  return OK;
}

/******************************************************************************
 * Name: spirit_csma_enable
 *
 * Description:
 *   Enables or Disables the CSMA.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - The state of the CSMA mode.  This parameter can be: S_ENABLE
 *              or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_enable(FAR struct spirit_library_s *spirit,
                       enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL1 register value */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Sets or resets the CSMA enable bit */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL1_CSMA_ON_MASK;
        }
      else
        {
          regval &= ~PROTOCOL1_CSMA_ON_MASK;
        }

      /* Write the new value on the PROTOCOL1 register */

      ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_csma_getstate
 *
 * Description:
 *   Gets the CSMA mode. Says if it is enabled or disabled.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CSMA mode.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_csma_getstate(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PROTOCOL1 register value */

  spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);

  /* Return if set or reset */

  if (regval & PROTOCOL1_CSMA_ON_MASK)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

/******************************************************************************
 * Name: spirit_csma_set_persistentmode
 *
 * Description:
 *   Enables or Disables the persistent CSMA mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - The state of the persistent CSMA mode.  This parameter can
 *              be: S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_persistentmode(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL1 register value */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enables/disables the CSMA persistent mode */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL1_CSMA_PERS_ON_MASK;
        }
      else
        {
          regval &= ~PROTOCOL1_CSMA_PERS_ON_MASK;
        }

      /* Write the new value on the PROTOCOL1 register */

      ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_csma_get_persistentmode
 *
 * Description:
 *   Gets the persistent CSMA mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CSMA persistent mode.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_csma_get_persistentmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PROTOCOL1 register value */

  spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);

  /* Return if set or reset */

  if (regval & PROTOCOL1_CSMA_PERS_ON_MASK)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

/******************************************************************************
 * Name: spirit_csma_set_seedreload
 *
 * Description:
 *   Enables or Disables the seed reload mode (if enabled it reloads the
 *   back- off generator seed using the value written in the BU_COUNTER_SEED
 *   register).
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - The state of the seed reload mode.  This parameter can be:
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_seedreload(FAR struct spirit_library_s *spirit,
                               enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL1 register value */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enables/disables the seed reload mode */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL1_SEED_RELOAD_MASK;
        }
      else
        {
          regval &= ~PROTOCOL1_SEED_RELOAD_MASK;
        }

      /* Write the new value on the PROTOCOL1 register */

      ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_csma_get_seedreload
 *
 * Description:
 *   Gets the seed reload state.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CSMA seed reload mode.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_csma_get_seedreload(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PROTOCOL1 register value */

  spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);

  /* Return if set or reset */

  if (regval & PROTOCOL1_SEED_RELOAD_MASK)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

/******************************************************************************
 * Name: spirit_csma_set_bucounterseed
 *
 * Description:
 *   Sets the BU counter seed (BU_COUNTER_SEED register). The CSMA back off
 *   time is given by the formula: BO = rand(2^NB)*BU.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   seed   - Seed of the random number generator used to apply the BBE
 *            algorithm.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_bucounterseed(FAR struct spirit_library_s *spirit,
                                  uint16_t seed)
{
  uint8_t regval[2];

  /* Check parameters */

  DEBUGASSERT(IS_BU_COUNTER_SEED(seed));

  /* Build value (MSB) */

  regval[0] = (uint8_t) (seed >> 8);

  /* Build value (LSB) */

  regval[1] = (uint8_t) seed;

  /* Writes the CSMA_CONFIG3 registers */

  return spirit_reg_write(spirit, CSMA_CONFIG3_BASE, regval, 2);
}

/******************************************************************************
 * Name: spirit_csma_get_bucounterseed
 *
 * Description:
 *   Returns the BU counter seed (BU_COUNTER_SEED register).
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Seed of the random number generator used to apply the BBE algorithm.
 *
 ******************************************************************************/

uint16_t spirit_csma_get_bucounterseed(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];

  /* Reads the CSMA_CONFIGx registers value */

  spirit_reg_read(spirit, CSMA_CONFIG3_BASE, regval, 2);

  /* Build the counter seed and return it */

  return ((uint16_t) regval[1] + (((uint16_t) regval[0]) << 8));
}

/******************************************************************************
 * Name: spirit_csma_set_buprescaler
 *
 * Description:
 *   Sets the BU prescaler. The CSMA back off time is given by the formula:
 *   BO = rand(2^NB)*BU.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   prescaler - Used to program the back-off unit BU.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_buprescaler(FAR struct spirit_library_s *spirit,
                                uint8_t prescaler)
{
  uint8_t regval;
  int ret;

  /* Check parameters */

  DEBUGASSERT(IS_BU_PRESCALER(prescaler));

  /* Reads the CSMA_CONFIG1 register value */

  ret = spirit_reg_read(spirit, CSMA_CONFIG1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the new value for the BU prescaler */

      regval &= 0x03;
      regval |= (prescaler << 2);

      /* Write the new value on the CSMA_CONFIG1_BASE register */

      ret = spirit_reg_write(spirit, CSMA_CONFIG1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_csma_get_buprescaler
 *
 * Description:
 *   Returns the BU prescaler.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Value back-off unit (BU).
 *
 ******************************************************************************/

uint8_t spirit_csma_get_buprescaler(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the CSMA_CONFIG1 register value */

  spirit_reg_read(spirit, CSMA_CONFIG1_BASE, &regval, 1);

  /* Build and return the BU prescaler value */

  return (regval >> 2);
}

/******************************************************************************
 * Name: spirit_csma_set_ccaperiod
 *
 * Description:
 *   Sets the CCA period.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   multbit - Value of CCA period to store.  This parameter can be a value
 *             of enum spirit_cca_period_e.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_ccaperiod(FAR struct spirit_library_s *spirit,
                              enum spirit_cca_period_e multbit)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_CCA_PERIOD(multbit));

  /* Reads the CSMA_CONFIG1 register value */

  ret = spirit_reg_read(spirit, CSMA_CONFIG1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the new value setting the the CCA period */

      regval &= 0xfc;
      regval |= multbit;

      /* Write the new value on the CSMA_CONFIG1 register */

      ret = spirit_reg_write(spirit, CSMA_CONFIG1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_csma_get_ccaperiod
 *
 * Description:
 *   Returns the CCA period.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CCA period.
 *
 ******************************************************************************/

enum spirit_cca_period_e
  spirit_csma_get_ccaperiod(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the CSMA_CONFIG1 register value */

  spirit_reg_read(spirit, CSMA_CONFIG1_BASE, &regval, 1);

  /* Build and return the CCA period value */

  return (enum spirit_cca_period_e) (regval & 0x03);
}

/******************************************************************************
 * Name: spirit_csma_set_ccalen
 *
 * Description:
 *   Sets the CCA length.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ccalen - The CCA length (a value between 1 and 15 that multiplies the
 *            CCA period).  This parameter can be any value of enum
 *            spirit_csmalen_e.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_ccalen(FAR struct spirit_library_s *spirit,
                           enum spirit_csmalen_e ccalen)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_CSMA_LENGTH(ccalen));

  /* Reads the CSMA_CONFIG0 register value */

  ret = spirit_reg_read(spirit, CSMA_CONFIG0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value of CCA length to be set */

      regval &= 0x0f;
      regval |= ccalen;

      /* Write the new value on the CSMA_CONFIG0 register */

      ret = spirit_reg_write(spirit, CSMA_CONFIG0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_csma_get_ccalen
 *
 * Description:
 *   Returns the CCA length.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CCA length.
 *
 ******************************************************************************/

uint8_t spirit_csma_get_ccalen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the CSMA_CONFIG0 register value */

  spirit_reg_read(spirit, CSMA_CONFIG0_BASE, &regval, 1);

  /* Build and return the CCA length */

  return regval >> 4;
}

/******************************************************************************
 * Name: spirit_csma_set_maxbackoff
 *
 * Description:
 *   Sets the max number of back-off. If reached Spirit stops the transmission.
 *
 * Input Parameters:
 *   maxnb the max number of back-off.
 *         This parameter is an uint8_t.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_set_maxbackoff(FAR struct spirit_library_s *spirit,
                               uint8_t maxnb)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_CMAX_NB(maxnb));

  /* Reads the CSMA_CONFIG0 register value */

  ret = spirit_reg_read(spirit, CSMA_CONFIG0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value of max back off to be set */

      regval &= 0xf8;
      regval |= maxnb;

      /* Write the new value on the CSMA_CONFIG0 register */

      ret = spirit_reg_write(spirit, CSMA_CONFIG0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_csma_get_maxbackoff
 *
 * Description:
 *   Returns the max number of back-off.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Max number of back-off.
 *
 ******************************************************************************/

uint8_t spirit_csma_get_maxbackoff(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the CSMA_CONFIG0 register value */

  spirit_reg_read(spirit, CSMA_CONFIG0_BASE, &regval, 1);

  /* Build and return the max number of back-off */

  return (regval & 0x07);
}
