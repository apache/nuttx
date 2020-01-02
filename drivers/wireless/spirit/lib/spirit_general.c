/******************************************************************************
 * drivers/wireless/spirit/spirit_general.c
 * Configuration and management of SPIRIT General functionalities.
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

#include "spirit_general.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_general_enable_batterylevel
 *
 * Description:
 *   Enables or Disables the output of battery level detector.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for battery level detector.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_general_enable_batterylevel(FAR struct spirit_library_s *spirit,
                                       enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the ANA_FUNC_CONF0_BASE register value */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be stored */

      if (newstate == S_ENABLE)
        {
          regval |= BATTERY_LEVEL_MASK;
        }
      else
        {
          regval &= ~BATTERY_LEVEL_MASK;
        }

      /* Write the new value */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_general_set_batterylevel
 *
 * Description:
 *   Sets the battery level.
 *
 * Input Parameters:
 *   spirit       - Reference to a Spirit library state structure instance
 *   batterylevel - New state for battery level.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_general_set_batterylevel(FAR struct spirit_library_s *spirit,
                                    enum battery_level_e batterylevel)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_BLD_LVL(batterylevel));

  /* Reads the ANA_FUNC_CONF1_BASE register value */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be stored */

      regval &= ~ANA_FUNC_CONF1_SET_BLD_LVL_MASK;
      switch (batterylevel)
        {
        case BLD_LVL_2_7_V:
          regval |= BLD_LVL_2_7;
          break;

        case BLD_LVL_2_5_V:
          regval |= BLD_LVL_2_5;
          break;

        case BLD_LVL_2_3_V:
          regval |= BLD_LVL_2_3;
          break;

        case BLD_LVL_2_1_V:
          regval |= BLD_LVL_2_1;
          break;
        }

      /* Write the new value */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_general_get_batterylevel
 *
 * Description:
 *   Returns the settled battery level.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled battery level.
 *
 ******************************************************************************/

enum battery_level_e
  spirit_general_get_batterylevel(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the ANA_FUNC_CONF1_BASE register value */

  spirit_reg_read(spirit, ANA_FUNC_CONF1_BASE, &regval, 1);

  /* Mask the battery level field and returns the settled battery level */

  return ((enum battery_level_e)(regval & ANA_FUNC_CONF1_SET_BLD_LVL_MASK));
}

/******************************************************************************
 * Name: spirit_general_enable_brownout
 *
 * Description:
 *   Enables or Disables the output of brown out detector.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for brown out detector.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_general_enable_brownout(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the ANA_FUNC_CONF0_BASE register value */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be stored */

      if (newstate == S_ENABLE)
        {
          regval |= BROWN_OUT_MASK;
        }
      else
        {
          regval &= ~BROWN_OUT_MASK;
        }

      /* Write the value to the register */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_general_enable_highpower
 *
 * Description:
 *   Sets High Power Mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for High Power Mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_general_enable_highpower(FAR struct spirit_library_s *spirit,
                                    enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the ANA_FUNC_CONF0_BASE register value */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to write */

      if (newstate == S_ENABLE)
        {
          regval |= HIGH_POWER_MODE_MASK;
        }
      else
        {
          regval &= ~HIGH_POWER_MODE_MASK;
        }

      /* Write the new value to the register */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_general_set_extref
 *
 * Description:
 *   Sets External Reference.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   extmode - New state for the external reference.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_general_set_extref(FAR struct spirit_library_s *spirit,
                              enum mode_extref_e extmode)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_MODE_EXT(extmode));

  /* Reads the ANA_FUNC_CONF0_BASE register value */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to write */

      if (extmode == MODE_EXT_XO)
        {
          regval &= ~EXT_REF_MASK;
        }
      else
        {
          regval |= EXT_REF_MASK;
        }

      /* Write the value to the register */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_general_get_extref
 *
 * Description:
 *   Returns External Reference.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled external reference.
 *
 ******************************************************************************/

enum mode_extref_e
  spirit_general_get_extref(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the ANA_FUNC_CONF0_BASE register value and return the result */

  spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);

  /* Mask the EXT_REF field field and returns the settled reference signal */

  return ((enum mode_extref_e) ((regval & 0x10) >> 4));
}

/******************************************************************************
 * Name: spirit_general_set_xogm
 *
 * Description:
 *   Sets XO gm at startup.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   gm     - Transconductance value of XO at startup.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_general_set_xogm(FAR struct spirit_library_s *spirit,
                            enum gm_conf_e gm)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_GM_CONF(gm));

  /* Reads the ANA_FUNC_CONF1_BASE register value */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to write */

      regval &= ~ANA_FUNC_CONF1_GMCONF_MASK;
      switch (gm)
        {
        case GM_SU_13_2:
          regval |= GM_13_2;
          break;

        case GM_SU_18_2:
          regval |= GM_18_2;
          break;

        case GM_SU_21_5:
          regval |= GM_21_5;
          break;

        case GM_SU_25_6:
          regval |= GM_25_6;
          break;

        case GM_SU_28_8:
          regval |= GM_28_8;
          break;

        case GM_SU_33_9:
          regval |= GM_33_9;
          break;

        case GM_SU_38_5:
          regval |= GM_38_5;
          break;

        case GM_SU_43_0:
          regval |= GM_43_0;
          break;
        }

      /* Write the new value to the register */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_general_get_xogm
 *
 * Description:
 *   Returns the configured XO gm at startup.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled XO gm
 *
 ******************************************************************************/

enum gm_conf_e spirit_general_get_xogm(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the ANA_FUNC_CONF1_BASE register value */

  spirit_reg_read(spirit, ANA_FUNC_CONF1_BASE, &regval, 1);

  /* Mask the GM_CONF field field and returns the settled transconductance of
   * the XO at startup.
   */

  return ((enum gm_conf_e) ((regval & 0x1c) >> 2));
}

/******************************************************************************
 * Name: spirit_general_get_pkttype
 *
 * Description:
 *   Returns the settled packet format.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled packet type.
 *
 ******************************************************************************/

enum packet_type_e
  spirit_general_get_pkttype(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PROTOCOL1 register */

  spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);

  /* cast and return value */

  return (enum packet_type_e)(regval >> 6);
}

/******************************************************************************
 * Name: spirit_general_get_partnumber
 *
 * Description:
 *   Returns device part number.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Device part number.
 *
 ******************************************************************************/

uint16_t spirit_general_get_partnumber(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];

  /* Reads the register value containing the device part number */

  spirit_reg_read(spirit, DEVICE_INFO1_PARTNUM, regval, 2);

  return (((uint16_t)regval[0] << 8) | (uint16_t)regval[1]);
}

/******************************************************************************
 * Name: spirit_general_get_version
 *
 * Description:
 *   Returns SPIRIT RF board version.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   SPIRIT RF board version: 0x30 is the only admitted value
 *
 ******************************************************************************/

uint8_t spirit_general_get_version(FAR struct spirit_library_s *spirit)
{
  uint8_t ver;

  spirit_reg_read(spirit, DEVICE_INFO0_VERSION, &ver, 1);
  return ver;
}
