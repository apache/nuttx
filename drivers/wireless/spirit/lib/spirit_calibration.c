/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_calibration.c
 * Configuration and management of SPIRIT VCO-RCO calibration.
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

#include <stdbool.h>
#include <assert.h>

#include "spirit_calibration.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_calib_enable_rco
 *
 * Description:
 *   Enables or Disables the RCO calibration.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for RCO calibration.  This parameter can be
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_enable_rco(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the register value */

  ret = spirit_reg_read(spirit, PROTOCOL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build new value for the register */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL2_RCO_CALIBRATION_MASK;
        }
      else
        {
          regval &= ~PROTOCOL2_RCO_CALIBRATION_MASK;
        }

      /* Write register to enable or disable the RCO calibration */

      ret = spirit_reg_write(spirit, PROTOCOL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calib_enable_vco
 *
 * Description:
 *   Enables or Disables the VCO calibration.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for VCO calibration. This parameter can be
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_enable_vco(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the register value */

  ret = spirit_reg_read(spirit, PROTOCOL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build new value for the register */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL2_VCO_CALIBRATION_MASK;
        }
      else
        {
          regval &= ~PROTOCOL2_VCO_CALIBRATION_MASK;
        }

      /* Writes register to enable or disable the VCO calibration */

      ret = spirit_reg_write(spirit, PROTOCOL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calibration_set_rcocal
 *
 * Description:
 *   Sets the RCO calibration words.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   rwt    - RWT word for RCO calibration.
 *   rfb    - RFB word for RCO calibration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calibration_set_rcocal(FAR struct spirit_library_s *spirit,
                                  uint8_t rwt, uint8_t rfb)
{
  uint8_t regval[2];
  int ret;

  /* Build the value of RWT and the MSbits of the RFB word */

  regval[0] = (rwt << 4) | (rfb >> 1);

  /* Reads the register value to update the LSbit of RFB */

  ret = spirit_reg_read(spirit, RCO_VCO_CALIBR_IN1_BASE, &regval[1], 1);
  if (ret >= 0)
    {
      /* Build new value for the register */

      regval[1] = (regval[1] & 0x7f) | (rfb << 7);

      /* Write the new value for RCO calibration words */

      ret = spirit_reg_write(spirit, RCO_VCO_CALIBR_IN2_BASE, regval, 2);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calibration_get_rcocal
 *
 * Description:
 *   Returns the RCO calibration words.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   rwt     - Pointer to the variable in which the RWT word has to be
 *             stored.
 *   rfb     - Pointer to the variable in which the RFB word has to be
 *             stored.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calibration_get_rcocal(FAR struct spirit_library_s *spirit,
                                  FAR uint8_t *rwt, FAR uint8_t *rfb)
{
  uint8_t regval[2];
  int ret;

  /* Read the registers values */

  ret = spirit_reg_read(spirit, RCO_VCO_CALIBR_OUT1_BASE, regval, 2);
  if (ret >= 0)
    {
      /* Build the RWT value */

      (*rwt) = regval[0] >> 4;

      /* Build the RFB value */

      (*rfb) = (regval[0] & 0x0F) << 1 | (regval[1] >> 7);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calib_get_vcocal
 *
 * Description:
 *   Returns the VCO calibration data from internal VCO calibrator.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   VCO calibration data byte.
 *
 ******************************************************************************/

uint8_t spirit_calib_get_vcocal(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register value */

  (void)spirit_reg_read(spirit, RCO_VCO_CALIBR_OUT0_BASE, &regval, 1);

  /* Build and return the VCO calibration data value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_calib_set_vcotxcal
 *
 * Description:
 *   Sets the VCO calibration data to be used in TX mode.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   caldata - Calibration data word to be set.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_set_vcotxcal(FAR struct spirit_library_s *spirit,
                               uint8_t caldata)
{
  uint8_t regval;
  int ret;

  /* Reads the register value */

  ret = spirit_reg_read(spirit, RCO_VCO_CALIBR_IN1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be written */

      regval &= 0x80;
      regval |= caldata;

      /* Writes the new value of calibration data in TX */

      ret = spirit_reg_write(spirit, RCO_VCO_CALIBR_IN1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calib_get_vcotxcal
 *
 * Description:
 *   Returns the actual VCO calibration data used in TX mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   VCO calibration data used in TX mode
 *
 ******************************************************************************/

uint8_t spirit_calib_get_vcotxcal(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register containing the calibration data word used in TX mode */

  (void)spirit_reg_read(spirit, RCO_VCO_CALIBR_IN1_BASE, &regval, 1);

  /* Mask the VCO_CALIBR_TX field and returns the value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_calib_set_vcorxcal
 *
 * Description:
 *   Sets the VCO calibration data to be used in RX mode.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   caldata - Calibration data word to be set.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_set_vcorxcal(FAR struct spirit_library_s *spirit,
                              uint8_t caldata)
{
  uint8_t regval;
  int ret;

  /* Reads the register value */

  ret = spirit_reg_read(spirit, RCO_VCO_CALIBR_IN0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be written */

      regval &= 0x80;
      regval |= caldata;

      /* Write the new value of calibration data in RX */

      ret = spirit_reg_write(spirit, RCO_VCO_CALIBR_IN0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calib_get_vcorxcal
 *
 * Description:
 *   Returns the actual VCO calibration data used in RX mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Calibration data word used in RX mode.
 *
 ******************************************************************************/

uint8_t spirit_calib_get_vcorxcal(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register containing the calibration data word used in TX mode */

  (void)spirit_reg_read(spirit, RCO_VCO_CALIBR_IN0_BASE, &regval, 1);

  /* Mask the VCO_CALIBR_RX field and returns the value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_calibration_set_vcowin
 *
 * Description:
 *   Sets the VCO calibration window.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   refword - Value of REFWORD corresponding to the Ref_period according to
 *             the formula:
 *
 *               CALIBRATION_WIN = 11*Ref_period/fxo.
 *
 *            This parameter can be a value of enum spirit_vcowin_e.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calibration_set_vcowin(FAR struct spirit_library_s *spirit,
                                  enum spirit_vcowin_e refword)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_VCO_WIN(refword));

  /* Reads the register value */

  ret = spirit_reg_read(spirit, SYNTH_CONFIG1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the values to be written */

      regval &= 0xfc;
      regval |= refword;

      /* Write the new value of VCO calibration window */

      ret = spirit_reg_write(spirit, SYNTH_CONFIG1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calibration_get_vcowin
 *
 * Description:
 *   Returns the VCO calibration window.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Value of REFWORD corresponding to the Ref_period according to the
 *   formula:
 *
 *     CALIBRATION_WIN = 11*Ref_period/fxo.
 *
 *   This parameter can be a value of enum spirit_vcowin_e.
 *
 ******************************************************************************/

enum spirit_vcowin_e
  spirit_calibration_get_vcowin(FAR struct spirit_library_s *spirit)
{
  uint8_t regval1, regval2;
  enum spirit_vcowin_e refword;

  /* Reads the register containing the REFWORD value */

  (void)spirit_reg_read(spirit, SYNTH_CONFIG1_BASE, &regval1, 1);

  /* Reads the Xtal configuration */

  (void)spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval2, 1);

  /* Mask the REFWORD field */

  regval1 &= 0x03;

  /* Mask the 24_26_MHz_SELECT field */

  regval2 = ((regval2 & 0x40) >> 6);

  /* In case of 26 MHz crystal */

  if (regval2 != 0)
    {
      switch (regval1)
        {
        case 0:
          refword = CALIB_TIME_6_77_US_26MHZ;
          break;

        case 1:
          refword = CALIB_TIME_13_54_US_26MHZ;
          break;

        case 2:
          refword = CALIB_TIME_27_08_US_26MHZ;
          break;

        case 3:
          refword = CALIB_TIME_54_15_US_26MHZ;
          break;
        }
    }

  /* In case of 24 MHz crystal */

  else
    {
      switch (regval1)
        {
        case 0:
          refword = CALIB_TIME_7_33_US_24MHZ;
          break;

        case 1:
          refword = CALIB_TIME_14_67_US_24MHZ;
          break;

        case 2:
          refword = CALIB_TIME_29_33_US_24MHZ;
          break;

        case 3:
          refword = CALIB_TIME_58_67_US_24MHZ;
          break;

        }
    }

  return refword;
}

/******************************************************************************
 * Name: spirit_calib_select_vco
 *
 * Description:
 *   Selects a VCO.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   vco    - Can be VCO_H or VCO_L according to which VCO select.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on any failure.
 *
 ******************************************************************************/

int spirit_calib_select_vco(FAR struct spirit_library_s *spirit,
                            enum spirit_vcoselect_e vco)
{
  uint8_t regval;
  int ret;

  /* Check the parameter */

  DEBUGASSERT(IS_VCO_SEL(vco));

  ret = spirit_reg_read(spirit, SYNTH_CONFIG1_BASE, &regval, 1);
  if (ret >= 0)
    {
      regval &= 0xf9;

      if (vco == VCO_H)
        {
          regval |= 0x02;
        }
      else
        {
          regval |= 0x04;
        }

      ret = spirit_reg_write(spirit, SYNTH_CONFIG1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_calib_get_vco
 *
 * Description:
 *   Returns the VCO selected.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   VCO_H or VCO_L according to which VCO selected.
 *
 ******************************************************************************/

enum spirit_vcoselect_e spirit_calib_get_vco(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  (void)spirit_reg_read(spirit, SYNTH_CONFIG1_BASE, &regval, 1);

  regval = (regval >> 1) & 0x3;
  if (regval == 0x01)
    {
      return VCO_H;
    }
  else
    {
      return VCO_L;
    }
}
