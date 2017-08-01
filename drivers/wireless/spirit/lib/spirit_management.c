/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_management.c
 * The management layer for SPIRIT1 library.
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#include "spirit_commands.h"
#include "spirit_spi.h"
#include "spirit_radio.h"
#include "spirit_calibration.h"
#include "spirit_management.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#define COMMUNICATION_STATE_TX          0
#define COMMUNICATION_STATE_RX          1
#define COMMUNICATION_STATE_NONE        2

/******************************************************************************
 * Private Data
 ******************************************************************************/

/* Factor is: B/2 used in the formula for SYNTH word calculation */

static const uint8_t g_vectc_bhalf[4] =
{
  (HIGH_BAND_FACTOR / 2),
  (MIDDLE_BAND_FACTOR / 2),
  (LOW_BAND_FACTOR / 2),
  (VERY_LOW_BAND_FACTOR / 2)
};

/* BS value to write in the SYNT0 register according to the selected band */

static const uint8_t g_vectc_bandreg[4] =
{
  SYNT0_BS_6, SYNT0_BS_12, SYNT0_BS_16, SYNT0_BS_32
};

/* BS value to write in the SYNT0 register according to the selected band */

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_management_set_basefrequency
 *
 * Description:
 *   spirit_management_set_basefrequency function only used in
 *   spirit_managment_wavco_calibration.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fbase  - the base carrier frequency expressed in Hz as unsigned word.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

static int
  spirit_management_set_basefrequency(FAR struct spirit_library_s *spirit,
                                      uint32_t fbase)
{
  int32_t foffset;
  uint32_t synthword;
  uint32_t chspace;
  uint32_t fc;
  uint8_t band = 0;
  uint8_t anaregs[4];
  uint8_t wcp;
  uint8_t chnum;
  uint8_t refdiv;

  /* Check the parameter */

  DEBUGASSERT(IS_FREQUENCY_BAND(fbase));

  /* Search the operating band */

  if (IS_FREQUENCY_BAND_HIGH(fbase))
    {
      band = HIGH_BAND;
    }
  else if (IS_FREQUENCY_BAND_MIDDLE(fbase))
    {
      band = MIDDLE_BAND;
    }
  else if (IS_FREQUENCY_BAND_LOW(fbase))
    {
      band = LOW_BAND;
    }
  else if (IS_FREQUENCY_BAND_VERY_LOW(fbase))
    {
      band = VERY_LOW_BAND;
    }

  foffset = spirit_radio_get_foffset(spirit);
  chspace = spirit_radio_get_chspace(spirit);
  chnum   = spirit_radio_get_channel(spirit);

  /* Calculates the channel center frequency */

  fc = fbase + foffset + chspace * chnum;

  /* Reads the reference divider */

  refdiv = (uint8_t)spirit_radio_get_refdiv(spirit) + 1;

  switch (band)
    {
    case VERY_LOW_BAND:
      if (fc < 161281250)
        {
          spirit_calib_select_vco(spirit, VCO_L);
        }
      else
        {
          spirit_calib_select_vco(spirit, VCO_H);
        }
      break;

    case LOW_BAND:
      if (fc < 322562500)
        {
          spirit_calib_select_vco(spirit, VCO_L);
        }
      else
        {
          spirit_calib_select_vco(spirit, VCO_H);
        }
      break;

    case MIDDLE_BAND:
      if (fc < 430083334)
        {
          spirit_calib_select_vco(spirit, VCO_L);
        }
      else
        {
          spirit_calib_select_vco(spirit, VCO_H);
        }
      break;

    case HIGH_BAND:
      if (fc < 860166667)
        {
          spirit_calib_select_vco(spirit, VCO_L);
        }
      else
        {
          spirit_calib_select_vco(spirit, VCO_H);
        }
    }

  /* Search the VCO charge pump word and set the corresponding register */

  wcp = spirit_radio_search_wcp(spirit, fc);

  synthword = (uint32_t)
    (fbase * ((double)(FBASE_DIVIDER * refdiv * g_vectc_bhalf[band]) /
     spirit->xtal_frequency));

  /* Build the array of registers values for the analog part */

  anaregs[0] = (uint8_t)(((synthword >> 21) & 0x0000001f) | (wcp << 5));
  anaregs[1] = (uint8_t)((synthword >> 13) & 0x000000ff);
  anaregs[2] = (uint8_t)((synthword >> 5) & 0x000000ff);
  anaregs[3] = (uint8_t)(((synthword & 0x0000001f) << 3) | g_vectc_bandreg[band]);

  /* Configures the needed Analog Radio registers */

  return spirit_reg_write(spirit, SYNT3_BASE, anaregs, 4);
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_managment_wavco_calibration
 *
 * Description:
 *   Perform VCO calbration WA.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

uint8_t spirit_managment_wavco_calibration(FAR struct spirit_library_s *spirit)
{
  uint32_t basefreq;
  uint8_t vco_rxword;
  uint8_t vco_txword;
  uint8_t tmp;
  bool restore = false;
  bool standby = false;
  int ret;

  /* Enable the reference divider if the XTAL is between 48 and 52 MHz */

  if (spirit->xtal_frequency > DOUBLE_XTAL_THR)
    {
      if (spirit_radio_get_refdiv(spirit) == S_DISABLE)
        {
          restore  = true;
          basefreq = spirit_radio_get_basefrequency(spirit);
          ret = spirit_radio_set_refdiv(spirit, S_ENABLE);

          ret = spirit_management_set_basefrequency(spirit, basefreq);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  basefreq = spirit_radio_get_basefrequency(spirit);

  /* Increase the VCO current */

  tmp = 0x19;
  ret = spirit_reg_write(spirit, 0xa1, &tmp, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_calib_enable_vco(spirit, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_update_status(spirit);
  if (ret < 0)
    {
      return ret;
    }

  if (spirit->u.state.MC_STATE == MC_STATE_STANDBY)
    {
      standby = true;
      ret = spirit_command(spirit, CMD_READY);
      if (ret < 0)
        {
          return ret;
        }

      do
        {
          ret = spirit_update_status(spirit);
          if (ret < 0)
            {
              return ret;
            }

          if (spirit->u.state.MC_STATE == 0x13)
            {
              return -EIO;
            }
        }
      while (spirit->u.state.MC_STATE != MC_STATE_READY);
    }

  ret = spirit_command(spirit, CMD_LOCKTX);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
        }

      if (spirit->u.state.MC_STATE == 0x13)
        {
          return -EIO;
        }
    }
  while (spirit->u.state.MC_STATE != MC_STATE_LOCK);

  vco_txword = spirit_calib_get_vcocal(spirit);

  ret = spirit_command(spirit, CMD_READY);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_waitstatus(spirit, MC_STATE_READY, 5000);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_command(spirit, CMD_LOCKRX);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
        }

      if (spirit->u.state.MC_STATE == 0x13)
        {
          return -EIO;
        }
    }
  while (spirit->u.state.MC_STATE != MC_STATE_LOCK);

  vco_rxword = spirit_calib_get_vcocal(spirit);

  ret = spirit_command(spirit, CMD_READY);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
        }

      if (spirit->u.state.MC_STATE == 0x13)
        {
          return 1;
        }
    }
  while (spirit->u.state.MC_STATE != MC_STATE_READY);

  if (standby)
    {
      ret = spirit_command(spirit, CMD_STANDBY);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = spirit_calib_enable_vco(spirit, S_DISABLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Disable the reference divider if the XTAL is between 48 and 52 MHz */

  if (restore)
    {
      ret = spirit_radio_set_refdiv(spirit, S_DISABLE);
      if (ret < 0)
        {
          return ret;
        }

      ret = spirit_management_set_basefrequency(spirit, basefreq);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Restore the VCO current */

  tmp = 0x11;
  ret = spirit_reg_write(spirit, 0xa1, &tmp, 1);
  if (ret < 0)
    {
      return ret;
    }

  spirit_calib_set_vcotxcal(spirit, vco_txword);
  spirit_calib_set_vcorxcal(spirit, vco_rxword);

  return OK;
}

/******************************************************************************
 * Name: spirit_management_txstrobe
 *
 * Description:
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_management_txstrobe(FAR struct spirit_library_s *spirit)
{
  if (spirit->commstate != COMMUNICATION_STATE_TX)
    {
      uint8_t tmp;
      int ret;

      /* To achive the max output power */

      if (spirit->commfrequency >= 150000000 &&
          spirit->commfrequency <= 470000000)
        {
          /* Optimal setting for Tx mode only */

          ret = spirit_radio_set_outputload(spirit, LOAD_3_6_PF);
        }
      else
        {
          /* Optimal setting for Tx mode only */

          ret = spirit_radio_set_outputload(spirit, LOAD_0_PF);
        }

      if (ret < 0)
        {
          return ret;
        }

     /* Enable VCO_L buffer */

      tmp = 0x11;
      ret = spirit_reg_write(spirit, 0xa9, &tmp, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Set SMPS switching frequency */

      tmp = 0x20;
      ret = spirit_reg_write(spirit, PM_CONFIG1_BASE, &tmp, 1);
      if (ret < 0)
        {
          return ret;
        }

      spirit->commstate = COMMUNICATION_STATE_TX;
    }

  return OK;
}

/******************************************************************************
 * Name: spirit_management_rxstrobe
 *
 * Description:
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_management_rxstrobe(FAR struct spirit_library_s *spirit)
{
  uint8_t tmp;
  int ret;

  if (spirit->commstate != COMMUNICATION_STATE_RX)
    {
      /* Set SMPS switching frequency */

      tmp = 0x98;
      ret = spirit_reg_write(spirit, PM_CONFIG1_BASE, &tmp, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Set the correct CWC parameter */

      ret = spirit_radio_set_outputload(spirit, LOAD_0_PF);
      if (ret < 0)
        {
          return ret;
        }

      spirit->commstate = COMMUNICATION_STATE_RX;
    }

  return OK;
}

/******************************************************************************
 * Name: spirit_management_waextracurrent
 *
 * Description:
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_management_waextracurrent(FAR struct spirit_library_s *spirit)
{
  uint8_t tmp;
  int ret;

  tmp = 0xca;
  ret = spirit_reg_write(spirit, 0xb2, &tmp, 1);
  if (ret < 0)
    {
      return ret;
    }

  tmp = 0x04;
  ret = spirit_reg_write(spirit, 0xa8, &tmp, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Just a read to lose a few more microseconds */

  ret = spirit_reg_read(spirit, 0xa8, &tmp, 1);
  if (ret < 0)
    {
      return ret;
    }

  tmp = 0x00;
  ret = spirit_reg_write(spirit, 0xa8, &tmp, 1);
  return ret;
}

/******************************************************************************
 * Name: spirit_management_initcommstate
 *
 * Description:
 *   Initialize communication state
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   frequency - Desired communication frequency
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

void spirit_management_initcommstate(FAR struct spirit_library_s *spirit,
                                     uint32_t frequency)
{
  spirit->commstate     = COMMUNICATION_STATE_NONE;
  spirit->commfrequency = frequency;
}
