/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_radio.c
 * This file provides all the low level API to manage Analog and Digital radio
 * part of SPIRIT.
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
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <errno.h>

#include "spirit_config.h"
#include "spirit_types.h"
#include "spirit_management.h"
#include "spirit_calibration.h"
#include "spirit_radio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#define XTAL_FLAG(xtalFrequency) \
  (xtalFrequency >= 25e6) ? XTAL_FLAG_26_MHz : XTAL_FLAG_24_MHz
#define ROUND(a) \
  (((a - (uint32_t)a) > 0.5) ? (uint32_t)a + 1 : (uint32_t)a)

/* Returns the absolute value. */

#define S_ABS(a) ((a) > 0 ? (a) : -(a))

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

static const uint8_t g_vectc_bandval[4] =
{
  SYNT0_BS_6, SYNT0_BS_12, SYNT0_BS_16, SYNT0_BS_32
};

/* It represents the available channel bandwidth times 10 for 26 Mhz xtal.
 * NOTE: The channel bandwidth for others xtal frequencies can be computed
 * since this table multiplying the current table by a factor
 * xtal_frequency/26e6.
 */

static const uint16_t g_vectn_bandwidth[90] =
{
  8001, 7951, 7684, 7368, 7051, 6709, 6423, 5867, 5414,
  4509, 4259, 4032, 3808, 3621, 3417, 3254, 2945, 2703,
  2247, 2124, 2015, 1900, 1807, 1706, 1624, 1471, 1350,
  1123, 1062, 1005,  950,  903,  853,  812,  735,  675,
   561,  530,  502,  474,  451,  426,  406,  367,  337,
   280,  265,  251,  237,  226,  213,  203,  184,  169,
   140,  133,  126,  119,  113,  106,  101,   92,   84,
    70,   66,   63,   59,   56,   53,   51,   46,   42,
    35,   33,   31,   30,   28,   27,   25,   23,   21,
    18,   17,   16,   15,   14,   13,   13,   12,   11
};

/* These values are used to interpolate the power curves.  Interpolation
 * curves are linear in the following 3 regions:
 *
 * - reg value: 1 to 13    (up region)
 * - reg value: 13 to 40   (mid region)
 * - reg value: 41 to 90   (low region)
 *
 * power_reg = m*power_dBm + q
 *
 * For each band the order is: {m-up, q-up, m-mid, q-mid, m-low, q-low}.
 *
 * NOTE: The power interpolation curves have been extracted by
 * measurements done on the divisional evaluation boards.
 */

static const float g_power_factors[5][6] =
{
  {-2.11, 25.66, -2.11, 25.66, -2.00, 31.28},   /* 915 */
  {-2.04, 23.45, -2.04, 23.45, -1.95, 27.66},   /* 868 */
  {-3.48, 38.45, -1.89, 27.66, -1.92, 30.23},   /* 433 */
  {-3.27, 35.43, -1.80, 26.31, -1.89, 29.61},   /* 315 */
  {-4.18, 50.66, -1.80, 30.04, -1.86, 32.22},   /* 169 */
};

/* It represents the available VCO frequencies */

static const uint16_t g_vectn_vcofreq[16] =
{
  4644, 4708, 4772, 4836, 4902, 4966, 5030, 5095,
  5161, 5232, 5303, 5375, 5448, 5519, 5592, 5663
};

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
 * Name: spirit_radio_initialize
 *
 * Description:
 *   Initializes the SPIRIT analog and digital radio part according to the
 *   specified parameters in the radioinit.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   radioinit - Pointer to a struct radio_init_s that contains the
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_initialize(FAR struct spirit_library_s *spirit,
                            FAR const struct radio_init_s *radioinit)
{
  float ifoff;
  int32_t offset;
  int16_t fcoffset;
  uint8_t anaoffset;
  uint8_t digregs[4];
  uint8_t anaregs[8];
  uint8_t drm;
  uint8_t dre;
  uint8_t fdevm;
  uint8_t fdeve;
  uint8_t bwm;
  uint8_t bwe;
  uint8_t regval;
  uint8_t value;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_FREQUENCY_BAND(radioinit->base_frequency));
  DEBUGASSERT(IS_MODULATION_SELECTED(radioinit->modselect));
  DEBUGASSERT(IS_DATARATE(radioinit->datarate));
  DEBUGASSERT(IS_FREQUENCY_OFFSET(offset, spirit->xtal_frequency));
  DEBUGASSERT(IS_CHANNEL_SPACE
                 (radioinit->chspace, spirit->xtal_frequency));
  DEBUGASSERT(IS_F_DEV(radioinit->freqdev, spirit->xtal_frequency));

  /* Workaround for Vtune */

  value = 0xa0;
  ret = spirit_reg_write(spirit, 0x9F, &value, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Calculates the offset respect to RF frequency and according to xtal_ppm
   * parameter: (xtal_ppm*FBase)/10^6
   */

  offset = (int32_t)(((float)radioinit->xtal_offset_ppm *
                     radioinit->base_frequency) / PPM_FACTOR);

  /* Disable the digital, ADC, SMPS reference clock divider if fXO > 24MHz or
   * fXO < 26MHz
   */

  ret = spirit_command(spirit, COMMAND_STANDBY);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      volatile uint8_t i;

      /* Delay for state transition */

      for (i = 0; i != 0xff; i++);

      /* Reads the MC_STATUS register */

      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
        }
    }
  while (spirit->u.state.MC_STATE != MC_STATE_STANDBY);

  if (spirit->xtal_frequency < DOUBLE_XTAL_THR)
    {
      ret = spirit_radio_enable_digdivider(spirit, S_DISABLE);
      DEBUGASSERT(IS_CH_BW(radioinit->bandwidth, spirit->xtal_frequency));
    }
  else
    {
      ret = spirit_radio_enable_digdivider(spirit, S_ENABLE);
      DEBUGASSERT(IS_CH_BW(radioinit->bandwidth, (spirit->xtal_frequency >> 1)));
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Go to READY state */

  ret = spirit_command(spirit, COMMAND_READY);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      volatile uint8_t i;

      /* Delay for state transition */

      for (i = 0; i != 0xff; i++);

      /* Reads the MC_STATUS register */

      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
       }
    }
  while (spirit->u.state.MC_STATE != MC_STATE_READY);

  /* Calculates the FC_OFFSET parameter and cast as signed int: offset =
   * (Fxtal/2^18)*FC_OFFSET
   */

  fcoffset = (int16_t)(((float)offset * FBASE_DIVIDER) /
                       spirit->xtal_frequency);
  anaregs[2] = (uint8_t)((((uint16_t)fcoffset) >> 8) & 0x0f);
  anaregs[3] = (uint8_t)fcoffset;

  /* Calculates the channel space factor */

  anaregs[0] = ((uint32_t)radioinit->chspace << 9) /
                (spirit->xtal_frequency >> 6) + 1;

  spirit_management_initcommstate(spirit, radioinit->base_frequency);

  /* 2nd order DEM algorithm enabling */

  ret = spirit_reg_read(spirit, 0xa3, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~0x02;
  ret = spirit_reg_write(spirit, 0xa3, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Check the channel center frequency is in one of the possible range */

  DEBUGASSERT(IS_FREQUENCY_BAND((radioinit->base_frequency +
                                ((fcoffset * spirit->xtal_frequency) / FBASE_DIVIDER) +
                                radioinit->chspace * radioinit->chnum)));

  /* Calculates the datarate mantissa and exponent */

  ret = spirit_radio_convert_datarate(spirit, radioinit->datarate,
                                      &drm, &dre);
  if (ret < 0)
    {
      return ret;
    }

  digregs[0] = (uint8_t)(drm);
  digregs[1] = (uint8_t)(radioinit->modselect | dre);

  /* Read the fdev register to preserve the clock recovery algo bit */

  ret = spirit_reg_read(spirit, 0x1c, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Calculates the frequency deviation mantissa and exponent */

  ret = spirit_radio_convert_freqdev(spirit, radioinit->freqdev,
                                     &fdevm, &fdeve);
  if (ret < 0)
    {
      return ret;
    }

  digregs[2] = (uint8_t)((fdeve << 4) | (regval & 0x08) | fdevm);

  /* Calculates the channel filter mantissa and exponent */

  ret = spirit_radio_convert_chbandwidth(spirit, radioinit->bandwidth, &bwm, &bwe);
  if (ret < 0)
    {
      return ret;
    }

  digregs[3] = (uint8_t) ((bwm << 4) | bwe);

  ifoff      = (3.0 * 480140) / (spirit->xtal_frequency >> 12) - 64;
  anaoffset  = ROUND(ifoff);

  if (spirit->xtal_frequency < DOUBLE_XTAL_THR)
    {
      /* if offset digital is the same in case of single xtal */

      anaregs[1] = anaoffset;
    }
  else
    {
      ifoff = (3.0 * 480140) / (spirit->xtal_frequency >> 13) - 64;

      /* ... otherwise recompute it */

      anaregs[1] = ROUND(ifoff);
    }

#if 0
  if (spirit->xtal_frequency == 24000000)
    {
      anaoffset  = 0xb6;
      anaregs[1] = 0xb6;
    }

  if (spirit->xtal_frequency == 25000000)
    {
      anaoffset  = 0xac;
      anaregs[1] = 0xac;
    }

  if (spirit->xtal_frequency == 26000000)
    {
      anaoffset  = 0xa3;
      anaregs[1] = 0xa3;
    }

  if (spirit->xtal_frequency == 48000000)
    {
      anaoffset  = 0x3b;
      anaregs[1] = 0xb6;
    }

  if (spirit->xtal_frequency == 50000000)
    {
      anaoffset  = 0x36;
      anaregs[1] = 0xac;
    }

  if (spirit->xtal_frequency == 52000000)
    {
      anaoffset  = 0x31;
      anaregs[1] = 0xa3;
    }
#endif

  ret = spirit_reg_write(spirit, IF_OFFSET_ANA_BASE, &anaoffset, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Set Xtal configuration */

  if (spirit->xtal_frequency > DOUBLE_XTAL_THR)
    {
      enum xtal_flag_e xtlflag = XTAL_FLAG((spirit->xtal_frequency / 2));
      ret = spirit_radio_set_xtalflag(spirit, xtlflag);
    }
  else
    {
      enum xtal_flag_e xtlflag = XTAL_FLAG(spirit->xtal_frequency);
      ret = spirit_radio_set_xtalflag(spirit, xtlflag);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Sets the channel number in the corresponding register */

  ret = spirit_reg_write(spirit, CHNUM_BASE, &radioinit->chnum, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Configures the Analog Radio registers */

  ret = spirit_reg_write(spirit, CHSPACE_BASE, anaregs, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Configures the Digital Radio registers */

  ret = spirit_reg_write(spirit, MOD1_BASE, digregs, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the freeze option of the AFC on the SYNC word */

  ret = spirit_radio_afcfreezeonsync(spirit, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the IQC correction optimal value */

  anaregs[0] = 0x80;
  anaregs[1] = 0xe3;

  ret = spirit_reg_write(spirit, 0x99, anaregs, 2);
  if (ret < 0)
    {
      return ret;
    }

  return spirit_radio_set_basefrequency(spirit, radioinit->base_frequency);
}

/******************************************************************************
 * Name: spirit_radio_set_xtalflag
 *
 * Description:
 *   Sets the Xtal configuration in the ANA_FUNC_CONF0 register.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   xtalflag one of the possible value of the enum type xtal_flag_e.
 *         XTAL_FLAG_24_MHz:  in case of 24 MHz crystal
 *         XTAL_FLAG_26_MHz:  in case of 26 MHz crystal
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_set_xtalflag(FAR struct spirit_library_s *spirit,
                              enum xtal_flag_e xtalflag)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_XTAL_FLAG(xtalflag));

  /* Reads the ANA_FUNC_CONF_0 register */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (xtalflag == XTAL_FLAG_26_MHz)
        {
          regval |= SELECT_24_26_MHZ_MASK;
        }
      else
        {
          regval &= (~SELECT_24_26_MHZ_MASK);
        }

      /* Sets the 24_26MHz_SELECT field in the ANA_FUNC_CONF_0 register */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_xtalflag
 *
 * Description:
 *   Returns the Xtal configuration in the ANA_FUNC_CONF0 register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   XtalFrequency Settled Xtal configuration.
 *
 ******************************************************************************/

enum xtal_flag_e spirit_radio_get_xtalflag(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the Xtal configuration in the ANA_FUNC_CONF_0 register and return
   * the value.
   */

  (void)spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);

  return (enum xtal_flag_e)((regval & 0x40) >> 6);
}

/******************************************************************************
 * Name: spirit_radio_search_wcp
 *
 * Description:
 *   Returns the charge pump word for a given VCO frequency.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fc     - Channel center frequency expressed in Hz. This parameter may
 *            be a value in one of the following ranges:
 *
 *              High_Band:     from 779 MHz to 915 MHz
 *              Middle Band:   from 387 MHz to 470 MHz
 *              Low Band:      from 300 MHz to 348 MHz
 *              Very low Band: from 150 MHz to 174 MHz
 *
 * Returned Value:
 *   Charge pump word.
 *
 ******************************************************************************/

uint8_t spirit_radio_search_wcp(FAR struct spirit_library_s *spirit,
                                uint32_t fc)
{
  uint32_t vcofreq;
  uint8_t bfactor;
  int8_t i;

  /* Check the channel center frequency is in one of the possible range */

  DEBUGASSERT(IS_FREQUENCY_BAND(fc));

  /* Search the operating band */

  if (IS_FREQUENCY_BAND_HIGH(fc))
    {
      bfactor = HIGH_BAND_FACTOR;
    }
  else if (IS_FREQUENCY_BAND_MIDDLE(fc))
    {
      bfactor = MIDDLE_BAND_FACTOR;
    }
  else if (IS_FREQUENCY_BAND_LOW(fc))
    {
      bfactor = LOW_BAND_FACTOR;
    }
  else
    {
      bfactor = VERY_LOW_BAND_FACTOR;
    }

  /* Calculates the VCO frequency VCOFreq = fc*B */

  vcofreq = (fc / 1000000) * bfactor;

  /* Search in the vco frequency array the charge pump word */

  if (vcofreq >= g_vectn_vcofreq[15])
    {
      i = 15;
    }
  else
    {
      /* Search the value */

      for (i = 0; i < 15 && vcofreq > g_vectn_vcofreq[i]; i++);

      /* Be sure that it is the best approssimation */

      if (i != 0 &&
          g_vectn_vcofreq[i] - vcofreq > vcofreq - g_vectn_vcofreq[i - 1])
        {
          i--;
        }
    }

  /* Return index */

  return (i & 7);
}

/******************************************************************************
 * Name: spirit_radio_get_synthword
 *
 * Description:
 *   Returns the synth word.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   32-bit Synth word.  Errors are not reported.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_synthword(FAR struct spirit_library_s *spirit)
{
  uint8_t regvalues[4];

  /* Reads the SYNTH registers, build the synth word and return it */

  (void)spirit_reg_read(spirit, SYNT3_BASE, regvalues, 4);
  return ((((uint32_t) (regvalues[0] & 0x1f)) << 21) +
          (((uint32_t) (regvalues[1])) << 13) +
          (((uint32_t) (regvalues[2])) << 5) +
          (((uint32_t) (regvalues[3])) >> 3));
}

/******************************************************************************
 * Name: spirit_radio_set_synthword
 *
 * Description:
 *   Sets the SYNTH registers.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   synthword - The synth word to write in the SYNTH[3:0] registers.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_synthword(FAR struct spirit_library_s *spirit,
                               uint32_t synthword)
{
  uint8_t regvalues[4];
  uint8_t synt0;
  int ret;

  /* Reads the SYNT0 register */

  ret = spirit_reg_read(spirit, SYNT0_BASE, &synt0, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Mask the Band selected field */

  synt0 &= 0x07;

  /* Build the array for SYNTH registers */

  regvalues[0] = (uint8_t)((synthword >> 21) & (0x0000001f));
  regvalues[1] = (uint8_t)((synthword >> 13) & (0x000000ff));
  regvalues[2] = (uint8_t)((synthword >> 5) & (0x000000ff));
  regvalues[3] = (uint8_t)(((synthword & 0x0000001f) << 3) | synt0);

  /* Write the synth word to the SYNTH registers */

  ret = spirit_reg_write(spirit, SYNT3_BASE, regvalues, 4);
  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_band
 *
 * Description:
 *   Sets the operating band.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   band   - The band to set.  This parameter can be one of following values:
 *             HIGH_BAND      High_Band selected: from 779 MHz to 915 MHz
 *             MIDDLE_BAND:   Middle Band selected: from 387 MHz to 470 MHz
 *             LOW_BAND:      Low Band selected: from 300 MHz to 348 MHz
 *             VERY_LOW_BAND: Very low Band selected: from 150 MHz to 174 MHz
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_band(FAR struct spirit_library_s *spirit,
                          enum spirit_bandselect_e band)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_BAND_SELECTED(band));

  /* Reads the SYNT0 register */

  ret = spirit_reg_read(spirit, SYNT0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the SYNTH[4;0] field and write the BS value */

      regval &= 0xf8;
      regval |= g_vectc_bandval[band];

      /* Configures the SYNT0 register setting the operating band */

      ret = spirit_reg_write(spirit, SYNT0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_band
 *
 * Description:
 *   Returns the operating band.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   BandSelect Settled band.  This returned value may be one of the
 *   following values:
 *     HIGH_BAND      High_Band selected: from 779 MHz to 915 MHz
 *     MIDDLE_BAND:   Middle Band selected: from 387 MHz to 470 MHz
 *     LOW_BAND:      Low Band selected: from 300 MHz to 348 MHz
 *     VERY_LOW_BAND: Very low Band selected: from 150 MHz to 174 MHz
 *
 ******************************************************************************/

enum spirit_bandselect_e
  spirit_radio_get_band(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the SYNT0 register */

  (void)spirit_reg_read(spirit, SYNT0_BASE, &regval, 1);

  /* Mask the Band selected field */

  if ((regval & 0x07) == SYNT0_BS_6)
    {
      return HIGH_BAND;
    }
  else if ((regval & 0x07) == SYNT0_BS_12)
    {
      return MIDDLE_BAND;
    }
  else if ((regval & 0x07) == SYNT0_BS_16)
    {
      return LOW_BAND;
    }
  else
    {
      return VERY_LOW_BAND;
    }
}

/******************************************************************************
 * Name: spirit_radio_set_chspace
 *
 * Description:
 *   Sets the channel space factor in channel space register.  The channel
 *   spacing step is computed as F_Xo/32768.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   chspace - The channel space expressed in Hz.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_set_chspace(FAR struct spirit_library_s *spirit,
                              uint32_t chspace)
{
  uint8_t factor;

  /* Round to the nearest integer */

  factor = ((uint32_t)chspace * CHSPACE_DIVIDER) / spirit->xtal_frequency;

  /* Write value into the register */

  return spirit_reg_write(spirit, CHSPACE_BASE, &factor, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_chspace
 *
 * Description:
 *   Returns the channel space register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Channel space. The channel space is:
 *
 *     CS = channel_space_factor x XtalFrequency/2^15
 *
 *   where channel_space_factor is the CHSPACE register value.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_chspace(FAR struct spirit_library_s *spirit)
{
  uint8_t factor;

  /* Reads the CHSPACE register, calculate the channel space and return it */

  (void)spirit_reg_read(spirit, CHSPACE_BASE, &factor, 1);

  /* Compute the Hertz value and return it */

  return ((factor * spirit->xtal_frequency) / CHSPACE_DIVIDER);
}

/******************************************************************************
 * Name: spirit_radio_set_channel
 *
 * Description:
 *   Sets the channel number.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   chnum the channel number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_set_channel(FAR struct spirit_library_s *spirit,
                             uint8_t chnum)
{
  /* Writes the CHNUM register */

  return spirit_reg_write(spirit, CHNUM_BASE, &chnum, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_channel
 *
 * Description:
 *   Returns the actual channel number.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Actual channel number.
 *
 ******************************************************************************/

uint8_t spirit_radio_get_channel(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the CHNUM register and return the value */

  (void)spirit_reg_read(spirit, CHNUM_BASE, &regval, 1);
  return regval;
}

/******************************************************************************
 * Name: spirit_radio_get_foffset
 *
 * Description:
 *   Returns the actual frequency offset.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Frequency offset expressed in Hz as signed word.
 *
 ******************************************************************************/

int32_t spirit_radio_get_foffset(FAR struct spirit_library_s *spirit)
{
  uint16_t offtmp;
  int16_t fcoffset;
  uint8_t tmp[2];

  /* Reads the FC_OFFSET registers */

  (void)spirit_reg_read(spirit, FC_OFFSET1_BASE, tmp, 2);

  /* Calculates the Offset Factor */

  offtmp = (((uint16_t)tmp[0] << 8) + (uint16_t)tmp[1]);

  if ((offtmp & 0x0800) != 0)
    {
      offtmp |= 0xf000;
    }
  else
    {
      offtmp &= 0x0fff;
    }

  fcoffset = *((int16_t *)(&offtmp));

  /* Calculates the frequency offset and return it */

  return ((int32_t)(fcoffset * spirit->xtal_frequency) / FBASE_DIVIDER);
}

/******************************************************************************
 * Name: spirit_radio_set_basefrequency
 *
 * Description:
 *   Sets the Synth word and the Band Select register according to desired
 *   base carrier frequency.  In this API the Xtal configuration is read out
 *   from the corresponding register. The user shall fix it before call this
 *   API.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fbase  - The base carrier frequency expressed in Hz as unsigned word.
 *
 * Returned Value:
 *   Error code: 0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_set_basefrequency(FAR struct spirit_library_s *spirit,
                                   uint32_t fbase)
{
  int32_t foffset;
  uint32_t synthword;
  uint32_t chspace;
  uint32_t fc;
  uint8_t anaregs[4];
  uint8_t refdiv;
  uint8_t band;
  uint8_t wcp;
  uint8_t chnum;
  int ret;

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
  else
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

  /* Selects the VCO */

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

  synthword = (uint32_t)(fbase * g_vectc_bhalf[band] *
                         (((double)(FBASE_DIVIDER * refdiv)) /
                         spirit->xtal_frequency));

  /* Build the array of registers values for the analog part */

  anaregs[0] = (uint8_t)(((synthword >> 21) & 0x0000001f) | (wcp << 5));
  anaregs[1] = (uint8_t)((synthword >> 13) & 0x000000ff);
  anaregs[2] = (uint8_t)((synthword >> 5) & 0x000000ff);
  anaregs[3] = (uint8_t)(((synthword & 0x0000001f) << 3) | g_vectc_bandval[band]);

  /* Configures the needed Analog Radio registers */

  ret = spirit_reg_write(spirit, SYNT3_BASE, anaregs, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Should be perform the VCO calibration WA? */

  if (spirit->vcocalib == S_ENABLE)
    {
      return spirit_managment_wavco_calibration(spirit);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_enable_wavco_calibration
 *
 * Description:
 *   Enable/disabe the VCO calibration WA at the end of
 *   spirit_radio_set_basefrequency()
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   S_ENABLE or S_DISABLE the WA procedure.
 *
 * Returned Value:
 *   None.
 *
 ******************************************************************************/

void spirit_radio_enable_wavco_calibration(FAR struct spirit_library_s *spirit,
                                           enum spirit_functional_state_e newstate)
{
  spirit->vcocalib = newstate;
}

/******************************************************************************
 * Name: spirit_radio_get_basefrequency
 *
 * Description:
 *   Returns the base carrier frequency.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Base carrier frequency expressed in Hz as unsigned word.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_basefrequency(FAR struct spirit_library_s *spirit)
{
  enum spirit_bandselect_e band;
  uint32_t synthword;
  uint8_t refdiv;

  /* Read the synth word */

  synthword = spirit_radio_get_synthword(spirit);

  /* Read the operating band */

  band = spirit_radio_get_band(spirit);

  refdiv = (uint8_t)spirit_radio_get_refdiv(spirit) + 1;

  /* Calculates the frequency base and return it */

  return (uint32_t) round(synthword * (((double)spirit->xtal_frequency) /
                                       (FBASE_DIVIDER * refdiv *
                                        g_vectc_bhalf[band])));
}

/******************************************************************************
 * Name: spirit_radio_convert_datarate
 *
 * Description:
 *   Returns the mantissa and exponent, whose value used in the datarate
 *   formula will give the datarate value closer to the given datarate.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   datarate - datarate expressed in bps. This parameter ranging between 100 and 500000.
 *   pcm      - pointer to the returned mantissa value.
 *   pce      - pointer to the returned exponent value.
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_convert_datarate(FAR struct spirit_library_s *spirit,
                                  uint32_t datarate, FAR uint8_t *pcm,
                                  FAR uint8_t *pce)
{
  int16_t intermediate[3];
  uint16_t delta;
  uint8_t mantissa;
  uint8_t divider = 0;
  int8_t i = 15;
  uint8_t j;
  volatile bool find = false;

  /* Check the parameters */

  DEBUGASSERT(IS_DATARATE(datarate));

  divider = (uint8_t)spirit_radio_isenabled_digdivider(spirit);

  /* Search in the datarate array the exponent value */

  while (!find && i >= 0)
    {
      if (datarate >= (spirit->xtal_frequency >> (20 - i + divider)))
        {
          find = true;
        }
      else
        {
          i--;
        }
    }

  i < 0 ? i = 0 : i;
  *pce = i;

  /* Calculates the mantissa value according to the datarate formula */

  mantissa = (datarate * ((uint32_t)1 << (23 - i))) /
              (spirit->xtal_frequency >> (5 + divider)) - 256;

  /* Finds the mantissa value with less approximation */

  for (j = 0; j < 3; j++)
    {
      if ((mantissa + j - 1))
        {
          intermediate[j] = datarate - (((256 + mantissa + j - 1) *
                            (spirit->xtal_frequency >> (5 + divider))) >>
                            (23 - i));
        }
      else
        {
          intermediate[j] = 0x7fff;
        }
    }

  delta = 0xffff;
  for (j = 0; j < 3; j++)
    {
      if (S_ABS(intermediate[j]) < delta)
        {
          delta = S_ABS(intermediate[j]);
          *pcm = mantissa + j - 1;
        }
    }

  return OK;
}

/******************************************************************************
 * Name: spirit_radio_convert_freqdev
 *
 * Description:
 *   Returns the mantissa and exponent, whose value used in the frequency
 *   deviation formula will give a frequency deviation value most closer to
 *   the given frequency deviation.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fdev   - Frequency deviation expressed in Hz. This parameter can be a
 *            value in the range [F_Xo*8/2^18, F_Xo*7680/2^18].
 *   pcm    - pointer to the returned mantissa value.
 *   pce    - pointer to the returned exponent value.
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_convert_freqdev(FAR struct spirit_library_s *spirit,
                                 uint32_t fdev, FAR uint8_t *pcm,
                                 FAR uint8_t *pce)
{
  uint32_t a;
  uint32_t bp;
  uint32_t b = 0;
  uint8_t i;
  float xtalDivtmp = (float)spirit->xtal_frequency / (((uint32_t) 1) << 18);

  /* Check the parameters */

  DEBUGASSERT(IS_F_DEV(fdev, spirit->xtal_frequency));

  for (i = 0; i < 10; i++)
    {
      a = (uint32_t) (xtalDivtmp * (uint32_t) (7.5 * (1 << i)));
      if (fdev < a)
        {
          break;
        }
    }

  *pce = i;

  for (i = 0; i < 8; i++)
    {
      bp = b;
      b = (uint32_t)(xtalDivtmp * (uint32_t)((8.0 + i) / 2 * (1 << (*pce))));
      if (fdev < b)
        {
          break;
        }
    }

  if ((fdev - bp) < (b - fdev))
    {
      i--;
    }

  *pcm = i;
  return OK;
}

/******************************************************************************
 * Name:
 *
 * Description:
 *   Returns the mantissa and exponent for a given bandwidth.  Even if it is
 *   possible to pass as parameter any value in the below mentioned range, the
 *   API will search the closer value according to a fixed table of channel
 *   bandwidth values (@ref s_vectnBandwidth), as defined in the datasheet,
 *   returning the corresponding mantissa and exponent value.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   bandwidth - bandwidth expressed in Hz. This parameter ranging between
 *               1100 and 800100.
 *   pcm       - pointer to the returned mantissa value.
 *   pce       - pointer to the returned exponent value.
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_convert_chbandwidth(FAR struct spirit_library_s *spirit,
                                     uint32_t bandwidth, FAR uint8_t *pcm,
                                     FAR uint8_t *pce)
{
  uint32_t chfltfactor;
  int16_t intermediate[3];
  uint16_t delta;
  uint8_t divider = 1;
  int8_t tmp;
  int8_t i;
  int8_t j;

  /* Search in the channel filter bandwidth table the exponent value */

  if (spirit_radio_isenabled_digdivider(spirit) != S_DISABLE)
    {
      divider = 2;
    }
  else
    {
      divider = 1;
    }

  DEBUGASSERT(IS_CH_BW(bandwidth, spirit->xtal_frequency / divider));

  chfltfactor = (spirit->xtal_frequency / divider) / 100;

  for (i = 0;
       i < 90 && (bandwidth < (uint32_t)((g_vectn_bandwidth[i] *
                  chfltfactor) / 2600));
       i++);

  if (i != 0)
    {
      /* Finds the mantissa value with less approximation */

      tmp = i;
      for (j = 0; j < 3; j++)
        {
          if (((tmp + j - 1) >= 0) || ((tmp + j - 1) <= 89))
            {
              intermediate[j] = bandwidth -
                               (uint32_t)((g_vectn_bandwidth[tmp + j - 1] *
                                chfltfactor) / 2600);
            }
          else
            {
              intermediate[j] = 0x7fff;
            }
        }

      delta = 0xFFFF;

      for (j = 0; j < 3; j++)
        {
          if (S_ABS(intermediate[j]) < delta)
            {
              delta = S_ABS(intermediate[j]);
              i = tmp + j - 1;
            }
        }
    }

  *pce = (uint8_t)(i / 9);
  *pcm = (uint8_t)(i % 9);
  return OK;
}

/******************************************************************************
 * Name: spirit_radio_dbm2reg
 *
 * Description:
 *   Returns the PA register value that corresponds to the passed dBm power.
 *
 *   NOTE: The power interpolation curves used by this function have been
 *   extracted by measurements done on the divisional evaluation boards.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   fbase    Frequency base expressed in Hz.
 *   powerdbm Desired power in dBm.
 *
 * Returned Value:
 *   Register value as byte.
 *
 ******************************************************************************/

uint8_t spirit_radio_dbm2reg(FAR struct spirit_library_s *spirit,
                             uint32_t fbase, float powerdbm)
{
  float pavalue;
  uint8_t i = 0;
  uint8_t j = 0;

  if (IS_FREQUENCY_BAND_HIGH(fbase))
    {
      i = 0;
      if (fbase < 900000000)
        {
          i = 1;  /* 868 */
        }
    }
  else if (IS_FREQUENCY_BAND_MIDDLE(fbase))
    {
      i = 2;
    }
  else if (IS_FREQUENCY_BAND_LOW(fbase))
    {
      i = 3;
    }
  else if (IS_FREQUENCY_BAND_VERY_LOW(fbase))
    {
      i = 4;
    }

  j = 1;
  if (powerdbm > 0 &&
      13.0 / g_power_factors[i][2] - g_power_factors[i][3] /
             g_power_factors[i][2] < powerdbm)
    {
      j = 0;
    }
  else if (powerdbm <= 0 &&
           40.0 / g_power_factors[i][2] - g_power_factors[i][3] /
                  g_power_factors[i][2] > powerdbm)
    {
      j = 2;
    }

  pavalue = g_power_factors[i][2 * j] * powerdbm +
            g_power_factors[i][2 * j + 1];

  if (pavalue < 1)
    {
      pavalue = 1;
    }
  else if (pavalue > 90)
    {
      pavalue = 90;
    }

  return (uint8_t)pavalue;
}

/******************************************************************************
 * Name: spirit_radio_set_palevel
 *
 * Description:
 *   Sets a specific PA_LEVEL register, with a value given in dBm.
 *
 *   NOTE: This function makes use of the @ref spirit_radio_dbm2reg fcn to
 *   interpolate the power value.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   ndx      - PA_LEVEL to set. This parameter shall be in the range [0:7].
 *   powerdbm - PA value to write expressed in dBm . Be sure that this values
 *              is in the correct range [-PA_LOWER_LIMIT: PA_UPPER_LIMIT] dBm.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_palevel(FAR struct spirit_library_s *spirit,
                             uint8_t ndx, float powerdbm)
{
  uint32_t basefrequency;
  uint8_t address;
  uint8_t level;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(ndx));
  DEBUGASSERT(IS_PAPOWER_DBM(powerdbm));

  /* Interpolate the power level */

  basefrequency = spirit_radio_get_basefrequency(spirit);
  level         = spirit_radio_dbm2reg(spirit, basefrequency, powerdbm);

  /* Sets the base address */

  address = PA_POWER8_BASE + 7 - ndx;

  /* Configures the PA_LEVEL register */

  return spirit_reg_write(spirit, address, &level, 1);
}

/******************************************************************************
 * Name: spirit_radio_set_outputload
 *
 * Description:
 *   Sets the output stage additional load capacitor bank.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   load one of the possible value of the enum type enum spirit_paload_capacitor_e.
 *         LOAD_0_PF    No additional PA load capacitor
 *         LOAD_1_2_PF  1.2pF additional PA load capacitor
 *         LOAD_2_4_PF  2.4pF additional PA load capacitor
 *         LOAD_3_6_PF  3.6pF additional PA load capacitor
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_outputload(FAR struct spirit_library_s *spirit,
                                enum spirit_paload_capacitor_e load)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_LOAD_CAP(load));

  /* Reads the PA_POWER_0 register */

  ret = spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the CWC[1:0] field and write the new value */

      regval &= 0x3f;
      regval |= load;

      /* Configures the PA_POWER_0 register */

      ret = spirit_reg_write(spirit, PA_POWER0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_outputload
 *
 * Description:
 *   Returns the output stage additional load capacitor bank.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Output stage additional load capacitor bank.  This value may be:
 *
 *     LOAD_0_PF    No additional PA load capacitor
 *     LOAD_1_2_PF  1.2pF additional PA load capacitor
 *     LOAD_2_4_PF  2.4pF additional PA load capacitor
 *     LOAD_3_6_PF  3.6pF additional PA load capacitor
 *
 ******************************************************************************/

enum spirit_paload_capacitor_e
  spirit_radio_get_outputload(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PA_POWER_0 register */

  (void)spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);

  /* Mask the CWC[1:0] field and return the value */

  return (enum spirit_paload_capacitor_e) (regval & 0xc0);
}

/******************************************************************************
 * Name: spirit_radio_set_palevel_maxindex
 *
 * Description:
 *   Sets a specific PA_LEVEL_MAX_INDEX.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ndx    - PA_LEVEL_MAX_INDEX to set. This parameter must be in the range
 *            [0:7].
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_palevel_maxindex(FAR struct spirit_library_s *spirit,
                                      uint8_t ndx)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(ndx));

  /* Reads the PA_POWER_0 register */

  ret = spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the PA_LEVEL_MAX_INDEX[1:0] field and write the new value */

      regval &= 0xf8;
      regval |= ndx;

      /* Configures the PA_POWER_0 register */

      ret = spirit_reg_write(spirit, PA_POWER0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_afcfreezeonsync
 *
 * Description:
 *   Enables or Disables the AFC freeze on sync word detection.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - new state for AFC freeze on sync word detection.
 *              This parameter can be: S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_afcfreezeonsync(FAR struct spirit_library_s *spirit,
                                 enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the AFC_2 register and configure the AFC Freeze on Sync field */

  ret = spirit_reg_read(spirit, AFC2_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AFC2_AFC_FREEZE_ON_SYNC_MASK;
        }
      else
        {
          regval &= (~AFC2_AFC_FREEZE_ON_SYNC_MASK);
        }

      /* Sets the AFC_2 register */

      ret = spirit_reg_write(spirit, AFC2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_enable_csblanking
 *
 * Description:
 *   Enables or Disables the received data blanking when the CS is under the
 *   threshold.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state of this mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_enable_csblanking(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the ANT_SELECT_CONF_BASE and mask the CS_BLANKING BIT field */

  ret = spirit_reg_read(spirit, ANT_SELECT_CONF_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= ANT_SELECT_CS_BLANKING_MASK;
        }
      else
        {
          regval &= (~ANT_SELECT_CS_BLANKING_MASK);
        }

      /* Write the new value to the ANT_SELECT_CONF register */

      ret = spirit_reg_write(spirit, ANT_SELECT_CONF_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_persistentrx
 *
 * Description:
 *   Enables or Disables the persistent RX mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state of this mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_persistentrx(FAR struct spirit_library_s *spirit,
                              enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL0_BASE and mask the PROTOCOL0_PERS_RX_MASK bitfield */

  ret = spirit_reg_read(spirit, PROTOCOL0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL0_PERS_RX_MASK;
        }
      else
        {
          regval &= (~PROTOCOL0_PERS_RX_MASK);
        }

      /* Writes the new value in the PROTOCOL0_BASE register */

      ret = spirit_reg_write(spirit, PROTOCOL0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_refdiv
 *
 * Description:
 *   Enables or Disables the synthesizer reference divider.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate new state for synthesizer reference divider.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_refdiv(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the SYNTH_CONFIG1_BASE and mask the REFDIV bit field */

  ret = spirit_reg_read(spirit, SYNTH_CONFIG1_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= 0x80;
        }
      else
        {
          regval &= 0x7f;
        }

      /* Writes the new value in the SYNTH_CONFIG1_BASE register */

      ret = spirit_reg_write(spirit, SYNTH_CONFIG1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_refdiv
 *
 * Description:
 *   Get the the synthesizer reference divider state.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.  Errors are not reported.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_radio_get_refdiv(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  (void)spirit_reg_read(spirit, SYNTH_CONFIG1_BASE, &regval, 1);

  if (((regval >> 7) & 0x1) != 0)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

/******************************************************************************
 * Name: spirit_radio_enable_digdivider
 *
 * Description:
 *   Enables or Disables the synthesizer reference divider.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for synthesizer reference divider.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_enable_digdivider(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the XO_RCO_TEST_BASE and mask the PD_CLKDIV bit field */

  ret = spirit_reg_read(spirit, XO_RCO_TEST_BASE, &regval, 1);
  if (ret > 0)
    {
      if (newstate == S_ENABLE)
        {
          regval &= 0xf7;
        }
      else
        {
          regval |= 0x08;
        }

      /* Write the new value to the XO_RCO_TEST_BASE register */

      ret = spirit_reg_write(spirit, XO_RCO_TEST_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_isenabled_digdivider
 *
 * Description:
 *   Get the the synthesizer reference divider state.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.  Error conditions are not detected.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_radio_isenabled_digdivider(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  (void)spirit_reg_read(spirit, XO_RCO_TEST_BASE, &regval, 1);

  if (((regval >> 3) & 0x1) != 0)
    {
      return S_DISABLE;
    }
  else
    {
      return S_ENABLE;
    }
}
