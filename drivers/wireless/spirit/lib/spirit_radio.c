/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_radio.c
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
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <sys/types.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>

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

/* It represents the available channel bandwidth times 10 for 26 MHz xtal.
 * NOTE: The channel bandwidth for others xtal frequencies can be computed
 * since this table multiplying the current table by a factor
 * xtal_frequency/26e6.
 */

static const uint16_t g_vectn_bandwidth[90] =
{
  8001, 7951, 7684, 7368, 7051, 6709, 6423, 5867, 5414,
  4509, 4259, 4032, 3808, 3621, 3417, 3254, 2945, 2703,
  2247, 2124, 2015, 1900, 1807, 1706, 1624, 1471, 1350,
  1123, 1062, 1005, 950,  903,  853,  812,  735,  675,
  561,  530,  502,  474,  451,  426,  406,  367,  337,
  280,  265,  251,  237,  226,  213,  203,  184,  169,
  140,  133,  126,  119,  113,  106,  101,  92,   84,
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

  /* Workaround for Vtune */

  value = 0xa0;
  ret = spirit_reg_write(spirit, 0x9f, &value, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Calculates the offset respect to RF frequency and according to xtal_ppm
   * parameter: (xtal_ppm*FBase)/10^6
   */

  offset = (int32_t)(((float)radioinit->foffset * radioinit->base_frequency) /
                     PPM_FACTOR);

  /* Check the parameters */

  DEBUGASSERT(IS_FREQUENCY_BAND(radioinit->base_frequency));
  DEBUGASSERT(IS_MODULATION_SELECTED(radioinit->modselect));
  DEBUGASSERT(IS_DATARATE(radioinit->datarate));
  DEBUGASSERT(IS_FREQUENCY_OFFSET(offset, spirit->xtal_frequency));
  DEBUGASSERT(IS_CHANNEL_SPACE(radioinit->chspace, spirit->xtal_frequency));
  DEBUGASSERT(IS_F_DEV(radioinit->freqdev, spirit->xtal_frequency));

  /* Make sure that we are in the READY state */

  ret = spirit_waitstatus(spirit, MC_STATE_READY, 5);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to go to READY state: %d\n", ret);
      return ret;
    }

  /* Disable the digital, ADC, SMPS reference clock divider if fXO > 24MHz or
   * fXO < 26MHz
   */

  ret = spirit_command(spirit, COMMAND_STANDBY);
  if (ret < 0)
    {
      return ret;
    }

  /* Delay for state transition */

  nxsig_usleep(100);

  /* Wait for the device to enter STANDBY */

  ret = spirit_waitstatus(spirit, MC_STATE_STANDBY, 5);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to go to STANDBY state: %d\n", ret);
      return ret;
    }

  if (spirit->xtal_frequency < DOUBLE_XTAL_THR)
    {
      ret = spirit_radio_enable_digdivider(spirit, S_DISABLE);
      DEBUGASSERT(IS_CH_BW(radioinit->bandwidth, spirit->xtal_frequency));
    }
  else
    {
      ret = spirit_radio_enable_digdivider(spirit, S_ENABLE);
      DEBUGASSERT(IS_CH_BW(radioinit->bandwidth,
                 (spirit->xtal_frequency >> 1)));
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

  /* Delay for state transition */

  nxsig_usleep(100);

  /* Make sure that the device becomes READY */

  ret = spirit_waitstatus(spirit, MC_STATE_READY, 5);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to go to READY state: %d\n", ret);
      return ret;
    }

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
                                ((fcoffset * spirit->xtal_frequency) /
                                 FBASE_DIVIDER) +
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

  ret = spirit_radio_convert_chbandwidth(spirit, radioinit->bandwidth,
                                         &bwm, &bwe);
  if (ret < 0)
    {
      return ret;
    }

  digregs[3] = (uint8_t)((bwm << 4) | bwe);

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
 * Name: spirit_radio_get_setup
 *
 * Description:
 *   Returns the SPIRIT analog and digital radio structure according to the
 *   registers value.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   radioinit pointer to a struct radio_init_s thatcontains the
 *              configuration information for the analog radio part of
 *              SPIRIT.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_get_setup(FAR struct spirit_library_s *spirit,
                           FAR struct radio_init_s *radioinit)
{
  uint32_t synthword;
  uint16_t xtaloff;
  int16_t fcoffset;
  uint8_t anaregs[8];
  uint8_t digregs[4];
  uint8_t refdiv;
  uint8_t fdevm;
  uint8_t fdeve;
  uint8_t bwm;
  uint8_t bwe;
  uint8_t divider;
  enum spirit_bandselect_e band;
  int ret;

#if 0
  /* Get the RF board version */

  enum spirit_version_e spirit_version = spirit_general_get_version(spirit);
#endif

  /* Read the Analog Radio registers */

  ret = spirit_reg_read(spirit, SYNT3_BASE, anaregs, 8);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the Digital Radio registers */

  ret = spirit_reg_read(spirit, MOD1_BASE, digregs, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the operating band masking the Band selected field */

  if ((anaregs[3] & 0x07) == SYNT0_BS_6)
    {
      band = HIGH_BAND;
    }
  else if ((anaregs[3] & 0x07) == SYNT0_BS_12)
    {
      band = MIDDLE_BAND;
    }
  else if ((anaregs[3] & 0x07) == SYNT0_BS_16)
    {
      band = LOW_BAND;
    }
  else if ((anaregs[3] & 0x07) == SYNT0_BS_32)
    {
      band = VERY_LOW_BAND;
    }
  else
    {
      uint8_t regval;

      /* If it is another value, set it to a valid one in order to avoid access
       * violation
       */

      regval = (anaregs[3] & 0xf8) | SYNT0_BS_6;
      ret = spirit_reg_write(spirit, SYNT0_BASE, &regval, 1);
      if (ret < 0)
        {
          return ret;
        }

      band = HIGH_BAND;
    }

  /* Compute the synth word */

  synthword = ((((uint32_t)(anaregs[0] & 0x1f)) << 21) +
               (((uint32_t)(anaregs[1])) << 13) +
               (((uint32_t)(anaregs[2])) << 5) +
               (((uint32_t)(anaregs[3])) >> 3));

  /* Calculate the frequency base */

  refdiv = (uint8_t)spirit_radio_get_refdiv(spirit) + 1;
  radioinit->base_frequency = (uint32_t)
    round(synthword * (((double)spirit->xtal_frequency) /
                      (FBASE_DIVIDER * refdiv * g_vectc_bhalf[band])));

  /* Calculate the Offset Factor */

  xtaloff = (((uint16_t)anaregs[6] << 8) | (uint16_t)anaregs[7]);

  /* If a negative number then convert the 12 bit 2-complement in a 16 bit
   * number
   */

  if ((xtaloff & 0x0800) != 0)
    {
      xtaloff = xtaloff | 0xf000;
    }
  else
    {
      xtaloff = xtaloff & 0x0fff;
    }

  fcoffset = (int16_t)xtaloff;

  /* Calculate the frequency offset in ppm */

  radioinit->foffset = (int16_t)
    ((uint32_t)fcoffset * spirit->xtal_frequency * PPM_FACTOR) /
    ((uint32_t)FBASE_DIVIDER * radioinit->base_frequency);

  /* Channel space */

  radioinit->chspace = anaregs[4] * (spirit->xtal_frequency >> 15);

  /* Channel number */

  radioinit->chnum = spirit_radio_get_channel(spirit);

  /* Modulation select */

  radioinit->modselect = (enum modulation_select_e)(digregs[1] & 0x70);

  /* Get the frequency deviation for mantissa and exponent */

  fdevm =  digregs[2] & 0x07;
  fdeve = (digregs[2] & 0xf0) >> 4;

  /* Get the channel filter register for mantissa and exponent */

  bwm = (digregs[3] & 0xf0) >> 4;
  bwe =  digregs[3] & 0x0f;

  divider = spirit_radio_isenabled_digdivider(spirit);

  /* Calculate the datarate */

  radioinit->datarate =
    ((spirit->xtal_frequency >> (5 + divider)) *
     (256 + digregs[0])) >> (23 - (digregs[1] & 0x0f));

  /* Calculates the frequency deviation */

  radioinit->freqdev = (uint32_t)
    ((float)spirit->xtal_frequency / (((uint32_t) 1) << 18) *
     (uint32_t)((8.0 + fdevm) / 2 * (1 << fdeve)));

  /* Get the channel filter bandwidth from the look-up table and return it */

  radioinit->bandwidth = (uint32_t)
    (100.0 * g_vectn_bandwidth[bwm + (bwe * 9)] *
     ((spirit->xtal_frequency >> divider) / 26e6));

  return OK;
}

/******************************************************************************
 * Name: spirit_radio_set_xtalflag
 *
 * Description:
 *   Sets the Xtal configuration in the ANA_FUNC_CONF0 register.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   xtalflag - One of the possible value of the enum type xtal_flag_e.
 *              XTAL_FLAG_24_MHz:  in case of 24 MHz crystal
 *              XTAL_FLAG_26_MHz:  in case of 26 MHz crystal
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

  /* Read the ANA_FUNC_CONF_0 register */

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

  /* Read the Xtal configuration in the ANA_FUNC_CONF_0 register and return
   * the value.
   */

  spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);

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

  /* Read the SYNTH registers, build the synth word and return it */

  spirit_reg_read(spirit, SYNT3_BASE, regvalues, 4);
  return ((((uint32_t)(regvalues[0] & 0x1f)) << 21) +
          (((uint32_t)(regvalues[1])) << 13) +
          (((uint32_t)(regvalues[2])) << 5) +
          (((uint32_t)(regvalues[3])) >> 3));
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

  /* Read the SYNT0 register */

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

  /* Read the SYNT0 register */

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

  /* Read the SYNT0 register */

  spirit_reg_read(spirit, SYNT0_BASE, &regval, 1);

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
  /* Write the CHNUM register */

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

  /* Read the CHNUM register and return the value */

  spirit_reg_read(spirit, CHNUM_BASE, &regval, 1);
  return regval;
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

  /* Read the CHSPACE register, calculate the channel space and return it */

  spirit_reg_read(spirit, CHSPACE_BASE, &factor, 1);

  /* Compute the Hertz value and return it */

  return ((factor * spirit->xtal_frequency) / CHSPACE_DIVIDER);
}

/******************************************************************************
 * Name: spirit_radio_set_foffset_ppm
 *
 * Description:
 *   Sets the FC OFFSET register starting from xtal ppm value.
 *
 * Input Parameters:
 *   spirit      - Reference to a Spirit library state structure instance
 *   xtaloffset - The xtal offset expressed in ppm.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_set_foffset_ppm(FAR struct spirit_library_s *spirit,
                                 int16_t xtaloffset)
{
  enum spirit_bandselect_e band;
  uint32_t synthword;
  uint32_t fbase;
  int32_t offset;
  int16_t fcoffset;
  uint8_t refdiv;
  uint8_t tmp[2];

  /* Get the synth word */

  synthword = spirit_radio_get_synthword(spirit);

  /* Get the operating band */

  band = spirit_radio_get_band(spirit);

  /* Calculate the frequency base */

  refdiv = (uint8_t)spirit_radio_get_refdiv(spirit) + 1;
  fbase  = synthword * (spirit->xtal_frequency /
                        (g_vectc_bhalf[band] * refdiv) /
                        FBASE_DIVIDER);

  /* Calculate the offset respect to RF frequency and according to xtal_ppm
   * parameter.
   */

  offset = (int32_t)(((float)xtaloffset * fbase) / PPM_FACTOR);

  /* Check the Offset is in the correct range */

  DEBUGASSERT(IS_FREQUENCY_OFFSET(offset, spirit->xtal_frequency));

  /* Calculate the FC_OFFSET value to write in the corresponding register */

  fcoffset = (int16_t)(((float)offset * FBASE_DIVIDER) /
                       spirit->xtal_frequency);

  /* Build the array related to the FC_OFFSET_1 and FC_OFFSET_0 register */

  tmp[0] = (uint8_t)((((uint16_t) fcoffset) >> 8) & 0x0f);
  tmp[1] = (uint8_t)fcoffset;

  /* Write the FC_OFFSET registers */

  return spirit_reg_write(spirit, FC_OFFSET1_BASE, tmp, 2);
}

/******************************************************************************
 * Name: spirit_radio_set_foffset_hz
 *
 * Description:
 *   Sets the FC OFFSET register starting from frequency offset expressed in
 *   Hz.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   foffset - Frequency offset expressed in Hz as signed word.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_set_foffset_hz(FAR struct spirit_library_s *spirit,
                                int32_t foffset)
{
  uint8_t tmp[2];
  int16_t offset;

  /* Check that the Offset is in the correct range */

  DEBUGASSERT(IS_FREQUENCY_OFFSET(foffset, spirit->xtal_frequency));

  /* Calculates the offset value to write in the FC_OFFSET register */

  offset = (int16_t)(((float)foffset * FBASE_DIVIDER) /
                     spirit->xtal_frequency);

  /* Build the array related to the FC_OFFSET_1 and FC_OFFSET_0 register */

  tmp[0] = (uint8_t)((((uint16_t) offset) >> 8) & 0x0f);
  tmp[1] = (uint8_t)offset;

  /* Write the FC_OFFSET registers */

  return spirit_reg_write(spirit, FC_OFFSET1_BASE, tmp, 2);
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

  /* Read the FC_OFFSET registers */

  spirit_reg_read(spirit, FC_OFFSET1_BASE, tmp, 2);

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

  /* Read the reference divider */

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
  anaregs[3] = (uint8_t)(((synthword & 0x0000001f) << 3) |
                           g_vectc_bandval[band]);

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
 *   Enable/disable the VCO calibration WA at the end of
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
 * Name: spirit_radio_get_centerfreq
 *
 * Description:
 *   Returns the actual channel center frequency.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Actual channel center frequency expressed in Hz.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_centerfreq(FAR struct spirit_library_s *spirit)
{
  int32_t offset;
  uint8_t channel;
  uint32_t fbase;
  uint32_t chspace;

  /* Get the frequency base */

  fbase = spirit_radio_get_basefrequency(spirit);

  /* Get the frequency offset */

  offset = spirit_radio_get_foffset(spirit);

  /* Get the channel space */

  chspace = spirit_radio_get_chspace(spirit);

  /* Get the channel number */

  channel = spirit_radio_get_channel(spirit);

  /* Calculate the channel center frequency and return it */

  return (uint32_t)(fbase + offset + (uint32_t)(chspace * channel));
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
 *   datarate - datarate expressed in bps. This parameter ranging between
 *              100 and 500000.
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
 * Name: spirit_radio_convert_chbandwidth
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

      delta = 0xffff;

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
  float divtmp = (float)spirit->xtal_frequency / (((uint32_t) 1) << 18);

  /* Check the parameters */

  DEBUGASSERT(IS_F_DEV(fdev, spirit->xtal_frequency));

  for (i = 0; i < 10; i++)
    {
      a = (uint32_t)(divtmp * (uint32_t)(7.5 * (1 << i)));
      if (fdev < a)
        {
          break;
        }
    }

  *pce = i;

  for (i = 0; i < 8; i++)
    {
      bp = b;
      b = (uint32_t)(divtmp * (uint32_t)((8.0 + i) / 2 * (1 << (*pce))));
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
 * Name: spirit_radio_set_datarate
 *
 * Description:
 *   Sets the datarate.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   datarate - Datarate expressed in bps. This value must be in the range
 *              [100 500000].
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errnor value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_set_datarate(FAR struct spirit_library_s *spirit,
                              uint32_t datarate)
{
  uint8_t regval[2];
  uint8_t dre;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_DATARATE(datarate));

  /* Calculate the datarate mantissa and exponent */

  ret = spirit_radio_convert_datarate(spirit, datarate, &regval[0], &dre);
  if (ret >= 0)
    {
      /* Read the MOD_O register */

      spirit_reg_read(spirit, MOD0_BASE, &regval[1], 1);

      /* Mask the other fields and set the datarate exponent */

      regval[1] &= 0xf0;
      regval[1] |= dre;

      /* Write the Datarate registers */

      ret = spirit_reg_write(spirit, MOD1_BASE, regval, 2);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_datarate
 *
 * Description:
 *   Returns the datarate.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled datarate expressed in bps.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_datarate(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];
  uint8_t divider = 0;

  /* Read the datarate registers for mantissa and exponent */

  spirit_reg_read(spirit, MOD1_BASE, regval, 2);

  /* Calculates the datarate */

  divider = (uint8_t) spirit_radio_isenabled_digdivider(spirit);

  return (((spirit->xtal_frequency >> (5 + divider)) *
          (256 + regval[0])) >> (23 - (regval[1] & 0x0f)));
}

/******************************************************************************
 * Name: spirit_radio_set_freqdev
 *
 * Description:
 *   Sets the frequency deviation.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fdev   - Frequency deviation expressed in Hz. Be sure that this value
 *            is in the correct range [F_Xo*8/2^18, F_Xo*7680/2^18] Hz.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errnor value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_set_freqdev(FAR struct spirit_library_s *spirit,
                             uint32_t fdev)
{
  uint8_t fdevm;
  uint8_t fdeve;
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_F_DEV(fdev, spirit->xtal_frequency));

  /* Calculates the frequency deviation mantissa and exponent */

  ret = spirit_radio_convert_freqdev(spirit, fdev, &fdevm, &fdeve);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the FDEV0 register */

  ret = spirit_reg_read(spirit, FDEV0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the other fields and set the frequency deviation mantissa and
       * exponent
       */

      regval &= 0x08;
      regval |= ((fdeve << 4) | (fdevm));

      /* Write the Frequency deviation register */

      ret = spirit_reg_write(spirit, FDEV0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_freqdev
 *
 * Description:
 *   Returns the frequency deviation.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Frequency deviation value expressed in Hz.
 *         This value will be in the range [F_Xo*8/2^18, F_Xo*7680/2^18] Hz.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_freqdev(FAR struct spirit_library_s *spirit)
{
  uint8_t fdevm;
  uint8_t fdeve;
  uint8_t regval;

  /* Read the frequency deviation register for mantissa and exponent */

  spirit_reg_read(spirit, FDEV0_BASE, &regval, 1);
  fdevm = regval & 0x07;
  fdeve = (regval & 0xf0) >> 4;

  /* Calculates the frequency deviation and return it */

  return (uint32_t)((float)spirit->xtal_frequency / (((uint32_t) 1) << 18) *
                    (uint32_t)((8.0 + fdevm) / 2 * (1 << fdeve)));
}

/******************************************************************************
 * Name: spirit_radio_set_chfilterbw
 *
 * Description:
 *   Sets the channel filter bandwidth.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   bandwidth - Channel filter bandwidth expressed in Hz. This parameter
 *               must be in the range [1100 800100].  The API will search the
 *               closest value according to a fixed table of channel bandwidth
 *               values, as defined in the datasheet. To verify the settled
 *               channel bandwidth it is possible to use the
 *               spirit_radio_get_chfilterbw() API.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errnor value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_set_chfilterbw(FAR struct spirit_library_s *spirit,
                                uint32_t bandwidth)
{
  uint8_t bwm;
  uint8_t bwe;
  uint8_t regval;
  int ret;

  /* Search in the channel filter bandwidth table the exponent value */

  if (spirit_radio_isenabled_digdivider(spirit))
    {
      DEBUGASSERT(IS_CH_BW(bandwidth, (spirit->xtal_frequency / 2)));
    }
  else
    {
      DEBUGASSERT(IS_CH_BW(bandwidth, (spirit->xtal_frequency)));
    }

  /* Calculates the channel bandwidth mantissa and exponent */

  ret = spirit_radio_convert_chbandwidth(spirit, bandwidth, &bwm, &bwe);
  if (ret >= 0)
    {
      regval = (bwm << 4) | (bwe);

      /* Write the Channel filter register */

      ret = spirit_reg_write(spirit, CHFLT_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_chfilterbw
 *
 * Description:
 *   Returns the channel filter bandwidth.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Channel filter bandwidth expressed in Hz.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_chfilterbw(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;
  uint8_t bwm;
  uint8_t bwe;

  /* Read the channel filter register for mantissa and exponent */

  spirit_reg_read(spirit, CHFLT_BASE, &regval, 1);
  bwm = (regval & 0xf0) >> 4;
  bwe = regval & 0x0f;

  /* Read the channel filter bandwidth from the look-up table and return it */

  return (uint32_t)(100.0 * g_vectn_bandwidth[bwm + (bwe * 9)] *
                    spirit->xtal_frequency / 26e6);
}

/******************************************************************************
 * Name: spirit_radio_set_modulation
 *
 * Description:
 *   Sets the modulation type.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   modulation - Modulation to set.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errnor value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_set_modulation(FAR struct spirit_library_s *spirit,
                                enum modulation_select_e modulation)
{
  uint8_t regval;

  /* Check the parameters */

  DEBUGASSERT(IS_MODULATION_SELECTED(modulation));

  /* Read the modulation register */

  spirit_reg_read(spirit, MOD0_BASE, &regval, 1);

  /* Mask the other fields and set the modulation type */

  regval &= 0x8f;
  regval |= modulation;

  /* Write the modulation register */

  return spirit_reg_write(spirit, MOD0_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_modulation
 *
 * Description:
 *   Returns the modulation type used.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled modulation type.
 *
 ******************************************************************************/

enum modulation_select_e
  spirit_radio_get_modulation(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the modulation register MOD0 */

  spirit_reg_read(spirit, MOD0_BASE, &regval, 1);

  /* Return the modulation type */

  return (enum modulation_select_e)(regval & 0x70);
}

/******************************************************************************
 * Name: spirit_radio_enable_cwtxmode
 *
 * Description:
 *   Enables or Disables the Continuous Wave transmit mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for power ramping. This parameter can be: S_ENABLE
 *              or S_DISABLE .
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errnor value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_enable_cwtxmode(FAR struct spirit_library_s *spirit,
                                 enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the modulation register MOD0 and mask the CW field */

  ret = spirit_reg_read(spirit, MOD0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= MOD0_CW;
        }
      else
        {
          regval &= (~MOD0_CW);
        }

      /* Write the new value to the MOD0 register */

      ret = spirit_reg_write(spirit, MOD0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_ookpeackdecay
 *
 * Description:
 *   Sets the OOK Peak Decay.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   ookdelay - Peak decay control for OOK.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errnor value is returned on
 *   any failure.
 *
 ******************************************************************************/

int spirit_radio_set_ookpeackdecay(FAR struct spirit_library_s *spirit,
                                    enum spirit_ookpeakdelay_e ookdelay)
{
  uint8_t regval;

  /* Check the parameters */

  DEBUGASSERT(IS_OOK_PEAK_DECAY(ookdelay));

  /* Read the RSSI_FLT register */

  spirit_reg_read(spirit, RSSI_FLT_BASE, &regval, 1);

  /* Mask the other fields and set OOK Peak Decay */

  regval &= 0xfc;
  regval |= ookdelay;

  /* Write the RSSI_FLT register to set the new OOK peak dacay value */

  return spirit_reg_write(spirit, RSSI_FLT_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_ookpeackdecay
 *
 * Description:
 *   Returns the OOK Peak Decay.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Ook peak decay value.
 *
 ******************************************************************************/

enum spirit_ookpeakdelay_e
  spirit_radio_get_ookpeackdecay(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the OOK peak decay register RSSI_FLT_BASE */

  spirit_reg_read(spirit, RSSI_FLT_BASE, &regval, 1);

  /* Returns the OOK peak decay */

  return (enum spirit_ookpeakdelay_e)(regval & 0x03);
}

/******************************************************************************
 * Name: spirit_radio_convert_power2reg
 *
 * Description:
 *   Returns the PA register value that corresponds to the passed dBm power.
 *
 *   NOTE: The power interpolation curves used by this function have been
 *   extracted by measurements done on the divisional evaluation boards.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   fbase    - Frequency base expressed in Hz.
 *   power - Desired power in dBm.
 *
 * Returned Value:
 *   Register value as byte.
 *
 ******************************************************************************/

uint8_t spirit_radio_convert_power2reg(FAR struct spirit_library_s *spirit,
                                       uint32_t fbase, float power)
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
  if (power > 0 &&
      13.0 / g_power_factors[i][2] - g_power_factors[i][3] /
             g_power_factors[i][2] < power)
    {
      j = 0;
    }
  else if (power <= 0 &&
           40.0 / g_power_factors[i][2] - g_power_factors[i][3] /
                  g_power_factors[i][2] > power)
    {
      j = 2;
    }

  pavalue = g_power_factors[i][2 * j] * power +
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
 * Name: spirit_radio_convert_reg2power
 *
 * Description:
 *   Returns the dBm power that corresponds to the value of PA register.
 *
 *   NOTE: The power interpolation curves used by this function have been
 *   extracted by measurements done on the divisional evaluation boards.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fbase  - Frequency base expressed in Hz.
 *   regval - Register value of the PA.
 *
 * Returned Value:
 *   Power in dBm as float.
 *
 ******************************************************************************/

float spirit_radio_convert_reg2power(FAR struct spirit_library_s *spirit,
                                     uint32_t fbase, uint8_t regval)
{
  uint8_t i = 0;
  uint8_t j = 0;
  float power;

  if (regval == 0 || regval > 90)
    {
      return -130.0;
    }

  if (IS_FREQUENCY_BAND_HIGH(fbase))
    {
      i = 0;
      if (fbase < 900000000)
        {
          i = 1; /* 868 */
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
  if (regval < 13)
    {
      j = 0;
    }
  else if (regval > 40)
    {
      j = 2;
    }

  power = (((float)regval) / g_power_factors[i][2 * j] -
           g_power_factors[i][2 * j + 1] / g_power_factors[i][2 * j]);

  return power;
}

/******************************************************************************
 * Name: spirit_radio_config_patable_dbm
 *
 * Description:
 *   Configures the Power Amplifier Table and registers with value expressed
 *   in dBm.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   nlevels - Number of levels to set. This parameter must be in the range
 *             [0:7].
 *   width   - Step width expressed in terms of bit period units Tb/8.  This
 *             parameter must be in the range [1:4].
 *   load    - The of the possible value of the enum type enum
 *             pirit_paload_capacitor_e.
 *
 *               LOAD_0_PF    No additional PA load capacitor
 *               LOAD_1_2_PF  1.2pF additional PA load capacitor
 *               LOAD_2_4_PF  2.4pF additional PA load capacitor
 *               LOAD_3_6_PF  3.6pF additional PA load capacitor
 *
 *   table   - Pointer to an array of PA values in dbm between
 *             [-PA_LOWER_LIMIT: PA_UPPER_LIMIT] dbm.  The first element must
 *             be the lower level (PA_LEVEL[0]) value and the last element
 *             the higher level one (PA_LEVEL[paLevelMaxIndex]).
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_config_patable_dbm(FAR struct spirit_library_s *spirit,
                                    uint8_t nlevels, uint8_t width,
                                    enum spirit_paload_capacitor_e load,
                                    FAR float *table)
{
  uint8_t palevel[9];
  uint8_t regaddr;
  uint8_t value;
  uint32_t fbase = spirit_radio_get_basefrequency(spirit);
  int i;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(nlevels));
  DEBUGASSERT(IS_PA_STEP_WIDTH(width));
  DEBUGASSERT(IS_PA_LOAD_CAP(load));

  /* Check the PA level in dBm is in the range and calculate the PA_LEVEL value
   * to write in the corresponding register using the linearization formula.
   */

  for (i = 0; i <= nlevels; i++)
    {
      DEBUGASSERT(IS_PAPOWER_DBM(*table));
      value = spirit_radio_convert_power2reg(spirit, fbase, *table);
      palevel[nlevels - i] = value;
      table++;
    }

  /* Set the PA_POWER[0] register */

  palevel[nlevels + 1] = load | (width - 1) << 3 | nlevels;

  /* Get the base address */

  regaddr = PA_POWER8_BASE + 7 - nlevels;

  /* Configuresthe PA_POWER registers */

  return spirit_reg_write(spirit, regaddr, palevel, nlevels + 2);
}

/******************************************************************************
 * Name: spirit_radio_get_patable_dbm
 *
 * Description:
 *   Returns the Power Amplifier Table and registers, returning values in dBm.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   nlevels - Pointer to the number of levels settled. This parameter must
 *             be in the range [0:7].
 *   table   - Pointer to an array of 8 elements containing the PA value in
 *             dbm.
 *             The first element will be the PA_LEVEL_0 and the last element
 *             will be PA_LEVEL_7. Any value higher than PA_UPPER_LIMIT
 *             implies no output power (output stage is in high impedance).
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_get_patable_dbm(FAR struct spirit_library_s *spirit,
                                 FAR uint8_t *nlevels, FAR float *table)
{
  uint32_t fbase = spirit_radio_get_basefrequency(spirit);
  uint8_t palevel[9];
  int ret;
  int i;

  /* Reads the PA_LEVEL_x registers and the PA_POWER_0 register */

  ret = spirit_reg_read(spirit, PA_POWER8_BASE, palevel, 9);
  if (ret >= 0)
    {
      /* Fill the PAtable */

      for (i = 7; i >= 0; i--)
        {
          *table++ = spirit_radio_convert_reg2power(spirit, fbase, palevel[i]);
        }

      /* Return the settled index */

      *nlevels = palevel[8] & 0x07;
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_palevel_dbm
 *
 * Description:
 *   Sets a specific PA_LEVEL register, with a value given in dBm.
 *
 *   NOTE: This function makes use of the spirit_radio_convert_power2reg fcn
 *   to interpolate the power value.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ndx    - PA_LEVEL to set. This parameter shall be in the range [0:7].
 *   power  - PA value to write expressed in dBm . Be sure that this values
 *            is in the correct range [-PA_LOWER_LIMIT: PA_UPPER_LIMIT] dBm.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_palevel_dbm(FAR struct spirit_library_s *spirit,
                                 uint8_t ndx, float power)
{
  uint32_t basefrequency;
  uint8_t address;
  uint8_t level;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(ndx));
  DEBUGASSERT(IS_PAPOWER_DBM(power));

  /* Interpolate the power level */

  basefrequency = spirit_radio_get_basefrequency(spirit);
  level         = spirit_radio_convert_power2reg(spirit, basefrequency, power);

  /* Sets the base address */

  address = PA_POWER8_BASE + 7 - ndx;

  /* Configures the PA_LEVEL register */

  return spirit_reg_write(spirit, address, &level, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_palevel_dbm
 *
 * Description:
 *   Returns a specific PA_LEVEL register, returning a value in dBm.
 *
 * NOTE:
 *   This function makes use of the @ref spirit_radio_convert_reg2power fcn
 *   to interpolate the power value.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ndx    - PA_LEVEL to read. This parameter shall be in the range [0:7]
 *
 * Returned Value:
 *   Settled power level expressed in dBm. A value higher than PA_UPPER_LIMIT
 *   dBm implies no output power (output stage is in high impedance).
 *
 ******************************************************************************/

float spirit_radio_get_palevel_dbm(FAR struct spirit_library_s *spirit,
                                   uint8_t ndx)
{
  uint8_t regaddr;
  uint8_t value;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(ndx));

  /* Sets the base address */

  regaddr = PA_POWER8_BASE + 7 - ndx;

  /* Reads the PA_LEVEL[ndx] register */

  spirit_reg_read(spirit, regaddr, &value, 1);

  return spirit_radio_convert_reg2power(spirit,
                                    spirit_radio_get_basefrequency(spirit),
                                    value);
}

/******************************************************************************
 * Name: spirit_radio_config_patable
 *
 * Description:
 *   Configures the Power Amplifier Table and registers.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   nlevels - Number of levels to set. This parameter must be in the range
 *             [0:7].
 *   width   - Step width expressed in terms of bit period units Tb/8.  This
 *             parameter must be in the range [1:4].
 *   load    - One of the possible value of the enum type enum
 *             spirit_paload_capacitor_e.
 *
 *               LOAD_0_PF    No additional PA load capacitor
 *               LOAD_1_2_PF  1.2pF additional PA load capacitor
 *               LOAD_2_4_PF  2.4pF additional PA load capacitor
 *               LOAD_3_6_PF  3.6pF additional PA load capacitor
 *
 *   table   - Pointer to an array of PA values in the range [0: 90], where
 *             0 implies no output power, 1 will be the maximum level and 90
 *             the minimum one. The first element must be the lower level
 *             (PA_LEVEL[0]) value and the last element the higher level one
 *             (PA_LEVEL[paLevelMaxIndex]).
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_config_patable(FAR struct spirit_library_s *spirit,
                                uint8_t nlevels, uint8_t width,
                                enum spirit_paload_capacitor_e load,
                                FAR uint8_t *table)
{
  uint8_t palevel[9];
  uint8_t regaddr;
  int i;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(nlevels));
  DEBUGASSERT(IS_PA_STEP_WIDTH(width));
  DEBUGASSERT(IS_PA_LOAD_CAP(load));

  /* Check the PA levels are in the range */

  for (i = 0; i <= nlevels; i++)
    {
      DEBUGASSERT(IS_PAPOWER(*table));
      palevel[nlevels - i] = *table;
      table++;
    }

  /* Sets the PA_POWER[0] register */

  palevel[nlevels + 1] = load | ((width - 1) << 3) | nlevels;

  /* Sets the base address */

  regaddr = PA_POWER8_BASE + 7 - nlevels;

  /* Configures the PA_POWER registers */

  return spirit_reg_write(spirit, regaddr, palevel, nlevels + 2);
}

/******************************************************************************
 * Name: spirit_radio_get_patable
 *
 * Description:
 *   Returns the Power Amplifier Table and registers.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   nlevels - Pointer to the number of levels settled.  This parameter must
 *             be in the range [0:7].
 *   table   - Pointer to an array of 8 elements containing the PA value. The
 *             first element will be the PA_LEVEL_0 and the last element will
 *             be PA_LEVEL_7. Any value equals to 0 implies that level has no
 *             output power (output stage is in high impedance).
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_get_patable(FAR struct spirit_library_s *spirit,
                             FAR uint8_t *nlevels, FAR uint8_t *table)
{
  uint8_t palevels[9];
  int ret;
  int i;

  /* Reads the PA_LEVEL_x registers and the PA_POWER_0 register */

  ret = spirit_reg_read(spirit, PA_POWER8_BASE, palevels, 9);
  if (ret >= 0)
    {
      /* Fill the PAtable */

      for (i = 7; i >= 0; i--)
        {
          *table++ = palevels[i];
        }

      /* Return the settled index */

      *nlevels = palevels[8] & 0x07;
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_palevel
 *
 * Description:
 *   Sets a specific PA_LEVEL register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ndx    - PA_LEVEL to set. This parameter shall be in the range [0:7].
 *   power  - PA value to write in the register. Be sure that this values is
 *            in the correct range [0 : 90].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_palevel(FAR struct spirit_library_s *spirit,
                             uint8_t ndx, uint8_t power)
{
  uint8_t regaddr;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(ndx));
  DEBUGASSERT(IS_PAPOWER(power));

  /* Sets the base address */

  regaddr = PA_POWER8_BASE + 7 - ndx;

  /* Configures the PA_LEVEL register */

  return spirit_reg_write(spirit, regaddr, &power, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_palevel
 *
 * Description:
 *   Returns a specific PA_LEVEL register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ndx    - PA_LEVEL to read. This parameter shall be in the range [0:7].
 *
 * Returned Value:
 *   PA_LEVEL value. A value equal to zero
 *         implies no output power (output stage is in high impedance).
 *
 ******************************************************************************/

uint8_t spirit_radio_get_palevel(FAR struct spirit_library_s *spirit,
                                 uint8_t ndx)
{
  uint8_t regaddr;
  uint8_t regval;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_MAX_INDEX(ndx));

  /* Sets the base address */

  regaddr = PA_POWER8_BASE + 7 - ndx;

  /* Reads the PA_LEVEL[ndx] register and return the value */

  spirit_reg_read(spirit, regaddr, &regval, 1);
  return regval;
}

/******************************************************************************
 * Name: spirit_radio_set_outputload
 *
 * Description:
 *   Sets the output stage additional load capacitor bank.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   load one of the possible value of the enum type enum
 *   spirit_paload_capacitor_e.
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

  /* Read the PA_POWER_0 register */

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

  /* Read the PA_POWER_0 register */

  spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);

  /* Mask the CWC[1:0] field and return the value */

  return (enum spirit_paload_capacitor_e)(regval & 0xc0);
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

  /* Read the PA_POWER_0 register */

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
 * Name: spirit_radio_get_palevel_maxindex
 *
 * Description:
 *   Returns the actual PA_LEVEL_MAX_INDEX.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Actual PA_LEVEL_MAX_INDEX. This value will be in the range [0:7].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_palevel_maxindex(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PA_POWER_0 register */

  spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);

  /* Mask the PA_LEVEL_MAX_INDEX[1:0] field and return the value */

  return (regval & 0x07);
}

/******************************************************************************
 * Name: spirit_radio_set_pastep_width
 *
 * Description:
 *   Sets a specific PA_RAMP_STEP_WIDTH.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   width  - Step width expressed in terms of bit period units Tb/8.  This
 *            value must be in the range [1:4].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_pastep_width(FAR struct spirit_library_s *spirit,
                                  uint8_t width)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PA_STEP_WIDTH(width));

  /* Read the PA_POWER_0 register */

  ret = spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the PA_RAMP_STEP_WIDTH[1:0] field and write the new value */

      regval &= 0xe7;
      regval |= (width - 1) << 3;

      /* Configures the PA_POWER_0 register */

      ret = spirit_reg_write(spirit, PA_POWER0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_pastep_width
 *
 * Description:
 *   Returns the actual PA_RAMP_STEP_WIDTH.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Step width value expressed in terms of bit period units Tb/8.  This
 *   value will be in the range [1:4].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_pastep_width(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PA_POWER_0 register */

  spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);

  /* Mask the PA_RAMP_STEP_WIDTH[1:0] field and return the value */

  regval &= 0x18;
  return ((regval >> 3) + 1);
}

/******************************************************************************
 * Name: spirit_radio_enable_paramp
 *
 * Description:
 *   Enables or Disables the Power Ramping.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for power ramping.  This value can be: S_ENABLE
 *              or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_enable_paramp(FAR struct spirit_library_s *spirit,
                               enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the PA_POWER_0 register and configure the PA_RAMP_ENABLE field */

  ret = spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= PA_POWER0_PA_RAMP_MASK;
        }
      else
        {
          regval &= (~PA_POWER0_PA_RAMP_MASK);
        }

      /* Set the PA_POWER_0 register */

      ret = spirit_reg_write(spirit, PA_POWER0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_isenabled_paramp
 *
 * Description:
 *   Returns the state of the Power Ramping enable bit.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for power ramping. This value can be: S_ENABLE
 *              or S_DISABLE.
 *
 * Returned Value:
 *   Power Ramping enable state.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_radio_isenabled_paramp(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PA_POWER_0 register and configure the PA_RAMP_ENABLE field */

  spirit_reg_read(spirit, PA_POWER0_BASE, &regval, 1);

  /* Mask and return data */

  return (enum spirit_functional_state_e)((regval >> 5) & 0x01);
}

/******************************************************************************
 * Name: spirit_radio_enable_afc
 *
 * Description:
 *   Enables or Disables the AFC.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for AFC. This value can be: S_ENABLE or
 *              S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_enable_afc(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the AFC_2 register and configure the AFC Enabled field */

  ret = spirit_reg_read(spirit, AFC2_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AFC2_AFC_MASK;
        }
      else
        {
          regval &= (~AFC2_AFC_MASK);
        }

      /* Set the AFC_2 register */

      ret = spirit_reg_write(spirit, AFC2_BASE, &regval, 1);
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

  /* Read the AFC_2 register and configure the AFC Freeze on Sync field */

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
 * Name: spirit_radio_set_afcmode
 *
 * Description:
 *   Sets the AFC working mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   mode   - The AFC mode. This value can be one of the values defined in
 *            enum spirit_afcmode_e:
 *
 *              AFC_SLICER_CORRECTION  AFC loop closed on slicer
 *              AFC_2ND_IF_CORRECTION  AFC loop closed on 2nd conversion stage
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_afcmode(FAR struct spirit_library_s *spirit,
                             enum spirit_afcmode_e mode)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_AFC_MODE(mode));

  /* Read the AFC_2 register and configure the AFC Mode field */

  ret = spirit_reg_read(spirit, AFC2_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (mode == AFC_2ND_IF_CORRECTION)
        {
          regval |= AFC_2ND_IF_CORRECTION;
        }
      else
        {
          regval &= (~AFC_2ND_IF_CORRECTION);
        }

      /* Set the AFC_2 register */

      ret = spirit_reg_write(spirit, AFC2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_afcmode
 *
 * Description:
 *   Returns the AFC working mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled AFC mode. This value will be one of the values defined in
 *   enum spirit_afcmode_e:
 *
 *     AFC_SLICER_CORRECTION  AFC loop closed on slicer
 *     AFC_2ND_IF_CORRECTION  AFC loop closed on 2nd conversion stage
 *
 ******************************************************************************/

enum spirit_afcmode_e
  spirit_radio_get_afcmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AFC_2 register */

  spirit_reg_read(spirit, AFC2_BASE, &regval, 1);

  /* Mask the AFC Mode field and returns the value */

  return (enum spirit_afcmode_e)(regval & 0x20);
}

/******************************************************************************
 * Name: spirit_radio_set_afcpdleakage
 *
 * Description:
 *   Sets the AFC peak detector leakage.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   leakage - The peak detector leakage. This value must be in the
 *             range: [0:31].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_afcpdleakage(FAR struct spirit_library_s *spirit,
                                  uint8_t leakage)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_AFC_PD_LEAKAGE(leakage));

  /* Read the AFC_2 register and configure the AFC PD leakage field */

  ret = spirit_reg_read(spirit, AFC2_BASE, &regval, 1);
  if (ret >= 0)
    {
      regval &= 0xe0;
      regval |= leakage;

      /* Set the AFC_2 register */

      ret = spirit_reg_write(spirit, AFC2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_afcpdleakage
 *
 * Description:
 *   Returns the AFC peak detector leakage.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Peak detector leakage value. This value will be in the range: [0:31].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_afcpdleakage(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AFC_2 register */

  spirit_reg_read(spirit, AFC2_BASE, &regval, 1);

  /* Mask the AFC PD leakage field and return the value */

  return (regval & 0x1f);
}

/******************************************************************************
 * Name: spirit_radio_set_afcfastperiod
 *
 * Description:
 *   Sets the length of the AFC fast period expressed as number of samples.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   length - Length of the fast period in number of samples.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_afcfastperiod(FAR struct spirit_library_s *spirit,
                                   uint8_t length)
{
  /* Set the AFC_1 register */

  return spirit_reg_write(spirit, AFC1_BASE, &length, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_afcfastperiod
 *
 * Description:
 *   Returns the AFC fast period expressed as number of samples.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Length of the fast period in number of samples.
 *
 ******************************************************************************/

uint8_t spirit_radio_get_afcfastperiod(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AFC 1 register and return the value */

  spirit_reg_read(spirit, AFC1_BASE, &regval, 1);

  return regval;
}

/******************************************************************************
 * Name: spirit_radio_set_afcfastgain
 *
 * Description:
 *   Sets the AFC loop gain in fast mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   gain   - AFC loop gain in fast mode. This value must be in the range:
 *            [0:15].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_afcfastgain(FAR struct spirit_library_s *spirit,
                                 uint8_t gain)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_AFC_FAST_GAIN(gain));

  /* Read the AFC_0 register and configure the AFC Fast Gain field */

  ret = spirit_reg_read(spirit, AFC0_BASE, &regval, 1);
  if (ret >= 0)
    {
      regval &= 0x0f;
      regval |= gain << 4;

      /* Set the AFC_0 register */

      ret = spirit_reg_write(spirit, AFC0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_afcfastgain
 *
 * Description:
 *   Returns the AFC loop gain in fast mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   uint8_t AFC loop gain in fast mode. This value will be in the range:
 *         [0:15].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_afcfastgain(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AFC_0 register, mask the AFC Fast Gain field and return the
   * value.
   */

  spirit_reg_read(spirit, AFC0_BASE, &regval, 1);

  return ((regval & 0xf0) >> 4);
}

/******************************************************************************
 * Name: spirit_radio_set_afclowgain
 *
 * Description:
 *   Sets the AFC loop gain in slow mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   gain   - AFC loop gain in slow mode. This value must be in the range:
 *            [0:15].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_afclowgain(FAR struct spirit_library_s *spirit,
                                uint8_t gain)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_AFC_SLOW_GAIN(gain));

  /* Read the AFC_0 register and configure the AFC Slow Gain field */

  ret = spirit_reg_read(spirit, AFC0_BASE, &regval, 1);
  if (ret >= 0)
    {
      regval &= 0xf0;
      regval |= gain;

      /* Set the AFC_0 register */

      ret = spirit_reg_write(spirit, AFC0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_afclowgain
 *
 * Description:
 *   Returns the AFC loop gain in slow mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AFC loop gain in slow mode. This value will be in the range: [0:15].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_afclowgain(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AFC_0 register, mask the AFC Slow Gain field and return the
   * value.
   */

  spirit_reg_read(spirit, AFC0_BASE, &regval, 1);

  return (regval & 0x0f);
}

/******************************************************************************
 * Name: spirit_radio_get_afccorrection
 *
 * Description:
 *   Returns the AFC correction from the corresponding register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AFC correction, read from the corresponding register.  This value
 *   will be in the range [-128:127].
 *
 ******************************************************************************/

int8_t spirit_radio_get_afccorrection(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AFC_CORR register, cast the read value as signed char and return
   * it.
   */

  spirit_reg_read(spirit, AFC_CORR_BASE, &regval, 1);

  return (int8_t)regval;
}

/******************************************************************************
 * Name: spirit_radio_get_afccorrection_hz
 *
 * Description:
 *   Returns the AFC correction expressed in Hz.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AFC correction expressed in Hz according to the following formula:
 *
 *     Fafc[Hz]= (Fdig/(12*2^10))*AFC_CORR
 *
 *   where  AFC_CORR is the value read in the AFC_CORR register
 *
 ******************************************************************************/

int32_t spirit_radio_get_afccorrection_hz(FAR struct spirit_library_s *spirit)
{
  int8_t correction;
  uint32_t xtal = spirit->xtal_frequency;

  /* Read the AFC correction register */

  correction = spirit_radio_get_afccorrection(spirit);

  if (xtal > DOUBLE_XTAL_THR)
    {
      xtal /= 2;
    }

  /* Calculates and return the Frequency Correction */

  return (int32_t)(xtal / (12 * pow(2, 10)) * correction);
}

/******************************************************************************
 * Name: spirit_radio_enable_agc
 *
 * Description:
 *   Enables or Disables the AGC.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for AGC.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_enable_agc(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the AGCCTRL_0 register and configure the AGC Enabled field */

  ret = spirit_reg_read(spirit, AGCCTRL0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AGCCTRL0_AGC_MASK;
        }
      else
        {
          regval &= (~AGCCTRL0_AGC_MASK);
        }

      /* Set the AGCCTRL_0 register */

      ret = spirit_reg_write(spirit, AGCCTRL0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_agcmode
 *
 * Description:
 *   Sets the AGC working mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   mode   - The AGC mode. This value can be one of the values defined in
 *            enum spirit_agcmode_e:
 *
 *              AGC_LINEAR_MODE  AGC works in linear mode
 *              AGC_BINARY_MODE  AGC works in binary mode
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_agcmode(FAR struct spirit_library_s *spirit,
                             enum spirit_agcmode_e mode)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_AGC_MODE(mode));

  /* Read the AGCCTRL_0 register and configure the AGC Mode field */

  ret = spirit_reg_read(spirit, AGCCTRL0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (mode == AGC_BINARY_MODE)
        {
          regval |= AGC_BINARY_MODE;
        }
      else
        {
          regval &= (~AGC_BINARY_MODE);
        }

      /* Set the AGCCTRL_0 register */

      ret = spirit_reg_write(spirit, AGCCTRL0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_agcmode
 *
 * Description:
 *   Returns the AGC working mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled AGC mode.  This value can be one of the values defined in
 *   enum spirit_agcmode_e:
 *
 *     AGC_LINEAR_MODE  AGC works in linear mode
 *     AGC_BINARY_MODE  AGC works in binary mode
 *
 ******************************************************************************/

enum spirit_agcmode_e
  spirit_radio_get_agcmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AGCCTRL_0 register, mask the AGC Mode field and return the
   * value
   */

  spirit_reg_read(spirit, AGCCTRL0_BASE, &regval, 1);

  return (enum spirit_agcmode_e)(regval & 0x40);
}

/******************************************************************************
 * Name: spirit_radio_enable_agcfreeze_steady
 *
 * Description:
 *   Enables or Disables the AGC freeze on steady state.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for AGC freeze on steady state.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_enable_agcfreeze_steady(FAR struct spirit_library_s *spirit,
                                     enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the AGCCTRL_2 register and configure the AGC Freeze On Steady field */

  ret = spirit_reg_read(spirit, AGCCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AGCCTRL2_FREEZE_ON_STEADY_MASK;
        }
      else
        {
          regval &= (~AGCCTRL2_FREEZE_ON_STEADY_MASK);
        }

      /* Set the AGCCTRL_2 register */

      ret = spirit_reg_write(spirit, AGCCTRL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_enable_agcfreeze_sync
 *
 * Description:
 *   Enable or Disable the AGC freeze on sync detection.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for AGC freeze on sync detection.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_enable_agcfreeze_sync(FAR struct spirit_library_s *spirit,
                                       enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the AGCCTRL_2 register and configure the AGC Freeze On Sync field */

  ret = spirit_reg_read(spirit, AGCCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AGCCTRL2_FREEZE_ON_SYNC_MASK;
        }
      else
        {
          regval &= (~AGCCTRL2_FREEZE_ON_SYNC_MASK);
        }

      /* Set the AGCCTRL_2 register */

      ret = spirit_reg_write(spirit, AGCCTRL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_enable_agcfreeze_maxatten
 *
 * Description:
 *   Enable or Disable the AGC to start with max attenuation.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for AGC start with max attenuation mode.
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_enable_agcfreeze_maxatten(FAR struct spirit_library_s *spirit,
                                       enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the AGCCTRL_2 register and configure the AGC Start Max Attenuation
   * field.
   */

  ret = spirit_reg_read(spirit, AGCCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AGCCTRL2_START_MAX_ATTENUATION_MASK;
        }
      else
        {
          regval &= (~AGCCTRL2_START_MAX_ATTENUATION_MASK);
        }

      /* Set the AGCCTRL_2 register */

      ret = spirit_reg_write(spirit, AGCCTRL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_agcmeasure_us
 *
 * Description:
 *   Sets the AGC measure time.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   time   - AGC measure time expressed in us. This value must be in the
 *            range [0, 393216/F_Xo].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_agcmeasure_us(FAR struct spirit_library_s *spirit,
                                   uint16_t time)
{
  uint8_t regval;
  uint8_t measure;
  int ret;

  /* Check the parameter */

  DEBUGASSERT(IS_AGC_MEASURE_TIME_US(time, spirit->xtal_frequency));

  /* Read the AGCCTRL_2 register */

  ret = spirit_reg_read(spirit, AGCCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Calculates the measure time value to write in the register */

      measure = (uint8_t)
        lroundf(log2((float)time / 1e6 * spirit->xtal_frequency / 12));

      if (measure > 15)
        {
          measure = 15;
        }

      /* Mask the MEAS_TIME field and write the new value */

      regval &= 0xf0;
      regval |= measure;

      /* Set the AGCCTRL_2 register */

      ret = spirit_reg_write(spirit, AGCCTRL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_agcmeasure_us
 *
 * Description:
 *   Returns the AGC measure time in microseconds.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AGC measure time expressed in microseconds. This value will be in
 *   the range [0, 393216/Fxo].
 *
 ******************************************************************************/

uint16_t spirit_radio_get_agcmeasure_us(FAR struct spirit_library_s *spirit)
{
  uint8_t measure;

  /* Read the AGCCTRL_2 register */

  spirit_reg_read(spirit, AGCCTRL2_BASE, &measure, 1);

  /* Mask the MEAS_TIME field */

  measure &= 0x0f;

  /* Calculates the measure time value to write in the register */

  return (uint16_t)((12.0 / spirit->xtal_frequency) *
                    (float)pow(2, measure) * 1e6);
}

/******************************************************************************
 * Name: spirit_radio_set_agcmeasure
 *
 * Description:
 *   Sets the AGC measure time.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   time   - AGC measure time to write in the MEAS_TIME field of AGCCTRL_2
 *            register.  This value must be in the range [0:15].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_agcmeasure(FAR struct spirit_library_s *spirit,
                                uint8_t time)
{
  uint8_t regval;
  int ret;

  /* Check the parameter */

  DEBUGASSERT(IS_AGC_MEASURE_TIME(time));

  /* Read the AGCCTRL_2 register */

  ret = spirit_reg_read(spirit, AGCCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the MEAS_TIME field and write the new value */

      regval &= 0xf0;
      regval |= time;

      /* Set the AGCCTRL_2 register */

      ret = spirit_reg_write(spirit, AGCCTRL2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_agcmeasure
 *
 * Description:
 *   Returns the AGC measure time.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AGC measure time read from the MEAS_TIME field of AGCCTRL_2 register.
 *   This value will be in the range [0:15].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_agcmeasure(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AGCCTRL_2 register, mask the MEAS_TIME field and return the
   * value.
   */

  spirit_reg_read(spirit, AGCCTRL2_BASE, &regval, 1);

  return (regval & 0x0f);
}

/******************************************************************************
 * Name: spirit_radio_set_agcholdtime_us
 *
 * Description:
 *   Sets the AGC hold time.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   time   - AGC hold time expressed in us. This value must be in the
 *            range [0, 756/F_Xo].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_agcholdtime_us(FAR struct spirit_library_s *spirit,
                                    uint8_t time)
{
  uint8_t regval;
  uint8_t hold;

  /* Check the parameter */

  DEBUGASSERT(IS_AGC_HOLD_TIME_US(time, spirit->xtal_frequency));

  /* Read the AGCCTRL_0 register */

  spirit_reg_read(spirit, AGCCTRL0_BASE, &regval, 1);

  /* Calculates the hold time value to write in the register */

  hold = (uint8_t) lroundf(((float)time / 1e6 * spirit->xtal_frequency) / 12);
  (hold > 63) ? (hold = 63) : (hold);

  /* Mask the HOLD_TIME field and write the new value */

  regval &= 0xc0;
  regval |= hold;

  /* Set the AGCCTRL_0 register */

  return spirit_reg_write(spirit, AGCCTRL0_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_agcholdtime_us
 *
 * Description:
 *   Returns the AGC hold time.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AGC hold time expressed in us. This value will be in the range:
 *   [0, 756/F_Xo].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_agcholdtime_us(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AGCCTRL_0 register */

  spirit_reg_read(spirit, AGCCTRL0_BASE, &regval, 1);

  /* Mask the HOLD_TIME field */

  regval &= 0x3f;

  /* Calculates the hold time value and return it */

  return (uint8_t) lroundf((12.0 / spirit->xtal_frequency) * (regval * 1e6));
}

/******************************************************************************
 * Name: spirit_radio_set_agcholdtime
 *
 * Description:
 *   Sets the AGC hold time.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   time   - AGC hold time to write in the HOLD_TIME field of AGCCTRL_0
 *            register.  This value must be in the range [0:63].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_agcholdtime(FAR struct spirit_library_s *spirit,
                                 uint8_t time)
{
  uint8_t regval;
  int ret;

  /* Check the parameter */

  DEBUGASSERT(IS_AGC_HOLD_TIME(time));

  /* Read the AGCCTRL_0 register */

  ret = spirit_reg_read(spirit, AGCCTRL0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the HOLD_TIME field and write the new value */

      regval &= 0xc0;
      regval |= time;

      /* Set the AGCCTRL_0 register */

      ret = spirit_reg_write(spirit, AGCCTRL0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_get_agcholdtime
 *
 * Description:
 *   Returns the AGC hold time.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AGC hold time read from the HOLD_TIME field of AGCCTRL_0 register.  This
 *   value will be in the range [0:63].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_agcholdtime(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AGCCTRL_0 register, mask the MEAS_TIME field and return the
   * value.
   */

  spirit_reg_read(spirit, AGCCTRL0_BASE, &regval, 1);

  return (regval & 0x3f);
}

/******************************************************************************
 * Name: spirit_radio_set_agchighthres
 *
 * Description:
 *   Sets the AGC high threshold.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   highthres - AGC high threshold to write in the THRESHOLD_HIGH field of
 *               AGCCTRL_1 register.  This value must be in the range
 *               [0:15].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_agchighthres(FAR struct spirit_library_s *spirit,
                                  uint8_t highthres)
{
  uint8_t regval;

  /* Check the parameter */

  DEBUGASSERT(IS_AGC_THRESHOLD(highthres));

  /* Read the AGCCTRL_1 register */

  spirit_reg_read(spirit, AGCCTRL1_BASE, &regval, 1);

  /* Mask the THRESHOLD_HIGH field and write the new value */

  regval &= 0x0f;
  regval |= highthres << 4;

  /* Set the AGCCTRL_1 register */

  return spirit_reg_write(spirit, AGCCTRL1_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_agchighthres
 *
 * Description:
 *   Returns the AGC high threshold.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AGC high threshold read from the THRESHOLD_HIGH field of AGCCTRL_1
 *   register.  This value will be in the range [0:15].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_agchighthres(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AGCCTRL_1 register, mask the THRESHOLD_HIGH field and return the
   * value.
   */

  spirit_reg_read(spirit, AGCCTRL1_BASE, &regval, 1);

  return ((regval & 0xf0) >> 4);
}

/******************************************************************************
 * Name: spirit_radio_set_agclowthres
 *
 * Description:
 *   Sets the AGC low threshold.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   lowthres - AGC low threshold to write in the THRESHOLD_LOW field of
 *              AGCCTRL_1 register. This value must be in the range
 *              0:15].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_agclowthres(FAR struct spirit_library_s *spirit,
                                 uint8_t lowthres)
{
  uint8_t regval;

  /* Check the parameter */

  DEBUGASSERT(IS_AGC_THRESHOLD(lowthres));

  /* Read the AGCCTRL_1 register */

  spirit_reg_read(spirit, AGCCTRL1_BASE, &regval, 1);

  /* Mask the THRESHOLD_LOW field and write the new value */

  regval &= 0xf0;
  regval |= lowthres;

  /* Set the AGCCTRL_1 register */

  return spirit_reg_write(spirit, AGCCTRL1_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_agclowthres
 *
 * Description:
 *   Returns the AGC low threshold.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   AGC low threshold read from the THRESHOLD_LOW field of AGCCTRL_1 register.
 *   This value will be in the range [0:15].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_agclowthres(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the AGCCTRL_1 register, mask the THRESHOLD_LOW field and return the
   * value.
   */

  spirit_reg_read(spirit, AGCCTRL1_BASE, &regval, 1);

  return (regval & 0x0f);
}

/******************************************************************************
 * Name: spirit_radio_set_clkrecmode
 *
 * Description:
 *   Sets the clock recovery algorithm.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   mode   - The Clock Recovery mode. This value can be one of the values
 *            defined in enum spirit_clkrecmode_e :
 *
 *              CLK_REC_PLL  PLL algorithm for clock recovery
 *              CLK_REC_DLL  DLL algorithm for clock recovery
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_clkrecmode(FAR struct spirit_library_s *spirit,
                                enum spirit_clkrecmode_e mode)
{
  uint8_t regval;

  /* Check the parameter */

  DEBUGASSERT(IS_CLK_REC_MODE(mode));

  /* Read the FDEV_0 register */

  spirit_reg_read(spirit, FDEV0_BASE, &regval, 1);

  /* Mask the CLOCK_REC_ALGO_SEL field and write the new value */

  regval &= 0xf7;
  regval |= (uint8_t) mode;

  /* Set the FDEV_0 register */

  return spirit_reg_write(spirit, FDEV0_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_clkrecmode
 *
 * Description:
 *   Returns the Clock Recovery working mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Clock Recovery mode. This value can be one of the values defined in
 *   enum spirit_clkrecmode_e:
 *
 *     CLK_REC_PLL  PLL algorithm for clock recovery
 *     CLK_REC_DLL  DLL algorithm for clock recovery
 *
 ******************************************************************************/

enum spirit_clkrecmode_e
  spirit_radio_get_clkrecmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the FDEV_0 register, mask the CLOCK_REC_ALGO_SEL field and return
   * the value.
   */

  spirit_reg_read(spirit, FDEV0_BASE, &regval, 1);

  return (enum spirit_clkrecmode_e)(regval & 0x08);
}

/******************************************************************************
 * Name: spirit_radio_set_clkrecgain
 *
 * Description:
 *   Sets the clock recovery proportional gain.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   pgain  - The Clock Recovery proportional gain to write in the
 *            CLK_REC_P_GAIN field of CLOCKREC register.  This is the log2
 *            value of the clock recovery proportional gain.  This value
 *            must be in the range [0:7].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_clkrecgain(FAR struct spirit_library_s *spirit,
                                uint8_t pgain)
{
  uint8_t regval;

  /* Check the parameter */

  DEBUGASSERT(IS_CLK_REC_P_GAIN(pgain));

  /* Read the CLOCKREC register */

  spirit_reg_read(spirit, CLOCKREC_BASE, &regval, 1);

  /* Mask the CLK_REC_P_GAIN field and write the new value */

  regval &= 0x1f;
  regval |= (pgain << 5);

  /* Set the CLOCKREC register */

  return spirit_reg_write(spirit, CLOCKREC_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_clkrecgain
 *
 * Description:
 *   Returns the log2 of the clock recovery proportional gain.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Clock Recovery proportional gain read from the CLK_REC_P_GAIN field of
 *   CLOCKREC register.  This value will be in the range [0:7].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_clkrecgain(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the CLOCKREC register, mask the CLK_REC_P_GAIN field and return the
   * value.
   */

  spirit_reg_read(spirit, CLOCKREC_BASE, &regval, 1);

  return ((regval & 0xef) >> 5);
}

/******************************************************************************
 * Name: spirit_radio_set_clkrecigain
 *
 * Description:
 *   Sets the clock recovery integral gain.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   igain  - The Clock Recovery integral gain to write in the CLK_REC_I_GAIN
 *            field of CLOCKREC register. This value must be in the range
 *            [0:15].
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_clkrecigain(FAR struct spirit_library_s *spirit,
                                 uint8_t igain)
{
  uint8_t regval;

  /* Check the parameter */

  DEBUGASSERT(IS_CLK_REC_I_GAIN(igain));

  /* Read the CLOCKREC register */

  spirit_reg_read(spirit, CLOCKREC_BASE, &regval, 1);

  /* Mask the CLK_REC_P_GAIN field and write the new value */

  regval &= 0xf0;
  regval |= igain;

  /* Set the CLOCKREC register */

  return spirit_reg_write(spirit, CLOCKREC_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_clkrecigain
 *
 * Description:
 *   Returns the clock recovery integral gain.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Clock Recovery integral gain read from the CLK_REC_I_GAIN field of
 *   CLOCKREC register.  This value will be in the range [0:15].
 *
 ******************************************************************************/

uint8_t spirit_radio_get_clkrecigain(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the CLOCKREC register, mask the CLK_REC_I_GAIN field and return the
   * value.
   */

  spirit_reg_read(spirit, CLOCKREC_BASE, &regval, 1);

  return (regval & 0x0f);
}

/******************************************************************************
 * Name: spirit_radio_set_clkrecpstfltlen
 *
 * Description:
 *   Sets the postfilter length for clock recovery algorithm.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   length - The postfilter length in symbols. This value can be one of
 *            the values defined in enum spirit_pstfltlen_e :
 *
 *              PSTFLT_LENGTH_8   Postfilter length is 8 symbols
 *              PSTFLT_LENGTH_16  Postfilter length is 16 symbols
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_clkrecpstfltlen(FAR struct spirit_library_s *spirit,
                                     enum spirit_pstfltlen_e length)
{
  uint8_t regval;

  /* Check the parameter */

  DEBUGASSERT(IS_PST_FLT_LENGTH(length));

  /* Read the CLOCKREC register */

  spirit_reg_read(spirit, CLOCKREC_BASE, &regval, 1);

  /* Mask the PSTFLT_LEN field and write the new value */

  regval &= 0xef;
  regval |= (uint8_t) length;

  /* Set the CLOCKREC register */

  return spirit_reg_write(spirit, CLOCKREC_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_radio_get_clkrecpstfltlen
 *
 * Description:
 *   Returns the postfilter length for clock recovery algorithm.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Postfilter length in symbols. This value can be one of the values
 *   defined in enum spirit_pstfltlen_e:
 *
 *     PSTFLT_LENGTH_8   Postfilter length is 8 symbols
 *     PSTFLT_LENGTH_16  Postfilter length is 16 symbols
 *
 ******************************************************************************/

enum spirit_pstfltlen_e
  spirit_radio_get_clkrecpstfltlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the CLOCKREC register, mask the PSTFLT_LEN field and return the
   * value.
   */

  spirit_reg_read(spirit, CLOCKREC_BASE, &regval, 1);

  return (enum spirit_pstfltlen_e)(regval & 0x10);
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

  /* Read the ANT_SELECT_CONF_BASE and mask the CS_BLANKING BIT field */

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

  /* Read the PROTOCOL0_BASE and mask the PROTOCOL0_PERS_RX_MASK bitfield */

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

      /* Write the new value to the PROTOCOL0_BASE register */

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

  /* Read the SYNTH_CONFIG1_BASE and mask the REFDIV bit field */

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

      /* Write the new value to the SYNTH_CONFIG1_BASE register */

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

  spirit_reg_read(spirit, SYNTH_CONFIG1_BASE, &regval, 1);

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

  /* Read the XO_RCO_TEST_BASE and mask the PD_CLKDIV bit field */

  ret = spirit_reg_read(spirit, XO_RCO_TEST_BASE, &regval, 1);
  if (ret >= 0)
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

  spirit_reg_read(spirit, XO_RCO_TEST_BASE, &regval, 1);

  if (((regval >> 3) & 0x1) != 0)
    {
      return S_DISABLE;
    }
  else
    {
      return S_ENABLE;
    }
}
