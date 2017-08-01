/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_radio.h
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_RADIO_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_RADIO_H

/* In order to configure the Radio main parameters, the user can fit struct
 * radio_init_s structure the and call the spirit_radio_initialize()
 * function passing its pointer* as an argument.
 *
 * Example:
 *
 * struct radio_init_s g_radio_init =
 * {
 *    433.4e6,                 # base frequency
 *    20e3,                    # channel space
 *    0,                       # Xtal offset in ppm
 *    0,                       # channel number
 *    FSK,                     # modulation select
 *    38400,                   # datarate
 *    20e3,                    # frequency deviation
 *    100.5e3                  # channel filter bandwidth
 * };
 *
 * ...
 *
 * spirit_radio_initialize(spirit, &g_radio_init);
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <stdint.h>
#include <math.h>

#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Radio_Band */

#define FBASE_DIVIDER               262144  /* 2^18 factor dividing fxo in fbase
                                             * formula */

#define HIGH_BAND_FACTOR            6       /* Band select factor for high band.
                                             * Factor B in the equation 2 */
#define MIDDLE_BAND_FACTOR          12      /* Band select factor for middle
                                             * band. Factor B in the equation 2 */
#define LOW_BAND_FACTOR             16      /* Band select factor for low band.
                                             * Factor B in the equation 2 */
#define VERY_LOW_BAND_FACTOR        32      /* Band select factor for very low
                                             * band. Factor B in the equation 2 */

#define HIGH_BAND_LOWER_LIMIT       778000000  /* Lower limit of the high
                                                * band: 779 MHz */
#define HIGH_BAND_UPPER_LIMIT       957100000  /* Upper limit of the high
                                                * band: 956 MHz */
#define MIDDLE_BAND_LOWER_LIMIT     386000000  /* Lower limit of the middle
                                                * band: 387 MHz */
#define MIDDLE_BAND_UPPER_LIMIT     471100000  /* Upper limit of the middle
                                                * band: 470 MHz */
#define LOW_BAND_LOWER_LIMIT        299000000  /* Lower limit of the low
                                                * band: 300 MHz */
#define LOW_BAND_UPPER_LIMIT        349100000  /* Upper limit of the low
                                                * band: 348 MHz */
#define VERY_LOW_BAND_LOWER_LIMIT   149000000  /* Lower limit of the very
                                                * low band: 150 MHz */
#define VERY_LOW_BAND_UPPER_LIMIT   175100000  /* Upper limit of the very
                                                * low band: 174 MHz */

#define IS_FREQUENCY_BAND_HIGH(frequency) \
  ((frequency) >= HIGH_BAND_LOWER_LIMIT && (frequency) <= HIGH_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_MIDDLE(frequency) \
  ((frequency) >= MIDDLE_BAND_LOWER_LIMIT && (frequency) <= MIDDLE_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_LOW(frequency) \
  ((frequency) >= LOW_BAND_LOWER_LIMIT && (frequency) <= LOW_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_VERY_LOW(frequency) \
  ((frequency) >= VERY_LOW_BAND_LOWER_LIMIT && (frequency)<=VERY_LOW_BAND_UPPER_LIMIT)

#define IS_FREQUENCY_BAND(frequency) \
  (IS_FREQUENCY_BAND_HIGH(frequency)|| IS_FREQUENCY_BAND_MIDDLE(frequency)|| \
   IS_FREQUENCY_BAND_LOW(frequency) || IS_FREQUENCY_BAND_VERY_LOW(frequency))

/* Radio_IF_Offset.
 *
 * This represents the IF_OFFSET_ANA inorder to have an intermediate frequency
 * of 480 kHz.
 */

#define IF_OFFSET_ANA(fxo)          (lroundf(480140.0 / (fxo) * 12288 - 64.0))

/* Radio_FC_Offset */

#define F_OFFSET_DIVIDER            262144     /* 2^18 factor dividing fxo
                                                * in foffset formula */
#define PPM_FACTOR                  1000000    /* 10^6 factor to use with
                                                * Xtal_offset_ppm */

#define F_OFFSET_LOWER_LIMIT(fxo)   ((-(int32_t)fxo) / F_OFFSET_DIVIDER * 2048)
#define F_OFFSET_UPPER_LIMIT(fxo)   ((int32_t)(fxo / F_OFFSET_DIVIDER * 2047))

#define IS_FREQUENCY_OFFSET(offset, fxo) \
  (offset >= F_OFFSET_LOWER_LIMIT(fxo) && offset <= F_OFFSET_UPPER_LIMIT(fxo))

/* Radio_Channel_Space */

#define CHSPACE_DIVIDER             32768      /* 2^15 factor dividing fxo in
                                                * channel space formula */

#define IS_CHANNEL_SPACE(chspace, fxo) \
  (chspace <= (fxo / 32768 * 255))

/* Radio_Datarate */

#define MINIMUM_DATARATE            100        /* Minimum datarate
                                                * supported by SPIRIT1 100 bps */
#define MAXIMUM_DATARATE            510000     /* Maximum datarate
                                                * supported by SPIRIT1 500 kbps */

#define IS_DATARATE(datarate) \
  (datarate >= MINIMUM_DATARATE && datarate <= MAXIMUM_DATARATE)

/* Radio_Frequency_Deviation */

#define F_DEV_MANTISSA_UPPER_LIMIT  7     /* Maximum value for the
                                                 * mantissa in frequency
                                                 * deviation formula */
#define F_DEV_EXPONENT_UPPER_LIMIT  9     /* Maximum value for the
                                                 * exponent in frequency
                                                 * deviation formula */

#define F_DEV_LOWER_LIMIT(fxo)      (fxo >> 16)
#define F_DEV_UPPER_LIMIT(fxo)      ((fxo * 15) >> 10)

#define IS_F_DEV(fdev,fxo) \
  (fdev >= F_DEV_LOWER_LIMIT(fxo) && fdev <= F_DEV_UPPER_LIMIT(fxo))

/* Radio_Channel_Bandwidth
 *
 *  CH_BW_LOWER_LIMIT - Minimum value of the channel filter bandwidth
 *  CH_BW_UPPER_LIMIT - Maximum value of the channel filter bandwidth
 */

#define CH_BW_LOWER_LIMIT(fxo)      1100 * (fxo / 1000000) / 26
#define CH_BW_UPPER_LIMIT(fxo)      800100*(fxo / 1000000) / 26

#define IS_CH_BW(bw,fxo) \
  ((bw) >= CH_BW_LOWER_LIMIT(fxo) && (bw) <= CH_BW_UPPER_LIMIT(fxo))

/* Radio_Power_Amplifier */

#define IS_PA_MAX_INDEX(index)      ((index) <= 7)
#define IS_PAPOWER_DBM(patable)     ((patable) >= (-31) && (patable) <= (12))
#define IS_PAPOWER(patable)         ((patable) <= 90)
#define IS_PA_STEP_WIDTH(width)     ((width) >= 1 && (width) <= 4)

/* Radio_Automatic_Frequency_Correction */

#define IS_AFC_FAST_GAIN(gain)      ((gain) <= 5)
#define IS_AFC_SLOW_GAIN(gain)      ((gain) <= 15)
#define IS_AFC_PD_LEAKAGE(leakage)  ((leakage) <= 31)

/* Radio_Automatic_Gain_Control */

#define AGC_MEASURE_TIME_UPPER_LIMIT_US(fxo) \
  (393216.0 / fxo)

#define IS_AGC_MEASURE_TIME_US(time, fxo) \
  (time <= AGC_MEASURE_TIME_UPPER_LIMIT_US(fxo))

#define IS_AGC_MEASURE_TIME(time)   (time<=15)

#define AGC_HOLD_TIME_UPPER_LIMIT_US(fxo) \
  (756.0 / fxo)

#define IS_AGC_HOLD_TIME_US(time,fxo) \
  (time <= AGC_HOLD_TIME_UPPER_LIMIT_US(fxo))

#define IS_AGC_HOLD_TIME(time)      (time <= 63)

#define IS_AGC_THRESHOLD(threshold) (threshold <= 15)

/* Radio_Clock_Recovery */

#define IS_CLK_REC_P_GAIN(gain)     ((gain) <= 7)
#define IS_CLK_REC_I_GAIN(gain)     ((gain) <= 15)

/* Other macros used in assertions */

#define IS_XTAL_FLAG(flag) \
  (((flag) == XTAL_FLAG_24_MHz)    || ((flag) == XTAL_FLAG_26_MHz))
#define IS_BAND_SELECTED(band) \
  (((band) == HIGH_BAND)           || ((band) == MIDDLE_BAND)      || \
   ((band) == LOW_BAND)            || ((band) == VERY_LOW_BAND))
#define IS_MODULATION_SELECTED(mod) \
  (((mod)  == FSK)                 || ((mod)  == GFSK_BT05)        || \
   ((mod)  == GFSK_BT1)            || ((mod)  == ASK_OOK)          || \
   ((mod)  == MSK))
#define IS_PA_LOAD_CAP(cwc) \
  (((cwc) == LOAD_0_PF)            || ((cwc) == LOAD_1_2_PF)       || \
   ((cwc) == LOAD_2_4_PF)          || ((cwc) == LOAD_3_6_PF))
#define IS_AFC_MODE(mode) \
  ((mode) == AFC_SLICER_CORRECTION || (mode) == AFC_2ND_IF_CORRECTION)
#define IS_AGC_MODE(mode) \
  ((mode) == AGC_LINEAR_MODE       || (mode) == AGC_BINARY_MODE)
#define IS_CLK_REC_MODE(mode) \
  ((mode) == CLK_REC_PLL           || (mode) == CLK_REC_DLL)
#define IS_PST_FLT_LENGTH(len) \
  ((len) == PSTFLT_LENGTH_8        || (len) == PSTFLT_LENGTH_16)
#define IS_OOK_PEAK_DECAY(decay) \
  (((decay) == FAST_DECAY)         || ((decay) == MEDIUM_FAST_DECAY) ||\
   ((decay) == MEDIUM_SLOW_DECAY)  || ((decay) == SLOW_DECAY))

/******************************************************************************
 * Public Types
 ******************************************************************************/

/* SPIRIT XTAL frequency enumeration */

enum xtal_flag_e
{
  XTAL_FLAG_24_MHz = 0x00,  /* 24 MHz Xtal selected */
  XTAL_FLAG_26_MHz = 0x01   /* 26 MHz Xtal selected */
};

/* SPIRIT Band enumeration */

enum spirit_bandselect_e
{
  HIGH_BAND        = 0x00,  /* High_Band selected: from 779 MHz to 915 MHz */
  MIDDLE_BAND      = 0x01,  /* Middle Band selected: from 387 MHz to 470 MHz */
  LOW_BAND         = 0x02,  /* Low Band selected: from 300 MHz to 348 MHz */
  VERY_LOW_BAND    = 0x03   /* Vary low Band selected: from 150 MHz to 174 MHz */
};

/* SPIRIT Modulation enumeration */

enum modulation_select_e
{
  FSK              = 0x00,  /* 2-FSK modulation selected */
  GFSK_BT05        = 0x50,  /* GFSK modulation selected with BT=0.5 */
  GFSK_BT1         = 0x10,  /* GFSK modulation selected with BT=1 */
  ASK_OOK          = 0x20,  /* ASK or OOK modulation selected. ASK will
                             * use power ramping */
  MSK              = 0x30   /* MSK modulation selected */
};

/* SPIRIT PA additional load capacitors bank enumeration */

enum spirit_paload_capacitor_e
{
  LOAD_0_PF        = PA_POWER0_CWC_0,    /* No additional PA load capacitor */
  LOAD_1_2_PF      = PA_POWER0_CWC_1_2P, /* 1.2pF additional PA load capacitor */
  LOAD_2_4_PF      = PA_POWER0_CWC_2_4P, /* 2.4pF additional PA load capacitor */
  LOAD_3_6_PF      = PA_POWER0_CWC_3_6P  /* 3.6pF additional PA load capacitor */
};

/* SPIRIT AFC Mode selection */

enum spirit_afcmode_e
{
  AFC_SLICER_CORRECTION = AFC2_AFC_MODE_SLICER, /* AFC loop closed on slicer */
  AFC_2ND_IF_CORRECTION = AFC2_AFC_MODE_MIXER   /* AFC loop closed on 2nd
                                                 * conversion stage */
};

/* SPIRIT AGC Mode selection */

enum spirit_agcmode_e
{
  AGC_LINEAR_MODE  = AGCCTRL0_AGC_MODE_LINEAR,   /* AGC works in linear mode */
  AGC_BINARY_MODE  = AGCCTRL0_AGC_MODE_BINARY    /* AGC works in binary mode */
};

/* SPIRIT Clock Recovery Mode selection */

enum spirit_clkrecmode_e
{
  CLK_REC_PLL      = FDEV0_CLOCK_REG_ALGO_SEL_PLL, /* PLL alogrithm for clock recovery */
  CLK_REC_DLL      = FDEV0_CLOCK_REG_ALGO_SEL_DLL  /* DLL alogrithm for clock recovery */
};

/* SPIRIT Postfilter length */

enum spirit_pstfltlen_e
{
  PSTFLT_LENGTH_8  = 0x00,  /* Postfilter length is 8 symbols */
  PSTFLT_LENGTH_16 = 0x10   /* Postfilter length is 16 symbols */
};

/* SPIRIT OOK Peak Decay */

enum spirit_ookpeakdelay_e
{
  FAST_DECAY        = 0x00, /* Peak decay control for OOK: fast decay */
  MEDIUM_FAST_DECAY = 0x01, /* Peak decay control for OOK: medium_fast decay */
  MEDIUM_SLOW_DECAY = 0x02, /* Peak decay control for OOK: medium_fast decay */
  SLOW_DECAY        = 0x03  /* Peak decay control for OOK: slow decay */
};

/* SPIRIT Radio initialization structure definition */

struct radio_init_s
{
  uint32_t base_frequency;  /* Specifies the base carrier frequency (in
                             * Hz), i.e. the carrier frequency of channel
                             * #0. This parameter can be in one of the
                             * following ranges: High_Band: from 779 MHz to
                             * 915 MHz Middle Band: from 387 MHz to 470 MHz
                             * Low Band: from 300 MHz to 348 MHz */
  uint32_t chspace;         /* Specifies the channel spacing expressed
                             * in Hz. The channel spacing is expressed as:
                             * NxFREQUENCY_STEPS, where frequency STEPS is
                             * F_Xo/2^15. This parameter can be in the
                             * range: [0, F_Xo/2^15*255] Hz */
  int16_t foffset;          /* Specifies the offset frequency (in ppm)
                             * to compensate crystal inaccuracy expressed
                             * as signed value. */
  uint8_t chnum;            /* Specifies the channel number. This value
                             * is multiplied by the channel spacing and
                             * added to synthesizer base frequency to
                             * generate the actual RF carrier frequency */
  uint8_t modselect;        /* Specifies the modulation. This parameter can
                             * be any value from enum modulation_select_e */
  uint32_t datarate;        /* Specifies the datarate expressed in bps.
                             * This parameter can be in the range between
                             * 100 bps and 500 kbps */
  uint32_t freqdev;         /* Specifies the frequency deviation
                             * expressed in Hz. This parameter can be in
                             * the range: [F_Xo*8/2^18, F_Xo*7680/2^18] Hz */
  uint32_t bandwidth;       /* Specifies the channel filter bandwidth
                             * expressed in Hz. This parameter can be in
                             * the range between 1100 and 800100 Hz */
};

/******************************************************************************
 * Public Function Prototypes
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
                            FAR const struct radio_init_s *radioinit);

/******************************************************************************
 * Name: spirit_radio_get_setup
 *
 * Description:
 *   Returns the SPIRIT analog and digital radio structure according to the
 *   registers value.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   radioinit pointer to a struct radio_init_s that
 *         contains the configuration information for the analog radio part of SPIRIT.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_get_setup(FAR struct spirit_library_s *spirit,
                           FAR struct radio_init_s *radioinit);

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
                              enum xtal_flag_e xtalflag);

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

enum xtal_flag_e spirit_radio_get_xtalflag(FAR struct spirit_library_s *spirit);

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
                                uint32_t fc);

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

uint32_t spirit_radio_get_synthword(FAR struct spirit_library_s *spirit);

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
                               uint32_t synthword);

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
                          enum spirit_bandselect_e band);

/******************************************************************************
 * Name: spirit_radio_get_band
 *
 * Description:
 *   Returns the operating band.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
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
  spirit_radio_get_band(FAR struct spirit_library_s *spirit);

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

uint8_t spirit_radio_get_channel(FAR struct spirit_library_s *spirit);

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
                                 int16_t xtaloffset);

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
                              uint32_t chspace);

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

uint32_t spirit_radio_get_chspace(FAR struct spirit_library_s *spirit);

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
                             uint8_t chnum);

/******************************************************************************
 * Name: spirit_radio_set_foffset_hz
 *
 * Description:
 *   Sets the FC OFFSET register starting from frequency offset expressed in Hz.
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
                                int32_t foffset);

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

int32_t spirit_radio_get_foffset(FAR struct spirit_library_s *spirit);

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
                                   uint32_t fbase);

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
                                           enum spirit_functional_state_e newstate);

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

uint32_t spirit_radio_get_basefrequency(FAR struct spirit_library_s *spirit);

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

uint32_t spirit_radio_get_centerfreq(FAR struct spirit_library_s *spirit);

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
                                  FAR uint8_t *pce);

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
                                     FAR uint8_t *pce);

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
                                 FAR uint8_t *pce);

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
 *   Zero (OK) is returned on succes; a negated errnor value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_set_datarate(FAR struct spirit_library_s *spirit,
                              uint32_t datarate);

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

uint32_t spirit_radio_get_datarate(FAR struct spirit_library_s *spirit);

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
 *   Zero (OK) is returned on succes; a negated errnor value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_set_freqdev(FAR struct spirit_library_s *spirit,
                             uint32_t fdev);

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

uint32_t spirit_radio_get_freqdev(FAR struct spirit_library_s *spirit);

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
 *   Zero (OK) is returned on succes; a negated errnor value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_set_chfilterbw(FAR struct spirit_library_s *spirit,
                                uint32_t bandwidth);

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

uint32_t spirit_radio_get_chfilterbw(FAR struct spirit_library_s *spirit);

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
 *   Zero (OK) is returned on succes; a negated errnor value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_set_modulation(FAR struct spirit_library_s *spirit,
                                enum modulation_select_e modulation);

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
  spirit_radio_get_modulation(FAR struct spirit_library_s *spirit);

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
 *   Zero (OK) is returned on succes; a negated errnor value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_enable_cwtxmode(FAR struct spirit_library_s *spirit,
                                 enum spirit_functional_state_e newstate);

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
 *   Zero (OK) is returned on succes; a negated errnor value is returned on any
 *   failure.
 *
 ******************************************************************************/

int spirit_radio_set_ookpeackdecay(FAR struct spirit_library_s *spirit,
                                    enum spirit_ookpeakdelay_e ookdelay);

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
  spirit_radio_get_ookpeackdecay(FAR struct spirit_library_s *spirit);

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
 *   fbase    Frequency base expressed in Hz.
 *   powerdbm Desired power in dBm.
 *
 * Returned Value:
 *   Register value as byte.
 *
 ******************************************************************************/

uint8_t spirit_radio_convert_power2reg(FAR struct spirit_library_s *spirit,
                                       uint32_t fbase, float powerdbm);

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
                                     uint32_t fbase, uint8_t regval);

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
                                    FAR float *table);

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
 *   table   - Pointer to an array of 8 elements containing the PA value in dbm.
 *             The first element will be the PA_LEVEL_0 and the last element
 *             will be PA_LEVEL_7. Any value higher than PA_UPPER_LIMIT implies
 (             no output power (output stage is in high impedance).
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_get_patable_dbm(FAR struct spirit_library_s *spirit,
                                 FAR uint8_t *nlevels, FAR float *table);

/******************************************************************************
 * Name: spirit_radio_set_palevel_dbm
 *
 * Description:
 *   Sets a specific PA_LEVEL register, with a value given in dBm.
 *
 *   NOTE: This function makes use of the spirit_radio_convert_power2reg function
 *   to interpolate the power value.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   ndx      - PA_LEVEL to set. This parameter shall be in the range [0:7].
 *   powerdbm - PA value to write expressed in dBm . Be sure that this values
 *              is in the correct range [-PA_LOWER_LIMIT: PA_UPPER_LIMIT] dBm.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_palevel_dbm(FAR struct spirit_library_s *spirit,
                                 uint8_t ndx, float powerdbm);

/******************************************************************************
 * Name: spirit_radio_get_palevel_dbm
 *
 * Description:
 *   Returns a specific PA_LEVEL register, returning a value in dBm.
 *
 *   NOTE: This function makes use of the @ref spirit_radio_convert_reg2power fcn to
 *   interpolate the power value.
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
                                   uint8_t ndx);

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
                                FAR uint8_t *table);

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
                             FAR uint8_t *nlevels, FAR uint8_t *table);

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
                             uint8_t ndx, uint8_t power);

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
                                 uint8_t ndx);

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
                                enum spirit_paload_capacitor_e load);

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
  spirit_radio_get_outputload(FAR struct spirit_library_s *spirit);

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
                                      uint8_t ndx);

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

uint8_t spirit_radio_get_palevel_maxindex(FAR struct spirit_library_s *spirit);

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
                                  uint8_t width);

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

uint8_t spirit_radio_get_pastep_width(FAR struct spirit_library_s *spirit);

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
                               enum spirit_functional_state_e newstate);

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
  spirit_radio_isenabled_paramp(FAR struct spirit_library_s *spirit);

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
                            enum spirit_functional_state_e newstate);

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
                                 enum spirit_functional_state_e newstate);

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
                             enum spirit_afcmode_e mode);

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
  spirit_radio_get_afcmode(FAR struct spirit_library_s *spirit);

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
                                  uint8_t leakage);

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

uint8_t spirit_radio_get_afcpdleakage(FAR struct spirit_library_s *spirit);

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
                                   uint8_t length);

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

uint8_t spirit_radio_get_afcfastperiod(FAR struct spirit_library_s *spirit);

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
                                 uint8_t gain);

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

uint8_t spirit_radio_get_afcfastgain(FAR struct spirit_library_s *spirit);

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
                                uint8_t gain);

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

uint8_t spirit_radio_get_afclowgain(FAR struct spirit_library_s *spirit);

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

int8_t spirit_radio_get_afccorrection(FAR struct spirit_library_s *spirit);

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

int32_t spirit_radio_get_afccorrection_hz(FAR struct spirit_library_s *spirit);

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
                            enum spirit_functional_state_e newstate);

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
                             enum spirit_agcmode_e mode);

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
  spirit_radio_get_agcmode(FAR struct spirit_library_s *spirit);

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
                                         enum spirit_functional_state_e newstate);

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
                                       enum spirit_functional_state_e newstate);

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
                                           enum spirit_functional_state_e newstate);

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
                                   uint16_t time);

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

uint16_t spirit_radio_get_agcmeasure_us(FAR struct spirit_library_s *spirit);

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
                                uint8_t time);

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

uint8_t spirit_radio_get_agcmeasure(FAR struct spirit_library_s *spirit);

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
                                    uint8_t time);

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

uint8_t spirit_radio_get_agcholdtime_us(FAR struct spirit_library_s *spirit);

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
                                    uint8_t time);

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

uint8_t spirit_radio_get_agcholdtime(FAR struct spirit_library_s *spirit);

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
                                  uint8_t highthres);

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

uint8_t spirit_radio_get_agchighthres(FAR struct spirit_library_s *spirit);

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
                                 uint8_t lowthres);

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

uint8_t spirit_radio_get_agclowthres(FAR struct spirit_library_s *spirit);

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
 *              CLK_REC_PLL  PLL alogrithm for clock recovery
 *              CLK_REC_DLL  DLL alogrithm for clock recovery
 *
 * Returned Value:
 *   Zero (OK) on success.  A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_radio_set_clkrecmode(FAR struct spirit_library_s *spirit,
                                enum spirit_clkrecmode_e mode);

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
 *     CLK_REC_PLL  PLL alogrithm for clock recovery
 *     CLK_REC_DLL  DLL alogrithm for clock recovery
 *
 ******************************************************************************/

enum spirit_clkrecmode_e
  spirit_radio_get_clkrecmode(FAR struct spirit_library_s *spirit);

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
                                uint8_t pgain);

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

uint8_t spirit_radio_get_clkrecgain(FAR struct spirit_library_s *spirit);

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
                                 uint8_t igain);

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

uint8_t spirit_radio_get_clkrecigain(FAR struct spirit_library_s *spirit);

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
                                     enum spirit_pstfltlen_e length);

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
  spirit_radio_get_clkrecpstfltlen(FAR struct spirit_library_s *spirit);

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
                                   enum spirit_functional_state_e newstate);

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
                              enum spirit_functional_state_e newstate);

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
                            enum spirit_functional_state_e newstate);

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
  spirit_radio_get_refdiv(FAR struct spirit_library_s *spirit);

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
                                   enum spirit_functional_state_e newstate);

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
  spirit_radio_isenabled_digdivider(FAR struct spirit_library_s *spirit);

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_RADIO_H*/
