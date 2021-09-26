/******************************************************************************
 * drivers/wireless/spirit/include/spirit_qi.h
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_QI_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_QI_H

/* This module can be used to configure and read some quality indicators
 * used by Spirit.  API to set thresholds and to read values in raw mode or in
 * dBm are provided.
 *
 * Example:
 *
 *   float   rssi;
 *   uint8_t pqi;
 *   uint8_t sqi;
 *
 *   spirit_qi_enable_pqicheck(spirit, S_ENABLE);
 *   spirit_qi_enable_sqicheck(spirit, S_ENABLE);
 *
 *   ...
 *
 *   rssi = spirit_qi_get_rssidbm(spirit);
 *   pqi  = spirit_qi_get_pqi(spirit);
 *   sqi  = spirit_qi_get_sqi(spirit);
 *
 *   ...
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macro to obtain the RSSI value in dBm (type float) */

#define spirit_qi_get_rssidbm(spirit) \
  (-120.0 + ((float)(spirit_qi_get_rssi(spirit) - 20)) / 2)

/* Macros used in debug assertions */

#define IS_PQI_THR(value)  \
  (value == PQI_TH_0   || value == PQI_TH_1   ||\
   value == PQI_TH_2   || value == PQI_TH_3   ||\
   value == PQI_TH_4   || value == PQI_TH_5   ||\
   value == PQI_TH_6   || value == PQI_TH_7   ||\
   value == PQI_TH_8   || value == PQI_TH_9   ||\
   value == PQI_TH_10  || value == PQI_TH_11  ||\
   value == PQI_TH_12  || value == PQI_TH_13  ||\
   value == PQI_TH_14  || value == PQI_TH_15)
#define IS_SQI_THR(value)  \
  (value  ==  SQI_TH_0 || value == SQI_TH_1 || \
   value == SQI_TH_2 || value == SQI_TH_3)
#define IS_RSSI_FILTER_GAIN(value)  \
  (value == RSSI_FG_0  || value == RSSI_FG_1  ||\
   value == RSSI_FG_2  || value == RSSI_FG_3  ||\
   value == RSSI_FG_4  || value == RSSI_FG_5  ||\
   value == RSSI_FG_6  || value == RSSI_FG_7  ||\
   value == RSSI_FG_8  || value == RSSI_FG_9  ||\
   value == RSSI_FG_10 || value == RSSI_FG_11 ||\
   value == RSSI_FG_12 || value == RSSI_FG_13 ||\
   value == RSSI_FG_14 || value == RSSI_FG_15)
#define IS_CS_MODE(mode) \
   (mode == CS_MODE_STATIC_3DB   || mode == CS_MODE_DYNAMIC_6DB  || \
    mode == CS_MODE_DYNAMIC_12DB || mode == CS_MODE_DYNAMIC_18DB)

/* Range for the RSSI Threshold in dBm  */

#define IS_RSSI_THR_DBM(value)  (value >= -130 && value <= -2)

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* PQI threshold value enumeration. */

enum spirit_pqi_threshold_e
{
  PQI_TH_0   = 0x00,
  PQI_TH_1   = 0x04,
  PQI_TH_2   = 0x08,
  PQI_TH_3   = 0x0c,
  PQI_TH_4   = 0x10,
  PQI_TH_5   = 0x14,
  PQI_TH_6   = 0x18,
  PQI_TH_7   = 0x1c,
  PQI_TH_8   = 0x20,
  PQI_TH_9   = 0x24,
  PQI_TH_10  = 0x28,
  PQI_TH_11  = 0x2c,
  PQI_TH_12  = 0x30,
  PQI_TH_13  = 0x34,
  PQI_TH_14  = 0x38,
  PQI_TH_15  = 0x3c
};

/* SQI threshold value enumeration. */

enum spirit_sqi_threshold_e
{
  SQI_TH_0   = 0x00,
  SQI_TH_1   = 0x40,
  SQI_TH_2   = 0x80,
  SQI_TH_3   = 0xc0
};

/* RSSI filter gain value enumeration. */

enum spirit_rssi_filtergain_e
{
  RSSI_FG_0  = 0x00,
  RSSI_FG_1  = 0x10,
  RSSI_FG_2  = 0x20,
  RSSI_FG_3  = 0x30,
  RSSI_FG_4  = 0x40,
  RSSI_FG_5  = 0x50,
  RSSI_FG_6  = 0x60,
  RSSI_FG_7  = 0x70,
  RSSI_FG_8  = 0x80,
  RSSI_FG_9  = 0x90,
  RSSI_FG_10 = 0xa0,
  RSSI_FG_11 = 0xb0,
  RSSI_FG_12 = 0xc0,
  RSSI_FG_13 = 0xd0,
  RSSI_FG_14 = 0xe0,        /* Recommended value */
  RSSI_FG_15 = 0xf0
};

/* CS mode enumeration. */

enum spirit_csmode_e
{
  CS_MODE_STATIC_3DB   = 0x00,
  CS_MODE_DYNAMIC_6DB  = 0x04,
  CS_MODE_DYNAMIC_12DB = 0x08,
  CS_MODE_DYNAMIC_18DB = 0x0c
};

/******************************************************************************
 * Public Function Prototypes
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
                              enum spirit_functional_state_e newstate);

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
                              enum spirit_functional_state_e newstate);

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
                                enum spirit_pqi_threshold_e pqithr);

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
  spirit_qi_get_pqithreshold(FAR struct spirit_library_s *spirit);

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
                               enum spirit_sqi_threshold_e sqithr);

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
  spirit_qi_get_sqithreshold(FAR struct spirit_library_s *spirit);

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

uint8_t spirit_qi_get_pqi(FAR struct spirit_library_s *spirit);

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

uint8_t spirit_qi_get_sqi(FAR struct spirit_library_s *spirit);

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

uint8_t spirit_qi_get_lqi(FAR struct spirit_library_s *spirit);

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

enum
  spirit_flag_status_e spirit_qi_get_cs(FAR struct spirit_library_s *spirit);

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

uint8_t spirit_qi_get_rssi(FAR struct spirit_library_s *spirit);

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
                                uint8_t rssithr);

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

uint8_t spirit_qi_get_rssithreshold(FAR struct spirit_library_s *spirit);

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
                                    int dbmvalue);

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
                                    int dbmvalue);

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
                                 enum spirit_rssi_filtergain_e rssfg);

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
  spirit_qi_get_rssifiltergain(FAR struct spirit_library_s *spirit);

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
                         enum spirit_csmode_e csmode);

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

enum spirit_csmode_e spirit_qi_get_csmode(FAR struct spirit_library_s *spirit);

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
                               enum spirit_functional_state_e newstate);

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
                                enum spirit_functional_state_e newstate);

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
                                enum spirit_functional_state_e newstate);

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_GENERAL_H*/
