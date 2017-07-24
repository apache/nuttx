/******************************************************************************
 * include/nuttx/wireless/spirit/spirit_qi.h
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

#ifndef __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_QI_H
#define __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_QI_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in debug assertions */

#define IS_SQI_THR(value)  \
  (value == SQI_TH_0 || value == SQI_TH_1 || \
   value == SQI_TH_2 || value == SQI_TH_3)
#define IS_RSSI_FILTER_GAIN(value)  \
  (value==RSSI_FG_0  || value==RSSI_FG_1  ||\
   value==RSSI_FG_2  || value==RSSI_FG_3  ||\
   value==RSSI_FG_4  || value==RSSI_FG_5  ||\
   value==RSSI_FG_6  || value==RSSI_FG_7  ||\
   value==RSSI_FG_8  || value==RSSI_FG_9  ||\
   value==RSSI_FG_10 || value==RSSI_FG_11 ||\
   value==RSSI_FG_12 || value==RSSI_FG_13 ||\
   value==RSSI_FG_14 || value==RSSI_FG_15)

/* Range for the RSSI Threshold in dBm  */

#define IS_RSSI_THR_DBM(value)  (value >= -130 && value <= -2)

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* SQI threshold value enumeration. */

enum spirit_sqi_threshold_e
{
  SQI_TH_0 = 0x00,
  SQI_TH_1 = 0x40,
  SQI_TH_2 = 0x80,
  SQI_TH_3 = 0xC0
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

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_qi_sqicheck
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

int spirit_qi_sqicheck(FAR struct spirit_library_s *spirit,
                       enum spirit_functional_state_e newstate);

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
 * Name: spirit_qi_set_rssithreshold
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

int spirit_qi_set_rssithreshold(FAR struct spirit_library_s *spirit,
                                int dbmvalue);

#endif /* __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_GENERAL_H*/
