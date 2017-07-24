/******************************************************************************
 * include/nuttx/wireless/spirit/spirit_timer.h
 * Configuration and management of SPIRIT timers.
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

#ifndef __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_TIMER_H
#define __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_TIMER_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_regs.h"
#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in debug assertions */

#define IS_RX_TIMEOUT_STOP_CONDITION(cond)  \
  (cond == NO_TIMEOUT_STOP              || cond == TIMEOUT_ALWAYS_STOPPED || \
   cond == RSSI_ABOVE_THRESHOLD         || cond == SQI_ABOVE_THRESHOLD || \
   cond == PQI_ABOVE_THRESHOLD          || cond == RSSI_AND_SQI_ABOVE_THRESHOLD || \
   cond == RSSI_AND_PQI_ABOVE_THRESHOLD || cond == SQI_AND_PQI_ABOVE_THRESHOLD || \
   cond == ALL_ABOVE_THRESHOLD          || cond == RSSI_OR_SQI_ABOVE_THRESHOLD || \
   cond == RSSI_OR_PQI_ABOVE_THRESHOLD  || cond == SQI_OR_PQI_ABOVE_THRESHOLD || \
   cond == ANY_ABOVE_THRESHOLD)

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* All the possible RX timeout stop conditions enumeration. */

enum spirit_rxtimeout_stopcondition_e
{
  NO_TIMEOUT_STOP              = 0x00,  /* Timeout never stopped */
  TIMEOUT_ALWAYS_STOPPED       = 0x08,  /* Timeout always stopped (default) */
  RSSI_ABOVE_THRESHOLD         = 0x04,  /* Timeout stopped on RSSI above
                                         * threshold */
  SQI_ABOVE_THRESHOLD          = 0x02,  /* Timeout stopped on SQI above
                                         * threshold */
  PQI_ABOVE_THRESHOLD          = 0x01,  /* Timeout stopped on PQI above
                                         * threshold */
  RSSI_AND_SQI_ABOVE_THRESHOLD = 0x06,  /* Timeout stopped on both
                                         * RSSI and SQI above threshold */
  RSSI_AND_PQI_ABOVE_THRESHOLD = 0x05,  /* Timeout stopped on both
                                         * RSSI and PQI above threshold */
  SQI_AND_PQI_ABOVE_THRESHOLD  = 0x03,  /* Timeout stopped on both
                                         * SQI and PQI above threshold */
  ALL_ABOVE_THRESHOLD          = 0x07,  /* Timeout stopped only if RSSI, SQI
                                         * and PQI are above threshold */
  RSSI_OR_SQI_ABOVE_THRESHOLD  = 0x0e,  /* Timeout stopped if one
                                         * between RSSI or SQI are
                                         * above threshold */
  RSSI_OR_PQI_ABOVE_THRESHOLD  = 0x0d,  /* Timeout stopped if one
                                         * between RSSI or PQI are
                                         * above threshold */
  SQI_OR_PQI_ABOVE_THRESHOLD   = 0x0b,  /* Timeout stopped if one
                                         * between SQI or PQI are above
                                         * threshold */
  ANY_ABOVE_THRESHOLD          = 0x0f   /* Timeout stopped if one among
                                         * RSSI, SQI or SQI are above threshold */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_timer_set_rxtimeout
 *
 * Description:
 *   Sets the RX timeout timer counter.  If 'counter' is equal to 0 the
 *   timeout is disabled.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   counter - value for the timer counter.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_rxtimeout(FAR struct spirit_library_s *spirit,
                               uint8_t counter);

/******************************************************************************
 * Name: spirit_timer_set_rxtimeout_stopcondition
 *
 * Description:
 *   Sets the RX timeout stop conditions.
 *
 * Input Parameters:
 *   spirit        - Reference to a Spirit library state structure instance
 *   stopcondition - New stop condition.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_rxtimeout_stopcondition(FAR struct spirit_library_s *spirit,
                                             enum spirit_rxtimeout_stopcondition_e
                                             stopcondition);

#endif /* __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_TIMER_H */
