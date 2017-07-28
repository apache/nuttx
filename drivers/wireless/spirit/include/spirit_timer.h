/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_timer.h
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_TIMER_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_TIMER_H

/* This module provides API to configure the Spirit timing mechanisms.
 * This allows the user to set the timer registers using raw values or
 * compute them since the desired timer value is expressed in milliseconds.
 * In addition, the management of the Spirit LDCR mode can be done using
 * these interfaces.
 *
 * Example:
 *   ...
 *
 *   spirit_timer_set_rxtimeout(spirit, 50.0);
 *   spirit_timer_set_wakeuptimer(spirit, 150.0);
 *
 *   # IRQ configuration for RX_TIMEOUT and WAKEUP_TIMEOUT
 *   ...
 *
 *   spirit_timer_enable_ldcrmode(spirit, S_ENABLE);
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

/* This represents the Time Step for RX_Timeout timer in case of 24 MHz
 * Crystal, expressed in microseconds.  It is equal to 1210/(24*10^6). With
 * this time step it is possible to fix the RX_Timeout to a minimum value of
 * 50.417us to a maximum value of about 3.278 seconds.  Remember that it is
 * possible to have infinite RX_Timeout writing 0 in the RX_Timeout_Counter
 * and/or RX_Timeout_Prescaler registers.
 */

#define RX_TCLK_24MHz    50.417f

/* This represents the Time Step for RX_Timeout timer in case of 26 MHz
 * Crystal, expressed in microseconds.  It is equal to 1210/(26*10^6). With
 * this time step it is possible to fix the RX_Timeout to a minimum value of
 * 46.538us to a maximum value of about 3.026 seconds.  Remember that it is
 * possible to have infinite RX_Timeout writing 0 in the RX_Timeout_Counter
 * register.
 */

#define RX_TCLK_26MHz    46.538f

/* This represents the Time Step for RX_Wakeup timer expressed in
 * microseconds. This timer is based on RCO (about 34.7 kHZ).  With this
 * time step it is possible to fix the Wakeup_Timeout to a minimum value of
 * 28.818us to a maximum value of about 1.888 seconds.
 */

#define WAKEUP_TCLK      28.818f

/* Macros used in debug assertions */

#define IS_RX_TIMEOUT_STOP_CONDITION(cond)  \
  (cond == NO_TIMEOUT_STOP              || cond == TIMEOUT_ALWAYS_STOPPED || \
   cond == RSSI_ABOVE_THRESHOLD         || cond == SQI_ABOVE_THRESHOLD || \
   cond == PQI_ABOVE_THRESHOLD          || cond == RSSI_AND_SQI_ABOVE_THRESHOLD || \
   cond == RSSI_AND_PQI_ABOVE_THRESHOLD || cond == SQI_AND_PQI_ABOVE_THRESHOLD || \
   cond == ALL_ABOVE_THRESHOLD          || cond == RSSI_OR_SQI_ABOVE_THRESHOLD || \
   cond == RSSI_OR_PQI_ABOVE_THRESHOLD  || cond == SQI_OR_PQI_ABOVE_THRESHOLD || \
   cond == ANY_ABOVE_THRESHOLD)
#define IS_RX_TIMEOUT_24MHz(timeout)    ((timeout * 1000) >= RX_TCLK_24MHz)
#define IS_RX_TIMEOUT_26MHz(timeout)    ((timeout * 1000) >= RX_TCLK_26MHz)
#define IS_WKUP_TIMEOUT(timeout)        ((timeout * 1000) >= WAKEUP_TCLK)

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
 * Name: spirit_timer_enable_ldcrmode
 *
 * Description:
 *   Enables or Disables the LDCR mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for LDCR mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_enable_ldcrmode(FAR struct spirit_library_s *spirit,
                                 enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_timer_enable_autoreload
 *
 * Description:
 *   Enables or Disables the LDCR timer reloading with the value stored in the
 *   LDCR_RELOAD registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for LDCR reloading.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_enable_autoreload(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_timer_get_autoreload
 *
 * Description:
 *   Returns the LDCR timer reload bit.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Value of the reload bit.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_timer_get_autoreload(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_timer_setup_rxtimeout
 *
 * Description:
 *   Sets the RX timeout timer initialization registers with the values of
 *   COUNTER and PRESCALER according to the formula: Trx=PRESCALER*COUNTER*Tck.
 *   Remember that it is possible to have infinite RX_Timeout writing 0 in the
 *   RX_Timeout_Counter and/or RX_Timeout_Prescaler registers.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   counter   - Value for the timer counter.
 *   prescaler - Value for the timer prescaler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_setup_rxtimeout(FAR struct spirit_library_s *spirit,
                                 uint8_t counter, uint8_t prescaler);

/******************************************************************************
 * Name: spirit_timer_set_rxtimeout
 *
 * Description:
 *   Sets the RX timeout timer counter and prescaler from the desired value in
 *   ms. it is possible to fix the RX_Timeout to a minimum value of 50.417us
 *   to a maximum value of about 3.28 s.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   desired - Desired timer value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_rxtimeout(FAR struct spirit_library_s *spirit,
                               float desired);

/******************************************************************************
 * Name: spirit_timer_set_rxtimeout_counter
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

int spirit_timer_set_rxtimeout_counter(FAR struct spirit_library_s *spirit,
                                       uint8_t counter);

/******************************************************************************
 * Name: spirit_timer_set_rxtimeout_prescaler
 *
 * Description:
 *   Sets the RX timeout timer prescaler. If it is equal to 0 the timeout is
 *   infinite.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   prescaler - Value for the timer prescaler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_rxtimeout_prescaler(FAR struct spirit_library_s *spirit,
                                         uint8_t prescaler);

/******************************************************************************
 * Name: spirit_timer_get_rxtimeout_setup
 *
 * Description:
 *   Returns the RX timeout timer.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   mstimeout - Pointer to the variable in which the timeout expressed in
 *               milliseconds has to be stored.  If the returned value is 0,
 *               it means that the RX_Timeout is infinite.
 *   counter   - Pointer to the variable in which the timer counter has to
 *               be stored.
 *   prescaler - Pointer to the variable in which the timer prescaler has to
 *               be stored.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_get_rxtimeout_setup(FAR struct spirit_library_s *spirit,
                                     FAR float *mstimeout,
                                     FAR uint8_t *counter,
                                     FAR uint8_t *prescaler);

/******************************************************************************
 * Name: spirit_timer_setup_wakeuptimer
 *
 * Description:
 *   Sets the LDCR wake up timer initialization registers with the values of
 *   COUNTER and PRESCALER according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck,
 *   where Tck = 28.818 us. The minimum vale of the wakeup timeout is 28.818us
 *   (PRESCALER and COUNTER equals to 0) and the maximum value is about 1.89 s
 *   (PRESCALER anc COUNTER equals to 255).
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   counter   - Value for the timer counter.
 *   prescaler - Value for the timer prescaler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_setup_wakeuptimer(FAR struct spirit_library_s *spirit,
                                   uint8_t counter, uint8_t prescaler);

/******************************************************************************
 * Name: spirit_timer_set_wakeuptimer
 *
 * Description:
 *   Sets the LDCR wake up timer counter and prescaler from the desired value
 *   in ms, according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck,
 *   where Tck = 28.818 us.  The minimum vale of the wakeup timeout is
 *   28.818us (PRESCALER and COUNTER equals to 0) and the maximum value is
 *   about 1.89 s (PRESCALER anc COUNTER equals to 255).
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   desired - Desired timer value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_wakeuptimer(FAR struct spirit_library_s *spirit,
                                 float desired);

/******************************************************************************
 * Name: spirit_timer_set_wakeuptimer_counter
 *
 * Description:
 *   Sets the LDCR wake up timer counter. Remember that this value is
 *   increased by one in the Twu calculation.
 *   Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   counter - Value for the timer counter.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_wakeuptimer_counter(FAR struct spirit_library_s *spirit,
                                          uint8_t counter);

/******************************************************************************
 * Name: spirit_timer_set_wakeuptimer_prescaler
 *
 * Description:
 *   Sets the LDCR wake up timer prescaler. Remember that this value is
 *   increased by one in the Twu calculation.
 *   Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   prescaler - Value for the timer prescaler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_wakeuptimer_prescaler(FAR struct spirit_library_s *spirit,
                                           uint8_t prescaler);

/******************************************************************************
 * Name: spirit_timer_get_wakeuptimer_setup
 *
 * Description:
 *   Returns the LDCR wake up timer, according to the formula:
 *   Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   wakeupmsec - Pointer to the variable in which the wake-up time expressed
 *                in milliseconds has to be stored.
 *   counter    - Pointer to the variable in which the timer counter has to
 *                be stored.
 *   prescaler  - Pointer to the variable in which the timer prescaler has to
 *                be stored.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_get_wakeuptimer_setup(FAR struct spirit_library_s *spirit,
                                       FAR float *wakeupmsec,
                                       FAR uint8_t *counter,
                                       FAR uint8_t *prescaler);

/******************************************************************************
 * Name: spirit_timer_setup_wakeuptimer_reload
 *
 * Description:
 *   Sets the LDCR wake up timer reloading registers with the values of
 *   COUNTER and PRESCALER according to the formula:
 *
 *     Twu=(PRESCALER +1)*(COUNTER+1)*Tck
 *
 *   where Tck = 28.818 us. The minimum vale of the wakeup timeout is
 *   28.818us (PRESCALER and COUNTER equals to 0) and the maximum value is
 *   about 1.89 s (PRESCALER anc COUNTER equals to 255).
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   counter   - Reload value for the timer counter.
 *   prescaler - Reload value for the timer prescaler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_setup_wakeuptimer_reload(FAR struct spirit_library_s *spirit,
                                          uint8_t counter, uint8_t prescaler);

/******************************************************************************
 * Name: spirit_timer_wakeuptimer_reload
 *
 * Description:
 *   Sets the LDCR wake up reload timer counter and prescaler from the desired
 *   value in ms, according to the formula:
 *
 *     Twu=(PRESCALER +1)*(COUNTER+1)*Tck
 *
 *   where Tck = 28.818 us.  The minimum vale of the wakeup timeout is 28.818us
 *   (PRESCALER and COUNTER equals to 0) and the maximum value is about 1.89 s
 *   (PRESCALER anc COUNTER equals to 255).
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   desired - Desired timer value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_wakeuptimer_reload(FAR struct spirit_library_s *spirit,
                                    float desired);

/******************************************************************************
 * Name: spirit_timer_set_wakeuptimer_reloadcounter
 *
 * Description:
 *   Sets the LDCR wake up timer reload counter. Remember that this value is
 *   increasedd by one in the Twu calculation.
 *
 *     Twu=(PRESCALER +1)*(COUNTER+1)*Tck
 *
 *   where Tck = 28.818 us.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   counter - Value for the timer counter.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_wakeuptimer_reloadcounter(FAR struct spirit_library_s *spirit,
                                               uint8_t counter);

/******************************************************************************
 * Name: spirit_timer_set_wakeuptimer_reloadprescaler
 *
 * Description:
 *   Sets the LDCR wake up timer reload prescaler. Remember that this value
 *   is increasedd by one in the Twu calculation.
 *
 *      Twu=(PRESCALER +1)*(COUNTER+1)*Tck
 *
 *   where Tck = 28.818 us.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   prescaler - Value for the timer prescaler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_set_wakeuptimer_reloadprescaler(FAR struct spirit_library_s *spirit,
                                                 uint8_t prescaler);

/******************************************************************************
 * Name: spirit_timer_get_wakeuptimer_reload_setup
 *
 * Description:
 *   Returns the LDCR wake up reload timer, according to the formula:
 *
 *     Twu=(PRESCALER +1)*(COUNTER+1)*Tck
 *
 *   where Tck = 28.818 us.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   reload    - Pointer to the variable in which the wake-up reload time
 *               expressed in milliseconds has to be stored.
 *   counter   - Pointer to the variable in which the timer counter has to be
 *               stored.
 *   prescaler - Pointer to the variable in which the timer prescaler has to
 *               be stored.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_get_wakeuptimer_reload_setup(FAR struct spirit_library_s *spirit,
                                              FAR float *reload,
                                              FAR uint8_t *counter,
                                              FAR uint8_t *prescaler);

/******************************************************************************
 * Name: spirit_timer_get_rcofrequency
 *
 * Description:
 *   Computes and returns the RCO frequency. This frequency depends on the
 *   xtal frequency and the XTAL bit in register 0x01.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   RCO frequency in Hz.
 *
 ******************************************************************************/

uint16_t spirit_timer_get_rcofrequency(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_timer_calc_wakeup_values
 *
 * Description:
 *   Computes the values of the wakeup timer counter and prescaler from the
 *   user time expressed in millisecond.  The prescaler and the counter values
 *   are computed maintaining the prescaler value as small as possible in
 *   order to obtain the best resolution, and in the meantime minimizing the error.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   desired   - Desired wakeup timeout in millisecs.  Since the counter and
 *               prescaler are 8 bit registers the maximum reachable value is
 *               maxTime = fTclk x 256 x 256.
 *   counter   - Pointer to the variable in which the value for the wakeup
 *               timer counter has to be stored.
 *   prescaler - Pointer to the variable in which the value for the wakeup
 *               timer prescaler has to be stored.
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

void spirit_timer_calc_wakeup_values(FAR struct spirit_library_s *spirit,
                                     float desired, FAR uint8_t *counter,
                                     FAR uint8_t *prescaler);

/******************************************************************************
 * Name: spirit_timer_calc_rxtimeout_values
 *
 * Description:
 *   Computes the values of the rx_timeout timer counter and prescaler from
 *   the user time expressed in millisecond.  The prescaler and the counter
 *   values are computed maintaining the prescaler value as small as possible
 *   in order to obtain the best resolution, and in the meantime minimizing
 *   the error.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   desired   - Desired rx_timeout in millisecs.  Since the counter and
 *               prescaler are 8 bit registers the maximum reachable value
 *               is maxTime = fTclk x 255 x 255.
 *   counter   - Pointer to the variable in which the value for the rx_timeout
 *               counter has to be stored.
 *   prescaler - Pointer to the variable in which the value for the rx_timeout
 *               prescaler has to be stored.
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

void spirit_timer_calc_rxtimeout_values(FAR struct spirit_library_s *spirit,
                                        float desired, FAR uint8_t *counter,
                                        FAR uint8_t *prescaler);

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

/******************************************************************************
 * Name: spirit_timer_cmd_reload
 *
 * Description:
 *   Sends the LDC_RELOAD command to SPIRIT. Reload the LDC timer with the
 *   value stored in the LDC_PRESCALER / COUNTER registers.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_timer_cmd_reload(FAR struct spirit_library_s *spirit);

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_TIMER_H */
