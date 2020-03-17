/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_timer.c
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

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <assert.h>

#include "spirit_config.h"
#include "spirit_timer.h"
#include "spirit_radio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Returns the absolute value. */

#define S_ABS(a) ((a) > 0 ? (a) : -(a))

/******************************************************************************
 * Public Functions
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
                                 enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Reads the register value */

  ret = spirit_reg_read(spirit, PROTOCOL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the read value to enable or disable the LDC mode */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL2_LDC_MODE_MASK;
        }
      else
        {
          regval &= ~PROTOCOL2_LDC_MODE_MASK;
        }

      /* Write to the register to Enable or Disable the LDCR mode */

      ret = spirit_reg_write(spirit, PROTOCOL2_BASE, &regval, 1);
    }

  return ret;
}

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
                                   enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Reads the register value */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the read value to enable or disable the reload on sync mode */

      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL1_LDC_RELOAD_ON_SYNC_MASK;
        }
      else
        {
          regval &= ~PROTOCOL1_LDC_RELOAD_ON_SYNC_MASK;
        }

      /* Writes the register to Enable or Disable the Auto Reload */

      ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval, 1);
    }

  return ret;
}

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
  spirit_timer_get_autoreload(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register value */

  spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);

  return (enum spirit_functional_state_e)(regval & 0x80);
}

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
                                 uint8_t counter, uint8_t prescaler)
{
  uint8_t regval[2] =
  {
    prescaler, counter
  };

  /* Writes the prescaler and counter value for RX timeout to the corresponding
   * register.
   */

  return spirit_reg_write(spirit, TIMERS5_RX_TIMEOUT_PRESCALER_BASE, regval, 2);
}

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
                               float desired)
{
  uint8_t regval[2];

  /* Computes the counter and prescaler value */

  spirit_timer_calc_rxtimeout_values(spirit, desired, &regval[1], &regval[0]);

  /* Writes the prescaler and counter value for RX timeout in the corresponding
   * register.
   */

  return spirit_reg_write(spirit, TIMERS5_RX_TIMEOUT_PRESCALER_BASE, regval, 2);
}

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
                                       uint8_t counter)
{
  /* Writes the counter value for RX timeout in the corresponding register */

  return spirit_reg_write(spirit, TIMERS4_RX_TIMEOUT_COUNTER_BASE, &counter, 1);
}

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
                                         uint8_t prescaler)
{
  /* Writes the prescaler value for RX timeout in the corresponding register */

  return spirit_reg_write(spirit, TIMERS5_RX_TIMEOUT_PRESCALER_BASE, &prescaler, 1);
}

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
                                     FAR uint8_t *prescaler)
{
  float xtal_frequency;
  uint8_t regval[2];
  int ret;

  /* Reads the RX timeout registers value */

  ret = spirit_reg_read(spirit, TIMERS5_RX_TIMEOUT_PRESCALER_BASE, regval, 2);
  if (ret >= 0)
    {
      /* Returns values */

      *prescaler = regval[0];
      *counter = regval[1];

      xtal_frequency = (float)spirit->xtal_frequency;
      if (xtal_frequency > DOUBLE_XTAL_THR)
        {
          xtal_frequency /= 2.0;
        }

      xtal_frequency /= 1000.0;
      *mstimeout = (float)((regval[0] + 1) * regval[1] * (1210.0 / xtal_frequency));
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_timer_setup_wakeuptimer
 *
 * Description:
 *   Sets the LDCR wake up timer initialization registers with the values of
 *   COUNTER and PRESCALER according to the formula:
 *
 *     Twu=(PRESCALER + 1) * (COUNTER +1 ) * Tck,
 *
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
                                        uint8_t counter, uint8_t prescaler)
{
  uint8_t regval[2] =
  {
    prescaler, counter
  };

  /* Writes the counter and prescaler value of wake-up timer in the
   * corresponding register.
   */

  return spirit_reg_write(spirit, TIMERS3_LDC_PRESCALER_BASE, regval, 2);
}

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
                                 float desired)
{
  uint8_t regval[2];

  /* Computes counter and prescaler */

  spirit_timer_calc_wakeup_values(spirit, desired, &regval[1], &regval[0]);

  /* Writes the counter and prescaler value of wake-up timer in the
   * corresponding register.
   */

  return spirit_reg_write(spirit, TIMERS3_LDC_PRESCALER_BASE, regval, 2);
}

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
                                         uint8_t counter)
{
  /* Writes the counter value for Wake_Up timer in the corresponding register */

  return spirit_reg_write(spirit, TIMERS2_LDC_COUNTER_BASE, &counter, 1);
}

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
                                           uint8_t prescaler)
{
  /* Writes the prescaler value for Wake_Up timer in the corresponding
   * register .
   */

  return spirit_reg_write(spirit, TIMERS3_LDC_PRESCALER_BASE, &prescaler, 1);
}

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
                                       FAR uint8_t *prescaler)
{
  uint8_t regval[2];
  float rco_freq;
  int ret;

  rco_freq = (float)spirit_timer_get_rcofrequency(spirit);

  /* Reads the Wake_Up timer registers value */

  ret = spirit_reg_read(spirit, TIMERS3_LDC_PRESCALER_BASE, regval, 2);
  if (ret >= 0)
    {
      /* Return values */

      *prescaler  = regval[0];
      *counter    = regval[1];
      *wakeupmsec = (float)((((*prescaler) + 1) * ((*counter) + 1) *
                            (1000.0 / rco_freq)));
    }

  return ret;
}

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
                                          uint8_t counter, uint8_t prescaler)
{
  uint8_t regval[2] =
  {
    prescaler, counter
  };

  /* Write the counter and prescaler value of reload wake-up timer in the
   * corresponding register
   */

  return spirit_reg_write(spirit, TIMERS1_LDC_RELOAD_PRESCALER_BASE, regval, 2);
}

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
                                    float desired)
{
  uint8_t regval[2];

  /* Computes counter and prescaler */

  spirit_timer_calc_wakeup_values(spirit, desired, &regval[1], &regval[0]);

  /* Writes the counter and prescaler value of reload wake-up timer in the
   * corresponding register.
   */

  return spirit_reg_write(spirit, TIMERS1_LDC_RELOAD_PRESCALER_BASE, regval, 2);
}

/******************************************************************************
 * Name: spirit_timer_set_wakeuptimer_reloadcounter
 *
 * Description:
 *   Sets the LDCR wake up timer reload counter. Remember that this value is
 *   increased by one in the Twu calculation.
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
                                               uint8_t counter)
{
  /* Write the counter value for reload Wake_Up timer in the corresponding
   * register.
   */

  return spirit_reg_write(spirit, TIMERS0_LDC_RELOAD_COUNTER_BASE, &counter, 1);
}

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
                                                 uint8_t prescaler)
{
  /* Writes the prescaler value for reload Wake_Up timer in the corresponding
   * register.
   */

  return spirit_reg_write(spirit, TIMERS1_LDC_RELOAD_PRESCALER_BASE, &prescaler, 1);
}

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
                                              FAR uint8_t *prescaler)
{
  uint8_t regval[2];
  float rco_freq;
  int ret;

  rco_freq = (float)spirit_timer_get_rcofrequency(spirit);

  /* Reads the reload Wake_Up timer registers value */

  ret = spirit_reg_read(spirit, TIMERS1_LDC_RELOAD_PRESCALER_BASE, regval, 2);
  if (ret >= 0)
    {
      /* Returns values */

      *prescaler = regval[0];
      *counter = regval[1];
      *reload = (float)((((*prescaler) + 1) * ((*counter) + 1) *
                         (1000.0 / rco_freq)));
    }

  return ret;
}

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

uint16_t spirit_timer_get_rcofrequency(FAR struct spirit_library_s *spirit)
{
  uint32_t xtal_frequency = spirit->xtal_frequency;
  uint16_t rco_freq = 34700;
  uint8_t xtal_flag;

  if (xtal_frequency > 30000000)
    {
      xtal_frequency /= 2;
    }

  if (xtal_frequency == 25000000)
    {
      spirit_reg_read(spirit, 0x01, &xtal_flag, 1);
      xtal_flag = (xtal_flag & 0x40);

      if (xtal_flag == 0)
        {
          rco_freq = 36100;
        }
      else
        {
          rco_freq = 33300;
        }
    }

  return rco_freq;
}

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
                                     FAR uint8_t *prescaler)
{
  float rco_freq;
  float err;
  uint32_t n;

  rco_freq = ((float)spirit_timer_get_rcofrequency(spirit)) / 1000;

  /* N cycles in the time base of the timer: - clock of the timer is RCO
   * frequency - divide times 1000 more because we have an input in ms
   * (variable rco_freq is already this frequency divided by 1000).
   */

  n = (uint32_t)(desired * rco_freq);

  /* check if it is possible to reach that target with prescaler and counter of
   * spirit1.
   */

  if (n / 0xff > 0xfd)
    {
      /* if not return the maximum possible value */

      *counter   = 0xff;
      *prescaler = 0xff;
      return;
    }

  /* prescaler is really 2 as min value */

  *prescaler = (n / 0xff) + 2;
  *counter   = n / (*prescaler);

  /* check if the error is minimum */

  err = S_ABS((float)(*counter) * (*prescaler) / rco_freq - desired);

  if ((*counter) <= 254)
    {
      if (S_ABS((float)((*counter) + 1) * (*prescaler) / rco_freq - desired) < err)
        {
          (*counter)++;
        }
    }

  /* decrement prescaler and counter according to the logic of this timer in
   * spirit1.
   */

  (*prescaler)--;
  if (*counter > 1)
    {
      (*counter)--;
    }
  else
    {
      *counter = 1;
    }
}

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
                                        FAR uint8_t *prescaler)
{
  uint32_t xtal_frequency = spirit->xtal_frequency;
  uint32_t n;
  float err;

  /* if xtal_frequency is doubled divide it by 2 */

  if (xtal_frequency > DOUBLE_XTAL_THR)
    {
      xtal_frequency >>= 1;
    }

  /* N cycles in the time base of the timer: - clock of the timer is xtal/1210
   * - divide times 1000 more because we have an input in ms.
   */

  n = (uint32_t)(desired * xtal_frequency / 1210000);

  /* check if it is possible to reach that target with prescaler and counter of
   * spirit1.
   */

  if (n / 0xff > 0xfd)
    {
      /* if not return the maximum possible value */

      *counter   = 0xff;
      *prescaler = 0xff;
      return;
    }

  /* prescaler is really 2 as min value */

  *prescaler = (n / 0xff) + 2;
  *counter   = n / (*prescaler);

  /* check if the error is minimum */

  err = S_ABS((float)(*counter) * (*prescaler) * 1210000 /
                     xtal_frequency - desired);

  if ((*counter) <= 254)
    {
      if (S_ABS((float)((*counter) + 1) * (*prescaler) * 1210000 /
                        xtal_frequency - desired) < err)
        {
          (*counter)++;
        }
    }

  /* decrement prescaler and counter according to the logic of this timer in
   * spirit1.
   */

  (*prescaler)--;
  if ((*counter) > 1)
    {
      (*counter)--;
    }
  else
    {
      *counter = 1;
    }
}

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
                                             stopcondition)
{
  uint8_t regval[2];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_RX_TIMEOUT_STOP_CONDITION(stopcondition));

  /* Reads value on the PKT_FLT_OPTIONS and PROTOCOL2 register */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, regval, 2);
  if (ret >= 0)
    {
      regval[0] &= 0xbf;
      regval[0] |= ((stopcondition & 0x08) << 3);

      regval[1] &= 0x1f;
      regval[1] |= (stopcondition << 5);

      /* Write value to the PKT_FLT_OPTIONS and PROTOCOL2 register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, regval, 2);
    }

  return ret;
}

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

int spirit_timer_cmd_reload(FAR struct spirit_library_s *spirit)
{
  /* Sends the CMD_LDC_RELOAD command */

  return spirit_command(spirit, COMMAND_LDC_RELOAD);
}
