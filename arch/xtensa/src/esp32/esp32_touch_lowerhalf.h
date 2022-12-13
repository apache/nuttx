/****************************************************************************
 * arch/xtensa/src/esp32/esp32_touch_lowerhalf.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_TOUCH_LOWERHALF_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_TOUCH_LOWERHALF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "xtensa.h"

#include "hardware/esp32_rtc_io.h"
#include "hardware/esp32_rtccntl.h"
#include "hardware/esp32_touch.h"
#include "hardware/esp32_sens.h"

#include "esp32_rt_timer.h"
#include "esp32_rtc_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some register bits of touch sensor 8 and 9 are mismatched,
 * we need to swap the bits.
 */

#define TOUCH_LH_BITSWAP(data, n, m) ((((data) >> (n)) &  0x1) == \
                                      (((data) >> (m)) & 0x1) ? (data) : \
                                      ((data) ^ ((0x1 << (n)) | \
                                      (0x1 << (m)))))

#define TOUCH_LH_BITS_SWAP(v) TOUCH_LH_BITSWAP(v,              \
                                               TOUCH_PAD_NUM8, \
                                               TOUCH_PAD_NUM9)

#define setbits(a, bs)   modifyreg32(a, 0, bs)
#define resetbits(a, bs) modifyreg32(a, bs, 0)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: touch_lh_num_wrap
 *
 * Description:
 *   Some registers have swapped bits. This function returns the correct
 *   touchpad id to read.
 *
 * Input Parameters:
 *   tp - The wanted touchpad.
 *
 * Returned Value:
 *   The actual ID to be read.
 *
 ****************************************************************************/

static inline enum touch_pad_e touch_lh_num_wrap(enum touch_pad_e tp)
{
  if (tp == TOUCH_PAD_NUM8)
    {
      return TOUCH_PAD_NUM9;
    }
  else if (tp == TOUCH_PAD_NUM9)
    {
      return TOUCH_PAD_NUM8;
    }

  return tp;
}

/****************************************************************************
 * Name: touch_lh_set_meas_time
 *
 * Description:
 *   Set the measurement time for the touch sensors.
 *
 * Input Parameters:
 *   meas_time - The desired measurement time.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_meas_time(uint16_t meas_time)
{
  /* Touch sensor measure time = meas_cycle / 8Mhz */

  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_MEAS_DELAY, meas_time);

  /* The waiting cycles (in 8MHz) between TOUCH_START and TOUCH_XPD */

  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL1_REG,
                SENS_TOUCH_XPD_WAIT,
                TOUCH_MEASURE_WAIT_MAX);
}

/****************************************************************************
 * Name: touch_lh_get_meas_time
 *
 * Description:
 *   Get the measurement time for the touch sensors.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current measurement time.
 *
 ****************************************************************************/

static inline uint16_t touch_lh_get_meas_time(void)
{
  return REG_GET_FIELD(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_MEAS_DELAY);
}

/****************************************************************************
 * Name: touch_lh_set_sleep_time
 *
 * Description:
 *   Set the sleep time for the touch sensors.
 *
 * Input Parameters:
 *   sleep_time - The desired sleep time.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_sleep_time(uint16_t sleep_time)
{
  /* Touch sensor sleep cycle Time = sleep_cycle / RTC_SLOW_CLK
   * (can be 150k or 32k depending on the options)
   */

  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL2_REG,
                SENS_TOUCH_SLEEP_CYCLES,
                sleep_time);
}

/****************************************************************************
 * Name: touch_lh_get_sleep_time
 *
 * Description:
 *   Get the sleep time for the touch sensors.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current sleep time.
 *
 ****************************************************************************/

static inline uint16_t touch_lh_get_sleep_time(void)
{
  return REG_GET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_SLEEP_CYCLES);
}

/****************************************************************************
 * Name: touch_lh_set_voltage_high
 *
 * Description:
 *   Set the touch sensor high reference voltage.
 *
 * Input Parameters:
 *   refh - The desired enum touch_high_volt_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_voltage_high(enum touch_high_volt_e refh)
{
  REG_SET_FIELD(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DREFH, refh);
}

/****************************************************************************
 * Name: touch_lh_get_voltage_high
 *
 * Description:
 *   Get the touch sensor high reference voltage.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_high_volt_e.
 *
 ****************************************************************************/

static inline enum touch_high_volt_e touch_lh_get_voltage_high(void)
{
  return (enum touch_high_volt_e)
    REG_GET_FIELD(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DREFH);
}

/****************************************************************************
 * Name: touch_lh_set_voltage_low
 *
 * Description:
 *   Set the touch sensor low reference voltage.
 *
 * Input Parameters:
 *   refl - The desired enum touch_low_volt_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_voltage_low(enum touch_low_volt_e refl)
{
  REG_SET_FIELD(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DREFL, refl);
}

/****************************************************************************
 * Name: touch_lh_get_voltage_low
 *
 * Description:
 *   Get the touch sensor low reference voltage.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_low_volt_e.
 *
 ****************************************************************************/

static inline enum touch_low_volt_e touch_lh_get_voltage_low(void)
{
  return (enum touch_low_volt_e)
    REG_GET_FIELD(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DREFL);
}

/****************************************************************************
 * Name: touch_lh_set_voltage_attenuation
 *
 * Description:
 *   Set the touch sensor voltage attenuation.
 *
 * Input Parameters:
 *   atten - The desired enum touch_volt_atten_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_set_voltage_attenuation (enum touch_volt_atten_e atten)
{
  REG_SET_FIELD(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DRANGE, atten);
}

/****************************************************************************
 * Name: touch_lh_get_voltage_attenuation
 *
 * Description:
 *   Get the touch sensor voltage attenuation.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_volt_atten_e.
 *
 ****************************************************************************/

static inline enum touch_volt_atten_e touch_lh_get_voltage_attenuation(void)
{
  return (enum touch_volt_atten_e)
    REG_GET_FIELD(RTC_IO_TOUCH_CFG_REG, RTC_IO_TOUCH_DRANGE);
}

/****************************************************************************
 * Name: touch_lh_set_slope
 *
 * Description:
 *   Set the charge/discharge slope for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   slope - The desired enum touch_cnt_slope_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_slope(enum touch_pad_e tp,
                                      enum touch_cnt_slope_e slope)
{
  /* All touch pads have the same position for the DAC bits.
   * We can use any RTC_IO_TOUCH_PADn_DAC.
   */

  const uint32_t reg_addr[] =
  {
    RTC_IO_TOUCH_PAD0_REG,
    RTC_IO_TOUCH_PAD1_REG,
    RTC_IO_TOUCH_PAD2_REG,
    RTC_IO_TOUCH_PAD3_REG,
    RTC_IO_TOUCH_PAD4_REG,
    RTC_IO_TOUCH_PAD5_REG,
    RTC_IO_TOUCH_PAD6_REG,
    RTC_IO_TOUCH_PAD7_REG,
    RTC_IO_TOUCH_PAD8_REG,
    RTC_IO_TOUCH_PAD9_REG
  };

  REG_SET_FIELD(reg_addr[tp], RTC_IO_TOUCH_PAD0_DAC, slope);
}

/****************************************************************************
 * Name: touch_lh_get_slope
 *
 * Description:
 *   Get the charge/discharge slope for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The current enum touch_cnt_slope_e for that touch pad.
 *
 ****************************************************************************/

static inline enum touch_cnt_slope_e touch_lh_get_slope(enum touch_pad_e tp)
{
  /* All touch pads have the same position for the DAC bits.
   * We can use any RTC_IO_TOUCH_PADn_DAC.
   */

  const uint32_t reg_addr[] =
  {
    RTC_IO_TOUCH_PAD0_REG,
    RTC_IO_TOUCH_PAD1_REG,
    RTC_IO_TOUCH_PAD2_REG,
    RTC_IO_TOUCH_PAD3_REG,
    RTC_IO_TOUCH_PAD4_REG,
    RTC_IO_TOUCH_PAD5_REG,
    RTC_IO_TOUCH_PAD6_REG,
    RTC_IO_TOUCH_PAD7_REG,
    RTC_IO_TOUCH_PAD8_REG,
    RTC_IO_TOUCH_PAD9_REG
  };

  return (enum touch_cnt_slope_e) REG_GET_FIELD(reg_addr[tp],
                                                RTC_IO_TOUCH_PAD0_DAC);
}

/****************************************************************************
 * Name: touch_lh_set_tie_option
 *
 * Description:
 *   Set the initial charging level for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   opt - The desired enum touch_tie_opt_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_tie_option(enum touch_pad_e tp,
                                           enum touch_tie_opt_e opt)
{
  enum touch_pad_e tp_wrap = touch_lh_num_wrap(tp);

  const uint32_t reg_addr[] =
  {
    RTC_IO_TOUCH_PAD0_REG,
    RTC_IO_TOUCH_PAD1_REG,
    RTC_IO_TOUCH_PAD2_REG,
    RTC_IO_TOUCH_PAD3_REG,
    RTC_IO_TOUCH_PAD4_REG,
    RTC_IO_TOUCH_PAD5_REG,
    RTC_IO_TOUCH_PAD6_REG,
    RTC_IO_TOUCH_PAD7_REG,
    RTC_IO_TOUCH_PAD8_REG,
    RTC_IO_TOUCH_PAD9_REG
  };

  /* All touch pads have the same position for the TIE_OPT bits.
   * We can use any RTC_IO_TOUCH_PADn_TIE_OPT.
   */

  REG_SET_FIELD(reg_addr[tp_wrap], RTC_IO_TOUCH_PAD0_TIE_OPT, opt);
}

/****************************************************************************
 * Name: touch_lh_get_tie_option
 *
 * Description:
 *   Get the initial charging level for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The current enum touch_tie_opt_e for that touch pad.
 *
 ****************************************************************************/

static inline enum touch_tie_opt_e
  touch_lh_get_tie_option(enum touch_pad_e tp)
{
  enum touch_pad_e tp_wrap = touch_lh_num_wrap(tp);

  const uint32_t reg_addr[] =
  {
    RTC_IO_TOUCH_PAD0_REG,
    RTC_IO_TOUCH_PAD1_REG,
    RTC_IO_TOUCH_PAD2_REG,
    RTC_IO_TOUCH_PAD3_REG,
    RTC_IO_TOUCH_PAD4_REG,
    RTC_IO_TOUCH_PAD5_REG,
    RTC_IO_TOUCH_PAD6_REG,
    RTC_IO_TOUCH_PAD7_REG,
    RTC_IO_TOUCH_PAD8_REG,
    RTC_IO_TOUCH_PAD9_REG
  };

  /* All touch pads have the same position for the TIE_OPT bits.
   * We can use any RTC_IO_TOUCH_PADn_TIE_OPT.
   */

  return (enum touch_tie_opt_e) REG_GET_FIELD(reg_addr[tp_wrap],
                                              RTC_IO_TOUCH_PAD0_TIE_OPT);
}

/****************************************************************************
 * Name: touch_lh_set_fsm_mode
 *
 * Description:
 *   Set the mode of the internal touch finite state machine.
 *
 * Input Parameters:
 *   mode - The desired enum touch_fsm_mode_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_fsm_mode(enum touch_fsm_mode_e mode)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_FSM_EN, true);
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN, false);
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_FORCE, mode);
}

/****************************************************************************
 * Name: touch_lh_get_fsm_mode
 *
 * Description:
 *   Get the mode of the internal touch finite state machine.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_fsm_mode_e.
 *
 ****************************************************************************/

static inline enum touch_fsm_mode_e touch_lh_get_fsm_mode(void)
{
  return (enum touch_fsm_mode_e)
    REG_GET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_FORCE);
}

/****************************************************************************
 * Name: touch_lh_start_fsm
 *
 * Description:
 *   Start the internal touch finite state machine.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_start_fsm(void)
{
  REG_SET_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_TOUCH_SLP_TIMER_EN, true);
}

/****************************************************************************
 * Name: touch_lh_stop_fsm
 *
 * Description:
 *   Stop the internal touch finite state machine.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_stop_fsm(void)
{
  REG_SET_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_TOUCH_SLP_TIMER_EN, false);
}

/****************************************************************************
 * Name: touch_lh_start_sw_meas
 *
 * Description:
 *   Start measurement controlled by software.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_start_sw_meas(void)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN, false);
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN, true);
}

/****************************************************************************
 * Name: touch_lh_set_threshold
 *
 * Description:
 *   Set the touch interrupt threshold for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   threshold - The desired threshold value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_threshold(enum touch_pad_e tp,
                                          uint16_t threshold)
{
  enum touch_pad_e tp_wrap = touch_lh_num_wrap(tp);

  const uint32_t reg_addr[] =
  {
    SENS_SAR_TOUCH_THRES1_REG,
    SENS_SAR_TOUCH_THRES2_REG,
    SENS_SAR_TOUCH_THRES3_REG,
    SENS_SAR_TOUCH_THRES4_REG,
    SENS_SAR_TOUCH_THRES5_REG
  };

  if (tp_wrap & 0x1) /* Odd */
    {
      /* All odd touch pads have the same position for the THN bits.
       * We can use any odd SENS_TOUCH_OUT_THN.
       */

      REG_SET_FIELD(reg_addr[tp_wrap / 2], SENS_TOUCH_OUT_TH1, threshold);
    }
  else /* Even */
    {
      /* All even touch pads have the same position for the THN bits.
       * We can use any even SENS_TOUCH_OUT_THN.
       */

      REG_SET_FIELD(reg_addr[tp_wrap / 2], SENS_TOUCH_OUT_TH0, threshold);
    }
}

/****************************************************************************
 * Name: touch_lh_get_threshold
 *
 * Description:
 *   Get the touch interrupt threshold for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The current interrupt threshold for that touch pad.
 *
 ****************************************************************************/

static inline uint16_t touch_lh_get_threshold(enum touch_pad_e tp)
{
  enum touch_pad_e tp_wrap = touch_lh_num_wrap(tp);

  const uint32_t reg_addr[] =
  {
    SENS_SAR_TOUCH_THRES1_REG,
    SENS_SAR_TOUCH_THRES2_REG,
    SENS_SAR_TOUCH_THRES3_REG,
    SENS_SAR_TOUCH_THRES4_REG,
    SENS_SAR_TOUCH_THRES5_REG
  };

  if (tp_wrap & 0x1) /* Odd */
    {
      /* All odd touch pads have the same position for the THN bits.
       * We can use any odd SENS_TOUCH_OUT_THN.
       */

      return REG_GET_FIELD(reg_addr[tp_wrap / 2], SENS_TOUCH_OUT_TH1);
    }
  else /* Even */
    {
      /* All even touch pads have the same position for the THN bits.
       * We can use any even SENS_TOUCH_OUT_THN.
       */

      return REG_GET_FIELD(reg_addr[tp_wrap / 2], SENS_TOUCH_OUT_TH0);
    }
}

/****************************************************************************
 * Name: touch_lh_set_trigger_mode
 *
 * Description:
 *   Set the touch interrupt trigger mode.
 *
 * Input Parameters:
 *   mode - The desired enum touch_trigger_mode_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_trigger_mode(enum touch_trigger_mode_e mode)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_OUT_SEL, mode);
}

/****************************************************************************
 * Name: touch_lh_get_trigger_mode
 *
 * Description:
 *   Get the touch interrupt trigger mode.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_trigger_mode_e.
 *
 ****************************************************************************/

static inline enum touch_trigger_mode_e touch_lh_get_trigger_mode(void)
{
  return (enum touch_trigger_mode_e)
    REG_GET_FIELD(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_OUT_SEL);
}

/****************************************************************************
 * Name: touch_lh_set_trigger_source
 *
 * Description:
 *   Set the touch interrupt trigger source.
 *
 * Input Parameters:
 *   src - The desired enum touch_trigger_src_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_trigger_source(enum touch_trigger_src_e src)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_OUT_1EN, src);
}

/****************************************************************************
 * Name: touch_lh_get_trigger_source
 *
 * Description:
 *   Get the touch interrupt trigger source.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_trigger_src_e.
 *
 ****************************************************************************/

static inline enum touch_trigger_src_e touch_lh_get_trigger_source(void)
{
  return (enum touch_trigger_src_e)
    REG_GET_FIELD(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_OUT_1EN);
}

/****************************************************************************
 * Name: touch_lh_set_channel_mask
 *
 * Description:
 *   Enable touch channels to be measured.
 *
 * Input Parameters:
 *   enable_mask - Bitmask containing the desired channels to be activated.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_channel_mask(uint16_t enable_mask)
{
  setbits(SENS_SAR_TOUCH_ENABLE_REG, (TOUCH_LH_BITS_SWAP(enable_mask) <<
    SENS_TOUCH_PAD_WORKEN_S));
}

/****************************************************************************
 * Name: touch_lh_get_channel_mask
 *
 * Description:
 *   Get the active touch channels to be measured.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Bitmask of the current chennels being measured.
 *
 ****************************************************************************/

static inline uint16_t touch_lh_get_channel_mask(void)
{
  return TOUCH_LH_BITS_SWAP(
    REG_GET_FIELD(SENS_SAR_TOUCH_ENABLE_REG, SENS_TOUCH_PAD_WORKEN));
}

/****************************************************************************
 * Name: touch_lh_clear_channel_mask
 *
 * Description:
 *   Disable touch channels being measured.
 *
 * Input Parameters:
 *   disable_mask - Bitmask containing the desired channels to be disabled.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_clear_channel_mask(uint16_t disable_mask)
{
  resetbits(SENS_SAR_TOUCH_ENABLE_REG, (TOUCH_LH_BITS_SWAP(disable_mask) <<
    SENS_TOUCH_PAD_WORKEN_S));
}

/****************************************************************************
 * Name: touch_lh_set_group_mask
 *
 * Description:
 *   Enable channels in touch interrupt groups.
 *
 * Input Parameters:
 *   group1_mask - Bitmask containing the desired channels to be
 *                 added to SET1;
 *   group2_mask - Bitmask containing the desired channels to be
 *                 added to SET2.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_set_group_mask(uint16_t group1_mask,
                                           uint16_t group2_mask)
{
  setbits(SENS_SAR_TOUCH_ENABLE_REG, (TOUCH_LH_BITS_SWAP(group1_mask) <<
    SENS_TOUCH_PAD_OUTEN1_S));

  setbits(SENS_SAR_TOUCH_ENABLE_REG, (TOUCH_LH_BITS_SWAP(group2_mask) <<
    SENS_TOUCH_PAD_OUTEN2_S));
}

/****************************************************************************
 * Name: touch_lh_get_channel_mask
 *
 * Description:
 *   Get the active touch interrupt groups.
 *
 * Input Parameters:
 *   group1_mask - Pointer to allocated uint16_t that will be modified to
 *                 return the current SET1 bitmask.
 *   group2_mask - Pointer to allocated uint16_t that will be modified to
 *                 return the current SET2 bitmask.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_get_group_mask(uint16_t *group1_mask,
                                           uint16_t *group2_mask)
{
  *group1_mask = TOUCH_LH_BITS_SWAP(
    REG_GET_FIELD(SENS_SAR_TOUCH_ENABLE_REG, SENS_TOUCH_PAD_OUTEN1));
  *group2_mask = TOUCH_LH_BITS_SWAP(
    REG_GET_FIELD(SENS_SAR_TOUCH_ENABLE_REG, SENS_TOUCH_PAD_OUTEN2));
}

/****************************************************************************
 * Name: touch_lh_clear_group_mask
 *
 * Description:
 *   Remove channels in touch interrupt groups.
 *
 * Input Parameters:
 *   group1_mask - Bitmask containing the desired channels to be
 *                 removed from SET1;
 *   group2_mask - Bitmask containing the desired channels to be
 *                 removed from SET2.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_clear_group_mask(uint16_t group1_mask,
                                             uint16_t group2_mask)
{
  resetbits(SENS_SAR_TOUCH_ENABLE_REG, (TOUCH_LH_BITS_SWAP(group1_mask) <<
    SENS_TOUCH_PAD_OUTEN1_S));

  resetbits(SENS_SAR_TOUCH_ENABLE_REG, (TOUCH_LH_BITS_SWAP(group2_mask) <<
    SENS_TOUCH_PAD_OUTEN2_S));
}

/****************************************************************************
 * Name: touch_lh_read_trigger_status_mask
 *
 * Description:
 *   Get the channels that triggered a touch interruption.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Bitmask of the channels that triggered a touch interruption.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_read_trigger_status_mask(void)
{
  return TOUCH_LH_BITS_SWAP(
    REG_GET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_EN));
}

/****************************************************************************
 * Name: touch_lh_clear_trigger_status_mask
 *
 * Description:
 *   Clear the touch interruption channels bitmask.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_clear_trigger_status_mask(void)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_EN_CLR, true);
}

/****************************************************************************
 * Name: touch_lh_intr_enable
 *
 * Description:
 *   Enable the touch interruption.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_intr_enable(void)
{
  REG_SET_FIELD(RTC_CNTL_INT_ENA_REG, RTC_CNTL_TOUCH_INT_ENA, true);
}

/****************************************************************************
 * Name: touch_lh_intr_disable
 *
 * Description:
 *   Disable the touch interruption.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_intr_disable(void)
{
  REG_SET_FIELD(RTC_CNTL_INT_ENA_REG, RTC_CNTL_TOUCH_INT_ENA, false);
}

/****************************************************************************
 * Name: touch_lh_intr_clear
 *
 * Description:
 *   Clear the touch interruption status.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_intr_clear(void)
{
  REG_SET_FIELD(RTC_CNTL_INT_CLR_REG, RTC_CNTL_TOUCH_INT_CLR, true);
}

/****************************************************************************
 * Name: touch_lh_read_raw_data
 *
 * Description:
 *   Get the measured value for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The current measured value for that touch pad.
 *
 ****************************************************************************/

static inline uint16_t touch_lh_read_raw_data(enum touch_pad_e tp)
{
  enum touch_pad_e tp_wrap = touch_lh_num_wrap(tp);

  const uint32_t reg_addr[] =
  {
    SENS_SAR_TOUCH_OUT1_REG,
    SENS_SAR_TOUCH_OUT2_REG,
    SENS_SAR_TOUCH_OUT3_REG,
    SENS_SAR_TOUCH_OUT4_REG,
    SENS_SAR_TOUCH_OUT5_REG,
  };

  if (tp_wrap & 0x1) /* Odd */
    {
      /* All odd touch pads have the same position for the MEAS_OUTN bits.
       * We can use any odd SENS_TOUCH_MEAS_OUTN.
       */

      return REG_GET_FIELD(reg_addr[tp_wrap / 2], SENS_TOUCH_MEAS_OUT1);
    }
  else /* Even */
    {
      /* All even touch pads have the same position for the MEAS_OUTN bits.
       * We can use any even SENS_TOUCH_MEAS_OUTN.
       */

      return REG_GET_FIELD(reg_addr[tp_wrap / 2], SENS_TOUCH_MEAS_OUT0);
    }
}

/****************************************************************************
 * Name: touch_lh_meas_is_done
 *
 * Description:
 *   Check if measurement is done.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   True if yes, false if no.
 *
 ****************************************************************************/

static inline bool touch_lh_meas_is_done(void)
{
  return (bool) REG_GET_FIELD(SENS_SAR_TOUCH_CTRL2_REG,
                              SENS_TOUCH_MEAS_DONE);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_TOUCH_LOWERHALF_H */
