/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_touch_lowerhalf.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TOUCH_LOWERHALF_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TOUCH_LOWERHALF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "xtensa.h"

#include "hardware/esp32s3_rtc_io.h"
#include "hardware/esp32s3_rtccntl.h"
#include "hardware/esp32s3_touch.h"
#include "hardware/esp32s3_sens.h"

#include "esp32s3_rt_timer.h"
#include "esp32s3_rtc_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOUCH_LH_READ_RAW           0x0
#define TOUCH_LH_READ_BENCHMARK     0x2
#define TOUCH_LH_READ_SMOOTH        0x3
#define TOUCH_LH_TIMER_FORCE_DONE   0x3
#define TOUCH_LH_TIMER_DONE         0x0

#define setbits(a, bs)   modifyreg32(a, 0, bs)
#define resetbits(a, bs) modifyreg32(a, bs, 0)

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const uint32_t rtcio_touch_reg[] =
{
  RTCIO_TOUCH_PAD0_REG,
  RTCIO_TOUCH_PAD1_REG,
  RTCIO_TOUCH_PAD2_REG,
  RTCIO_TOUCH_PAD3_REG,
  RTCIO_TOUCH_PAD4_REG,
  RTCIO_TOUCH_PAD5_REG,
  RTCIO_TOUCH_PAD6_REG,
  RTCIO_TOUCH_PAD7_REG,
  RTCIO_TOUCH_PAD8_REG,
  RTCIO_TOUCH_PAD9_REG,
  RTCIO_TOUCH_PAD10_REG,
  RTCIO_TOUCH_PAD11_REG,
  RTCIO_TOUCH_PAD12_REG,
  RTCIO_TOUCH_PAD13_REG,
  RTCIO_TOUCH_PAD14_REG
};

static const uint32_t sens_touch_thresh_reg[] =
{
  SENS_SAR_TOUCH_THRES1_REG,
  SENS_SAR_TOUCH_THRES2_REG,
  SENS_SAR_TOUCH_THRES3_REG,
  SENS_SAR_TOUCH_THRES4_REG,
  SENS_SAR_TOUCH_THRES5_REG,
  SENS_SAR_TOUCH_THRES6_REG,
  SENS_SAR_TOUCH_THRES7_REG,
  SENS_SAR_TOUCH_THRES8_REG,
  SENS_SAR_TOUCH_THRES9_REG,
  SENS_SAR_TOUCH_THRES10_REG,
  SENS_SAR_TOUCH_THRES11_REG,
  SENS_SAR_TOUCH_THRES12_REG,
  SENS_SAR_TOUCH_THRES13_REG,
  SENS_SAR_TOUCH_THRES14_REG
};

static const uint32_t sens_touch_status_reg[] =
{
  SENS_SAR_TOUCH_STATUS1_REG,
  SENS_SAR_TOUCH_STATUS2_REG,
  SENS_SAR_TOUCH_STATUS3_REG,
  SENS_SAR_TOUCH_STATUS4_REG,
  SENS_SAR_TOUCH_STATUS5_REG,
  SENS_SAR_TOUCH_STATUS6_REG,
  SENS_SAR_TOUCH_STATUS7_REG,
  SENS_SAR_TOUCH_STATUS8_REG,
  SENS_SAR_TOUCH_STATUS9_REG,
  SENS_SAR_TOUCH_STATUS10_REG,
  SENS_SAR_TOUCH_STATUS11_REG,
  SENS_SAR_TOUCH_STATUS12_REG,
  SENS_SAR_TOUCH_STATUS13_REG,
  SENS_SAR_TOUCH_STATUS14_REG
};

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

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL1_REG,
                RTC_CNTL_TOUCH_MEAS_NUM,
                meas_time);

  /* The waiting cycles (in 8MHz) between TOUCH_START and TOUCH_XPD */

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_XPD_WAIT,
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
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL1_REG,
                       RTC_CNTL_TOUCH_MEAS_NUM);
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
  /* Touch sensor sleep cycle Time = sleep_cycle / RTC_SLOW_CLK (150k) */

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL1_REG,
                RTC_CNTL_TOUCH_SLEEP_CYCLES,
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
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL1_REG,
                       RTC_CNTL_TOUCH_SLEEP_CYCLES);
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
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_DREFH, refh);
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
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_DREFH);
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
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_DREFL, refl);
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
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_DREFL);
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
  touch_lh_set_voltage_attenuation(enum touch_volt_atten_e atten)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_DRANGE, atten);
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
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_DRANGE);
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
  if (tp < TOUCH_PAD_NUM10)
    {
      modifyreg32(RTC_CNTL_TOUCH_DAC_REG,
        RTC_CNTL_TOUCH_PAD0_DAC_V <<
        (RTC_CNTL_TOUCH_PAD0_DAC_S - tp * 3),
        (slope & RTC_CNTL_TOUCH_PAD0_DAC_V) <<
        (RTC_CNTL_TOUCH_PAD0_DAC_S - tp * 3));
    }
  else
    {
      modifyreg32(RTC_CNTL_TOUCH_DAC1_REG,
        RTC_CNTL_TOUCH_PAD10_DAC_V <<
        (RTC_CNTL_TOUCH_PAD10_DAC_S - (tp - TOUCH_PAD_NUM10) * 3),
        (slope & RTC_CNTL_TOUCH_PAD10_DAC_V) <<
        (RTC_CNTL_TOUCH_PAD10_DAC_S - (tp - TOUCH_PAD_NUM10) * 3));
    }
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
  enum touch_cnt_slope_e slope;

  if (tp < TOUCH_PAD_NUM10)
    {
      slope = (enum touch_cnt_slope_e) (getreg32(RTC_CNTL_TOUCH_DAC_REG) >>
        (RTC_CNTL_TOUCH_PAD0_DAC_S - tp * 3)) &
        RTC_CNTL_TOUCH_PAD0_DAC_V;
    }
  else
    {
      slope = (enum touch_cnt_slope_e) (getreg32(RTC_CNTL_TOUCH_DAC1_REG) >>
        (RTC_CNTL_TOUCH_PAD10_DAC_S - (tp - TOUCH_PAD_NUM10) * 3)) &
        RTC_CNTL_TOUCH_PAD10_DAC_V;
    }

  return slope;
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
  /* All touch pads have the same position for the TIE_OPT bits.
   * We can use any RTC_IO_TOUCH_PADn_TIE_OPT.
   */

  REG_SET_FIELD(rtcio_touch_reg[tp], RTCIO_TOUCH_PAD0_TIE_OPT, opt);
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
  /* All touch pads have the same position for the TIE_OPT bits.
   * We can use any RTC_IO_TOUCH_PADn_TIE_OPT.
   */

  return (enum touch_tie_opt_e) REG_GET_FIELD(rtcio_touch_reg[tp],
                                              RTCIO_TOUCH_PAD0_TIE_OPT);
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
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_START_FORCE,
                mode);
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
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_START_FORCE);
}

/****************************************************************************
 * Name: touch_lh_clkgate
 *
 * Description:
 *   Enable/disable clock gate of touch sensor.
 *
 * Input Parameters:
 *   enable - True or false.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_clkgate(bool enable)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_CLKGATE_EN,
                enable);
}

/****************************************************************************
 * Name: touch_lh_clkgate_get_state
 *
 * Description:
 *   Get touch sensor clkgate state.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   True if open, false if closed.
 *
 ****************************************************************************/

static inline bool touch_lh_clkgate_get_state(void)
{
  return (bool) REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                              RTC_CNTL_TOUCH_CLKGATE_EN);
}

/****************************************************************************
 * Name: touch_lh_timer_force_done
 *
 * Description:
 *   Touch timer triggers measurement and always waits measurement done.
 *   Force done for touch timer ensures that the timer always can get the
 *   measurement done signal.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_timer_force_done(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_TIMER_FORCE_DONE,
                TOUCH_LH_TIMER_FORCE_DONE);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_TIMER_FORCE_DONE,
                TOUCH_LH_TIMER_DONE);
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
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_TIMER_FORCE_DONE,
                TOUCH_LH_TIMER_FORCE_DONE);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_TIMER_FORCE_DONE,
                TOUCH_LH_TIMER_DONE);

  bool reg_val = (touch_lh_get_fsm_mode() == TOUCH_FSM_MODE_TIMER) ? 1 : 0;

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_SLP_TIMER_EN,
                reg_val);
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
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_START_EN,
                false);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_SLP_TIMER_EN,
                false);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_TIMER_FORCE_DONE,
                TOUCH_LH_TIMER_FORCE_DONE);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_TIMER_FORCE_DONE,
                TOUCH_LH_TIMER_DONE);
}

/****************************************************************************
 * Name: touch_lh_get_fsm_state
 *
 * Description:
 *   Get touch sensor FSM state.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   True if open, false if closed.
 *
 ****************************************************************************/

static inline bool touch_lh_get_fsm_state(void)
{
  return (bool) REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                              RTC_CNTL_TOUCH_SLP_TIMER_EN);
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
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_START_EN,
                false);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_START_EN,
                true);
}

/****************************************************************************
 * Name: touch_lh_set_threshold
 *
 * Description:
 *   Set the touch interrupt threshold for a given touch pad.
 *   The threshold determines the sensitivity of the touch sensor.
 *   The threshold is the original value of the trigger state minus the
 *   benchmark value.
 *   If set to "TOUCH_PAD_THRESHOLD_MAX", the touch is never be triggered.
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
                                          uint32_t threshold)
{
  /* All touch pads have the same position for the threshold bits.
   * We can use any SENS_TOUCH_OUT_THN.
   */

  REG_SET_FIELD(sens_touch_thresh_reg[tp - 1],
                SENS_TOUCH_OUT_TH1,
                threshold);
}

/****************************************************************************
 * Name: touch_lh_get_threshold
 *
 * Description:
 *   Get the touch interrupt threshold for a given touch pad.
 *   The threshold determines the sensitivity of the touch sensor.
 *   The threshold is the original value of the trigger state minus the
 *   benchmark value.
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
  /* All touch pads have the same position for the threshold bits.
   * We can use any SENS_TOUCH_OUT_THN.
   */

  return REG_GET_FIELD(sens_touch_thresh_reg[tp - 1], SENS_TOUCH_OUT_TH1);
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
  setbits(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
          (enable_mask & ((1 << TOUCH_SENSOR_PINS) - 1)) <<
          RTC_CNTL_TOUCH_SCAN_PAD_MAP_S);

  setbits(SENS_SAR_TOUCH_CONF_REG,
          (enable_mask & ((1 << TOUCH_SENSOR_PINS) - 1)) <<
          SENS_TOUCH_OUTEN_S);
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
  return (REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                        RTC_CNTL_TOUCH_SCAN_PAD_MAP) &
          REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                        SENS_TOUCH_OUTEN));
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
  resetbits(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
            (disable_mask & ((1 << TOUCH_SENSOR_PINS) - 1)) <<
            RTC_CNTL_TOUCH_SCAN_PAD_MAP_S);

  resetbits(SENS_SAR_TOUCH_CONF_REG,
            (disable_mask & ((1 << TOUCH_SENSOR_PINS) - 1)) <<
            SENS_TOUCH_OUTEN_S);
}

/****************************************************************************
 * Name: touch_lh_read_trigger_status_mask
 *
 * Description:
 *   Get the channels that triggered a touch interrupt.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Bitmask of the channels that triggered a touch interrupt.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_read_trigger_status_mask(void)
{
  return REG_GET_FIELD(SENS_SAR_TOUCH_CHN_ST_REG, SENS_TOUCH_PAD_ACTIVE);
}

/****************************************************************************
 * Name: touch_lh_clear_trigger_status_mask
 *
 * Description:
 *   Clear the touch interrupt channels bitmask.
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
  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG, SENS_TOUCH_STATUS_CLR, true);
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

static inline uint32_t touch_lh_read_raw_data(enum touch_pad_e tp)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_DATA_SEL,
                TOUCH_LH_READ_RAW);

  /* All touch pads have the same position for the data bits.
   * We can use any SENS_TOUCH_PADN_DATA.
   */

  return REG_GET_FIELD(sens_touch_status_reg[tp - 1],
                       SENS_TOUCH_PAD1_DATA);
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
  return (bool) REG_GET_FIELD(SENS_SAR_TOUCH_CHN_ST_REG,
                              SENS_TOUCH_MEAS_DONE);
}

/****************************************************************************
 * Name: touch_lh_reset
 *
 * Description:
 *   Reset the whole of touch module.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_reset(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_RESET,
                false);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_RESET,
                true);

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_RESET,
                false);
}

/****************************************************************************
 * Name: touch_lh_set_idle_channel_connect
 *
 * Description:
 *   Set connection type of touch channel in idle status.
 *   When a channel is in measurement mode, other initialized channels are in
 *   idle mode.
 *   The touch channel is generally adjacent to the trace, so the connection
 *   state of the idle channel affects the stability and sensitivity of the
 *   test channel.
 *   The `CONN_HIGHZ`(high resistance) setting increases the sensitivity of
 *   touch channels.
 *   The `CONN_GND`(grounding) setting increases the stability of touch
 *   channels.
 *
 * Input Parameters:
 *   type - The desired enum touch_conn_type_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_set_idle_channel_connect(enum touch_conn_type_e type)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                RTC_CNTL_TOUCH_INACTIVE_CONNECTION,
                type);
}

/****************************************************************************
 * Name: touch_lh_get_idle_channel_connect
 *
 * Description:
 *   Get connection type of touch channel in idle status.
 *   When a channel is in measurement mode, other initialized channels are in
 *   idle mode.
 *   The touch channel is generally adjacent to the trace, so the connection
 *   state of the idle channel affects the stability and sensitivity of the
 *   test channel.
 *   The `CONN_HIGHZ`(high resistance) setting increases the sensitivity of
 *   touch channels.
 *   The `CONN_GND`(grounding) setting increases the stability of touch
 *   channels.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_conn_type_e value.
 *
 ****************************************************************************/

static inline enum touch_conn_type_e
  touch_lh_get_idle_channel_connect(void)
{
  return (enum touch_conn_type_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                  RTC_CNTL_TOUCH_INACTIVE_CONNECTION);
}

/****************************************************************************
 * Name: touch_lh_get_idle_channel_connect
 *
 * Description:
 *   Get the current measure channel.
 *   Touch sensor measurement is cyclic scan mode.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current touch channel.
 *
 ****************************************************************************/

static inline enum touch_pad_e touch_lh_get_current_meas_channel(void)
{
  return (enum touch_pad_e)
    REG_GET_FIELD(SENS_SAR_TOUCH_STATUS0_REG,
                  SENS_TOUCH_SCAN_CURR);
}

/****************************************************************************
 * Name: touch_lh_intr_enable
 *
 * Description:
 *   Enable touch sensor interrupt by bitmask.
 *
 * Input Parameters:
 *   int_mask - enum touch_pad_intr_mask_e interrupt type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_intr_enable(enum touch_intr_mask_e int_mask)
{
  if (int_mask & TOUCH_INTR_MASK_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TS_REG,
                    RTC_CNTL_RTC_TOUCH_DONE_INT_ENA_W1TS,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_ACTIVE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TS_REG,
                    RTC_CNTL_RTC_TOUCH_ACTIVE_INT_ENA_W1TS,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_INACTIVE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TS_REG,
                    RTC_CNTL_RTC_TOUCH_INACTIVE_INT_ENA_W1TS,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_SCAN_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TS_REG,
                    RTC_CNTL_RTC_TOUCH_SCAN_DONE_INT_ENA_W1TS,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_TIMEOUT)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TS_REG,
                    RTC_CNTL_RTC_TOUCH_TIMEOUT_INT_ENA_W1TS,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_PROXI_MEAS_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TS_REG,
                    RTC_CNTL_RTC_TOUCH_APPROACH_LOOP_DONE_INT_ENA_W1TS,
                    true);
    }
}

/****************************************************************************
 * Name: touch_lh_intr_disable
 *
 * Description:
 *   Disable touch sensor interrupt by bitmask.
 *
 * Input Parameters:
 *   int_mask - enum touch_pad_intr_mask_e interrupt type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_intr_disable(enum touch_intr_mask_e int_mask)
{
  if (int_mask & TOUCH_INTR_MASK_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TC_REG,
                    RTC_CNTL_RTC_TOUCH_DONE_INT_ENA_W1TC,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_ACTIVE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TC_REG,
                    RTC_CNTL_RTC_TOUCH_ACTIVE_INT_ENA_W1TC,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_INACTIVE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TC_REG,
                    RTC_CNTL_RTC_TOUCH_INACTIVE_INT_ENA_W1TC,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_SCAN_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TC_REG,
                    RTC_CNTL_RTC_TOUCH_SCAN_DONE_INT_ENA_W1TC,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_TIMEOUT)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TC_REG,
                    RTC_CNTL_RTC_TOUCH_TIMEOUT_INT_ENA_W1TC,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_PROXI_MEAS_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_ENA_RTC_W1TC_REG,
                    RTC_CNTL_RTC_TOUCH_APPROACH_LOOP_DONE_INT_ENA_W1TC,
                    true);
    }
}

/****************************************************************************
 * Name: touch_lh_intr_clear
 *
 * Description:
 *   Clear touch sensor interrupt by bitmask.
 *
 * Input Parameters:
 *   int_mask - enum touch_pad_intr_mask_e interrupt type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_intr_clear(enum touch_intr_mask_e int_mask)
{
  if (int_mask & TOUCH_INTR_MASK_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_CLR_RTC_REG,
                    RTC_CNTL_RTC_TOUCH_DONE_INT_CLR,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_ACTIVE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_CLR_RTC_REG,
                    RTC_CNTL_RTC_TOUCH_ACTIVE_INT_CLR,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_INACTIVE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_CLR_RTC_REG,
                    RTC_CNTL_RTC_TOUCH_INACTIVE_INT_CLR,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_SCAN_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_CLR_RTC_REG,
                    RTC_CNTL_RTC_TOUCH_SCAN_DONE_INT_CLR,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_TIMEOUT)
    {
      REG_SET_FIELD(RTC_CNTL_INT_CLR_RTC_REG,
                    RTC_CNTL_RTC_TOUCH_TIMEOUT_INT_CLR,
                    true);
    }

  if (int_mask & TOUCH_INTR_MASK_PROXI_MEAS_DONE)
    {
      REG_SET_FIELD(RTC_CNTL_INT_CLR_RTC_REG,
                    RTC_CNTL_RTC_TOUCH_APPROACH_LOOP_DONE_INT_CLR,
                    true);
    }
}

/****************************************************************************
 * Name: touch_lh_read_intr_status_mask
 *
 * Description:
 *   Get the bitmask of touch sensor interrupt status.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   enum touch_pad_intr_mask_e interrupt type.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_read_intr_status_mask(void)
{
  uint32_t intr_st = getreg32(RTC_CNTL_INT_ST_RTC_REG);
  uint32_t intr_msk = 0;

  if (intr_st & RTC_CNTL_RTC_TOUCH_DONE_INT_ST_M)
    {
      intr_msk |= TOUCH_INTR_MASK_DONE;
    }

  if (intr_st & RTC_CNTL_RTC_TOUCH_ACTIVE_INT_ST_M)
    {
      intr_msk |= TOUCH_INTR_MASK_ACTIVE;
    }

  if (intr_st & RTC_CNTL_RTC_TOUCH_INACTIVE_INT_ST_M)
    {
      intr_msk |= TOUCH_INTR_MASK_INACTIVE;
    }

  if (intr_st & RTC_CNTL_RTC_TOUCH_SCAN_DONE_INT_ST_M)
    {
      intr_msk |= TOUCH_INTR_MASK_SCAN_DONE;
    }

  if (intr_st & RTC_CNTL_RTC_TOUCH_TIMEOUT_INT_ST_M)
    {
      intr_msk |= TOUCH_INTR_MASK_TIMEOUT;
    }

  if (intr_st & RTC_CNTL_RTC_TOUCH_APPROACH_LOOP_DONE_INT_ST_M)
    {
      intr_msk |= TOUCH_INTR_MASK_PROXI_MEAS_DONE;
    }

  return (intr_msk & TOUCH_INTR_MASK_ALL);
}

/****************************************************************************
 * Name: touch_lh_timeout_enable
 *
 * Description:
 *   Enable the timeout check for all touch sensor channels measurements.
 *   When the touch reading exceeds the measurement threshold,
 *   If enable: a timeout interrupt will be generated and it will go to the
 *   next channel measurement.
 *   If disable: the FSM is always on the channel, until the measurement of
 *   this channel is over.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_timeout_enable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_TIMEOUT_CTRL_REG,
                RTC_CNTL_TOUCH_TIMEOUT_EN,
                true);
}

/****************************************************************************
 * Name: touch_lh_timeout_disable
 *
 * Description:
 *   Disable the timeout check for all touch sensor channels measurements.
 *   When the touch reading exceeds the measurement threshold,
 *   If enable: a timeout interrupt will be generated and it will go to the
 *   next channel measurement.
 *   If disable: the FSM is always on the channel, until the measurement of
 *   this channel is over.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_timeout_disable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_TIMEOUT_CTRL_REG,
                RTC_CNTL_TOUCH_TIMEOUT_EN,
                false);
}

/****************************************************************************
 * Name: touch_lh_timeout_set_threshold
 *
 * Description:
 *   Set timeout threshold for all touch sensor channels measurements.
 *   Compared with touch readings.
 *
 * Input Parameters:
 *   threshold - The maximum time measured on one channel.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_timeout_set_threshold(uint32_t threshold)
{
  return REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_TIMEOUT_CTRL_REG,
                       RTC_CNTL_TOUCH_TIMEOUT_NUM,
                       threshold);
}

/****************************************************************************
 * Name: touch_lh_timeout_get_threshold
 *
 * Description:
 *   Get timeout threshold for all touch sensor channels measurements.
 *   Compared with touch readings.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current maximum time measured on one channel.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_timeout_get_threshold(void)
{
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_TIMEOUT_CTRL_REG,
                       RTC_CNTL_TOUCH_TIMEOUT_NUM);
}

/****************************************************************************
 * Name: touch_lh_filter_read_smooth
 *
 * Description:
 *   Get the smoothed measured value for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The current filtered value for that touch pad.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_filter_read_smooth(enum touch_pad_e tp)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_DATA_SEL,
                TOUCH_LH_READ_SMOOTH);

  /* All touch pads have the same position for the data bits.
   * We can use any SENS_TOUCH_PADN_DATA.
   */

  return REG_GET_FIELD(sens_touch_status_reg[tp - 1],
                       SENS_TOUCH_PAD1_DATA);
}

/****************************************************************************
 * Name: touch_lh_read_benchmark
 *
 * Description:
 *   Get the benchmark value for a given touch pad.
 *   After initialization, the benchmark value is the maximum during the
 *   first measurement period.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The current benchmark value for that touch pad.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_read_benchmark(enum touch_pad_e tp)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_DATA_SEL,
                TOUCH_LH_READ_BENCHMARK);

  /* All touch pads have the same position for the data bits.
   * We can use any SENS_TOUCH_PADN_DATA.
   */

  return REG_GET_FIELD(sens_touch_status_reg[tp - 1],
                       SENS_TOUCH_PAD1_DATA);
}

/****************************************************************************
 * Name: touch_lh_reset_benchmark
 *
 * Description:
 *   Force reset benchmark to raw data of touch sensor.
 *   If call this API, make sure enable clock gate first.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_reset_benchmark(enum touch_pad_e tp)
{
  if (tp == TOUCH_PAD_ALL)
    {
      REG_SET_FIELD(SENS_SAR_TOUCH_CHN_ST_REG,
                    SENS_TOUCH_CHANNEL_CLR,
                    TOUCH_BIT_MASK_ALL);
    }
  else
    {
      REG_SET_FIELD(SENS_SAR_TOUCH_CHN_ST_REG,
                    SENS_TOUCH_CHANNEL_CLR,
                    (1U << tp));
    }
}

/****************************************************************************
 * Name: touch_lh_filter_set_filter_mode
 *
 * Description:
 *   Set filter mode. The input of the filter is the raw value of touch
 *   reading, and the output of the filter is involved in the judgment of the
 *   touch state.
 *
 * Input Parameters:
 *   mode - The desired enum touch_filter_mode_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_filter_set_filter_mode(enum touch_filter_mode_e mode)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_FILTER_MODE,
                mode);
}

/****************************************************************************
 * Name: touch_lh_filter_get_filter_mode
 *
 * Description:
 *   Get filter mode. The input of the filter is the raw value of touch
 *   reading, and the output of the filter is involved in the judgment of the
 *   touch state.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_filter_mode_e value.
 *
 ****************************************************************************/

static inline enum touch_filter_mode_e
  touch_lh_filter_get_filter_mode(void)
{
  return (enum touch_filter_mode_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                  RTC_CNTL_TOUCH_FILTER_MODE);
}

/****************************************************************************
 * Name: touch_lh_filter_set_smooth_mode
 *
 * Description:
 *   Set filter mode. The input to the filter is raw data and the output is
 *   the smooth data.
 *   The smooth data is used to determine the touch status.
 *
 * Input Parameters:
 *   mode - The desired enum touch_smooth_mode_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_filter_set_smooth_mode(enum touch_smooth_mode_e mode)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_SMOOTH_LVL,
                mode);
}

/****************************************************************************
 * Name: touch_lh_filter_get_smooth_mode
 *
 * Description:
 *   Set filter mode. The input to the filter is raw data and the output is
 *   the smooth data.
 *   The smooth data is used to determine the touch status.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_smooth_mode_e value.
 *
 ****************************************************************************/

static inline enum touch_smooth_mode_e
  touch_lh_filter_get_smooth_mode(void)
{
  return (enum touch_smooth_mode_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                  RTC_CNTL_TOUCH_SMOOTH_LVL);
}

/****************************************************************************
 * Name: touch_lh_filter_set_debounce
 *
 * Description:
 *   Set debounce count, such as `n`. If the measured values continue to
 *   exceed the threshold for `n+1` times, it is determined that the touch
 *   sensor state changes.
 *
 * Input Parameters:
 *   dbc_cnt - Debounce count value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_filter_set_debounce(uint32_t dbc_cnt)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_DEBOUNCE,
                dbc_cnt);
}

/****************************************************************************
 * Name: touch_lh_filter_get_debounce
 *
 * Description:
 *   Get debounce count, such as `n`. If the measured values continue to
 *   exceed the threshold for `n+1` times, it is determined that the touch
 *   sensor state changes.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current debounce count value.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_filter_get_debounce(void)
{
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                       RTC_CNTL_TOUCH_DEBOUNCE);
}

/****************************************************************************
 * Name: touch_lh_filter_set_noise_thres
 *
 * Description:
 *   Set noise threshold coefficient. Higher = More noise resistance.
 *   The actual noise should be less than (noise coefficient * touch
 *   threshold).
 *   Range: 0 ~ 3. The coefficient is 0: 4/8; 1: 3/8; 2: 2/8; 3: 1;
 *
 * Input Parameters:
 *   noise_thr - Noise threshold coefficient.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_filter_set_noise_thres(uint32_t noise_thr)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_NOISE_THRES,
                noise_thr);

  /* config2 in IDF */

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_NEG_NOISE_THRES,
                noise_thr);

  /* config1 in IDF */

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_NEG_NOISE_LIMIT,
                0xf);

  /* config3 in IDF */

  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_HYSTERESIS,
                2);
}

/****************************************************************************
 * Name: touch_lh_filter_get_noise_thres
 *
 * Description:
 *   Get noise threshold coefficient. Higher = More noise resistance.
 *   The actual noise should be less than (noise coefficient * touch
 *   threshold).
 *   Range: 0 ~ 3. The coefficient is 0: 4/8; 1: 3/8; 2: 2/8; 3: 1;
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current noise threshold coefficient.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_filter_get_noise_thres(void)
{
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                       RTC_CNTL_TOUCH_NOISE_THRES);
}

/****************************************************************************
 * Name: touch_lh_filter_set_jitter_step
 *
 * Description:
 *   If filter mode is jitter, should set filter step for jitter.
 *   Range: 0 ~ 15
 *
 * Input Parameters:
 *   step - The step size of the data change.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_filter_set_jitter_step(uint32_t step)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_JITTER_STEP,
                step);
}

/****************************************************************************
 * Name: touch_lh_filter_get_jitter_step
 *
 * Description:
 *   If filter mode is jitter, should set filter step for jitter.
 *   Range: 0 ~ 15
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current step size of the data change.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_filter_get_jitter_step(void)
{
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                       RTC_CNTL_TOUCH_JITTER_STEP);
}

/****************************************************************************
 * Name: touch_lh_filter_enable
 *
 * Description:
 *   Enable touch sensor filter and detection algorithm.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_filter_enable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_FILTER_EN,
                true);
}

/****************************************************************************
 * Name: touch_lh_filter_disable
 *
 * Description:
 *   Disable touch sensor filter and detection algorithm.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_filter_disable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_FILTER_EN,
                false);
}

/****************************************************************************
 * Name: touch_lh_denoise_enable
 *
 * Description:
 *   Enable denoise function.
 *   T0 is an internal channel that does not have a corresponding external
 *   GPIO.
 *   T0 will work simultaneously with the measured channel Tn. Finally, the
 *   actual measured value of Tn is the value after subtracting lower bits
 *   of T0.
 *   This denoise function filters out interference introduced on all
 *   channels, such as noise introduced by the power supply and external EMI.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_denoise_enable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_DENOISE_EN,
                true);
}

/****************************************************************************
 * Name: touch_lh_denoise_disable
 *
 * Description:
 *   Disable denoise function.
 *   T0 is an internal channel that does not have a corresponding external
 *   GPIO.
 *   T0 will work simultaneously with the measured channel Tn. Finally, the
 *   actual measured value of Tn is the value after subtracting lower bits
 *   of T0.
 *   This denoise function filters out interference introduced on all
 *   channels, such as noise introduced by the power supply and external EMI.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_denoise_disable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_FILTER_CTRL_REG,
                RTC_CNTL_TOUCH_DENOISE_EN,
                false);
}

/****************************************************************************
 * Name: touch_lh_denoise_set_cap_level
 *
 * Description:
 *   Set internal reference capacitance of denoise channel.
 *   Select the appropriate internal reference capacitance value so that the
 *   reading of denoise channel is closest to the reading of the channel
 *   being measured.
 *
 * Input Parameters:
 *   cap_level - The desired enum touch_pad_denoise_cap_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_denoise_set_cap_level(enum touch_denoise_cap_e cap_level)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_REFC,
                cap_level);
}

/****************************************************************************
 * Name: touch_lh_denoise_get_cap_level
 *
 * Description:
 *   Get internal reference capacitance of denoise channel.
 *   Select the appropriate internal reference capacitance value so that the
 *   reading of denoise channel is closest to the reading of the channel
 *   being measured.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_pad_denoise_cap_e value.
 *
 ****************************************************************************/

static inline enum touch_denoise_cap_e touch_lh_denoise_get_cap_level(void)
{
  return (enum touch_denoise_cap_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG, RTC_CNTL_TOUCH_REFC);
}

/****************************************************************************
 * Name: touch_lh_denoise_set_grade
 *
 * Description:
 *   Set denoise range of denoise channel.
 *   Determined by measuring the noise amplitude of the denoise channel.
 *
 * Input Parameters:
 *   grade - The desired enum touch_denoise_grade_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_denoise_set_grade(enum touch_denoise_grade_e grade)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                RTC_CNTL_TOUCH_DENOISE_RES,
                grade);
}

/****************************************************************************
 * Name: touch_lh_denoise_get_grade
 *
 * Description:
 *   Get denoise range of denoise channel.
 *   Determined by measuring the noise amplitude of the denoise channel.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_denoise_grade_e value.
 *
 ****************************************************************************/

static inline enum touch_denoise_grade_e touch_lh_denoise_get_grade(void)
{
  return (enum touch_denoise_grade_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                  RTC_CNTL_TOUCH_DENOISE_RES);
}

/****************************************************************************
 * Name: touch_lh_denoise_read_data
 *
 * Description:
 *   Read denoise measure value (TOUCH_PAD_NUM0).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current denoise value.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_denoise_read_data(void)
{
  return REG_GET_FIELD(SENS_SAR_TOUCH_DENOISE_REG, SENS_TOUCH_DENOISE_DATA);
}

/****************************************************************************
 * Name: touch_lh_waterproof_set_guard_pad
 *
 * Description:
 *   Set touch channel used for guard pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_waterproof_set_guard_pad(enum touch_pad_e tp)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                RTC_CNTL_TOUCH_OUT_RING,
                tp);
}

/****************************************************************************
 * Name: touch_lh_waterproof_get_guard_pad
 *
 * Description:
 *   Get touch channel used for guard pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_pad_e channel.
 *
 ****************************************************************************/

static inline enum touch_pad_e touch_lh_waterproof_get_guard_pad(void)
{
  return (enum touch_pad_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG, RTC_CNTL_TOUCH_OUT_RING);
}

/****************************************************************************
 * Name: touch_lh_waterproof_set_sheild_driver
 *
 * Description:
 *   Set max equivalent capacitance for sheild channel.
 *   The equivalent capacitance of the shielded channel can be calculated
 *   from the reading of denoise channel.
 *
 * Input Parameters:
 *   level - The desired enum touch_shield_driver_e value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_waterproof_set_sheild_driver(enum touch_shield_driver_e level)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                RTC_CNTL_TOUCH_BUFDRV,
                level);
}

/****************************************************************************
 * Name: touch_lh_waterproof_get_sheild_driver
 *
 * Description:
 *   Set max equivalent capacitance for sheild channel.
 *   The equivalent capacitance of the shielded channel can be calculated
 *   from the reading of denoise channel.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_shield_driver_e value.
 *
 ****************************************************************************/

static inline enum touch_shield_driver_e
  touch_lh_waterproof_get_sheild_driver(void)
{
  return (enum touch_shield_driver_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG, RTC_CNTL_TOUCH_BUFDRV);
}

/****************************************************************************
 * Name: touch_lh_waterproof_enable
 *
 * Description:
 *   Enable parameter of waterproof function.
 *   The waterproof function includes a shielded channel (TOUCH_PAD_NUM14)
 *   and a guard channel.
 *   Guard pad is used to detect the large area of water covering the touch
 *   panel.
 *   Shield pad is used to shield the influence of water droplets covering
 *   the touch panel.
 *   It is generally designed as a grid and is placed around the touch
 *   buttons.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_waterproof_enable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                RTC_CNTL_TOUCH_SHIELD_PAD_EN,
                true);
}

/****************************************************************************
 * Name: touch_lh_waterproof_disable
 *
 * Description:
 *   Disable parameter of waterproof function.
 *   The waterproof function includes a shielded channel (TOUCH_PAD_NUM14)
 *   and a guard channel.
 *   Guard pad is used to detect the large area of water covering the touch
 *   panel.
 *   Shield pad is used to shield the influence of water droplets covering
 *   the touch panel.
 *   It is generally designed as a grid and is placed around the touch
 *   buttons.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_waterproof_disable(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SCAN_CTRL_REG,
                RTC_CNTL_TOUCH_SHIELD_PAD_EN,
                false);
}

/****************************************************************************
 * Name: touch_lh_proximity_set_channel_num
 *
 * Description:
 *   Set touch channel number for proximity pad.
 *   If disable the proximity pad, point this pad to TOUCH_PAD_NUM0
 *
 * Input Parameters:
 *   prox_pad - The array of three proximity pads.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void
  touch_lh_proximity_set_channel_num(const enum touch_pad_e prox_pad[])
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_APPROACH_PAD0,
                prox_pad[0]);

  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_APPROACH_PAD1,
                prox_pad[1]);

  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_APPROACH_PAD2,
                prox_pad[2]);
}

/****************************************************************************
 * Name: touch_lh_proximity_get_channel_num
 *
 * Description:
 *   Get touch channel number for proximity pad.
 *   If disable the proximity pad, point this pad to TOUCH_PAD_NUM0
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The array of three proximity pads through prox_pad.
 *
 ****************************************************************************/

static inline void
  touch_lh_proximity_get_channel_num(enum touch_pad_e prox_pad[])
{
  prox_pad[0] = REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                              SENS_TOUCH_APPROACH_PAD0);

  prox_pad[1] = REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                              SENS_TOUCH_APPROACH_PAD1);

  prox_pad[2] = REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                              SENS_TOUCH_APPROACH_PAD2);
}

/****************************************************************************
 * Name: touch_lh_proximity_set_meas_times
 *
 * Description:
 *   Set cumulative measurement times for proximity pad.
 *
 * Input Parameters:
 *   level - The cumulative number of measurement cycles.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_proximity_set_meas_times(uint32_t times)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_APPROACH_REG,
                RTC_CNTL_TOUCH_APPROACH_MEAS_TIME,
                times);
}

/****************************************************************************
 * Name: touch_lh_proximity_get_meas_times
 *
 * Description:
 *   Get cumulative measurement times for proximity pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current cumulative number of measurement cycles.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_proximity_get_meas_times(void)
{
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_APPROACH_REG,
                       RTC_CNTL_TOUCH_APPROACH_MEAS_TIME);
}

/****************************************************************************
 * Name: touch_lh_proximity_read_meas_cnt
 *
 * Description:
 *   Read current cumulative measurement times for proximity pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The number of measurement cycles for touch pad channel.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_proximity_read_meas_cnt(enum touch_pad_e tp)
{
  if (REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG, SENS_TOUCH_APPROACH_PAD0) == tp)
    {
      return REG_GET_FIELD(SENS_SAR_TOUCH_APPR_STATUS_REG,
                           SENS_TOUCH_APPROACH_PAD0_CNT);
    }
  else if (REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
           SENS_TOUCH_APPROACH_PAD1) == tp)
    {
      return REG_GET_FIELD(SENS_SAR_TOUCH_APPR_STATUS_REG,
                           SENS_TOUCH_APPROACH_PAD1_CNT);
    }
  else if (REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
           SENS_TOUCH_APPROACH_PAD2) == tp)
    {
      return REG_GET_FIELD(SENS_SAR_TOUCH_APPR_STATUS_REG,
                           SENS_TOUCH_APPROACH_PAD2_CNT);
    }

  return 0;
}

/****************************************************************************
 * Name: touch_lh_proximity_pad_check
 *
 * Description:
 *   Check if the touch sensor channel is the proximity pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   True if yes or false if no.
 *
 ****************************************************************************/

static inline bool touch_lh_proximity_pad_check(enum touch_pad_e tp)
{
  if ((REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
       SENS_TOUCH_APPROACH_PAD0) != tp) &&
      (REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
       SENS_TOUCH_APPROACH_PAD1) != tp) &&
      (REG_GET_FIELD(SENS_SAR_TOUCH_CONF_REG,
       SENS_TOUCH_APPROACH_PAD2) != tp))
    {
      return false;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: touch_lh_sleep_set_channel_num
 *
 * Description:
 *   Set touch channel number for sleep pad.
 *   Only one touch sensor channel is supported in deep sleep mode.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_sleep_set_channel_num(enum touch_pad_e tp)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                RTC_CNTL_TOUCH_SLP_PAD,
                tp);
}

/****************************************************************************
 * Name: touch_lh_sleep_get_channel_num
 *
 * Description:
 *   Get touch channel number for sleep pad.
 *   Only one touch sensor channel is supported in deep sleep mode.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current enum touch_pad_e channel.
 *
 ****************************************************************************/

static inline enum touch_pad_e touch_lh_sleep_get_channel_num(void)
{
  return (enum touch_pad_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG, RTC_CNTL_TOUCH_SLP_PAD);
}

/****************************************************************************
 * Name: touch_lh_sleep_set_threshold
 *
 * Description:
 *   Set the trigger threshold of touch sensor in deep sleep.
 *   The threshold determines the sensitivity of the touch sensor.
 *   The threshold is the original value of the trigger state minus the
 *   benchmark value.
 *   In general, the touch threshold during sleep can use the threshold
 *   parameter parameters before sleep.
 *
 * Input Parameters:
 *   touch_thres - The sleep threshold value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_sleep_set_threshold(uint32_t touch_thres)
{
  return REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                       RTC_CNTL_TOUCH_SLP_TH,
                       touch_thres);
}

/****************************************************************************
 * Name: touch_lh_sleep_get_threshold
 *
 * Description:
 *   Get the trigger threshold of touch sensor in deep sleep.
 *   The threshold determines the sensitivity of the touch sensor.
 *   The threshold is the original value of the trigger state minus the
 *   benchmark value.
 *   In general, the touch threshold during sleep can use the threshold
 *   parameter parameters before sleep.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current sleep threshold value.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_sleep_get_threshold(void)
{
  return REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                       RTC_CNTL_TOUCH_SLP_TH);
}

/****************************************************************************
 * Name: touch_lh_sleep_enable_approach
 *
 * Description:
 *   Enable proximity function for sleep pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_sleep_enable_approach(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                RTC_CNTL_TOUCH_SLP_APPROACH_EN,
                true);
}

/****************************************************************************
 * Name: touch_lh_sleep_disable_approach
 *
 * Description:
 *   Disable proximity function for sleep pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_sleep_disable_approach(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                RTC_CNTL_TOUCH_SLP_APPROACH_EN,
                false);
}

/****************************************************************************
 * Name: touch_lh_sleep_get_approach_status
 *
 * Description:
 *   Get proximity function status for sleep pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current approach status.
 *
 ****************************************************************************/

static inline bool touch_lh_sleep_get_approach_status(void)
{
  return (bool) REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                              RTC_CNTL_TOUCH_SLP_APPROACH_EN);
}

/****************************************************************************
 * Name: touch_lh_sleep_read_benchmark
 *
 * Description:
 *   Read benchmark of touch sensor for sleep pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current sleep pad benchmark value.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_sleep_read_benchmark(void)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_DATA_SEL,
                TOUCH_LH_READ_BENCHMARK);

  return REG_GET_FIELD(SENS_SAR_TOUCH_SLP_STATUS_REG, SENS_TOUCH_SLP_DATA);
}

/****************************************************************************
 * Name: touch_lh_sleep_read_smooth
 *
 * Description:
 *   Read smooth value of touch sensor for sleep pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current sleep pad smooth value.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_sleep_read_smooth(void)
{
  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_DATA_SEL,
                TOUCH_LH_READ_SMOOTH);

  return REG_GET_FIELD(SENS_SAR_TOUCH_SLP_STATUS_REG, SENS_TOUCH_SLP_DATA);
}

/****************************************************************************
 * Name: touch_lh_sleep_read_data
 *
 * Description:
 *   Read raw value of touch sensor for sleep pad.
 *   Sleep pad raw data is not in SENS_SAR_TOUCH_SLP_STATUS_REG.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current sleep pad raw value.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_sleep_read_data(void)
{
  uint32_t tp = REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                              RTC_CNTL_TOUCH_SLP_PAD);

  REG_SET_FIELD(SENS_SAR_TOUCH_CONF_REG,
                SENS_TOUCH_DATA_SEL,
                TOUCH_LH_READ_RAW);

  /* All touch pads have the same position for the data bits.
   * We can use any SENS_TOUCH_PADN_DATA.
   */

  return REG_GET_FIELD(sens_touch_status_reg[tp - 1], SENS_TOUCH_PAD1_DATA);
}

/****************************************************************************
 * Name: touch_lh_sleep_reset_benchmark
 *
 * Description:
 *   Reset sleep benchmark.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_sleep_reset_benchmark(void)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_APPROACH_REG,
                RTC_CNTL_TOUCH_SLP_CHANNEL_CLR,
                true);
}

/****************************************************************************
 * Name: touch_lh_sleep_low_power
 *
 * Description:
 *   Select touch sensor dbias to save power in sleep mode.
 *
 * Input Parameters:
 *   is_low_power - True or false.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void touch_lh_sleep_low_power(bool is_low_power)
{
  REG_SET_FIELD(RTC_CNTL_RTC_TOUCH_CTRL2_REG,
                RTC_CNTL_TOUCH_DBIAS,
                is_low_power);
}

/****************************************************************************
 * Name: touch_lh_sleep_read_debounce
 *
 * Description:
 *   Read debounce of touch sensor for sleep pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current sleep pad debounce value.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_sleep_read_debounce(void)
{
  return REG_GET_FIELD(SENS_SAR_TOUCH_SLP_STATUS_REG,
                       SENS_TOUCH_SLP_DEBOUNCE);
}

/****************************************************************************
 * Name: touch_lh_sleep_read_proximity_cnt
 *
 * Description:
 *   Read proximity count of touch sensor for sleep pad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current sleep pad proximity count.
 *
 ****************************************************************************/

static inline uint32_t touch_lh_sleep_read_proximity_cnt(void)
{
  return REG_GET_FIELD(SENS_SAR_TOUCH_APPR_STATUS_REG,
                       SENS_TOUCH_SLP_APPROACH_CNT);
}

/****************************************************************************
 * Name: touch_lh_get_wakeup_status
 *
 * Description:
 *   Get the touch pad which caused wakeup from deep sleep.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The touch pad channel.
 *
 ****************************************************************************/

static inline enum touch_pad_e touch_lh_get_wakeup_status(void)
{
  return (enum touch_pad_e)
    REG_GET_FIELD(RTC_CNTL_RTC_TOUCH_SLP_THRES_REG,
                  RTC_CNTL_TOUCH_SLP_PAD);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_TOUCH_LOWERHALF_H */
