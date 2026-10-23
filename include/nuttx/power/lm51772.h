/****************************************************************************
 * include/nuttx/power/lm51772.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_POWER_LM51772_H
#define __INCLUDE_NUTTX_POWER_LM51772_H

/* Lower half driver for the Texas Instruments LM51772 buck-boost
 * controller (SNVSC22D), controlled over I2C, on top of the SMPS upper
 * half (include/nuttx/power/smps.h).
 *
 * The controller regulates the output voltage, so only the constant
 * voltage mode is supported.  In smps_params_s, v_out sets the output
 * voltage and i_out the output current limit (0 keeps the value of the
 * chip); p_out is ignored.  The feedback in smps_state_s is not filled.
 *
 * The driver does not touch the controller until the first
 * PWRIOC_START: it then powers it (enable callback), writes the optional
 * register list, the output voltage and the current limit, and starts
 * the power stage.  Closing the device for the last time powers it down.
 * The SMPS upper half calls setup and shutdown with interrupts disabled,
 * so they never do I2C.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/mutex.h>
#include <nuttx/power/smps.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SMPS_LM51772)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define LM51772_REG_CLEAR_FAULTS      0x03
#define LM51772_REG_ILIM_THRESHOLD    0x0a  /* 0.5 mV across RISNS per LSB */
#define LM51772_REG_VOUT_TARGET1_LSB  0x0c
#define LM51772_REG_VOUT_TARGET1_MSB  0x0d  /* Bits 3:0 */
#define LM51772_REG_STATUS_BYTE       0x78
#define LM51772_REG_USB_PD_CONTROL_0  0x81
#define LM51772_REG_MFR_SPECIFIC_D0   0xd0
#define LM51772_REG_MFR_SPECIFIC_D1   0xd1
#define LM51772_REG_MFR_SPECIFIC_D2   0xd2
#define LM51772_REG_MFR_SPECIFIC_D3   0xd3
#define LM51772_REG_MFR_SPECIFIC_D4   0xd4
#define LM51772_REG_MFR_SPECIFIC_D5   0xd5
#define LM51772_REG_MFR_SPECIFIC_D6   0xd6
#define LM51772_REG_MFR_SPECIFIC_D7   0xd7
#define LM51772_REG_MFR_SPECIFIC_D8   0xd8
#define LM51772_REG_MFR_SPECIFIC_D9   0xd9
#define LM51772_REG_IVP_VOLTAGE       0xda

/* STATUS_BYTE */

#define LM51772_STATUS_OTHER          (1 << 0)
#define LM51772_STATUS_CML            (1 << 1)
#define LM51772_STATUS_TEMPERATURE    (1 << 2)
#define LM51772_STATUS_INPUT          (1 << 3)  /* VIN_UV */
#define LM51772_STATUS_IOUT           (1 << 4)  /* IOUT_OC */
#define LM51772_STATUS_VOUT           (1 << 5)  /* VOUT_OV */
#define LM51772_STATUS_OFF            (1 << 6)
#define LM51772_STATUS_BUSY           (1 << 7)

/* USB_PD_CONTROL_0 */

#define LM51772_PD0_CONV_EN2          (1 << 0)
#define LM51772_PD0_FORCE_DISCH       (1 << 1)

/* ILIM_THRESHOLD (0x0a) for a limit of ma milliamperes across a sense
 * resistor of mohm milliohms: 0.5 mV across the resistor per LSB, rounded
 */

#define LM51772_ILIM_THRESHOLD(ma, mohm) (((ma) * (mohm) + 250) / 500)

/* MFR_SPECIFIC_D0 (0xd0, reset 0x32): converter options */

#define LM51772_D0_CONV_EN            (1 << 0)  /* Enable the power stage */
#define LM51772_D0_USLEEP_EN          (1 << 1)  /* Micro sleep, PSM only */
#define LM51772_D0_DRSS_EN            (1 << 2)  /* Dual random spread spectrum */
#define LM51772_D0_HICCUP_EN          (1 << 3)  /* Hiccup short circuit
                                                 * protection */
#define LM51772_D0_IMON_LIMITER_EN    (1 << 4)  /* IMON in limiter config */
#define LM51772_D0_EN_VCC1            (1 << 5)  /* VCC1 auxiliary LDO */
#define LM51772_D0_EN_NEG_CL_LIMIT    (1 << 6)  /* Negative current limit */

/* MFR_SPECIFIC_D1 (0xd1, reset 0x09): thermal warning and start-up */

#define LM51772_D1_EN_BB_2P_PSM       (1 << 0)  /* 2 phase BB switching, PSM */
#define LM51772_D1_EN_BB_2P_FPWM      (1 << 1)  /* 2 phase BB switching, fPWM */
#define LM51772_D1_FORCE_BIASPIN      (1 << 2)  /* Supply VCC2 from BIAS */
#define LM51772_D1_EN_DTRK_STARTOVER  (1 << 3)  /* Start without waiting for
                                                 * the PWM of the DTRK pin */
#define LM51772_D1_EN_NINT            (1 << 4)  /* nFLT pin as interrupt */
#define LM51772_D1_THW_140C           (0 << 5)  /* Thermal warning threshold */
#define LM51772_D1_THW_125C           (1 << 5)
#define LM51772_D1_THW_110C           (2 << 5)
#define LM51772_D1_THW_95C            (3 << 5)
#define LM51772_D1_EN_THER_WARN       (1 << 7)  /* Thermal warning */

/* MFR_SPECIFIC_D2 (0xd2, reset 0x40): voltage scaling and discharge */

#define LM51772_D2_DISCHARGE_CONFIG1  (1 << 0)  /* Discharge until VTH_DISCH */
#define LM51772_D2_DISCHARGE_CONFIG0  (1 << 1)  /* Discharge with CONV_EN */
#define LM51772_D2_DISCH_25MA         (0 << 2)  /* Discharge current */
#define LM51772_D2_DISCH_50MA         (1 << 2)
#define LM51772_D2_DISCH_75MA         (2 << 2)
#define LM51772_D2_DVS_40MV_US        (0 << 4)  /* Vout slew rate, DVS */
#define LM51772_D2_DVS_20MV_US        (1 << 4)
#define LM51772_D2_DVS_1MV_US         (2 << 4)
#define LM51772_D2_DVS_0P5MV_US       (3 << 4)
#define LM51772_D2_EN_ACTIVE_DVS      (1 << 6)  /* Active discharge on a DVS
                                                 * down ramp */

/* MFR_SPECIFIC_D3 (0xd3, reset 0x20): input protection and VDET */

/* VDET falling threshold in mV, 2700 to 8900 in steps of 200 */

#define LM51772_D3_VDET_FALL_MV(mv)   (((mv) - 2700) / 200)

#define LM51772_D3_VDET_EN            (1 << 5)  /* Internal input UVLO */
#define LM51772_D3_SEL_IVR            (1 << 6)  /* Regulate VIN, not protect */
#define LM51772_D3_EN_IVP             (1 << 7)  /* Input voltage protection */

/* IVP_VOLTAGE (0xda, reset 0xff): input overvoltage protection threshold.
 * With EN_IVP (D3) the protection stops the reverse current, from the output
 * to the input, when VIN reaches it.  The value is in mV, 4750 to 36625 in
 * steps of 125 (the reset value is 36.6 V).
 */

#define LM51772_IVP_VOLTAGE_MV(mv)    (((mv) - 4750) / 125)

/* MFR_SPECIFIC_D4 (0xd4, reset 0x03): VDET rising threshold */

/* VDET rising threshold in mV, 2800 to 9000 in steps of 200 */

#define LM51772_D4_VDET_RISE_MV(mv)   (((mv) - 2800) / 200)

/* MFR_SPECIFIC_D6 (0xd6, reset 0x15): dead time and minimum switching time */

#define LM51772_D6_BB_MIN_TIME_0P75   (0 << 0)  /* Scale of BB min Ton/Toff */
#define LM51772_D6_BB_MIN_TIME_1P00   (1 << 0)
#define LM51772_D6_BB_MIN_TIME_1P25   (2 << 0)
#define LM51772_D6_BB_MIN_TIME_1P50   (3 << 0)
#define LM51772_D6_MIN_DEADTIME_10NS  (0 << 2)  /* Dead time at 2 MHz */
#define LM51772_D6_MIN_DEADTIME_20NS  (1 << 2)
#define LM51772_D6_MIN_DEADTIME_40NS  (2 << 2)
#define LM51772_D6_MIN_DEADTIME_60NS  (3 << 2)
#define LM51772_D6_SEL_SCALE_DT       (1 << 4)  /* Scale dead time with fsw */
#define LM51772_D6_EN_CONST_TDEAD     (1 << 5)  /* Constant dead time */
#define LM51772_D6_SYNC_IN_RISE       (0 << 6)  /* SYNC pin function */
#define LM51772_D6_SYNC_IN_FALL       (1 << 6)
#define LM51772_D6_SYNC_OUT_RISE      (2 << 6)
#define LM51772_D6_SYNC_OUT_FALL      (3 << 6)

/* MFR_SPECIFIC_D7 (0xd7, reset 0x28): PSM derating and slope compensation */

/* Slope compensation current, ratio of the RT current: 0x8 is 1.5 */

#define LM51772_D7_SLOPE_COMP_1P5     0x08

#define LM51772_D7_DERATE_OFF         (0 << 4)  /* Inductor derating, PSM */
#define LM51772_D7_DERATE_20          (1 << 4)
#define LM51772_D7_DERATE_30          (2 << 4)
#define LM51772_D7_DERATE_40          (3 << 4)

/* MFR_SPECIFIC_D8 (0xd8, reset 0x84): feedback divider and DRV1 */

#define LM51772_D8_SEL_DRV1_SUP_OD    (0 << 0)  /* DRV1 driver configuration */
#define LM51772_D8_SEL_DRV1_SUP_VOUT  (1 << 0)
#define LM51772_D8_SEL_DRV1_SUP_VBIAS (2 << 0)
#define LM51772_D8_SEL_DRV1_SUP_VCC2  (3 << 0)
#define LM51772_D8_DRV1_SEQ_OFF       (0 << 2)  /* Pull low/CP while converter
                                                 * is off */
#define LM51772_D8_DRV1_SEQ_ON        (1 << 2)  /* Pull low/CP while converter
                                                 * is on */
#define LM51772_D8_DRV1_SEQ_ACTIVE    (2 << 2)  /* Forced active */
#define LM51772_D8_DRV1_SEQ_FORCE_OFF (3 << 2)  /* Forced off */
#define LM51772_D8_CDC_GAIN_0P25V     (0 << 4)  /* Cable drop compensation */
#define LM51772_D8_CDC_GAIN_0P5V      (1 << 4)
#define LM51772_D8_CDC_GAIN_1V        (2 << 4)
#define LM51772_D8_CDC_GAIN_2V        (3 << 4)
#define LM51772_D8_EN_CDC             (1 << 6)  /* Cable drop compensation */
#define LM51772_D8_SEL_FB_DIV20       (1 << 7)  /* 20 mV step, 3.3 to 48 V */

/* MFR_SPECIFIC_D9 (0xd9, reset 0x2c): ILIM source and PCM window */

/* Lower PCM window in percent of VOUT, 0 to 77 in steps of 2.5 (use
 * multiples of 5)
 */

#define LM51772_D9_PCM_WINDOW_PCT(p)  ((p) * 2 / 5)

/* ISET pin in the I2C config, ILIM DAC disabled */

#define LM51772_D9_SEL_ISET_PIN       (1 << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A register and the value written to it at configuration time */

struct lm51772_reg_s
{
  uint8_t reg;
  uint8_t val;
};

/* Board configuration */

struct lm51772_config_s
{
  FAR struct i2c_master_s *i2c;         /* I2C bus */
  uint32_t                 frequency;   /* I2C frequency */
  uint8_t                  addr;        /* 7-bit address, 0x6a if 0 */
  uint16_t                 r_isns_mohm; /* Output sense resistor, mohm */

  /* Absolute maximum ratings of the board, 0 for none.  The limits and
   * the parameters set by the application are checked against them and
   * refused with -EINVAL if they exceed them: the application adjusts the
   * output inside this envelope.  With p_out_max the power v_out * i_out
   * is checked too, i_out being the current limit of the parameters, or
   * the one of the ILIM_THRESHOLD entry of the init list if the parameters
   * keep it.
   */

  float v_out_min;                      /* Minimum output voltage, V */
  float v_out_max;                      /* Maximum output voltage, V */
  float i_out_max;                      /* Maximum output current, A */
  float p_out_max;                      /* Maximum output power, W */

  /* Drives the enable of the controller (EN/UVLO pin).  Called with
   * interrupts disabled when switching off, so it must not sleep.
   */

  CODE void (*enable)(FAR void *arg, bool enable);
  FAR void                *arg;

  /* Registers written, in order, after the controller is powered and
   * before the output voltage.  Optional, the chip is loaded from its
   * nonvolatile memory otherwise.  MFR_SPECIFIC_D0 is written with
   * CONV_EN cleared, the power stage only starts with PWRIOC_START.
   * Include the step of the output voltage (MFR_SPECIFIC_D8) here if it
   * is not the default one: the driver reads it back.
   */

  FAR const struct lm51772_reg_s *init;
  size_t                          ninit;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: lm51772_register
 *
 * Description:
 *   Register a LM51772 as an SMPS device.  The controller stays powered
 *   down until the first PWRIOC_START.
 *
 * Input Parameters:
 *   path   - Device path, for example "/dev/smps0".
 *   config - Board configuration.  The structure is copied; the init list
 *            is not and must stay valid.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lm51772_register(FAR const char *path,
                     FAR const struct lm51772_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SMPS_LM51772 */
#endif /* __INCLUDE_NUTTX_POWER_LM51772_H */
