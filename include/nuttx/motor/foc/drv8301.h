/****************************************************************************
 * include/nuttx/motor/foc/drv8301.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_FOC_DRV8301_H
#define __INCLUDE_NUTTX_MOTOR_FOC_DRV8301_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/motor/foc/foc_pwr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define DRV8301_REG_STAT1               (0x00)    /* Status 1 register */
#define DRV8301_REG_STAT2               (0x01)    /* Status 2 register */
#define DRV8301_REG_CTRL1               (0x02)    /* Control 1 register */
#define DRV8301_REG_CTRL2               (0x03)    /* Control 2 register */

/* Status 1 register */

#define DRV8301_STAT1_FETLCOC           (1 << 0)  /* FET-high C overcurrent */
#define DRV8301_STAT1_FETHCOC           (1 << 1)  /* FET-low C overcurrent */
#define DRV8301_STAT1_FETLBOC           (1 << 2)  /* FET-high B overcurrent */
#define DRV8301_STAT1_FETHBOC           (1 << 3)  /* FET-low B overcurrent */
#define DRV8301_STAT1_FETLAOC           (1 << 4)  /* FET-high A overcurrent */
#define DRV8301_STAT1_FETHAOC           (1 << 5)  /* FET-low A overcurrent */
#define DRV8301_STAT1_OTW               (1 << 6)  /* Overtemperature warning */
#define DRV8301_STAT1_OTSD              (1 << 7)  /* Overtemperature latched shut down */
#define DRV8301_STAT1_PVDDUV            (1 << 8)  /* PVDD undervoltage protection */
#define DRV8301_STAT1_GVDDUV            (1 << 9)  /* GVDD undervoltage protection */
#define DRV8301_STAT1_FAULT             (1 << 10) /* Fault */

/* Status 2 register */

#define DRV8301_STAT2_DEVID_SHIFT       (0)      /* Device ID */
#define DRV8301_STAT2_DEVID_MASK        (0x0f << DRV8301_STAT2_DEVID_SHIFT)
#define DRV8301_STAT2_GVDD_OV           (1 << 7) /* GVDD overvoltage protection */

/* Control 1 register */

#define DRV8301_CTRL1_GCURR_SHIFT       (0)      /* Gate drive peak current */
#define DRV8301_CTRL1_GCURR_MASK        (0x2 << DRV8301_CTRL1_GCURR_SHIFT)
#  define DRV8301_CTRL1_GCURR_1p7A      (0x0 << DRV8301_CTRL1_GCURR_SHIFT)
#  define DRV8301_CTRL1_GCURR_0p7A      (0x1 << DRV8301_CTRL1_GCURR_SHIFT)
#  define DRV8301_CTRL1_GCURR_0p25A     (0x2 << DRV8301_CTRL1_GCURR_SHIFT)
#define DRV8301_CTRL1_GCURR(x)          ((x) << DRV8301_CTRL1_GCURR_SHIFT & DRV8301_CTRL1_GCURR_MASK)
#define DRV8301_CTRL1_GRESET            (1 << 2) /* Gate reset */
#define DRV8301_CTRL1_PWMMODE           (1 << 3) /* PWM input mode */
#define DRV8301_CTRL1_OCPMODE_SHIFT     (4)      /* Overcurrent protection mode */
#define DRV8301_CTRL1_OCPMODE_MASK      (0x2 << DRV8301_CTRL1_OCPMODE_SHIFT)
#  define DRV8301_CTRL1_OCPMODE_CLIMIT  (0x0 << DRV8301_CTRL1_OCPMODE_SHIFT)
#  define DRV8301_CTRL1_OCPMODE_OCLATCH (0x1 << DRV8301_CTRL1_OCPMODE_SHIFT)
#  define DRV8301_CTRL1_OCPMODE_REPORT  (0x2 << DRV8301_CTRL1_OCPMODE_SHIFT)
#  define DRV8301_CTRL1_OCPMODE_DIS     (0x3 << DRV8301_CTRL1_OCPMODE_SHIFT)
#define DRV8301_CTRL1_OCADJ_SHIFT       (6)      /* Overcurrent adjustment */
#define DRV8301_CTRL1_OCADJ_MASK        (0x1f << DRV8301_CTRL1_OCADJ_SHIFT)
#define DRV8301_CTRL1_OCADJ(x)          ((x) << DRV8301_CTRL1_OCADJ_SHIFT & DRV8301_CTRL1_OCADJ_MASK)
#define DRV8301_OCADJ_DEFAULT           (16)

/* Control 2 register */

#define DRV8301_CTRL2_OCTWMODE_SHIFT    (0)      /* nOCTW pin configuration */
#define DRV8301_CTRL2_OCTWMODE_MASK     (0x2 << DRV8301_CTRL2_OCTWMODE_SHIFT)
#  define DRV8301_CTRL2_OCTWMODE_OTOC   (0x0 << DRV8301_CTRL2_OCTWMODE_SHIFT)
#  define DRV8301_CTRL2_OCTWMODE_OTONLY (0x1 << DRV8301_CTRL2_OCTWMODE_SHIFT)
#  define DRV8301_CTRL2_OCTWMODE_OCONLY (0x2 << DRV8301_CTRL2_OCTWMODE_SHIFT)
#define DRV8301_CTRL2_GAIN_SHIFT        (2)      /* Gain of shunt amplifier */
#define DRV8301_CTRL2_GAIN_MASK         (0x3 << DRV8301_CTRL2_GAIN_SHIFT)
#define DRV8301_CTRL2_GAIN(x)           ((x) << DRV8301_CTRL2_GAIN_SHIFT & DRV8301_CTRL2_GAIN_MASK)
#  define DRV8301_CTRL2_GAIN_10         (0x0 << DRV8301_CTRL2_GAIN_SHIFT)
#  define DRV8301_CTRL2_GAIN_20         (0x1 << DRV8301_CTRL2_GAIN_SHIFT)
#  define DRV8301_CTRL2_GAIN_40         (0x2 << DRV8301_CTRL2_GAIN_SHIFT)
#  define DRV8301_CTRL2_GAIN_80         (0x3 << DRV8301_CTRL2_GAIN_SHIFT)
#define DRV8301_CTRL2_DCCALCH1          (1 << 4) /* Shunt amplifier 1 external calibration */
#define DRV8301_CTRL2_DCCALCH2          (1 << 5) /* Shunt amplifier 2 external calibration */
#define DRV8301_CTRL2_OCTOFF            (1 << 6) /* OC_TOFF */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Gate current */

enum drv8301_gatecurr_e
{
  DRV8301_GATECURR_1p7  = 0,
  DRV8301_GATECURR_0p7  = 1,
  DRV8301_GATECURR_0p25 = 2
};

/* Gain of shunt amplifier */

enum drv8301_gain_e
{
  DRV8301_GAIN_10 = 0,
  DRV8301_GAIN_20 = 1,
  DRV8301_GAIN_40 = 2,
  DRV8301_GAIN_80 = 3
};

/* PWM mode */

enum drv8301_pwmmode_e
{
  DRV8301_PWM_6IN = 0,
  DRV8301_PWM_3IN = 1,
};

/* DRV8301 board ops */

struct drv8301_ops_s
{
  CODE int (*fault_attach)(FAR struct focpwr_dev_s *dev, xcpt_t isr,
                           FAR void *arg);
  CODE int (*gate_enable)(FAR struct focpwr_dev_s *dev, bool enable);
  CODE int (*configure)(FAR struct focpwr_dev_s *dev);
  CODE void (*fault_handle)(FAR struct focpwr_dev_s *dev);
};

/* DRV8301 configuration */

struct drv8301_cfg_s
{
  /* SPI frequency */

  uint32_t freq;

  /* Control registers settings */

  uint8_t gain:2;                /* Gain of shunt amplifier */
  uint8_t gate_curr:2;           /* Gate current */
  uint8_t pwm_mode:1;            /* PWM 3 input mode if set to 1 */
  uint8_t oc_adj:5;              /* Overcurrent adjustment */
};

/* DRV8301 board data */

struct drv8301_board_s
{
  FAR struct spi_dev_s     *spi;
  FAR struct drv8301_ops_s *ops;
  FAR struct drv8301_cfg_s *cfg;
  int                       devno;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int drv8301_register(FAR const char *path,
                     FAR struct foc_dev_s *dev,
                     FAR struct drv8301_board_s *board);

#endif /* __INCLUDE_NUTTX_MOTOR_FOC_DRV8301_H */
