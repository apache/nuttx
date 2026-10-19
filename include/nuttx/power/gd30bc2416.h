/****************************************************************************
 * include/nuttx/power/gd30bc2416.h
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

#ifndef __INCLUDE_NUTTX_POWER_GD30BC2416_H
#define __INCLUDE_NUTTX_POWER_GD30BC2416_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD30BC2416 has no register address byte: the 7-bit slave address is
 * 2'b10 followed by the 5-bit register address, and every register is
 * 16 bits wide (high byte first).  The device answers on 0x40..0x5f.
 */

#define GD30BC2416_I2C_BASEADDR      0x40
#define GD30BC2416_I2C_ADDR(reg) \
  (GD30BC2416_I2C_BASEADDR | ((reg) & 0x1f))

/* Registers */

#define GD30BC2416_REG_FAULT         0x00  /* Fault register */
#define GD30BC2416_REG_STATUS1       0x01  /* Status register 1 */
#define GD30BC2416_REG_STATUS2       0x02  /* Status register 2 */
#define GD30BC2416_REG_CTRL          0x03  /* Control register */
#define GD30BC2416_REG_CFG1          0x04  /* User configure register 1 */
#define GD30BC2416_REG_CFG2          0x05  /* User configure register 2 */
#define GD30BC2416_REG_CFG3          0x06  /* User configure register 3 */
#define GD30BC2416_REG_WDG           0x0b  /* Watchdog register */
#define GD30BC2416_REG_LED           0x0d  /* LED register */
#define GD30BC2416_REG_ADC           0x0e  /* ADC register */
#define GD30BC2416_REG_DEBUG         0x0f  /* Debug register */

/* Fault register */

#define GD30BC2416_FAULT_I2C_ACK     (1 << 15) /* I2C ACK fault (write 1) */
#define GD30BC2416_FAULT_PCH_TO      (1 << 13) /* Pre-charge timeout */
#define GD30BC2416_FAULT_CH_TO       (1 << 12) /* Charge timeout */
#define GD30BC2416_FAULT_WDG         (1 << 11) /* Watchdog timeout */
#define GD30BC2416_FAULT_VUSB_OC     (1 << 10) /* VUSB over current */
#define GD30BC2416_FAULT_VUSB_OV     (1 << 9)  /* VUSB over voltage */
#define GD30BC2416_FAULT_VSYS_OV     (1 << 6)  /* VSYS over voltage */
#define GD30BC2416_FAULT_VSYS_UV     (1 << 5)  /* VSYS under voltage */
#define GD30BC2416_FAULT_NTC_HOT     (1 << 4)  /* NTC hot */
#define GD30BC2416_FAULT_NTC_COLD    (1 << 3)  /* NTC cold */
#define GD30BC2416_FAULT_VBAT_OV     (1 << 2)  /* VBAT over voltage */
#define GD30BC2416_FAULT_VBAT_UV     (1 << 1)  /* VBAT under voltage */
#define GD30BC2416_FAULT_TEMP        (1 << 0)  /* Over temperature */

/* Status register 1 */

#define GD30BC2416_STATUS1_PCH_EN    (1 << 15) /* Pre-charge enabled */
#define GD30BC2416_STATUS1_VUSB_EN   (1 << 14) /* VUSB enabled */
#define GD30BC2416_STATUS1_VUSBOV_EN (1 << 13) /* VUSB OV detection enabled */
#define GD30BC2416_STATUS1_VBAT_PCH  (1 << 12) /* VBAT below pre-charge */
#define GD30BC2416_STATUS1_VBAT_RCH  (1 << 11) /* VBAT below re-charge */
#define GD30BC2416_STATUS1_USB_PLUG  (1 << 7)  /* USB plugged in */
#define GD30BC2416_STATUS1_CH_END    (1 << 6)  /* Charge ended */
#define GD30BC2416_STATUS1_CCCH_END  (1 << 5)  /* Constant current ended */
#define GD30BC2416_STATUS1_BST_EN    (1 << 0)  /* Boost enabled */

/* Status register 2 */

#define GD30BC2416_STATUS2_RCHCMP_EN (1 << 5)  /* Re-charge comparator on */
#define GD30BC2416_STATUS2_VBATOV_EN (1 << 4)  /* VBAT OV detection on */
#define GD30BC2416_STATUS2_PCHCMP_EN (1 << 1)  /* Pre-charge comparator on */
#define GD30BC2416_STATUS2_CH_EN     (1 << 0)  /* Charge enabled */

/* Control register */

#define GD30BC2416_CTRL_TEMPREG_DIS  (1 << 14) /* Disable temp regulation */
#define GD30BC2416_CTRL_WDG_CLR      (1 << 13) /* Feed the watchdog */
#define GD30BC2416_CTRL_ADC_EN       (1 << 11) /* ADC enable */
#define GD30BC2416_CTRL_CLR_FAULT    (1 << 10) /* Clear all faults */
#define GD30BC2416_CTRL_RST_ALL      (1 << 9)  /* Reset the whole chip */
#define GD30BC2416_CTRL_RST_OTHS     (1 << 8)  /* Reset all but the I2C */
#define GD30BC2416_CTRL_ADC_CLK_SHIFT 6        /* ADC clock selection */
#define GD30BC2416_CTRL_ADC_CLK_MASK (3 << GD30BC2416_CTRL_ADC_CLK_SHIFT)
#define GD30BC2416_CTRL_BST_EN       (1 << 4)  /* Boost enable */
#define GD30BC2416_CTRL_CH_EN        (1 << 3)  /* Charge enable */
#define GD30BC2416_CTRL_SLP_EN       (1 << 0)  /* Sleep mode enable */

/* The command bits of the control register: never written back by a
 * read-modify-write.
 */

#define GD30BC2416_CTRL_CMD_MASK     (GD30BC2416_CTRL_WDG_CLR | \
                                      GD30BC2416_CTRL_CLR_FAULT | \
                                      GD30BC2416_CTRL_RST_ALL | \
                                      GD30BC2416_CTRL_RST_OTHS)

/* User configure register 2 */

#define GD30BC2416_CFG2_ICCCH_SHIFT  2         /* Constant current charge */
#define GD30BC2416_CFG2_ICCCH_MASK   (3 << GD30BC2416_CFG2_ICCCH_SHIFT)
#  define GD30BC2416_CFG2_ICCCH_100  (0 << GD30BC2416_CFG2_ICCCH_SHIFT)
#  define GD30BC2416_CFG2_ICCCH_75   (1 << GD30BC2416_CFG2_ICCCH_SHIFT)
#  define GD30BC2416_CFG2_ICCCH_50   (2 << GD30BC2416_CFG2_ICCCH_SHIFT)
#  define GD30BC2416_CFG2_ICCCH_25   (3 << GD30BC2416_CFG2_ICCCH_SHIFT)

/* User configure register 3 */

#define GD30BC2416_CFG3_VCCCHT_SHIFT 2         /* Charge terminate voltage */
#define GD30BC2416_CFG3_VCCCHT_MASK  (7 << GD30BC2416_CFG3_VCCCHT_SHIFT)
#  define GD30BC2416_CFG3_VCCCHT_4200 (0 << GD30BC2416_CFG3_VCCCHT_SHIFT)
#  define GD30BC2416_CFG3_VCCCHT_4100 (1 << GD30BC2416_CFG3_VCCCHT_SHIFT)
#  define GD30BC2416_CFG3_VCCCHT_4300 (2 << GD30BC2416_CFG3_VCCCHT_SHIFT)
#  define GD30BC2416_CFG3_VCCCHT_4350 (3 << GD30BC2416_CFG3_VCCCHT_SHIFT)
#  define GD30BC2416_CFG3_VCCCHT_4400 (4 << GD30BC2416_CFG3_VCCCHT_SHIFT)

/* Watchdog register */

#define GD30BC2416_WDG_EN            (1 << 0)  /* Watchdog enable */

/* ADC register */

#define GD30BC2416_ADC_DATA_SHIFT    4         /* 12-bit conversion */
#define GD30BC2416_ADC_DATA_MASK     (0xfff << GD30BC2416_ADC_DATA_SHIFT)
#define GD30BC2416_ADC_CHSEL_SHIFT   1         /* Channel selection */
#define GD30BC2416_ADC_CHSEL_MASK    (7 << GD30BC2416_ADC_CHSEL_SHIFT)
#  define GD30BC2416_ADC_CH_VBAT     (0 << GD30BC2416_ADC_CHSEL_SHIFT)
#  define GD30BC2416_ADC_CH_VRCH     (1 << GD30BC2416_ADC_CHSEL_SHIFT)
#  define GD30BC2416_ADC_CH_VIC      (5 << GD30BC2416_ADC_CHSEL_SHIFT)
#  define GD30BC2416_ADC_CH_VIR      (6 << GD30BC2416_ADC_CHSEL_SHIFT)
#  define GD30BC2416_ADC_CH_AVSS     (7 << GD30BC2416_ADC_CHSEL_SHIFT)

/* The ADC measures VBAT / 2 against a 2.5 V reference */

#define GD30BC2416_ADC_VBAT_MV(d)    ((d) * 5000 / 4096)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board configuration of a GD30BC2416 */

struct gd30bc2416_config_s
{
  uint32_t frequency;   /* I2C frequency (up to 400 kHz) */
  int      current;     /* Charge current set by the ISET resistor, mA */

  /* Attach (isr != NULL) or detach the handler of the IRQ output, which
   * pulses for 8 ms on each event of the charger (USB plugged or
   * unplugged, charge end, fault).  NULL if the IRQ is not wired.
   */

  CODE int (*irq_attach)(FAR const struct gd30bc2416_config_s *config,
                         xcpt_t isr, FAR void *arg);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_GD30BC2416)

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: gd30bc2416_initialize
 *
 * Description:
 *   Initialize the GD30BC2416 battery charger driver and return an
 *   instance of the lower-half interface that may be used with
 *   battery_charger_register().
 *
 *   The charger returns to its reset configuration when the USB power is
 *   removed: the driver keeps the charge voltage, the charge current and
 *   the charge/boost enables that were requested, and writes them again
 *   when they are lost (on the IRQ, and before each status read).
 *
 * Input Parameters:
 *   i2c    - An instance of the I2C interface to use to communicate with
 *            the GD30BC2416
 *   config - The board configuration.  It must stay valid while the
 *            driver is in use.
 *
 * Returned Value:
 *   A pointer to the initialized battery driver instance.  A NULL pointer
 *   is returned on a failure to initialize the GD30BC2416 lower half.
 *
 ****************************************************************************/

struct i2c_master_s;
FAR struct battery_charger_dev_s *
gd30bc2416_initialize(FAR struct i2c_master_s *i2c,
                      FAR const struct gd30bc2416_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_GD30BC2416 */
#endif /* __INCLUDE_NUTTX_POWER_GD30BC2416_H */
