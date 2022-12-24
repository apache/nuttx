/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_pmic.c
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

/* Reference:
 *
 * "Rendering PinePhone's Display (DE and TCON0)"
 * https://lupyuen.github.io/articles/de
 *
 * "NuttX RTOS for PinePhone: Render Graphics in Zig"
 * https://lupyuen.github.io/articles/de2
 *
 * "AXP803 Page" refers to X-Powers AXP803 Data Sheet
 * https://lupyuen.github.io/images/AXP803_Datasheet_V1.0.pdf
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include "chip.h"
#include "arm64_internal.h"
#include "a64_rsb.h"
#include "pinephone_pmic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Address of AXP803 PMIC on Reduced Serial Bus */

#define AXP803_RT_ADDR                      0x2d

/* AXP803 PMIC Registers and Bit Definitions ********************************/

/* DLDO1 Voltage Control (AXP803 Page 52) */

#define DLDO1_VOLTAGE_CONTROL               0x15
#define DLDO1_VOLTAGE(n)                    ((n) << 0)

/* Output Power On-Off Control 2 (AXP803 Page 51) */

#define OUTPUT_POWER_ON_OFF_CONTROL2        0x12
#define DLDO1_ON_OFF_CONTROL                (1 << 3)
#define DLDO2_ON_OFF_CONTROL                (1 << 4)

/* GPIO0LDO and GPIO0 High Level Voltage Setting (AXP803 Page 77) */

#define GPIO0LDO_HIGH_LEVEL_VOLTAGE_SETTING 0x91
#define GPIO0LDO_HIGH_LEVEL_VOLTAGE(n)      ((n) << 0)

/* GPIO0 (GPADC) Control (AXP803 Page 76) */

#define GPIO0_CONTROL                       0x90
#define GPIO0_PIN_FUNCTION(n)               ((n) << 0)

/* DLDO2 Voltage Control (AXP803 Page 52) */

#define DLDO2_VOLTAGE_CONTROL               0x16
#define DLDO2_VOLTAGE(n)                    ((n) << 0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmic_write
 *
 * Description:
 *   Write a byte to an AXP803 PMIC Register.
 *
 * Input Parameters:
 *   reg - AXP803 Register ID
 *   val - Byte to be written
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pmic_write(uint8_t reg, uint8_t val)
{
  int ret;

  /* Write to AXP803 PMIC on Reduced Serial Bus */

  batinfo("reg=0x%x, val=0x%x\n", reg, val);
  ret = a64_rsb_write(AXP803_RT_ADDR, reg, val);
  if (ret < 0)
    {
      baterr("PMIC Write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: pmic_clrsetbits
 *
 * Description:
 *   Clear and set the bits in an AXP803 PMIC Register.
 *
 * Input Parameters:
 *   reg      - AXP803 Register ID
 *   clr_mask - Bits to be masked
 *   set_mask - Bits to be set
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pmic_clrsetbits(uint8_t reg, uint8_t clr_mask, uint8_t set_mask)
{
  int ret;
  uint8_t regval;

  /* Read from AXP803 PMIC on Reduced Serial Bus */

  batinfo("reg=0x%x, clr_mask=0x%x, set_mask=0x%x\n",
          reg, clr_mask, set_mask);
  ret = a64_rsb_read(AXP803_RT_ADDR, reg);
  if (ret < 0)
    {
      baterr("PMIC Read failed: %d\n", ret);
      return ret;
    }

  /* Write to AXP803 PMIC on Reduced Serial Bus */

  regval = (ret & ~clr_mask) | set_mask;
  ret = a64_rsb_write(AXP803_RT_ADDR, reg, regval);
  if (ret < 0)
    {
      baterr("PMIC Write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_pmic_init
 *
 * Description:
 *   Initialize the X-Powers AXP803 Power Management Integrated Circuit,
 *   connected on Reduced Serial Bus.  Power on the MIPI DSI Interface of
 *   Xingbangda XBD599 LCD Panel.  Doesn't switch on the LCD Panel
 *   Backlight, which is controlled by PIO and PWM.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_pmic_init(void)
{
  int ret;

  /* Set DLDO1 Voltage to 3.3V. DLDO1 powers the Front Camera / USB HSIC /
   * I2C Sensors.
   */

  /* DLDO1 Voltage Control (AXP803 Page 52)
   * Set Voltage (Bits 0 to 4) to 26 (2.6V + 0.7V = 3.3V)
   */

  batinfo("Set DLDO1 Voltage to 3.3V\n");
  ret = pmic_write(DLDO1_VOLTAGE_CONTROL, DLDO1_VOLTAGE(26));
  if (ret < 0)
    {
      baterr("Set DLDO1 failed: %d\n", ret);
      return ret;
    }

  /* Power on DLDO1 */

  /* Output Power On-Off Control 2 (AXP803 Page 51)
   * Set DLDO1 On-Off Control (Bit 3) to 1 (Power On)
   */

  ret = pmic_clrsetbits(OUTPUT_POWER_ON_OFF_CONTROL2, 0,
                        DLDO1_ON_OFF_CONTROL);
  if (ret < 0)
    {
      baterr("Power on DLDO1 failed: %d\n", ret);
      return ret;
    }

  /* Set LDO Voltage to 3.3V. GPIO0LDO powers the Capacitive Touch Panel. */

  /* GPIO0LDO and GPIO0 High Level Voltage Setting (AXP803 Page 77)
   * Set GPIO0LDO and GPIO0 High Level Voltage (Bits 0 to 4) to 26
   * (2.6V + 0.7V = 3.3V)
   */

  batinfo("Set LDO Voltage to 3.3V\n");
  ret = pmic_write(GPIO0LDO_HIGH_LEVEL_VOLTAGE_SETTING,
                   GPIO0LDO_HIGH_LEVEL_VOLTAGE(26));
  if (ret < 0)
    {
      baterr("Set LDO failed: %d\n", ret);
      return ret;
    }

  /* Enable LDO Mode on GPIO0 */

  /* GPIO0 (GPADC) Control (AXP803 Page 76)
   * Set GPIO0 Pin Function Control (Bits 0 to 2) to 0b11 (Low Noise LDO on)
   */

  batinfo("Enable LDO mode on GPIO0\n");
  ret = pmic_write(GPIO0_CONTROL, GPIO0_PIN_FUNCTION(0b11));
  if (ret < 0)
    {
      baterr("Enable LDO failed: %d\n", ret);
      return ret;
    }

  /* Set DLDO2 Voltage to 1.8V. DLDO2 powers the MIPI DSI Interface of
   * Xingbangda XBD599 LCD Panel.
   */

  /* DLDO2 Voltage Control (AXP803 Page 52)
   * Set Voltage (Bits 0 to 4) to 11 (1.1V + 0.7V = 1.8V)
   */

  batinfo("Set DLDO2 Voltage to 1.8V\n");
  ret = pmic_write(DLDO2_VOLTAGE_CONTROL, DLDO2_VOLTAGE(11));
  if (ret < 0)
    {
      baterr("Set DLDO2 failed: %d\n", ret);
      return ret;
    }

  /* Power on DLDO2 */

  /* Output Power On-Off Control 2 (AXP803 Page 51)
   * Set DLDO2 On-Off Control (Bit 4) to 1 (Power On)
   */

  ret = pmic_clrsetbits(OUTPUT_POWER_ON_OFF_CONTROL2, 0,
                        DLDO2_ON_OFF_CONTROL);
  if (ret < 0)
    {
      baterr("Power on DLDO2 failed: %d\n", ret);
      return ret;
    }

  return OK;
}
