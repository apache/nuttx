/****************************************************************************
 * drivers/sensors/bh1749nuc_base.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BH1749NUC_BASE_H
#define __INCLUDE_NUTTX_SENSORS_BH1749NUC_BASE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/bh1749nuc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BH1749NUC_I2C_FREQ          400000

#define BH1749NUC_MANUFACTID        0xE0    /* Manufact ID */
#define BH1749NUC_PARTID            0x0D    /* Part ID */

/* BH1749NUC Registers */

#define BH1749NUC_SYSTEM_CONTROL    0x40
#define BH1749NUC_MODE_CONTROL1     0x41
#define BH1749NUC_MODE_CONTROL2     0x42
#define BH1749NUC_RED_DATA_LSB      0x50
#define BH1749NUC_GREEN_DATA_LSB    0x52
#define BH1749NUC_BLUE_DATA_LSB     0x54
#define BH1749NUC_IR_DATA_LSB       0x58
#define BH1749NUC_GREEN2_DATA_LSB   0x5a
#define BH1749NUC_MANUFACTURER_ID   0x92

/* Register SYSTEM_CONTROL */

#define BH1749NUC_SYSTEM_CONTROL_SW_RESET      (1 << 7)
#define BH1749NUC_SYSTEM_CONTROL_INT_RESET     (1 << 6)

/* Register MODE_CONTROL1 */

#define BH1749NUC_MODE_CONTROL1_IR_GAIN_X1     (0x20)
#define BH1749NUC_MODE_CONTROL1_IR_GAIN_X32    (0x60)
#define BH1749NUC_MODE_CONTROL1_RGB_GAIN_X1    (0x08)
#define BH1749NUC_MODE_CONTROL1_RGB_GAIN_X32   (0x18)
#define BH1749NUC_MODE_CONTROL1_MEAS_TIME160MS (0x02)

/* Register MODE_CONTROL2 */

#define BH1749NUC_MODE_CONTROL2_RGBI_EN        (1 << 4)
#define BH1749NUC_MODE_CONTROL2_VALID          (1 << 7)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure for bh1749nuc device */

struct bh1749nuc_dev_s
{
  FAR struct i2c_master_s *i2c;   /* I2C interface */
  int                      freq;  /* Frequency */
  uint8_t                  addr;  /* I2C address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint8_t bh1749nuc_getreg8(FAR struct bh1749nuc_dev_s *priv, uint8_t regaddr);
uint16_t bh1749nuc_read16(FAR struct bh1749nuc_dev_s *priv, uint8_t regaddr);
void bh1749nuc_putreg8(FAR struct bh1749nuc_dev_s *priv,
                       uint8_t regaddr, uint8_t regval);
int bh1749nuc_checkid(FAR struct bh1749nuc_dev_s *priv);

#endif /* __INCLUDE_NUTTX_SENSORS_BH1749NUC_BASE_H */
