/****************************************************************************
 * drivers/sensors/bmp180_base.h
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

/* Character driver for the Freescale BMP1801 Barometer Sensor */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmp180.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMP180_ADDR         0x77
#define BMP180_FREQ         100000
#define DEVID               0x55

#define BMP180_AC1_MSB      0xaa
#define BMP180_AC1_LSB      0xab
#define BMP180_AC2_MSB      0xac
#define BMP180_AC2_LSB      0xad
#define BMP180_AC3_MSB      0xae
#define BMP180_AC3_LSB      0xaf
#define BMP180_AC4_MSB      0xb0
#define BMP180_AC4_LSB      0xb1
#define BMP180_AC5_MSB      0xb2
#define BMP180_AC5_LSB      0xb3
#define BMP180_AC6_MSB      0xb4
#define BMP180_AC6_LSB      0xb5
#define BMP180_B1_MSB       0xb6
#define BMP180_B1_LSB       0xb7
#define BMP180_B2_MSB       0xb8
#define BMP180_B2_LSB       0xb9
#define BMP180_MB_MSB       0xba
#define BMP180_MB_LSB       0xbb
#define BMP180_MC_MSB       0xbc
#define BMP180_MC_LSB       0xbd
#define BMP180_MD_MSB       0xbe
#define BMP180_MD_LSB       0xbf

#define BMP180_DEVID        0xd0
#define BMP180_SOFT_RESET   0xe0
#define BMP180_CTRL_MEAS    0xf4
#define BMP180_ADC_OUT_MSB  0xf6
#define BMP180_ADC_OUT_LSB  0xf7
#define BMP180_ADC_OUT_XLSB 0xf8

#define BMP180_READ_TEMP    0x2e
#define BMP180_READ_PRESS   0x34

#define BMP180_NOOVERSAMPLE 0x00
#define BMP180_OVERSAMPLE2X 0x70
#define BMP180_OVERSAMPLE4X 0xb0
#define BMP180_OVERSAMPLE8X 0xc0

/* Current Oversampling */

#define CURRENT_OSS         (BMP180_OVERSAMPLE8X)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct bmp180_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* BMP180 I2C address */
  int freq;                     /* BMP180 Frequency <= 3.4MHz */
  int16_t bmp180_cal_ac1;       /* Calibration coefficients */
  int16_t bmp180_cal_ac2;
  int16_t bmp180_cal_ac3;
  uint16_t bmp180_cal_ac4;
  uint16_t bmp180_cal_ac5;
  uint16_t bmp180_cal_ac6;
  int16_t bmp180_cal_b1;
  int16_t bmp180_cal_b2;
  int16_t bmp180_cal_mb;
  int16_t bmp180_cal_mc;
  int16_t bmp180_cal_md;
  int32_t bmp180_utemp;         /* Uncompensated temperature read from BMP180 */
  int32_t bmp180_upress;        /* Uncompensated pressure read from BMP180 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint8_t bmp180_getreg8(FAR struct bmp180_dev_s *priv, uint8_t regaddr);

uint16_t bmp180_getreg16(FAR struct bmp180_dev_s *priv, uint8_t regaddr);
void bmp180_putreg8(FAR struct bmp180_dev_s *priv, uint8_t regaddr,
                    uint8_t regval);

int bmp180_checkid(FAR struct bmp180_dev_s *priv);

void bmp180_updatecaldata(FAR struct bmp180_dev_s *priv);
void bmp180_read_press_temp(FAR struct bmp180_dev_s *priv);
int bmp180_getpressure(FAR struct bmp180_dev_s *priv,
                       FAR float *temperature);

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */
