/****************************************************************************
 * include/nuttx/sensors/msa301.h
 * msa301 Driver declaration
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

#ifndef __INCLUDE_NUTTX_SENSORS_MSA301_H
#define __INCLUDE_NUTTX_SENSORS_MSA301_H

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MSA301)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MSA301_ACCEL_ADDR0	0x26

#define SNIOC_MSA301_START                _SNIOC(0x0001)
#define SNIOC_MSA301_STOP                 _SNIOC(0x0002)
#define SNIOC_MSA301_SET_RANGE            _SNIOC(0x0003)
#define SNIOC_MSA301_SET_RATE             _SNIOC(0x0004)

#define MSA301_REG_PARTID 0x01
#define MSA301_REG_OUT_X_L 0x02
#define MSA301_REG_OUT_X_H 0x03
#define MSA301_REG_OUT_Y_L 0x04
#define MSA301_REG_OUT_Y_H 0x05
#define MSA301_REG_OUT_Z_L 0x06
#define MSA301_REG_OUT_Z_H 0x07
#define MSA301_REG_MOTIONINT 0x09
#define MSA301_REG_DATAINT 0x0A
#define MSA301_REG_CLICKSTATUS 0x0B
#define MSA301_REG_RESRANGE 0x0F
#define MSA301_REG_ODR 0x10
#define MSA301_REG_POWERMODE 0x11
#define MSA301_REG_INTSET0 0x16
#define MSA301_REG_INTSET1 0x17
#define MSA301_REG_INTMAP0 0x19
#define MSA301_REG_INTMAP1 0x1A
#define MSA301_REG_TAPDUR 0x2A
#define MSA301_REG_TAPTH 0x2B

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The accelerometer ranges */

typedef enum
{
  MSA301_RANGE_2_G  = 0x0,  /* +/- 2g */
  MSA301_RANGE_4_G  = 0x1,  /* +/- 4g */
  MSA301_RANGE_8_G  = 0x2,  /* +/- 8g */
  MSA301_RANGE_16_G = 0x3,  /* +/- 16g */
} msa301_range_t;

#define MSA301_CTL_RANGE_SHIFT  (0x0)
#define MSA301_CTL_RANGE_MASK   (0x3<<0)

/* The accelerometer data rate */

typedef enum
{
  MSA301_RATE_1_HZ = 0,     /*  1 Hz */
  MSA301_RATE_1_95_HZ = 1,  /*  1.95 Hz */
  MSA301_RATE_3_9_HZ = 2,   /*  3.9 Hz */
  MSA301_RATE_7_81_HZ = 3,  /*  7.81 Hz */
  MSA301_RATE_15_63_HZ = 4, /*  15.63 Hz */
  MSA301_RATE_31_25_HZ = 5, /*  31.25 Hz */
  MSA301_RATE_62_5_HZ = 6,  /*  62.5 Hz */
  MSA301_RATE_125_HZ = 7,   /*  125 Hz */
  MSA301_RATE_250_HZ = 8,   /*  250 Hz */
  MSA301_RATE_500_HZ = 9,   /*  500 Hz */
  MSA301_RATE_1000_HZ = 10, /*  1000 Hz */
} msa301_rate_t;

#define MSA301_CTL_RATE_SHIFT (0x0)
#define MSA301_CTL_RATE_MASK  (0xF<<0)

#define MSA301_ENABLE_AXIS    (0x0)
#define MSA301_DISABLE_AXIS   (0x7)
#define MSA301_CTL_AXIS_SHIFT (0x05)
#define MSA301_CTL_AXIS_MASK  (0x7<<5)

/* The accelerometer power mode */

typedef enum
{
  MSA301_NORMALMODE = 0x00,   /* Normal (high speed) mode */
  MSA301_LOWPOWERMODE = 0x01, /* Low power (slow speed) mode */
  MSA301_SUSPENDMODE = 0x10,  /* Suspend (sleep) mode */
} msa301_powermode_t;

#define MSA301_CTL_POWERMODE_SHIFT  (0x6)
#define MSA301_CTL_POWERMODE_MASK   (0x3<<6)

/* The accelerometer read resolution */

typedef enum
{
  MSA301_RESOLUTION_14 = 0, /* 14-bit resolution */
  MSA301_RESOLUTION_12 = 1, /* 12-bit resolution */
  MSA301_RESOLUTION_10 = 2, /* 10-bit resolution */
  MSA301_RESOLUTION_8 = 3,  /* 8-bit resolution */
} msa301_resolution_t;

#define MSA301_CTL_RESOLUTION_SHIFT  (0x2)
#define MSA301_CTL_RESOLUTION_MASK   (0x3<<2)

struct msa301_sensor_data_s
{
  int16_t  x_data;
  int16_t  y_data;
  int16_t  z_data;

  float x_acc;                       /* X axis acceleration */
  float y_acc;                       /* Y axis acceleration */
  float z_acc;                       /* Z axis acceleration */
};

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int msa301_sensor_register(FAR const char *devpath,
                           FAR struct i2c_master_s *i2c
                          );

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MSA301 */
#endif /* __INCLUDE_NUTTX_SENSORS_MSA301_H */
