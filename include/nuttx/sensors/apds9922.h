/****************************************************************************
 * include/nuttx/sensors/apds9922.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_APDS9922_H
#define __INCLUDE_NUTTX_SENSORS_APDS9922_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SENSORS_APDS9922)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define APDS9922_I2C_ADDR   (0x53)
#define APDS9922_ID_VAL     (0xb3)

#define PS_DEF_THRESHU      (0x07ff)
#define PS_DEF_THRESHL      (0)
#define PS_DEF_CANCEL_LVL   (0)
#define PS_DEF_PULSES       (0x08)
#define ALS_DEF_THRESHU     (0x0fffff)
#define ALS_DEF_THRESHL     (0)
#define PS_DEF_PERSISTANCE  (0)
#define ALS_DEF_PERSISTANCE (0)
#define ALS_DEF_VAR         (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum
{
  ALS_RATE25MS,
  ALS_RATE50MS,
  ALS_RATE100MS,
  ALS_RATE200MS,
  ALS_RATE500MS,
  ALS_RATE1000MS,
  ALS_RATE2000MS,
  ALS_RATE4000MS,
};

enum
{
  ALS_RES400MS,
  ALS_RES200MS,
  ALS_RES100MS,
  ALS_RES50MS,
  ALS_RES25MS,
};

enum
{
  ALS_GAINX1,
  ALS_GAINX3,
  ALS_GAINX6,
  ALS_GAINX9,
  ALS_GAINX18,
};

enum
{
  ALS_VAR8,
  ALS_VAR16,
  ALS_VAR32,
  ALS_VAR64,
  ALS_VAR128,
  ALS_VAR256,
  ALS_VAR512,
  ALS_VAR1024,
};

enum
{
  ALS_IR,
  ALS_VISIBLE,
};

enum
{
  PS_RATE_RESERVED,
  PS_RATE6MS25,
  PS_RATE12MS5,
  PS_RATE25MS,
  PS_RATE50MS,
  PS_RATE100MS,
  PS_RATE200MS,
  PS_RATE400MS,
};

enum
{
  PS_RES8,
  PS_RES9,
  PS_RES10,
  PS_RES11,
};

enum
{
  PS_LED_FREQ60K = 3,
  PS_LED_FREQ70K,
  PS_LED_FREQ80K,
  PS_LED_FREQ90K,
  PS_LED_FREQ100K,
};

enum
{
  PS_LED_CURRENT2MA5,
  PS_LED_CURRENT5MA,
  PS_LED_CURRENT10MA,
  PS_LED_CURRENT25MA,
  PS_LED_CURRENT50MA,
  PS_LED_CURRENT75MA,
  PS_LED_CURRENT100MA,
  PS_LED_CURRENT125MA,
};

enum
{
  ALS_INT_MODE_DISABLED,
  ALS_INT_MODE_THRESHOLD,
  ALS_INT_MODE_VARIANCE,
};

enum
{
  PS_INT_MODE_DISABLED,
  PS_INT_MODE_LOGIC,
  PS_INT_MODE_NORMAL,
};

enum
{
  PS_ALL_INFO,
  PS_PROXIMITY_DATA_ONLY,
  PS_FAR_OR_CLOSE_ONLY,
};

/* Interrupt configuration data structure */

struct apds9922_config_s
{
  int (*irq_attach)(FAR struct apds9922_config_s *state, xcpt_t isr,
                    FAR void *arg);
  void (*irq_enable)(FAR struct apds9922_config_s *state, bool enable);
  FAR struct i2c_master_s *i2c;
  uint8_t i2c_addr;
};

struct adps9922_als_thresh
{
  uint32_t upper;       /* Upper threshold */
  uint32_t lower;       /* Lower threshold */
};

struct adps9922_ps_thresh
{
  uint16_t upper;       /* Upper threshold */
  uint16_t lower;       /* Lower threshold */
};

/* ambient light data setup data */

struct apds9922_als_setup_s
{
  int      rate;        /* als measurement rate              */
  int      res;         /* als resolution                    */
  int      gain;        /* als gain                          */
  struct   adps9922_als_thresh
           thresh;      /* Upper and lower thresholds        */
  int      thresh_var;  /* threshold variation               */
  int      int_mode;    /* Interrupt mode                    */
  uint8_t  persistance; /* Num events before interrupt       */
  uint32_t als_factor;  /* Lux correction factor applied     */
  uint32_t range_lim;   /* % limit of ADC full range
                         * allowed in autogain mode.
                         */
  bool     autogain;    /* Auto gain mode on/off             */
  int      channel;     /* Visible or IR light channel       */
};

/* proximity sensor data setup data */

struct apds9922_ps_setup_s
{
  int      rate;        /* Measurement rate                  */
  int      res;         /* Resolution, bits                  */
  int      led_f;       /* LED modulation frequency          */
  bool     led_pk_on;   /* LED current peaking on/off        */
  int      led_i;       /* LED pulsed current level          */
  uint8_t  pulses;      /* Number of LED pulses, 0-32        */
  struct adps9922_ps_thresh
           thresh;      /* Upper and lower thresholds        */
  uint16_t cancel_lev;  /* Intelligent cancellation lev.     */
  int      int_mode;    /* Interrupt mode                    */
  uint8_t  persistance; /* Num events before interrupt       */
  int      notify;      /* States that cause a notify        */
};

/* data that can be read from proximity sensor */

struct apds9922_ps_data
{
  uint16_t ps;          /* Current prximity measure          */
  bool     close;       /* Object is far (false) or close    */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: apds9922_register
 *
 * Description:
 *   Register the APDS9922 character devices.
 *
 * Input Parameters:
 *   devpath_als - The full path to the driver to register for the als,
 *   e.g., "/dev/als0"
 *
 *   devpath_ps - The full path to the driver to register for the als,
 *   e.g., "/dev/ps0"
 *
 *   config - Pointer to the device configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apds9922_register(FAR const char *devpath_als,
                      FAR const char *devpath_ps,
                      FAR struct apds9922_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_APDS9922 */
#endif /* __INCLUDE_NUTTX_SENSORS_APDS9922_H */
