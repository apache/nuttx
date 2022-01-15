/****************************************************************************
 * include/nuttx/sensors/hts221.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_HTS221_H
#define __INCLUDE_NUTTX_SENSORS_HTS221_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HTS221_TEMPERATURE_PRECISION  100
#define HTS221_HUMIDITY_PRECISION     10

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

/* Number of temperature samples */

typedef enum hts221_avrg_temp_e
{
  HTS221_AVGT2 = 0,
  HTS221_AVGT4,
  HTS221_AVGT8,
  HTS221_AVGT16,              /* Default value */
  HTS221_AVGT32,
  HTS221_AVGT64,
  HTS221_AVGT128,
  HTS221_AVGT256
} hts221_avrg_temp_t;

/* Number of humidity samples */

typedef enum hts221_avrg_humid_e
{
  HTS221_AVGH4 = 0,
  HTS221_AVGH8,
  HTS221_AVGH16,
  HTS221_AVGH32,              /* Default value */
  HTS221_AVGH64,
  HTS221_AVGH128,
  HTS221_AVGH256,
  HTS221_AVGH512
}hts221_avrg_humid_t;

/* Output data rate configuration */

typedef enum hts221_odr_e
{
  HTS221_ODR_ONESHOT = 0,
  HTS221_ODR_1HZ,
  HTS221_ODR_7HZ,
  HTS221_ODR_12_5HZ
} hts221_odr_t;

/* Configuration structure */

typedef struct hts221_settings_s
{
  hts221_avrg_temp_t temp_resol;      /* Temperature resolution. The more
                                       * samples sensor takes, the more power
                                       * it uses */
  hts221_avrg_humid_t humid_resol;    /* Humidity resolution. The more
                                       * samples sensor takes, the more power
                                       * it uses */
  hts221_odr_t odr;                   /* Output data rate */
  bool is_bdu;                        /* If read operation is not faster than output
                                       * operation, then this variable must be set to true */
  bool is_data_rdy;                   /* Must be set to true, if interrupt needed.
                                       * Default is 0, disabled */
  bool is_high_edge;                  /* High or low interrupt signal from device.
                                       * Default is high, 0 */
  bool is_open_drain;                 /* Open drain or push-pull on data-ready pin.
                                       * Default is push-pull, 0 */
  bool is_boot;                       /* Refresh the content of the internal registers */
} hts221_settings_t;

/* Interrupt configuration data structure */

typedef struct hts221_config_s
{
  int irq;
  CODE int  (*irq_attach)(FAR struct hts221_config_s * state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR const struct hts221_config_s *state,
                          bool enable);
  CODE void (*irq_clear)(FAR const struct hts221_config_s *state);
  CODE int  (*set_power)(FAR const struct hts221_config_s *state, bool on);
} hts221_config_t;

/* Raw data structure */

typedef struct hts221_raw_data_s
{
  uint8_t humid_low_bits;
  uint8_t humid_high_bits;
  uint8_t temp_low_bits;
  uint8_t temp_high_bits;
} hts221_raw_data_t;

typedef struct hts221_conv_data_s
{
  int temperature;
  unsigned int humidity;
} hts221_conv_data_t;

/* Status register data */

typedef struct hts221_status_s
{
  bool is_humid_ready;
  bool is_temp_ready;
} hts221_status_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int hts221_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, hts221_config_t * config);

#endif /* __INCLUDE_NUTTX_SENSORS_HTS221_H */
