/****************************************************************************
 * include/nuttx/analog/ltc1867l.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_LTC1867L_H
#define __INCLUDE_NUTTX_ANALOG_LTC1867L_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>

#if defined(CONFIG_ADC_LTC1867L)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LTC1867L Configuration ***************************************************/

#define LTC1867L_CONFIG_BIT_SLP      (1 << 1)
#define LTC1867L_CONFIG_BIT_UNI      (1 << 2)
#define LTC1867L_CONFIG_BIT_COM      (1 << 3)
#define LTC1867L_CONFIG_BIT_S0       (1 << 4)
#define LTC1867L_CONFIG_BIT_S1       (1 << 5)
#define LTC1867L_CONFIG_BIT_OS       (1 << 6)
#define LTC1867L_CONFIG_BIT_SD       (1 << 7)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ltc1867l_analog_multiplexer_config_e
{
  LTC1867L_P_CH0_M_CH1 = 0,
  LTC1867L_P_CH2_M_CH3 = LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH4_M_CH5 = LTC1867L_CONFIG_BIT_S1,
  LTC1867L_P_CH6_M_CH7 = LTC1867L_CONFIG_BIT_S1 |
                         LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH1_M_CH0 = LTC1867L_CONFIG_BIT_OS,
  LTC1867L_P_CH3_M_CH2 = LTC1867L_CONFIG_BIT_OS |
                         LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH5_M_CH4 = LTC1867L_CONFIG_BIT_OS |
                         LTC1867L_CONFIG_BIT_S1,
  LTC1867L_P_CH7_M_CH6 = LTC1867L_CONFIG_BIT_OS |
                         LTC1867L_CONFIG_BIT_S1 |
                         LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH0_M_GND = LTC1867L_CONFIG_BIT_SD,
  LTC1867L_P_CH2_M_GND = LTC1867L_CONFIG_BIT_SD |
                         LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH4_M_GND = LTC1867L_CONFIG_BIT_SD |
                         LTC1867L_CONFIG_BIT_S1,
  LTC1867L_P_CH6_M_GND = LTC1867L_CONFIG_BIT_SD |
                         LTC1867L_CONFIG_BIT_S1 |
                         LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH1_M_GND = LTC1867L_CONFIG_BIT_SD |
                         LTC1867L_CONFIG_BIT_OS,
  LTC1867L_P_CH3_M_GND = LTC1867L_CONFIG_BIT_SD |
                         LTC1867L_CONFIG_BIT_OS |
                         LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH5_M_GND = LTC1867L_CONFIG_BIT_SD |
                         LTC1867L_CONFIG_BIT_OS |
                         LTC1867L_CONFIG_BIT_S1,
  LTC1867L_P_CH7_M_GND = LTC1867L_CONFIG_BIT_SD |
                         LTC1867L_CONFIG_BIT_OS |
                         LTC1867L_CONFIG_BIT_S1 |
                         LTC1867L_CONFIG_BIT_S0,
  LTC1867L_P_CH0_M_CH7COM = LTC1867L_CONFIG_BIT_SD |
                            LTC1867L_CONFIG_BIT_COM,
  LTC1867L_P_CH2_M_CH7COM = LTC1867L_CONFIG_BIT_SD |
                            LTC1867L_CONFIG_BIT_S0 |
                            LTC1867L_CONFIG_BIT_COM,
  LTC1867L_P_CH4_M_CH7COM = LTC1867L_CONFIG_BIT_SD |
                            LTC1867L_CONFIG_BIT_S1 |
                            LTC1867L_CONFIG_BIT_COM,
  LTC1867L_P_CH6_M_CH7COM = LTC1867L_CONFIG_BIT_SD |
                            LTC1867L_CONFIG_BIT_S1 |
                            LTC1867L_CONFIG_BIT_S0 |
                            LTC1867L_CONFIG_BIT_COM,
  LTC1867L_P_CH1_M_CH7COM = LTC1867L_CONFIG_BIT_SD |
                            LTC1867L_CONFIG_BIT_OS |
                            LTC1867L_CONFIG_BIT_COM,
  LTC1867L_P_CH3_M_CH7COM = LTC1867L_CONFIG_BIT_SD |
                            LTC1867L_CONFIG_BIT_OS |
                            LTC1867L_CONFIG_BIT_S0 |
                            LTC1867L_CONFIG_BIT_COM,
  LTC1867L_P_CH5_M_CH7COM = LTC1867L_CONFIG_BIT_SD |
                            LTC1867L_CONFIG_BIT_OS |
                            LTC1867L_CONFIG_BIT_S1 |
                            LTC1867L_CONFIG_BIT_COM
};

enum ltc1867l_analog_input_mode_e
{
  LTC1867L_UNIPOLAR = LTC1867L_CONFIG_BIT_UNI,
  LTC1867L_BIPOLAR = 0,
};

begin_packed_struct struct ltc1867l_channel_config_s
{
  uint8_t channel;                                                     /* This will be the channel number returned in struct adc_msg_s for a conversion */
  enum ltc1867l_analog_multiplexer_config_e analog_multiplexer_config; /* Analog multiplexer configuration */
  enum ltc1867l_analog_input_mode_e analog_inputmode;                  /* Analog input mode */
} end_packed_struct;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ltc1867l_register
 *
 * Description:
 *   Register the LTC1867L character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/adc0"
 *   spi - An instance of the SPI interface to use to communicate with
 *     LTC1867L
 *   devno - SPI device number
 *   channel_config - A pointer to an array which holds the configuration
 *     for each sampled channel.
 *   channel_config_count - Number of channels to sample
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltc1867l_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                      unsigned int devno,
                      FAR struct ltc1867l_channel_config_s *channel_config,
                      int channel_config_count);

#endif /* CONFIG_ADC_LTC1867L */
#endif /* __INCLUDE_NUTTX_ANALOG_LTC1867L_H */
