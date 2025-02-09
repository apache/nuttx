/****************************************************************************
 * boards/arm/rp2040/rakwireless-rak11310/src/sx126x.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

 #include "sx126x.h"

 #include <nuttx/arch.h>
 #include <nuttx/config.h>
 
 #include <debug.h>
 #include <errno.h>
 #include <nuttx/spi/spi.h>
 #include <sched.h>
 #include <stdint.h>
 #include <stdio.h>
 #include <sys/endian.h>
 #include <syslog.h>
 #include <unistd.h>
 
 /****************************************************************************
  * Private prototypes
  ****************************************************************************/
 
 typedef FAR struct file file_t;
 static int sx126x_open(file_t *filep);
 static int sx126x_close(file_t *filep);
 static ssize_t sx126x_read(file_t *filep, FAR char *buffer, size_t buflen);
 static ssize_t sx126x_write(file_t *filep, FAR const char *buf,
                             size_t buflen);
 
 /****************************************************************************
  * Private data types
  ****************************************************************************/
 
 struct sx126x_dev_s
 {
   struct spi_dev_s *spi;
   const struct sx126x_lower_s *lower;
 };
 
 enum sx126x_cmd_status
 {
   SX126X_STATUS_RESERVED, SX126X_STATUS_RFU, SX126X_STATUS_DATA_AVAILABLE,
   SX126X_STATUS_TIMEOUT, SX126X_STATUS_ERROR, SX126X_STATUS_EXECUTE_FAIL,
   SX126X_STATUS_TX_DONE
 };
 
 enum sx126x_chip_mode
 {
   SX126X_MODE_UNUSED, SX126X_MODE_RFU, SX126X_MODE_STBY_RC,
   SX126X_MODE_STBY_XOSC, SX126X_MODE_FS, SX126X_MODE_RX, SX126X_MODE_TX
 };
 
 struct sx126x_status_s
 {
   enum sx126x_cmd_status cmd;
   enum sx126x_chip_mode mode;
 };
 
 /****************************************************************************
  * Globals
  ****************************************************************************/
 
 FAR struct sx126x_dev_s             g_sx126x_devices[SX126X_MAX_DEVICES];
 static const struct file_operations sx126x_ops =
 {
   sx126x_open, sx126x_close, sx126x_read, sx126x_write, 0, 0,
 };
 
 /****************************************************************************
  * Private prototypes
  ****************************************************************************/
 
 /* SPI and control **********************************************************/
 
 static void sx126x_command(FAR struct sx126x_dev_s *dev, uint8_t cmd,
                            FAR const uint8_t *params, size_t paramslen,
                            FAR uint8_t *returns);
 
 static void sx126x_reset(FAR struct sx126x_dev_s *dev);
 
 static void sx126x_get_status(FAR struct sx126x_dev_s *dev,
                               FAR struct sx126x_status_s *status);
  
 static void sx126x_test(FAR struct sx126x_dev_s *dev);
 
 static void sx126x_spi_lock(FAR struct sx126x_dev_s *dev);
 
 static void sx126x_spi_unlock(FAR struct sx126x_dev_s *dev);
 
 static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                   uint16_t address, uint8_t *data,
                                   size_t data_length);
 
 /* Operational modes functions **********************************************/
 
 static void sx126x_set_standby(FAR struct sx126x_dev_s *dev,
                                enum sx126x_standby_mode_e mode);
 
 static void sx126x_set_tx(FAR struct sx126x_dev_s *dev, uint32_t timeout);
 
 static void sx126x_set_cad(struct sx126x_dev_s *dev);
 
 static void sx126x_set_tx_continuous_wave(FAR struct sx126x_dev_s *dev);
 
 static void sx126x_set_regulator_mode(FAR struct sx126x_dev_s *dev,
                                       enum sx126x_regulator_mode_e mode);
 
 static void sx126x_set_pa_config(FAR struct sx126x_dev_s *dev,
                                  enum sx126x_device_e model, uint8_t hpmax,
                                  uint8_t padutycycle);
 
 static void sx126x_set_tx_infinite_preamble(FAR struct sx126x_dev_s *dev);
 
 /* DIO and IRQ control functions ********************************************/
 
 static void sx126x_set_dio_irq_params(FAR struct sx126x_dev_s *dev,
                                       uint16_t irq_mask,
                                       uint16_t dio1_mask,
                                       uint16_t dio2_mask,
                                       uint16_t dio3_mask);
 
 static void sx126x_set_dio2_as_rf_switch(FAR struct sx126x_dev_s *dev,
                                          bool enable);
 
 static void sx126x_set_dio3_as_tcxo(FAR struct sx126x_dev_s *dev,
                                     enum sx126x_tcxo_voltage_e voltage,
                                     uint32_t delay);
 
 /* RF Modulation and Packet-Related Functions *******************************/
 
 static void sx126x_set_packet_params_lora(FAR struct sx126x_dev_s *dev,
                                           FAR struct
                                           sx126x_packetparams_lora_s *
                                           pktparams);
 
 static void sx126x_set_modulation_params_lora(FAR struct sx126x_dev_s *dev,
                                               FAR struct
                                               sx126x_modparams_lora_s *
                                               modparams);
 
 static void sx126x_set_buffer_base_address(FAR struct sx126x_dev_s *dev,
                                            uint8_t tx, uint8_t rx);
 
 static void sx126x_set_tx_params(FAR struct sx126x_dev_s *dev, uint8_t power,
                                  enum sx126x_ramp_time_e ramp_time);
 
 static void sx126x_set_packet_type(FAR struct sx126x_dev_s *dev,
                                    enum sx126x_packet_type_e type);
 
 static void sx126x_get_rf_frequency(FAR struct sx126x_dev_s *dev,
                                     uint32_t frequency_hz);
 
 /* Communication status information *****************************************/
 
 static void sx126x_get_rssi_inst(FAR struct sx126x_dev_s *dev,
                                  FAR float *dBm);
 
 /* Registers and buffer *****************************************************/
 
 static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                   uint16_t address, uint8_t *data,
                                   size_t data_length);
 
 static void sx126x_write_buffer(FAR struct sx126x_dev_s *dev, uint8_t offset,
                                 FAR const uint8_t *payload, uint8_t len);
 
 static void sx126x_read_buffer(FAR struct sx126x_dev_s *dev, uint8_t offset,
                                FAR uint8_t *payload, uint8_t len);
 
 /* Register settings ********************************************************/
 
 static void sx126x_set_syncword(FAR struct sx126x_dev_s *dev,
                                 uint8_t *syncword, uint8_t syncword_length);
 
 /****************************************************************************
  * Private Functions
  ****************************************************************************/
 
 /* File operations **********************************************************/
 
 static int sx126x_open(file_t *filep)
 {
   struct sx126x_dev_s *dev;
 
   dev = filep->f_inode->i_private;
 
   syslog(LOG_INFO, "Opening SX126x port %d\n", dev->lower->port);
 
   sx126x_spi_lock(dev);
   sx126x_reset(dev);
   usleep(100000);
   sx126x_test(dev);
 
   return OK;
 }
 
 static int sx126x_close(file_t *filep)
 {
   struct sx126x_dev_s *dev;
 
   dev = filep->f_inode->i_private;
 
   syslog(LOG_INFO, "Closing SX126x\n");
   sx126x_spi_unlock(dev);
   return OK;
 }
 
 static ssize_t sx126x_read(file_t *filep, FAR char *buf, size_t buflen)
 {
   if (buf == NULL || buflen < 1)
     {
       return -EINVAL;
     }
 
   printf("Reading\n");
 
   return 1;
 }
 
 static ssize_t sx126x_write(file_t *filep,
                             FAR const char *buf,
                             size_t buflen)
 {
   if (buf == NULL || buflen < 1)
     {
       return -EINVAL;
     }
 
   printf("Trying to write\n");
 
   return 1;
 }
 
 /* Test *********************************************************************/
 
 uint8_t sx126x_temp[0xff];
 uint8_t sx126x_temp_rets[0xff];
 
 uint32_t sx126x_convert_freq_in_hz_to_pll_step(uint32_t freq_in_hz)
 {
   uint32_t  steps_int;
   uint32_t  steps_frac;
 
   steps_int     = freq_in_hz / SX126X_PLL_STEP_SCALED;
   steps_frac    = freq_in_hz - (steps_int * SX126X_PLL_STEP_SCALED);
 
   return (steps_int <<
     SX126X_PLL_STEP_SHIFT_AMOUNT) + (((steps_frac <<
     SX126X_PLL_STEP_SHIFT_AMOUNT) + (SX126X_PLL_STEP_SCALED >>
                                      1)) / SX126X_PLL_STEP_SCALED);
 }
 
 static void sx126x_test(FAR struct sx126x_dev_s *dev)
 {
   struct sx126x_status_s status;
 
   sx126x_get_status(dev, &status);
   syslog(LOG_INFO, "Mode %d, Cmd %d", status.mode, status.cmd);
 
   /* Regulator */
 
   sx126x_set_regulator_mode(dev, SX126X_DC_DC_LDO);
 
   /* Set packet type */
 
   sx126x_set_packet_type(dev, SX126X_PACKETTYPE_LORA);
 
   /* Set RF frequency */
 
   sx126x_get_rf_frequency(dev, 869252000);
 
   /* Set PA */
 
   sx126x_set_pa_config(dev, SX1262, 0x01, 0x01);
 
   /* Set TX params */
 
   sx126x_set_tx_params(dev, 0xef, SX126X_SET_RAMP_200U);
 
   /* Set base */
 
   sx126x_set_buffer_base_address(dev, 0, 0xaa);
 
   /* Set mod params */
 
   struct sx126x_modparams_lora_s modparams = {
     .spreading_factor           = SX126X_LORA_SF12,
     .bandwidth                  = SX126X_LORA_BW_125,
     .coding_rate                = SX126X_LORA_CR_4_5,
     .low_datarate_optimization  = true
   };
 
   sx126x_set_modulation_params_lora(dev, &modparams);
 
   /* Packet params */
 
   struct sx126x_packetparams_lora_s pktparams = {
     .crc_enable     = false,  .fixed_length_header   = true,
     .payload_length = 12,
     .invert_iq      = false, .preambles             = 4
   };
 
   sx126x_set_packet_params_lora(dev, &pktparams);
 
   /* Sync word */
 
   uint8_t syncword[] = {
     0xaa, 0xbb, 0xaa, 0xbb
   };
 
   sx126x_set_syncword(dev, syncword, sizeof(syncword));
 
   /* IRQ MASK */
 
   sx126x_set_dio_irq_params(dev, SX126X_IRQ_TXDONE_MASK,
                             SX126X_IRQ_TXDONE_MASK, 0x00, 0x00);
 
   /* DIO 2 */
 
   sx126x_set_dio2_as_rf_switch(dev, true);
 
   /* DIO 3 */
 
   sx126x_set_dio3_as_tcxo(dev, SX126X_TCXO_3_3V, 200);
 
   /* Data */
 
   const char payload[] =
     "hello";
   sx126x_write_buffer(dev, 0, (uint8_t *)payload, sizeof(payload));
 
   /* TX mode */
 
   sx126x_set_tx(dev, 0);
 
   sx126x_get_status(dev, &status);
   syslog(LOG_INFO, "Mode %d, Cmd %d", status.mode, status.cmd);
 }
 
 /* Operational modes functions **********************************************/
 
 static void sx126x_set_standby(FAR struct sx126x_dev_s *dev,
                                enum sx126x_standby_mode_e mode)
 {
   sx126x_command(dev, SX126X_SETSTANDBY, (uint8_t *)&mode,
                  SX126X_SETSTANDBY_PARAMS, NULL);
 }
 
 static void sx126x_set_tx(FAR struct sx126x_dev_s *dev, uint32_t timeout)
 {
   /* Convert timeout to BE24 */
 
   timeout = htobe32(timeout << 8);
 
   sx126x_command(dev, SX126X_SETTX, (uint8_t *)&timeout, SX126X_SETTX_PARAMS,
                  NULL);
 }
 
 static void sx126x_set_rx(FAR struct sx126x_dev_s *dev, uint32_t timeout)
 {
   /* Convert timeout to BE24 */
 
   timeout = htobe32(timeout << 8);
 
   sx126x_command(dev, SX126X_SETRX, (uint8_t *)&timeout, SX126X_SETRX_PARAMS,
                  NULL);
 }
 
 static void sx126x_stop_timer_on_preamble(FAR struct sx126x_dev_s *dev,
                                           bool enable)
 {
   sx126x_command(dev, SX126X_STOPTIMERONPREAMBLE, (uint8_t *)&enable,
                  SX126X_STOPTIMERONPREAMBLE_PARAMS, NULL);
 }
 
 static void sx126x_set_rx_duty_cycle(FAR struct sx126x_dev_s *dev,
                                      uint32_t rx_period,
                                      uint32_t sleep_period)
 {
   uint8_t params[SX126X_SETRXDUTYCYCLE_PARAMS];
 
   rx_period     = htobe32(rx_period << 8);
   sleep_period  = htobe32(sleep_period << 8);
 
   memcpy(params +
          SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAM,
          (uint8_t *)&rx_period,
          SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAMS);
   memcpy(params + SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAM,
          (uint8_t *)&sleep_period, SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAMS);
 
   sx126x_command(dev, SX126X_SETRXDUTYCYCLE, params,
                  SX126X_SETRXDUTYCYCLE_PARAMS, NULL);
 }
 
 static void sx126x_set_cad(FAR struct sx126x_dev_s *dev)
 {
   sx126x_command(dev, SX126X_SETCAD, NULL, 0, NULL);
 }
 
 static void sx126x_set_tx_continuous_wave(FAR struct sx126x_dev_s *dev)
 {
   sx126x_command(dev,
                 SX126X_SETTXCONTINUOUSWAVE,
                 NULL, 0, NULL);
 }
 
 static void sx126x_set_regulator_mode(FAR struct sx126x_dev_s *dev,
                                       enum sx126x_regulator_mode_e mode)
 {
   sx126x_command(dev, SX126X_SETREGULATORMODE, (uint8_t *)&mode,
                  SX126X_SETREGULATORMODE_PARAMS, NULL);
 }
 
 /* Caution! Exceeding the limits listed in DS_SX1261/2 V2.1
  * 13.1.14.1 may cause irreversible damage to the device
  */
 
 static void sx126x_set_pa_config(FAR struct sx126x_dev_s *dev,
                                  enum sx126x_device_e model, uint8_t hpmax,
                                  uint8_t padutycycle)
 {
   uint8_t params[SX126X_SETPACONFIG_PARMS];
 
   memset(params, 0, SX126X_SETPACONFIG_PARMS);
 
   params[SX126X_SETPACONFIG_PADUTYCYCLE_PARAM]  = padutycycle;
   params[SX126X_SETPACONFIG_HPMAX_PARAM]        = hpmax;
   params[SX126X_SETPACONFIG_DEVICESEL_PARAM]    = model;
   params[SX126X_SETPACONFIG_PALUT_PARAM]        = 0x01;
 
   sx126x_command(dev, SX126X_SETPACONFIG, params, SX126X_SETPACONFIG_PARMS,
                  NULL);
 }
 
 static void sx126x_set_tx_infinite_preamble(FAR struct sx126x_dev_s *dev)
 {
   sx126x_command(dev, SX126X_SETTXINFINITEPREAMBLE, NULL, 0, NULL);
 }
 
 static void sx126x_set_rx_tx_fallback_mode(FAR struct sx126x_dev_s *dev,
                                    enum sx126x_fallback_mode_e fallback)
 {
   sx126x_command(dev, SX126X_SETRXTXFALLBACKMODE,
                  (uint8_t *)&fallback,
                  SX126X_SETRXTXFALLBACKMODE_PARAMS,
                  NULL);
 }
 
 /* DIO and IRQ control functions */
 
 static void sx126x_set_dio_irq_params(FAR struct sx126x_dev_s *dev,
                                       uint16_t irq_mask, uint16_t dio1_mask,
                                       uint16_t dio2_mask, uint16_t dio3_mask)
 {
   irq_mask  = htobe16(irq_mask);
   dio1_mask = htobe16(dio1_mask);
   dio2_mask = htobe16(dio2_mask);
   dio3_mask = htobe16(dio3_mask);
 
   uint8_t params[SX126X_SETDIOIRQPARAMS_PARAMS];
 
   memcpy(params + SX126X_SETDIOIRQPARAMS_IRQMASK_PARAM, &irq_mask,
          SX126X_SETDIOIRQPARAMS_IRQMASK_PARAMS);
 
   memcpy(params + SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAM, &dio1_mask,
          SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAMS);
 
   memcpy(params + SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAM, &dio2_mask,
          SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAMS);
 
   memcpy(params + SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAM, &dio3_mask,
          SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAMS);
 
   sx126x_command(dev, SX126X_SETDIOIRQPARAMS, params,
                  SX126X_SETDIOIRQPARAMS_PARAMS, NULL);
 }
 
 static void sx126x_set_dio2_as_rf_switch(FAR struct sx126x_dev_s *dev,
                                          bool enable)
 {
   sx126x_command(dev, SX126X_SETDIO2RFSWCTRL, (uint8_t *)&enable,
                  SX126X_SETDIO2RFSWCTRL_PARAMS, NULL);
 }
 
 static void sx126x_set_dio3_as_tcxo(FAR struct sx126x_dev_s *dev,
                                     enum sx126x_tcxo_voltage_e voltage,
                                     uint32_t delay)
 {
   uint8_t params[SX126X_SETDIO3TCXOCTRL_PARAMS];
 
   params[SX126X_SETDIO3TCXOCTRL_TCXO_V_PARAM] = voltage;
 
   /* Convert delay to 24 bit and convert to BE */
 
   delay = htobe32(delay << 8);
   memcpy(params + SX126X_SETDIO3TCXOCTRL_DELAY_PARAM, &delay,
          SX126X_SETDIO3TCXOCTRL_DELAY_PARAMS);
 
   sx126x_command(dev, SX126X_SETDIO3TCXOCTRL, params,
                  SX126X_SETDIO3TCXOCTRL_PARAMS, NULL);
 }
 
 /* RF Modulation and Packet-Related Functions *******************************/
 
 static void sx126x_set_packet_params_lora(FAR struct sx126x_dev_s *dev,
                                           FAR struct
                                           sx126x_packetparams_lora_s *
                                           pktparams)
 {
   uint8_t params[SX126X_SETPACKETPARMS_PARAMS];
 
   memset(params, 0, SX126X_SETPACKETPARMS_PARAMS);
 
   uint16_t preambles = htobe16(pktparams->preambles);
   memcpy(params + SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAM, &preambles,
          SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAMS);
 
   params[SX126X_PKTPARAM3_LORA_HEADERTYPE_PARAM] =
     pktparams->fixed_length_header;
   params[SX126X_PKTPARAM4_LORA_PAYLOADLEN_PARAM] =
     pktparams->payload_length;
   params[SX126X_PKTPARAM5_LORA_CRCTYPE_PARAM]   = pktparams->crc_enable;
   params[SX126X_PKTPARAM6_LORA_INVERTIQ_PARAM]  = pktparams->invert_iq;
 
   sx126x_command(dev, SX126X_SETPACKETPARMS, params,
                  SX126X_SETPACKETPARMS_PARAMS, NULL);
 }
 
 static void sx126x_set_modulation_params_lora(FAR struct sx126x_dev_s *dev,
                                               FAR struct
                                               sx126x_modparams_lora_s *
                                               modparams)
 {
   uint8_t params[SX126X_SETMODULATIONPARAMS_PARAMS];
 
   memset(sx126x_temp, 0, SX126X_SETMODULATIONPARAMS_PARAMS);
 
   params[SX126X_MODPARAM1_LORA_SF_PARAM] =
     modparams->spreading_factor;
   params[SX126X_MODPARAM2_LORA_BW_PARAM] =
     modparams->bandwidth;
   params[SX126X_MODPARAM3_LORA_CR_PARAM] =
     modparams->coding_rate;
   params[SX126X_MODPARAM4_LORA_LOWDATRATE_OPTI_PARAM] =
     modparams->low_datarate_optimization;
 
   sx126x_command(dev, SX126X_SETMODULATIONPARAMS, params,
                  SX126X_SETMODULATIONPARAMS_PARAMS, NULL);
 }
 
 static void sx126x_set_buffer_base_address(FAR struct sx126x_dev_s *dev,
                                            uint8_t tx, uint8_t rx)
 {
   uint8_t params[SX126X_SETBUFFERBASEADDRESS_PARAMS];
 
   memset(params, 0, SX126X_SETBUFFERBASEADDRESS_PARAMS);
 
   params[SX126X_SETBUFFERBASEADDRESS_TX_PARAM]  = tx;
   params[SX126X_SETBUFFERBASEADDRESS_RX_PARAM]  = rx;
 
   sx126x_command(dev, SX126X_SETBUFFERBASEADDRESS, params,
                  SX126X_SETBUFFERBASEADDRESS_PARAMS, NULL);
 }
 
 static void sx126x_set_tx_params(FAR struct sx126x_dev_s *dev, uint8_t power,
                                  enum sx126x_ramp_time_e ramp_time)
 {
   uint8_t params[SX126X_SETTXPARMS_PARAMS];
 
   memset(params, 0, SX126X_SETTXPARMS_PARAMS);
 
   params[SX126X_SETTXPARMS_RAMPTIME_PARAM]  = ramp_time;
   params[SX126X_SETTXPARMS_POWER_PARAM]     = power;
 
   sx126x_command(dev, SX126X_SETTXPARMS, params, SX126X_SETTXPARMS_PARAMS,
                  NULL);
 }
 
 static void sx126x_set_packet_type(FAR struct sx126x_dev_s *dev,
                                    enum sx126x_packet_type_e type)
 {
   sx126x_command(dev, SX126X_SETPACKETTYPE, (uint8_t *)&type,
                  SX126X_SETPACKETTYPE_PARAMS, NULL);
 }
 
 static void sx126x_get_rf_frequency(FAR struct sx126x_dev_s *dev,
                                     uint32_t frequency_hz)
 {
   uint32_t corrected_freq =
     sx126x_convert_freq_in_hz_to_pll_step(frequency_hz);
 
   corrected_freq = htobe32(corrected_freq);
 
   sx126x_command(dev, SX126X_SETRFFREQUENCY, (uint8_t *)&corrected_freq,
                  SX126X_SETRFFREQUENCY_PARAMS, NULL);
 }
 
 static void sx126x_set_lora_symb_num_timout(FAR struct sx126x_dev_s *dev,
                                             uint8_t symbnum)
 {
   sx126x_command(dev,
                  SX126X_SETLORASYMBNUMTIMEOUT,
                  &symbnum,
                  SX126X_SETLORASYMBNUMTIMEOUT_PARAMS,
                  NULL);
 }
 
 /* Communication status information *****************************************/
 
 static void sx126x_get_status(FAR struct sx126x_dev_s *dev,
                               FAR struct sx126x_status_s *status)
 {
   /* NOP param to shift out result */
 
   uint8_t   parms[1] = {
     0x00
   };
 
   uint8_t   rets[1];
 
   /* Get, mask and shift result into readable mode numbers */
 
   sx126x_command(dev, SX126X_CMD_GETSTATUS, parms, sizeof(parms), rets);
   status->mode = (rets[0] & SX126X_STATUS_CHIPMODE_MASK) >>
                  SX126X_STATUS_CHIPMODE_SHIFT;
   status->cmd = (rets[0] & SX126X_STATUS_CMD_MASK) >>
                 SX126X_STATUS_CMD_SHIFT;
 }
 
 static void sx126x_get_rssi_inst(FAR struct sx126x_dev_s *dev,
                                  FAR float *dBm)
 {
   uint8_t rets[SX126X_GETRSSIINST_RETURNS];
   sx126x_command(dev,
                 SX126X_GETRSSIINST,
                 NULL,
                 SX126X_GETRSSIINST_RETURNS,
                 rets);
   
   /* Calculate dBm from returns */
 
   float rssi = rets[SX126X_GETRSSIINST_RSSI_RETURN];
   (*dBm) = -rssi/2.0;
 }
 
 /* Lower hardware control ***************************************************/
 
 static void sx126x_reset(FAR struct sx126x_dev_s *dev)
 {
   dev->lower->reset();
 }
 
 /* SPI Communication ********************************************************/
 
 static void sx126x_select(FAR struct sx126x_dev_s *dev)
 {
   SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->port), true);
 }
 
 static void sx126x_deselect(FAR struct sx126x_dev_s *dev)
 {
   SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->port), false);
 }
 
 static void sx126x_spi_lock(FAR struct sx126x_dev_s *dev)
 {
   struct spi_dev_s *spi = dev->spi;
 
   SPI_LOCK(spi, true);
   SPI_SETBITS(spi, 8);
   SPI_SETMODE(spi, SPIDEV_MODE0);
   SPI_SETFREQUENCY(spi, SX126X_SPI_SPEED);
 }
 
 static void sx126x_spi_unlock(FAR struct sx126x_dev_s *dev)
 {
   SPI_LOCK(dev->spi, false);
 }
 
 static void sx126x_command(FAR struct sx126x_dev_s *dev, uint8_t cmd,
                            const FAR uint8_t *params, size_t paramslen,
                            FAR uint8_t *returns)
 {
   sx126x_select(dev);
 
   /* First send the command. This does not return anything.
    * "RFU" according the manual
    */
 
   SPI_SEND(dev->spi, cmd);
 
   /* Send all the params and record the returning bytes */
 
   for (size_t i = 0; i < paramslen; i++)
     {
       uint8_t param=SX126X_NOP;
       if (params != NULL)
         {
           param = params[i];
         }
 
       uint8_t ret = SPI_SEND(dev->spi, param);
 
       if (returns != NULL)
         {
           returns[i] = ret;
         }
     }
 
   sx126x_deselect(dev);
 }
 
 /* Registers and buffer *****************************************************/
 
 static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                   uint16_t address, uint8_t *data,
                                   size_t data_length)
 {
   syslog(LOG_DEBUG, "Writing register %X", address);
   sx126x_select(dev);
 
   /* Send the opcode and address */
 
   SPI_SEND(dev->spi, SX126X_WRITEREGISTER);
   SPI_SEND(dev->spi, (uint8_t)(address >> 8));
   SPI_SEND(dev->spi, (uint8_t)address);
 
   /* Send data */
 
   for (size_t i = 0; i < data_length; i++)
     {
       SPI_SEND(dev->spi, data[i]);
     }
 
   sx126x_deselect(dev);
 }
 
 static void sx126x_read_register(FAR struct sx126x_dev_s *dev,
   uint16_t address, uint8_t *data,
   size_t data_length)
 {
   sx126x_select(dev);
 
   /* Send the opcode and address */
 
   SPI_SEND(dev->spi, SX126X_WRITEREGISTER);
   SPI_SEND(dev->spi, (uint8_t)(address >> 8));
   SPI_SEND(dev->spi, (uint8_t)address);
 
   /* Send data */
 
   for (size_t i = 0; i < data_length; i++)
     {
     data[i] = SPI_SEND(dev->spi, SX126X_NOP);
     }
 
   sx126x_deselect(dev);
 }
 
 static void sx126x_write_buffer(FAR struct sx126x_dev_s *dev, uint8_t offset,
                                 FAR const uint8_t *payload, uint8_t len)
 {
   sx126x_select(dev);
 
   /* Command */
 
   SPI_SEND(dev->spi, SX126X_WRITEBUFFER);
 
   /* Offset */
 
   SPI_SEND(dev->spi, offset);
 
   /* Data */
 
   for (size_t i = 0; i < len; i++)
     {
       SPI_SEND(dev->spi, payload[i]);
     }
 
   sx126x_deselect(dev);
 }
 
 static void sx126x_read_buffer(FAR struct sx126x_dev_s *dev, uint8_t offset,
                                FAR uint8_t *payload, uint8_t len)
 {
   sx126x_select(dev);
 
   /* Command */
 
   SPI_SEND(dev->spi, SX126X_READBUFFER);
 
   /* Offset */
 
   SPI_SEND(dev->spi, offset);
 
   /* Data */
 
   for (size_t i = 0; i < len; i++)
     {
       payload[i] = SPI_SEND(dev->spi, SX126X_NOP);
     }
 
   sx126x_deselect(dev);
 }
 
 /* Register settings ********************************************************/
 
 static void sx126x_set_syncword(FAR struct sx126x_dev_s *dev,
                                 uint8_t *syncword, uint8_t syncword_length)
 {
   if (syncword_length > SX126X_REG_SYNCWORD_LEN)
     {
       syncword_length = SX126X_REG_SYNCWORD_LEN;
       syslog(LOG_WARNING,
              "Syncword length was limited to the maximum 8 bytes.");
     }
 
   sx126x_write_register(dev, SX126X_REG_SYNCWORD, syncword, syncword_length);
 }
 
 /****************************************************************************
  * Public Functions
  ****************************************************************************/
 
 void sx126x_register(FAR struct spi_dev_s *spi,
                      FAR const struct sx126x_lower_s *lower)
 {
   if (lower->port >= SX126X_MAX_DEVICES)
     {
       syslog(LOG_ERR,
              "SX126x port %d is greater than \
              allowed amount of SX126x devices",
              lower->port);
       return;
     }
 
   g_sx126x_devices[lower->port].lower   = lower;
   g_sx126x_devices[lower->port].spi     = spi;
 
   (void)register_driver("/dev/sx126x", &sx126x_ops, 0444,
                         &g_sx126x_devices[lower->port]);
 }
 