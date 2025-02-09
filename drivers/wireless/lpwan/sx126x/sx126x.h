/****************************************************************************
 * boards/arm/rp2040/rakwireless-rak11310/src/sx126x.h
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

/* All functions and definitions are accurately recreated
 * from the official datasheet "DS_SX1261-2_V2_1"
 */

 #ifndef __DRIVERS_WIRELESS_LPWAN_SX126X_SX126X_H
 #define __DRIVERS_WIRELESS_LPWAN_SX126X_SX126X_H
 
 /****************************************************************************
  * Included Files
  ****************************************************************************/
 
 #include <debug.h>
 #include <nuttx/config.h>
 #include <nuttx/spi/spi.h>
 #include <nuttx/irq.h>
 #include <nuttx/wireless/ioctl.h>
 
 #include <stdint.h>
 #include <stdbool.h>
 #include <endian.h>
 
 /****************************************************************************
  * Settings
  ****************************************************************************/
 
 #define SX126X_MAX_DEVICES 2
 #define SX126X_SPI_SPEED   1000000
 
 /****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/
 
 /* Register addresses *******************************************************/
 
 #define SX126X_REG_SYNCWORD       0x06C0 /* Start of Byte 0 - Byte 7 */
 #define SX126X_REG_SYNCWORD_LEN   8      /* Bytes */
 #define SX126X_REG_NODEADDR       0x06CD /* Node address to filter. Default 0x00 */      
 #define SX126X_REG_BRDCASTADDR    0x06CE /* Broadcast address to filter. Default 0x00 */      
 #define SX126X_REG_CRC_INIT_MSB   0x06BC /* Default 0x1D */
 #define SX126X_REG_CRC_INIT_LSB   0x06BD /* Default 0x0F */
 #define SX126X_REG_CRC_POLY_MSB   0x06BE /* Default 0x10 */
 #define SX126X_REG_CRC_POLY_LSB   0x06BF /* Default 0x21 */
 #define SX126X_REG_WHITENING_MSB  0x06B8 /* Default 0x01 */
 #define SX126X_REG_WHITENING_LSB  0x06B9 /* Default 0x00 */
 
 /* IRQ Register bits ********************************************************/
 
 #define SX126X_IRQ_TXDONE_MASK              (1<<0)
 #define SX126X_IRQ_RXDONE_MASK              (1<<1)
 #define SX126X_IRQ_PREAMBLEDETECTED_MASK    (1<<2)
 #define SX126X_IRQ_SYNCWORDVALID_MASK       (1<<3)
 #define SX126X_IRQ_HEADERVALID_MASK         (1<<4)
 #define SX126X_IRQ_HEADERERR_MASK           (1<<5)
 #define SX126X_IRQ_CRCERR_MASK              (1<<6)
 #define SX126X_IRQ_CADDONE_MASK             (1<<7)
 #define SX126X_IRQ_CADDETECTED_MASK         (1<<8)
 #define SX126X_IRQ_TIMEOUT_MASK             (1<<9)
 #define SX126X_IRQ_LRFHSSHOP_MASK           (1<<14)
 
 /* Enum and constant definitions ********************************************/
 
 /* Public??????? */
 
 /* Others */
 
 #define SX126X_NOP                   0
 #define SX126X_NO_TIMEOUT            0
 #define SX126X_NO_DELAY              0
 
 /* Oscillators and PLLs */
 
 #define SX126X_OSC_MAIN_HZ           (32000000)
 #define SX126X_FXTAL                 SX126X_OSC_MAIN_HZ
 #define SX126X_PLL_STEP_SHIFT_AMOUNT (14)
 #define SX126X_PLL_STEP_SCALED       (SX126X_FXTAL>>(25-SX126X_PLL_STEP_SHIFT_AMOUNT))
 
 /* Standby config */
 
 enum sx126x_standby_mode_e
 {
   SX126X_STDBY_RC    = 0x00,
   SX126X_STDBY_XOSC  = 0x01
 };
 
 /* Packet Types */
 
 enum sx126x_packet_type_e
 {
   SX126X_PACKETTYPE_GFSK    = 0x00,
   SX126X_PACKETTYPE_LORA    = 0x01,
   SX126X_PACKETTYPE_LR_FHSS = 0x13
 };
 
 /* Ramp times */
 
 enum sx126x_ramp_time_e
 {
   SX126X_SET_RAMP_10U   = 0x00,
   SX126X_SET_RAMP_20U   = 0x01,
   SX126X_SET_RAMP_40U   = 0x02,
   SX126X_SET_RAMP_80U   = 0x03,
   SX126X_SET_RAMP_200U  = 0x04,
   SX126X_SET_RAMP_800U  = 0x05,
   SX126X_SET_RAMP_1700U = 0x06,
   SX126X_SET_RAMP_3400U = 0x07
 };
 
 /* GFSK Pulse shapes */
 
 enum sx126x_gfsk_pulseshape_e
 {
   SX126X_GFSK_PULSESHAPE_NONE            = 0x00,
   SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_3 = 0x08,
   SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_5 = 0x09,
   SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_7 = 0x0a,
   SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_1   = 0x0b
 };
 
 /* GFSK Bandwidths in Hz */
 
 enum sx126x_gfsk_bandwidth_e
 {
   SX126X_GFSK_BANDWIDTH_4800HZ   = 0x1f,
   SX126X_GFSK_BANDWIDTH_5800HZ   = 0x17,
   SX126X_GFSK_BANDWIDTH_7300HZ   = 0x0f,
   SX126X_GFSK_BANDWIDTH_9700HZ   = 0x1e,
   SX126X_GFSK_BANDWIDTH_11700HZ  = 0x16,
   SX126X_GFSK_BANDWIDTH_14600HZ  = 0x0e,
   SX126X_GFSK_BANDWIDTH_19500HZ  = 0x1d,
   SX126X_GFSK_BANDWIDTH_23400HZ  = 0x15,
   SX126X_GFSK_BANDWIDTH_29300HZ  = 0x0d,
   SX126X_GFSK_BANDWIDTH_39000HZ  = 0x1c,
   SX126X_GFSK_BANDWIDTH_46900HZ  = 0x14,
   SX126X_GFSK_BANDWIDTH_58600HZ  = 0x0c,
   SX126X_GFSK_BANDWIDTH_78200HZ  = 0x1b,
   SX126X_GFSK_BANDWIDTH_93800HZ  = 0x13,
   SX126X_GFSK_BANDWIDTH_117300HZ = 0x0b,
   SX126X_GFSK_BANDWIDTH_156200HZ = 0x1a,
   SX126X_GFSK_BANDWIDTH_187200HZ = 0x12,
   SX126X_GFSK_BANDWIDTH_234300HZ = 0x0a,
   SX126X_GFSK_BANDWIDTH_312000HZ = 0x19,
   SX126X_GFSK_BANDWIDTH_373600HZ = 0x11,
   SX126X_GFSK_BANDWIDTH_467000HZ = 0x09
 };
 
 /* LoRa Spreading Factors */
 
 enum sx126x_lora_sf_e
 {
   SX126X_LORA_SF5  = 0x05,
   SX126X_LORA_SF6  = 0x06,
   SX126X_LORA_SF7  = 0x07,
   SX126X_LORA_SF8  = 0x08,
   SX126X_LORA_SF9  = 0x09,
   SX126X_LORA_SF10 = 0x0a,
   SX126X_LORA_SF11 = 0x0b,
   SX126X_LORA_SF12 = 0x0c
 };
 
 /* LoRa Bandwidths */
 
 enum sx126x_lora_bw_e
 {
   SX126X_LORA_BW_7   = 0x00,
   SX126X_LORA_BW_10  = 0x08,
   SX126X_LORA_BW_15  = 0x01,
   SX126X_LORA_BW_20  = 0x09,
   SX126X_LORA_BW_31  = 0x02,
   SX126X_LORA_BW_41  = 0x0a,
   SX126X_LORA_BW_62  = 0x03,
   SX126X_LORA_BW_125 = 0x04,
   SX126X_LORA_BW_250 = 0x05,
   SX126X_LORA_BW_500 = 0x06
 };
 
 /* LoRa Coding Rates */
 
 enum sx126x_lora_cr_e
 {
   SX126X_LORA_CR_4_5 = 0x01,
   SX126X_LORA_CR_4_6 = 0x02,
   SX126X_LORA_CR_4_7 = 0x03,
   SX126X_LORA_CR_4_8 = 0x04
 };
 
 /* CAD Exit modes */
 
 enum sx126x_cad_exit_mode_e
 {
   SX126X_CAD_ONLY = 0x00,
   SX126X_CAD_RX   = 0x01
 };
 
 /* TCXO voltages */
 
 enum sx126x_tcxo_voltage_e
 {
   SX126X_TCXO_1_6V = 0x00,
   SX126X_TCXO_1_7V = 0x01,
   SX126X_TCXO_1_8V = 0x02,
   SX126X_TCXO_2_2V = 0x03,
   SX126X_TCXO_2_4V = 0x04,
   SX126X_TCXO_2_7V = 0x05,
   SX126X_TCXO_3_0V = 0x06,
   SX126X_TCXO_3_3V = 0x07
 };
 
 /* Fallback modes */
 
 enum sx126x_fallback_mode_e
 {
   SX126X_FALLBACK_FS          = 0x40,
   SX126X_FALLBACK_STDBY_XOSC  = 0x30,
   SX126X_FALLBACK_STDBY_RC    = 0x20
 };
 
 /* Regulator modes */
 
 enum sx126x_regulator_mode_e
 {
   SX126X_LDO         = 0x00,
   SX126X_DC_DC_LDO   = 0x01
 };
 
 /* Device */
 
 enum sx126x_device_e
 {
   SX1261 =            0x01,
   SX1262 =            0x00
 };
 
 enum sx126x_gfsk_preamble_detect_e
 {
   SX126X_GFSK_PREAMBLE_DETECT_OFF,
   SX126X_GFSK_PREAMBLE_DETECT_8B,
   SX126X_GFSK_PREAMBLE_DETECT_16B,
   SX126X_GFSK_PREAMBLE_DETECT_24B,
   SX126X_GFSK_PREAMBLE_DETECT_32B
 };
 
 /* Addr comp */
 
 enum sx126x_address_filtering_e
 {
   SX126X_ADDR_FILT_DISABLED,
   SX126X_ADDR_FILT_NODE,
   SX126X_ADDR_FILT_NODE_BROADCAST
 };
 
 /* GFSK CRC types */
 
 enum sx126x_gfsk_crc_type_e
 {
   SX126X_GFSK_CRCTYPE_OFF         = 0x01,
   SX126X_GFSK_CRCTYPE_1_BYTE      = 0x00,
   SX126X_GFSK_CRCTYPE_2_BYTE      = 0x02,
   SX126X_GFSK_CRCTYPE_1_BYTE_INV  = 0x04,
   SX126X_GFSK_CRCTYPE_2_BYTE_INV  = 0x06
 };
 
 /* LoRa mod params */
 
 struct sx126x_modparams_lora_s
 {
   enum sx126x_lora_sf_e spreading_factor;
   enum sx126x_lora_bw_e bandwidth;
   enum sx126x_lora_cr_e coding_rate;
   bool low_datarate_optimization;
 };
 
 /* GFSK mod params */
 
 struct sx126x_modparams_gfsk_s
 {
   uint32_t bitrate;
   enum sx126x_gfsk_pulseshape_e pulseshape;
   enum sx126x_gfsk_bandwidth_e bandwidth;
   uint32_t frequency_deviation;
 };
 
 /* LoRa packet params */
 
 struct sx126x_packetparams_lora_s
 {
   uint16_t preambles;
   bool fixed_length_header;
   uint8_t payload_length;
   bool crc_enable;
   bool invert_iq;
 };
 
 /* GFSK packet params */
 
 struct sx126x_packetparams_gfsk_s
 {
   uint16_t preambles;
   enum sx126x_gfsk_preamble_detect_e preamble_detect;
   uint8_t syncword_length;
   enum sx126x_address_filtering_e address_filtering;
   bool include_packet_size;
   uint8_t packet_length;
   enum sx126x_gfsk_crc_type_e crc_type;
   bool whitening_enable;
 };
 
 /* Packet parameters    *****************************************************/
 
 #define SX126X_PKTPARAM1_GFSK_PREAMBLELEN_PARAM           0 /* Takes 0x0001 to 0xFFFF preamble bits */
 #define SX126X_PKTPARAM1_GFSK_PREAMBLELEN_PARAMS          2
 #define SX126X_PKTPARAM3_GFSK_PREAMBLEDETECTORLEN_PARAM   2 /* Takes SX126X_GFSK_PREABMLE_DETECT_x */
 #define SX126X_PKTPARAM4_GFSK_SYNCWORDLEN_PARAM           3 /* 0x00 to 0x40, 0 to 8 bytes from SX126X_SYNCWORD register */
 #define SX126X_PKTPARAM5_GFSK_ADDRCOMP_PARAM              4 /* Takes SX126X_ADDR_FILT_x */
 #define SX126X_PKTPARAM6_GFSK_PKTTYPE_PARAM               5 /* Packet length unknown? false/true */
 #define SX126X_PKTPARAM7_GFSK_PAYLOADLEN_PARAM            6 /* 0x00 to 0xFF size of tx&rx payload in bytes */
 #define SX126X_PKTPARAM8_GFSK_CRCTYPE_PARAM               7 /* Takes SX126X_GFSK_CRCTYPE_x */
 #define SX126X_PKTPARAM9_GFSK_WHITENING_PARAM             8 /* Enable false/true */
 
 #define SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAM        0 /* 0x0001 to 0xFFFF preamble symbols */
 #define SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAMS       2
 #define SX126X_PKTPARAM3_LORA_HEADERTYPE_PARAM         2 /* Variable/Fixed length false/true */
 #define SX126X_PKTPARAM4_LORA_PAYLOADLEN_PARAM         3 /* 0x00 to 0xFF number of tx and rx bytes */
 #define SX126X_PKTPARAM5_LORA_CRCTYPE_PARAM            4 /* CRC OFF/ON false/true */
 #define SX126X_PKTPARAM6_LORA_INVERTIQ_PARAM           5 /* Standard/Inverted false/true */
 
 /* Modulation parameters ****************************************************/
 
 /* GFSK */
 
 #define SX126X_MODPARAM1_GFSK_BR_PARAM           0 /* Takes br = 32 * Fxtal / bit_rate */
 #define SX126X_MODPARAM1_GFSK_BR_PARAMS          3
 #define SX126X_MODPARAM4_GFSK_PULSESHAPE_PARAM   3 /* Takes SX126X_GFSK_PULSESHAPE_x */
 #define SX126X_MODPARAM5_GFSK_BANDWIDTH_PARAM    4 /* Takes SX126X_GFSK_BANDWIDTH_xHZ */
 #define SX126X_MODPARAM6_GFSK_FDEV_PARAM         5 /* Takes Fdev = (frequency_deviation * 2^25) / Fxtal */
 #define SX126X_MODPARAM6_GFSK_FDEV_PARAMS        3
 
 /* LoRa */
 
 #define SX126X_MODPARAM1_LORA_SF_PARAM                  0 /* Takes SF between SX126X_LORA_SF_MIN and MAX */
 #define SX126X_MODPARAM2_LORA_BW_PARAM                  1 /* Takes SX126X_LORA_BW_x */
 #define SX126X_MODPARAM3_LORA_CR_PARAM                  2 /* Takes SX126X_LORA_CR_x */
 #define SX126X_MODPARAM4_LORA_LOWDATRATE_OPTI_PARAM     3 /* Takes true/false*/
 
 /* Operational modes functions **********************************************/
 
 /* SetSleep */
 
 #define SX126X_SETSLEEP                     0x84        /* Opcode */
 #define SX126X_SETSLEEP_PARAMS              1
 #define SX126X_SETSLEEP_CONF_PARAM          0
 #define SX126X_SETSLEEP_CONF_RTC_SHIFT      0
 #  define SX126X_SETSLEEP_CONF_RTC_DISABLE  (0<<SX126X_SETSLEEP_CONF_RTC_SHIFT)
 #  define SX126X_SETSLEEP_CONF_RTC_ENABLE   (1<<SX126X_SETSLEEP_CONF_RTC_SHIFT)
 #define SX126X_SETSLEEP_CONF_START_SHIFT    2
 #  define SX126X_SETSLEEP_CONF_START_COLD   (0<<SX126X_SETSLEEP_CONF_START_SHIFT)
 #  define SX126X_SETSLEEP_CONF_START_WARM   (1<<SX126X_SETSLEEP_CONF_START_SHIFT)
 
 /* SetStandby */
 
 #define SX126X_SETSTANDBY               0x80                              /* Opcode */
 #define SX126X_SETSTANDBY_PARAMS        1
 #define SX126X_SETSTANDBY_CONF_PARAM    0
 #define SX126X_SETSTANDBY_CONF_SHIFT    0                                 /* Bit 0-1: STDBY mode */
 #  define SX126X_SETSTANDBY_CONF_RC     (0<<SX126X_SETSTANDBY_CONF_SHIFT) 
 #  define SX126X_SETSTANDBY_CONF_XOSC   (1<<SX126X_SETSTANDBY_CONF_SHIFT) 
 
 /* SetFS */
 
 #define SX126X_SETFS                    0xC1        /* Opcode */
 
 /* SetTX */
 
 #define SX126X_SETTX                    0x83        /* Opcode */
 #define SX126X_SETTX_PARAMS             2
 #define SX126X_SETTX_TIMEOUT_PARAM      0
 #define SX126X_SETTX_TIMEOUT_PARAMS     2
 #define SX126X_SETTX_NO_TIMEOUT         0x000000    /* Constant */
 
 /* SetRX */
 
 #define SX126X_SETRX                    0x82        /* Opcode */
 #define SX126X_SETRX_PARAMS             2
 #define SX126X_SETRX_TIMEOUT_PARAM      0
 #define SX126X_SETRX_TIMEOUT_PARAMS     2
 #define SX126X_SETRX_NO_TIMEOUT         0x000000    /* Constant */
 #define SX126X_SETRX_CONTINUOUS         0xFFFFFF    /* Constant */
 
 /* StopTimerOnPreamble */
 
 #define SX126X_STOPTIMERONPREAMBLE          0x9F    /* Opcode */
 #define SX126X_STOPTIMERONPREAMBLE_PARAMS   1
 #define SX126X_STOPTIMERONPREAMBLE_ENABLE   (1<<0)  /* Bit 0 disabled=stop on syncword/header, enabled=stop on preamble */
 
 /* SetRxDutyCycle */
 
 #define SX126X_SETRXDUTYCYCLE                       0x94 /* Opcode */
 #define SX126X_SETRXDUTYCYCLE_PARAMS                6
 #define SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAM        0
 #define SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAMS       3  
 #define SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAM     3
 #define SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAMS    3  
 
 /* SetCAD */
 
 #define SX126X_SETCAD                   0xC5
 
 /* SetTXContinuousWave */
 
 #define SX126X_SETTXCONTINUOUSWAVE      0xD1
 
 /* SetTXInfinitePreamble */
 
 #define SX126X_SETTXINFINITEPREAMBLE    0xD2
 
 /* SetRegulatorMode */
 
 #define SX126X_SETREGULATORMODE             0x96
 #define SX126X_SETREGULATORMODE_PARAMS      1
 #define SX126X_SETREGULATORMODE_PARAM       0
 
 /* Calibrate Function */
 
 #define SX126X_CALIBRATE                    0x89
 #define SX126X_CALIBRATE_RC64K_EN           (1<<0)
 #define SX126X_CALIBRATE_RC13M_EN           (1<<1)
 #define SX126X_CALIBRATE_PLL_EN             (1<<2)
 #define SX126X_CALIBRATE_ADC_PULSE_EN       (1<<3)
 #define SX126X_CALIBRATE_ADC_BULK_N_EN      (1<<4)
 #define SX126X_CALIBRATE_ADC_BULK_P_EN      (1<<5)
 #define SX126X_CALIBRATE_IMAGE_EN           (1<<6)
 
 /* CalibrateImage */
 
 #define SX126X_CALIBRATEIMAGE               0x98
 #define SX126X_CALIBRATEIMAGE_FREQ1_PARAM   0
 #define SX126X_CALIBRATEIMAGE_FREQ2_PARAM   1
 
 /* SetPAConfig */
 
 #define SX126X_SETPACONFIG                      0x95
 #define SX126X_SETPACONFIG_PARMS                4
 #define SX126X_SETPACONFIG_PADUTYCYCLE_PARAM    0
 #define SX126X_SETPACONFIG_HPMAX_PARAM          1
 #define SX126X_SETPACONFIG_DEVICESEL_PARAM      2
 #define SX126X_SETPACONFIG_PALUT_PARAM          3
 
 /* SetRXTXFallbackMode */
 
 #define SX126X_SETRXTXFALLBACKMODE              0x93
 #define SX126X_SETRXTXFALLBACKMODE_PARAMS       1 /* Takes SX126X_FALLBACK_x*/
 
 /* Registers and buffer access **********************************************/
 
 /* WriteRegister Function */
 
 #define SX126X_WRITEREGISTER                    0x0D
 #define SX126X_WRITEREGISTER_PARAMS             2
 #define SX126X_WRITEREGISTER_ADDRESS_PARAM      0
 #define SX126X_WRITEREGISTER_ADDRESS_PARAMS     2
 #define SX126X_WRITEREGISTER_DATA_PARAM         2 /* Data extends, address is auto incremented */
 #define SX126X_WRITEREGISTER_STATUS_RETURN      0 /* Gets returned every byte sent */ 
 
 /* ReadRegister Function */
 
 #define SX126X_READREGISTER                     0x1D
 #define SX126X_READREGISTER_ADDRESS_PARAM       0
 #define SX126X_READREGISTER_ADDRESS_PARAMS      2
 #define SX126X_READREGISTER_STATUS_RETURN       0
 #define SX126X_READREGISTER_STATUS_RETURNS      3
 #define SX126X_READREGISTER_DATA_RETURN         3 /* Data extends, address is auto incremented */
 
 /* WriteBuffer Function */
 
 #define SX126X_WRITEBUFFER                      0x0E
 #define SX126X_WRITEBUFFER_PARAMS_MIN           2
 #define SX126X_WRITEBUFFER_OFFSET_PARAM         0
 #define SX126X_WRITEBUFFER_DATA_PARAM           1 /* Data extends, offset(address) is auto incremented */
 #define SX126X_WRITEBUFFER_STATUS_RETURN        0 /* Gets returned every byte sent */
 
 /* ReadBuffer Function */
 
 #define SX126X_READBUFFER                       0x1E
 #define SX126X_READBUFFER_OFFSET_PARAM          0
 #define SX126X_READBUFFER_STATUS_RETURN         0
 #define SX126X_READBUFFER_STATUS_RETURNS        2
 #define SX126X_READBUFFER_DATA_RETURN           2 /* Data extends, offset(address) is auto incremented */
 
 /* DIO and IRQ Control Functions ********************************************/
 
 /* SetDioIrqParams */
 
 #define SX126X_SETDIOIRQPARAMS                  0x08
 #define SX126X_SETDIOIRQPARAMS_PARAMS           8 /* Takes SX126X_IRQ_x bit masks */
 #define SX126X_SETDIOIRQPARAMS_IRQMASK_PARAM    0
 #define SX126X_SETDIOIRQPARAMS_IRQMASK_PARAMS   2
 #define SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAM   2
 #define SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAMS  2
 #define SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAM   4
 #define SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAMS  2
 #define SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAM   6
 #define SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAMS  2
 
 /* SetDIO2AsRfSwitchCtrl */
 
 #define SX126X_SETDIO2RFSWCTRL                  0x9D
 #define SX126X_SETDIO2RFSWCTRL_PARAMS           1
 #define SX126X_SETDIO2RFSWCTRL_ENABLE_PARAM     0 /* true/false */
 
 /* SetDIO3AsTCXOCtrl */
 
 #define SX126X_SETDIO3TCXOCTRL                  0x97
 #define SX126X_SETDIO3TCXOCTRL_PARAMS           4
 #define SX126X_SETDIO3TCXOCTRL_TCXO_V_PARAM     0 /* Takes SX126X_TCXO_xV */
 #define SX126X_SETDIO3TCXOCTRL_DELAY_PARAM      1 /* time = delay(23:0) * 15.625 uS */
 #define SX126X_SETDIO3TCXOCTRL_DELAY_PARAMS     3
 
 /* GetIrqStatus */
 
 #define SX126X_GETIRQSTATUS                     0x12
 #define SX126X_GETIRQSTATUS_RETURNS             3
 #define SX126X_GETIRQSTATUS_STATUS_RETURN       0
 #define SX126X_GETIRQSTATUS_IRQSTATUS_RETURN    1
 #define SX126X_GETIRQSTATUS_IRQSTATUS_RETURNS   2
 
 /* RF Modulation and Packet-Related Functions *******************************/
 
 /* SetRfFrequency */
 
 #define SX126X_SETRFFREQUENCY                   0x86
 #define SX126X_SETRFFREQUENCY_PARAMS            4
 #define SX126X_SETRFFREQUENCY_RFFREQ_PARAM      0 /* Takes (freq * 2 ^ 25) / xtal */
 #define SX126X_SETRFFREQUENCY_RFFREQ_PARAMS     4
 
 /* SetPacketType */
 
 #define SX126X_SETPACKETTYPE                    0x8A
 #define SX126X_SETPACKETTYPE_PARAMS             1
 #define SX126X_SETPACKETTYPE_PACKETTYPE_PARAM   0       /* Takes SX126X_PACKETTYPE_x */
 
 /* GetPacketType */
 
 #define SX126X_GETPACKETTYPE                    0x11
 #define SX126X_GETPACKETTYPE_RETURNS            2
 #define SX126X_GETPACKETTYPE_STATUS_RETURN      0
 #define SX126X_GETPACKETTYPE_PACKETTYPE_RETURN  1       /* Gives SX126X_PACKETTYPE_x */
 
 /* SetTxParms */
 
 #define SX126X_SETTXPARMS                       0x8E
 #define SX126X_SETTXPARMS_PARAMS                2
 #define SX126X_SETTXPARMS_POWER_PARAM           0
 #define SX126X_SETTXPARMS_RAMPTIME_PARAM        1       /* Takes SX126X_SET_RAMP_xU*/
 
 /* SetModulationParams */
 
 #define SX126X_SETMODULATIONPARAMS              0x8B
 #define SX126X_SETMODULATIONPARAMS_PARAMS       8 /* Takes the corresponding SX126X_MODPARAMx_y */
 #define SX126X_SETMODULATIONPARAMS_PARAM1_PARAM 0
 #define SX126X_SETMODULATIONPARAMS_PARAM2_PARAM 1
 #define SX126X_SETMODULATIONPARAMS_PARAM3_PARAM 2
 #define SX126X_SETMODULATIONPARAMS_PARAM4_PARAM 3
 #define SX126X_SETMODULATIONPARAMS_PARAM5_PARAM 4
 #define SX126X_SETMODULATIONPARAMS_PARAM6_PARAM 5
 #define SX126X_SETMODULATIONPARAMS_PARAM7_PARAM 6
 #define SX126X_SETMODULATIONPARAMS_PARAM8_PARAM 7
 
 /* SetPacketParms */
 
 #define SX126X_SETPACKETPARMS                   0x8C
 #define SX126X_SETPACKETPARMS_PARAMS            9 /* Takes the corresponding SX126X_PKTPARAMx_y */
 #define SX126X_SETPACKETPARMS_PARAM1_PARAM      0
 #define SX126X_SETPACKETPARMS_PARAM2_PARAM      1
 #define SX126X_SETPACKETPARMS_PARAM3_PARAM      2
 #define SX126X_SETPACKETPARMS_PARAM4_PARAM      3
 #define SX126X_SETPACKETPARMS_PARAM5_PARAM      4
 #define SX126X_SETPACKETPARMS_PARAM6_PARAM      5
 #define SX126X_SETPACKETPARMS_PARAM7_PARAM      6
 #define SX126X_SETPACKETPARMS_PARAM8_PARAM      7
 #define SX126X_SETPACKETPARMS_PARAM9_PARAM      8
 
 /* SetCadParams */
 
 #define SX126X_SETCADPARAMS                     0x88
 #define SX126X_SETCADPARAMS_PARAMS              7
 #define SX126X_SETCADPARAMS_CADSYMNUM_PARAM     0 /* Takes SX126X_CAD_ON_x_SYMB */
 #define SX126X_SETCADPARAMS_CADDETPEAK_PARAM    1
 #define SX126X_SETCADPARAMS_CADDETMIN_PARAM     2
 #define SX126X_SETCADPARAMS_CADEXITMODE_PARAM   3 /* Takes SX126x_CAD_x */
 #define SX126X_SETCADPARAMS_CADTIMEOUT_PARAM    4 /* RxTimeout = cadTimeout * 15.625 */
 #define SX126X_SETCADPARAMS_CADTIMEOUI_PARAMS   3
 
 /* SetBufferBaseAddress */
 
 #define SX126X_SETBUFFERBASEADDRESS             0x8F
 #define SX126X_SETBUFFERBASEADDRESS_PARAMS      2
 #define SX126X_SETBUFFERBASEADDRESS_TX_PARAM    0
 #define SX126X_SETBUFFERBASEADDRESS_RX_PARAM    1
 
 /* SetLoRaSymbNumTimeout */
 
 #define SX126X_SETLORASYMBNUMTIMEOUT            0xA0
 #define SX126X_SETLORASYMBNUMTIMEOUT_PARAMS     1
 #define SX126X_SETLORASYMBNUMTIMEOUT_NUM_PARAM  0
 
 /* Communication status information *****************************************/
 
 /* GetStatus */
 
 #define SX126X_CMD_GETSTATUS            0xC0 /* Opcode */
 #define SX126X_STATUS_CMD_SHIFT         (1)
 #define SX126X_STATUS_CMD_MASK          (0b111<<SX126X_STATUS_CMD_SHIFT)
 #define SX126X_STATUS_CHIPMODE_SHIFT    (4)
 #define SX126X_STATUS_CHIPMODE_MASK     (0b111<<SX126X_STATUS_CHIPMODE_SHIFT)
 
 /* GetRSSIInst */
 
 #define SX126X_GETRSSIINST               0x15
 #define SX126X_GETRSSIINST_RETURNS       2
 #define SX126X_GETRSSIINST_STAT_RETURN   0
 #define SX126X_GETRSSIINST_RSSI_RETURN   1
 
 /* Misc *********************************************************************/
 
 /* GetDeviceErrors */
 
 #define SX126X_GETDEVICEERRORS                  0x17
 #define SX126X_GETDEVICEERRORS_RETURNS          3
 #define SX126X_GETDEVICEERRORS_STATUS_RETURN    0
 #define SX126X_GETDEVICEERRORS_OPERROR_RETURN   1
 #define SX126X_GETDEVICEERRORS_OPERROR_RETURNS  2 
 
 /* ClearDeviceErrors */
 
 #define SX126X_CLEARDEVICEERRORS                0x07
 #define SX126X_CLEARDEVICEERRORS_NOPS           2
 
 /****************************************************************************
  * Public Data Types
  ****************************************************************************/
 
 struct sx126x_lower_s
 {
     unsigned int port;
     void (*reset)(void);
 };
 
 /****************************************************************************
  * Public Functions Prototypes
  ****************************************************************************/
 
 void sx126x_register(FAR struct spi_dev_s *spi,
                      FAR const struct sx126x_lower_s *lower);
 
 #endif /* __DRIVERS_WIRELESS_LPWAN_SX126X_SX126X_H */