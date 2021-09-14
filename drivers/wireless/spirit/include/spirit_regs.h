/****************************************************************************
 * drivers/wireless/spirit/include/spirit_regs.h
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_REGS_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_REGS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General_Configuration_Registers */

/* ANA_FUNC_CONF register 1
 *
 *   Read Write
 *   Default value: 0x0c
 *   7:5 NUM_EN_PIPES: Number of enabled pipes (starting from Data Pipe 0).
 *   4:2 GM_CONF[2:0]: Sets the driver gm of the XO at start-up:
 *       GM_CONF2 | GM_CONF1 | GM_CONF0 | GM [mS]
 *       ------------------------------------------
 *           0    |    0     |     0    |  13.2
 *           0    |    0     |     1    |  18.2
 *           0    |    1     |     0    |  21.5
 *           0    |    1     |     1    |  25.6
 *           1    |    0     |     0    |  28.8
 *           1    |    0     |     1    |  33.9
 *           1    |    1     |     0    |  38.5
 *           1    |    1     |     1    |  43.0
 *   1:0 SET_BLD_LVL[1:0]: Sets the Battery Level Detector threshold:
 *       SET_BLD_LVL1 | SET_BLD_LVL0 | Threshold [V]
 *       ------------------------------------------
 *             0      |      0       |     2.7
 *             0      |      1       |     2.5
 *             1      |      0       |     2.3
 *             1      |      1       |     2.1
 */

#define ANA_FUNC_CONF1_BASE             ((uint8_t)0x00)  /* ANA_FUNC_CONF1 Address (R/W) */

#define ANA_FUNC_CONF1_NUM_PIPES_MASK   ((uint8_t)0xe0)  /* Mask for number of enabled pipes */
#define ANA_FUNC_CONF1_GMCONF_MASK      ((uint8_t)0x1c)  /* Mask of the GmConf field of
                                                          * ANA_FUNC_CONF1 register (R/W) */
#  define GM_13_2                       ((uint8_t)0x00)  /* Transconducatance Gm at start-up 13.2 mS */
#  define GM_18_2                       ((uint8_t)0x04)  /* Transconducatance Gm at start-up 18.2 mS */
#  define GM_21_5                       ((uint8_t)0x08)  /* Transconducatance Gm at start-up 21.5 mS */
#  define GM_25_6                       ((uint8_t)0x0c)  /* Transconducatance Gm at start-up 25.6 mS */
#  define GM_28_8                       ((uint8_t)0x10)  /* Transconducatance Gm at start-up 28.8 mS */
#  define GM_33_9                       ((uint8_t)0x14)  /* Transconducatance Gm at start-up 33.9 mS */
#  define GM_38_5                       ((uint8_t)0x18)  /* Transconducatance Gm at start-up 38.5 mS */
#  define GM_43_0                       ((uint8_t)0x1c)  /* Transconducatance Gm at start-up 43.0 mS */
#define ANA_FUNC_CONF1_SET_BLD_LVL_MASK ((uint8_t)0x03)  /* Mask of the SET_BLD_LV field of
                                                          * ANA_FUNC_CONF1 register (R/W) */
#define BLD_LVL_2_7                     ((uint8_t)0x00)  /* Sets the Battery Level Detector threshold
                                                          * to 2.7V */
#define BLD_LVL_2_5                     ((uint8_t)0x01)  /* Sets the Battery Level Detector threshold
                                                          * to 2.5V */
#define BLD_LVL_2_3                     ((uint8_t)0x02)  /* Sets the Battery Level Detector threshold
                                                          * to 2.3V */
#define BLD_LVL_2_1                     ((uint8_t)0x03)  /* Sets the Battery Level Detector threshold
                                                          * to 2.1V */

/* ANA_FUNC_CONF register 0
 *
 *   Read Write
 *   Default value: 0xc0
 *   7 Reserved.
 *   6 24_26_MHz_SELECT: 1 - 26 MHz configuration
 *                       0 - 24 MHz configuration
 *   5 AES_ON:           1 - AES engine enabled
 *                       0 - AES engine disabled
 *   4 EXT_REF:          1 - Reference signal from XIN pin
 *                       0 - Reference signal from XO circuit
 *   3 HIGH_POWER_MODE:  1 - SET_SMPS_LEVEL word will be set to the value to
 *                           PM_TEST register in RX state, while in TX state
 *                           it will be fixed to 111 (which programs the SMPS
 *                           output at max value 1.8V)
 *                       0 - SET_SMPS_LEVEL word will hold the value written
 *                           in the PM_TEST register both in RX and TX state
 *   2 BROWN_OUT:        1 - Brown_Out Detection enabled
 *                       0 - Brown_Out Detection disabled
 *   1 BATTERY_LEVEL:    1 - Battery level detector enabled
 *                       0 - Battery level detector disabled
 *   0 TS:               1 - Enable the "Temperature Sensor" function
 *                       0 - Disable the "Temperature Sensor" function
 */

#define ANA_FUNC_CONF0_BASE             ((uint8_t)0x01)  /* ANA_FUNC_CONF0 Address (R/W) */

#define SELECT_24_26_MHZ_MASK           ((uint8_t)0x40)  /* Configure the RCO if using 26 MHz or
                                                          * 24 MHz master clock/reference signal */
#define AES_MASK                        ((uint8_t)0x20)  /* AES engine on/off */
#define EXT_REF_MASK                    ((uint8_t)0x10)  /* Reference signal from XIN pin
                                                          * (oscillator external) or from XO
                                                          * circuit (oscillator internal) */
#define HIGH_POWER_MODE_MASK            ((uint8_t)0x08)  /* SET_SMPS_LEVEL word will be set to the
                                                          * value to PM_TEST register in RX state,
                                                          * while in TX state it will be fixed to
                                                          * 111 (which programs the SMPS output at
                                                          * max value, 1.8V) */
#define BROWN_OUT_MASK                  ((uint8_t)0x04)  /* Accurate Brown-Out detection on/off */
#define BATTERY_LEVEL_MASK              ((uint8_t)0x02)  /* Battery level detector circuit on/off */
#define TEMPERATURE_SENSOR_MASK         ((uint8_t)0x01)  /* The Temperature Sensor (available on
                                                          * GPIO0) on/off */

/* ANT_SELECT_CONF register
 *
 *   Read Write
 *   Default value: 0x05
 *
 *   7:5   Reserved.
 *   4   CS_BLANKING: Blank received data if signal is below the CS threshold
 *   3   AS_ENABLE: Enable antenna switching
 *                       1 - Enable
 *                       0 - Disable
 *   2:0 AS_MEAS_TIME[2:0]: Measurement time according to the formula
 *       Tmeas = 24*2^(EchFlt)*2^AS_MEAS_TIME/fxo
 */

#define ANT_SELECT_CONF_BASE            ((uint8_t)0x27)  /* Antenna diversity (works only in
                                                          * static carrier sense  mode) */

#define ANT_SELECT_CS_BLANKING_MASK     ((uint8_t)0x10)  /* CS data blanking on/off */
#define ANT_SELECT_CONF_AS_MASK         ((uint8_t)0x08)  /* Antenna diversity on/off */

/* DEVICE_INFO1[7:0] registers
 *
 *   Default value: 0x01
 *   Read
 *
 *   7:0       PARTNUM[7:0]:   Device part number
 */

#define DEVICE_INFO1_PARTNUM            ((uint8_t)0xf0)  /* Device part number [7:0] */

/* DEVICE_INFO0_Register */

/* DEVICE_INFO0[7:0] registers
 *
 *   Read
 *
 *   7:0       VERSION[7:0]:  Device version number
 */

#define DEVICE_INFO0_VERSION            ((uint8_t)0xf1)  /* Device version [7:0]; (0x55
                                                          * in CUT1.0) */

/* GPIO_Registers */

/* GPIOx_CONF_Registers */

/* GPIOx registers
 *
 *   Read Write
 *   Default value: 0x03
 *   7:3 GPIO_SELECT[4:0]: Specify the I/O signal.
 *       GPIO_SELECT[4:0] |    I/O    |      Signal
 *       ------------------------------------------------
 *               0        |  Output   |  nIRQ
 *               0        |  Input    |  TX command
 *               1        |  Output   |  POR inverted
 *               1        |  Input    |  RX command
 *               2        |  Output   |  Wake-Up timer expiration
 *               2        |  Input    |  TX data for direct modulation
 *               3        |  Output   |  Low Battery Detection
 *               3        |  Input    |  Wake-up from external input
 *               4        |  Output   |  TX clock output
 *               5        |  Output   |  TX state
 *               6        |  Output   |  TX FIFO Almost Empty Flag
 *               7        |  Output   |  TX FIFO ALmost Full Flag
 *               8        |  Output   |  RX data output
 *               9        |  Output   |  RX clock output
 *               10       |  Output   |  RX state
 *               11       |  Output   |  RX FIFO Almost Full Flag
 *               12       |  Output   |  RX FIFO Almost Empty Flag
 *               13       |  Output   |  Antenna switch
 *               14       |  Output   |  Valid preamble detected
 *               15       |  Output   |  Sync word detected
 *               16       |  Output   |  RSSI above threshold
 *               17       |  Output   |  MCU clock
 *               18       |  Output   |  TX or RX mode indicator
 *               19       |  Output   |  VDD
 *               20       |  Output   |  GND
 *               21       |  Output   |  External SMPS enable signal
 *               22-31    |  Not Used |  Not Used
 *   2 Reserved
 *   1:0 GpioMode[1:0]: Specify the mode:
 *        GPIO_MODE1  |  GPIO_MODE0  |               MODE
 *       ------------------------------------------------------------
 *             0      |      0       |    Analog (valid only for GPIO_0)
 *             0      |      1       |    Digital Input
 *             1      |      0       |    Digital Output Low Power
 *             1      |      1       |    Digital Output High Power
 *
 *   Note: The Analog mode is used only for temperature sensor indication.
 *   This is available only on GPIO_0 by setting the TS bit in the
 *   ANA_FUNC_CONF_0_Register.
 */

#define GPIO3_CONF_BASE                 ((uint8_t)0x02)  /* GPIO_3 register address */
#define GPIO2_CONF_BASE                 ((uint8_t)0x03)  /* GPIO_2 register address */
#define GPIO1_CONF_BASE                 ((uint8_t)0x04)  /* GPIO_1 register address */
#define GPIO0_CONF_BASE                 ((uint8_t)0x05)  /* GPIO_0 register address */

#define CONF_GPIO_IN_TX_Command         ((uint8_t)0x00)  /* TX command direct from PIN (rising
                                                          * edge, width min=50ns) */
#define CONF_GPIO_IN_RX_Command         ((uint8_t)0x08)  /* RX command direct from PIN (rising
                                                          * edge, width min=50ns) */
#define CONF_GPIO_IN_TX_Data            ((uint8_t)0x10)  /* TX data input for direct modulation */
#define CONF_GPIO_IN_WKUP_Ext           ((uint8_t)0x18)  /* Wake up from external input */
#define CONF_GPIO_OUT_nIRQ              ((uint8_t)0x00)  /* nIRQ (Interrupt Request, active
                                                          * low), default configuration after POR */
#define CONF_GPIO_OUT_POR_Inv           ((uint8_t)0x08)  /* POR inverted (active low) */
#define CONF_GPIO_OUT_WUT_Exp           ((uint8_t)0x10)  /* Wake-Up Timer expiration:
                                                          * ‘1’ when WUT has expired */
#define CONF_GPIO_OUT_LBD               ((uint8_t)0x18)  /* Low battery detection: ‘1’ when
                                                          * battery is below threshold setting */
#define CONF_GPIO_OUT_TX_Data           ((uint8_t)0x20)  /* TX data internal clock output (TX
                                                          * data are sampled on the rising edge
                                                          * of it) */
#define CONF_GPIO_OUT_TX_State          ((uint8_t)0x28)  /* TX state indication: ‘1’ when
                                                          * Spirit1 is transiting in the TX state */

#define CONF_GPIO_OUT_TX_FIFO_ALMOST_EMPTY ((uint8_t)0x30) /* TX FIFO Almost Empty  Flag */

#define CONF_GPIO_OUT_TX_FIFO_AMOST_FULL ((uint8_t)0x38) /* TX FIFO Almost Full Flag */
#define CONF_GPIO_OUT_RX_Data           ((uint8_t)0x40)  /* RX data output */
#define CONF_GPIO_OUT_RX_Clock          ((uint8_t)0x48)  /* RX clock output (recovered from
                                                          * received data) */
#define CONF_GPIO_OUT_RX_State          ((uint8_t)0x50)  /* RX state indication: ‘1’ when
                                                          * Spirit1 is transiting in the RX state */

#define CONF_GPIO_OUT_RX_FIFO_ALMOST_FULL ((uint8_t)0x58)  /* RX FIFO Almost Full Flag */
#define CONF_GPIO_OUT_RX_FIFO_ALMOST_EMPTY ((uint8_t)0x60) /* RX FIFO Almost Empty  Flag */

#define CONF_GPIO_OUT_Antenna_Switch    ((uint8_t)0x68)  /* Antenna switch used for antenna
                                                          * diversity */
#define CONF_GPIO_OUT_Valid_Preamble    ((uint8_t)0x70)  /* Valid Preamble Detected Flag */
#define CONF_GPIO_OUT_Sync_Detected     ((uint8_t)0x78)  /* Sync WordSync Word Detected Flag */
#define CONF_GPIO_OUT_RSSI_Threshold    ((uint8_t)0x80)  /* CCA Assessment Flag */
#define CONF_GPIO_OUT_MCU_Clock         ((uint8_t)0x88)  /* MCU Clock */
#define CONF_GPIO_OUT_TX_RX_Mode        ((uint8_t)0x90)  /* TX or RX mode indicator (to enable
                                                          * an external range extender) */
#define CONF_GPIO_OUT_VDD               ((uint8_t)0x98)  /* VDD (to emulate an additional GPIO
                                                          * of the MCU, programmable by SPI) */
#define CONF_GPIO_OUT_GND               ((uint8_t)0xa0)  /* GND (to emulate an additional GPIO
                                                          * of the MCU, programmable by SPI) */
#define CONF_GPIO_OUT_SMPS_Ext          ((uint8_t)0xa8)  /* External SMPS enable signal (active
                                                          * high) */
#define CONF_GPIO_MODE_ANALOG           ((uint8_t)0x00)  /* Analog test BUS on GPIO; used only
                                                          * in test mode (except for temperature
                                                          * sensor) */
#define CONF_GPIO_MODE_DIG_IN           ((uint8_t)0x01)  /* Digital Input on GPIO */
#define CONF_GPIO_MODE_DIG_OUTL         ((uint8_t)0x02)  /* Digital Output on GPIO (low current) */
#define CONF_GPIO_MODE_DIG_OUTH         ((uint8_t)0x03)  /* Digital Output on GPIO (high current) */

/* MCU_CK_CONF register
 *
 *   Read Write
 *   Default value: 0x00
 *   7   Reserved.
 *   6:5 CLOCK_TAIL[1:0]: Specifies the number of extra cylces provided
 *                        before entering in STANDBY state.
 *
 *       CLOCK_TAIL1   |   CLOCK_TAIL0    |  Number of Extra Cycles
 *       ------------------------------------------------------------
 *            0        |        0         |           0
 *            0        |        1         |           64
 *            1        |        0         |           256
 *            1        |        1         |           512
 *   4:1 XO_RATIO[3:0]: Specifies the division ratio when XO oscillator
 *                      is the clock source
 *
 *       XO_RATIO[3:0] |   Division Ratio
 *       -----------------------------------
 *               0     |        1
 *               1     |       2/3
 *               2     |       1/2
 *               3     |       1/3
 *               4     |       1/4
 *               5     |       1/6
 *               6     |       1/8
 *               7     |       1/12
 *               8     |       1/16
 *               9     |       1/24
 *               10    |       1/36
 *               11    |       1/48
 *               12    |       1/64
 *               13    |       1/96
 *               14    |       1/128
 *               15    |       1/256
 *   0   RCO_RATIO: Specifies the division ratio when RC oscillator is the
 *                  clock source
 *                0 - Division Ratio equal to 0
 *                1 - Division Ratio equal to 1/128
 */

#define MCU_CK_CONF_BASE                ((uint8_t)0x06)  /* MCU Clock Config register address */

#define MCU_CK_ENABLE                   ((uint8_t)0x80)  /* MCU clock enable bit */
#define MCU_CK_CONF_CLOCK_TAIL_0        ((uint8_t)0x00)  /* 0 extra clock cycles provided to the
                                                          * MCU before switching to STANDBY state */
#define MCU_CK_CONF_CLOCK_TAIL_64       ((uint8_t)0x20)  /* 64 extra clock cycles provided to the
                                                          * MCU before switching to STANDBY state */
#define MCU_CK_CONF_CLOCK_TAIL_256      ((uint8_t)0x40)  /* 256 extra clock cycles provided to the
                                                          * MCU before switching to STANDBY state */
#define MCU_CK_CONF_CLOCK_TAIL_512      ((uint8_t)0x60)  /* 512 extra clock cycles provided to the
                                                          * MCU before switching to STANDBY state */
#define MCU_CK_CONF_XO_RATIO_1          ((uint8_t)0x00)  /* XO Clock signal available on the GPIO
                                                          * divided by 1 */
#define MCU_CK_CONF_XO_RATIO_2_3        ((uint8_t)0x02)  /* XO Clock signal available on the GPIO
                                                          * divided by 2/3 */
#define MCU_CK_CONF_XO_RATIO_1_2        ((uint8_t)0x04)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/2 */
#define MCU_CK_CONF_XO_RATIO_1_3        ((uint8_t)0x06)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/3 */
#define MCU_CK_CONF_XO_RATIO_1_4        ((uint8_t)0x08)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/4 */
#define MCU_CK_CONF_XO_RATIO_1_6        ((uint8_t)0x0a)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/6 */
#define MCU_CK_CONF_XO_RATIO_1_8        ((uint8_t)0x0c)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/8 */
#define MCU_CK_CONF_XO_RATIO_1_12       ((uint8_t)0x0e)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/12 */
#define MCU_CK_CONF_XO_RATIO_1_16       ((uint8_t)0x10)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/16 */
#define MCU_CK_CONF_XO_RATIO_1_24       ((uint8_t)0x12)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/24 */
#define MCU_CK_CONF_XO_RATIO_1_36       ((uint8_t)0x14)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/36 */
#define MCU_CK_CONF_XO_RATIO_1_48       ((uint8_t)0x16)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/48 */
#define MCU_CK_CONF_XO_RATIO_1_64       ((uint8_t)0x18)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/64 */
#define MCU_CK_CONF_XO_RATIO_1_96       ((uint8_t)0x1a)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/96 */
#define MCU_CK_CONF_XO_RATIO_1_128      ((uint8_t)0x1c)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/128 */
#define MCU_CK_CONF_XO_RATIO_1_192      ((uint8_t)0x1e)  /* XO Clock signal available on the GPIO
                                                          * divided by 1/196 */
#define MCU_CK_CONF_RCO_RATIO_1         ((uint8_t)0x00)  /* RCO Clock signal available on the GPIO
                                                          * divided by 1 */
#define MCU_CK_CONF_RCO_RATIO_1_128     ((uint8_t)0x01)  /* RCO Clock signal available on the GPIO
                                                          * divided by 1/128 */

/* Radio_Configuration_Registers */

/* SYNT3 register
 *
 *   Read Write
 *   Default value: 0x0c
 *
 *   7:5 WCP[2:0]: Set the charge pump current according to the VCO
 *                 frequency in RX mode.
 *
 *    VCO Frequency |  WCP2 |  WCP1 |  WCP0 |  Charge Pump Current (uA)
 *    -----------------------------------------------------------------
 *      4644-4678   |   0   |   0   |   0   |             378.4
 *      4708-4772   |   0   |   0   |   1   |             368.9
 *      4772-4836   |   0   |   1   |   0   |             359.5
 *      4836-4902   |   0   |   1   |   1   |             350
 *      4902-4966   |   1   |   0   |   0   |             340.5
 *      4966-5030   |   1   |   0   |   1   |             331.1
 *      5030-5095   |   1   |   1   |   0   |             321.6
 *      5095-5161   |   1   |   1   |   1   |             312.2
 *      5161-5232   |   0   |   0   |   0   |             378.4
 *      5232-5303   |   0   |   0   |   1   |             368.9
 *      5303-5375   |   0   |   1   |   0   |             359.5
 *      5375-5448   |   0   |   1   |   1   |             350
 *      5448-5519   |   1   |   0   |   0   |             340.5
 *      5519-5592   |   1   |   0   |   1   |             331.1
 *      5592-5663   |   1   |   1   |   0   |             321.6
 *      5663-5736   |   1   |   1   |   1   |             312.2
 *
 *
 *   4:0  SYNT[25:21]: highest 5 bits of the PLL programmable divider
 *                     The valid range depends on fXO and REFDIV settings;
 *                     for fXO=26MHz
 *                     REFDIV = 0 - SYNT[25:21] = 11...13
 *                     REFDIV = 1 - SYNT[25:21] = 22…27
 */

#define SYNT3_BASE                      ((uint8_t)0x08)  /* [4:0] -> SYNT[25:21], highest 5
                                                          * bits of the PLL programmable divider */

#define WCP_CONF_WCP_378UA              ((uint8_t)0x00)  /* Charge pump current nominal value =
                                                          * 378uA [VCO 4644-4708]&[VCO 5161-5232] */
#define WCP_CONF_WCP_369UA              ((uint8_t)0x01)  /* Charge pump current nominal value =
                                                          * 369uA [VCO 4708-4772]&[VCO 5232-5303] */
#define WCP_CONF_WCP_359UA              ((uint8_t)0x02)  /* Charge pump current nominal value =
                                                          * 359uA [VCO 4772-4836]&[VCO 5303-5375] */
#define WCP_CONF_WCP_350UA              ((uint8_t)0x03)  /* Charge pump current nominal value =
                                                          * 350uA [VCO 4836-4902]&[VCO 5375-5448] */
#define WCP_CONF_WCP_340UA              ((uint8_t)0x04)  /* Charge pump current nominal value =
                                                          * 340uA [VCO 4902-4966]&[VCO 5448-5519] */
#define WCP_CONF_WCP_331UA              ((uint8_t)0x05)  /* Charge pump current nominal value =
                                                          * 331uA [VCO 4966-5030]&[VCO 5519-5592] */
#define WCP_CONF_WCP_321UA              ((uint8_t)0x06)  /* Charge pump current nominal value =
                                                          * 321uA [VCO 5030-5095]&[VCO 5592-5563] */
#define WCP_CONF_WCP_312UA              ((uint8_t)0x07)  /* Charge pump current nominal value =
                                                          * 312uA [VCO 5095-5160]&[VCO 5563-5736] */

/* SYNT2 register
 *
 *   Read Write
 *   Default value: 0x84
 *   7:0  SYNT[20:13]: intermediate bits of the PLL programmable divider.
 *
 */

#define SYNT2_BASE                      ((uint8_t)0x09)  /* SYNT[20:13], intermediate bits of
                                                          * the PLL programmable divider */

/* SYNT1 register
 *
 *   Read Write
 *   Default value: 0xec
 *   7:0  SYNT[12:5]: intermediate bits of the PLL programmable divider.
 *
 */

#define SYNT1_BASE                      ((uint8_t)0x0a)  /* SYNT[12:5], intermediate bits of
                                                          * the PLL programmable divider */

/* SYNT0 register
 *
 *   Read Write
 *   Default value: 0x51
 *   7:3  SYNT[4:0]: lowest bits of the PLL programmable divider.
 *   2:0  BS[2:0]:   Synthesizer band select. This parameter selects the
 *                   out-of-loop divide factor of the synthesizer according
 *                   to the formula fxo/(B/2)/D*SYNT/2^18
 *
 *              BS2        |      BS1      |      BS0       |    value of B
 *       --------------------------------------------------------------------
 *               0         |       0       |       1        |       6
 *               0         |       1       |       0        |       8
 *               0         |       1       |       1        |       12
 *               1         |       0       |       0        |       16
 *               1         |       0       |       1        |       32
 *
 */

#define SYNT0_BASE                      ((uint8_t)0x0b)  /* [7:3] -> SYNT[4:0], lowest bits of
                                                          * the PLL programmable divider */

#define SYNT0_BS_6                      ((uint8_t)0x01)  /* Synthesizer band select (out-of-loop
                                                          * divide factor of the synthesizer)=6
                                                          * (779-956MHz) */
#define SYNT0_BS_8                      ((uint8_t)0x02)  /* Synthesizer band select (out-of-loop
                                                          * divide factor of the synthesizer)=8
                                                          * (387-470MHz) */
#define SYNT0_BS_12                     ((uint8_t)0x03)  /* Synthesizer band select (out-of-loop
                                                          * divide factor of the synthesizer)=12
                                                          * (387-470MHz) */
#define SYNT0_BS_16                     ((uint8_t)0x04)  /* Synthesizer band select (out-of-loop
                                                          * divide factor of the synthesizer)=16
                                                          * (300-348MHz) */
#define SYNT0_BS_32                     ((uint8_t)0x05)  /* Synthesizer band select (out-of-loop
                                                          * divide factor of the synthesizer)=32
                                                          * (150-174MHz) */

/* CHSPACE register
 *
 *   Read Write
 *   Default value: 0xfc
 *   7:0  CH_SPACING[7:0]: Channel spacing. From ~793Hz to ~200KHz in 793Hz
 *                         steps (in general, frequency step is
 *                         fXO/215=26MHz/215~793Hz).
 */

#define CHSPACE_BASE                    ((uint8_t)0x0c)  /* Channel spacing. From ~0.8KHz to
                                                          * ~200KHz in (fXO/2^15)Hz (793Hz for
                                                          * 26MHz XO) steps */

/* IF_OFFSET_DIG register
 *
 *   Read Write
 *   Default value: 0xa3
 *
 *   7:0  IF_OFFSET_DIG[7:0]: Intermediate frequency setting for the digital
 *        shift-to-baseband circuits. According to the formula:
 *        fIF = fXO*(IF_OFFSET_ANA+64)/(12*2^10) = fCLK*(IF_OFFSET_DIG+64)/
 *                                                 (12*2^10) Hz.
 */

#define IF_OFFSET_DIG_BASE              ((uint8_t)0x0d)  /* Intermediate frequency fIF=
                                                          * fXO*(IF_OFFSET_ANA+64)/(12*2^10)=
                                                          * fCLK*(IF_OFFSET_DIG+64)/(12*2^10) Hz */

/* IF_OFFSET_ANA register
 *
 *   Read Write
 *   Default value: 0xa3
 *
 *   7:0  IF_OFFSET_ANA[7:0]: Intermediate frequency setting for the digital
 *        shift-to-baseband circuits. According to the formula:
 *        fIF = fXO*(IF_OFFSET_ANA+64)/(12*2^10) = fCLK*(IF_OFFSET_DIG+64)/
 *                                                    (12*2^10) Hz.
 */

#define IF_OFFSET_ANA_BASE              ((uint8_t)0x07)  /* Intermediate frequency fIF=
                                                          * fXO*(IF_OFFSET_ANA+64)/(12*2^10)=
                                                          * fCLK*(IF_OFFSET_DIG+64)/(12*2^10) Hz */

/* FC_OFFSET1 registers
 *
 *   Read Write
 *   Default value: 0xa3
 *
 *   7:4  Reserved.
 *   3:0  FC_OFFSET[11:8]: Carrier offset. This value is the higher part of a
 *                         12-bit 2’s complement integer representing an
 *                         offset in 99Hz(2) units added/subtracted to the
 *                         carrier frequency set by registers SYNT3…SYNT0.
 *                         This register can be used to set a fixed
 *                         correction value obtained e.g. from crystal
 *                         measurements.
 */

#define FC_OFFSET1_BASE                 ((uint8_t)0x0e)  /* [3:0] -> [11:8] Carrier offset
                                                          * (upper part) */

/* FC_OFFSET0 registers
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0  FC_OFFSET[7:0]: Carrier offset. This value is the lower part of a
 *                        12-bit 2’s complement integer representing an
 *                        offset in 99Hz(2) units added/subtracted to the
 *                        carrier frequency set by registers SYNT3…SYNT0.
 *                        This register can be used to set a fixed correction
 *                        value obtained e.g. from crystal measurements.
 */

#define FC_OFFSET0_BASE                 ((uint8_t)0x0f)  /* [7:0] -> [7:0] Carrier offset
                                                          * (lower part). This value is a 12-bit
                                                          * 2’s complement integer representing
                                                          * an offset in fXO/2^18 (99Hz for 26
                                                          * MHz XO) units added/subtracted to
                                                          * the carrier frequency set by registers
                                                          * SYNT3…SYNT0. Range is +/-200kHz with 26
                                                          * MHz XO */

/* PA_LEVEL_x_Registers */

/* PA_POWER_x[8:1] registers
 *
 *   Default values from 8 to 1:
 *   [0x03, 0x0e, 0x1a, 0x25, 0x35, 0x40, 0x4e, 0x00]
 *   Read Write
 *
 *   7    Reserved.
 *   6:0  PA_LEVEL_(x-1)[6:0]: Output power level for x-th slot.
 */

#define PA_POWER8_BASE                  ((uint8_t)0x10)  /* PA Power level for 8th slot of PA
                                                          * ramping or ASK modulation */
#define PA_POWER7_BASE                  ((uint8_t)0x11)  /* PA Power level for 7th slot of PA
                                                          * ramping or ASK modulation */
#define PA_POWER6_BASE                  ((uint8_t)0x12)  /* PA Power level for 6th slot of PA
                                                          * ramping or ASK modulation */
#define PA_POWER5_BASE                   ((uint8_t)0x13) /* PA Power level for 5th slot of PA
                                                          * ramping or ASK modulation */
#define PA_POWER4_BASE                  ((uint8_t)0x14)  /* PA Power level for 4th slot of PA
                                                          * ramping or ASK modulation */
#define PA_POWER3_BASE                  ((uint8_t)0x15)  /* PA Power level for 3rd slot of PA
                                                          * ramping or ASK modulation */
#define PA_POWER2_BASE                  ((uint8_t)0x16)  /* PA Power level for 2nd slot of PA
                                                          * ramping or ASK modulation */
#define PA_POWER1_BASE                  ((uint8_t)0x17)  /* PA Power level for 1st slot of PA
                                                          * ramping or ASK modulation */

/* PA_POWER_CONF_Registers
 *
 *   Default value:0x07
 *   Read Write
 *
 *   7:6  CWC[1:0]: Output stage additional load capacitors bank
 *                 (to be used to optimize the PA for different sub-bands).
 *
 *         CWC1       |        CWC0      |     Total capacity in pF
 *       ---------------------------------------------------------
 *          0         |         0        |        0
 *          0         |         1        |        1.2
 *          1         |         0        |        2.4
 *          1         |         1        |        3.6
 *
 *   5   PA_RAMP_ENABLE:
 *                       1 - Enable the power ramping
 *                       0 - Disable the power ramping
 *   4:3 PA_RAMP_STEP_WIDTH[1:0]: Step width in bit period
 *
 *         PA_RAMP_STEP_WIDTH1  | PA_RAMP_STEP_WIDTH0 | PA ramping time step
 *       -------------------------------------------------------------------
 *                 0            |        0            |  1/8 Bit period
 *                 0            |        1            |  2/8 Bit period
 *                 1            |        0            |  3/8 Bit period
 *                 1            |        1            |  4/8 Bit period
 *
 *   2:0 PA_LEVEL_MAX_INDEX[2:0]: Fixes the MAX PA LEVEL in PA ramping or
 *                                ASK modulation
 */

#define PA_POWER0_BASE                  ((uint8_t)0x18)  /* PA ramping settings and additional
                                                          * load capacitor banks used for PA
                                                          * optimization in different sub bands */

#define PA_POWER0_CWC_MASK              ((uint8_t)0x20)  /* Output stage additional load
                                                          * capacitors  bank */
#define PA_POWER0_CWC_0                 ((uint8_t)0x00)  /* No additional PA load capacitor */
#define PA_POWER0_CWC_1_2P              ((uint8_t)0x40)  /* 1.2pF additional PA load capacitor */
#define PA_POWER0_CWC_2_4P              ((uint8_t)0x80)  /* 2.4pF additional PA load capacitor */
#define PA_POWER0_CWC_3_6P              ((uint8_t)0xc0)  /* 3.6pF additional PA load capacitor */
#define PA_POWER0_PA_RAMP_MASK          ((uint8_t)0x20)  /* The PA power ramping */

#define PA_POWER0_PA_RAMP_STEP_WIDTH_MASK ((uint8_t)0x20)  /* The step width */
#define PA_POWER0_PA_RAMP_STEP_WIDTH_TB_8 ((uint8_t)0x00)  /* PA ramping time step = 1/8 Bit period */
#define PA_POWER0_PA_RAMP_STEP_WIDTH_TB_4 ((uint8_t)0x08)  /* PA ramping time step = 2/8 Bit period */
#define PA_POWER0_PA_RAMP_STEP_WIDTH_3TB_8 ((uint8_t)0x10) /* PA ramping time step = 3/8 Bit period */
#define PA_POWER0_PA_RAMP_STEP_WIDTH_TB_2 ((uint8_t)0x18)  /* PA ramping time  step = 4/8 Bit period */

#define PA_POWER0_PA_LEVEL_MAX_INDEX    ((uint8_t)0x20)  /* Final level for power ramping */
#define PA_POWER0_PA_LEVEL_MAX_INDEX_0  ((uint8_t)0x00)  /* */
#define PA_POWER0_PA_LEVEL_MAX_INDEX_1  ((uint8_t)0x01)  /* Fixes the MAX PA LEVEL in PA ramping or
                                                          * ASK modulation */

#define PA_POWER0_PA_LEVEL_MAX_INDEX_2  ((uint8_t)0x02) /*                   */
#define PA_POWER0_PA_LEVEL_MAX_INDEX_3  ((uint8_t)0x03) /*         _________ */
#define PA_POWER0_PA_LEVEL_MAX_INDEX_4  ((uint8_t)0x04) /* PA_LVL2 _| <--|   */
#define PA_POWER0_PA_LEVEL_MAX_INDEX_5  ((uint8_t)0x05) /*         _|    |   */
#define PA_POWER0_PA_LEVEL_MAX_INDEX_6  ((uint8_t)0x06) /* PA_LVL1 _|    |   */
#define PA_POWER0_PA_LEVEL_MAX_INDEX_7  ((uint8_t)0x07) /* PA_LVL0 _|    MAX_INDEX- */

/* MOD1 register
 *
 *   Read Write
 *   Default value: 0x83
 *   7:0  DATARATE_M[7:0]: The Mantissa of the specified data rate
 */

#define MOD1_BASE                       ((uint8_t)0x1a)  /* The Mantissa of the specified data rate */

/* MOD0 register
 *
 *   Read Write
 *   Default value: 0x1a
 *   7  CW:              1 - CW Mode enabled - enables the generation of a
 *                           continuous wave carrier without any modulation
 *                       0 - CW Mode disabled
 *
 *   6 BT_SEL:  Select BT value for GFSK
 *                       1 - BT=0.5
 *                       0 - BT=1
 *
 *   5:4 MOD_TYPE[1:0]: Modulation type
 *
 *
 *         MOD_TYPE1       |        MOD_TYPE0      |     Modulation
 *       ---------------------------------------------------------
 *             0           |           0           |        2-FSK,MSK
 *             0           |           1           |        GFSK,GMSK
 *             1           |           0           |        ASK/OOK
 *
 *   3:0  DATARATE_E[3:0]: The Exponent of the specified data rate
 */

#define MOD0_BASE                       ((uint8_t)0x1b)  /* Modulation Settings, Exponent of the
                                                          * specified data rate, CW mode */

#define MOD0_MOD_TYPE_2_FSK             ((uint8_t)0x00)  /* Modulation type 2-FSK (MSK if the
                                                          * frequency deviation is identical to a
                                                          * quarter of the data rate) */
#define MOD0_MOD_TYPE_GFSK              ((uint8_t)0x10)  /* Modulation type GFSK (GMSK if the
                                                          * frequency deviation is identical to a
                                                          * quarter of the data rate) */
#define MOD0_MOD_TYPE_ASK               ((uint8_t)0x20)  /* Modulation type ASK (OOK the PA is
                                                          * switched off for symbol "0") */
#define MOD0_MOD_TYPE_MSK               ((uint8_t)0x00)  /* Modulation type MSK (the frequency
                                                          * deviation must be identical to a
                                                          * quarter of the data rate) */
#define MOD0_MOD_TYPE_GMSK              ((uint8_t)0x10)  /* Modulation type GMSK (the frequency
                                                          * deviation must be identical to a
                                                          * quarter of the data rate) */
#define MOD0_BT_SEL_BT_MASK             ((uint8_t)0x00)  /* Select the BT = 1 or BT = 0.5
                                                          * valid only for GFSK or GMSK
                                                          * modulation */
#define MOD0_CW                         ((uint8_t)0x80)  /* Set the Continuous Wave (no
                                                          * modulation) transmit mode */

/* FDEV0 register
 *
 *   Read Write
 *   Default value: 0x45
 *   7:4  FDEV_E[3:0]:  Exponent of the frequency deviation
 *                      (allowed values from 0 to 9)
 *
 *   3 CLOCK_REC_ALGO_SEL:  Select PLL or DLL mode for clock recovery
 *                       1 - DLL mode
 *                       0 - PLL mode
 *
 *   2:0 FDEV_M[1:0]: Mantissa of the frequency deviation
 *                    (allowed values from 0 to 7)
 */

#define FDEV0_BASE                      ((uint8_t)0x1c)  /* Sets the Mantissa and exponent of
                                                          * frequency deviation (frequency
                                                          * separation/2) and PLL or DLL
                                                          * algorithm from clock
                                                          * recovery in RX digital demod */

#define FDEV0_CLOCK_REG_ALGO_SEL_MASK   ((uint8_t)0x08)  /* Can be DLL or PLL algorithm for
                                                          * clock recovery in RX digital demod
                                                          * (see CLOCKREC reg) */
#define FDEV0_CLOCK_REG_ALGO_SEL_PLL    ((uint8_t)0x00)  /* Sets PLL algorithm for clock
                                                          * recovery in RX digital demod (see
                                                          * CLOCKREC reg) */
#define FDEV0_CLOCK_REG_ALGO_SEL_DLL    ((uint8_t)0x08)  /* Sets DLL algorithm for clock
                                                          * recovery in RX digital demod (see
                                                          * CLOCKREC reg) */

/* CHFLT register
 *
 *   Read Write
 *   Default value: 0x23
 *   7:4  CHFLT_M[3:0]:       Mantissa of the channel filter BW
 *                            (allowed values from 0 to 8)
 *
 *   3:0  CHFLT_E[3:0]:       Exponent of the channel filter BW
 *                            (allowed values from 0 to 9)
 *
 *         M\E |   0  |   1  |   2  |   3  |   4 |   5 |  6  |  7 |  8 |  9 |
 *        -----+------+------+------+------+-----+-----+-----+----+----+----+
 *         0   | 800.1| 450.9| 224.7| 112.3| 56.1| 28.0| 14.0| 7.0| 3.5| 1.8|
 *         1   | 795.1| 425.9| 212.4| 106.2| 53.0| 26.5| 13.3| 6.6| 3.3| 1.7|
 *         2   | 768.4| 403.2| 201.1| 100.5| 50.2| 25.1| 12.6| 6.3| 3.1| 1.6|
 *         3   | 736.8| 380.8| 190.0| 95.0 | 47.4| 23.7| 11.9| 5.9| 3.0| 1.5|
 *         4   | 705.1| 362.1| 180.7| 90.3 | 45.1| 22.6| 11.3| 5.6| 2.8| 1.4|
 *         5   | 670.9| 341.7| 170.6| 85.3 | 42.6| 21.3| 10.6| 5.3| 2.7| 1.3|
 *         6   | 642.3| 325.4| 162.4| 81.2 | 40.6| 20.3| 10.1| 5.1| 2.5| 1.3|
 *         7   | 586.7| 294.5| 147.1| 73.5 | 36.7| 18.4|  9.2| 4.6| 2.3| 1.2|
 *         8   | 541.4| 270.3| 135.0| 67.5 | 33.7| 16.9|  8.4| 4.2| 2.1| 1.1|
 */

#define CHFLT_BASE                      ((uint8_t)0x1d)  /* RX Channel Filter Bandwidth */

#define CHFLT_800_1                     ((uint8_t)0x00)  /* RX Channel Filter Bandwidth = 800.1 kHz */
#define CHFLT_795_1                     ((uint8_t)0x10)  /* RX Channel Filter Bandwidth = 795.1 kHz */
#define CHFLT_768_4                     ((uint8_t)0x20)  /* RX Channel Filter Bandwidth = 768.4 kHz */
#define CHFLT_736_8                     ((uint8_t)0x30)  /* RX Channel Filter Bandwidth = 736.8 kHz */
#define CHFLT_705_1                     ((uint8_t)0x40)  /* RX Channel Filter Bandwidth = 705.1 kHz */
#define CHFLT_670_9                     ((uint8_t)0x50)  /* RX Channel Filter Bandwidth = 670.9 kHz */
#define CHFLT_642_3                     ((uint8_t)0x60)  /* RX Channel Filter Bandwidth = 642.3 kHz */
#define CHFLT_586_7                     ((uint8_t)0x70)  /* RX Channel Filter Bandwidth = 586.7 kHz */
#define CHFLT_541_4                     ((uint8_t)0x80)  /* RX Channel Filter Bandwidth = 541.4 kHz */
#define CHFLT_450_9                     ((uint8_t)0x01)  /* RX Channel Filter Bandwidth = 450.9 kHz */
#define CHFLT_425_9                     ((uint8_t)0x11)  /* RX Channel Filter Bandwidth = 425.9 kHz */
#define CHFLT_403_2                     ((uint8_t)0x21)  /* RX Channel Filter Bandwidth = 403.2 kHz */
#define CHFLT_380_8                     ((uint8_t)0x31)  /* RX Channel Filter Bandwidth = 380.8 kHz */
#define CHFLT_362_1                     ((uint8_t)0x41)  /* RX Channel Filter Bandwidth = 362.1 kHz */
#define CHFLT_341_7                     ((uint8_t)0x51)  /* RX Channel Filter Bandwidth = 341.7 kHz */
#define CHFLT_325_4                     ((uint8_t)0x61)  /* RX Channel Filter Bandwidth = 325.4 kHz */
#define CHFLT_294_5                     ((uint8_t)0x71)  /* RX Channel Filter Bandwidth = 294.5 kHz */
#define CHFLT_270_3                     ((uint8_t)0x81)  /* RX Channel Filter Bandwidth = 270.3 kHz */
#define CHFLT_224_7                     ((uint8_t)0x02)  /* RX Channel Filter Bandwidth = 224.7 kHz */
#define CHFLT_212_4                     ((uint8_t)0x12)  /* RX Channel Filter Bandwidth = 212.4 kHz */
#define CHFLT_201_1                     ((uint8_t)0x22)  /* RX Channel Filter Bandwidth = 201.1 kHz */
#define CHFLT_190                       ((uint8_t)0x32)  /* RX Channel Filter Bandwidth = 190.0 kHz */
#define CHFLT_180_7                     ((uint8_t)0x42)  /* RX Channel Filter Bandwidth = 180.7 kHz */
#define CHFLT_170_6                     ((uint8_t)0x52)  /* RX Channel Filter Bandwidth = 170.6 kHz */
#define CHFLT_162_4                     ((uint8_t)0x62)  /* RX Channel Filter Bandwidth = 162.4 kHz */
#define CHFLT_147_1                     ((uint8_t)0x72)  /* RX Channel Filter Bandwidth = 147.1 kHz */
#define CHFLT_135                       ((uint8_t)0x82)  /* RX Channel Filter Bandwidth = 135.0 kHz */
#define CHFLT_112_3                     ((uint8_t)0x03)  /* RX Channel Filter Bandwidth = 112.3 kHz */
#define CHFLT_106_2                     ((uint8_t)0x13)  /* RX Channel Filter Bandwidth = 106.2 kHz */
#define CHFLT_100_5                     ((uint8_t)0x23)  /* RX Channel Filter Bandwidth = 100.5 kHz */
#define CHFLT_95                        ((uint8_t)0x33)  /* RX Channel Filter Bandwidth = 95.0 kHz */
#define CHFLT_90_3                      ((uint8_t)0x43)  /* RX Channel Filter Bandwidth = 90.3 kHz */
#define CHFLT_85_3                      ((uint8_t)0x53)  /* RX Channel Filter Bandwidth = 85.3 kHz */
#define CHFLT_81_2                      ((uint8_t)0x63)  /* RX Channel Filter Bandwidth = 81.2 kHz */
#define CHFLT_73_5                      ((uint8_t)0x73)  /* RX Channel Filter Bandwidth = 73.5 kHz */
#define CHFLT_67_5                      ((uint8_t)0x83)  /* RX Channel Filter Bandwidth = 67.5 kHz */
#define CHFLT_56_1                      ((uint8_t)0x04)  /* RX Channel Filter Bandwidth = 56.1 kHz */
#define CHFLT_53                        ((uint8_t)0x14)  /* RX Channel Filter Bandwidth = 53.0 kHz */
#define CHFLT_50_2                      ((uint8_t)0x24)  /* RX Channel Filter Bandwidth = 50.2 kHz */
#define CHFLT_47_4                      ((uint8_t)0x34)  /* RX Channel Filter Bandwidth = 47.4 kHz */
#define CHFLT_45_1                      ((uint8_t)0x44)  /* RX Channel Filter Bandwidth = 45.1 kHz */
#define CHFLT_42_6                      ((uint8_t)0x54)  /* RX Channel Filter Bandwidth = 42.6 kHz */
#define CHFLT_40_6                      ((uint8_t)0x64)  /* RX Channel Filter Bandwidth = 40.6 kHz */
#define CHFLT_36_7                      ((uint8_t)0x74)  /* RX Channel Filter Bandwidth = 36.7 kHz */
#define CHFLT_33_7                      ((uint8_t)0x84)  /* RX Channel Filter Bandwidth = 33.7 kHz */
#define CHFLT_28                        ((uint8_t)0x05)  /* RX Channel Filter Bandwidth = 28.0 kHz */
#define CHFLT_26_5                      ((uint8_t)0x15)  /* RX Channel Filter Bandwidth = 26.5 kHz */
#define CHFLT_25_1                      ((uint8_t)0x25)  /* RX Channel Filter Bandwidth = 25.1 kHz */
#define CHFLT_23_7                      ((uint8_t)0x35)  /* RX Channel Filter Bandwidth = 23.7 kHz */
#define CHFLT_22_6                      ((uint8_t)0x45)  /* RX Channel Filter Bandwidth = 22.6 kHz */
#define CHFLT_21_3                      ((uint8_t)0x55)  /* RX Channel Filter Bandwidth = 21.3 kHz */
#define CHFLT_20_3                      ((uint8_t)0x65)  /* RX Channel Filter Bandwidth = 20.3 kHz */
#define CHFLT_18_4                      ((uint8_t)0x75)  /* RX Channel Filter Bandwidth = 18.4 kHz */
#define CHFLT_16_9                      ((uint8_t)0x85)  /* RX Channel Filter Bandwidth = 16.9 kHz */
#define CHFLT_14                        ((uint8_t)0x06)  /* RX Channel Filter Bandwidth = 14.0 kHz */
#define CHFLT_13_3                      ((uint8_t)0x16)  /* RX Channel Filter Bandwidth = 13.3 kHz */
#define CHFLT_12_6                      ((uint8_t)0x26)  /* RX Channel Filter Bandwidth = 12.6 kHz */
#define CHFLT_11_9                      ((uint8_t)0x36)  /* RX Channel Filter Bandwidth = 11.9 kHz */
#define CHFLT_11_3                      ((uint8_t)0x46)  /* RX Channel Filter Bandwidth = 11.3 kHz */
#define CHFLT_10_6                      ((uint8_t)0x56)  /* RX Channel Filter Bandwidth = 10.6 kHz */
#define CHFLT_10_1                      ((uint8_t)0x66)  /* RX Channel Filter Bandwidth = 10.1 kHz */
#define CHFLT_9_2                       ((uint8_t)0x76)  /* RX Channel Filter Bandwidth = 9.2 kHz */
#define CHFLT_8_4                       ((uint8_t)0x86)  /* RX Channel Filter Bandwidth = 8.4 kHz */
#define CHFLT_7                         ((uint8_t)0x07)  /* RX Channel Filter Bandwidth = 7.0 kHz */
#define CHFLT_6_6                       ((uint8_t)0x17)  /* RX Channel Filter Bandwidth = 6.6 kHz */
#define CHFLT_6_3                       ((uint8_t)0x27)  /* RX Channel Filter Bandwidth = 6.3 kHz */
#define CHFLT_5_9                       ((uint8_t)0x37)  /* RX Channel Filter Bandwidth = 5.9 kHz */
#define CHFLT_5_6                       ((uint8_t)0x47)  /* RX Channel Filter Bandwidth = 5.6 kHz */
#define CHFLT_5_3                       ((uint8_t)0x57)  /* RX Channel Filter Bandwidth = 5.3 kHz */
#define CHFLT_5_1                       ((uint8_t)0x67)  /* RX Channel Filter Bandwidth = 5.1 kHz */
#define CHFLT_4_6                       ((uint8_t)0x77)  /* RX Channel Filter Bandwidth = 4.6 kHz */
#define CHFLT_4_2                       ((uint8_t)0x87)  /* RX Channel Filter Bandwidth = 4.2 kHz */
#define CHFLT_3_5                       ((uint8_t)0x08)  /* RX Channel Filter Bandwidth = 3.5 kHz */
#define CHFLT_3_3                       ((uint8_t)0x18)  /* RX Channel Filter Bandwidth = 3.3 kHz */
#define CHFLT_3_1                       ((uint8_t)0x28)  /* RX Channel Filter Bandwidth = 3.1 kHz */
#define CHFLT_3                         ((uint8_t)0x38)  /* RX Channel Filter Bandwidth = 3.0 kHz */
#define CHFLT_2_8                       ((uint8_t)0x48)  /* RX Channel Filter Bandwidth = 2.8 kHz */
#define CHFLT_2_7                       ((uint8_t)0x58)  /* RX Channel Filter Bandwidth = 2.7 kHz */
#define CHFLT_2_5                       ((uint8_t)0x68)  /* RX Channel Filter Bandwidth = 2.5 kHz */
#define CHFLT_2_3                       ((uint8_t)0x78)  /* RX Channel Filter Bandwidth = 2.3 kHz */
#define CHFLT_2_1                       ((uint8_t)0x88)  /* RX Channel Filter Bandwidth = 2.1 kHz */
#define CHFLT_1_8                       ((uint8_t)0x09)  /* RX Channel Filter Bandwidth = 1.8 kHz */
#define CHFLT_1_7                       ((uint8_t)0x19)  /* RX Channel Filter Bandwidth = 1.7 kHz */
#define CHFLT_1_6                       ((uint8_t)0x29)  /* RX Channel Filter Bandwidth = 1.6 kHz */
#define CHFLT_1_5                       ((uint8_t)0x39)  /* RX Channel Filter Bandwidth = 1.5 kHz */
#define CHFLT_1_4                       ((uint8_t)0x49)  /* RX Channel Filter Bandwidth = 1.4 kHz */
#define CHFLT_1_3a                      ((uint8_t)0x59)  /* RX Channel Filter Bandwidth = 1.3 kHz */
#define CHFLT_1_3                       ((uint8_t)0x69)  /* RX Channel Filter Bandwidth = 1.3 kHz */
#define CHFLT_1_2                       ((uint8_t)0x79)  /* RX Channel Filter Bandwidth = 1.2 kHz */
#define CHFLT_1_1                       ((uint8_t)0x89)  /* RX Channel Filter Bandwidth = 1.1 kHz */

/* AFC2 register
 *
 *   Read Write
 *   Default value: 0x48
 *   7  AFC Freeze on Sync: Freeze AFC correction upon sync word detection.
 *                       1 - AFC Freeze enabled
 *                       0 - AFC Freeze disabled
 *
 *   6  AFC Enabled:  Enable AFC
 *                       1 - AFC enabled
 *                       0 - AFC disabled
 *
 *   5  AFC Mode:    Select AFC mode
 *                       1 - AFC Loop closed on 2nd conversion stage.
 *                       0 - AFC Loop closed on slicer
 *
 *   4:0  AFC PD leakage[4:0]: Peak detector leakage. This parameter sets the
 *        decay speed of the min/max frequency peak detector (AFC2 register),
 *        the range allowed is 0..31 (0 - no leakage, 31 - high leakage).
 *        The recommended value for this parameter is 4.
 */

#define AFC2_BASE                       ((uint8_t)0x1e)  /* Automatic frequency compensation
                                                          * algorithm parameters (FSK/GFSK/MSK) */

#define AFC2_AFC_FREEZE_ON_SYNC_MASK    ((uint8_t)0x80)  /* The frequency correction value is
                                                          * frozen when SYNC word is detected */
#define AFC2_AFC_MASK                   ((uint8_t)0x40)  /* Mask of Automatic Frequency Correction */
#define AFC2_AFC_MODE_MASK              ((uint8_t)0x20)  /* Automatic Frequency Correction can
                                                          * be in Main MODE or Auxiliary MODE */
#define AFC2_AFC_MODE_SLICER            ((uint8_t)0x00)  /* Automatic Frequency Correction Main
                                                          * MODE */
#define AFC2_AFC_MODE_MIXER             ((uint8_t)0x20)  /* Automatic Frequency Correction
                                                          * Auxiliary MODE */

/* AFC1 register
 *
 *   Read Write
 *   Default value: 0x18
 *
 *   7:0  AFC_FAST_PERIOD: Length of the AFC fast period. this parameter sets
 *                         the length of the fast period in number of samples
 *                         (AFC1 register), the range allowed is 0..255. The
 *                         recommended setting for this parameter is such
 *                         that the fast period equals the preamble length.
 *                         Since the algorithm operates typically on 2
 *                         samples per symbol, the programmed value should be
 *                         twice the number of preamble symbols.
 */

#define AFC1_BASE                       ((uint8_t)0x1f)  /* Length of the AFC fast period */

/* AFC0 register
 *
 *   Read Write
 *   Default value: 0x25
 *
 *   7:4  AFC_FAST_GAIN_LOG2[3:0]: AFC loop gain in fast mode (2's log)
 *   3:0  AFC_SLOW_GAIN_LOG2[3:0]: AFC loop gain in slow mode (2's log)
 */

#define AFC0_BASE                       ((uint8_t)0x20) /* AFC loop gain in fast and slow modes
                                                         * (2's log) */

/* CLOCKREC register
 *
 *   Read Write
 *   Default value: 0x58
 *
 *   7:5 CLK_REC_P_GAIN [2:0]: Clock recovery loop gain (log2)
 *   4   PSTFLT_LEN: Set Postfilter length
 *                       1 - 16 symbols
 *                       0 - 8 symbols
 *   3:0 CLK_REC_I_GAIN[3:0]: Integral gain for the clock recovery loop
 */

#define CLOCKREC_BASE                   ((uint8_t)0x23)  /* Gain of clock recovery loop -
                                                          * Postfilter length 0-8 symbols, 1-16
                                                          * symbols */

/* AGCCTRL2 register
 *
 *   Read Write
 *   Default value: 0x22
 *
 *   7   Reserved
 *   6   FREEZE_ON_STEADY: Enable freezing on steady state
 *                       1 - Enable
 *                       0 - Disable
 *   5   FREEZE_ON_SYNC: Enable freezing on sync detection
 *                       1 - Enable
 *                       0 - Disable
 *   4   START_MAX_ATTENUATION: Start with max attenuation
 *                       1 - Enable
 *                       0 - Disable
 *   3:0  MEAS_TIME[3:0]: Measure time during which the signal peak is
 *        detected (according to the formula 12/fxo*2^MEAS_TIME)
 */

#define AGCCTRL2_BASE                   ((uint8_t)0x24)  /* AGC freeze strategy, AGC attenuation
                                                          * strategy, AGC measure time */

#define AGCCTRL2_FREEZE_ON_STEADY_MASK  ((uint8_t)0x40)  /* The attenuation settings will be
                                                          * frozen as soon as signal level is
                                                          * between min and max threshold (see
                                                          * AGCCTRL1) */
#define AGCCTRL2_FREEZE_ON_SYNC_MASK    ((uint8_t)0x20)  /* The attenuation settings will be
                                                          * frozen as soon sync word is detected */
#define AGCCTRL2_START_MAX_ATTENUATION_MASK ((uint8_t)0x10)
                                                         /* The AGC algorithm can start with MAX
                                                          * attenuation or MIN attenuation */

/* AGCCTRL1 register
 *
 *   Read Write
 *   Default value: 0x65
 *
 *   7:4   THRESHOLD_HIGH[3:0]: High threshold for the AGC
 *   3:0   THRESHOLD_LOW[3:0]: Low threshold for the AGC
 */

#define AGCCTRL1_BASE                   ((uint8_t)0x25)  /* Sets low and high threshold for AGC */

/* AGCCTRL0 register
 *
 *   Read Write
 *   Default value: 0x8a
 *
 *   7   AGC S_ENABLE: Enable AGC
 *                       1 - Enable
 *                       0 - Disable
 *   6   AGC_MODE: Set linear-Binary AGC mode
 *                       1 - Enable
 *                       0 - Disable
 *   5:0 HOLD_TIME[5:0]: Hold time after gain adjustment according to formula
 *                          12/fxo*HOLD_TIME
 */

#define AGCCTRL0_BASE                   ((uint8_t)0x26)  /* Enables AGC, set AGC algo between
                                                          * linear/binary mode, set hold time to
                                                          * account signal propagation through
                                                          * RX chain */

#define AGCCTRL0_AGC_MASK               ((uint8_t)0x80)  /* AGC on/off */
#define AGCCTRL0_AGC_MODE_MASK          ((uint8_t)0x40)  /* AGC search correct attenuation in
                                                          * binary mode or sequential mode */
#define AGCCTRL0_AGC_MODE_LINEAR        ((uint8_t)0x00)  /* AGC search correct attenuation in
                                                          * sequential mode (recommended) */
#define AGCCTRL0_AGC_MODE_BINARY        ((uint8_t)0x40)  /* AGC search correct attenuation in
                                                          * binary mode */

/* CHNUM registers
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0     CH_NUM[7:0]: Channel number. This value is multiplied by the
 *                        channel spacing and added to the synthesizer base
 *                        frequency to generate the actual RF carrier
 *                        frequency.
 */

#define CHNUM_BASE                      ((uint8_t)0x6c)  /* Channel number. This value is
                                                          * multiplied by the channel spacing
                                                          * and added to the synthesizer base
                                                          * frequency to generate the actual
                                                          * RF carrier frequency */

/* AFC_CORR registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   AFC_CORR[7:0]: AFC word of the received packet
 */

#define AFC_CORR_BASE                   ((uint8_t)0xc4)  /* AFC word of the received packet */

/* Packet_Configuration_Registers */

/* PCKTCTRL4 register
 *
 *   Read Write
 *   Default value: 0x00
 *
 *   7:5   NOT_USED.
 *   4:3   ADDRESS_LEN[1:0]: length of address field in bytes
 *   2:0   control_len[2:0]: length of control field in bytes
 */

#define PCKTCTRL4_BASE                  ((uint8_t)0x30)  /* lengths of address and control field */

#define PCKTCTRL4_ADDRESS_LEN_MASK      ((uint8_t)0x18)
#define PCKTCTRL4_CONTROL_LEN_MASK      ((uint8_t)0x07)

/* PCKTCTRL3 register
 *
 *   Read Write
 *   Default value: 0x07
 *
 *   7:6   PCKT_FRMT[1:0]: format of packet
 *
 *          PCKT_FRMT1       |        PCKT_FRMT0       |      Format
 *       --------------------------------------------------------------------
 *             0             |           0             |      BASIC
 *             1             |           0             |      MBUS
 *             1             |           1             |      STACK
 *
 *   5:4   RX_MODE[1:0]: length of address 0x30 field in bytes
 *
 *          RX_MODE1       |        RX_MODE0       |           Rx Mode
 *       -------------------------------------------------------------------
 *             0           |           0           |            normal
 *             0           |           1           |      direct through FIFO
 *             1           |           0           |      direct through GPIO
 *
 *   3:0   LEN_WID[3:0]: length of length field in bits
 */

#define PCKTCTRL3_BASE                  ((uint8_t)0x31)  /* Packet format, RX mode, length of
                                                          * length field */

#define PCKTCTRL3_PCKT_FRMT_BASIC       ((uint8_t)0x00)  /* Basic Packet Format */
#define PCKTCTRL3_PCKT_FRMT_MBUS        ((uint8_t)0x80)  /* Wireless M-BUS Packet Format */
#define PCKTCTRL3_PCKT_FRMT_STACK       ((uint8_t)0xc0)  /* STack Packet Format */
#define PCKTCTRL3_RX_MODE_NORMAL        ((uint8_t)0x00)  /* Normal RX Mode */
#define PCKTCTRL3_RX_MODE_DIRECT_FIFO   ((uint8_t)0x10)  /* RX Direct Mode; data available
                                                          * through FIFO */
#define PCKTCTRL3_RX_MODE_DIRECT_GPIO   ((uint8_t)0x20)  /* RX Direct Mode; data available
                                                          * through selected GPIO */
#define PCKTCTRL3_PKT_FRMT_MASK         ((uint8_t)0xc0)
#define PCKTCTRL3_RX_MODE_MASK          ((uint8_t)0x30)
#define PCKTCTRL3_LEN_WID_MASK          ((uint8_t)0x0f)

/* PCKTCTRL2 register
 *
 *   Read Write
 *   Default value: 0x1e
 *
 *   7:3   PREAMBLE_LENGTH[4:0]: length of preamble field in bytes (0..31)
 *   2:1   SYNC_LENGTH[1:0]: length of sync field in bytes
 *   0     FIX_VAR_LEN: fixed/variable packet length
 *                       1 - Variable
 *                       0 - Fixed
 */

#define PCKTCTRL2_BASE                  ((uint8_t)0x32)  /* length of preamble and sync fields
                                                          * (in bytes), fix or variable packet
                                                          * length */

#define PCKTCTRL2_FIX_VAR_LEN_MASK      ((uint8_t)0x01)  /* Enable/disable  the length mode */
#define PCKTCTRL2_PREAMBLE_LENGTH_MASK  ((uint8_t)0xf8)
#define PCKTCTRL2_SYNC_LENGTH_MASK      ((uint8_t)0x06)

/* PCKTCTRL1 register
 *
 *   Read Write
 *   Default value: 0x20
 *
 *   7:5   CRC_MODE[2:0]: CRC type (0, 8, 16, 24 bits)
 *
 *          CRC_MODE2 | CRC_MODE1 | CRC_MODE0 | CRC Mode  (n. bits - poly)
 *       -----------------------------------------------------------------
 *             0      |     0     |     1     |   8 - 0x07
 *             0      |     1     |     0     |  16 -  0x8005
 *             0      |     1     |     1     |  16 - 0x1021
 *             1      |     0     |     0     |  24 - 0x864CBF
 *
 *   4     WHIT_EN[0]: Enable Whitening
 *                       1 - Enable
 *                       0 - Disable
 *
 *   3:2   TX_SOURCE[1:0]: length of sync field in bytes
 *
 *          TX_SOURCE1     |        TX_SOURCE0     |       Tx Mode
 *       -------------------------------------------------------------------
 *             0           |           0           |      normal
 *             0           |           1           |      direct through FIFO
 *             1           |           0           |      direct through GPIO
 *             1           |           1           |      pn9
 *
 *   1   NOT_USED
 *
 *   0   FEC_EN: enable FEC
 *                       1 - FEC in TX , Viterbi decoding in RX
 *                       0 - Disabled
 */

#define PCKTCTRL1_BASE                  ((uint8_t)0x33)  /* CRC type, whitening enable, TX mode */

#define PCKTCTRL1_FEC_MASK              ((uint8_t)0x01)  /* Enable/disable the Forward Error Correction */
#define PCKTCTRL1_TX_SOURCE_MASK        ((uint8_t)0x0c)  /* TX source mode */
#define PCKTCTRL1_CRC_MODE_MASK         ((uint8_t)0xe0)  /* CRC type */
#define PCKTCTRL1_WHIT_MASK             ((uint8_t)0x10)  /* Enable/disable the Whitening */

/* PCKTLEN1 register
 *
 *   Read Write
 *   Default value: 0x00
 *
 *   7:0   pktlen1[7:0]: length of packet in bytes (upper field) LENGTH/256
 */

#define PCKTLEN1_BASE                   ((uint8_t)0x34)  /* length of packet in bytes (upper field) */

/* PCKTLEN0 register
 *
 *   Read Write
 *   Default value: 0x14
 *
 *   7:0   pktlen0[7:0]: length of packet in bytes (lower field) LENGTH%256
 */

#define PCKTLEN0_BASE                   ((uint8_t)0x35)  /* length of packet in bytes (lower field)
                                                          * [PCKTLEN=PCKTLEN1x256+PCKTLEN0] */

/* SYNCx[4:1] Registers
 *
 *   Read Write
 *   Default value: 0x88
 *
 *   7:0   SYNCx[7:0]: xth sync word
 */

#define SYNC4_BASE                      ((uint8_t)0x36)  /* Sync word 4 */
#define SYNC3_BASE                      ((uint8_t)0x37)  /* Sync word 3 */
#define SYNC2_BASE                      ((uint8_t)0x38)  /* Sync word 2 */
#define SYNC1_BASE                      ((uint8_t)0x39)  /* Sync word 1 */

/* MBUS_PRMBL register
 *
 *   Read Write
 *   Default value: 0x20
 *
 *   7:0 MBUS_PRMBL[7:0]: MBUS preamble control
 */

#define MBUS_PRMBL_BASE                 ((uint8_t)0x3b)  /* MBUS preamble length (in 01 bit pairs) */

/* MBUS_PSTMBL register
 *
 *   Read Write
 *   Default value: 0x20
 *
 *   7:0 MBUS_PSTMBL[7:0]: MBUS postamble control
 */

#define MBUS_PSTMBL_BASE                ((uint8_t)0x3c)  /* MBUS postamble length (in 01 bit pairs) */

/* MBUS_CTRL register
 *
 *   Read Write
 *   Default value: 0x00
 *
 *   7:4   NOT_USED
 *   3:1   MBUS_SUBMODE[2:0]: MBUS submode (allowed values are 0,1,3,5)
 *   0     NOT_USED
 */

#define MBUS_CTRL_BASE                  ((uint8_t)0x3d)  /* MBUS sub-modes (S1, S2 short/long
                                                          * header, T1, T2, R2) */

#define MBUS_CTRL_MBUS_SUBMODE_S1_S2L   ((uint8_t)0x00)  /* MBUS sub-modes S1 & S2L, header
                                                          * length min 279, sync 0x7696,
                                                          * Manchester */
#define MBUS_CTRL_MBUS_SUBMODE_S2_S1M_T2_OTHER ((uint8_t)0x02)
                                                         /* MBUS sub-modes S2, S1-m, T2 (only
                                                          * other to meter) short header, header
                                                          * length min 15, sync 0x7696, Manchester */
#define MBUS_CTRL_MBUS_SUBMODE_T1_T2_METER ((uint8_t)0x06)
                                                         /* MBUS sub-modes T1, T2 (only meter
                                                          * to other), header length min 19, sync
                                                          * 0x3d, 3 out of 6 */
#define MBUS_CTRL_MBUS_SUBMODE_R2      ((uint8_t)0x0a)   /* MBUS sub-mode R2, header length min
                                                          * 39, sync 0x7696, Manchester */

/* PCKT_FLT_GOALS_CONTROLx_MASK registers
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0    CONTROLx_MASK[7:0]:   All 0s - no filtering
 */

#define PCKT_FLT_GOALS_CONTROL0_MASK_BASE ((uint8_t)0x42) /* Packet control
                                                           * field #3 mask,
                                                           * all 0s ->
                                                           * no filtering */
#define PCKT_FLT_GOALS_CONTROL1_MASK_BASE ((uint8_t)0x43) /* Packet control
                                                           * field #2 mask,
                                                           * all 0s ->
                                                           * no filtering */
#define PCKT_FLT_GOALS_CONTROL2_MASK_BASE ((uint8_t)0x44) /* Packet control
                                                           * field #1 mask,
                                                           * all 0s ->
                                                           * no filtering */
#define PCKT_FLT_GOALS_CONTROL3_MASK_BASE ((uint8_t)0x45) /* Packet control
                                                           * field #0 mask,
                                                           * all 0s ->
                                                           * no filtering */

/* PCKT_FLT_GOALS_CONTROLx_FIELD registers
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0    CONTROLx_FIELD[7:0]:   Control field (byte x) to be used as
 *                                 reference
 */

#define PCKT_FLT_GOALS_CONTROL0_FIELD_BASE ((uint8_t)0x46)  /* Control field (byte  #3) */
#define PCKT_FLT_GOALS_CONTROL1_FIELD_BASE ((uint8_t)0x47)  /* Control field (byte  #2) */
#define PCKT_FLT_GOALS_CONTROL2_FIELD_BASE ((uint8_t)0x48)  /* Control field (byte  #1) */
#define PCKT_FLT_GOALS_CONTROL3_FIELD_BASE ((uint8_t)0x49)  /* Control field (byte  #0) */

/* PCKT_FLT_GOALS_SOURCE_MASK register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0    RX_SOURCE_MASK[7:0]:   For received packet only:
 *                                 all 0s - no filtering
 */

#define PCKT_FLT_GOALS_SOURCE_MASK_BASE ((uint8_t)0x4a)  /* Source address mask, valid
                                                          * in RX mode */

/* PCKT_FLT_GOALS_SOURCE_ADDR register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0    RX_SOURCE_ADDR[7:0]:  RX packet source / TX packet destination
 *                                fields
 */

#define PCKT_FLT_GOALS_SOURCE_ADDR_BASE ((uint8_t)0x4b)  /* Source address */

/* PCKT_FLT_GOALS_BROADCAST register
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0    BROADCAST[7:0]:  Address shared for broadcast communication link
 */

#define PCKT_FLT_GOALS_BROADCAST_BASE   ((uint8_t)0x4c) /* Address shared for broadcast
                                                         * communication  links */

/* PCKT_FLT_GOALS_MULTICAST register
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0    MULTICAST[7:0]:  Address shared for multicast communication links
 */

#define PCKT_FLT_GOALS_MULTICAST_BASE   ((uint8_t)0x4d)  /* Address shared for multicast
                                                          * communication links */

/* PCKT_FLT_GOALS_TX_SOURCE_ADDR register
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0    TX_SOURCE_ADDR[7:0]:  TX packet source / RX packet destination
 *                                fields
 */

#define PCKT_FLT_GOALS_TX_ADDR_BASE     ((uint8_t)0x4e)  /* Address of the destination (also
                                                          * device own address) */

/* PCKT_FLT_OPTIONS register
 *
 *   Default value: 0x70
 *   Read Write
 *
 *   7   Reserved.
 *   6   RX_TIMEOUT_AND_OR_SELECT[0]: 1 - ‘OR’ logical function applied to
 *                                        CS/SQI/PQI values (masked by 7:5
 *                                        bits in PROTOCOL register)
 *   5   CONTROL_FILTERING[0]:        1 - RX packet accepted if its control
 *                                        fields matches with masked
 *                                        CONTROLx_FIELD registers.
 *   4   SOURCE_FILTERING[0]:         1 - RX packet accepted if its source
 *                                        field matches w/ masked
 *                                        RX_SOURCE_ADDR register.
 *   3   DEST_VS_ SOURCE _ADDR[0]:    1 - RX packet accepted if its
 *                                        destination address matches with
 *                                        TX_SOURCE_ADDR reg.
 *   2   DEST_VS_MULTICAST_ADDR[0]:   1 - RX packet accepted if its
 *                                        destination address matches with
 *                                        MULTICAST register
 *   1   DEST_VS_BROADCAST_ADDR[0]:   1 - RX packet accepted if its
 *                                        destination address matches with
 *                                        BROADCAST register.
 *   0   CRC_CHECK[0]:                1 - packet discarded if CRC not valid.
 */

#define PCKT_FLT_OPTIONS_BASE           ((uint8_t)0x4f)  /* Options relative to packet filtering */

#define PCKT_FLT_OPTIONS_CRC_CHECK_MASK ((uint8_t)0x01)  /* Enable/disable of CRC check: packet
                                                          * is discarded if CRC is not valid [RX] */
#define PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK ((uint8_t)0x02)
                                                         /* Packet discarded if destination address
                                                          * differs from BROADCAST register [RX] */
#define PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK ((uint8_t)0x04)
                                                         /* Packet discarded if destination address
                                                          * differs from MULTICAST register [RX] */
#define PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK ((uint8_t)0x08)
                                                         /* Packet discarded if destination address
                                                          * differs from TX_ADDR register [RX] */
#define PCKT_FLT_OPTIONS_SOURCE_FILTERING_MASK ((uint8_t)0x10)
                                                         /* Packet discarded if source address
                                                          * (masked by the SOURCE_MASK register)
                                                          * differs from SOURCE_ADDR register
                                                          * [RX] */
#define PCKT_FLT_OPTIONS_CONTROL_FILTERING_MASK ((uint8_t)0x20)
                                                         /* Packet discarded if the x-byte
                                                          * (x=1¸4) control field (masked by
                                                          * the CONTROLx_MASK register) differs
                                                          * from CONTROLx_FIELD register [RX] */
#define PCKT_FLT_OPTIONS_RX_TIMEOUT_AND_OR_SELECT ((uint8_t)0x40)
                                                         /* Logical function applied to CS/SQI/PQI
                                                          * values (masked by [7:5] bits i
                                                          * PROTOCOL[2] register) */

/* TX_CTRL_FIELDx registers
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0     TX_CTRLx[7:0]: Control field value to be used in TX packet as
 *                          byte n.x
 */

#define TX_CTRL_FIELD3_BASE             ((uint8_t)0x68)  /* Control field value to be used in
                                                          * TX packet as byte n.3 */
#define TX_CTRL_FIELD2_BASE             ((uint8_t)0x69)  /* Control field value to be used in
                                                          * TX packet as byte n.2 */
#define TX_CTRL_FIELD1_BASE             ((uint8_t)0x6a)  /* Control field value to be used in
                                                          * TX packet as byte n.1 */
#define TX_CTRL_FIELD0_BASE             ((uint8_t)0x6b)  /* Control field value to be used in
                                                          * TX packet as byte n.0 */

/* TX_PCKT_INFO registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:6     Not used.
 *
 *   5:4     TX_SEQ_NUM:   Current TX packet sequence number
 *
 *   0       N_RETX[3:0]:  Number of retransmissions done on the
 *                         last TX packet
 */

#define TX_PCKT_INFO_BASE               ((uint8_t)0xc2)  /* Current TX packet sequence number
                                                          * [5:4]; Number of retransmissions done
                                                          * on the last TX packet [3:0] */

/* RX_PCKT_INFO registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:3     Not used.
 *
 *   2       NACK_RX:   NACK field of the received packet
 *
 *   1:0     RX_SEQ_NUM[1:0]:  Sequence number of the received packet
 */

#define RX_PCKT_INFO_BASE               ((uint8_t)0xc3)  /* NO_ACK field of the received packet
                                                          * [2]; sequence number of the received
                                                          * packet [1:0] */

#define TX_PCKT_INFO_NACK_RX            ((uint8_t)0x04)  /* NACK field of the received packet */

/* RX_PCKT_LEN1 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   RX_PCKT_LEN1[7:0]:  Length (number of bytes) of the received
 *                             packet:
 *                             RX_PCKT_LEN=RX_PCKT_LEN1 × 256 + RX_PCKT_LEN0
 *                             This value is packet_length/256
 */

#define RX_PCKT_LEN1_BASE               ((uint8_t)0xc9)  /* Length (number of bytes) of the
                                                          * received packet */

/* RX_PCKT_LEN0 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   RX_PCKT_LEN0[7:0]:  Length (number of bytes) of the received
 *                             packet:
 *                             RX_PCKT_LEN=RX_PCKT_LEN1 × 256 + RX_PCKT_LEN0
 *                             This value is packet_length%256
 */
#define RX_PCKT_LEN0_BASE               ((uint8_t)0xca)  /* RX_PCKT_LEN=RX_PCKT_LEN1
                                                          * × 256 + RX_PCKT_LEN0 */

/* CRC_FIELD[2:0] registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   CRC_FIELDx[7:0]:    upper(x=2), middle(x=1) and lower(x=0) part
 *                             of the crc field of the received packet
 */

#define CRC_FIELD2_BASE                 ((uint8_t)0xcb)  /* CRC2 field of the received packet */
#define CRC_FIELD1_BASE                 ((uint8_t)0xcc)  /* CRC1 field of the received packet */
#define CRC_FIELD0_BASE                 ((uint8_t)0xcd)  /* CRC0 field of the received packet */

/* RX_CTRL_FIELD[3:0] registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   RX_CTRL_FIELDx[7:0]:    upper(x=3), middle(x=2), middle(x=1) and
 *                                 lower(x=0) part of the control field of
 *                                 the received packet
 */

#define RX_CTRL_FIELD0_BASE             ((uint8_t)0xce)  /* CRTL3 Control field of the received packet */
#define RX_CTRL_FIELD1_BASE             ((uint8_t)0xcf)  /* CRTL2 Control field of the received packet */
#define RX_CTRL_FIELD2_BASE             ((uint8_t)0xd0)  /* CRTL1 Control field of the received packet */
#define RX_CTRL_FIELD3_BASE             ((uint8_t)0xd1)  /* CRTL0 Control field of the received packet */

/* RX_ADDR_FIELD[1:0] registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   RX_ADDR_FIELDx[7:0]:    source(x=1) and destination(x=0) address
 *                                 field of the received packet
 */

#define RX_ADDR_FIELD1_BASE             ((uint8_t)0xd2)  /* ADDR1 Address field of the received packet */
#define RX_ADDR_FIELD0_BASE             ((uint8_t)0xd3)  /* ADDR0 Address field of the received packet */

/* Protocol_Registers */

/* PROTOCOL2 register
 *
 *   Default value: 0x06
 *   Read Write
 *   7   CS_TIMEOUT_MASK:      1 - CS value contributes to timeout disabling
 *   6   SQI_TIMEOUT_MASK:     1 - SQI value contributes to timeout disabling
 *   5   PQI_TIMEOUT_MASK:     1 - PQI value contributes to timeout disabling
 *
 *   4:3  TX_SEQ_NUM_RELOAD[1:0]: TX sequence number to be used when counting
 *                              reset is required using the related command.
 *
 *   2   RCO_CALIBRATION[0]:   1 - Enables the automatic RCO calibration
 *   1   VCO_CALIBRATION[0]:   1 - Enables the automatic VCO calibration
 *   0   LDCR_MODE[0]:         1 - LDCR mode enabled
 */

#define PROTOCOL2_BASE                  ((uint8_t)0x50)  /* Protocol2 regisetr address */

#define PROTOCOL2_LDC_MODE_MASK         ((uint8_t)0x01)  /* Enable/disable Low duty Cycle mode */
#define PROTOCOL2_VCO_CALIBRATION_MASK  ((uint8_t)0x02)  /* Enable/disable VCO automatic
                                                          * calibration */
#define PROTOCOL2_RCO_CALIBRATION_MASK  ((uint8_t)0x04)  /* Enable/disable RCO automatic
                                                          * calibration */
#define PROTOCOL2_PQI_TIMEOUT_MASK      ((uint8_t)0x20)  /* PQI value contributes to timeout
                                                          * disabling */
#define PROTOCOL2_SQI_TIMEOUT_MASK      ((uint8_t)0x40)  /* SQI value contributes to timeout
                                                          * disabling */
#define PROTOCOL2_CS_TIMEOUT_MASK       ((uint8_t)0x80)  /* CS value contributes to timeout
                                                          * disabling */

/* PROTOCOL1 register
 *
 *   Default value: 0x00
 *   Read Write
 *   7   LDCR_RELOAD_ON_SYNC:   1 - LDCR timer will be reloaded with the
 *                                  value stored in the LDCR_RELOAD registers
 *   6   PIGGYBACKING:          1 - PIGGYBACKING enabled
 *   5:4   Reserved.
 *   3   SEED_RELOAD[0]:        1 - Reload the back-off random generator
 *                                  seed using the value written in the
 *                                  BU_COUNTER_SEED_MSByte / LSByte registers
 *   2   CSMA_ON   [0]:         1 - CSMA channel access mode enabled
 *   1   CSMA_PERS_ON[0]:       1 - CSMA persistent (no back-off) enabled
 *   0   AUTO_PCKT_FLT[0]:      1 - automatic packet filtering mode enabled
 */

#define PROTOCOL1_BASE                  ((uint8_t)0x51)  /* Protocol1 register address */

#define PROTOCOL1_AUTO_PCKT_FLT_MASK    ((uint8_t)0x01)  /* Enable/disable automatic packet
                                                          * filtering mode */
#define PROTOCOL1_CSMA_PERS_ON_MASK     ((uint8_t)0x02)  /* Enable/disable CSMA persistent
                                                          * (no back-off) */
#define PROTOCOL1_CSMA_ON_MASK          ((uint8_t)0x04)  /* Enable/disable CSMA channel
                                                          * access mode */
#define PROTOCOL1_SEED_RELOAD_MASK      ((uint8_t)0x08)  /* Reloads the seed of the PN generator
                                                          * for CSMA procedure */
#define PROTOCOL1_PIGGYBACKING_MASK     ((uint8_t)0x40)  /* Enable/disable Piggybacking */
#define PROTOCOL1_LDC_RELOAD_ON_SYNC_MASK ((uint8_t)0x80)
                                                         /* LDC timer will be reloaded with the
                                                          * value stored in the LDC_RELOAD
                                                          * registers */

/* PROTOCOL0 register
 *
 *   Default value: 0x08
 *   Read Write
 *   7:4   NMAX_RETX[3:0]:   Max number of re-TX.
 *                           0 - re-transmission is not performed
 *
 *   3     NACK_TX[0]:        1 - field NO_ACK=1 on transmitted packet
 *   2     AUTO_ACK[0]:       1 - automatic ack after RX
 *   1     PERS_RX[0]:        1 - persistent reception enabled
 *   0     PERS_TX[0]:        1 - persistent transmission enabled
 */

#define PROTOCOL0_BASE                  ((uint8_t)0x52)  /* Persistent RX/TX, autoack, Max
                                                          * number of retransmissions */

#define PROTOCOL0_PERS_TX_MASK          ((uint8_t)0x01)  /* Enables persistent transmission */
#define PROTOCOL0_PERS_RX_MASK          ((uint8_t)0x02)  /* Enables persistent reception */
#define PROTOCOL0_AUTO_ACK_MASK         ((uint8_t)0x04)  /* Enables auto acknowlegment */
#define PROTOCOL0_NACK_TX_MASK          ((uint8_t)0x08)  /* Writes field NO_ACK=1 on
                                                          * transmitted packet */
#define PROTOCOL0_NMAX_RETX_MASK        ((uint8_t)0xf0)  /* Retransmission mask */

/* TIMERS5 register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0   RX_TIMEOUT_PRESCALER[7:0] :  RX operation timeout:
 *                                      prescaler value
 */

#define TIMERS5_RX_TIMEOUT_PRESCALER_BASE ((uint8_t)0x53) /* RX operation
                                                           * timeout:
                                                           * prescaler
                                                           * value */

/* TIMERS4 register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0   RX_TIMEOUT_COUNTER[7:0] :  RX operation timeout: counter value
 */

#define TIMERS4_RX_TIMEOUT_COUNTER_BASE ((uint8_t)0x54)  /* RX operation timeout: counter value */

/* TIMERS3 register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0   LDCR_PRESCALER[7:0] :  LDC Mode:
 *                                Prescaler part of the wake-up value
 */

#define TIMERS3_LDC_PRESCALER_BASE      ((uint8_t)0x55)  /* LDC Mode: Prescaler of the wake-up
                                                          * timer */

/* TIMERS2 register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0   LDCR_COUNTER[7:0] :  LDC Mode: counter part of the wake-up value
 */

#define TIMERS2_LDC_COUNTER_BASE        ((uint8_t)0x56)  /* LDC Mode: counter of the wake-up timer */

/* TIMERS1 register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0   LDCR_RELOAD_PRESCALER[7:0] :  LDC Mode:
 *                                       Prescaler part of the reload value
 */

#define TIMERS1_LDC_RELOAD_PRESCALER_BASE ((uint8_t)0x57) /* LDC Mode:
                                                           * Prescaler part
                                                           * of the reload
                                                           * value */

/* TIMERS0 register
 *
 *   Default value: 0x00
 *   Read Write
 *   7:0   LDCR_RELOAD_COUNTER[7:0] :  LDC Mode:
 *                                     Counter part of the reload value
 */

#define TIMERS0_LDC_RELOAD_COUNTER_BASE ((uint8_t)0x58)  /* LDC Mode:  Counter part of the
                                                          * reload value */

/* CSMA_CONFIG3 registers
 *
 *   Default value: 0xff
 *   Read Write
 *   7:0     BU_COUNTER_SEED_MSByte: Seed of the random number generator used
 *           to apply the BEB (Binary Exponential Backoff) algorithm (MSB)
 */

#define CSMA_CONFIG3_BASE               ((uint8_t)0x64)  /* CSMA/CA:  Seed of the random number
                                                          * generator used to apply the BEB
                                                          * (Binary Exponential Backoff)
                                                          * algorithm (MSB) */

/* CSMA_CONFIG2 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0     BU_COUNTER_SEED_LSByte: Seed of the random number generator used
 *                                   to apply the BEB (Binary Exponential
 *                                   Backoff) algorithm (LSB)
 */

#define CSMA_CONFIG2_BASE               ((uint8_t)0x65)  /* CSMA/CA: Seed of the random
                                                          * number generator used to apply
                                                          * the BEB (Binary Exponential
                                                          * Backoff) algorithm (LSB) */

/* CSMA_CONFIG1 registers
 *
 *   Default value: 0x04
 *   Read Write
 *   7:2     BU_PRESCALER[5:0]: Used to program the back-off unit BU
 *   1:0     CCA_PERIOD[1:0]: Used to program the Tcca time
 *                            (64 / 128 /256 / 512 × Tbit.
 */

#define CSMA_CONFIG1_BASE               ((uint8_t)0x66)  /* CSMA/CA: Prescaler of the back-off
                                                          * time unit (BU); CCA period */

#define CSMA_CCA_PERIOD_64TBIT          ((uint8_t)0x00)  /* CSMA/CA: Sets CCA period to 64*TBIT */
#define CSMA_CCA_PERIOD_128TBIT         ((uint8_t)0x01)  /* CSMA/CA: Sets CCA period to 128*TBIT */
#define CSMA_CCA_PERIOD_256TBIT         ((uint8_t)0x02)  /* CSMA/CA: Sets CCA period to 256*TBIT */
#define CSMA_CCA_PERIOD_512TBIT         ((uint8_t)0x03)  /* CSMA/CA: Sets CCA period to 512*TBIT */

/* CSMA_CONFIG0 registers
 *
 *   Default value: 0x00
 *   Read Write
 *   7:4     CCA_LENGTH[3:0]: Used to program the Tlisten time
 *   3       Reserved.
 *   2:0     NBACKOFF_MAX[2:0]: Max number of back-off cycles.
 */

#define CSMA_CONFIG0_BASE               ((uint8_t)0x67)  /* CSMA/CA: CCA length; Max number
                                                          * of backoff cycles */

/* Link_Quality_Registers */

/* QI register
 *
 *   Read Write
 *   Default value: 0x02
 *
 *   7:6   SQI_TH[1:0]: SQI threshold according to the formula:
 *                      8*SYNC_LEN - 2*SQI_TH
 *   5:2   PQI_TH[3:0]: PQI threshold according to the formula:
 *                      4*PQI_THR
 *   1     SQI_EN[0]:  SQI enable
 *                 1 - Enable
 *                 0 - Disable
 *   0     PQI_EN[0]: PQI enable
 *                 1 - Enable
 *                 0 - Disable
 */

#define QI_BASE                         ((uint8_t)0x3a) /* QI register */

#define QI_PQI_MASK                     ((uint8_t)0x01) /* PQI enable/disable */
#define QI_SQI_MASK                     ((uint8_t)0x02) /* SQI enable/disable */

/* LINK_QUALIF2 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   PQI[7:0]: PQI value of the received packet
 */

#define LINK_QUALIF2_BASE               ((uint8_t)0xc5)  /* PQI value of the received packet */

/* LINK_QUALIF1 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7 CS:  Carrier Sense indication
 *   6:0   SQI[6:0]: SQI value of the received packet
 */

#define LINK_QUALIF1_BASE               ((uint8_t)0xc6)  /* Carrier sense indication [7]; SQI
                                                          * value of the received packet */

#define LINK_QUALIF1_CS                 ((uint8_t)0x80)  /* Carrier sense indication [7] */

/* LINK_QUALIF0 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:4   LQI [3:0]:  LQI value of the received packet
 *   3:0   AGC_WORD[3:0]: AGC word of the received packet
 */

#define LINK_QUALIF0_BASE               ((uint8_t)0xc7)  /* LQI value of the received packet
                                                          * [7:4]; AGC word of the received
                                                          * packet [3:0] */

/* RSSI_LEVEL registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   RSSI_LEVEL[7:0]:  RSSI level of the received packet
 */

#define RSSI_LEVEL_BASE                 ((uint8_t)0xc8)  /* RSSI level of the received packet */

/* RSSI_FLT register
 *
 *   Read Write
 *   Default value: 0xf3
 *   7:4  RSSI_FLT[3:0]: Gain of the RSSI filter
 *   3:2  CS_MODE[1:0]: AFC loop gain in slow mode (2's log)
 *
 *          CS_MODE1     | CS_MODE0  |            CS Mode
 *       --------------------------------------------------------------------
 *             0         |     0     | Static CS
 *             0         |     1     | Dynamic CS with 6dB dynamic threshold
 *             1         |     0     | Dynamic CS with 12dB dynamic threshold
 *             1         |     1     | Dynamic CS with 18dB dynamic threshold
 *
 *   1:0   OOK_PEAK_DECAY[1:0]: Peak decay control for OOK:
 *                              3 slow decay; 0 fast decay
 */

#define RSSI_FLT_BASE                   ((uint8_t)0x21)  /* Gain of the RSSI filter; lower value
                                                          * is fast but inaccurate, higher value
                                                          * is slow and more accurate */

#define RSSI_FLT_CS_MODE_MASK           ((uint8_t)0x0c)  /* Carrier sense mode mask */
#define RSSI_FLT_CS_MODE_STATIC         ((uint8_t)0x00)  /* Carrier sense mode; static carrier
                                                          * sensing */
#define RSSI_FLT_CS_MODE_DYNAMIC_6      ((uint8_t)0x04)  /* Carrier sense mode; dynamic carrier
                                                          * sensing with 6dB threshold */
#define RSSI_FLT_CS_MODE_DYNAMIC_12     ((uint8_t)0x08)  /* Carrier sense mode; dynamic carrier
                                                          * sensing with 12dB threshold */
#define RSSI_FLT_CS_MODE_DYNAMIC_18     ((uint8_t)0x0c)  /* Carrier sense mode; dynamic carrier
                                                          * sensing with 18dB threshold */
#define RSSI_FLT_OOK_PEAK_DECAY_MASK    ((uint8_t)0x03)  /* Peak decay control for OOK mask */
#define RSSI_FLT_OOK_PEAK_DECAY_FAST    ((uint8_t)0x00)  /* Peak decay control for OOK: fast
                                                          * decay */
#define RSSI_FLT_OOK_PEAK_DECAY_MEDIUM_FAST ((uint8_t)0x01)
                                                         /* Peak decay control for OOK:
                                                          * medium_fast decay */
#define RSSI_FLT_OOK_PEAK_DECAY_MEDIUM_SLOW ((uint8_t)0x02)
                                                         /* Peak decay control for OOK:
                                                          * medium_slow decay */
#define RSSI_FLT_OOK_PEAK_DECAY_SLOW    ((uint8_t)0x03)  /* Peak decay control for OOK: slow
                                                          * decay */

/* RSSI_TH register
 *
 *   Read Write
 *   Default value: 0x24
 *
 *   7:0 RSSI_THRESHOLD [7:0]:  Signal detect threshold in 0.5dB.
 *                              -120dBm corresponds to 20
 */

#define RSSI_TH_BASE                    ((uint8_t)0x22)  /* Signal detect threshold in 0.5dB
                                                          * stp. 20 correspond to -120 dBm */

/* FIFO_Registers */

/* FIFO_CONFIG3 registers
 *
 *   Default value: 0x30
 *   Read Write
 *
 *   7    Reserved.
 *   6:0  rxafthr [6:0]: FIFO Almost Full threshold for rx fifo.
 */

#define FIFO_CONFIG3_RXAFTHR_BASE       ((uint8_t)0x3e)  /* FIFO Almost Full threshold for rx
                                                          * fifo [6:0] */

/* FIFO_CONFIG2 registers
 *
 *   Default value: 0x30
 *   Read Write
 *
 *   7    Reserved.
 *   6:0  rxaethr [6:0]: FIFO Almost Empty threshold for rx fifo.
 */

#define FIFO_CONFIG2_RXAETHR_BASE       ((uint8_t)0x3f)  /* FIFO Almost Empty threshold for rx
                                                          * fifo [6:0] */

/* FIFO_CONFIG1 registers
 *
 *   Default value: 0x30
 *   Read Write
 *
 *   7    Reserved.
 *   6:0  txafthr [6:0]: FIFO Almost Full threshold for tx fifo.
 */

#define FIFO_CONFIG1_TXAFTHR_BASE       ((uint8_t)0x40)  /* FIFO Almost Full threshold for tx
                                                          * fifo [6:0] */

/* FIFO_CONFIG0 registers
 *
 *   Default value: 0x30
 *   Read Write
 *
 *   7    Reserved.
 *   6:0  txaethr [6:0]: FIFO Almost Empty threshold for tx fifo.
 */

#define FIFO_CONFIG0_TXAETHR_BASE       ((uint8_t)0x41)  /* FIFO Almost Empty threshold for tx
                                                          * fifo [6:0] */

/* LINEAR_FIFO_STATUS1 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7     Reserved.
 *   6:0   elem_txfifo[6:0]:  Number of elements in the linear TXFIFO (<=96)
 */

#define LINEAR_FIFO_STATUS1_BASE        ((uint8_t)0xe6)  /* Number of elements in the linear TX
                                                          * FIFO [6:0] (<=96) */

/* LINEAR_FIFO_STATUS0 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7     Reserved.
 *
 *   6:0   elem_rxfifo[6:0]:  Number of elements in the linear RXFIFO (<=96)
 */

#define LINEAR_FIFO_STATUS0_BASE        ((uint8_t)0xe7)  /* Number of elements in the linear RX
                                                          * FIFO [6:0] (<=96) */

/* Calibration_Registers */

/* RCO_VCO_CALIBR_IN2 registers
 *
 *   Default value: 0x70
 *   Read Write
 *
 *   7:4     RWT_IN[3:0]:  RaWThermometric word value for the RCO [7:4]
 *   3:0     RFB_IN[4:1]:  ResistorFineBit word value for the RCO
 *                         (first 4 bits)
 */

#define RCO_VCO_CALIBR_IN2_BASE         ((uint8_t)0x6d)  /* RaWThermometric word value for the
                                                          * RCO [7:4]; ResistorFineBit word value
                                                          * for the RCO [3:0] */

/* RCO_VCO_CALIBR_IN1 registers
 *
 *   Default value: 0x48
 *   Read Write
 *
 *   7     RFB_IN[0]: ResistorFineBit word value for the RCO (LSb)
 *   6:0   VCO_CALIBR_TX[6:0]:  Word value for the VCO to be used in TX mode
 */

#define RCO_VCO_CALIBR_IN1_BASE         ((uint8_t)0x6e)  /* ResistorFineBit word value for the
                                                          * RCO [7]; Word value for the VCO to
                                                          * be used in TX mode [6:0] */

/* RCO_VCO_CALIBR_IN0 registers
 *
 *   Default value: 0x48
 *   Read Write
 *
 *   7     Reserved.
 *   6:0   VCO_CALIBR_RX[6:0]:  Word value for the VCO to be used in RX mode
 */

#define RCO_VCO_CALIBR_IN0_BASE         ((uint8_t)0x6f)  /* Word value for the VCO to be used
                                                          * in RX mode [6:0] */

/* RCO_VCO_CALIBR_OUT1 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:4   RWT_OUT[3:0]:  RWT word from internal RCO calibrator
 *   3:0   RFB_OUT[4:1]:  RFB word from internal RCO calibrator
 *                        (upper part)
 */

 #define RCO_VCO_CALIBR_OUT1_BASE       ((uint8_t)0xe4)  /* RaWThermometric RWT word from
                                                          * internal RCO  calibrator [7];
                                                          * ResistorFineBit RFB word from
                                                          * internal RCO oscillator [6:0] */

/* RCO_VCO_CALIBR_OUT0 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7     RFB_OUT[0]:            RFB word from internal RCO calibrator
 *                                (last bit LSB)
 *   6:0   VCO_CALIBR_DATA[6:0]:  Output word from internal VCO calibrator
 */

#define RCO_VCO_CALIBR_OUT0_BASE        ((uint8_t)0xe5)  /* ResistorFineBit RFB word from
                                                          * internal RCO oscillator [0]; Output
                                                          * word from internal calibrator [6:0]; */

/* AES_Registers */

/* AES_KEY_INx registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   AES_KEY_INx[7:0]:  AES engine key input (total - 128 bits)
 */

#define AES_KEY_IN_15_BASE              ((uint8_t)0x70)  /* AES engine key input 15 */
#define AES_KEY_IN_14_BASE              ((uint8_t)0x71)  /* AES engine key input 14 */
#define AES_KEY_IN_13_BASE              ((uint8_t)0x72)  /* AES engine key input 13 */
#define AES_KEY_IN_12_BASE              ((uint8_t)0x73)  /* AES engine key input 12 */
#define AES_KEY_IN_11_BASE              ((uint8_t)0x74)  /* AES engine key input 11 */
#define AES_KEY_IN_10_BASE              ((uint8_t)0x75)  /* AES engine key input 10 */
#define AES_KEY_IN_9_BASE               ((uint8_t)0x76)  /* AES engine key input 9 */
#define AES_KEY_IN_8_BASE               ((uint8_t)0x77)  /* AES engine key input 8 */
#define AES_KEY_IN_7_BASE               ((uint8_t)0x78)  /* AES engine key input 7 */
#define AES_KEY_IN_6_BASE               ((uint8_t)0x79)  /* AES engine key input 6 */
#define AES_KEY_IN_5_BASE               ((uint8_t)0x7a)  /* AES engine key input 5 */
#define AES_KEY_IN_4_BASE               ((uint8_t)0x7b)  /* AES engine key input 4 */
#define AES_KEY_IN_3_BASE               ((uint8_t)0x7c)  /* AES engine key input 3 */
#define AES_KEY_IN_2_BASE               ((uint8_t)0x7d)  /* AES engine key input 2 */
#define AES_KEY_IN_1_BASE               ((uint8_t)0x7e)  /* AES engine key input 1 */
#define AES_KEY_IN_0_BASE               ((uint8_t)0x7f)  /* AES engine key input 0 */

/* AES_DATA_INx registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   AES_DATA_INx[7:0]:  AES engine data input (total - 128 bits)
 */

#define AES_DATA_IN_15_BASE             ((uint8_t)0x80)  /* AES engine data input 15 Take care:
                                                          * Address is in reverse order respect
                                                          * data numbering; eg.: 0x81 ->
                                                          * AES_data14[7:0] */
#define AES_DATA_IN_14_BASE             ((uint8_t)0x81)  /* AES engine data input 14 */
#define AES_DATA_IN_13_BASE             ((uint8_t)0x82)  /* AES engine data input 13 */
#define AES_DATA_IN_12_BASE             ((uint8_t)0x83)  /* AES engine data input 12 */
#define AES_DATA_IN_11_BASE             ((uint8_t)0x84)  /* AES engine data input 11 */
#define AES_DATA_IN_10_BASE             ((uint8_t)0x85)  /* AES engine data input 10 */
#define AES_DATA_IN_9_BASE              ((uint8_t)0x86)  /* AES engine data input 9 */
#define AES_DATA_IN_8_BASE              ((uint8_t)0x87)  /* AES engine data input 8 */
#define AES_DATA_IN_7_BASE              ((uint8_t)0x88)  /* AES engine data input 7 */
#define AES_DATA_IN_6_BASE              ((uint8_t)0x89)  /* AES engine data input 6 */
#define AES_DATA_IN_5_BASE              ((uint8_t)0x8a)  /* AES engine data input 5 */
#define AES_DATA_IN_4_BASE              ((uint8_t)0x8b)  /* AES engine data input 4 */
#define AES_DATA_IN_3_BASE              ((uint8_t)0x8c)  /* AES engine data input 3 */
#define AES_DATA_IN_2_BASE              ((uint8_t)0x8d)  /* AES engine data input 2 */
#define AES_DATA_IN_1_BASE              ((uint8_t)0x8e)  /* AES engine data input 1 */
#define AES_DATA_IN_0_BASE              ((uint8_t)0x8f)  /* AES engine data input 0 */

/* AES_DATA_OUT[15:0] registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:0   AES_DATA_OUTx[7:0]:  AES engine data output (128 bits)
 */

#define AES_DATA_OUT_15_BASE            ((uint8_t)0xd4)  /* AES engine data output 15 */
#define AES_DATA_OUT_14_BASE            ((uint8_t)0xd5)  /* AES engine data output 14 */
#define AES_DATA_OUT_13_BASE            ((uint8_t)0xd6)  /* AES engine data output 13 */
#define AES_DATA_OUT_12_BASE            ((uint8_t)0xd7)  /* AES engine data output 12 */
#define AES_DATA_OUT_11_BASE            ((uint8_t)0xd8)  /* AES engine data output 11 */
#define AES_DATA_OUT_10_BASE            ((uint8_t)0xd9)  /* AES engine data output 10 */
#define AES_DATA_OUT_9_BASE             ((uint8_t)0xda)  /* AES engine data output 9 */
#define AES_DATA_OUT_8_BASE             ((uint8_t)0xdb)  /* AES engine data output 8 */
#define AES_DATA_OUT_7_BASE             ((uint8_t)0xdc)  /* AES engine data output 7 */
#define AES_DATA_OUT_6_BASE             ((uint8_t)0xdd)  /* AES engine data output 6 */
#define AES_DATA_OUT_5_BASE             ((uint8_t)0xde)  /* AES engine data output 5 */
#define AES_DATA_OUT_4_BASE             ((uint8_t)0xdf)  /* AES engine data output 4 */
#define AES_DATA_OUT_3_BASE             ((uint8_t)0xe0)  /* AES engine data output 3 */
#define AES_DATA_OUT_2_BASE             ((uint8_t)0xe1)  /* AES engine data output 2 */
#define AES_DATA_OUT_1_BASE             ((uint8_t)0xe2)  /* AES engine data output 1 */
#define AES_DATA_OUT_0_BASE             ((uint8_t)0xe3)  /* AES engine data output 0 */

/* IRQ_Registers */

/* IRQ_MASK0 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_MASK0:  IRQ mask, if the correspondent bit is set and IRQ
 *                     can be generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            0   |       RX data ready
 *            1   |       RX data discarded (upon filtering)
 *            2   |       TX data sent
 *            3   |       Max re-TX reached
 *            4   |       CRC error
 *            5   |       TX FIFO underflow/overflow error
 *            6   |       RX FIFO underflow/overflow error
 *            7   |       TX FIFO almost full
 */

#define IRQ_MASK0_BASE                  ((uint8_t)0x93)  /* IRQ_MASK is split into 4 registers */

#define IRQ_MASK0_RX_DATA_READY         ((uint8_t)0x01)  /* IRQ: RX data ready */
#define IRQ_MASK0_RX_DATA_DISC          ((uint8_t)0x02)  /* IRQ: RX data discarded (upon filtering) */
#define IRQ_MASK0_TX_DATA_SENT          ((uint8_t)0x04)  /* IRQ: TX data sent */
#define IRQ_MASK0_MAX_RE_TX_REACH       ((uint8_t)0x08)  /* IRQ: Max re-TX reached */
#define IRQ_MASK0_CRC_ERROR             ((uint8_t)0x10)  /* IRQ: CRC error */
#define IRQ_MASK0_TX_FIFO_ERROR         ((uint8_t)0x20)  /* IRQ: TX FIFO underflow/overflow error */
#define IRQ_MASK0_RX_FIFO_ERROR         ((uint8_t)0x40)  /* IRQ: RX FIFO underflow/overflow error */
#define IRQ_MASK0_TX_FIFO_ALMOST_FULL   ((uint8_t)0x80)  /* IRQ: TX FIFO almost full */

/* IRQ_MASK1 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_MASK1:  IRQ mask, if the correspondent bit is set and IRQ
 *                     can be generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            8   |       TX FIFO almost empty
 *            9   |       RX FIFO almost full
 *            10  |       RX FIFO almost empty
 *            11  |       Max number of back-off during CCA
 *            12  |       Valid preamble detected
 *            13  |       Sync word detected
 *            14  |       RSSI above threshold (Carrier Sense)
 *            15  |       Wake-up timeout in LDCR mode13
 */

#define IRQ_MASK1_BASE                  ((uint8_t)0x92)  /* IRQ_MASK is split into 4 registers */

#define IRQ_MASK1_TX_FIFO_ALMOST_EMPTY  ((uint8_t)0x01)  /* IRQ: TX FIFO almost empty */
#define IRQ_MASK1_RX_FIFO_ALMOST_FULL   ((uint8_t)0x02)  /* IRQ: RX FIFO almost full */
#define IRQ_MASK1_RX_FIFO_ALMOST_EMPTY  ((uint8_t)0x04)  /* IRQ: RX FIFO almost empty */
#define IRQ_MASK1_MAX_BO_CCA_REACH      ((uint8_t)0x08)  /* IRQ: Max number of back-off during CCA */
#define IRQ_MASK1_VALID_PREAMBLE        ((uint8_t)0x10)  /* IRQ: Valid preamble detected */
#define IRQ_MASK1_VALID_SYNC            ((uint8_t)0x20)  /* IRQ: Sync word detected */
#define IRQ_MASK1_RSSI_ABOVE_TH         ((uint8_t)0x40)  /* IRQ: RSSI above threshold */
#define IRQ_MASK1_WKUP_TOUT_LDC         ((uint8_t)0x80)  /* IRQ: Wake-up timeout in LDC mode */

/* IRQ_MASK2 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_MASK2:  IRQ mask, if the correspondent bit is set and IRQ
 *                     can be generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            16  |       READY state in steady condition14
 *            17  |       STANDBY state switching in progress
 *            18  |       Low battery level
 *            19  |       Power-On reset
 *            20  |       Brown-Out event
 *            21  |       LOCK state in steady condition
 *            22  |       PM start-up timer expiration
 *            23  |       XO settling timeout
 */

#define IRQ_MASK2_BASE                  ((uint8_t)0x91)  /* IRQ_MASK is split into 4 registers */

#define IRQ_MASK2_READY                 ((uint8_t)0x01)  /* IRQ: READY state */
#define IRQ_MASK2_STANDBY_DELAYED       ((uint8_t)0x02)  /* IRQ: STANDBY state after
                                                          * MCU_CK_CONF_CLOCK_TAIL_X clock cycles */
#define IRQ_MASK2_LOW_BATT_LVL          ((uint8_t)0x04)  /* IRQ: Battery level below threshold */
#define IRQ_MASK2_POR                   ((uint8_t)0x08)  /* IRQ: Power On Reset */
#define IRQ_MASK2_BOR                   ((uint8_t)0x10)  /* IRQ: Brown out event (both accurate
                                                          * and inaccurate) */
#define IRQ_MASK2_LOCK                  ((uint8_t)0x20)  /* IRQ: LOCK state */
#define IRQ_MASK2_PM_COUNT_EXPIRED      ((uint8_t)0x40)  /* IRQ: only for debug; Power Management
                                                          * startup timer expiration (see reg
                                                          * PM_START_COUNTER, 0xb5) */
#define IRQ_MASK2_XO_COUNT_EXPIRED      ((uint8_t)0x80)  /* IRQ: only for debug; Crystal
                                                          * oscillator settling time counter
                                                          * expired */

/* IRQ_MASK3 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_MASK3:  IRQ mask, if the correspondent bit is set and IRQ
 *                     can be generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            24  |       SYNTH locking timeout
 *            25  |       SYNTH calibration start-up time
 *            26  |       SYNTH calibration timeout
 *            27  |       TX circuitry start-up time
 *            28  |       RX circuitry start-up time
 *            29  |       RX operation timeout
 *            30  |       Others AES End–of –Operation
 *            31  |       Reserved
 */

#define IRQ_MASK3_BASE                  ((uint8_t)0x90)  /* IRQ_MASK is split into 4 registers */

#define IRQ_MASK3_SYNTH_LOCK_TIMEOUT    ((uint8_t)0x01)  /* IRQ: only for debug; LOCK state
                                                          * timeout */
#define IRQ_MASK3_SYNTH_LOCK_STARTUP    ((uint8_t)0x02)  /* IRQ: only for debug; see
                                                          * CALIBR_START_COUNTER */
#define IRQ_MASK3_SYNTH_CAL_TIMEOUT     ((uint8_t)0x04)  /* IRQ: only for debug; SYNTH
                                                          * calibration timeout */
#define IRQ_MASK3_TX_START_TIME         ((uint8_t)0x08)  /* IRQ: only for debug; TX circuitry
                                                          * startup time; see TX_START_COUNTER */
#define IRQ_MASK3_RX_START_TIME         ((uint8_t)0x10)  /* IRQ: only for debug; RX circuitry
                                                          * startup time; see TX_START_COUNTER */
#define IRQ_MASK3_RX_TIMEOUT            ((uint8_t)0x20)  /* IRQ: RX operation timeout */
#define IRQ_MASK3_AES_END               ((uint8_t)0x40)  /* IRQ: AES End of operation */

/* IRQ_STATUS0 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_STATUS0:  IRQ status, if the correspondent bit is set and IRQ
 *                       has been generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            0   |       RX data ready
 *            1   |       RX data discarded (upon filtering)
 *            2   |       TX data sent
 *            3   |       Max re-TX reached
 *            4   |       CRC error
 *            5   |       TX FIFO underflow/overflow error
 *            6   |       RX FIFO underflow/overflow error
 *            7   |       TX FIFO almost full
 */

#define IRQ_STATUS0_BASE                ((uint8_t)0xfd)  /* IRQ Events(RR, split into 4 registers) */

#define IRQ_STATUS0_SYNTH_LOCK_TIMEOUT  ((uint8_t)0x01)  /* IRQ: LOCK  state timeout */
#define IRQ_STATUS0_SYNTH_LOCK_STARTUP  ((uint8_t)0x02)  /* IRQ: only for debug; see
                                                          * CALIBR_START_COUNTER */
#define IRQ_STATUS0_SYNTH_CAL_TIMEOUT   ((uint8_t)0x04)  /* IRQ: SYNTH locking timeout */
#define IRQ_STATUS0_TX_START_TIME       ((uint8_t)0x08)  /* IRQ: only for debug; TX circuitry
                                                          * startup time; see TX_START_COUNTER */
#define IRQ_STATUS0_RX_START_TIME       ((uint8_t)0x10)  /* IRQ: only for debug; RX circuitry
                                                          * startup time; see TX_START_COUNTER */
#define IRQ_STATUS0_RX_TIMEOUT          ((uint8_t)0x20)  /* IRQ: RX operation timeout expiration */
#define IRQ_STATUS0_AES_END             ((uint8_t)0x40)  /* IRQ: AES End of operation */

/* IRQ_STATUS1 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_STATUS1:  IRQ status, if the correspondent bit is set and IRQ
 *                       has been generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            8   |       TX FIFO almost empty
 *            9   |       RX FIFO almost full
 *            10  |       RX FIFO almost empty
 *            11  |       Max number of back-off during CCA
 *            12  |       Valid preamble detected
 *            13  |       Sync word detected
 *            14  |       RSSI above threshold (Carrier Sense)
 *            15  |       Wake-up timeout in LDCR mode13
 */

#define IRQ_STATUS1_BASE                ((uint8_t)0xfc)  /* IRQ Events(RR, split into 4 registers) */

#define IRQ_STATUS1_READY               ((uint8_t)0x01)  /* IRQ: READY state in steady condition */
#define IRQ_STATUS1_STANDBY_DELAYED     ((uint8_t)0x02)  /* IRQ: STANDBY state after
                                                          * MCU_CK_CONF_CLOCK_TAIL_X clock cycles */
#define IRQ_STATUS1_LOW_BATT_LVL        ((uint8_t)0x04)  /* IRQ: Battery level below threshold */
#define IRQ_STATUS1_POR                 ((uint8_t)0x08)  /* IRQ: Power On Reset */
#define IRQ_STATUS1_BOR                 ((uint8_t)0x10)  /* IRQ: Brown out event (both accurate
                                                          * and inaccurate) */
#define IRQ_STATUS1_LOCK                ((uint8_t)0x20)  /* IRQ: LOCK state in steady condition */
#define IRQ_STATUS1_PM_COUNT_EXPIRED    ((uint8_t)0x40)  /* IRQ: Power Management startup timer
                                                          * expiration (see reg PM_START_COUNTER,
                                                          * 0xb5) */
#define IRQ_STATUS1_XO_COUNT_EXPIRED    ((uint8_t)0x80)  /* IRQ: Crystal oscillator settling
                                                          * time counter expired */

/* IRQ_STATUS2 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_STATUS2:  IRQ status, if the correspondent bit is set and IRQ
 *                       has been generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            16  |       READY state in steady condition14
 *            17  |       STANDBY state switching in progress
 *            18  |       Low battery level
 *            19  |       Power-On reset
 *            20  |       Brown-Out event
 *            21  |       LOCK state in steady condition
 *            22  |       PM start-up timer expiration
 *            23  |       XO settling timeout
 */

#define IRQ_STATUS2_BASE                 ((uint8_t)0xfb)  /* IRQ Events(RR, split into 4 registers) */

#define IRQ_STATUS2_TX_FIFO_ALMOST_EMPTY ((uint8_t)0x01) /* IRQ: TX FIFO almost empty */
#define IRQ_STATUS2_RX_FIFO_ALMOST_FULL ((uint8_t)0x02)  /* IRQ: RX FIFO almost full */
#define IRQ_STATUS2_RX_FIFO_ALMOST_EMPTY ((uint8_t)0x04) /* IRQ: RX FIFO almost empty */
#define IRQ_STATUS2_MAX_BO_CCA_REACH    ((uint8_t)0x08)  /* IRQ: Max number of back-off during
                                                          * CCA */
#define IRQ_STATUS2_VALID_PREAMBLE      ((uint8_t)0x10)  /* IRQ: Valid preamble detected */
#define IRQ_STATUS2_VALID_SYNC          ((uint8_t)0x20)  /* IRQ: Sync word detected */
#define IRQ_STATUS2_RSSI_ABOVE_TH       ((uint8_t)0x40)  /* IRQ: RSSI above threshold */
#define IRQ_STATUS2_WKUP_TOUT_LDC       ((uint8_t)0x80)  /* IRQ: Wake-up timeout in LDC mode */

/* IRQ_STATUS3 registers
 *
 *   Default value: 0x00
 *   Read Write
 *
 *   7:0   INT_STATUS3:  IRQ status, if the correspondent bit is set and IRQ
 *                       has been generated (according to the next table)
 *
 *            Bit |     Events Group Interrupt Event
 *           -------------------------------------------------------
 *            24  |       SYNTH locking timeout
 *            25  |       SYNTH calibration start-up time
 *            26  |       SYNTH calibration timeout
 *            27  |       TX circuitry start-up time
 *            28  |       RX circuitry start-up time
 *            29  |       RX operation timeout
 *            30  |       Others AES End–of –Operation
 *            31  |       Reserved
 */

#define IRQ_STATUS3_BASE                ((uint8_t)0xfa)  /* IRQ Events(RR, split into 4 registers) */

#define IRQ_STATUS3_RX_DATA_READY       ((uint8_t)0x01)  /* IRQ: RX data ready */
#define IRQ_STATUS3_RX_DATA_DISC        ((uint8_t)0x02)  /* IRQ: RX data discarded (upon filtering) */
#define IRQ_STATUS3_TX_DATA_SENT        ((uint8_t)0x04)  /* IRQ: TX data sent */
#define IRQ_STATUS3_MAX_RE_TX_REACH     ((uint8_t)0x08)  /* IRQ: Max re-TX reached */
#define IRQ_STATUS3_CRC_ERROR           ((uint8_t)0x10)  /* IRQ: CRC error */
#define IRQ_STATUS3_TX_FIFO_ERROR       ((uint8_t)0x20)  /* IRQ: TX FIFO underflow/overflow error */
#define IRQ_STATUS3_RX_FIFO_ERROR       ((uint8_t)0x40)  /* IRQ: RX FIFO underflow/overflow error */
#define IRQ_STATUS3_TX_FIFO_ALMOST_FULL ((uint8_t)0x80)  /* IRQ: TX FIFO almost full */

/* MC_STATE_Registers */

/* MC_STATE1 registers
 *
 *   Default value: 0x50
 *   Read
 *
 *   7:4     Reserved.
 *   3       ANT_SELECT:  Currently selected antenna
 *   2       TX_FIFO_Full:     1 - TX FIFO is full
 *   1       RX_FIFO_Empty:    1 - RX FIFO is empty
 *   0       ERROR_LOCK:       1 - RCO calibrator error
 */

#define MC_STATE1_BASE                  ((uint8_t)0xc0)  /* MC_STATE1 register address (see the
                                                          * struct spirit_status_s */

/* MC_STATE0 registers
 *
 *   Default value: 0x00
 *   Read
 *
 *   7:1     STATE[6:0]: Current MC state.
 *
 *      REGISTER VALUE |         STATE
 *   --------------------------------------------
 *            0x40     |         STANDBY
 *            0x36     |         SLEEP
 *            0x03     |         READY
 *            0x3b     |         PM setup
 *            0x23     |         XO settling
 *            0x53     |         SYNTH setup
 *            0x1f     |         PROTOCOL
 *            0x4f     |         SYNTH calibration
 *            0x0f     |         LOCK
 *            0x33     |         RX
 *            0x5f     |         TX
 *
 *   0       XO_ON:       1 - XO is operating
 */

#define MC_STATE0_BASE                  ((uint8_t)0xc1)  /* MC_STATE0 register address. In this
                                                          * version ALL existing states have
                                                          * been inserted and are still to be
                                                          * verified */

/* Engineering-Test_Registers */

#define SYNTH_CONFIG1_BASE              ((uint8_t)0x9e)  /* Synthesizier registers: M, A, K
                                                          * data sync on positive/negative
                                                          * clock edges [4], Enable Linearization
                                                          * of the charge pump [3], split time
                                                          * 1.75/3.45ns [2], VCO calibration window
                                                          * 16,32,64,128 clock cycles [1:0] */
#define SYNTH_CONFIG0_BASE              ((uint8_t)0x9f)  /* Enable DSM randomizer [7], Window width
                                                          * 1.2-7.5ns (Down-up) of lock detector */
#define VCOTH_BASE                      ((uint8_t)0xa0)  /* Controls the threshold frequency
                                                          * between VCO low and VCO high [7:0] VCOth
                                                          * frequency=2*fXO*(96+VCO_TH/16),
                                                          * fmin=4992 MHz, fmax=5820 MHz */
#define PM_CONFIG2_BASE                 ((uint8_t)0xa4)  /* Enables high current buffer on
                                                          * Temperature sensor, sets SMPS options */
#define PM_CONFIG1_BASE                 ((uint8_t)0xa5)  /* Set SMPS options */
#define PM_CONFIG0_BASE                 ((uint8_t)0xa6)  /* Set SMPS options */
#define VCO_CONFIG_BASE                 ((uint8_t)0xa1)  /* Set VCO current [5:2]part and [1:0] part */
#define XO_CONFIG_BASE                  ((uint8_t)0xa7)  /* Clock management options from XO to
                                                          * digital part */
#define XO_RCO_TEST_BASE                ((uint8_t)0xb4)  /* Test of XO and RCO */

/* Commands */

#define COMMAND_TX                      ((uint8_t)0x60)  /* Start to transmit; valid only from
                                                          * READY */
#define COMMAND_RX                      ((uint8_t)0x61)  /* Start to receive; valid only from
                                                          * READY */
#define COMMAND_READY                   ((uint8_t)0x62)  /* Go to READY; valid only from STANDBY
                                                          * or SLEEP or LOCK */
#define COMMAND_STANDBY                 ((uint8_t)0x63)  /* Go to STANDBY; valid only from READY */
#define COMMAND_SLEEP                   ((uint8_t)0x64)  /* Go to SLEEP; valid only from READY */
#define COMMAND_LOCKRX                  ((uint8_t)0x65)  /* Go to LOCK state by using the RX
                                                          * configuration of the synth; valid
                                                          * only from READY */
#define COMMAND_LOCKTX                  ((uint8_t)0x66)  /* Go to LOCK state by using the TX
                                                          * configuration of the synth; valid
                                                          * only from READY */
#define COMMAND_SABORT                  ((uint8_t)0x67)  /* Force exit form TX or RX states and
                                                          * go to READY state; valid only from
                                                          * TX or RX */
#define COMMAND_LDC_RELOAD              ((uint8_t)0x68)  /* LDC Mode: Reload the LDC timer with
                                                          * the value stored in the LDC_PRESCALER
                                                          * / COUNTER registers; valid from all
                                                          * states */
#define COMMAND_SEQUENCE_UPDATE         ((uint8_t)0x69)  /* Autoretransmission: Reload the
                                                          * Packet sequence counter with the
                                                          * value stored in the PROTOCOL[2]
                                                          * register valid from all states */
#define COMMAND_AES_ENC                 ((uint8_t)0x6a)  /* AES: Start the encryption routine;
                                                          * valid from all states */
#define COMMAND_AES_KEY                 ((uint8_t)0x6b)  /* AES: Start the procedure to compute
                                                          * the key for the decryption; valid
                                                          * from all states */
#define COMMAND_AES_DEC                 ((uint8_t)0x6c)  /* AES: Start the decryption routine
                                                          * using the current key; valid from
                                                          * all states */
#define COMMAND_AES_KEY_DEC             ((uint8_t)0x6d)  /* AES: Compute the key and start the
                                                          * decryption; valid from all states */
#define COMMAND_SRES                    ((uint8_t)0x70)  /* Reset of all digital part, except
                                                          * SPI registers */
#define COMMAND_FLUSHRXFIFO             ((uint8_t)0x71)  /* Clean the RX FIFO; valid from all
                                                          * states */
#define COMMAND_FLUSHTXFIFO             ((uint8_t)0x72)  /* Clean the TX FIFO; valid from all
                                                          * states */

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_REGS_H */
