/****************************************************************************
 * drivers/wireless/lpwan/sx127x.h
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

#ifndef __DRIVERS_WIRELESS_LPWAN_SX127X_H
#define __DRIVERS_WIRELESS_LPWAN_SX127X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers ****************************************************************/

#define SX127X_R_REGISTER                 0x00 /* SX127X read register */
#define SX127X_W_REGISTER                 0x80 /* SX127X write register */

/* Registers common for FSK/OOK Mode and LORA Mode */

#define SX127X_CMN_FIFO                   0x00 /* FSK/OOK/LORA: FIFO read/write access */
#define SX127X_CMN_OPMODE                 0x01 /* FSK/OOK/LORA: Operating mode & LORA/FSK selection */
#define SX127X_CMN_FRFMSB                 0x06 /* FSK/OOK/LORA: RF Carrier Frequnecy, MSB */
#define SX127X_CMN_FRFMID                 0x07 /* FSK/OOK/LORA: RF Carrier Frequency, Intermediate Bits */
#define SX127X_CMN_FRFLSB                 0x08 /* FSK/OOK/LORA: RF Carrier Frequency, LSB */
#define SX127X_CMN_PACFG                  0x09 /* FSK/OOK/LORA: PA selection and Output Power control */
#define SX127X_CMN_PARAMP                 0x0a /* FSK/OOK/LORA: Control of PA ramp time, low phase noise PLL */
#define SX127X_CMN_OCP                    0x0b /* FSK/OOK/LORA: Over Current Protection control */
#define SX127X_CMN_LNA                    0x0c /* FSK/OOK/LORA: LNA settings */
#define SX127X_CMN_DIOMAP1                0x40 /* FSK/OOK/LORA: Mapping of pins DIO0 to DIO3 */
#define SX127X_CMN_DIOMAP2                0x41 /* FSK/OOK/LORA: Mapping of pins DIO4 and DIO5, ClkOut freq */
#define SX127X_CMN_VERSION                0x42 /* FSK/OOK/LORA: Semtech ID relating the silicon version */
#define SX127X_CMN_TCXO                   0x4b /* FSK/OOK/LORA: TCXO or XTAL input setting */
#define SX127X_CMN_PADAC                  0x4d /* FSK/OOK/LORA: Higher power settings of the PA */
#define SX127X_CMN_FTEMP                  0x5b /* FSK/OOK/LORA: Stored temperature during the former IQ calibration */
#define SX127X_CMN_AGCREF                 0x61 /* FSK/OOK/LORA: AGC thresholds reference */
#define SX127X_CMN_AGCTHR1                0x62 /* FSK/OOK/LORA: AGC thresholds 1 */
#define SX127X_CMN_AGCTHR2                0x63 /* FSK/OOK/LORA: AGC thresholds 2 */
#define SX127X_CMN_AGCTHR3                0x64 /* FSK/OOK/LORA: AGC thresholds 3 */
#define SX127X_CMN_PLL                    0x70 /* FSK/OOK/LORA: Control of the PLL bandwidth  */

/* Registers specific for FSK/OOK Mode */

#define SX127X_FOM_BITRATEMSB             0x02 /* FSK/OOK: Bit Rate setting, MSB */
#define SX127X_FOM_BITRATELSB             0x03 /* FSK/OOK: Bit Rate setting, LSB */
#define SX127X_FOM_FDEVMSB                0x04 /* FSK/OOK: Frequency Deviation setting, MSB */
#define SX127X_FOM_FDEVLSB                0x05 /* FSK/OOK: Frequency Deviation setting, LSB */
#define SX127X_FOM_RXCFG                  0x0d /* FSK/OOK: AFC, AGC, ctrl */
#define SX127X_FOM_RSSICFG                0x0e /* FSK/OOK: RSSI */
#define SX127X_FOM_RSSICOLL               0x0f /* FSK/OOK: RSSI Collision detector */
#define SX127X_FOM_RSSITHR                0x10 /* FSK/OOK: RSSI Threshold control */
#define SX127X_FOM_RSSIVAL                0x11 /* FSK/OOK: RSSI value in dBm */
#define SX127X_FOM_RXBW                   0x12 /* FSK/OOK: Channel Filter BW Control */
#define SX127X_FOM_AFCBW                  0x13 /* FSK/OOK: AFC Channel Filter BW */
#define SX127X_FOM_OOKPEAK                0x14 /* FSK/OOK: OOK demodulator */
#define SX127X_FOM_OOKFIX                 0x15 /* FSK/OOK: Threshold of the OOK demod */
#define SX127X_FOM_OOKAVG                 0x16 /* FSK/OOK: Average of the OOK demod */
#define SX127X_FOM_AFCFEI                 0x1A /* FSK/OOK: AFC and FEI control */
#define SX127X_FOM_AFCMSB                 0x1B /* FSK/OOK: Frequency correction value of the AFC MSB */
#define SX127X_FOM_AFCLSB                 0x1C /* FSK/OOK: Frequnecy correction value of the AFC LSB */
#define SX127X_FOM_FEIMSB                 0x1D /* FSK/OOK: Value of the calculated frequency error MSB */
#define SX127X_FOM_FEILSB                 0x1E /* FSK/OOK: Value of the calculated frequency error LSB */
#define SX127X_FOM_PREDET                 0x1F /* FSK/OOK: Settings of the Preamble Detector */
#define SX127X_FOM_RXTIMEOUT1             0x20 /* FSK/OOK: Timeout RX request and RSSI */
#define SX127X_FOM_RXTIMEOUT2             0x21 /* FSK/OOK: Timeout RSSI and PayloadReady */
#define SX127X_FOM_RXTIMEOUT3             0x22 /* FSK/OOK: Timeout RSSI and SyncAddress */
#define SX127X_FOM_RXDELAY                0x23 /* FSK/OOK: Delay between RX cycles */
#define SX127X_FOM_OSC                    0x24 /* FSK/OOK: RC oscillators settings, CLKOUT frequency */
#define SX127X_FOM_PREMSB                 0x25 /* FSK/OOK: Preamble length MSB */
#define SX127X_FOM_PRELSB                 0x26 /* FSK/OOK: Preamble length LSB */
#define SX127X_FOM_SYNCCFG                0x27 /* FSK/OOK: Sync Word Recognition control */
#define SX127X_FOM_SYNCVAL1               0x28 /* FSK/OOK: Sync Word bytes 1 */
#define SX127X_FOM_SYNCVAL2               0x29 /* FSK/OOK: Sync Word bytes 2 */
#define SX127X_FOM_SYNCVAL3               0x2a /* FSK/OOK: Sync Word bytes 3 */
#define SX127X_FOM_SYNCVAL4               0x2b /* FSK/OOK: Sync Word bytes 4 */
#define SX127X_FOM_SYNCVAL5               0x2c /* FSK/OOK: Sync Word bytes 5 */
#define SX127X_FOM_SYNCVAL6               0x2d /* FSK/OOK: Sync Word bytes 6 */
#define SX127X_FOM_SYNCVAL7               0x2e /* FSK/OOK: Sync Word bytes 7 */
#define SX127X_FOM_SYNCVAL8               0x2f /* FSK/OOK: Sync Word bytes 8 */
#define SX127X_FOM_PKTCFG1                0x30 /* FSK/OOK: Parcket mode settings 1 */
#define SX127X_FOM_PKTCFG2                0x31 /* FSK/OOK: Packet mode settings 2 */
#define SX127X_FOM_PAYLOADLEN             0x32 /* FSK/OOK: Payload length setting */
#define SX127X_FOM_NODEADDR               0x33 /* FSK/OOK: Node address */
#define SX127X_FOM_BROADCAST              0x34 /* FSK/OOK: Broadcast address */
#define SX127X_FOM_FIFOTHR                0x35 /* FSK/OOK: FIFO threshold, TX start condition */
#define SX127X_FOM_SEQCFG1                0x36 /* FSK/OOK: Top level Sequencer settings 1 */
#define SX127X_FOM_SEQCFG2                0x37 /* FSK/OOK: Top level Sequencer settings 2 */
#define SX127X_FOM_TIMRES                 0x38 /* FSK/OOK: Timer 1 and 2 resolution control */
#define SX127X_FOM_TIMER1COEF             0x39 /* FSK/OOK: Timer 1 setting */
#define SX127X_FOM_TIMER2COEF             0x3a /* FSK/OOK: Timer 2 setting */
#define SX127X_FOM_IMAGECAL               0x3b /* FSK/OOK: Image calibration engine control */
#define SX127X_FOM_TEMP                   0x3c /* FSK/OOK: Temperature Sensor value */
#define SX127X_FOM_LOWBAT                 0x3d /* FSK/OOK: Low Battery Indicator settings */
#define SX127X_FOM_IRQ1                   0x3e /* FSK/OOK: Status register 1: PLL Lock state, Timeout, RSSI */
#define SX127X_FOM_IRQ2                   0x3f /* FSK/OOK: Status register 2: FIFO handling flags, Low Battery */
#define SX127X_FOM_PLLHOP                 0x44 /* FSK/OOK: Control the fast frequency hopping mode */
#define SX127X_FOM_BITRATEFRAC            0x5d /* FSK/OOK: Fractional part in the Bit Rate division ratio */

/* Registers specific for LORA Mode */

#define SX127X_LRM_ADDRPTR                0x0d /* LORA: FIFO SPI pointer */
#define SX127X_LRM_TXBASE                 0x0e /* LORA: Start TX data */
#define SX127X_LRM_RXBASE                 0x0f /* LORA: Start RX data */
#define SX127X_LRM_RXCURR                 0x10 /* LORA: Start address of last packet received */
#define SX127X_LRM_IRQMASK                0x11 /* LORA: Optional IRQ flag mask */
#define SX127X_LRM_IRQ                    0x12 /* LORA: IRQ flags */
#define SX127X_LRM_RXBYTES                0x13 /* LORA: Number of received bytes */
#define SX127X_LRM_RXHDRMSB               0x14 /* LORA: Number of valid headers received MSB */
#define SX127X_LRM_RXHDRLSB               0x15 /* LORA: Number of valid headers received LSB */
#define SX127X_LRM_RXPKTMSB               0x16 /* LORA: Number of valid packets received MSB */
#define SX127X_LRM_RXPKTLSB               0x17 /* LORA: Number of valid packets received LSB */
#define SX127X_LRM_MODSTAT                0x18 /* LORA: Live LORA modem status */
#define SX127X_LRM_PKTSNR                 0x19 /* LORA: Estimation of last packet SNR */
#define SX127X_LRM_PKTRSSI                0x1A /* LORA: RSSI of last packet */
#define SX127X_LRM_RSSIVAL                0x1B /* LORA: Current RSSI */
#define SX127X_LRM_HOPCHAN                0x1C /* LORA: FHSS start channel */
#define SX127X_LRM_MDMCFG1                0x1D /* LORA: Modem PHY config 1 */
#define SX127X_LRM_MDMCFG2                0x1E /* LORA: Modem PHY config 2 */
#define SX127X_LRM_RXTIMEOUTLSB           0x1F /* LORA: Receiver timeout value LSB */
#define SX127X_LRM_PREMSB                 0x20 /* LORA: Size of preamble MSB */
#define SX127X_LRM_PRELSB                 0x21 /* LORA: Size of preamble LSB */
#define SX127X_LRM_PAYLOADLEN             0x22 /* LORA: LORA payload length */
#define SX127X_LRM_PAYLOADMAX             0x23 /* LORA: LORA maximum payload length */
#define SX127X_LRM_HOPPER                 0x24 /* LORA: FHSS Hop period */
#define SX127X_LRM_RXFIFOADDR             0x25 /* LORA: Address of last byte written in FIFO */
#define SX127X_LRM_MODEMCFG3              0x26 /* LORA: Modem PHY confgi 3*/
#define SX127X_LRM_FEIMSB                 0x28 /* LORA: Estimated frequency error MSB */
#define SX127X_LRM_FEIMID                 0x29 /* LORA: Estimated frequency error, MID */
#define SX127X_LRM_FEILSB                 0x2a /* LORA: Estimated frequency error, LSB*/
#define SX127X_LRM_RSSIWIDEBAND           0x2c /* LORA: Wideband RSSI measurement */
#define SX127X_LRM_DETECTOPT              0x31 /* LORA: LORA detection optimize for SF6 */
#define SX127X_LRM_INVERTIQ               0x33 /* LORA: Invert LORA I and Q signals */
#define SX127X_LRM_DETECTTHR              0x37 /* LORA: LORA detection threshold for SF6 */
#define SX127X_LRM_SYNCWORD               0x39 /* LORA: LORA Sync Word */

/* Common *******************************************************************/

/* FSK/OOK/LORA: FIFO read/write access */

#define SX127X_CMN_FIFO_MASK              0xff

/* Operating mode & LORA/FSK selection */

#define SX127X_CMN_OPMODE_MODE_SHIFT      (0)                                 /* Bits 0-2: Transceiver mode */
#define SX127X_CMN_OPMODE_MODE_MASK       (7 << SX127X_CMN_OPMODE_MODE_SHIFT)
#  define SX127X_CMN_OPMODE_MODE_SLEEP    (0 << SX127X_CMN_OPMODE_MODE_SHIFT) /* SLEEP */
#  define SX127X_CMN_OPMODE_MODE_STBY     (1 << SX127X_CMN_OPMODE_MODE_SHIFT) /* STDBY */
#  define SX127X_CMN_OPMODE_MODE_FSTX     (2 << SX127X_CMN_OPMODE_MODE_SHIFT) /* FSTX */
#  define SX127X_CMN_OPMODE_MODE_TX       (3 << SX127X_CMN_OPMODE_MODE_SHIFT) /* TX */
#  define SX127X_CMN_OPMODE_MODE_FSRX     (4 << SX127X_CMN_OPMODE_MODE_SHIFT) /* FSRX */
#  define SX127X_CMN_OPMODE_MODE_RX       (5 << SX127X_CMN_OPMODE_MODE_SHIFT) /* RX in FSK/OOK, RXCONTINOUS in LORA */
#  define SX127X_CMN_OPMODE_MODE_RXSINGLE (6 << SX127X_CMN_OPMODE_MODE_SHIFT) /* RXSINGLE (only LORA) */
#  define SX127X_CMN_OPMODE_MODE_CAD      (7 << SX127X_CMN_OPMODE_MODE_SHIFT) /* CAD (only LORA) */
#define SX127X_CMN_OPMODE_LFMODEON        (1 << 3)                            /* Bit 3: Low Frequency Mode ON */
#define SX127X_CMN_OPMODE_MODTYPE_SHIFT   (5)                                 /* Bits 5-6: Modulation type (only FSK/OOK) */
#  define SX127X_CMN_OPMODE_MODTYPE_MASK  (3 << SX127X_CMN_OPMODE_MODTYPE_SHIFT)
#  define SX127X_CMN_OPMODE_MODTYPE_FSK   (0 << SX127X_CMN_OPMODE_MODTYPE_SHIFT)
#  define SX127X_CMN_OPMODE_MODTYPE_OOK   (1 << SX127X_CMN_OPMODE_MODTYPE_SHIFT)
#define SX127X_CMN_OPMODE_LRMODE          (1 << 7)                            /* Bit 7: Long Range Mode 0-FSK/OOK, 1-LORA */

/* FSK/OOK/LORA: RF carrier frequency */

#define SX127X_CMN_FRF_MAX                (0xffffff)
#define SX127X_FRF_FROM_FREQ(freq)        (freq/SX127X_FSTEP)
#define SX127X_CMN_FRF_MSB(frf)           ((frf >> 16) & 0xff)
#define SX127X_CMN_FRF_MID(frf)           ((frf >> 8) & 0xff)
#define SX127X_CMN_FRF_LSB(frf)           ((frf >> 0) & 0xff)

/* FSK/OOK/LORA: PA selection and Output Power control */

#define SX127X_CMN_PACFG_OUTPOWER_SHIFT   (0)      /* Bits 0-4: Pout */
#define SX127X_CMN_PACFG_OUTPOWER_MASK    (15 << SX127X_CMN_PACFG_OUTPOWER_SHIFT)
#define SX127X_CMN_PACFG_MAXPOWER_SHIFT   (4)      /* Bits 4-6: Max power output: Pmax=10.8 +0.6*MaxPower */
#define SX127X_CMN_PACFG_MAXPOWER_MASK    (7 << SX127X_CMN_PACFG_MAXPOWER_SHIFT)
#define SX127X_CMN_PACFG_PASELECT         (1 << 7) /* Bit 7: PA output pin */

/* FSK/OOK/LORA: Control of PA ramp time, low phase noise PLL */

#define SX127X_CMN_PARAMP_PARAMP_SHIFT    (0)      /* Bits 0-3: Rise/fall time of ramp up/down */
#define SX127X_CMN_PARAMP_PARAMP_MASK     (15 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_3p4ms  (0 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_2ms    (1 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_1ms    (2 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_500us  (3 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_250us  (4 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_125us  (5 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_100us  (6 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_62us   (7 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_50us   (8 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_40us   (9 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_31us   (10 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_25us   (11 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_20us   (12 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_15us   (13 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_12us   (14 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_PARAMP_11us   (15 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#define SX127X_CMN_PARAMP_SHAPING_SHIFT   (5)      /* Bits 5-6: Data shaping */
#define SX127X_CMN_PARAMP_SHAPING_MASK    (3 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_SHAPING_NONE  (0 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_SHAPING_1     (1 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_SHAPING_2     (2 << SX127X_CMN_PARAMP_PARAMP_SHIFT)
#  define SX127X_CMN_PARAMP_SHAPING_3     (3 << SX127X_CMN_PARAMP_PARAMP_SHIFT)

/* FSK/OOK/LORA: Over Current Protection control */

#define SX127X_CMN_OCP_SHIFT              (0)      /* Bits 0-4: Trimming of OCP current */
#define SX127X_CMN_OCP_MASK               (15 << SX127X_CMN_OCP_SHIFT)
#define SX127X_CMN_OCP_OCPON              (1 << 5) /* Bit 5: Enable overload current protection for the PA */

/* FSK/OOK/LORA: LNA settings */

#define SX127X_CMN_LNA_BOOSTHF_SHIFT      0        /* Bits 0-1: HF LNA current adjustment */
#define SX127X_CMN_LNA_BOOSTHF_MASK       (3 << SX127X_CMN_LNA_BOOSTHF_SHIFT)
#  define SX127X_CMN_LNA_BOOSTHF_OFF      (0 << SX127X_CMN_LNA_BOOSTHF_SHIFT)
#  define SX127X_CMN_LNA_BOOSTHF_ON       (3 << SX127X_CMN_LNA_BOOSTHF_SHIFT)
#define SX127X_CMN_LNA_BOOSTLF_SHIFT      3        /* Bits 3-4: LF LNA current adjustment */
#define SX127X_CMN_LNA_BOOSTLF_MASK       (3 << SX127X_CMN_LNA_BOOSTLF_SHIFT)
#  define SX127X_CMN_LNA_BOOSTLF_OFF      (0 << SX127X_CMN_LNA_BOOSTLF_SHIFT)
#define SX127X_CMN_LNA_GAIN_SHIFT         5        /* Bits 5-7: LNA gain setting */
#define SX127X_CMN_LNA_GAIN_MASK          (7 << SX127X_CMN_LNA_GAIN_SHIFT)
#  define SX127X_CMN_LNA_GAIN_1           (0 << SX127X_CMN_LNA_GAIN_SHIFT)
#  define SX127X_CMN_LNA_GAIN_2           (1 << SX127X_CMN_LNA_GAIN_SHIFT)
#  define SX127X_CMN_LNA_GAIN_3           (2 << SX127X_CMN_LNA_GAIN_SHIFT)
#  define SX127X_CMN_LNA_GAIN_4           (3 << SX127X_CMN_LNA_GAIN_SHIFT)
#  define SX127X_CMN_LNA_GAIN_5           (4 << SX127X_CMN_LNA_GAIN_SHIFT)
#  define SX127X_CMN_LNA_GAIN_6           (5 << SX127X_CMN_LNA_GAIN_SHIFT)
#  define SX127X_CMN_LNA_GAIN_7           (6 << SX127X_CMN_LNA_GAIN_SHIFT)

/* FSK/OOK/LORA: Mapping of pins DIO0 to DIO3 */

#define SX127X_CMN_DIOMAP1_DIO3_SHIFT     (0)      /* Bits 0-1: */
#define SX127X_CMN_DIOMAP1_DIO3_MASK      (3 << SX127X_CMN_DIOMAP1_DIO3_SHIFT)
#define SX127X_CMN_DIOMAP1_DIO2_SHIFT     (2)      /* Bits 2-3: */
#define SX127X_CMN_DIOMAP1_DIO2_MASK      (3 << SX127X_CMN_DIOMAP1_DIO2_SHIFT)
#define SX127X_CMN_DIOMAP1_DIO1_SHIFT     (4)      /* Bits 4-5: */
#define SX127X_CMN_DIOMAP1_DIO1_MASK      (3 << SX127X_CMN_DIOMAP1_DIO1_SHIFT)
#define SX127X_CMN_DIOMAP1_DIO0_SHIFT     (6)      /* Bits 6-7: */
#define SX127X_CMN_DIOMAP1_DIO0_MASK      (3 << SX127X_CMN_DIOMAP1_DIO0_SHIFT)
#  define SX127X_FOM_DIOMAP1_DIO0_RXTX    (0 << SX127X_CMN_DIOMAP1_DIO0_SHIFT)
#  define SX127X_LRM_DIOMAP1_DIO0_RXDONE  (0 << SX127X_CMN_DIOMAP1_DIO0_SHIFT)
#  define SX127X_LRM_DIOMAP1_DIO0_TXDONE  (1 << SX127X_CMN_DIOMAP1_DIO0_SHIFT)
#  define SX127X_LRM_DIOMAP1_DIO0_CADDONE (2 << SX127X_CMN_DIOMAP1_DIO0_SHIFT)

/* FSK/OOK/LORA: Mapping of pins DIO4 and DIO5, ClkOut freq */

#define SX127X_CMN_DIOMAP2_MPD            (1 << 0) /* Bit 0: MapPreambleDetect */
#define SX127X_CMN_DIOMAP2_DIO5_SHIFT     (4)      /* Bits 4-5: */
#define SX127X_CMN_DIOMAP2_DIO5_MASK      (3 << SX127X_CMN_DIOMAP2_DIO5_SHIFT)
#define SX127X_CMN_DIOMAP2_DIO4_SHIFT     (6)      /* Bits 6-7: */
#define SX127X_CMN_DIOMAP2_DIO4_MASK      (3 << SX127X_CMN_DIOMAP2_DIO4_SHIFT)

/* FSK/OOK/LORA: Semtech ID relating the silicon version */

#define SX127X_CMN_VERSION_MASK           0xff

/* FSK/OOK/LORA: TCXO or XTAL input setting */

#define SX127X_CMN_TCXO_INPUTON           (1 << 4)

/* FSK/OOK/LORA: Higher power settings of the PA */

#define SX127X_CMN_PADAC_DEFAULT          0x04
#define SX127X_CMN_PADAC_BOOST            0x07

/* FSK/OOK/LORA: Stored temperature during the former IQ calibration */

#define SX127X_CMN_FTEMP_MASK             0xff

/* FSK/OOK/LORA: AGC thresholds reference */

#define SX127X_CMN_AGCREF_REF_SHIFT       0        /* Bits 0-5: */
#define SX127X_CMN_AGCREF_REF_MASK        (63 << SX127X_CMN_AGCREF_REF_SHIFT)
#define SX127X_CMN_AGCREF_REF(v)          ((v << SX127X_CMN_AGCREF_REF_SHIFT) & SX127X_CMN_AGCREF_REF_MASK)

/* FSK/OOK/LORA: AGC thresholds 1 */

#define SX127X_CMN_AGCTHR1_STEP1_SHIFT    0        /* Bits 0-4: 1st AGC threshold */
#define SX127X_CMN_AGCTHR1_STEP1_MASK     (31 << SX127X_CMN_AGCTHR1_STEP1_SHIFT)
#define SX127X_CMN_AGCTHR1_STEP1(v)       ((v << SX127X_CMN_AGCTHR1_STEP1_SHIFT) & SX127X_CMN_AGCTHR1_STEP1_MASK)

/* FSK/OOK/LORA: AGC thresholds 2 */

#define SX127X_CMN_AGCTHR2_STEP3_SHIFT    0        /* Bits 0-3: 3rd AGC threshold */
#define SX127X_CMN_AGCTHR2_STEP3_MASK     (15 << SX127X_CMN_AGCTHR1_STEP3_SHIFT)
#define SX127X_CMN_AGCTHR2_STEP3(v)       ((v << SX127X_CMN_AGCTHR2_STEP3_SHIFT) & SX127X_CMN_AGCTHR2_STEP3_MASK)
#define SX127X_CMN_AGCTHR2_STEP2_SHIFT    4        /* Bits 4-7: 2nd AGC threshold */
#define SX127X_CMN_AGCTHR2_STEP2_MASK     (15 << SX127X_CMN_AGCTHR1_STEP2_SHIFT)
#define SX127X_CMN_AGCTHR2_STEP2(v)       ((v << SX127X_CMN_AGCTHR2_STEP2_SHIFT) & SX127X_CMN_AGCTHR2_STEP2_MASK)

/* FSK/OOK/LORA: AGC thresholds 3 */

#define SX127X_CMN_AGCTHR3_STEP5_SHIFT    0        /* Bits 0-3: 5th AGC threshold */
#define SX127X_CMN_AGCTHR3_STEP5_MASK     (15 << SX127X_CMN_AGCTHR3_STEP5_SHIFT)
#define SX127X_CMN_AGCTHR3_STEP5(v)       ((v << SX127X_CMN_AGCTHR3_STEP5_SHIFT) & SX127X_CMN_AGCTHR3_STEP5_MASK)
#define SX127X_CMN_AGCTHR3_STEP4_SHIFT    4        /* Bits 4-7: 4th AGC threshold */
#define SX127X_CMN_AGCTHR3_STEP4_MASK     (15 << SX127X_CMN_AGCTHR3_STEP4_SHIFT)
#define SX127X_CMN_AGCTHR3_STEP4(v)       ((v << SX127X_CMN_AGCTHR3_STEP4_SHIFT) & SX127X_CMN_AGCTHR3_STEP4_MASK)

/* FSK/OOK/LORA: Control of the PLL bandwidth  */

#define SX127X_CMN_PLL_BW_SHIFT           (6)      /* Bits 6-7: PLL bandwidth */
#define SX127X_CMN_PLL_BW_MASK            (3 << SX127X_CMN_PLL_BW_SHIFT)
#  define SX127X_CMN_PLL_BW_75kHz         (0 << SX127X_CMN_PLL_BW_SHIFT)
#  define SX127X_CMN_PLL_BW_150kHz        (1 << SX127X_CMN_PLL_BW_SHIFT)
#  define SX127X_CMN_PLL_BW_225kHz        (2 << SX127X_CMN_PLL_BW_SHIFT)
#  define SX127X_CMN_PLL_BW_300kHz        (3 << SX127X_CMN_PLL_BW_SHIFT)

/* FSK/OOK ******************************************************************/

/* FSK/OOK: Bit Rate setting */

#define SX127X_FOM_BITRATE_MAX            (0xffff)
#define SX127X_FOM_BITRATE_MSB(v)         ((v >> 8) & 0xff)
#define SX127X_FOM_BITRATE_LSB(v)         ((v >> 0) & 0xff)

/* FSK/OOK: Frequency Deviation setting */

#define SX127X_FOM_FDEV_MSB_MASK          (0x3f)
#define SX127X_FOM_FDEV_MAX               (0x3fff)
#define SX127X_FDEV_FROM_FREQ(freq)       (freq/SX127X_FSTEP)
#define SX127X_FOM_FDEV_MSB(v)            ((v >> 8) & 0xff)
#define SX127X_FOM_FDEV_LSB(v)            ((v >> 0) & 0xff)

/* FSK/OOK: AFC, AGC, ctrl */

#define SX127X_FOM_RXCFG_TRG_SHIFT        (0)                               /* Bits 0-2: RX trigger */
#define SX127X_FOM_RXCFG_TRG_MASK         (7 << SX127X_FOM_RXCFG_TRG_SHIFT)
#  define SX127X_FOM_RXCFG_TRG_NONE       (0 << SX127X_FOM_RXCFG_TRG_SHIFT) /* 000: */
#  define SX127X_FOM_RXCFG_TRG_RSSI       (1 << SX127X_FOM_RXCFG_TRG_SHIFT) /* 001: */
#  define SX127X_FOM_RXCFG_TRG_PREDET     (6 << SX127X_FOM_RXCFG_TRG_SHIFT) /* 110: */
#  define SX127X_FOM_RXCFG_TRG_RSSIPREDET (7 << SX127X_FOM_RXCFG_TRG_SHIFT) /* 111: */
#define SX127X_FOM_RXCFG_AGCAUTOON        (1 << 3)                          /* Bit 3: AGC auto ON */
#define SX127X_FOM_RXCFG_AFCAUTOON        (1 << 4)                          /* Bit 4: AFC auto ON */
#define SX127X_FOM_RXCFG_RESRXWITHPLL     (1 << 5)                          /* Bit 5: Restar RX with PLL lock */
#define SX127X_FOM_RXCFG_RESRXWITHOUTPLL  (1 << 6)                          /* Bit 6: Restart RX without PLL lock */
#define SX127X_FOM_RXCFG_RESRXONCOLLSION  (1 << 7)                          /* Bit 7: Restart RX on collision */

/* FSK/OOK: RSSI */

#define SX127X_FOM_RSSICFG_SMOOTH_SHIFT   (0)       /* Bits 0-2: RSSI smoothing */
#define SX127X_FOM_RSSICFG_SMOOTH_MASK    (7 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_2     (0 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_4     (1 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_8     (2 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_16    (3 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_32    (4 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_64    (5 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_128   (6 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#  define SX127X_FOM_RSSICFG_SMOOTH_256   (7 << SX127X_FOM_RSSICFG_SMOOTH_SHIFT)
#define SX127X_FOM_RSSICFG_OFFSET_SHIFT   (3)       /* Bits 3-7: RSSI offset */
#define SX127X_FOM_RSSICFG_OFFSET_MASK    (15 << SX127X_FOM_RSSICFG_OFFSET_SHIFT)

/* FSK/OOK: RSSI value in dBm */

#define SX127X_FOM_RSSIVAL_GET(rssi)      (-rssi/2)

/* FSK/OOK: Channel Filter BW Control */

#define SX127X_FOM_RXBW_EXP_SHIFT         (0)      /* Bits 0-2: RXBW exp */
#define SX127X_FOM_RXBW_EXP_MASK          (7 << SX127X_FOM_RXBW_EXP_SHIFT)
#define SX127X_FOM_RXBW_MANT_SHIFT        (3)      /* Bits 3-4: RXBW mant */
#define SX127X_FOM_RXBW_MANT_MASK         (3 << SX127X_FOM_RXBW_MANT_SHIFT)
#  define SX127X_FOM_RXBW_MANT_16         (0 << SX127X_FOM_RXBW_MANT_SHIFT)
#  define SX127X_FOM_RXBW_MANT_20         (1 << SX127X_FOM_RXBW_MANT_SHIFT)
#  define SX127X_FOM_RXBW_MANT_24         (2 << SX127X_FOM_RXBW_MANT_SHIFT)

#define FSKOOK_BANDWIDTH_GET(mant, exp)   (((exp << SX127X_FOM_RXBW_EXP_SHIFT) & SX127X_FOM_RXBW_EXP_MASK) | \
                                           ((mant << SX127X_FOM_RXBW_MANT_SHIFT) & SX127X_FOM_RXBW_MANT_MASK))

/* FSK/OOK: AFC Channel Filter BW */

#define SX127X_FOM_AFCBW_EXP_SHIFT        (0)      /* Bits 0-2: AFC exp */
#define SX127X_FOM_AFCBW_EXP_MASK         (7 << SX127X_FOM_AFCBW_EXP_SHIFT)
#define SX127X_FOM_AFCBW_MANT_SHIFT       (3)      /* Bits 3-4: AFC mant */
#define SX127X_FOM_AFCBW_MANT_MASK        (7 << SX127X_FOM_AFCBW_MANT_SHIFT)

/* FSK/OOK: OOK demodulator */

#define SX127X_FOM_OOKPEAK_THRSTEP_SHIFT  (0)      /* Bits 0-2: OOK peak threshold step */
#define SX127X_FOM_OOKPEAK_THRSTEP_MASK   (7 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_0p5  (0 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_1p0  (1 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_1p5  (2 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_2p0  (3 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_3p0  (4 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_4p0  (5 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_5p0  (6 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRSTEP_6p0  (7 << SX127X_FOM_OOKPEAK_THRSTEP_SHIFT)
#define SX127X_FOM_OOKPEAK_THRTYPE_SHIFT  (3)      /* Bits 3-4: OOK threshold type */
#define SX127X_FOM_OOKPEAK_THRTYPE_MASK   (7 << SX127X_FOM_OOKPEAK_STEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRTYPE_FIX  (0 << SX127X_FOM_OOKPEAK_STEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRTYPE_PEAK (1 << SX127X_FOM_OOKPEAK_STEP_SHIFT)
#  define SX127X_FOM_OOKPEAK_THRTYPE_AVG  (2 << SX127X_FOM_OOKPEAK_STEP_SHIFT)
#define SX127X_FOM_OOKPEAK_BITSYNCON      (1 << 5) /* Bit 5: Enable bit synchronizer */

/* FSK/OOK: Average of the OOK demod */

#define SX127X_FOM_OOKAVG_THRFILT_SHIFT   (0)      /* Bits 0-1: OOK average threshold filter */
#define SX127X_FOM_OOKAVG_THRFILT_MASK    (3 << SX127X_FOM_OOKAVG_THRFILT_SHIFT)
#  define SX127X_FOM_OOKAVG_THRFILT_32    (0 << SX127X_FOM_OOKAVG_THRFILT_SHIFT)
#  define SX127X_FOM_OOKAVG_THRFILT_8     (1 << SX127X_FOM_OOKAVG_THRFILT_SHIFT)
#  define SX127X_FOM_OOKAVG_THRFILT_4     (2 << SX127X_FOM_OOKAVG_THRFILT_SHIFT)
#  define SX127X_FOM_OOKAVG_THRFILT_2     (3 << SX127X_FOM_OOKAVG_THRFILT_SHIFT)
#define SX127X_FOM_OOKAVG_OFFSET_SHIFT    (2)      /* Bits 2-3: OOK average offset */
#define SX127X_FOM_OOKAVG_OFFSET_MASK     (3 << SX127X_FOM_OOKAVG_OFFSET_SHIFT)
#  define SX127X_FOM_OOKAVG_OFFSET_0      (0 << SX127X_FOM_OOKAVG_OFFSET_SHIFT)
#  define SX127X_FOM_OOKAVG_OFFSET_2      (1 << SX127X_FOM_OOKAVG_OFFSET_SHIFT)
#  define SX127X_FOM_OOKAVG_OFFSET_4      (2 << SX127X_FOM_OOKAVG_OFFSET_SHIFT)
#  define SX127X_FOM_OOKAVG_OFFSET_6      (3 << SX127X_FOM_OOKAVG_OFFSET_SHIFT)
#define SX127X_FOM_OOKAVG_THRDEC_SHIFT    (5)      /* Bits 5-7: OOK peak threshold decrement */
#define SX127X_FOM_OOKAVG_THRDEC_MASK     (7 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_1      (0 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_2      (1 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_3      (2 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_4      (3 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_5      (4 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_6      (5 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_7      (6 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)
#  define SX127X_FOM_OOKAVG_THRDEC_8      (7 << SX127X_FOM_OOKAVG_THRDEC_SHIFT)

/* FSK/OOK: AFC and FEI control */

#define SX127X_FOM_AFCFEI_AUTOCLEARON     (1 << 0) /* Bit 0: AFC auto clear ON */
#define SX127X_FOM_AFCFEI_AFCCLEAR        (1 << 1) /* Bit 1: AFC clear */
#define SX127X_FOM_AFCFEI_AGCSTART        (1 << 2) /* Bit 2: AGC start */

/* FSK/OOK: Frequency correction value of the AFC */

#define SX127X_FOM_AFC_MAX                (0xffff)
#define SX127X_FOM_AFC_MSB(afc)           ((afc >> 8) & 0xff)
#define SX127X_FOM_AFC_LSB(afc)           ((afc >> 0) & 0xff)

/* FSK/OOK: Value of the calculated frequency error */

#define SX127X_FOM_FEI_MAX                (0xffff)
#define SX127X_FOM_FEI_MSB(fei)           ((fei >> 8) & 0xff)
#define SX127X_FOM_FEI_LSB(fei)           ((fei >> 0) & 0xff)

/* FSK/OOK: Settings of the Preamble Detector */

#define SX127X_FOM_PREDET_TOL_SHIFT       (0)      /* Bits 0-4: Preamble detector tolerance */
#define SX127X_FOM_PREDET_TOL_MASK        (31 << SX127X_FOM_PREDET_TOL_SHIFT)
#define SX127X_FOM_PREDET_TOL(tol)        ((tol << SX127X_FOM_PREDET_TOL_SHIFT) & SX127X_FOM_PREDET_TOL_MASK)
#define SX127X_FOM_PREDET_SIZE_SHIFT      (5)      /* Bits 5-6: Preamble detector size */
#define SX127X_FOM_PREDET_SIZE_MASK       (3 << SX127X_FOM_PREDET_SIZE_SHIFT)
#  define SX127X_FOM_PREDET_SIZE_1B       (0 << SX127X_FOM_PREDET_SIZE_SHIFT)
#  define SX127X_FOM_PREDET_SIZE_2B       (1 << SX127X_FOM_PREDET_SIZE_SHIFT)
#  define SX127X_FOM_PREDET_SIZE_3B       (2 << SX127X_FOM_PREDET_SIZE_SHIFT)
#define SX127X_FOM_PREDET_ON              (1 << 7) /* Bit 7: Preamble detector ON */

/* FSK/OOK: RC oscillators settings, CLKOUT frequency */

#define SX127X_FOM_OSC_CLKOUT_SHIFT       (0)      /* Bits 0-2: CLKOUT frequency */
#define SX127X_FOM_OSC_CLKOUT_MASK        (7 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_FXOSC     (0 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_FXOSCd2   (1 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_FXOSCd4   (2 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_FXOSCd8   (3 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_FXOSCd16  (4 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_FXOSCd32  (5 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_RC        (6 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#  define SX127X_FOM_OSC_CLKOUT_OFF       (7 << SX127X_FOM_OSC_CLKOUT_SHIFT)
#define SX127X_FOM_OSC_CLKOUT_RCCALSTART  (3 << 1) /* Bit 3: Trigger the RC oscilator calibration */

/* FSK/OOK: Preamble length MSB */

#define SX127X_FOM_PRE_MAX                (0xffff)
#define SX127X_FOM_PRE_MSB(pre)           ((pre >> 8) & 0xff)
#define SX127X_FOM_PRE_LSB(pre)           ((pre >> 0) & 0xff)

/* FSK/OOK: Sync Word Recognition control */

#define SX127X_FOM_SYNCCFG_SYNCSIZE_SHIFT (0)      /* Bits 0-2: Sync word size */
#define SX127X_FOM_SYNCCFG_SYNCSIZE_MASK  (7 << SX127X_FOM_SYNCCFG_SYNCSIZE_SHIFT)
#define SX127X_FOM_SYNCCFG_SYNCSIZE(s)    (((s) << SX127X_FOM_SYNCCFG_SYNCSIZE_SHIFT) & SX127X_FOM_SYNCCFG_SYNCSIZE_MASK)
#define SX127X_FOM_SYNCCFG_SYNCON         (1 << 4) /* Bit 4: Sync word generation and detection enable */
#define SX127X_FOM_SYNCCFG_PREPOL         (1 << 5) /* Bit 5: Preamble Polarity */
#define SX127X_FOM_SYNCSIZE_MAX           (8)

/* FSK/OOK: Sync Word bytes */

#define SX127X_FOM_SYNCVAL_1(val)         ((val >> 56) & 0xff)
#define SX127X_FOM_SYNCVAL_2(val)         ((val >> 48) & 0xff)
#define SX127X_FOM_SYNCVAL_3(val)         ((val >> 40) & 0xff)
#define SX127X_FOM_SYNCVAL_4(val)         ((val >> 32) & 0xff)
#define SX127X_FOM_SYNCVAL_5(val)         ((val >> 24) & 0xff)
#define SX127X_FOM_SYNCVAL_6(val)         ((val >> 16) & 0xff)
#define SX127X_FOM_SYNCVAL_7(val)         ((val >> 8) & 0xff)
#define SX127X_FOM_SYNCVAL_8(val)         ((val >> 0) & 0xff)

/* FSK/OOK: Packet mode settings 1 */

#define SX127X_FOM_PKTCFG1_CRCTYPE        (1 << 0)                               /* Bit 0: CRC type: 0 -> CCITT CRC, 1 -> IBM CRC with alternate whitening */
#define SX127X_FOM_PKTCFG1_ADDRFLT_SHIFT  (1)                                    /* Bits 1-2: Address basef filtering in RX */
#define SX127X_FOM_PKTCFG1_ADDRFLT_MASK   (3 << SX127X_FOM_PKTCFG1_ADDRFLT_SHIFT)
#  define SX127X_FOM_PKTCFG1_ADDRFLT_OFF  (0 << SX127X_FOM_PKTCFG1_ADDRFLT_SHIFT)
#  define SX127X_FOM_PKTCFG1_ADDRFLT_NA   (1 << SX127X_FOM_PKTCFG1_ADDRFLT_SHIFT)
#  define SX127X_FOM_PKTCFG1_ADDRFLT_NABA (2 << SX127X_FOM_PKTCFG1_ADDRFLT_SHIFT)
#define SX127X_FOM_PKTCFG1_CRCAUTOCLROFF  (1 << 3)                               /* Bit 3: CRC auto clear OFF */
#define SX127X_FOM_PKTCFG1_CRCON          (1 << 4)                               /* Bit 4: TX/RX CRC enable */
#define SX127X_FOM_PKTCFG1_DCFREE_SHIFT   (5)                                    /* Bits 5-6: DC-free encodeing/decoding */
#define SX127X_FOM_PKTCFG1_DCFREE_MASK    (3 << SX127X_FOM_PKTCFG1_DCFREE_SHIFT)
#  define SX127X_FOM_PKTCFG1_DCFREE_OFF   (0 << SX127X_FOM_PKTCFG1_DCFREE_SHIFT) /* 00: None */
#  define SX127X_FOM_PKTCFG1_DCFREE_M     (1 << SX127X_FOM_PKTCFG1_DCFREE_SHIFT) /* 01: Manchaster */
#  define SX127X_FOM_PKTCFG1_DCFREE_W     (2 << SX127X_FOM_PKTCFG1_DCFREE_SHIFT) /* 10: Whitening */
#define SX127X_FOM_PKTCFG1_PCKFORMAT      (1 << 7)                               /* Bit 7: 0 -> fixed length, 1 -> variable length*/

/* FSK/OOK: Packet mode settings 2 */

#define SX127X_FOM_PKTCFG2_PLENMSB_SHIFT  (0)      /* Bits 0-2: Packet length MSB */
#define SX127X_FOM_PKTCFG2_PLENMSB_MASK   (7 << SX127X_FOM_PKTCFG2_PLEN_SHIFT)
#define SX127X_FOM_PKTCFG2_PLENMSB(plen)  ((plen >> 8) & SX127X_FOM_PKTCFG2_PLENMSB_MASK)
#define SX127X_FOM_PKTCFG2_BEACONON       (1 << 3) /* Bit 3: Beacon mode in fixed packed format */
#define SX127X_FOM_PKTCFG2_IHPF           (1 << 4) /* Bit 4: reserved */
#define SX127X_FOM_PKTCFG2_IOHOMEON       (1 << 5) /* Bit 5: IO-HOMECONTROL compatibility mode */
#define SX127X_FOM_PKTCFG2_DATAMODE       (1 << 6) /* Bit 6: 0 -> contrinous mode, 1 -> packet mode */

/* FSK/OOK: Payload length setting */

#define SX127X_FOM_PAYLOADLEN_MAX         (0x7f)
#define SX127X_FOM_PAYLOADLEN_LSB(plen)   ((plen >> 0) & 0xff)

/* FSK/OOK: FIFO threshold, TX start condition */

#define SX127X_FOM_FIFOTHR_THR_SHIFT      (0)      /* Bits 0-5: FIFO threshold */
#define SX127X_FOM_FIFOTHR_THR_MASK       (63 << SX127X_FOM_FIFOTHR_THR_SHIFT)
#define SX127X_FOM_FIFOTHR_TXSTARTCOND    (1 << 7) /* Bit 7: Packet transmission start condition */

/* FSK/OOK: Top level Sequencer settings 1 */

#define SX127X_FOM_SEQCFG1_TX             (1 << 0)                              /* Bit 0: From Transmit  0 -> LowPowerSelection, 1 -> Receive */
#define SX127X_FOM_SEQCFG1_IDLE           (1 << 1)                              /* Bit 1: From IDLE on T1: 0 -> TX, 1 -> RX*/
#define SX127X_FOM_SEQCFG1_LOWPOWERSEL    (1 << 2)                              /* Bit 2: Low Power Selection */
#define SX127X_FOM_SEQCFG1_START_SHIFT    (3)                                   /* Bits 3-4: From Start */
#define SX127X_FOM_SEQCFG1_START_MASK     (3 << SX127X_FOM_SEQCFG1_START_SHIFT)
#  define SX127X_FOM_SEQCFG1_START_LPS    (0 << SX127X_FOM_SEQCFG1_START_SHIFT) /* LowPowerSelection */
#  define SX127X_FOM_SEQCFG1_START_RS     (1 << SX127X_FOM_SEQCFG1_START_SHIFT) /* RX */
#  define SX127X_FOM_SEQCFG1_START_TS     (2 << SX127X_FOM_SEQCFG1_START_SHIFT) /* TX */
#  define SX127X_FOM_SEQCFG1_START_TSFL   (3 << SX127X_FOM_SEQCFG1_START_SHIFT) /* TX on FifoLevel */
#define SX127X_FOM_SEQCFG1_IDLEMODE       (1 << 5)                              /* Bit 5: IDLE Mode 0 -> standby, 1 -> sleep */
#define SX127X_FOM_SEQCFG1_SEQSTOP        (1 << 6)                              /* Bit 6: Sequencer Stop */
#define SX127X_FOM_SEQCFG1_SEQSTART       (1 << 7)                              /* Bit 7: Sequencer Start */

/* FSK/OOK: Top level Sequencer settings 2 */

#define SX127X_FOM_SEQCFG2_PCKRX_SHIFT    (0)      /* Bits 0-2: From Packet Received */
#define SX127X_FOM_SEQCFG2_PCKRX_MASK     (7 << SX127X_FOM_SEQCFG2_PCKRX_SHIFT)
#  define SX127X_FOM_SEQCFG2_PCKRX_1      (0 << SX127X_FOM_SEQCFG2_PCKRX_SHIFT)
#  define SX127X_FOM_SEQCFG2_PCKRX_2      (1 << SX127X_FOM_SEQCFG2_PCKRX_SHIFT)
#  define SX127X_FOM_SEQCFG2_PCKRX_3      (2 << SX127X_FOM_SEQCFG2_PCKRX_SHIFT)
#  define SX127X_FOM_SEQCFG2_PCKRX_4      (3 << SX127X_FOM_SEQCFG2_PCKRX_SHIFT)
#  define SX127X_FOM_SEQCFG2_PCKRX_5      (4 << SX127X_FOM_SEQCFG2_PCKRX_SHIFT)
#define SX127X_FOM_SEQCFG2_RXTOUT_SHIFT   (3)      /* Bits 3-4: From RX Timeout */
#define SX127X_FOM_SEQCFG2_RXTOUT_MASK    (7 << SX127X_FOM_SEQCFG2_RXTOUT_SHIFT)
#  define SX127X_FOM_SEQCFG2_RXTOUT_1     (0 << SX127X_FOM_SEQCFG2_RXTOUT_SHIFT)
#  define SX127X_FOM_SEQCFG2_RXTOUT_2     (1 << SX127X_FOM_SEQCFG2_RXTOUT_SHIFT)
#  define SX127X_FOM_SEQCFG2_RXTOUT_3     (2 << SX127X_FOM_SEQCFG2_RXTOUT_SHIFT)
#  define SX127X_FOM_SEQCFG2_RXTOUT_4     (3 << SX127X_FOM_SEQCFG2_RXTOUT_SHIFT)
#define SX127X_FOM_SEQCFG2_RX_SHIFT       (5)      /* Bits 5: From Receiver */
#define SX127X_FOM_SEQCFG2_RX_MASK        (7 << SX127X_FOM_SEQCFG2_RX_SHIFT)
#  define SX127X_FOM_SEQCFG2_RX_1         (0 << SX127X_FOM_SEQCFG2_RX_SHIFT)
#  define SX127X_FOM_SEQCFG2_RX_2         (1 << SX127X_FOM_SEQCFG2_RX_SHIFT)
#  define SX127X_FOM_SEQCFG2_RX_3         (2 << SX127X_FOM_SEQCFG2_RX_SHIFT)
#  define SX127X_FOM_SEQCFG2_RX_4         (3 << SX127X_FOM_SEQCFG2_RX_SHIFT)
#  define SX127X_FOM_SEQCFG2_RX_5         (4 << SX127X_FOM_SEQCFG2_RX_SHIFT)
#  define SX127X_FOM_SEQCFG2_RX_6         (5 << SX127X_FOM_SEQCFG2_RX_SHIFT)
#  define SX127X_FOM_SEQCFG2_RX_7         (6 << SX127X_FOM_SEQCFG2_RX_SHIFT)

/* FSK/OOK: Timer 1 and 2 resolution control */

#define SX127X_FOM_TIMRES_TIM1RES_SHIFT   (0)      /* Bits 0-1: Timer 1 Resolution */
#define SX127X_FOM_TIMRES_TIM1RES_MASK    (3 << SX127X_FOM_TIMRES_TIM1RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM1RES_OFF   (0 << SX127X_FOM_TIMRES_TIM1RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM1RES_64us  (1 << SX127X_FOM_TIMRES_TIM1RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM1RES_4p1ms (2 << SX127X_FOM_TIMRES_TIM1RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM1RES_262ms (3 << SX127X_FOM_TIMRES_TIM1RES_SHIFT)
#define SX127X_FOM_TIMRES_TIM2RES_SHIFT   (2)      /* Bits 2-3: Timer 2 Resolution */
#define SX127X_FOM_TIMRES_TIM2RES_MASK    (3 << SX127X_FOM_TIMRES_TIM2RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM2RES_OFF   (0 << SX127X_FOM_TIMRES_TIM2RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM2RES_64us  (1 << SX127X_FOM_TIMRES_TIM2RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM2RES_4p1ms (2 << SX127X_FOM_TIMRES_TIM2RES_SHIFT)
#  define SX127X_FOM_TIMRES_TIM2RES_262ms (3 << SX127X_FOM_TIMRES_TIM2RES_SHIFT)

/* FSK/OOK: Image calibration engine control */

#define SX127X_FOM_IMAGECAL_TEMPMONOFF    (1 << 0) /* Bit 0: Temperature monitor OFF */
#define SX127X_FOM_IMAGECAL_TEMPTHR_SHIFT (1)      /* Bit 1-2: Temperature threshold */
#define SX127X_FOM_IMAGECAL_TEMPTHR_MASK  (3 << SX127X_FOM_IMAGECAL_TEMPTHR_SHIFT)
#  define SX127X_FOM_IMAGECAL_TEMPTHR_5C  (0 << SX127X_FOM_IMAGECAL_TEMPTHR_SHIFT)
#  define SX127X_FOM_IMAGECAL_TEMPTHR_10C (1 << SX127X_FOM_IMAGECAL_TEMPTHR_SHIFT)
#  define SX127X_FOM_IMAGECAL_TEMPTHR_15C (2 << SX127X_FOM_IMAGECAL_TEMPTHR_SHIFT)
#  define SX127X_FOM_IMAGECAL_TEMPTHR_20C (3 << SX127X_FOM_IMAGECAL_TEMPTHR_SHIFT)
#define SX127X_FOM_IMAGECAL_TEMPCHANGE    (1 << 3) /* Bit 3: Temperature change */
#define SX127X_FOM_IMAGECAL_IMGCALRUN     (1 << 5) /* Bit 5: Image Calibration are running */
#define SX127X_FOM_IMAGECAL_IMGCALSTART   (1 << 6) /* Bit 6: Image Calibration start */
#define SX127X_FOM_IMAGECAL_AUTOIMGCALON  (1 << 7) /* Bit 7: Auto Image Calibration ON */

/* FSK/OOK: Low Battery Indicator settings */

#define SX127X_FOM_LOWBAT_TRIM_SHIFT      (0)      /* Bits 0-2: Low battery threshold */
#define SX127X_FOM_LOWBAT_TRIM_MASK       (7 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_1p695V   (0 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_1p764V   (1 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_1p835V   (2 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_1p905V   (3 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_1p976V   (4 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_2p045V   (5 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_2p116V   (6 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#  define SX127X_FOM_LOWBAT_TRIM_2p185V   (7 << SX127X_FOM_LOWBAT_TRIM_SHIFT)
#define SX127X_FOM_LOWBAT_LOWBATON        (1 << 3) /* Bit 3: Low Battery detecotr enable */

/* FSK/OOK: Status register 1: PLL Lock state, Timeout, RSSI */

#define SX127X_FOM_IRQ1_SYNCADDRMATCH     (1 << 0) /* Bit 0: Sync and Address detected */
#define SX127X_FOM_IRQ1_PREAMBE           (1 << 1) /* Bit 1: Preamble detected */
#define SX127X_FOM_IRQ1_TIMEOUT           (1 << 2) /* Bit 2: Timeout */
#define SX127X_FOM_IRQ1_RSSI              (1 << 3) /* Bit 3: RssiValue exceeds RssiThreshold in RX */
#define SX127X_FOM_IRQ1_PLLLOCK           (1 << 4) /* Bit 4: PLL is locked (in FS, RX or TX) */
#define SX127X_FOM_IRQ1_TXRDY             (1 << 5) /* Bit 5: set in TX mode after PA ramp-up */
#define SX127X_FOM_IRQ1_RXRDY             (1 << 6) /* Bit 6: set in RX mode after RSSI, AGC and AFC */
#define SX127X_FOM_IRQ1_MODERDY           (1 << 7) /* Bit 7: operation mode request is ready */

/* FSK/OOK: Status register 2: FIFO handling flags, Low Battery */

#define SX127X_FOM_IRQ2_LOWBAT            (1 << 0) /* Bit 0: Low Battery */
#define SX127X_FOM_IRQ2_CRCOK             (1 << 1) /* Bit 1: CRC of the payload is OK in RX */
#define SX127X_FOM_IRQ2_PAYLOADRDY        (1 << 2) /* Bit 2: payload is ready in RX */
#define SX127X_FOM_IRQ2_PCKSENT           (1 << 3) /* Bit 3: packet has been sent in TX */
#define SX127X_FOM_IRQ2_FIFOOVR           (1 << 4) /* Bit 4: FIFO overrun */
#define SX127X_FOM_IRQ2_FIFOLVL           (1 << 5) /* Bit 5: FIFO level */
#define SX127X_FOM_IRQ2_FIFOEMPTY         (1 << 6) /* Bit 6: FIFO empty */
#define SX127X_FOM_IRQ2_FIFOFULL          (1 << 7) /* Bit 7: FIFO full */

/* FSK/OOK: Control the fast frequency hopping mode */

#define SX127X_FOM_PLLHOP_FASTHOPON       (1 << 7) /* Bit 7: Fast frequency hop enable */

/* FSK/OOK: Fractional part in the Bit Rate division ratio */

#define SX127X_FOM_BITRATEFRAC_SHIFT      (0)
#define SX127X_FOM_BITRATEFRAC_MASK       (0x0f)

/* LORA *********************************************************************/

/* LORA: FIFO SPI pointer */

#define SX127X_LRM_ADDRPTR_DEFAULT        (0x00)

/* LORA: Start TX data */

#define SX127X_LRM_TXBASE_DEFAULT         (0x80)

/* LORA: Start RX data */

#define SX127X_LRM_RXBASE_DEFAULT         (0x00)

/* LORA: IRQ flag / IRQ Flags mask */

#define SX127X_LRM_IRQ_CADDETECT          (1 << 0) /* Bit 0: CAD detection */
#define SX127X_LRM_IRQ_FHSSCHANGECHAN     (1 << 1) /* Bit 1: FHHS change channel */
#define SX127X_LRM_IRQ_CADDONE            (1 << 2) /* Bit 2: CAD done */
#define SX127X_LRM_IRQ_TXDONE             (1 << 3) /* Bit 3: TX done */
#define SX127X_LRM_IRQ_VALIDHDR           (1 << 4) /* Bit 4: Valid header received in RX */
#define SX127X_LRM_IRQ_PAYLOADCRCERR      (1 << 5) /* Bit 5: Payload CRC error */
#define SX127X_LRM_IRQ_RXDONE             (1 << 6) /* Bit 6: RX done */
#define SX127X_LRM_IRQ_RXTIMEOUT          (1 << 7) /* Bit 7: RX timeout */

/* LORA: Live LORA modem status */

#define SX127X_LRM_MODSTAT_SIGDETECT      (1 << 0) /* Bit 0: Signal detected */
#define SX127X_LRM_MODSTAT_SIGSYNC        (1 << 1) /* Bit 1: Signal synchronized */
#define SX127X_LRM_MODSTAT_RXONGOING      (1 << 2) /* Bit 2: RX on-going */
#define SX127X_LRM_MODSTAT_HDRVALID       (1 << 3) /* Bit 3: Header info valid */
#define SX127X_LRM_MODSTAT_MODEMCLR       (1 << 4) /* Bit 4: Modem clear */
#define SX127X_LRM_MODSTAT_RXRATE_SHIFT   (5)      /* Bits 5-7: Coding rate of last header received */
#define SX127X_LRM_MODSTAT_RXRATE_MASK    (7 << SX127X_LRM_MODSTAT_RXRATE_SHIFT)

/* LORA: Current RSSI */

#define SX127X_LRM_RSSIVAL_HF_OFFSET      (-127)
#define SX127X_LRM_RSSIVAL_LF_OFFSET      (-164)

/* LORA: FHSS start channel */

#define SX127X_LRM_HOPCHAN_FHHSP_SHIFT    (0)      /* Bits 0-5: current value of frequency hopping in use */
#define SX127X_LRM_HOPCHAN_FHHSPN_MASK    (63 << SX127X_LRM_HOPCHAN_FHHSPRESCHAN_SHIFT)
#define SX127X_LRM_HOPCHAN_CRCONPAYLOAD   (1 << 6) /* Bit 6: CRC on payload */
#define SX127X_LRM_HOPCHAN_PLLTIMEOUT     (1 << 7) /* Bit 7: PLL timeout */

/* LORA: Modem PHY config 1 */

#define SX127X_LRM_MDMCFG1_IMPLHDRON      (1 << 0) /* Bit 0: Implicit header mode ON */
#define SX127X_LRM_MDMCFG1_CDRATE_SHIFT   (1)      /* Bits 1-3: Error coding rate */
#define SX127X_LRM_MDMCFG1_CDRATE_MASK    (7 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_CDRATE_4d5   (1 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_CDRATE_4d6   (2 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_CDRATE_4d7   (3 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_CDRATE_4d8   (4 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#define SX127X_LRM_MDMCFG1_BW_SHIFT       (4)      /* Bits 4-7: Signal bandwidth */
#define SX127X_LRM_MDMCFG1_BW_MASK        (31 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_7p8kHz    (0 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_10p4kHz   (1 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_15p6kHz   (2 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_20p8kHz   (3 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_31p25kHz  (4 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_41p7kHz   (5 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_62p5kHz   (6 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_125kHz    (7 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_250kHz    (8 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)
#  define SX127X_LRM_MDMCFG1_BW_500kHz    (9 << SX127X_LRM_MDMCFG1_CDRATE_SHIFT)

/* LORA: Modem PHY config 2 */

#define SX127X_LRM_MDMCFG2_RXTIMOUT_SHIFT (0)      /* Bits 0-1: RX timeout MSB */
#define SX127X_LRM_MDMCFG2_RXTIMOUT_MASK  (3 << SX127X_LRM_MDMCFG2_RXTIMOUT_SHIFT)
#define SX127X_LRM_MDMCFG2_RXCRCON        (1 << 2) /* Bit 2: RX payload CRC ON */
#define SX127X_LRM_MDMCFG2_TXCONT         (1 << 3) /* Bit 3: TX continuous mode */
#define SX127X_LRM_MDMCFG2_SPRFACT_SHIFT  (4)      /* Bits 4-7: Spreading factor */
#define SX127X_LRM_MDMCFG2_SPRFACT_MASK   (15 << SX127X_LRM_MDMCFG2_SPRFACT_SHIFT)

/* LORA: Receiver timeout value */

#define SX127X_LRM_RXTIMEOUT_MAX          (0x3f)
#define SX127X_LRM_RXTIMEOUT_LSB(t)       ((t >> 0) & 0xff)
#define SX127X_LRM_RXTIMEOUT_MSB(t)       ((t >> 8) & SX127X_LRM_MDMCFG2_RXTIMOUT_MASK)

/* LORA: Size of preamble */

#define SX127X_LRM_PREMSB_DEFAULT         (0x0008)
#define SX127X_LRM_PREMSB_MAX             (0xffff)
#define SX127X_LRM_PRE_LSB(p)             ((p >> 0) & 0xff)
#define SX127X_LRM_PRE_MSB(p)             ((p >> 8) & 0xff)

/* LORA: LORA payload length */

#define SX127X_LRM_PAYLOADLEN_DEFAULT     (0x01)
#define SX127X_LRM_PAYLOADLEN_MAX         (0xff)

/* LORA: FHSS Hop period */

#define SX127X_LRM_HOPPER_DEFAULT         (0x00)

/* LORA: Modem PHY confgi 3 */

#define SX127X_LRM_MODEMCFG3_AGCAUTOON    (1 << 2) /* Bit 2: AGC auto ON */
#define SX127X_LRM_MODEMCFG3_LOWDRATEOPT  (1 << 3) /* Bit 3: Low data rate optimize enable */

/* LORA: Estimated frequency error */

#define SX127X_LRM_FEIMSB_SHIFT           (0)
#define SX127X_LRM_FEIMSB_MASK            (15 << SX127X_LRM_FEIMSB_SHIFT)
#define SX127X_LRM_FEI(msb, mid, lsb)     ((lsb << 0) | (mid << 8) | ((msb & SX127X_LRM_FEIMSB_MASK) << 16))
#define SX127X_LRM_FEI_GET(fei)           ()       /* TODO */

/* LORA: LORA detection optimize for SF6 */

#define SX127X_LRM_DETECTOPT_DO_SHIFT     (0)     /* Bits 0-2: Detection optimize */
#define SX127X_LRM_DETECTOPT_DO_MASK      (7 << SX127X_LRM_DETECTOPT_DO_SHIFT)
#  define SX127X_LRM_DETECTOPT_DO_SF7SF12 (3 << SX127X_LRM_DETECTOPT_DO_SHIFT) /* 0x03: SF7 to SF12 */
#  define SX127X_LRM_DETECTOPT_DO_SF6     (5 << SX127X_LRM_DETECTOPT_DO_SHIFT) /* 0x05: SF6 */

/* LORA: Invert LORA I and Q signals */

#define SX127X_LRM_INVERTIQ_IIQ           (1 << 6) /* Bit 6: Invert the LORA I and Q signals */

/* LORA: LORA detection threshold for SF6 */

#define SX127X_LRM_DETECTTHR_SF7SF12      (0x0a)
#define SX127X_LRM_DETECTTHR_SF6          (0x0c)

/* LORA: LORA Sync Word */

#define SX127X_LRM_SYNCWORD_DEFAULT       (0x12)
#define SX127X_LRM_SYNCWORD_LORAWAN       (0x34)

/* Lora data rate:
 * DR = SF * BW * CR / (2**SF)
 *
 *   DR - Data rate
 *   SF - Spreading factor
 *   BW - Bandwidth
 *   CR - Coding rate
 */

#define LORA_DATARATE_GET(sf, bw, cr)     (sf * bw * cr / (2<<sf))

/* Constants ****************************************************************/

/* FXOSC is 32 MHz */

#define SX127X_FXOSC                      (32000000)

/* FSTEP is FXOSC/(2**19) =~ 61 Hz */

#define SX127X_FSTEP                      (SX127X_FXOSC/(2<<18))

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
  extern "C"
  {
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
  }
#endif

#endif /* __DRIVERS_WIRELESS_LPWAN_SX127X_H */
