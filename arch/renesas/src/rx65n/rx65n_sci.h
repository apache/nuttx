/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_sci.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_SCI_H
#define __ARCH_RENESAS_SRC_RX65N_SCI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Transmit FIFO Data Register (FTDR) */

/* Transmit Multi-Processor */

#define _FC00_SCI_DATA_TRANSMIT (0xfc00u) /* Data transmission cycles */
#define _FE00_SCI_ID_TRANSMIT   (0xfe00u) /* ID transmission cycles */

/* Serial Mode Register (SMR) */

/* Clock Select (CKS) */

#define _00_SCI_CLOCK_PCLK    (0x00u)   /* PCLK */
#define _01_SCI_CLOCK_PCLK_4  (0x01u)   /* PCLK/4 */
#define _02_SCI_CLOCK_PCLK_16 (0x02u)   /* PCLK/16 */
#define _03_SCI_CLOCK_PCLK_64 (0x03u)   /* PCLK/64 */

/* Multi-Processor Mode (MP) */

/* Disable multiprocessor mode */

#define _00_SCI_MULTI_PROCESSOR_DISABLE  (0x00u)

/* Enable multiprocessor mode */

#define _04_SCI_MULTI_PROCESSOR_ENABLE   (0x04u)

/* Stop Bit Length (STOP) */

#define _00_SCI_STOP_1       (0x00u)   /* 1 stop bit length */
#define _08_SCI_STOP_2       (0x08u)   /* 2 stop bits length */

/* Parity Mode (PM) */

#define _00_SCI_PARITY_EVEN  (0x00u)   /* Parity even */
#define _10_SCI_PARITY_ODD   (0x10u)   /* Parity odd */

/* Parity Enable (PE) */

#define _00_SCI_PARITY_DISABLE (0x00u)   /* Parity disable */
#define _20_SCI_PARITY_ENABLE  (0x20u)   /* Parity enable */

/* Character Length (CHR) */

#define _00_SCI_DATA_LENGTH_8  (0x00u)   /* Data length 8 bits */
#define _40_SCI_DATA_LENGTH_7  (0x40u)   /* Data length 7 bits */

/* Communications Mode (CM) */

/* Asynchronous or simple I2C mode */

#define _00_SCI_ASYNCHRONOUS_OR_I2C_MODE    (0x00u)

/* Clock synchronous or simple SPI mode */

#define _80_SCI_CLOCK_SYNCHRONOUS_OR_SPI_MODE (0x80u)

/* Base Clock Pulse (BCP) */

#define _00_SCI_32_93_CLOCK_CYCLES    (0x00u)   /* 32 or 93 clock cycles */
#define _04_SCI_64_128_CLOCK_CYCLES   (0x04u)   /* 64 or 128 clock cycles */
#define _08_SCI_186_372_CLOCK_CYCLES  (0x08u)   /* 186 or 372 clock cycles */
#define _0C_SCI_256_512_CLOCK_CYCLES  (0x0cu)   /* 256 or 512 clock cycles */

/* Block Transfer Mode (BLK) */

#define _00_SCI_BLK_TRANSFER_DISABLE (0x00u)   /* Block transfer disable */
#define _40_SCI_BLK_TRANSFER_ENABLE  (0x40u)   /* Block transfer enable */

/* GSM Mode (GSM) */

#define _00_SCI_GSM_DISABLE  (0x00u)   /* Normal mode operation */
#define _80_SCI_GSM_ENABLE   (0x80u)   /* GSM mode operation */

/* Serial Control Register (SCR) */

/* Clock Enable (CKE) */

/* Internal clock selected, SCK pin unused */

#define _00_SCI_INTERNAL_SCK_UNUSED     (0x00u)

/* Internal clock selected, SCK pin output low */

#define _00_SCI_INTERNAL_SCK_FIXED_LOW  (0x00u)

/* Internal clock selected, SCK pin as clock output */

#define _01_SCI_INTERNAL_SCK_OUTPUT     (0x01u)

/* Internal clock selected, SCK pin output high */

#define _02_SCI_INTERNAL_SCK_FIXED_HIGH (0x02u)

/* External clock selected */

#define _02_SCI_EXTERNAL       (0x02u)

/* External or TMR clock selected */

#define _02_SCI_EXTERNAL_TMR   (0x02u)

/* Transmit End Interrupt Enable (TEIE) */

/* TEI interrupt request disable */

#define _00_SCI_TEI_INTERRUPT_DISABLE  (0x00u)

/* TEI interrupt request enable */

#define _04_SCI_TEI_INTERRUPT_ENABLE   (0x04u)

/* Multi-Processor Interrupt Enable (MPIE) */

#define _00_SCI_MP_INTERRUPT_NORMAL    (0x00u)   /* Normal reception */

/* Multi-processor ID reception */

#define _08_SCI_MP_INTERRUPT_SPECIAL   (0x08u)

/* Receive Enable (RE) */

#define _00_SCI_RECEIVE_DISABLE (0x00u)   /* Disable receive mode */
#define _10_SCI_RECEIVE_ENABLE  (0x10u)   /* Enable receive mode */

/* Transmit Enable (TE) */

#define _00_SCI_TRANSMIT_DISABLE  (0x00u)   /* Disable transmit mode */
#define _20_SCI_TRANSMIT_ENABLE   (0x20u)   /* Enable transmit mode */

/* Receive Interrupt Enable (RIE) */

#define _00_SCI_RXI_ERI_DISABLE  (0x00u)

/* Disable RXI and ERI interrupt requests */

#define _40_SCI_RXI_ERI_ENABLE   (0x40u)

/* Enable RXI and ERI interrupt requests */

/* Transmit Interrupt Enable (TIE) */

#define _00_SCI_TXI_DISABLE    (0x00u)   /* Disable TXI interrupt requests */
#define _80_SCI_TXI_ENABLE     (0x80u)   /* Enable TXI interrupt requests */

/* Smart Card Mode Register (SCMR) */

/* Smart Card Interface Mode Select (SMIF) */

#define _00_SCI_SERIAL_MODE     (0x00u)

/* Serial communications interface mode */

#define _01_SCI_SMART_CARD_MODE (0x01u)

/* Smart card interface mode */

/* Transmitted / Received Data Invert (SINV) */

#define _00_SCI_DATA_INVERT_NONE   (0x00u)   /* Data is not inverted */
#define _04_SCI_DATA_INVERTED      (0x04u)   /* Data is inverted */

/* Transmitted / Received Data Transfer Direction (SDIR) */

#define _00_SCI_DATA_LSB_FIRST (0x00u)   /* Transfer data LSB first */
#define _08_SCI_DATA_MSB_FIRST (0x08u)   /* Transfer data MSB first */

/* Character Length 1 (CHR1) */

#define _00_SCI_DATA_LENGTH_9      (0x00u)

/* Transmit/receive in 9-bit data length */

#define _10_SCI_DATA_LENGTH_8_OR_7 (0x10u)

/* Transmit/receive in 8-bit or 7-bit data length */

/* Base Clock Pulse 2 (BCP2) */

/* 93, 128, 186, or 512 clock cycles */

#define _00_SCI_93_128_186_512_CLK (0x00u)

/* 32, 64, 256, or 372 clock cycles */

#define _80_SCI_32_64_256_372_CLK  (0x80u)

/* SCMR default value */

#define _62_SCI_SCMR_DEFAULT  (0x62u)   /* Write default value of SCMR */

/* Serial Extended Mode Register (SEMR) */

/* Asynchronous Mode Clock Source Select (ACS0) */

#define _00_SCI_ASYNC_SOURCE_EXTERNAL (0x00u)   /* External clock input */

/* Logical AND of two clock cycles output from TMR */

#define _01_SCI_ASYNC_SOURCE_TMR      (0x01u)

/* Bit Modulation Enable (BRME) */

/* Bit rate modulation function is disabled */

#define _00_SCI_BIT_MODULATION_DISABLE (0x00u)
#define _04_SCI_BIT_MODULATION_ENABLE  (0x04u)

/* Bit rate modulation function is enabled */

/* Asynchronous Mode Base Clock Select (ABCS) */

/* Selects 16 base clock cycles for 1 bit period */

#define _00_SCI_16_BASE_CLOCK  (0x00u)

/* Selects 8 base clock cycles for 1 bit period */

#define _10_SCI_8_BASE_CLOCK   (0x10u)

/* Digital Noise Filter Function Enable (NFEN) */

#define _00_SCI_NOISE_FILTER_DISABLE  (0x00u)   /* Noise filter is disabled */
#define _20_SCI_NOISE_FILTER_ENABLE   (0x20u)   /* Noise filter is enabled */

/* Baud Rate Generator Double-Speed Mode Select (BGDM) */

/* Baud rate generator outputs normal frequency */

#define _00_SCI_BAUDRATE_SINGLE (0x00u)

/* Baud rate generator doubles output frequency */

#define _40_SCI_BAUDRATE_DOUBLE (0x40u)

/* Asynchronous Start Bit Edge Detections Select (RXDESEL) */

/* Low level on RXDn pin selected as start bit */

#define _00_SCI_LOW_LEVEL_START_BIT    (0x00u)

/* Falling edge on RXDn pin selected as start bit */

#define _80_SCI_FALLING_EDGE_START_BIT (0x80u)

/* Noise Filter Setting Register (SNFR) */

/* Noise Filter Clock Select (NFCS) */

#define _00_SCI_ASYNC_DIV_1  (0x00u)   /* Clock signal divided by 1 */
#define _01_SCI_IIC_DIV_1    (0x01u)   /* Clock signal divided by 1 */
#define _02_SCI_IIC_DIV_2    (0x02u)   /* Clock signal divided by 2 */
#define _03_SCI_IIC_DIV_4    (0x03u)   /* Clock signal divided by 4 */
#define _04_SCI_IIC_DIV_8    (0x04u)   /* Clock signal divided by 8 */

/* I2C Mode Register 1 (SIMR1) */

/* Simple IIC Mode Select (IICM) */

/* Serial or smart card mode */

#define _00_SCI_SERIAL_SMART_CARD_MODE  (0x00u)
#define _01_SCI_IIC_MODE         (0x01u)   /* Simple IIC mode */

/* SSDA Output Delay Select (IICDL) */

#define _00_SCI_NONE           (0x00u)   /* No output delay */
#define _08_SCI_0_TO_1_CYCLE   (0x08u)   /* 0 to 1 cycle */
#define _10_SCI_1_TO_2_CYCLE   (0x10u)   /* 1 to 2 cycles */
#define _18_SCI_2_TO_3_CYCLE   (0x18u)   /* 2 to 3 cycles */
#define _20_SCI_3_TO_4_CYCLE   (0x20u)   /* 3 to 4 cycles */
#define _28_SCI_4_TO_5_CYCLE   (0x28u)   /* 4 to 5 cycles */
#define _30_SCI_5_TO_6_CYCLE   (0x30u)   /* 5 to 6 cycles */
#define _38_SCI_6_TO_7_CYCLE   (0x38u)   /* 6 to 7 cycles */
#define _40_SCI_7_TO_8_CYCLE   (0x40u)   /* 7 to 8 cycles */
#define _48_SCI_8_TO_9_CYCLE   (0x48u)   /* 8 to 9 cycles */
#define _50_SCI_9_TO_10_CYCLE  (0x50u)   /* 9 to 10 cycles */
#define _58_SCI_10_TO_11_CYCLE (0x58u)   /* 10 to 11 cycles */
#define _60_SCI_11_TO_12_CYCLE (0x60u)   /* 11 to 12 cycles */
#define _68_SCI_12_TO_13_CYCLE (0x68u)   /* 12 to 13 cycles */
#define _70_SCI_13_TO_14_CYCLE (0x70u)   /* 13 to 14 cycles */
#define _78_SCI_14_TO_15_CYCLE (0x78u)   /* 14 to 15 cycles */
#define _80_SCI_15_TO_16_CYCLE (0x80u)   /* 15 to 16 cycles */
#define _88_SCI_16_TO_17_CYCLE (0x88u)   /* 16 to 17 cycles */
#define _90_SCI_17_TO_18_CYCLE (0x90u)   /* 17 to 18 cycles */
#define _98_SCI_18_TO_19_CYCLE (0x98u)   /* 18 to 19 cycles */
#define _A0_SCI_19_TO_20_CYCLE (0xa0u)   /* 19 to 20 cycles */
#define _A8_SCI_20_TO_21_CYCLE (0xa8u)   /* 20 to 21 cycles */
#define _B0_SCI_21_TO_22_CYCLE (0xb0u)   /* 21 to 22 cycles */
#define _B8_SCI_22_TO_23_CYCLE (0xb8u)   /* 22 to 23 cycles */
#define _C0_SCI_23_TO_24_CYCLE (0xc0u)   /* 23 to 24 cycles */
#define _C8_SCI_24_TO_25_CYCLE (0xc8u)   /* 24 to 25 cycles */
#define _D0_SCI_25_TO_26_CYCLE (0xd0u)   /* 25 to 26 cycles */
#define _D8_SCI_26_TO_27_CYCLE (0xd8u)   /* 26 to 27 cycles */
#define _E0_SCI_27_TO_28_CYCLE (0xe0u)   /* 27 to 28 cycles */
#define _E8_SCI_28_TO_29_CYCLE (0xe8u)   /* 28 to 29 cycles */
#define _F0_SCI_29_TO_30_CYCLE (0xf0u)   /* 29 to 30 cycles */
#define _F8_SCI_30_TO_31_CYCLE (0xf8u)   /* 30 to 31 cycles */

/* I2C Mode Register 2 (SIMR2) */

/* IIC Interrupt Mode Select (IICINTM) */

#define _00_SCI_ACK_NACK_INTERRUPTS  (0x00u)   /* Use ACK/NACK interrupts */

/* Use reception/transmission interrupts */

#define _01_SCI_RX_TX_INTERRUPTS     (0x01u)

/* Clock Synchronization (IICCSC) */

/* No synchronization with the clock signal */

#define _00_SCI_NO_SYNCHRONIZATION  (0x00u)

/* Synchronization with the clock signal */

#define _02_SCI_SYNCHRONIZATION     (0x02u)

/* ACK Transmission Data (IICACKT) */

#define _00_SCI_ACK_TRANSMISSION    (0x00u)   /* ACK transmission */

/* NACK transmission and reception of ACK/NACK */

#define _20_SCI_NACK_TRANSMISSION   (0x20u)

/* I2C Mode Register 3 (SIMR3) */

/* Start Condition Generation (IICSTAREQ) */

/* Start condition is not generated */

#define _00_SCI_START_CONDITION_OFF  (0x00u)

/* Start condition is generated */

#define _01_SCI_START_CONDITION_ON   (0x01u)

/* Restart Condition Generation (IICRSTAREQ) */

/* Restart condition is not generated */

#define _00_SCI_RESTART_CONDITION_OFF (0x00u)

/* Restart condition is generated */

#define _02_SCI_RESTART_CONDITION_ON  (0x02u)

/* Stop Condition Generation (IICSTPREQ) */

/* Stop condition is not generated */

#define _00_SCI_STOP_CONDITION_OFF  (0x00u)

/* Stop condition is generated */

#define _04_SCI_STOP_CONDITION_ON   (0x04u)

/* Issuing of Start, Restart, or Stop Condition Completed Flag (IICSTIF) */

/* No requests to generate conditions/conditions generated */

#define _00_SCI_CONDITION_GENERATED   (0x00u)

/* All request generation has been completed */

#define _08_SCI_GENERATION_COMPLETED  (0x08u)

/* SSDA Output Select (IICSDAS) */

/* SSDA output is serial data output */

#define _00_SCI_SSDA_DATA_OUTPUT     (0x00u)

/* SSDA output generates start, restart or stop condition */

#define _10_SCI_SSDA_START_RESTART_STOP_CONDITION (0x10u)

/* SSDA output low level */

#define _20_SCI_SSDA_LOW_LEVEL       (0x20u)

/* SSDA output high impedance */

#define _30_SCI_SSDA_HIGH_IMPEDANCE  (0x30u)

/* SSCL Output Select (IICSCLS) */

/* SSCL output is serial clock output */

#define _00_SCI_SSCL_CLOCK_OUTPUT    (0x00u)

/* SSCL output generates start, restart or stop condition */

#define _40_SCI_SSCL_START_RESTART_STOP_CONDITION (0x40u)

/* SSCL output low level */

#define _80_SCI_SSCL_LOW_LEVEL       (0x80u)

/* SSCL output high impedance */

#define _C0_SCI_SSCL_HIGH_IMPEDANCE  (0xc0u)

/* SPI Mode Register (SPMR) */

/* SS Pin Function Enable (SSE) */

#define _00_SCI_SS_PIN_DISABLE  (0x00u)   /* SS pin function disabled */
#define _01_SCI_SS_PIN_ENABLE   (0x01u)   /* SS pin function enabled */

/* CTS Enable (CTSE) */

#define _00_SCI_RTS     (0x00u)   /* RTS function is enabled */
#define _02_SCI_CTS     (0x02u)   /* CTS function is enabled */

/* Master Slave Select (MSS) */

#define _00_SCI_SPI_MASTER  (0x00u)   /* Master mode */
#define _04_SCI_SPI_SLAVE   (0x04u)   /* Slave mode */

/* Mode Fault Flag (MFF) */

#define _00_SCI_NO_MODE_FAULT  (0x00u)   /* No mode fault */
#define _10_SCI_MODE_FAULT     (0x10u)   /* Mode fault */

/* Clock Polarity Select (CKPOL) */

/* Clock polarity is not inverted */

#define _00_SCI_CLOCK_NOT_INVERTED (0x00u)

/* Clock polarity is inverted */

#define _40_SCI_CLOCK_INVERTED     (0x40u)

/* Clock Phase Select (CKPH) */

#define _00_SCI_CLOCK_NOT_DELAYED (0x00u)   /* Clock is not delayed */
#define _80_SCI_CLOCK_DELAYED     (0x80u)   /* Clock is delayed */

/* FIFO Control Register (FCR) */

/* FIFO Mode Select (FM) */

#define _0000_SCI_NON_FIFO_MODE  (0x0000u) /* Non-FIFO mode */
#define _0001_SCI_FIFO_MODE      (0x0001u) /* FIFO mode */

/* Receive FIFO Reset (RFRST) */

/* Select receive data full interrupt (RXI) */

#define _0000_SCI_RX_FIFO_RESET_DISABLE (0x0000u)
#define _0002_SCI_RX_FIFO_RESET_ENABLE  (0x0002u) /* FIFO mode */

/* Transmit FIFO Reset (TFRST) */

/* Select receive data full interrupt (RXI) */

#define _0000_SCI_TX_FIFO_RESET_DISABLE (0x0000u)
#define _0004_SCI_TX_FIFO_RESET_ENABLE  (0x0004u) /* FIFO mode */

/* Receive Data Ready Interrupt Select (DRES) */

/* Select receive data full interrupt (RXI) */

#define _0000_SCI_RXI_SELECT (0x0000u)
#define _0008_SCI_ERI_SELECT (0x0008u) /* Select error interrupt (RXI) */

/* Transmit FIFO Threshold Setting (TTRG) */

/* Transmit FIFO threshold value is 0 */

#define _0000_SCI_TX_TRIGGER_NUM_0 (0x0000u)

/* Transmit FIFO threshold value is 1 */

#define _0010_SCI_TX_TRIGGER_NUM_1 (0x0010u)

/* Transmit FIFO threshold value is 2 */

#define _0020_SCI_TX_TRIGGER_NUM_2 (0x0020u)

/* Transmit FIFO threshold value is 3 */

#define _0030_SCI_TX_TRIGGER_NUM_3  (0x0030u)

/* Transmit FIFO threshold value is 4 */

#define _0040_SCI_TX_TRIGGER_NUM_4  (0x0040u)

/* Transmit FIFO threshold value is 5 */

#define _0050_SCI_TX_TRIGGER_NUM_5  (0x0050u)

/* Transmit FIFO threshold value is 6 */

#define _0060_SCI_TX_TRIGGER_NUM_6  (0x0060u)

/* Transmit FIFO threshold value is 7 */

#define _0070_SCI_TX_TRIGGER_NUM_7  (0x0070u)

/* Transmit FIFO threshold value is 8 */

#define _0080_SCI_TX_TRIGGER_NUM_8  (0x0080u)

/* Transmit FIFO threshold value is 9 */

#define _0090_SCI_TX_TRIGGER_NUM_9  (0x0090u)

/* Transmit FIFO threshold value is 10 */

#define _00A0_SCI_TX_TRIGGER_NUM_10 (0x00a0u)

/* Transmit FIFO threshold value is 11 */

#define _00B0_SCI_TX_TRIGGER_NUM_11 (0x00b0u)

/* Transmit FIFO threshold value is 12 */

#define _00C0_SCI_TX_TRIGGER_NUM_12 (0x00c0u)

/* Transmit FIFO threshold value is 13 */

#define _00D0_SCI_TX_TRIGGER_NUM_13 (0x00d0u)

/* Transmit FIFO threshold value is 14 */

#define _00E0_SCI_TX_TRIGGER_NUM_14 (0x00e0u)
#define _00F0_SCI_TX_TRIGGER_NUM_15 (0x00f0u)

/* Transmit FIFO threshold value is 15 */

/* Receive FIFO Threshold Setting (RTRG) */

/* Receive FIFO threshold value is 1 */

#define _0100_SCI_RX_TRIGGER_NUM_1   (0x0100u)

/* Receive FIFO threshold value is 2 */

#define _0200_SCI_RX_TRIGGER_NUM_2   (0x0200u)

/* Receive FIFO threshold value is 3 */

#define _0300_SCI_RX_TRIGGER_NUM_3   (0x0300u)

/* Receive FIFO threshold value is 4 */

#define _0400_SCI_RX_TRIGGER_NUM_4   (0x0400u)

/* Receive FIFO threshold value is 5 */

#define _0500_SCI_RX_TRIGGER_NUM_5   (0x0500u)

/* Receive FIFO threshold value is 6 */

#define _0600_SCI_RX_TRIGGER_NUM_6   (0x0600u)

/* Receive FIFO threshold value is 7 */

#define _0700_SCI_RX_TRIGGER_NUM_7   (0x0700u)

/* Receive FIFO threshold value is 8 */

#define _0800_SCI_RX_TRIGGER_NUM_8   (0x0800u)

/* Receive FIFO threshold value is 9 */

#define _0900_SCI_RX_TRIGGER_NUM_9   (0x0900u)

/* Receive FIFO threshold value is 10 */

#define _0A00_SCI_RX_TRIGGER_NUM_10  (0x0a00u)

/* Receive FIFO threshold value is 11 */

#define _0B00_SCI_RX_TRIGGER_NUM_11  (0x0b00u)

/* Receive FIFO threshold value is 12 */

#define _0C00_SCI_RX_TRIGGER_NUM_12  (0x0c00u)

/* Receive FIFO threshold value is 13 */

#define _0D00_SCI_RX_TRIGGER_NUM_13  (0x0d00u)

/* Receive FIFO threshold value is 14 */

#define _0E00_SCI_RX_TRIGGER_NUM_14  (0x0e00u)

/* Receive FIFO threshold value is 15 */

#define _0F00_SCI_RX_TRIGGER_NUM_15  (0x0f00u)

/* RTS# Output Threshold Setting (RSTRG) */

/* Receive FIFO threshold value is 1 */

#define _1000_SCI_RTS_TRIGGER_NUM_1   (0x1000u)

/* Receive FIFO threshold value is 2 */

#define _2000_SCI_RTS_TRIGGER_NUM_2   (0x2000u)

/* Receive FIFO threshold value is 3 */

#define _3000_SCI_RTS_TRIGGER_NUM_3   (0x3000u)

/* Receive FIFO threshold value is 4 */

#define _4000_SCI_RTS_TRIGGER_NUM_4   (0x4000u)

/* Receive FIFO threshold value is 5 */

#define _5000_SCI_RTS_TRIGGER_NUM_5   (0x5000u)

/* Receive FIFO threshold value is 6 */

#define _6000_SCI_RTS_TRIGGER_NUM_6   (0x6000u)

/* Receive FIFO threshold value is 7 */

#define _7000_SCI_RTS_TRIGGER_NUM_7   (0x7000u)

/* Receive FIFO threshold value is 8 */

#define _8000_SCI_RTS_TRIGGER_NUM_8   (0x8000u)

/* Receive FIFO threshold value is 9 */

#define _9000_SCI_RTS_TRIGGER_NUM_9   (0x9000u)

/* Receive FIFO threshold value is 10 */

#define _A000_SCI_RTS_TRIGGER_NUM_10  (0xa000u)

/* Receive FIFO threshold value is 11 */

#define _B000_SCI_RTS_TRIGGER_NUM_11  (0xb000u)

/* Receive FIFO threshold value is 12 */

#define _C000_SCI_RTS_TRIGGER_NUM_12  (0xc000u)

/* Receive FIFO threshold value is 13 */

#define _D000_SCI_RTS_TRIGGER_NUM_13  (0xd000u)

/* Receive FIFO threshold value is 14 */

#define _E000_SCI_RTS_TRIGGER_NUM_14  (0xe000u)

/* Receive FIFO threshold value is 15 */

#define _F000_SCI_RTS_TRIGGER_NUM_15  (0xf000u)

/* Data Comparison Control Register (DCCR) */

/* ID Frame Select (IDSEL) */

#define _00_SCI_ALL_DATA_COMPARED  (0x00u)

/* All data is to be compared */

#define _40_SCI_ID_FRAME_COMPARED  (0x40u)

/* Only data in ID frames is compared */

/* Data Match Detection Enable (DCME) */

/* Data match detection is disabled */

#define _00_SCI_DATA_MATCH_DISABLE (0x00u)

/* Data match detection is enabled */

#define _80_SCI_DATA_MATCH_ENABLE  (0x80u)

/* Interrupt Source Priority Register n (IPRn) */

/* Interrupt Priority Level Select (IPR[3:0]) */

#define _00_SCI_PRIORITY_LEVEL0  (0x00u)   /* Level 0 (interrupt disabled) */
#define _01_SCI_PRIORITY_LEVEL1  (0x01u)   /* Level 1 */
#define _02_SCI_PRIORITY_LEVEL2  (0x02u)   /* Level 2 */
#define _03_SCI_PRIORITY_LEVEL3  (0x03u)   /* Level 3 */
#define _04_SCI_PRIORITY_LEVEL4  (0x04u)   /* Level 4 */
#define _05_SCI_PRIORITY_LEVEL5  (0x05u)   /* Level 5 */
#define _06_SCI_PRIORITY_LEVEL6  (0x06u)   /* Level 6 */
#define _07_SCI_PRIORITY_LEVEL7  (0x07u)   /* Level 7 */
#define _08_SCI_PRIORITY_LEVEL8  (0x08u)   /* Level 8 */
#define _09_SCI_PRIORITY_LEVEL9  (0x09u)   /* Level 9 */
#define _0A_SCI_PRIORITY_LEVEL10 (0x0au)   /* Level 10 */
#define _0B_SCI_PRIORITY_LEVEL11 (0x0bu)   /* Level 11 */
#define _0C_SCI_PRIORITY_LEVEL12 (0x0cu)   /* Level 12 */
#define _0D_SCI_PRIORITY_LEVEL13 (0x0du)   /* Level 13 */
#define _0E_SCI_PRIORITY_LEVEL14 (0x0eu)   /* Level 14 */
#define _0F_SCI_PRIORITY_LEVEL15 (0x0fu)   /* Level 15 (highest) */

/* Transfer Status Control Value */

/* Simple IIC Transmit Receive Flag */

#define _80_SCI_IIC_TRANSMISSION (0x80u)  /* Simple IIC Transmission State */
#define _00_SCI_IIC_RECEPTION    (0x00u)  /* Simple IIC Reception State */

/* Simple IIC Start Stop Flag */

#define _80_SCI_IIC_START_CYCLE (0x80u)   /* Simple IIC Start Cycle */
#define _00_SCI_IIC_STOP_CYCLE  (0x00u)   /* Simple IIC Stop Cycle */

/* Multiprocessor Asynchronous Communication Flag */

/* Multiprocessor Asynchronous ID Transmission Cycle */

#define _80_SCI_ID_TRANSMISSION_CYCLE   (0x80u)

/* Multiprocessor Asynchronous Data Transmission Cycle */

#define _00_SCI_DATA_TRANSMISSION_CYCLE (0x00u)

/* FIFO Buffer Maximum Size */

#define _10_SCI_FIFO_MAX_SIZE (0x10u)   /* Size of 16-stage FIFO buffer */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: r_sci0_create
 *
 * Description:
 *   Initializes SCI0
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci0_create(void);

/****************************************************************************
 * Name: r_sci0_start
 *
 * Description:
 *   Starts SCI0
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci0_start(void);

/****************************************************************************
 * Name: r_sci0_stop
 *
 * Description:
 *   Stops SCI0
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci0_stop(void);

/****************************************************************************
 * Name: r_sci1_create
 *
 * Description:
 *   Initializes SCI1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci1_create(void);

/****************************************************************************
 * Name: r_sci1_start
 *
 * Description:
 *   Starts SCI1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci1_start(void);

/****************************************************************************
 * Name: r_sci1_stop
 *
 * Description:
 *   Stops SCI1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci1_stop(void);

/****************************************************************************
 * Name: r_sci2_create
 *
 * Description:
 *   Initializes SCI2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci2_create(void);

/****************************************************************************
 * Name: r_sci2_start
 *
 * Description:
 *   Starts SCI2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci2_start(void);

/****************************************************************************
 * Name: r_sci2_stop
 *
 * Description:
 *   Stops SCI2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci2_stop(void);

/****************************************************************************
 * Name: r_sci3_create
 *
 * Description:
 *   Initializes SCI3
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci3_create(void);

/****************************************************************************
 * Name: r_sci3_start
 *
 * Description:
 *   Starts SCI3
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci3_start(void);

/****************************************************************************
 * Name: r_sci3_stop
 *
 * Description:
 *   Stops SCI3
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci3_stop(void);

/****************************************************************************
 * Name: r_sci4_create
 *
 * Description:
 *   Creates SCI3
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci4_create(void);

/****************************************************************************
 * Name: r_sci4_start
 *
 * Description:
 *   Starts SCI4
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci4_start(void);

/****************************************************************************
 * Name: r_sci4_stop
 *
 * Description:
 *   Stops SCI4
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci4_stop(void);

/****************************************************************************
 * Name: r_sci5_create
 *
 * Description:
 *   Initialization of SCI5
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci5_create(void);

/****************************************************************************
 * Name: r_sci5_start
 *
 * Description:
 *   Starts SCI5
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci5_start(void);

/****************************************************************************
 * Name: r_sci5_stop
 *
 * Description:
 *   Stops SCI5
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci5_stop(void);

/****************************************************************************
 * Name: r_sci6_create
 *
 * Description:
 *   Initialization of SCI6
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci6_create(void);

/****************************************************************************
 * Name: r_sci6_start
 *
 * Description:
 *   Starts SCI6
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci6_start(void);

/****************************************************************************
 * Name: r_sci6_stop
 *
 * Description:
 *   Stops SCI6
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci6_stop(void);

/****************************************************************************
 * Name: r_sci7_create
 *
 * Description:
 *   Initialization of SCI7
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci7_create(void);

/****************************************************************************
 * Name: r_sci7_start
 *
 * Description:
 *   Starts SCI7
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci7_start(void);

/****************************************************************************
 * Name: r_sci7_stop
 *
 * Description:
 *   Stops SCI7
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci7_stop(void);

/****************************************************************************
 * Name: r_sci8_create
 *
 * Description:
 *  SCI8 Initialization
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci8_create(void);

/****************************************************************************
 * Name: r_sci8_start
 *
 * Description:
 *  SCI8 Start
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci8_start(void);

/****************************************************************************
 * Name: r_sci8_stop
 *
 * Description:
 *  SCI8 Stop
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci8_stop(void);

/****************************************************************************
 * Name: r_sci9_create
 *
 * Description:
 *  SCI9 Initialization
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci9_create(void);

/****************************************************************************
 * Name: r_sci9_start
 *
 * Description:
 *  SCI9 Start
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci9_start(void);

/****************************************************************************
 * Name: r_sci9_stop
 *
 * Description:
 *  SCI9 Stop
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci9_stop(void);

/****************************************************************************
 * Name: r_sci10_create
 *
 * Description:
 *  SCI10 Initialization
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci10_create(void);

/****************************************************************************
 * Name: r_sci10_start
 *
 * Description:
 *  SCI10 Start
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci10_start(void);

/****************************************************************************
 * Name: r_sci10_stop
 *
 * Description:
 *  SCI10 Stop
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci10_stop(void);

/****************************************************************************
 * Name: r_sci11_create
 *
 * Description:
 *  SCI8 Initialization
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci11_create(void);

/****************************************************************************
 * Name: r_sci11_start
 *
 * Description:
 *  SCI11 Start
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci11_start(void);

/****************************************************************************
 * Name: r_sci11_stop
 *
 * Description:
 *  SCI11 Stop
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci11_stop(void);

/****************************************************************************
 * Name: r_sci12_create
 *
 * Description:
 *  SCI12 Initialization
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci12_create(void);

/****************************************************************************
 * Name: r_sci12_start
 *
 * Description:
 *  SCI12 Start
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci12_start(void);

/****************************************************************************
 * Name: r_sci12_stop
 *
 * Description:
 *  SCI12 Stop
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_sci12_stop(void);

#endif /* __ARCH_RENESAS_SRC_RX65N_SCI_H */
