/****************************************************************************
 * arch/arm64/src/a64/hardware/a64_twi.h
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

#ifndef __ARCH_ARM_SRC_A64_HARDWARE_A64_TWI_H
#define __ARCH_ARM_SRC_A64_HARDWARE_A64_TWI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/a64_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TWI register offsets *****************************************************/

#define A64_TWI_ADDR_OFFSET    	    (0x00)  /* 31:8 bit reserved,7-1 bit for slave addr,0 bit for GCE */
#define A64_TWI_XADDR_OFFSET   	    (0x04)  /* 31:8 bit reserved,7-0 bit for second addr in 10bit addr */
#define A64_TWI_DATA_OFFSET    	    (0x08)  /* 31:8 bit reserved,7-0 bit send or receive data byte */
#define A64_TWI_CNTR_OFFSET         (0x0c)  /* 31:8 bit reserved, INT_EN, BUS_EN, M_STA, INT_FLAG, A_ACK */
#define A64_TWI_STAT_OFFSET    	    (0x10) 	/* 28 interrupt types + 0xF8 normal type = 29 */
#define A64_TWI_CCR_OFFSET     	    (0x14) 	/* 31:7 bit reserved,6-3bit,CLK_M,2-0bit CLK_N */
#define A64_TWI_SRST_OFFSET    	    (0x18) 	/* 31:1 bit reserved;0bit,write 1 to clear 0. */
#define A64_TWI_EFR_OFFSET     	    (0x1c)  /* 31:2 bit reserved,1:0 bit data byte follow read comand */
#define A64_TWI_LCR_OFFSET     	    (0x20) 	/* 31:6 bits reserved  5:0 bit for sda&scl control*/
#define A64_TWI_DVFS_OFFSET         (0x24)  /* 31:3 bits reserved  2:0 bit for dvfs control. only A10 support */

/* TWI register addresses ***************************************************/

#define A64_TWI0_SADDR              (A64_TWI0_ADDR+A64_TWI_SADDR_OFFSET)
#define A64_TWI0_XADDR              (A64_TWI0_ADDR+A64_TWI_XADDR_OFFSET)
#define A64_TWI0_DATA               (A64_TWI0_ADDR+A64_TWI_DATA_OFFSET)
#define A64_TWI0_CNTR               (A64_TWI0_ADDR+A64_TWI_CNTR_OFFSET)
#define A64_TWI0_STAT               (A64_TWI0_ADDR+A64_TWI_STAT_OFFSET)
#define A64_TWI0_CCR                (A64_TWI0_ADDR+A64_TWI_CCR_OFFSET)
#define A64_TWI0_SRST               (A64_TWI0_ADDR+A64_TWI_SRST_OFFSET)
#define A64_TWI0_EFR                (A64_TWI0_ADDR+A64_TWI_EFR_OFFSET)
#define A64_TWI0_LCR                (A64_TWI0_ADDR+A64_TWI_LCR_OFFSET)

#define A64_TWI1_SADDR              (A64_TWI1_ADDR+A64_TWI_SADDR_OFFSET)
#define A64_TWI1_XADDR              (A64_TWI1_ADDR+A64_TWI_XADDR_OFFSET)
#define A64_TWI1_DATA               (A64_TWI1_ADDR+A64_TWI_DATA_OFFSET)
#define A64_TWI1_CNTR               (A64_TWI1_ADDR+A64_TWI_CNTR_OFFSET)
#define A64_TWI1_STAT               (A64_TWI1_ADDR+A64_TWI_STAT_OFFSET)
#define A64_TWI1_CCR                (A64_TWI1_ADDR+A64_TWI_CCR_OFFSET)
#define A64_TWI1_SRST               (A64_TWI1_ADDR+A64_TWI_SRST_OFFSET)
#define A64_TWI1_EFR                (A64_TWI1_ADDR+A64_TWI_EFR_OFFSET)
#define A64_TWI1_LCR                (A64_TWI1_ADDR+A64_TWI_LCR_OFFSET)

#define A64_TWI2_SADDR              (A64_TWI2_ADDR+A64_TWI_SADDR_OFFSET)
#define A64_TWI2_XADDR              (A64_TWI2_ADDR+A64_TWI_XADDR_OFFSET)
#define A64_TWI2_DATA               (A64_TWI2_ADDR+A64_TWI_DATA_OFFSET)
#define A64_TWI2_CNTR               (A64_TWI2_ADDR+A64_TWI_CNTR_OFFSET)
#define A64_TWI2_STAT               (A64_TWI2_ADDR+A64_TWI_STAT_OFFSET)
#define A64_TWI2_CCR                (A64_TWI2_ADDR+A64_TWI_CCR_OFFSET)
#define A64_TWI2_SRST               (A64_TWI2_ADDR+A64_TWI_SRST_OFFSET)
#define A64_TWI2_EFR                (A64_TWI2_ADDR+A64_TWI_EFR_OFFSET)
#define A64_TWI2_LCR                (A64_TWI2_ADDR+A64_TWI_LCR_OFFSET)

#define A64_RTWI_SADDR              (A64_RTWI_ADDR+A64_TWI_SADDR_OFFSET)
#define A64_RTWI_XADDR              (A64_RTWI_ADDR+A64_TWI_XADDR_OFFSET)
#define A64_RTWI_DATA               (A64_RTWI_ADDR+A64_TWI_DATA_OFFSET)
#define A64_RTWI_CNTR               (A64_RTWI_ADDR+A64_TWI_CNTR_OFFSET)
#define A64_RTWI_STAT               (A64_RTWI_ADDR+A64_TWI_STAT_OFFSET)
#define A64_RTWI_CCR                (A64_RTWI_ADDR+A64_TWI_CCR_OFFSET)
#define A64_RTWI_SRST               (A64_RTWI_ADDR+A64_TWI_SRST_OFFSET)
#define A64_RTWI_EFR                (A64_RTWI_ADDR+A64_TWI_EFR_OFFSET)
#define A64_RTWI_LCR                (A64_RTWI_ADDR+A64_TWI_LCR_OFFSET)

/* TWI register bit definitions *********************************************/

/* TWI address register */

#define TWI_GCE_EN      	(0x1<<0)  /* general call address enable for slave mode */
#define TWI_ADDR_MASK   	(0x7f<<1) /* 7:1 bits */

/* TWI extend address register */

#define TWI_XADDR_MASK  (0xff)   /* 7:0 bits for extend slave address */

/* TWI Data register default is 0x0000_0000 */

#define TWI_DATA_MASK   (0xff)   /* 7:0 bits for send or received */

/* TWI Control Register Bit Fields & Masks, default value: 0x0000_0000 */

#define TWI_CNTR_ACK    (0x1<<2) /* set 1 to send A_ACK */
#define TWI_CNTR_INTFLG (0x1<<3) /* INT_FLAG,interrupt status flag */
#define TWI_CNTR_STP    (0x1<<4) /* M_STP,Automatic clear 0 */
#define TWI_CNTR_STA    (0x1<<5) /* M_STA,atutomatic clear 0 */
#define TWI_CNTR_BUSEN  (0x1<<6) /* BUS_EN, master mode should be set 1 */
#define TWI_CNTR_INTEN  (0x1<<7) /* INT_EN */

/* TWI Clock Register Bit Fields & Masks,default value:0x0000_0000 */

#define TWI_CLK_DIV_M       (0xf<<3) /* 6:3 bit */
#define TWI_CLK_DIV_N       (0x7<<0) /* 2:0 bit */

/* TWI Soft Reset Register Bit Fields & Masks  */

#define TWI_SRST_SRST       (0x1<<0) /* write 1 to clear 0, when complete soft reset clear 0 */

/* TWI Enhance Feature Register Bit Fields & Masks  */

#define TWI_EFR_MASK        (0x3<<0) /* 00:no,01: 1byte, 10:2 bytes, 11: 3bytes */
#define TWI_EFR_WARC_0      (0x0<<0)
#define TWI_EFR_WARC_1      (0x1<<0)
#define TWI_EFR_WARC_2      (0x2<<0)
#define TWI_EFR_WARC_3      (0x3<<0)

/* twi line control register -default value: 0x0000_003a */

#define TWI_LCR_SDA_EN          (0x01<<0) /* SDA line state control enable ,1:enable;0:disable */
#define TWI_LCR_SDA_CTL         (0x01<<1) /* SDA line state control bit, 1:high level;0:low level */
#define TWI_LCR_SCL_EN          (0x01<<2) /* SCL line state control enable ,1:enable;0:disable */
#define TWI_LCR_SCL_CTL         (0x01<<3) /* SCL line state control bit, 1:high level;0:low level */
#define TWI_LCR_SDA_STATE_MASK  (0x01<<4) /* current state of SDA,readonly bit */
#define TWI_LCR_SCL_STATE_MASK  (0x01<<5) /* current state of SCL,readonly bit */

/* 31:6bits reserved */

#define TWI_LCR_MASK            (0x3f)
#define TWI_LCR_IDLE_STATUS     (0x3a)

/* TWI Status Register Bit Fields & Masks */

#define TWI_STAT_MASK           (0xff)

/* 7:0 bits use only,default is 0xF8 */

#define TWI_STAT_BUS_ERR        (0x00) 	/* BUS ERROR */

/* Master mode use only */

#define TWI_STAT_TX_STA     (0x08) 	/* START condition transmitted */
#define TWI_STAT_TX_RESTA   (0x10) 	/* Repeated START condition transmitted */
#define TWI_STAT_TX_AW_ACK  (0x18) 	/* Address+Write bit transmitted, ACK received */
#define TWI_STAT_TX_AW_NAK  (0x20) 	/* Address+Write bit transmitted, ACK not received */
#define TWI_STAT_TXD_ACK    (0x28) 	/* data byte transmitted in master mode,ack received */
#define TWI_STAT_TXD_NAK    (0x30) 	/* data byte transmitted in master mode ,ack not received */
#define TWI_STAT_ARBLOST    (0x38) 	/* arbitration lost in address or data byte */
#define TWI_STAT_TX_AR_ACK  (0x40) 	/* Address+Read bit transmitted, ACK received */
#define TWI_STAT_TX_AR_NAK  (0x48) 	/* Address+Read bit transmitted, ACK not received */
#define TWI_STAT_RXD_ACK    (0x50) 	/* data byte received in master mode ,ack transmitted */
#define TWI_STAT_RXD_NAK    (0x58) 	/* date byte received in master mode,not ack transmitted */

/* Slave mode use only */

#define TWI_STAT_RXWS_ACK           (0x60) /* Slave address+Write bit received, ACK transmitted */
#define TWI_STAT_ARBLOST_RXWS_ACK   (0x68)
#define TWI_STAT_RXGCAS_ACK         (0x70) /* General Call address received, ACK transmitted */
#define TWI_STAT_ARBLOST_RXGCAS_ACK (0x78)
#define TWI_STAT_RXDS_ACK           (0x80)
#define TWI_STAT_RXDS_NAK           (0x88)
#define TWI_STAT_RXDGCAS_ACK        (0x90)
#define TWI_STAT_RXDGCAS_NAK        (0x98)
#define TWI_STAT_RXSTPS_RXRESTAS    (0xa0)
#define TWI_STAT_RXRS_ACK           (0xa8)

#define TWI_STAT_ARBLOST_SLAR_ACK   (0xb0)

/* 10bit Address, second part of address */

#define TWI_STAT_TX_SAW_ACK         (0xd0) /* Second Address byte+Write bit transmitted,ACK received */
#define TWI_STAT_TX_SAW_NAK         (0xd8) /* Second Address byte+Write bit transmitted,ACK not received */

#define TWI_STAT_IDLE               (0xf8) /* No relevant status information,INT_FLAG = 0 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_A64_HARDWARE_A64_TWI_H */
