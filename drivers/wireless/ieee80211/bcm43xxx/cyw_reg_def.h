/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/cyw_reg_def.h
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

#ifndef __DRIVERS_WIRELESS_CYW43439_CYW_REG_DEF_H
#define __DRIVERS_WIRELESS_CYW43439_CYW_REG_DEF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>

#include <stdint.h>
#include <stdbool.h>

#include "bcmf_sdio_regs.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* --- gSPI Registers --- */

#define CYW_REG_SETUP          (0x0000)    /* 32-bit register */
#define CYW_REG_INTERRUPT      (0x0004)    /* 16-bit register */
#define CYW_REG_INTR_ENA       (0x0006)    /* 16-bit register */
#define CYW_REG_STATUS         (0x0008)    /* 32-bit register */
#define CYW_REG_F1_INFO        (0x000c)    /* 16-bit register */
#define CYW_REG_F2_INFO        (0x000e)    /* 16-bit register */
#define CYW_REG_TEST_RO        (0x0014)    /* 32-bit register */
#define CYW_REG_TEST_RW        (0x0018)    /* 32-bit register */
#define CYW_REG_RESP_DELAY_F0  (0x001c)    /*  8-bit register */
#define CYW_REG_RESP_DELAY_F1  (0x001d)    /*  8-bit register */
#define CYW_REG_RESP_DELAY_F2  (0x001e)    /*  8-bit register */
#define CYW_REG_RESP_DELAY_F3  (0x001f)    /*  8-bit register */

/* --- Registers --- */

#define CYW_REG_SETUP_WORD_LEN_32        (1<<0)
#define CYW_REG_SETUP_BIG_ENDIAN         (1<<1)
#define CYW_REG_SETUP_HIGH_SPEED         (1<<4)
#define CYW_REG_SETUP_INT_POLARITY       (1<<5)
#define CYW_REG_SETUP_WAKE_UP            (1<<7)
#define CYW_REG_SETUP_RESP_DELAY_SHIFT   (8)
#define CYW_REG_SETUP_RESP_DELAY_MASK    (0x00FF << CYW_REG_SETUP_RESP_DELAY_SHIFT)
#define CYW_REG_STAT_ENA_STAT_ENA        (1<<16)
#define CYW_REG_STAT_ENA_INTR_STAT       (1<<17)

#define CYW_REG_INTERRUPT_DATA_NOT_AVAIL (1<<0)
#define CYW_REG_INTERRUPT_FIFO_UNDERFLOW (1<<1)
#define CYW_REG_INTERRUPT_FIFO_OVERFLOW  (1<<2)
#define CYW_REG_INTERRUPT_COMMAND_ERROR  (1<<3)
#define CYW_REG_INTERRUPT_DATA_ERROR     (1<<4)
#define CYW_REG_INTERRUPT_F2_PKT_AVAIL   (1<<5)
#define CYW_REG_INTERRUPT_F3_PKT_AVAIL   (1<<6)
#define CYW_REG_INTERRUPT_F1_OVERFLOW    (1<<7)
#define CYW_REG_INTERRUPT_F1_INTERRUPT   (1<<13)
#define CYW_REG_INTERRUPT_F2_INTERRUPT   (1<<14)
#define CYW_REG_INTERRUPT_F3_INTERRUPT   (1<<15)

#define CYW_REG_INTR_ENA_DATA_NOT_AVAIL  (1<<0)
#define CYW_REG_INTR_ENA_FIFO_UNDERFLOW  (1<<1)
#define CYW_REG_INTR_ENA_FIFO_OVERFLOW   (1<<2)
#define CYW_REG_INTR_ENA_COMMAND_ERROR   (1<<3)
#define CYW_REG_INTR_ENA_DATA_ERROR      (1<<4)
#define CYW_REG_INTR_ENA_F2_PKT_AVAIL    (1<<5)
#define CYW_REG_INTR_ENA_F3_PKT_AVAIL    (1<<6)
#define CYW_REG_INTR_ENA_F1_OVERFLOW     (1<<7)
#define CYW_REG_INTR_ENA_F1_INTERRUPT    (1<<13)
#define CYW_REG_INTR_ENA_F2_INTERRUPT    (1<<14)
#define CYW_REG_INTR_ENA_F3_INTERRUPT    (1<<15)

#define CYW_REG_STATUS_DATA_NOT_AVAIL    (1<<0)
#define CYW_REG_STATUS_FIFO_UNDERFLOW    (1<<1)
#define CYW_REG_STATUS_FIFO_OVERFLOW     (1<<2)
#define CYW_REG_STATUS_F2_INTERRUPT      (1<<3)
#define CYW_REG_STATUS_F3_INTERRUPT      (1<<4)
#define CYW_REG_STATUS_F2_RECEIVE_RDY    (1<<5)
#define CYW_REG_STATUS_F3_RECEIVE_RDY    (1<<6)
#define CYW_REG_STATUS_CMD_DATA_ERROR    (1<<7)
#define CYW_REG_STATUS_F2_PKT_AVAIL      (1<<8)
#define CYW_REG_STATUS_F2_PKT_LEN_SHIFT  (9)
#define CYW_REG_STATUS_F2_PKT_LEN_MASK   (0x7FF << CYW_REG_STATUS_F2_PKT_LEN_SHIFT)
#define CYW_REG_STATUS_F3_PKT_AVAIL      (1<<20)
#define CYW_REG_STATUS_F3_PKT_LEN_SHIFT  (21)
#define CYW_REG_STATUS_F3_PKT_LEN_MASK   (0x7FF << CYW_REG_STATUS_F3_PKT_LEN_SHIFT)

#define CYW_REG_F1_INFO_ENABLED          (1<<0)
#define CYW_REG_F1_INFO_READY            (1<<1)
#define CYW_REG_F1_INFO_MAX_SIZE_SHIFT   (2)
#define CYW_REG_F1_INFO_MAX_SIZE_MASK    (0x0FFF << CYW_REG_F1_INFO_MAX_SIZE_SHIFT)

#define CYW_REG_F2_INFO_ENABLED          (1<<0)
#define CYW_REG_F2_INFO_READY            (1<<1)
#define CYW_REG_F2_INFO_MAX_SIZE_SHIFT   (2)
#define CYW_REG_F2_INFO_MAX_SIZE_MASK    (0x0FFF << CYW_REG_F2_INFO_MAX_SIZE_SHIFT)

#define CYW_REG_RESP_DELAY_F0_SHIFT      (0)
#define CYW_REG_RESP_DELAY_F0_MASK       (0x00FF << CYW_REG_RESP_DELAY_F0_SHIFT)
#define CYW_REG_RESP_DELAY_F1_SHIFT      (8)
#define CYW_REG_RESP_DELAY_F1_MASK       (0x00FF << CYW_REG_RESP_DELAY_F1_SHIFT)
#define CYW_REG_RESP_DELAY_F2_SHIFT      (16)
#define CYW_REG_RESP_DELAY_F2_MASK       (0x00FF << CYW_REG_RESP_DELAY_F2_SHIFT)
#define CYW_REG_RESP_DELAY_F3_SHIFT      (24)
#define CYW_REG_RESP_DELAY_F3_MASK       (0x00FF << CYW_REG_RESP_DELAY_F3_SHIFT)

#define CYW_REG_TEST_RO_PATTERN          (0xfeedbead)

#define CYW_STATUS_DATA_NOT_AVAIL        (1<<0)  /* data not avail on read       */
#define CYW_STATUS_FIFO_UNDERFLOW        (1<<1)  /* read underflow (F2, F3)      */
#define CYW_STATUS_FIFO_OVERFLOW         (1<<2)  /* write overflow  (F1, F2, F3) */
#define CYW_STATUS_F2_INTERRUPT          (1<<3)  /* F2 channel interrupt         */
#define CYW_STATUS_F3_INTERRUPT          (1<<4)
#define CYW_STATUS_F2_RECEIVE_RDY        (1<<5)  /* F2 ready to receive data     */
#define CYW_STATUS_F3_RECEIVE_RDY        (1<<6)
#define CYW_STATUS_CMD_DATA_ERROR        (1<<7)
#define CYW_STATUS_F2_PKT_AVAIL          (1<<8)  /* F2 has data to read          */
#define CYW_STATUS_F2_PKT_LEN_SHIFT      (9)     /* Length of avail F2 data      */
#define CYW_STATUS_F2_PKT_LEN_MASK       (0x7FF << CYW_REG_STATUS_F2_PKT_LEN_SHIFT)
#define CYW_STATUS_F3_PKT_AVAIL          (1<<20)
#define CYW_STATUS_F3_PKT_LEN_SHIFT      (21)
#define CYW_STATUS_F3_PKT_LEN_MASK       (0x7FF << CYW_REG_STATUS_F3_PKT_LEN_SHIFT)

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

#endif /* __DRIVERS_WIRELESS_CYW43439_CYW_REG_DEF_H */
