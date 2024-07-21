/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_bsc.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_BSC_H
#define __ARCH_ARM64_SRC_BCM2711_BSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BSC interface base addresses */

#define BCM_BSC0                                                             \
  (BCM_PERIPHERAL_BASEADDR + 0x000205000) /* BSC/I2C interface 0 */
#define BCM_BSC1                                                             \
  (BCM_PERIPHERAL_BASEADDR + 0x000804000) /* BSC/I2C interface 1 */
#define BCM_BSC3                                                             \
  (BCM_PERIPHERAL_BASEADDR + 0x000205600) /* BSC/I2C interface 2 */
#define BCM_BSC4                                                             \
  (BCM_PERIPHERAL_BASEADDR + 0x000205800) /* BSC/I2C interface 3 */
#define BCM_BSC5                                                             \
  (BCM_PERIPHERAL_BASEADDR + 0x000205a80) /* BSC/I2C interface 4 */
#define BCM_BSC6                                                             \
  (BCM_PERIPHERAL_BASEADDR + 0x000205c00) /* BSC/I2C interface 5 */

/* Number of BSC interfaces. */

#define BCM_BSCS_NUM 7

/* BSC register offsets */

#define BCM_BSC_C_OFFSET 0x00    /* Control */
#define BCM_BSC_S_OFFSET 0x04    /* Status */
#define BCM_BSC_DLEN_OFFSET 0x08 /* Data Length */
#define BCM_BSC_A_OFFSET 0x0c    /* Slave Address */
#define BCM_BSC_FIFO_OFFSET 0x10 /* Data FIFO */
#define BCM_BSC_DIV_OFFSET 0x14  /* Clock Divider */
#define BCM_BSC_DEL_OFFSET 0x18  /* Data Delay */
#define BCM_BSC_CLKT_OFFSET 0x1c /* Clock Stretch Timeout */

/* BSC registers */

#define BCM_BSC_C(reg) (reg + BCM_BSC_C_OFFSET)       /* Control */
#define BCM_BSC_S(reg) (reg + BCM_BSC_S_OFFSET)       /* Status */
#define BCM_BSC_DLEN(reg) (reg + BCM_BSC_DLEN_OFFSET) /* Data Length */
#define BCM_BSC_A(reg) (reg + BCM_BSC_A_OFFSET)       /* Slave Address */
#define BCM_BSC_FIFO(reg) (reg + BCM_BSC_FIFO_OFFSET) /* Data FIFO */
#define BCM_BSC_DIV(reg) (reg + BCM_BSC_DIV_OFFSET)   /* Clock Divider */
#define BCM_BSC_DEL(reg) (reg + BCM_BSC_DEL_OFFSET)   /* Data Delay */
#define BCM_BSC_CLKT(reg)                                                    \
  (reg + BCM_BSC_CLKT_OFFSET) /* Clock Stretch Timeout */

/* BSC register bit definitions */

#define BCM_BSC_C_I2CEN (1 << 15)   /* Enable I2C */
#define BCM_BSC_C_INTR (1 << 10)    /* Interrupt on RX */
#define BCM_BSC_C_INTT (1 << 9)     /* Interrupt on TX */
#define BCM_BSC_C_INTD (1 << 8)     /* Interrupt on DONE */
#define BCM_BSC_C_ST (1 << 7)       /* Start transfer */
#define BCM_BSC_C_CLRMSK (0x3 << 3) /* FIFO clear mask */
#define BCM_BSC_C_NOCLR (0 << 3)    /* No clear action */
#define BCM_BSC_C_CLRFIFO (1 << 3)  /* Clear FIFO one shot */
#define BCM_BSC_C_READ (1 << 0)     /* Read = 1, Write = 0 */

#define BCM_BSC_S_CLKT (1 << 9) /* CLK stretch detected */
#define BCM_BSC_S_ERR (1 << 8)  /* ACK error */
#define BCM_BSC_S_RXF (1 << 7)  /* FIFO full */
#define BCM_BSC_S_TXE (1 << 6)  /* FIFO empty */
#define BCM_BSC_S_RXD (1 << 5)  /* FIFO has data */
#define BCM_BSC_S_TXD (1 << 4)  /* FIFO can accept data */
#define BCM_BSC_S_RXR (1 << 3)  /* FIFO needs reading */
#define BCM_BSC_S_TXW (1 << 2)  /* FIFO needs writing */
#define BCM_BSC_S_DONE (1 << 1) /* Transfer done */
#define BCM_BSC_S_TA (1 << 0)   /* Transfer active */

#define BCM_BSC_DLEN_MASK (0xffff) /* Data length mask */

#define BCM_BSC_A_ADDR (0x7f) /* Slave address */

#define BCM_BSC_FIFO_DATA (0xff) /* FIFO data (RW) */

#define BCM_BSC_DIV_CDIV (0xffff) /* Clock divider */

#define BCM_BSC_DEL_FEDL (0xffff << 16) /* Falling edge delay */
#define BCM_BSC_DEL_REDL (0xffff)       /* Rising edge delay */

#define BCM_BSC_CLKT_TOUT (0xffff) /* Clock stretch timeout value */

#endif /* __ARCH_ARM64_SRC_BCM2711_BSC_H */
