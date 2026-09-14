/****************************************************************************
 * arch/arm/src/am67/am67_i2c_hw.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_I2C_HW_H
#define __ARCH_ARM_SRC_AM67_AM67_I2C_HW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define AM67_I2C_SYSC_OFFSET                  0x0010
#define AM67_I2C_IRQ_STAT_RAW_OFFSET          0x0024
#define AM67_I2C_IRQ_STAT_OFFSET              0x0028
#define AM67_I2C_IRQ_EN_SET_OFFSET            0x002c
#define AM67_I2C_IRQ_EN_CLR_OFFSET            0x0030
#define AM67_I2C_WE_OFFSET                    0x0034
#define AM67_I2C_DMA_RX_EN_SET_OFFSET         0x0038
#define AM67_I2C_DMA_TX_EN_SET_OFFSET         0x003c
#define AM67_I2C_DMA_RX_EN_CLR_OFFSET         0x0040
#define AM67_I2C_DMA_TX_EN_CLR_OFFSET         0x0044
#define AM67_I2C_DMA_RX_WAKE_EN_OFFSET        0x0048
#define AM67_I2C_DMA_TX_WAKE_EN_OFFSET        0x004c
#define AM67_I2C_SYSS_OFFSET                  0x0090
#define AM67_I2C_BUF_OFFSET                   0x0094
#define AM67_I2C_CNT_OFFSET                   0x0098
#define AM67_I2C_DATA_OFFSET                  0x009c
#define AM67_I2C_CON_OFFSET                   0x00a4
#define AM67_I2C_OA_OFFSET                    0x00a8
#define AM67_I2C_SA_OFFSET                    0x00ac
#define AM67_I2C_PSC_OFFSET                   0x00b0
#define AM67_I2C_SCLL_OFFSET                  0x00b4
#define AM67_I2C_SCLH_OFFSET                  0x00b8
#define AM67_I2C_SYSTEST_OFFSET               0x00bc
#define AM67_I2C_BUFSTAT_OFFSET               0x00c0
#define AM67_I2C_OA1_OFFSET                   0x00c4
#define AM67_I2C_OA2_OFFSET                   0x00c8
#define AM67_I2C_OA3_OFFSET                   0x00cc
#define AM67_I2C_ACTOA_OFFSET                 0x00d0
#define AM67_I2C_SBLOCK_OFFSET                0x00d4

/* Peripheral base addresses ************************************************/

#define AM67_I2C0_VADDR                       0x04900000
#define AM67_WKUP_I2C0_VADDR                  0x2b200000

/* Register bit field definitions *******************************************/

#define I2C_SYSC_AUTOIDLE                       (1 << 0)  /* Bit 0:  Auto-idle */
#define I2C_SYSC_SRST                           (1 << 1)  /* Bit 1:  SoftReset */
#define I2C_SYSC_WAKEUP                         (1 << 2)  /* Bit 2:  Enable Wakeup control */
#define I2C_SYSC_IDLE_SHIFT                     (3)       /* Bits 3-4:  Idle Mode selection */
#define I2C_SYSC_IDLE_MASK                      (3 << I2C_SYSC_IDLE_SHIFT)
#  define I2C_SYSC_IDLE_FORCE                   (0 << I2C_SYSC_IDLE_SHIFT) /* Force-idle mode */
#  define I2C_SYSC_IDLE_NO                      (1 << I2C_SYSC_IDLE_SHIFT) /* No-idle mode */
#  define I2C_SYSC_IDLE_SMART                   (2 << I2C_SYSC_IDLE_SHIFT) /* Smart-idle mode */
#  define I2C_SYSC_IDLE_SMART_WKUP              (3 << I2C_SYSC_IDLE_SHIFT) /* Smart-idle Wakeup mode */

#define I2C_SYSC_CLK_SHIFT                      (8)  /* Bits 8-9:  Clock Activity selection */
#define I2C_SYSC_CLK_MASK                       (3 << I2C_SYSC_CLK_SHIFT)
#  define I2C_SYSC_CLK_NONE                     (0 << I2C_SYSC_CLK_SHIFT) /* Both clocks can be cut off */
#  define I2C_SYSC_CLK_OCP                      (1 << I2C_SYSC_CLK_SHIFT) /* Only Interface/OCP clock must be kept active */
#  define I2C_SYSC_CLK_FUNC                     (2 << I2C_SYSC_CLK_SHIFT) /* Only functions clock must be kept active */
#  define I2C_SYSC_CLK_BOTH                     (3 << I2C_SYSC_CLK_SHIFT) /* Both clocks must be kept active */

#define I2C_IRQ_AL                              (1 << 0)   /* Bit 0:  Arbitration lost */
#define I2C_IRQ_NACK                            (1 << 1)   /* Bit 1:  No acknowledgment */
#define I2C_IRQ_ARDY                            (1 << 2)   /* Bit 2:  Register access ready */
#define I2C_IRQ_RRDY                            (1 << 3)   /* Bit 3:  Receive data ready */
#define I2C_IRQ_XRDY                            (1 << 4)   /* Bit 4:  Transmit data ready */
#define I2C_IRQ_GC                              (1 << 5)   /* Bit 5:  General call */
#define I2C_IRQ_STC                             (1 << 6)   /* Bit 6:  Start Condition */
#define I2C_IRQ_AERR                            (1 << 7)   /* Bit 7:  Access Error */
#define I2C_IRQ_BF                              (1 << 8)   /* Bit 8:  Bus Free */
#define I2C_IRQ_AAS                             (1 << 9)   /* Bit 9:  Address recognized as slave */
#define I2C_IRQ_XUDF                            (1 << 10)  /* Bit 10:  Transmit underflow */
#define I2C_IRQ_ROVR                            (1 << 11)  /* Bit 11:  Receive overrun */
#define I2C_IRQ_BB                              (1 << 12)  /* Bit 12:  Bus busy */
#define I2C_IRQ_RDR                             (1 << 13)  /* Bit 13:  Receive draining IRQ */
#define I2C_IRQ_XDR                             (1 << 14)  /* Bit 14:  Transmit draining IRQ */

#define I2C_IRQ_ERRORMASK (I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_AERR | I2C_IRQ_XUDF | I2C_IRQ_ROVR)

#define I2C_STS_CLEARMASK (I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_ARDY | I2C_IRQ_RRDY | I2C_IRQ_XRDY \
                           | I2C_IRQ_GC | I2C_IRQ_STC | I2C_IRQ_AERR | I2C_IRQ_BF | I2C_IRQ_AAS \
                           | I2C_IRQ_XUDF | I2C_IRQ_ROVR | I2C_IRQ_BB | I2C_IRQ_RDR | I2C_IRQ_XDR)

#define I2C_ICR_CLEARMASK (I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_ARDY | I2C_IRQ_RRDY | I2C_IRQ_XRDY \
                           | I2C_IRQ_GC | I2C_IRQ_STC | I2C_IRQ_AERR | I2C_IRQ_BF | I2C_IRQ_AAS \
                           | I2C_IRQ_XUDF | I2C_IRQ_ROVR | I2C_IRQ_RDR | I2C_IRQ_XDR)

#define I2C_WE_AL                               (1 << 0)   /* Bit 0:  Arbitration lost */
#define I2C_WE_NACK                             (1 << 1)   /* Bit 1:  No acknowledgment */
#define I2C_WE_ARDY                             (1 << 2)   /* Bit 2:  Register access ready */
#define I2C_WE_DRDY                             (1 << 3)   /* Bit 3:  Receive/Transmit data ready */
#define I2C_WE_GC                               (1 << 5)   /* Bit 5:  General call */
#define I2C_WE_STC                              (1 << 6)   /* Bit 6:  Start Condition */
#define I2C_WE_BF                               (1 << 8)   /* Bit 8:  Bus Free */
#define I2C_WE_AAS                              (1 << 9)   /* Bit 9:  Address recognized as slave */
#define I2C_WE_XUDF                             (1 << 10)  /* Bit 10:  Transmit underflow */
#define I2C_WE_ROVR                             (1 << 11)  /* Bit 11:  Receive overrun */
#define I2C_WE_RDR                              (1 << 13)  /* Bit 13:  Receive draining IRQ */
#define I2C_WE_XDR                              (1 << 14)  /* Bit 14:  Transmit draining IRQ */

#define I2C_DMA_ENABLE                          (1 << 0)  /* Bit 0:  DMA channel enable */

#define I2C_SYSS_RST_DONE                       (1 << 0)  /* Bit 0:  Reset done */

#define I2C_BUF_TXTRSH_SHIFT                    (0)  /* Bits 0-5:  Threshold value for FIFO buffer in TX mode */
#define I2C_BUF_TXTRSH_MASK                     (63 << I2C_BUF_TXTRSH_SHIFT)
#define I2C_BUF_TXFIFO_CLR                      (1 << 6)  /* Bit 6:  Transmit FIFO clear */
#define I2C_BUF_XDMA_EN                         (1 << 7)  /* Bit 7:  Transmit DMA channel enable */

#define I2C_BUF_RXTRSH_SHIFT                    (8)  /* Bits 8-13:  Threshold value for FIFO buffer in RX mode */
#define I2C_BUF_RXTRSH_MASK                     (63 << I2C_BUF_RXTRSH_SHIFT)
#define I2C_BUF_RXFIFO_CLR                      (1 << 14)  /* Bit 14:  Receive FIFO clear */
#define I2C_BUF_RDMA_EN                         (1 << 15)  /* Bit 15:  Receive DMA channel enable */

#define I2C_CNT_SHIFT                           (0)  /* Bits 0-15:  Data count */
#define I2C_CNT_MASK                            (65535 << I2C_CNT_SHIFT)

#define I2C_DATA_SHIFT                          (0)  /* Bits 0-7:  Transmit/Receive data FIFO endpoint */
#define I2C_DATA_MASK                           (255 << I2C_DATA_SHIFT)

#define I2C_CON_STT                             (1 << 0)   /* Bit 0:  Start condition (I2C master mode only) */
#define I2C_CON_STP                             (1 << 1)   /* Bit 1:  Stop condition (I2C master mode only) */
#define I2C_CON_XOA3                            (1 << 4)   /* Bit 4:  Expand own address 3 */
#define I2C_CON_XOA2                            (1 << 5)   /* Bit 5:  Expand own address 2 */
#define I2C_CON_XOA1                            (1 << 6)   /* Bit 6:  Expand own address 1 */
#define I2C_CON_XOA0                            (1 << 7)   /* Bit 7:  Expand own address 0 */
#define I2C_CON_XSA                             (1 << 8)   /* Bit 8:  Expand slave address */
#define I2C_CON_TRX                             (1 << 9)   /* Bit 9:  Transmitter/receiver mode (I2C master mode only) */
#define I2C_CON_MST                             (1 << 10)  /* Bit 10:  Master/slave mode */
#define I2C_CON_STB                             (1 << 11)  /* Bit 11:  Start byte mode (I2C master mode only) */
#define I2C_CON_OPMODE_SHIFT                    (12)       /* Bits 12-13:  Operation mode selection */
#define I2C_CON_OPMODE_MASK                     (3 << I2C_CON_OPMODE_SHIFT)
#  define I2C_CON_OPMODE_FAST                   (0 << I2C_CON_OPMODE_SHIFT)
#define I2C_CON_EN                              (1 << 15)  /* Bit 15:  I2C module enable */

#define I2C_SA_SHIFT                            (0)  /* Bits 0-9:  Slave address */
#define I2C_SA_MASK                             (0x3ff << I2C_SA_SHIFT)

#define I2C_PSC_SHIFT                           (0)  /* Bits 0-7:  Fast/Standard mode prescale sampling clock divider */
#define I2C_PSC_MASK                            (255 << I2C_PSC_SHIFT)

#define I2C_SCLL_SHIFT                          (0)  /* Bits 0-7:  Fast/Standard mode SCL low time */
#define I2C_SCLL_MASK                           (255 << I2C_SCLL_SHIFT)

#define I2C_SCLH_SHIFT                          (0)  /* Bits 0-7:  Fast/Standard mode SCL high time. */
#define I2C_SCLH_MASK                           (255 << I2C_SCLH_SHIFT)

#define I2C_SYSTEST_SDA_O                       (1 << 0)   /* Bit 0:  SDA line drive output value */
#define I2C_SYSTEST_SDA_I                       (1 << 1)   /* Bit 1:  SDA line sense input value */
#define I2C_SYSTEST_SCL_O                       (1 << 2)   /* Bit 2:  SCL line drive output value */
#define I2C_SYSTEST_SCL_I                       (1 << 3)   /* Bit 3:  SCL line sense input value */
#define I2C_SYSTEST_SDA_O_FUNC                  (1 << 5)   /* Bit 5:  SDA line output value (functional mode) */
#define I2C_SYSTEST_SDA_I_FUNC                  (1 << 6)   /* Bit 6:  SDA line input value (functional mode) */
#define I2C_SYSTEST_SCL_O_FUNC                  (1 << 7)   /* Bit 7:  SCL line output value (functional mode) */
#define I2C_SYSTEST_SCL_I_FUNC                  (1 << 8)   /* Bit 8:  SCL line input value (functional mode) */
#define I2C_SYSTEST_SSB                         (1 << 11)  /* Bit 11:  Set status bits */

#define I2C_SYSTEST_TMODE_SHIFT                 (12)  /* Bits 12-13:  Test mode select */
#define I2C_SYSTEST_TMODE_MASK                  (3 << I2C_SYSTEST_TMODE_SHIFT)
#  define I2C_SYSTEST_TMODE_FUNC                (0 << I2C_SYSTEST_TMODE_SHIFT) /* Functional mode */
#  define I2C_SYSTEST_TMODE_SCL                 (2 << I2C_SYSTEST_TMODE_SHIFT) /* Test of SCL counters (SCLL, SCLH, PSC) */
#  define I2C_SYSTEST_TMODE_LOOPBACK            (3 << I2C_SYSTEST_TMODE_SHIFT) /* Loop back mode select + SDA/SCL IO mode select */

#define I2C_SYSTEST_FREE                        (1 << 14)  /* Bit 14:  Free running mode (on breakpoint) */
#define I2C_SYSTEST_ST_EN                       (1 << 15)  /* Bit 15:  System test enable */

#define I2C_BUFSTAT_TXSTAT_SHIFT                (0)  /* Bits 0-5:  TX buffer status */
#define I2C_BUFSTAT_TXSTAT_MASK                 (63 << I2C_BUFSTAT_TXSTAT_SHIFT)
#define I2C_BUFSTAT_RXSTAT_SHIFT                (8)  /* Bits 8-13:  RX buffer status */
#define I2C_BUFSTAT_RXSTAT_MASK                 (63 << I2C_BUFSTAT_RXSTAT_SHIFT)
#define I2C_BUFSTAT_FIFODEPTH_SHIFT             (14)  /* Bits 14-15:  Internal FIFO buffers depth */
#define I2C_BUFSTAT_FIFODEPTH_MASK              (3 << I2C_BUFSTAT_FIFODEPTH_SHIFT)

#define I2C_OA_SHIFT                            (0)  /* Bits 0-9:  Own address */
#define I2C_OA_MASK                             (0x3ff << I2C_OA_SHIFT)

#define I2C_OA0_SELECT                          (1 << 0)  /* Bit 0:  Own address 0 */
#define I2C_OA1_SELECT                          (1 << 1)  /* Bit 1:  Own address 1 */
#define I2C_OA2_SELECT                          (1 << 2)  /* Bit 2:  Own address 2 */
#define I2C_OA3_SELECT                          (1 << 3)  /* Bit 3:  Own address 3 */

#endif /* __ARCH_ARM_SRC_AM67_AM67_I2C_HW_H */
