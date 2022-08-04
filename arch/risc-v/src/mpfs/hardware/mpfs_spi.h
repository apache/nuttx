/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_spi.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SPI_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SPI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONTROL register */

#define MPFS_SPI_RESET                (1 << 31)
#define MPFS_SPI_OENOFF               (1 << 30)
#define MPFS_SPI_BIGFIFO              (1 << 29)
#define MPFS_SPI_CLKMODE              (1 << 28)
#define MPFS_SPI_FRAMEURUN            (1 << 27)
#define MPFS_SPI_SPS                  (1 << 26)
#define MPFS_SPI_SPH                  (1 << 25)
#define MPFS_SPI_SPO                  (1 << 24)
#define MPFS_SPI_FRAMECNT             (0xffff << 8)
#define MPFS_SPI_INTTXTURUN           (1 << 7)
#define MPFS_SPI_INTRXOVRFLOW         (1 << 6)
#define MPFS_SPI_INTTXDATA            (1 << 5)
#define MPFS_SPI_INTRXDATA            (1 << 4)
#define MPFS_SPI_TRANSFPRTL           (1 << 2) | (1 << 3)
#define MPFS_SPI_MODE                 (1 << 1)
#define MPFS_SPI_ENABLE               (1 << 0)

/* FRAMESIZE register */

#define MPFS_SPI_FRAMESIZE            (0x3F)

/* STATUS register */

#define MPFS_SPI_ACTIVE               (1 << 14)
#define MPFS_SPI_SSEL                 (1 << 13)
#define MPFS_SPI_FRAMESTART           (1 << 12)
#define MPFS_SPI_TXFIFOEMPNXT         (1 << 11)
#define MPFS_SPI_TXFIFOEMP            (1 << 10)
#define MPFS_SPI_TXFIFOFULNXT         (1 << 9)
#define MPFS_SPI_TXFIFOFUL            (1 << 8)
#define MPFS_SPI_RXFIFOEMPNXT         (1 << 7)
#define MPFS_SPI_RXFIFOEMP            (1 << 6)
#define MPFS_SPI_RXFIFOFULNXT         (1 << 5)
#define MPFS_SPI_RXFIFOFUL            (1 << 4)
#define MPFS_SPI_TXUNDERRUN           (1 << 3)
#define MPFS_SPI_RXOVERFLOW           (1 << 2)
#define MPFS_SPI_RXDATRCED            (1 << 1)
#define MPFS_SPI_TXDATSENT            (1 << 0)

/* INT_CLEAR register */

#define MPFS_SPI_SSEND                (1 << 5)
#define MPFS_SPI_CMDINT               (1 << 4)
#define MPFS_SPI_TXCHUNDRUN           (1 << 3)
#define MPFS_SPI_RXCHOVRFLW           (1 << 2)
#define MPFS_SPI_RXRDONECLR           (1 << 1)
#define MPFS_SPI_TXDONECLR            (1 << 0)

/* INTMASK register */

#define MPFS_SPI_SSENDMSKINT          (1 << 5)
#define MPFS_SPI_CMDMSKINT            (1 << 4)
#define MPFS_SPI_TXCHUNDDMSKINT       (1 << 3)
#define MPFS_SPI_RXCHOVRFMSKINT       (1 << 2)
#define MPFS_SPI_RXRDYMSKINT          (1 << 1)
#define MPFS_SPI_TXDONEMSKINT         (1 << 0)

/* INTRAW register */

#define MPFS_SPI_TXCHUNDR             (1 << 3)
#define MPFS_SPI_RXOVRFLW             (1 << 2)
#define MPFS_SPI_RXRDY                (1 << 1)
#define MPFS_SPI_TXDONE               (1 << 0)

/* CONTROL2 register */

#define MPFS_SPI_INTEN_SSEND          (1 << 5)
#define MPFS_SPI_INTEN_CMD            (1 << 4)
#define MPFS_SPI_DISFRMCNT            (1 << 2)
#define MPFS_SPI_AUTOPOLL             (1 << 1)
#define MPFS_SPI_AUTOSTATUS           (1 << 0)

/* COMMAND register */

#define MPFS_SPI_TXNOW                (1 << 6)
#define MPFS_SPI_AUTOSTALL            (1 << 5)
#define MPFS_SPI_CLRFRAMECNT          (1 << 4)
#define MPFS_SPI_TXFIFORST            (1 << 3)
#define MPFS_SPI_RXFIFORST            (1 << 2)
#define MPFS_SPI_AUTOEMPTY            (1 << 1)
#define MPFS_SPI_AUTOFILL             (1 << 0)

/* HWSTATUS register */

#define MPFS_SPI_USER                 (1 << 2) | (1 << 3)
#define MPFS_SPI_TXBUSY               (1 << 1)
#define MPFS_SPI_RXBUSY               (1 << 0)

/* STAT8 register */

#define MPFS_SPI_S8_ACTIVEL           (1 << 6)
#define MPFS_SPI_S8_SSEL              (1 << 6)
#define MPFS_SPI_S8_TXUNDERRUN        (1 << 5)
#define MPFS_SPI_S8_RXOVERFLOW        (1 << 4)
#define MPFS_SPI_S8_TXFIFOFUL         (1 << 3)
#define MPFS_SPI_S8_RXFIFOEMP         (1 << 2)
#define MPFS_SPI_S8_DONE              (1 << 1)
#define MPFS_SPI_S8_FRAMESTART        (1 << 0)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SPI_H */
