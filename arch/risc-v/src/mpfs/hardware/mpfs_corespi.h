/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_corespi.h
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

#ifndef __ARCH_RISC_V_SRC_MPFS_HARDWARE_MPFS_CORESPI_H
#define __ARCH_RISC_V_SRC_MPFS_HARDWARE_MPFS_CORESPI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONTROL register */

#define MPFS_SPI_OENOFF               (1 << 7)
#define MPFS_SPI_FRAMEURUN            (1 << 6)
#define MPFS_SPI_INTTXTURUN           (1 << 5)
#define MPFS_SPI_INTRXOVRFLOW         (1 << 4)
#define MPFS_SPI_INTTXDONE            (1 << 3)
#define MPFS_SPI_MODE                 (1 << 1)
#define MPFS_SPI_ENABLE               (1 << 0)

/* INT_CLEAR/RAW/MASK register */

#define MPFS_SPI_TXRFM                (1 << 7)
#define MPFS_SPI_DATA_RX              (1 << 6)
#define MPFS_SPI_SSEND                (1 << 5)
#define MPFS_SPI_CMDINT               (1 << 4)
#define MPFS_SPI_TXCHUNDRUN           (1 << 3)
#define MPFS_SPI_RXCHOVRFLW           (1 << 2)
#define MPFS_SPI_TXDONE               (1 << 0)

/* STATUS register */

#define MPFS_SPI_ACTIVE               (1 << 7)
#define MPFS_SPI_SSEL                 (1 << 6)
#define MPFS_SPI_TXUNDERRUN           (1 << 5)
#define MPFS_SPI_RXOVERFLOW           (1 << 4)
#define MPFS_SPI_TXFULL               (1 << 3)
#define MPFS_SPI_RXEMPTY              (1 << 2)
#define MPFS_SPI_DONE                 (1 << 1)
#define MPFS_SPI_FIRSTFRAME           (1 << 0)

/* CONTROL2 register */

#define MPFS_SPI_INTEN_TXFRM          (1 << 7)
#define MPFS_SPI_INTEN_DATA_RX        (1 << 6)
#define MPFS_SPI_INTEN_SSEND          (1 << 5)
#define MPFS_SPI_INTEN_CMD            (1 << 4)
#define MPFS_SPI_CMDSIZE_MASK         (7 << 0)
#define MPFS_SPI_CMDSIZE_SHIFT        (1)

/* COMMAND register */

#define MPFS_SPI_TXFIFORST            (1 << 1)
#define MPFS_SPI_RXFIFORST            (1 << 0)

#endif /* __ARCH_RISC_V_SRC_MPFS_HARDWARE_MPFS_CORESPI_H */
