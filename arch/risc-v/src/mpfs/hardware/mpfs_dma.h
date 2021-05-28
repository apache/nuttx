/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_dma.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_DMA_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define MPFS_DMA_CONTROL_OFFSET          0x0000 /* Channel control register */
#define MPFS_DMA_NEXT_CONFIG_OFFSET      0x0004 /* Next transfer type */
#define MPFS_DMA_NEXT_BYTES_OFFSET       0x0008 /* Number of bytes to move */
#define MPFS_DMA_NEXT_DESTINATION_OFFSET 0x0010 /* Destination start address */
#define MPFS_DMA_NEXT_SOURCE_OFFSET      0x0018 /* Source start address */
#define MPFS_DMA_EXEC_CONFIG_OFFSET      0x0104 /* Active transfer type */
#define MPFS_DMA_EXEC_BYTES_OFFSET       0x0108 /* Number of bytes remaining */
#define MPFS_DMA_EXEC_DESTINATION_OFFSET 0x0110 /* Destination current address */
#define MPFS_DMA_EXEC_SOURCE_OFFSET      0x0118 /* Source current address */

#define MPFS_DMA_CHANNEL_OFFSET          0x1000 /* Offset to channels */

#define MPFS_DMA_REG_OFFSET(x) \
          (uint64_t)(MPFS_PDMA_BASE + (MPFS_DMA_CHANNEL_OFFSET * (x)))

/* Register bit field definitions *******************************************/

/* Control register */

/* Indicates that the channel is in use. Setting this bit clears all of the
 * channel’s Next registers (NextConfig, NextBytes, NextDestination, and
 * NextSource). This bit can only be cleared when run (CR bit 0) is low.
 */

#define DMA_CONTROL_CLAIM_SHIFT      (0) /* Bit: 0: claim */
#define DMA_CONTROL_CLAIM_MASK       (1 << DMA_CONTROL_CLAIM_SHIFT)
#  define DMA_CONTROL_CLAIM          (0 << DMA_CONTROL_CLAIM_SHIFT)

/* Setting this bit starts a DMA transfer by copying the Next registers
 * into their Exec counterparts.
 */

#define DMA_CONTROL_RUN_SHIFT        (1) /* Bit: 1: run */
#define DMA_CONTROL_RUN_MASK         (1 << DMA_CONTROL_RUN_SHIFT)
#  define DMA_CONTROL_RUN            (1 << DMA_CONTROL_RUN_SHIFT)

/* Setting this bit will trigger the channel’s Done interrupt once
 * a transfer is complete.
 */

#define DMA_CONTROL_DONEIE_SHIFT     (14) /* Bit: 14: Done Irq enable */
#define DMA_CONTROL_DONEIE_MASK      (1 << DMA_CONTROL_DONEIE_SHIFT)
#  define DMA_CONTROL_DONEIE         (1 << DMA_CONTROL_DONEIE_SHIFT)

/* Setting this bit will trigger the channel’s Done interrupt once
 * a transfer is complete.
 */

#define DMA_CONTROL_ERRORIE_SHIFT    (15) /* Bit: 15: Error Irq enable */
#define DMA_CONTROL_ERRORIE_MASK     (1 << DMA_CONTROL_ERRORIE_SHIFT)
#  define DMA_CONTROL_ERRORIE        (1 << DMA_CONTROL_ERRORIE_SHIFT)

/* Indicates that a transfer has completed since the channel was claimed */

#define DMA_CONTROL_DONE_SHIFT       (30) /* Bit: 30: Done */
#define DMA_CONTROL_DONE_MASK        (1 << DMA_CONTROL_DONE_SHIFT)
#  define DMA_CONTROL_DONE           (1 << DMA_CONTROL_DONE_SHIFT)

/* Indicates that a transfer error has occurred since the channel
 * was claimed
 */

#define DMA_CONTROL_ERROR_SHIFT      (31) /* Bit: 31: Error */
#define DMA_CONTROL_ERROR_MASK       (1 << DMA_CONTROL_ERROR_SHIFT)
#  define DMA_CONTROL_ERROR          (1 << DMA_CONTROL_ERROR_SHIFT)

/* Channel Next Configuration Register */

/* If set, the Exec registers are reloaded from the Next registers once a
 * transfer is complete. The repeat bit must be cleared by software
 * for the sequence to stop
 */

#define DMA_NEXT_CONFIG_REPEAT_SHIFT (2) /* Bit: 2: repeat */
#define DMA_NEXT_CONFIG_REPEAT_MASK  (1 << DMA_NEXT_CONFIG_REPEAT_SHIFT)
#  define DMA_NEXT_CONFIG_REPEAT     (1 << DMA_NEXT_CONFIG_REPEAT_SHIFT)

/* Enforces strict ordering by only allowing one of each transfer type
 * in-flight at a time
 */

#define DMA_NEXT_CONFIG_ORDER_SHIFT  (3) /* Bit: 3: order */
#define DMA_NEXT_CONFIG_ORDER_MASK   (1 << DMA_NEXT_CONFIG_ORDER_SHIFT)
#  define DMA_NEXT_CONFIG_ORDER      (1 << DMA_NEXT_CONFIG_ORDER_SHIFT)

/* WSIZE and RSIZE. Base 2 Logarithm of DMA transaction sizes.
 * Example: 0 is 1 byte, 3 is 8 bytes, 5 is 32 bytes
 * These fields are WARL (Write-Any Read-Legal), so the actual size used
 * can be determined by reading the field after writing the requested size.
 * */

#define DMA_NEXT_CONFIG_WSIZE_SHIFT  (24) /* Bits: 24-27: write size */
#define DMA_NEXT_CONFIG_WSIZE_MASK   (15 << DMA_NEXT_CONFIG_WSIZE_SHIFT)
#  define DMA_NEXT_CONFIG_WSIZE(x)   (x << DMA_NEXT_CONFIG_WSIZE_SHIFT)

#define DMA_NEXT_CONFIG_RSIZE_SHIFT  (28) /* Bits: 28-31: read size */
#define DMA_NEXT_CONFIG_RSIZE_MASK   (15 << DMA_NEXT_CONFIG_RSIZE_SHIFT)
#  define DMA_NEXT_CONFIG_RSIZE(x)   (x << DMA_NEXT_CONFIG_RSIZE_SHIFT)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_DMA_H */
