/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_sio.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SIO_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_SIO_CPUID_OFFSET               0x000000  /* Processor core identifier Value is 0 when read from processor core 0, and 1 when read from processor core 1. */
#define RP2040_SIO_GPIO_IN_OFFSET             0x000004  /* Input value for GPIO pins */
#define RP2040_SIO_GPIO_HI_IN_OFFSET          0x000008  /* Input value for QSPI pins */
#define RP2040_SIO_GPIO_OUT_OFFSET            0x000010  /* GPIO output value */
#define RP2040_SIO_GPIO_OUT_SET_OFFSET        0x000014  /* GPIO output value set */
#define RP2040_SIO_GPIO_OUT_CLR_OFFSET        0x000018  /* GPIO output value clear */
#define RP2040_SIO_GPIO_OUT_XOR_OFFSET        0x00001c  /* GPIO output value XOR */
#define RP2040_SIO_GPIO_OE_OFFSET             0x000020  /* GPIO output enable */
#define RP2040_SIO_GPIO_OE_SET_OFFSET         0x000024  /* GPIO output enable set */
#define RP2040_SIO_GPIO_OE_CLR_OFFSET         0x000028  /* GPIO output enable clear */
#define RP2040_SIO_GPIO_OE_XOR_OFFSET         0x00002c  /* GPIO output enable XOR */
#define RP2040_SIO_GPIO_HI_OUT_OFFSET         0x000030  /* QSPI output value */
#define RP2040_SIO_GPIO_HI_OUT_SET_OFFSET     0x000034  /* QSPI output value set */
#define RP2040_SIO_GPIO_HI_OUT_CLR_OFFSET     0x000038  /* QSPI output value clear */
#define RP2040_SIO_GPIO_HI_OUT_XOR_OFFSET     0x00003c  /* QSPI output value XOR */
#define RP2040_SIO_GPIO_HI_OE_OFFSET          0x000040  /* QSPI output enable */
#define RP2040_SIO_GPIO_HI_OE_SET_OFFSET      0x000044  /* QSPI output enable set */
#define RP2040_SIO_GPIO_HI_OE_CLR_OFFSET      0x000048  /* QSPI output enable clear */
#define RP2040_SIO_GPIO_HI_OE_XOR_OFFSET      0x00004c  /* QSPI output enable XOR */
#define RP2040_SIO_FIFO_ST_OFFSET             0x000050  /* Status register for inter-core FIFOs (mailboxes). There is one FIFO in the core 0 -> core 1 direction, and one core 1 -> core 0. Both are 32 bits wide and 8 words deep. Core 0 can see the read side of the 1->0 FIFO (RX), and the write side of 0->1 FIFO (TX). Core 1 can see the read side of the 0->1 FIFO (RX), and the write side of 1->0 FIFO (TX). The SIO IRQ for each core is the logical OR of the VLD, WOF and ROE fields of its FIFO_ST register. */
#define RP2040_SIO_FIFO_WR_OFFSET             0x000054  /* Write access to this core's TX FIFO */
#define RP2040_SIO_FIFO_RD_OFFSET             0x000058  /* Read access to this core's RX FIFO */
#define RP2040_SIO_SPINLOCK_ST_OFFSET         0x00005c  /* Spinlock state A bitmap containing the state of all 32 spinlocks (1=locked). Mainly intended for debugging. */
#define RP2040_SIO_DIV_UDIVIDEND_OFFSET       0x000060  /* Divider unsigned dividend Write to the DIVIDEND operand of the divider, i.e. the p in `p / q`. Any operand write starts a new calculation. The results appear in QUOTIENT, REMAINDER. UDIVIDEND/SDIVIDEND are aliases of the same internal register. The U alias starts an unsigned calculation, and the S alias starts a signed calculation. */
#define RP2040_SIO_DIV_UDIVISOR_OFFSET        0x000064  /* Divider unsigned divisor Write to the DIVISOR operand of the divider, i.e. the q in `p / q`. Any operand write starts a new calculation. The results appear in QUOTIENT, REMAINDER. UDIVIDEND/SDIVIDEND are aliases of the same internal register. The U alias starts an unsigned calculation, and the S alias starts a signed calculation. */
#define RP2040_SIO_DIV_SDIVIDEND_OFFSET       0x000068  /* Divider signed dividend The same as UDIVIDEND, but starts a signed calculation, rather than unsigned. */
#define RP2040_SIO_DIV_SDIVISOR_OFFSET        0x00006c  /* Divider signed divisor The same as UDIVISOR, but starts a signed calculation, rather than unsigned. */
#define RP2040_SIO_DIV_QUOTIENT_OFFSET        0x000070  /* Divider result quotient The result of `DIVIDEND / DIVISOR` (division). Contents undefined while CSR_READY is low. For signed calculations, QUOTIENT is negative when the signs of DIVIDEND and DIVISOR differ. This register can be written to directly, for context save/restore purposes. This halts any in-progress calculation and sets the CSR_READY and CSR_DIRTY flags. Reading from QUOTIENT clears the CSR_DIRTY flag, so should read results in the order REMAINDER, QUOTIENT if CSR_DIRTY is used. */
#define RP2040_SIO_DIV_REMAINDER_OFFSET       0x000074  /* Divider result remainder The result of `DIVIDEND % DIVISOR` (modulo). Contents undefined while CSR_READY is low. For signed calculations, REMAINDER is negative only when DIVIDEND is negative. This register can be written to directly, for context save/restore purposes. This halts any in-progress calculation and sets the CSR_READY and CSR_DIRTY flags. */
#define RP2040_SIO_DIV_CSR_OFFSET             0x000078  /* Control and status register for divider. */
#define RP2040_SIO_INTERP0_ACCUM0_OFFSET      0x000080  /* Read/write access to accumulator 0 */
#define RP2040_SIO_INTERP0_ACCUM1_OFFSET      0x000084  /* Read/write access to accumulator 1 */
#define RP2040_SIO_INTERP0_BASE0_OFFSET       0x000088  /* Read/write access to BASE0 register. */
#define RP2040_SIO_INTERP0_BASE1_OFFSET       0x00008c  /* Read/write access to BASE1 register. */
#define RP2040_SIO_INTERP0_BASE2_OFFSET       0x000090  /* Read/write access to BASE2 register. */
#define RP2040_SIO_INTERP0_POP_LANE0_OFFSET   0x000094  /* Read LANE0 result, and simultaneously write lane results to both accumulators (POP). */
#define RP2040_SIO_INTERP0_POP_LANE1_OFFSET   0x000098  /* Read LANE1 result, and simultaneously write lane results to both accumulators (POP). */
#define RP2040_SIO_INTERP0_POP_FULL_OFFSET    0x00009c  /* Read FULL result, and simultaneously write lane results to both accumulators (POP). */
#define RP2040_SIO_INTERP0_PEEK_LANE0_OFFSET  0x0000a0  /* Read LANE0 result, without altering any internal state (PEEK). */
#define RP2040_SIO_INTERP0_PEEK_LANE1_OFFSET  0x0000a4  /* Read LANE1 result, without altering any internal state (PEEK). */
#define RP2040_SIO_INTERP0_PEEK_FULL_OFFSET   0x0000a8  /* Read FULL result, without altering any internal state (PEEK). */
#define RP2040_SIO_INTERP0_CTRL_LANE0_OFFSET  0x0000ac  /* Control register for lane 0 */
#define RP2040_SIO_INTERP0_CTRL_LANE1_OFFSET  0x0000b0  /* Control register for lane 1 */
#define RP2040_SIO_INTERP0_ACCUM0_ADD_OFFSET  0x0000b4  /* Values written here are atomically added to ACCUM0 Reading yields lane 0's raw shift and mask value (BASE0 not added). */
#define RP2040_SIO_INTERP0_ACCUM1_ADD_OFFSET  0x0000b8  /* Values written here are atomically added to ACCUM1 Reading yields lane 1's raw shift and mask value (BASE1 not added). */
#define RP2040_SIO_INTERP0_BASE_1AND0_OFFSET  0x0000bc  /* On write, the lower 16 bits go to BASE0, upper bits to BASE1 simultaneously. Each half is sign-extended to 32 bits if that lane's SIGNED flag is set. */
#define RP2040_SIO_INTERP1_ACCUM0_OFFSET      0x0000c0  /* Read/write access to accumulator 0 */
#define RP2040_SIO_INTERP1_ACCUM1_OFFSET      0x0000c4  /* Read/write access to accumulator 1 */
#define RP2040_SIO_INTERP1_BASE0_OFFSET       0x0000c8  /* Read/write access to BASE0 register. */
#define RP2040_SIO_INTERP1_BASE1_OFFSET       0x0000cc  /* Read/write access to BASE1 register. */
#define RP2040_SIO_INTERP1_BASE2_OFFSET       0x0000d0  /* Read/write access to BASE2 register. */
#define RP2040_SIO_INTERP1_POP_LANE0_OFFSET   0x0000d4  /* Read LANE0 result, and simultaneously write lane results to both accumulators (POP). */
#define RP2040_SIO_INTERP1_POP_LANE1_OFFSET   0x0000d8  /* Read LANE1 result, and simultaneously write lane results to both accumulators (POP). */
#define RP2040_SIO_INTERP1_POP_FULL_OFFSET    0x0000dc  /* Read FULL result, and simultaneously write lane results to both accumulators (POP). */
#define RP2040_SIO_INTERP1_PEEK_LANE0_OFFSET  0x0000e0  /* Read LANE0 result, without altering any internal state (PEEK). */
#define RP2040_SIO_INTERP1_PEEK_LANE1_OFFSET  0x0000e4  /* Read LANE1 result, without altering any internal state (PEEK). */
#define RP2040_SIO_INTERP1_PEEK_FULL_OFFSET   0x0000e8  /* Read FULL result, without altering any internal state (PEEK). */
#define RP2040_SIO_INTERP1_CTRL_LANE0_OFFSET  0x0000ec  /* Control register for lane 0 */
#define RP2040_SIO_INTERP1_CTRL_LANE1_OFFSET  0x0000f0  /* Control register for lane 1 */
#define RP2040_SIO_INTERP1_ACCUM0_ADD_OFFSET  0x0000f4  /* Values written here are atomically added to ACCUM0 Reading yields lane 0's raw shift and mask value (BASE0 not added). */
#define RP2040_SIO_INTERP1_ACCUM1_ADD_OFFSET  0x0000f8  /* Values written here are atomically added to ACCUM1 Reading yields lane 1's raw shift and mask value (BASE1 not added). */
#define RP2040_SIO_INTERP1_BASE_1AND0_OFFSET  0x0000fc  /* On write, the lower 16 bits go to BASE0, upper bits to BASE1 simultaneously. Each half is sign-extended to 32 bits if that lane's SIGNED flag is set. */
#define RP2040_SIO_SPINLOCK_OFFSET(n)         ((n) * 4 + 0x000100)
                                                        /* Reading from a spinlock address will: - Return 0 if lock is already locked - Otherwise return nonzero, and simultaneously claim the lock  Writing (any value) releases the lock. If core 0 and core 1 attempt to claim the same lock simultaneously, core 0 wins. The value returned on success is 0x1 << lock number. */

/* Register definitions *****************************************************/

#define RP2040_SIO_CPUID               (RP2040_SIO_BASE + RP2040_SIO_CPUID_OFFSET)
#define RP2040_SIO_GPIO_IN             (RP2040_SIO_BASE + RP2040_SIO_GPIO_IN_OFFSET)
#define RP2040_SIO_GPIO_HI_IN          (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_IN_OFFSET)
#define RP2040_SIO_GPIO_OUT            (RP2040_SIO_BASE + RP2040_SIO_GPIO_OUT_OFFSET)
#define RP2040_SIO_GPIO_OUT_SET        (RP2040_SIO_BASE + RP2040_SIO_GPIO_OUT_SET_OFFSET)
#define RP2040_SIO_GPIO_OUT_CLR        (RP2040_SIO_BASE + RP2040_SIO_GPIO_OUT_CLR_OFFSET)
#define RP2040_SIO_GPIO_OUT_XOR        (RP2040_SIO_BASE + RP2040_SIO_GPIO_OUT_XOR_OFFSET)
#define RP2040_SIO_GPIO_OE             (RP2040_SIO_BASE + RP2040_SIO_GPIO_OE_OFFSET)
#define RP2040_SIO_GPIO_OE_SET         (RP2040_SIO_BASE + RP2040_SIO_GPIO_OE_SET_OFFSET)
#define RP2040_SIO_GPIO_OE_CLR         (RP2040_SIO_BASE + RP2040_SIO_GPIO_OE_CLR_OFFSET)
#define RP2040_SIO_GPIO_OE_XOR         (RP2040_SIO_BASE + RP2040_SIO_GPIO_OE_XOR_OFFSET)
#define RP2040_SIO_GPIO_HI_OUT         (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OUT_OFFSET)
#define RP2040_SIO_GPIO_HI_OUT_SET     (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OUT_SET_OFFSET)
#define RP2040_SIO_GPIO_HI_OUT_CLR     (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OUT_CLR_OFFSET)
#define RP2040_SIO_GPIO_HI_OUT_XOR     (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OUT_XOR_OFFSET)
#define RP2040_SIO_GPIO_HI_OE          (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OE_OFFSET)
#define RP2040_SIO_GPIO_HI_OE_SET      (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OE_SET_OFFSET)
#define RP2040_SIO_GPIO_HI_OE_CLR      (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OE_CLR_OFFSET)
#define RP2040_SIO_GPIO_HI_OE_XOR      (RP2040_SIO_BASE + RP2040_SIO_GPIO_HI_OE_XOR_OFFSET)
#define RP2040_SIO_FIFO_ST             (RP2040_SIO_BASE + RP2040_SIO_FIFO_ST_OFFSET)
#define RP2040_SIO_FIFO_WR             (RP2040_SIO_BASE + RP2040_SIO_FIFO_WR_OFFSET)
#define RP2040_SIO_FIFO_RD             (RP2040_SIO_BASE + RP2040_SIO_FIFO_RD_OFFSET)
#define RP2040_SIO_SPINLOCK_ST         (RP2040_SIO_BASE + RP2040_SIO_SPINLOCK_ST_OFFSET)
#define RP2040_SIO_DIV_UDIVIDEND       (RP2040_SIO_BASE + RP2040_SIO_DIV_UDIVIDEND_OFFSET)
#define RP2040_SIO_DIV_UDIVISOR        (RP2040_SIO_BASE + RP2040_SIO_DIV_UDIVISOR_OFFSET)
#define RP2040_SIO_DIV_SDIVIDEND       (RP2040_SIO_BASE + RP2040_SIO_DIV_SDIVIDEND_OFFSET)
#define RP2040_SIO_DIV_SDIVISOR        (RP2040_SIO_BASE + RP2040_SIO_DIV_SDIVISOR_OFFSET)
#define RP2040_SIO_DIV_QUOTIENT        (RP2040_SIO_BASE + RP2040_SIO_DIV_QUOTIENT_OFFSET)
#define RP2040_SIO_DIV_REMAINDER       (RP2040_SIO_BASE + RP2040_SIO_DIV_REMAINDER_OFFSET)
#define RP2040_SIO_DIV_CSR             (RP2040_SIO_BASE + RP2040_SIO_DIV_CSR_OFFSET)
#define RP2040_SIO_INTERP0_ACCUM0      (RP2040_SIO_BASE + RP2040_SIO_INTERP0_ACCUM0_OFFSET)
#define RP2040_SIO_INTERP0_ACCUM1      (RP2040_SIO_BASE + RP2040_SIO_INTERP0_ACCUM1_OFFSET)
#define RP2040_SIO_INTERP0_BASE0       (RP2040_SIO_BASE + RP2040_SIO_INTERP0_BASE0_OFFSET)
#define RP2040_SIO_INTERP0_BASE1       (RP2040_SIO_BASE + RP2040_SIO_INTERP0_BASE1_OFFSET)
#define RP2040_SIO_INTERP0_BASE2       (RP2040_SIO_BASE + RP2040_SIO_INTERP0_BASE2_OFFSET)
#define RP2040_SIO_INTERP0_POP_LANE0   (RP2040_SIO_BASE + RP2040_SIO_INTERP0_POP_LANE0_OFFSET)
#define RP2040_SIO_INTERP0_POP_LANE1   (RP2040_SIO_BASE + RP2040_SIO_INTERP0_POP_LANE1_OFFSET)
#define RP2040_SIO_INTERP0_POP_FULL    (RP2040_SIO_BASE + RP2040_SIO_INTERP0_POP_FULL_OFFSET)
#define RP2040_SIO_INTERP0_PEEK_LANE0  (RP2040_SIO_BASE + RP2040_SIO_INTERP0_PEEK_LANE0_OFFSET)
#define RP2040_SIO_INTERP0_PEEK_LANE1  (RP2040_SIO_BASE + RP2040_SIO_INTERP0_PEEK_LANE1_OFFSET)
#define RP2040_SIO_INTERP0_PEEK_FULL   (RP2040_SIO_BASE + RP2040_SIO_INTERP0_PEEK_FULL_OFFSET)
#define RP2040_SIO_INTERP0_CTRL_LANE0  (RP2040_SIO_BASE + RP2040_SIO_INTERP0_CTRL_LANE0_OFFSET)
#define RP2040_SIO_INTERP0_CTRL_LANE1  (RP2040_SIO_BASE + RP2040_SIO_INTERP0_CTRL_LANE1_OFFSET)
#define RP2040_SIO_INTERP0_ACCUM0_ADD  (RP2040_SIO_BASE + RP2040_SIO_INTERP0_ACCUM0_ADD_OFFSET)
#define RP2040_SIO_INTERP0_ACCUM1_ADD  (RP2040_SIO_BASE + RP2040_SIO_INTERP0_ACCUM1_ADD_OFFSET)
#define RP2040_SIO_INTERP0_BASE_1AND0  (RP2040_SIO_BASE + RP2040_SIO_INTERP0_BASE_1AND0_OFFSET)
#define RP2040_SIO_INTERP1_ACCUM0      (RP2040_SIO_BASE + RP2040_SIO_INTERP1_ACCUM0_OFFSET)
#define RP2040_SIO_INTERP1_ACCUM1      (RP2040_SIO_BASE + RP2040_SIO_INTERP1_ACCUM1_OFFSET)
#define RP2040_SIO_INTERP1_BASE0       (RP2040_SIO_BASE + RP2040_SIO_INTERP1_BASE0_OFFSET)
#define RP2040_SIO_INTERP1_BASE1       (RP2040_SIO_BASE + RP2040_SIO_INTERP1_BASE1_OFFSET)
#define RP2040_SIO_INTERP1_BASE2       (RP2040_SIO_BASE + RP2040_SIO_INTERP1_BASE2_OFFSET)
#define RP2040_SIO_INTERP1_POP_LANE0   (RP2040_SIO_BASE + RP2040_SIO_INTERP1_POP_LANE0_OFFSET)
#define RP2040_SIO_INTERP1_POP_LANE1   (RP2040_SIO_BASE + RP2040_SIO_INTERP1_POP_LANE1_OFFSET)
#define RP2040_SIO_INTERP1_POP_FULL    (RP2040_SIO_BASE + RP2040_SIO_INTERP1_POP_FULL_OFFSET)
#define RP2040_SIO_INTERP1_PEEK_LANE0  (RP2040_SIO_BASE + RP2040_SIO_INTERP1_PEEK_LANE0_OFFSET)
#define RP2040_SIO_INTERP1_PEEK_LANE1  (RP2040_SIO_BASE + RP2040_SIO_INTERP1_PEEK_LANE1_OFFSET)
#define RP2040_SIO_INTERP1_PEEK_FULL   (RP2040_SIO_BASE + RP2040_SIO_INTERP1_PEEK_FULL_OFFSET)
#define RP2040_SIO_INTERP1_CTRL_LANE0  (RP2040_SIO_BASE + RP2040_SIO_INTERP1_CTRL_LANE0_OFFSET)
#define RP2040_SIO_INTERP1_CTRL_LANE1  (RP2040_SIO_BASE + RP2040_SIO_INTERP1_CTRL_LANE1_OFFSET)
#define RP2040_SIO_INTERP1_ACCUM0_ADD  (RP2040_SIO_BASE + RP2040_SIO_INTERP1_ACCUM0_ADD_OFFSET)
#define RP2040_SIO_INTERP1_ACCUM1_ADD  (RP2040_SIO_BASE + RP2040_SIO_INTERP1_ACCUM1_ADD_OFFSET)
#define RP2040_SIO_INTERP1_BASE_1AND0  (RP2040_SIO_BASE + RP2040_SIO_INTERP1_BASE_1AND0_OFFSET)
#define RP2040_SIO_SPINLOCK(n)         (RP2040_SIO_BASE + RP2040_SIO_SPINLOCK_OFFSET(n))

/* Register bit definitions *************************************************/

#define RP2040_SIO_GPIO_IN_MASK                        (0x3fffffff)  /* Input value for GPIO0...29 */

#define RP2040_SIO_GPIO_HI_IN_MASK                     (0x3f)        /* Input value on QSPI IO in order 0..5: SCLK, SSn, SD0, SD1, SD2, SD3 */

#define RP2040_SIO_GPIO_OUT_MASK                       (0x3fffffff)  /* Set output level (1/0 -> high/low) for GPIO0...29. Reading back gives the last value written, NOT the input value from the pins. If core 0 and core 1 both write to GPIO_OUT simultaneously (or to a SET/CLR/XOR alias), the result is as though the write from core 0 took place first, and the write from core 1 was then applied to that intermediate result. */

#define RP2040_SIO_GPIO_OUT_SET_MASK                   (0x3fffffff)  /* Perform an atomic bit-set on GPIO_OUT, i.e. `GPIO_OUT |= wdata` */

#define RP2040_SIO_GPIO_OUT_CLR_MASK                   (0x3fffffff)  /* Perform an atomic bit-clear on GPIO_OUT, i.e. `GPIO_OUT &= ~wdata` */

#define RP2040_SIO_GPIO_OUT_XOR_MASK                   (0x3fffffff)  /* Perform an atomic bitwise XOR on GPIO_OUT, i.e. `GPIO_OUT ^= wdata` */

#define RP2040_SIO_GPIO_OE_MASK                        (0x3fffffff)  /* Set output enable (1/0 -> output/input) for GPIO0...29. Reading back gives the last value written. If core 0 and core 1 both write to GPIO_OE simultaneously (or to a SET/CLR/XOR alias), the result is as though the write from core 0 took place first, and the write from core 1 was then applied to that intermediate result. */

#define RP2040_SIO_GPIO_OE_SET_MASK                    (0x3fffffff)  /* Perform an atomic bit-set on GPIO_OE, i.e. `GPIO_OE |= wdata` */

#define RP2040_SIO_GPIO_OE_CLR_MASK                    (0x3fffffff)  /* Perform an atomic bit-clear on GPIO_OE, i.e. `GPIO_OE &= ~wdata` */

#define RP2040_SIO_GPIO_OE_XOR_MASK                    (0x3fffffff)  /* Perform an atomic bitwise XOR on GPIO_OE, i.e. `GPIO_OE ^= wdata` */

#define RP2040_SIO_GPIO_HI_OUT_MASK                    (0x3f)  /* Set output level (1/0 -> high/low) for QSPI IO0...5. Reading back gives the last value written, NOT the input value from the pins. If core 0 and core 1 both write to GPIO_HI_OUT simultaneously (or to a SET/CLR/XOR alias), the result is as though the write from core 0 took place first, and the write from core 1 was then applied to that intermediate result. */

#define RP2040_SIO_GPIO_HI_OUT_SET_MASK                (0x3f)  /* Perform an atomic bit-set on GPIO_HI_OUT, i.e. `GPIO_HI_OUT |= wdata` */

#define RP2040_SIO_GPIO_HI_OUT_CLR_MASK                (0x3f)  /* Perform an atomic bit-clear on GPIO_HI_OUT, i.e. `GPIO_HI_OUT &= ~wdata` */

#define RP2040_SIO_GPIO_HI_OUT_XOR_MASK                (0x3f)  /* Perform an atomic bitwise XOR on GPIO_HI_OUT, i.e. `GPIO_HI_OUT ^= wdata` */

#define RP2040_SIO_GPIO_HI_OE_MASK                     (0x3f)  /* Set output enable (1/0 -> output/input) for QSPI IO0...5. Reading back gives the last value written. If core 0 and core 1 both write to GPIO_HI_OE simultaneously (or to a SET/CLR/XOR alias), the result is as though the write from core 0 took place first, and the write from core 1 was then applied to that intermediate result. */

#define RP2040_SIO_GPIO_HI_OE_SET_MASK                 (0x3f)  /* Perform an atomic bit-set on GPIO_HI_OE, i.e. `GPIO_HI_OE |= wdata` */

#define RP2040_SIO_GPIO_HI_OE_CLR_MASK                 (0x3f)  /* Perform an atomic bit-clear on GPIO_HI_OE, i.e. `GPIO_HI_OE &= ~wdata` */

#define RP2040_SIO_GPIO_HI_OE_XOR_MASK                 (0x3f)  /* Perform an atomic bitwise XOR on GPIO_HI_OE, i.e. `GPIO_HI_OE ^= wdata` */

#define RP2040_SIO_FIFO_ST_ROE                         (1 << 3)  /* Sticky flag indicating the RX FIFO was read when empty. This read was ignored by the FIFO. */
#define RP2040_SIO_FIFO_ST_WOF                         (1 << 2)  /* Sticky flag indicating the TX FIFO was written when full. This write was ignored by the FIFO. */
#define RP2040_SIO_FIFO_ST_RDY                         (1 << 1)  /* Value is 1 if this core's TX FIFO is not full (i.e. if FIFO_WR is ready for more data) */
#define RP2040_SIO_FIFO_ST_VLD                         (1 << 0)  /* Value is 1 if this core's RX FIFO is not empty (i.e. if FIFO_RD is valid) */

#define RP2040_SIO_DIV_CSR_DIRTY                       (1 << 1)  /* Changes to 1 when any register is written, and back to 0 when QUOTIENT is read. Software can use this flag to make save/restore more efficient (skip if not DIRTY). If the flag is used in this way, it's recommended to either read QUOTIENT only, or REMAINDER and then QUOTIENT, to prevent data loss on context switch. */
#define RP2040_SIO_DIV_CSR_READY                       (1 << 0)  /* Reads as 0 when a calculation is in progress, 1 otherwise. Writing an operand (xDIVIDEND, xDIVISOR) will immediately start a new calculation, no matter if one is already in progress. Writing to a result register will immediately terminate any in-progress calculation and set the READY and DIRTY flags. */

#define RP2040_SIO_INTERP0_CTRL_LANE0_OVERF            (1 << 25)  /* Set if either OVERF0 or OVERF1 is set. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_OVERF1           (1 << 24)  /* Indicates if any masked-off MSBs in ACCUM1 are set. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_OVERF0           (1 << 23)  /* Indicates if any masked-off MSBs in ACCUM0 are set. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_BLEND            (1 << 21)  /* Only present on INTERP0 on each core. If BLEND mode is enabled: - LANE1 result is a linear interpolation between BASE0 and BASE1, controlled by the 8 LSBs of lane 1 shift and mask value (a fractional number between 0 and 255/256ths) - LANE0 result does not have BASE0 added (yields only the 8 LSBs of lane 1 shift+mask value) - FULL result does not have lane 1 shift+mask value added (BASE2 + lane 0 shift+mask) LANE1 SIGNED flag controls whether the interpolation is signed or unsigned. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_FORCE_MSB_SHIFT  (19)       /* ORed into bits 29:28 of the lane result presented to the processor on the bus. No effect on the internal 32-bit datapath. Handy for using a lane to generate sequence of pointers into flash or SRAM. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_FORCE_MSB_MASK   (0x03 << RP2040_SIO_INTERP0_CTRL_LANE0_FORCE_MSB_SHIFT)
#define RP2040_SIO_INTERP0_CTRL_LANE0_ADD_RAW          (1 << 18)  /* If 1, mask + shift is bypassed for LANE0 result. This does not affect FULL result. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_CROSS_RESULT     (1 << 17)  /* If 1, feed the opposite lane's result into this lane's accumulator on POP. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_CROSS_INPUT      (1 << 16)  /* If 1, feed the opposite lane's accumulator into this lane's shift + mask hardware. Takes effect even if ADD_RAW is set (the CROSS_INPUT mux is before the shift+mask bypass) */
#define RP2040_SIO_INTERP0_CTRL_LANE0_SIGNED           (1 << 15)  /* If SIGNED is set, the shifted and masked accumulator value is sign-extended to 32 bits before adding to BASE0, and LANE0 PEEK/POP appear extended to 32 bits when read by processor. */
#define RP2040_SIO_INTERP0_CTRL_LANE0_MASK_MSB_SHIFT   (10)       /* The most-significant bit allowed to pass by the mask (inclusive) Setting MSB < LSB may cause chip to turn inside-out */
#define RP2040_SIO_INTERP0_CTRL_LANE0_MASK_MSB_MASK    (0x1f << RP2040_SIO_INTERP0_CTRL_LANE0_MASK_MSB_SHIFT)
#define RP2040_SIO_INTERP0_CTRL_LANE0_MASK_LSB_SHIFT   (5)        /* The least-significant bit allowed to pass by the mask (inclusive) */
#define RP2040_SIO_INTERP0_CTRL_LANE0_MASK_LSB_MASK    (0x1f << RP2040_SIO_INTERP0_CTRL_LANE0_MASK_LSB_SHIFT)
#define RP2040_SIO_INTERP0_CTRL_LANE0_SHIFT_MASK       (0x1f)     /* Logical right-shift applied to accumulator before masking */

#define RP2040_SIO_INTERP0_CTRL_LANE1_FORCE_MSB_SHIFT  (19)       /* ORed into bits 29:28 of the lane result presented to the processor on the bus. No effect on the internal 32-bit datapath. Handy for using a lane to generate sequence of pointers into flash or SRAM. */
#define RP2040_SIO_INTERP0_CTRL_LANE1_FORCE_MSB_MASK   (0x03 << RP2040_SIO_INTERP0_CTRL_LANE1_FORCE_MSB_SHIFT)
#define RP2040_SIO_INTERP0_CTRL_LANE1_ADD_RAW          (1 << 18)  /* If 1, mask + shift is bypassed for LANE1 result. This does not affect FULL result. */
#define RP2040_SIO_INTERP0_CTRL_LANE1_CROSS_RESULT     (1 << 17)  /* If 1, feed the opposite lane's result into this lane's accumulator on POP. */
#define RP2040_SIO_INTERP0_CTRL_LANE1_CROSS_INPUT      (1 << 16)  /* If 1, feed the opposite lane's accumulator into this lane's shift + mask hardware. Takes effect even if ADD_RAW is set (the CROSS_INPUT mux is before the shift+mask bypass) */
#define RP2040_SIO_INTERP0_CTRL_LANE1_SIGNED           (1 << 15)  /* If SIGNED is set, the shifted and masked accumulator value is sign-extended to 32 bits before adding to BASE1, and LANE1 PEEK/POP appear extended to 32 bits when read by processor. */
#define RP2040_SIO_INTERP0_CTRL_LANE1_MASK_MSB_SHIFT   (10)       /* The most-significant bit allowed to pass by the mask (inclusive) Setting MSB < LSB may cause chip to turn inside-out */
#define RP2040_SIO_INTERP0_CTRL_LANE1_MASK_MSB_MASK    (0x1f << RP2040_SIO_INTERP0_CTRL_LANE1_MASK_MSB_SHIFT)
#define RP2040_SIO_INTERP0_CTRL_LANE1_MASK_LSB_SHIFT   (5)        /* The least-significant bit allowed to pass by the mask (inclusive) */
#define RP2040_SIO_INTERP0_CTRL_LANE1_MASK_LSB_MASK    (0x1f << RP2040_SIO_INTERP0_CTRL_LANE1_MASK_LSB_SHIFT)
#define RP2040_SIO_INTERP0_CTRL_LANE1_SHIFT_MASK       (0x1f)     /* Logical right-shift applied to accumulator before masking */

#define RP2040_SIO_INTERP0_ACCUM0_ADD_MASK             (0xffffff)

#define RP2040_SIO_INTERP0_ACCUM1_ADD_MASK             (0xffffff)

#define RP2040_SIO_INTERP1_CTRL_LANE0_OVERF            (1 << 25)  /* Set if either OVERF0 or OVERF1 is set. */
#define RP2040_SIO_INTERP1_CTRL_LANE0_OVERF1           (1 << 24)  /* Indicates if any masked-off MSBs in ACCUM1 are set. */
#define RP2040_SIO_INTERP1_CTRL_LANE0_OVERF0           (1 << 23)  /* Indicates if any masked-off MSBs in ACCUM0 are set. */
#define RP2040_SIO_INTERP1_CTRL_LANE0_CLAMP            (1 << 22)  /* Only present on INTERP1 on each core. If CLAMP mode is enabled: - LANE0 result is shifted and masked ACCUM0, clamped by a lower bound of BASE0 and an upper bound of BASE1. - Signedness of these comparisons is determined by LANE0_CTRL_SIGNED */
#define RP2040_SIO_INTERP1_CTRL_LANE0_FORCE_MSB_SHIFT  (19)       /* ORed into bits 29:28 of the lane result presented to the processor on the bus. No effect on the internal 32-bit datapath. Handy for using a lane to generate sequence of pointers into flash or SRAM. */
#define RP2040_SIO_INTERP1_CTRL_LANE0_FORCE_MSB_MASK   (0x03 << RP2040_SIO_INTERP1_CTRL_LANE0_FORCE_MSB_SHIFT)
#define RP2040_SIO_INTERP1_CTRL_LANE0_ADD_RAW          (1 << 18)  /* If 1, mask + shift is bypassed for LANE0 result. This does not affect FULL result. */
#define RP2040_SIO_INTERP1_CTRL_LANE0_CROSS_RESULT     (1 << 17)  /* If 1, feed the opposite lane's result into this lane's accumulator on POP. */
#define RP2040_SIO_INTERP1_CTRL_LANE0_CROSS_INPUT      (1 << 16)  /* If 1, feed the opposite lane's accumulator into this lane's shift + mask hardware. Takes effect even if ADD_RAW is set (the CROSS_INPUT mux is before the shift+mask bypass) */
#define RP2040_SIO_INTERP1_CTRL_LANE0_SIGNED           (1 << 15)  /* If SIGNED is set, the shifted and masked accumulator value is sign-extended to 32 bits before adding to BASE0, and LANE0 PEEK/POP appear extended to 32 bits when read by processor. */
#define RP2040_SIO_INTERP1_CTRL_LANE0_MASK_MSB_SHIFT   (10)       /* The most-significant bit allowed to pass by the mask (inclusive) Setting MSB < LSB may cause chip to turn inside-out */
#define RP2040_SIO_INTERP1_CTRL_LANE0_MASK_MSB_MASK    (0x1f << RP2040_SIO_INTERP1_CTRL_LANE0_MASK_MSB_SHIFT)
#define RP2040_SIO_INTERP1_CTRL_LANE0_MASK_LSB_SHIFT   (5)        /* The least-significant bit allowed to pass by the mask (inclusive) */
#define RP2040_SIO_INTERP1_CTRL_LANE0_MASK_LSB_MASK    (0x1f << RP2040_SIO_INTERP1_CTRL_LANE0_MASK_LSB_SHIFT)
#define RP2040_SIO_INTERP1_CTRL_LANE0_SHIFT_MASK       (0x1f)     /* Logical right-shift applied to accumulator before masking */

#define RP2040_SIO_INTERP1_CTRL_LANE1_FORCE_MSB_SHIFT  (19)       /* ORed into bits 29:28 of the lane result presented to the processor on the bus. No effect on the internal 32-bit datapath. Handy for using a lane to generate sequence of pointers into flash or SRAM. */
#define RP2040_SIO_INTERP1_CTRL_LANE1_FORCE_MSB_MASK   (0x03 << RP2040_SIO_INTERP1_CTRL_LANE1_FORCE_MSB_SHIFT)
#define RP2040_SIO_INTERP1_CTRL_LANE1_ADD_RAW          (1 << 18)  /* If 1, mask + shift is bypassed for LANE1 result. This does not affect FULL result. */
#define RP2040_SIO_INTERP1_CTRL_LANE1_CROSS_RESULT     (1 << 17)  /* If 1, feed the opposite lane's result into this lane's accumulator on POP. */
#define RP2040_SIO_INTERP1_CTRL_LANE1_CROSS_INPUT      (1 << 16)  /* If 1, feed the opposite lane's accumulator into this lane's shift + mask hardware. Takes effect even if ADD_RAW is set (the CROSS_INPUT mux is before the shift+mask bypass) */
#define RP2040_SIO_INTERP1_CTRL_LANE1_SIGNED           (1 << 15)  /* If SIGNED is set, the shifted and masked accumulator value is sign-extended to 32 bits before adding to BASE1, and LANE1 PEEK/POP appear extended to 32 bits when read by processor. */
#define RP2040_SIO_INTERP1_CTRL_LANE1_MASK_MSB_SHIFT   (10)       /* The most-significant bit allowed to pass by the mask (inclusive) Setting MSB < LSB may cause chip to turn inside-out */
#define RP2040_SIO_INTERP1_CTRL_LANE1_MASK_MSB_MASK    (0x1f << RP2040_SIO_INTERP1_CTRL_LANE1_MASK_MSB_SHIFT)
#define RP2040_SIO_INTERP1_CTRL_LANE1_MASK_LSB_SHIFT   (5)        /* The least-significant bit allowed to pass by the mask (inclusive) */
#define RP2040_SIO_INTERP1_CTRL_LANE1_MASK_LSB_MASK    (0x1f << RP2040_SIO_INTERP1_CTRL_LANE1_MASK_LSB_SHIFT)
#define RP2040_SIO_INTERP1_CTRL_LANE1_SHIFT_MASK       (0x1f)     /* Logical right-shift applied to accumulator before masking */

#define RP2040_SIO_INTERP1_ACCUM0_ADD_MASK             (0xffffff)

#define RP2040_SIO_INTERP1_ACCUM1_ADD_MASK             (0xffffff)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SIO_H */
