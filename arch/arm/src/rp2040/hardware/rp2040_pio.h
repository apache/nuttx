/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_pio.h
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

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PIO_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_PIO_CTRL_OFFSET               0x000000                /* PIO control register */
#define RP2040_PIO_FSTAT_OFFSET              0x000004                /* FIFO status register */
#define RP2040_PIO_FDEBUG_OFFSET             0x000008                /* FIFO debug register */
#define RP2040_PIO_FLEVEL_OFFSET             0x00000c                /* FIFO levels */
#define RP2040_PIO_TXF_OFFSET(m)             (0x000010 + (m) * 4)    /* Direct write access to the TX FIFO for this state machine. Each write pushes one word to the FIFO. */
#define RP2040_PIO_RXF_OFFSET(m)             (0x000020 + (m) * 4)    /* Direct read access to the RX FIFO for this state machine. Each read pops one word from the FIFO. */
#define RP2040_PIO_IRQ_OFFSET                0x000030                /* Interrupt request register. Write 1 to clear */
#define RP2040_PIO_IRQ_FORCE_OFFSET          0x000034                /* Writing a 1 to each of these bits will forcibly assert the corresponding IRQ. Note this is different to the INTF register: writing here affects PIO internal state. INTF just asserts the processor-facing IRQ signal for testing ISRs, and is not visible to the state machines. */
#define RP2040_PIO_INPUT_SYNC_BYPASS_OFFSET  0x000038                /* There is a 2-flipflop synchronizer on each GPIO input, which protects PIO logic from metastabilities. This increases input delay, and for fast synchronous IO (e.g. SPI) these synchronizers may need to be bypassed. Each bit in this register corresponds to one GPIO. 0 -> input is synchronized (default) 1 -> synchronizer is bypassed If in doubt, leave this register as all zeroes. */
#define RP2040_PIO_DBG_PADOUT_OFFSET         0x00003c                /* Read to sample the pad output values PIO is currently driving to the GPIOs. */
#define RP2040_PIO_DBG_PADOE_OFFSET          0x000040                /* Read to sample the pad output enables (direction) PIO is currently driving to the GPIOs. */
#define RP2040_PIO_DBG_CFGINFO_OFFSET        0x000044                /* The PIO hardware has some free parameters that may vary between chip products. These should be provided in the chip datasheet, but are also exposed here. */
#define RP2040_PIO_INSTR_MEM_OFFSET(m)       (0x000048 + (m) * 4)    /* Write-only access to instruction memory location m */
#define RP2040_PIO_SM_CLKDIV_OFFSET(m)       (0x0000c8 + (m) * 0x18) /* Clock divider register for state machine 0 Frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256) */
#define RP2040_PIO_SM_EXECCTRL_OFFSET(m)     (0x0000cc + (m) * 0x18) /* Execution/behavioural settings for state machine 0 */
#define RP2040_PIO_SM_SHIFTCTRL_OFFSET(m)    (0x0000d0 + (m) * 0x18) /* Control behaviour of the input/output shift registers for state machine 0 */
#define RP2040_PIO_SM_ADDR_OFFSET(m)         (0x0000d4 + (m) * 0x18) /* Current instruction address of state machine 0 */
#define RP2040_PIO_SM_INSTR_OFFSET(m)        (0x0000d8 + (m) * 0x18) /* Instruction currently being executed by state machine 0 Write to execute an instruction immediately (including jumps) and then resume execution. */
#define RP2040_PIO_SM_PINCTRL_OFFSET(m)      (0x0000dc + (m) * 0x18) /* State machine pin control */
#define RP2040_PIO_SM0_CLKDIV_OFFSET         0x0000c8                /* Clock divider register for state machine 0 Frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256) */
#define RP2040_PIO_SM0_EXECCTRL_OFFSET       0x0000cc                /* Execution/behavioural settings for state machine 0 */
#define RP2040_PIO_SM0_SHIFTCTRL_OFFSET      0x0000d0                /* Control behaviour of the input/output shift registers for state machine 0 */
#define RP2040_PIO_SM0_ADDR_OFFSET           0x0000d4                /* Current instruction address of state machine 0 */
#define RP2040_PIO_SM0_INSTR_OFFSET          0x0000d8                /* Instruction currently being executed by state machine 0 Write to execute an instruction immediately (including jumps) and then resume execution. */
#define RP2040_PIO_SM0_PINCTRL_OFFSET        0x0000dc                /* State machine pin control */
#define RP2040_PIO_SM1_CLKDIV_OFFSET         0x0000e0                /* Clock divider register for state machine 1 Frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256) */
#define RP2040_PIO_SM1_EXECCTRL_OFFSET       0x0000e4                /* Execution/behavioural settings for state machine 1 */
#define RP2040_PIO_SM1_SHIFTCTRL_OFFSET      0x0000e8                /* Control behaviour of the input/output shift registers for state machine 1 */
#define RP2040_PIO_SM1_ADDR_OFFSET           0x0000ec                /* Current instruction address of state machine 1 */
#define RP2040_PIO_SM1_INSTR_OFFSET          0x0000f0                /* Instruction currently being executed by state machine 1 Write to execute an instruction immediately (including jumps) and then resume execution. */
#define RP2040_PIO_SM1_PINCTRL_OFFSET        0x0000f4                /* State machine pin control */
#define RP2040_PIO_SM2_CLKDIV_OFFSET         0x0000f8                /* Clock divider register for state machine 2 Frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256) */
#define RP2040_PIO_SM2_EXECCTRL_OFFSET       0x0000fc                /* Execution/behavioural settings for state machine 2 */
#define RP2040_PIO_SM2_SHIFTCTRL_OFFSET      0x000100                /* Control behaviour of the input/output shift registers for state machine 2 */
#define RP2040_PIO_SM2_ADDR_OFFSET           0x000104                /* Current instruction address of state machine 2 */
#define RP2040_PIO_SM2_INSTR_OFFSET          0x000108                /* Instruction currently being executed by state machine 2 Write to execute an instruction immediately (including jumps) and then resume execution. */
#define RP2040_PIO_SM2_PINCTRL_OFFSET        0x00010c                /* State machine pin control */
#define RP2040_PIO_SM3_CLKDIV_OFFSET         0x000110                /* Clock divider register for state machine 3 Frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256) */
#define RP2040_PIO_SM3_EXECCTRL_OFFSET       0x000114                /* Execution/behavioural settings for state machine 3 */
#define RP2040_PIO_SM3_SHIFTCTRL_OFFSET      0x000118                /* Control behaviour of the input/output shift registers for state machine 3 */
#define RP2040_PIO_SM3_ADDR_OFFSET           0x00011c                /* Current instruction address of state machine 3 */
#define RP2040_PIO_SM3_INSTR_OFFSET          0x000120                /* Instruction currently being executed by state machine 3 Write to execute an instruction immediately (including jumps) and then resume execution. */
#define RP2040_PIO_SM3_PINCTRL_OFFSET        0x000124                /* State machine pin control */
#define RP2040_PIO_INTR_OFFSET               0x000128                /* Raw Interrupts */
#define RP2040_PIO_IRQ0_INTE_OFFSET          0x00012c                /* Interrupt Enable for irq0 */
#define RP2040_PIO_IRQ0_INTF_OFFSET          0x000130                /* Interrupt Force for irq0 */
#define RP2040_PIO_IRQ0_INTS_OFFSET          0x000134                /* Interrupt status after masking & forcing for irq0 */
#define RP2040_PIO_IRQ1_INTE_OFFSET          0x000138                /* Interrupt Enable for irq1 */
#define RP2040_PIO_IRQ1_INTF_OFFSET          0x00013c                /* Interrupt Force for irq1 */
#define RP2040_PIO_IRQ1_INTS_OFFSET          0x000140                /* Interrupt status after masking & forcing for irq1 */

/* Register definitions *****************************************************/

#define RP2040_PIO_CTRL(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_CTRL_OFFSET)
#define RP2040_PIO_FSTAT(n)              (RP2040_PIO_BASE(n) + RP2040_PIO_FSTAT_OFFSET)
#define RP2040_PIO_FDEBUG(n)             (RP2040_PIO_BASE(n) + RP2040_PIO_FDEBUG_OFFSET)
#define RP2040_PIO_FLEVEL(n)             (RP2040_PIO_BASE(n) + RP2040_PIO_FLEVEL_OFFSET)
#define RP2040_PIO_TXF(n, m)             (RP2040_PIO_BASE(n) + RP2040_PIO_TXF_OFFSET(m))
#define RP2040_PIO_TXF0(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_TXF0_OFFSET)
#define RP2040_PIO_TXF1(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_TXF1_OFFSET)
#define RP2040_PIO_TXF2(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_TXF2_OFFSET)
#define RP2040_PIO_TXF3(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_TXF3_OFFSET)
#define RP2040_PIO_RXF(n, m)             (RP2040_PIO_BASE(n) + RP2040_PIO_RXF_OFFSET(m))
#define RP2040_PIO_RXF0(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_RXF0_OFFSET)
#define RP2040_PIO_RXF1(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_RXF1_OFFSET)
#define RP2040_PIO_RXF2(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_RXF2_OFFSET)
#define RP2040_PIO_RXF3(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_RXF3_OFFSET)
#define RP2040_PIO_IRQ(n)                (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ_OFFSET)
#define RP2040_PIO_IRQ_FORCE(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ_FORCE_OFFSET)
#define RP2040_PIO_INPUT_SYNC_BYPASS(n)  (RP2040_PIO_BASE(n) + RP2040_PIO_INPUT_SYNC_BYPASS_OFFSET)
#define RP2040_PIO_DBG_PADOUT(n)         (RP2040_PIO_BASE(n) + RP2040_PIO_DBG_PADOUT_OFFSET)
#define RP2040_PIO_DBG_PADOE(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_DBG_PADOE_OFFSET)
#define RP2040_PIO_DBG_CFGINFO(n)        (RP2040_PIO_BASE(n) + RP2040_PIO_DBG_CFGINFO_OFFSET)
#define RP2040_PIO_INSTR_MEM(n, m)       (RP2040_PIO_BASE(n) + RP2040_PIO_INSTR_MEM_OFFSET(m))

#define RP2040_PIO_SM_CLKDIV(n, m)       (RP2040_PIO_BASE(n) + RP2040_PIO_SM_CLKDIV_OFFSET(m))
#define RP2040_PIO_SM_EXECCTRL(n, m)     (RP2040_PIO_BASE(n) + RP2040_PIO_SM_EXECCTRL_OFFSET(m))
#define RP2040_PIO_SM_SHIFTCTRL(n, m)    (RP2040_PIO_BASE(n) + RP2040_PIO_SM_SHIFTCTRL_OFFSET(m))
#define RP2040_PIO_SM_ADDR(n, m)         (RP2040_PIO_BASE(n) + RP2040_PIO_SM_ADDR_OFFSET(m))
#define RP2040_PIO_SM_INSTR(n, m)        (RP2040_PIO_BASE(n) + RP2040_PIO_SM_INSTR_OFFSET(m))
#define RP2040_PIO_SM_PINCTRL(n, m)      (RP2040_PIO_BASE(n) + RP2040_PIO_SM_PINCTRL_OFFSET(m))

#define RP2040_PIO_SM0_CLKDIV(n)         (RP2040_PIO_BASE(n) + RP2040_PIO_SM0_CLKDIV_OFFSET)
#define RP2040_PIO_SM0_EXECCTRL(n)       (RP2040_PIO_BASE(n) + RP2040_PIO_SM0_EXECCTRL_OFFSET)
#define RP2040_PIO_SM0_SHIFTCTRL(n)      (RP2040_PIO_BASE(n) + RP2040_PIO_SM0_SHIFTCTRL_OFFSET)
#define RP2040_PIO_SM0_ADDR(n)           (RP2040_PIO_BASE(n) + RP2040_PIO_SM0_ADDR_OFFSET)
#define RP2040_PIO_SM0_INSTR(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_SM0_INSTR_OFFSET)
#define RP2040_PIO_SM0_PINCTRL(n)        (RP2040_PIO_BASE(n) + RP2040_PIO_SM0_PINCTRL_OFFSET)
#define RP2040_PIO_SM1_CLKDIV(n)         (RP2040_PIO_BASE(n) + RP2040_PIO_SM1_CLKDIV_OFFSET)
#define RP2040_PIO_SM1_EXECCTRL(n)       (RP2040_PIO_BASE(n) + RP2040_PIO_SM1_EXECCTRL_OFFSET)
#define RP2040_PIO_SM1_SHIFTCTRL(n)      (RP2040_PIO_BASE(n) + RP2040_PIO_SM1_SHIFTCTRL_OFFSET)
#define RP2040_PIO_SM1_ADDR(n)           (RP2040_PIO_BASE(n) + RP2040_PIO_SM1_ADDR_OFFSET)
#define RP2040_PIO_SM1_INSTR(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_SM1_INSTR_OFFSET)
#define RP2040_PIO_SM1_PINCTRL(n)        (RP2040_PIO_BASE(n) + RP2040_PIO_SM1_PINCTRL_OFFSET)
#define RP2040_PIO_SM2_CLKDIV(n)         (RP2040_PIO_BASE(n) + RP2040_PIO_SM2_CLKDIV_OFFSET)
#define RP2040_PIO_SM2_EXECCTRL(n)       (RP2040_PIO_BASE(n) + RP2040_PIO_SM2_EXECCTRL_OFFSET)
#define RP2040_PIO_SM2_SHIFTCTRL(n)      (RP2040_PIO_BASE(n) + RP2040_PIO_SM2_SHIFTCTRL_OFFSET)
#define RP2040_PIO_SM2_ADDR(n)           (RP2040_PIO_BASE(n) + RP2040_PIO_SM2_ADDR_OFFSET)
#define RP2040_PIO_SM2_INSTR(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_SM2_INSTR_OFFSET)
#define RP2040_PIO_SM2_PINCTRL(n)        (RP2040_PIO_BASE(n) + RP2040_PIO_SM2_PINCTRL_OFFSET)
#define RP2040_PIO_SM3_CLKDIV(n)         (RP2040_PIO_BASE(n) + RP2040_PIO_SM3_CLKDIV_OFFSET)
#define RP2040_PIO_SM3_EXECCTRL(n)       (RP2040_PIO_BASE(n) + RP2040_PIO_SM3_EXECCTRL_OFFSET)
#define RP2040_PIO_SM3_SHIFTCTRL(n)      (RP2040_PIO_BASE(n) + RP2040_PIO_SM3_SHIFTCTRL_OFFSET)
#define RP2040_PIO_SM3_ADDR(n)           (RP2040_PIO_BASE(n) + RP2040_PIO_SM3_ADDR_OFFSET)
#define RP2040_PIO_SM3_INSTR(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_SM3_INSTR_OFFSET)
#define RP2040_PIO_SM3_PINCTRL(n)        (RP2040_PIO_BASE(n) + RP2040_PIO_SM3_PINCTRL_OFFSET)
#define RP2040_PIO_INTR(n)               (RP2040_PIO_BASE(n) + RP2040_PIO_INTR_OFFSET)
#define RP2040_PIO_IRQ0_INTE(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ0_INTE_OFFSET)
#define RP2040_PIO_IRQ0_INTF(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ0_INTF_OFFSET)
#define RP2040_PIO_IRQ0_INTS(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ0_INTS_OFFSET)
#define RP2040_PIO_IRQ1_INTE(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ1_INTE_OFFSET)
#define RP2040_PIO_IRQ1_INTF(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ1_INTF_OFFSET)
#define RP2040_PIO_IRQ1_INTS(n)          (RP2040_PIO_BASE(n) + RP2040_PIO_IRQ1_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_PIO_CTRL_CLKDIV_RESTART_SHIFT        (8)     /* Force clock dividers to restart their count and clear fractional accumulators. Restart multiple dividers to synchronise them. */
#define RP2040_PIO_CTRL_CLKDIV_RESTART_MASK         (0x0f << RP2040_PIO_CTRL_CLKDIV_RESTART_SHIFT)
#define RP2040_PIO_CTRL_SM_RESTART_SHIFT            (4)     /* Clear internal SM state which is otherwise difficult to access (e.g. shift counters). Self-clearing. */
#define RP2040_PIO_CTRL_SM_RESTART_MASK             (0x0f << RP2040_PIO_CTRL_SM_RESTART_SHIFT)
#define RP2040_PIO_CTRL_SM_ENABLE_MASK              (0x0f)  /* Enable state machine */

#define RP2040_PIO_FSTAT_TXEMPTY_SHIFT              (24)    /* State machine TX FIFO is empty */
#define RP2040_PIO_FSTAT_TXEMPTY_MASK               (0x0f << RP2040_PIO_FSTAT_TXEMPTY_SHIFT)
#define RP2040_PIO_FSTAT_TXFULL_SHIFT               (16)    /* State machine TX FIFO is full */
#define RP2040_PIO_FSTAT_TXFULL_MASK                (0x0f << RP2040_PIO_FSTAT_TXFULL_SHIFT)
#define RP2040_PIO_FSTAT_RXEMPTY_SHIFT              (8)     /* State machine RX FIFO is empty */
#define RP2040_PIO_FSTAT_RXEMPTY_MASK               (0x0f << RP2040_PIO_FSTAT_RXEMPTY_SHIFT)
#define RP2040_PIO_FSTAT_RXFULL_SHIFT               (0)     /* State machine RX FIFO is full */
#define RP2040_PIO_FSTAT_RXFULL_MASK                (0x0f)  /* State machine RX FIFO is full */

#define RP2040_PIO_FDEBUG_TXSTALL_SHIFT             (24)    /* State machine has stalled on empty TX FIFO. Write 1 to clear. */
#define RP2040_PIO_FDEBUG_TXSTALL_MASK              (0x0f << RP2040_PIO_FDEBUG_TXSTALL_SHIFT)
#define RP2040_PIO_FDEBUG_TXOVER_SHIFT              (16)    /* TX FIFO overflow has occurred. Write 1 to clear. */
#define RP2040_PIO_FDEBUG_TXOVER_MASK               (0x0f << RP2040_PIO_FDEBUG_TXOVER_SHIFT)
#define RP2040_PIO_FDEBUG_RXUNDER_SHIFT             (8)     /* RX FIFO underflow has occurred. Write 1 to clear. */
#define RP2040_PIO_FDEBUG_RXUNDER_MASK              (0x0f << RP2040_PIO_FDEBUG_RXUNDER_SHIFT)
#define RP2040_PIO_FDEBUG_RXSTALL_SHIFT             (0)     /* State machine has stalled on full RX FIFO. Write 1 to clear. */
#define RP2040_PIO_FDEBUG_RXSTALL_MASK              (0x0f)  /* State machine has stalled on full RX FIFO. Write 1 to clear. */

#define RP2040_PIO_FLEVEL_RX3_SHIFT                 (28)
#define RP2040_PIO_FLEVEL_RX3_MASK                  (0x0f << RP2040_PIO_FLEVEL_RX3_SHIFT)
#define RP2040_PIO_FLEVEL_TX3_SHIFT                 (24)
#define RP2040_PIO_FLEVEL_TX3_MASK                  (0x0f << RP2040_PIO_FLEVEL_TX3_SHIFT)
#define RP2040_PIO_FLEVEL_RX2_SHIFT                 (20)
#define RP2040_PIO_FLEVEL_RX2_MASK                  (0x0f << RP2040_PIO_FLEVEL_RX2_SHIFT)
#define RP2040_PIO_FLEVEL_TX2_SHIFT                 (16)
#define RP2040_PIO_FLEVEL_TX2_MASK                  (0x0f << RP2040_PIO_FLEVEL_TX2_SHIFT)
#define RP2040_PIO_FLEVEL_RX1_SHIFT                 (12)
#define RP2040_PIO_FLEVEL_RX1_MASK                  (0x0f << RP2040_PIO_FLEVEL_RX1_SHIFT)
#define RP2040_PIO_FLEVEL_TX1_SHIFT                 (8)
#define RP2040_PIO_FLEVEL_TX1_MASK                  (0x0f << RP2040_PIO_FLEVEL_TX1_SHIFT)
#define RP2040_PIO_FLEVEL_RX0_SHIFT                 (4)
#define RP2040_PIO_FLEVEL_RX0_MASK                  (0x0f << RP2040_PIO_FLEVEL_RX0_SHIFT)
#define RP2040_PIO_FLEVEL_TX0_SHIFT                 (0)
#define RP2040_PIO_FLEVEL_TX0_MASK                  (0x0f)

#define RP2040_PIO_FLEVEL_TX_MASK(n)                (0x0f << 8*n)
#define RP2040_PIO_FLEVEL_RX_MASK(n)                (0xf0 << 8*n)

#define RP2040_PIO_IRQ_MASK                         (0xff)

#define RP2040_PIO_IRQ_FORCE_MASK                   (0xff)

#define RP2040_PIO_DBG_CFGINFO_IMEM_SIZE_SHIFT      (16)    /* The size of the instruction memory, measured in units of one instruction */
#define RP2040_PIO_DBG_CFGINFO_IMEM_SIZE_MASK       (0x3f << RP2040_PIO_DBG_CFGINFO_IMEM_SIZE_SHIFT)
#define RP2040_PIO_DBG_CFGINFO_SM_COUNT_SHIFT       (8)     /* The number of state machines this PIO instance is equipped with. */
#define RP2040_PIO_DBG_CFGINFO_SM_COUNT_MASK        (0x0f << RP2040_PIO_DBG_CFGINFO_SM_COUNT_SHIFT)
#define RP2040_PIO_DBG_CFGINFO_FIFO_DEPTH_MASK      (0x3f)  /* The depth of the state machine TX/RX FIFOs, measured in words. Joining fifos via SHIFTCTRL_FJOIN gives one FIFO with double this depth. */

#define RP2040_PIO_INSTR_MEM_MASK                   (0xffff)

#define RP2040_PIO_SM_CLKDIV_INT_SHIFT             (16)  /* Effective frequency is sysclk/int. Value of 0 is interpreted as max possible value */
#define RP2040_PIO_SM_CLKDIV_INT_MASK              (0xffff << RP2040_PIO_SM_CLKDIV_INT_SHIFT)
#define RP2040_PIO_SM_CLKDIV_FRAC_SHIFT            (8)   /* Fractional part of clock divider */
#define RP2040_PIO_SM_CLKDIV_FRAC_MASK             (0xff << RP2040_PIO_SM_CLKDIV_FRAC_SHIFT)

#define RP2040_PIO_SM_EXECCTRL_EXEC_STALLED        (1 << 31)  /* An instruction written to SMx_INSTR is stalled, and latched by the state machine. Will clear once the instruction completes. */
#define RP2040_PIO_SM_EXECCTRL_SIDE_EN             (1 << 30)  /* If 1, the delay MSB is used as side-set enable, rather than a side-set data bit. This allows instructions to perform side-set optionally, rather than on every instruction. */
#define RP2040_PIO_SM_EXECCTRL_SIDE_PINDIR         (1 << 29)  /* Side-set data is asserted to pin OEs instead of pin values */
#define RP2040_PIO_SM_EXECCTRL_JMP_PIN_SHIFT       (24)       /* The GPIO number to use as condition for JMP PIN. Unaffected by input mapping. */
#define RP2040_PIO_SM_EXECCTRL_JMP_PIN_MASK        (0x1f << RP2040_PIO_SM_EXECCTRL_JMP_PIN_SHIFT)
#define RP2040_PIO_SM_EXECCTRL_OUT_EN_SEL_SHIFT    (19)       /* Which data bit to use for inline OUT enable */
#define RP2040_PIO_SM_EXECCTRL_OUT_EN_SEL_MASK     (0x1f << RP2040_PIO_SM_EXECCTRL_OUT_EN_SEL_SHIFT)
#define RP2040_PIO_SM_EXECCTRL_INLINE_OUT_EN       (1 << 18)  /* If 1, use a bit of OUT data as an auxiliary write enable When used in conjunction with OUT_STICKY, writes with an enable of 0 will deassert the latest pin write. This can create useful masking/override behaviour due to the priority ordering of state machine pin writes (SM0 < SM1 < ...) */
#define RP2040_PIO_SM_EXECCTRL_OUT_STICKY          (1 << 17)  /* Continuously assert the most recent OUT/SET to the pins */
#define RP2040_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT      (12)       /* After reaching this address, execution is wrapped to wrap_bottom. If the instruction is a jump, and the jump condition is true, the jump takes priority. */
#define RP2040_PIO_SM_EXECCTRL_WRAP_TOP_MASK       (0x1f << RP2040_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT)
#define RP2040_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT   (7)        /* After reaching wrap_top, execution is wrapped to this address. */
#define RP2040_PIO_SM_EXECCTRL_WRAP_BOTTOM_MASK    (0x1f << RP2040_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT)
#define RP2040_PIO_SM_EXECCTRL_STATUS_SEL          (1 << 4)   /* All-ones if RX FIFO level < N, otherwise all-zeroes */
#define RP2040_PIO_SM_EXECCTRL_STATUS_N_MASK       (0x0f)     /* Comparison level for the MOV x, STATUS instruction */

#define RP2040_PIO_SM_SHIFTCTRL_FJOIN_RX           (1 << 31)  /* When 1, RX FIFO steals the TX FIFO's storage, and becomes twice as deep. TX FIFO is disabled as a result (always reads as both full and empty). FIFOs are flushed when this bit is changed. */
#define RP2040_PIO_SM_SHIFTCTRL_FJOIN_TX           (1 << 30)  /* When 1, TX FIFO steals the RX FIFO's storage, and becomes twice as deep. RX FIFO is disabled as a result (always reads as both full and empty). FIFOs are flushed when this bit is changed. */
#define RP2040_PIO_SM_SHIFTCTRL_PULL_THRESH_SHIFT  (25)       /* Number of bits shifted out of TXSR before autopull or conditional pull. Write 0 for value of 32. */
#define RP2040_PIO_SM_SHIFTCTRL_PULL_THRESH_MASK   (0x1f << RP2040_PIO_SM_SHIFTCTRL_PULL_THRESH_SHIFT)
#define RP2040_PIO_SM_SHIFTCTRL_PUSH_THRESH_SHIFT  (20)       /* Number of bits shifted into RXSR before autopush or conditional push. Write 0 for value of 32. */
#define RP2040_PIO_SM_SHIFTCTRL_PUSH_THRESH_MASK   (0x1f << RP2040_PIO_SM_SHIFTCTRL_PUSH_THRESH_SHIFT)
#define RP2040_PIO_SM_SHIFTCTRL_OUT_SHIFTDIR       (1 << 19)  /* 1 = shift out of output shift register to right. 0 = to left. */
#define RP2040_PIO_SM_SHIFTCTRL_IN_SHIFTDIR        (1 << 18)  /* 1 = shift input shift register to right (data enters from left). 0 = to left. */
#define RP2040_PIO_SM_SHIFTCTRL_AUTOPULL           (1 << 17)  /* Pull automatically when the output shift register is emptied */
#define RP2040_PIO_SM_SHIFTCTRL_AUTOPUSH           (1 << 16)  /* Push automatically when the input shift register is filled */

#define RP2040_PIO_SM_ADDR_MASK                    (0x1f)

#define RP2040_PIO_SM_INSTR_MASK                   (0xffff)

#define RP2040_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT  (29)    /* The number of delay bits co-opted for side-set. Inclusive of the enable bit, if present. */
#define RP2040_PIO_SM_PINCTRL_SIDESET_COUNT_MASK   (0x07 << RP2040_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT)
#define RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT      (26)    /* The number of pins asserted by a SET. Max of 5 */
#define RP2040_PIO_SM_PINCTRL_SET_COUNT_MASK       (0x07 << RP2040_PIO_SM_PINCTRL_SET_COUNT_SHIFT)
#define RP2040_PIO_SM_PINCTRL_OUT_COUNT_SHIFT      (20)    /* The number of pins asserted by an OUT. Value of 0 -> 32 pins */
#define RP2040_PIO_SM_PINCTRL_OUT_COUNT_MASK       (0x3f << RP2040_PIO_SM_PINCTRL_OUT_COUNT_SHIFT)
#define RP2040_PIO_SM_PINCTRL_IN_BASE_SHIFT        (15)    /* The virtual pin corresponding to IN bit 0 */
#define RP2040_PIO_SM_PINCTRL_IN_BASE_MASK         (0x1f << RP2040_PIO_SM_PINCTRL_IN_BASE_SHIFT)
#define RP2040_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT   (10)    /* The virtual pin corresponding to delay field bit 0 */
#define RP2040_PIO_SM_PINCTRL_SIDESET_BASE_MASK    (0x1f << RP2040_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT)
#define RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT       (5)     /* The virtual pin corresponding to SET bit 0 */
#define RP2040_PIO_SM_PINCTRL_SET_BASE_MASK        (0x1f << RP2040_PIO_SM_PINCTRL_SET_BASE_SHIFT)
#define RP2040_PIO_SM_PINCTRL_OUT_BASE_SHIFT       (0)     /* The virtual pin corresponding to OUT bit 0 */
#define RP2040_PIO_SM_PINCTRL_OUT_BASE_MASK        (0x1f)  /* The virtual pin corresponding to OUT bit 0 */

#define RP2040_PIO_INTR_SM3                         (1 << 11)
#define RP2040_PIO_INTR_SM2                         (1 << 10)
#define RP2040_PIO_INTR_SM1                         (1 << 9)
#define RP2040_PIO_INTR_SM0                         (1 << 8)
#define RP2040_PIO_INTR_SM3_TXNFULL                 (1 << 7)
#define RP2040_PIO_INTR_SM2_TXNFULL                 (1 << 6)
#define RP2040_PIO_INTR_SM1_TXNFULL                 (1 << 5)
#define RP2040_PIO_INTR_SM0_TXNFULL                 (1 << 4)
#define RP2040_PIO_INTR_SM3_RXNEMPTY                (1 << 3)
#define RP2040_PIO_INTR_SM2_RXNEMPTY                (1 << 2)
#define RP2040_PIO_INTR_SM1_RXNEMPTY                (1 << 1)
#define RP2040_PIO_INTR_SM0_RXNEMPTY                (1 << 0)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_PIO_H */
