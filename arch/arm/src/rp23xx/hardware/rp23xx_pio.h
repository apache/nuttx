/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_pio.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PIO_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_PIO_CTRL_OFFSET               0x000000                /* PIO control register */
#define RP23XX_PIO_FSTAT_OFFSET              0x000004                /* FIFO status register */
#define RP23XX_PIO_FDEBUG_OFFSET             0x000008                /* FIFO debug register */
#define RP23XX_PIO_FLEVEL_OFFSET             0x00000c                /* FIFO levels */
#define RP23XX_PIO_TXF_OFFSET(m)             (0x000010 + (m) * 4)    /* Direct write access to the TX FIFO for this state machine. Each write pushes one word to the FIFO. */
#define RP23XX_PIO_RXF_OFFSET(m)             (0x000020 + (m) * 4)    /* Direct read access to the RX FIFO for this state machine. Each read pops one word from the FIFO. */
#define RP23XX_PIO_IRQ_OFFSET                0x000030                /* Interrupt request register. Write 1 to clear */
#define RP23XX_PIO_IRQ_FORCE_OFFSET          0x000034                /* Writing a 1 to each of these bits will forcibly assert the corresponding IRQ. Note this is different to the INTF register: writing here affects PIO internal state. INTF just asserts the processor-facing IRQ signal for testing ISRs, and is not visible to the state machines. */
#define RP23XX_PIO_INPUT_SYNC_BYPASS_OFFSET  0x000038                /* There is a 2-flipflop synchronizer on each GPIO input, which protects PIO logic from metastabilities. This increases input delay, and for fast synchronous IO (e.g. SPI) these synchronizers may need to be bypassed. Each bit in this register corresponds to one GPIO. 0 -> input is synchronized (default) 1 -> synchronizer is bypassed If in doubt, leave this register as all zeroes. */
#define RP23XX_PIO_DBG_PADOUT_OFFSET         0x00003c                /* Read to sample the pad output values PIO is currently driving to the GPIOs. */
#define RP23XX_PIO_DBG_PADOE_OFFSET          0x000040                /* Read to sample the pad output enables (direction) PIO is currently driving to the GPIOs. */
#define RP23XX_PIO_DBG_CFGINFO_OFFSET        0x000044                /* The PIO hardware has some free parameters that may vary between chip products. These should be provided in the chip datasheet, but are also exposed here. */
#define RP23XX_PIO_INSTR_MEM_OFFSET(m)       (0x000048 + (m) * 4)    /* Write-only access to instruction memory location m */
#define RP23XX_PIO_SM_CLKDIV_OFFSET(m)       (0x0000c8 + (m) * 0x18) /* Clock divider register for state machine 0 Frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256) */
#define RP23XX_PIO_SM_EXECCTRL_OFFSET(m)     (0x0000cc + (m) * 0x18) /* Execution/behavioural settings for state machine 0 */
#define RP23XX_PIO_SM_SHIFTCTRL_OFFSET(m)    (0x0000d0 + (m) * 0x18) /* Control behaviour of the input/output shift registers for state machine 0 */
#define RP23XX_PIO_SM_ADDR_OFFSET(m)         (0x0000d4 + (m) * 0x18) /* Current instruction address of state machine 0 */
#define RP23XX_PIO_SM_INSTR_OFFSET(m)        (0x0000d8 + (m) * 0x18) /* Instruction currently being executed by state machine 0 Write to execute an instruction immediately (including jumps) and then resume execution. */
#define RP23XX_PIO_SM_PINCTRL_OFFSET(m)      (0x0000dc + (m) * 0x18) /* State machine pin control */
#define RP23XX_PIO_RXF_PUTGET0_OFFSET(m)     (0x000128 + (m) * 0x10) /* Direct read/write access to entry 0 of SM’s RX FIFO, if SHIFTCTRL_FJOIN_RX_PUT xor SHIFTCTRL_FJOIN_RX_GET is set. */
#define RP23XX_PIO_RXF_PUTGET1_OFFSET(m)     (0x00012c + (m) * 0x10) /* Direct read/write access to entry 0 of SM’s RX FIFO, if SHIFTCTRL_FJOIN_RX_PUT xor SHIFTCTRL_FJOIN_RX_GET is set. */
#define RP23XX_PIO_RXF_PUTGET2_OFFSET(m)     (0x000130 + (m) * 0x10) /* Direct read/write access to entry 0 of SM’s RX FIFO, if SHIFTCTRL_FJOIN_RX_PUT xor SHIFTCTRL_FJOIN_RX_GET is set. */
#define RP23XX_PIO_RXF_PUTGET3_OFFSET(m)     (0x000134 + (m) * 0x10) /* Direct read/write access to entry 0 of SM’s RX FIFO, if SHIFTCTRL_FJOIN_RX_PUT xor SHIFTCTRL_FJOIN_RX_GET is set. */
#define RP23XX_PIO_GPIOBASE_OFFSET           0x000168                /* Relocate GPIO 0 (from PIO’s point of view) in the system GPIO numbering, to access more than 32 GPIOs from PIO. Only the values 0 and 16 are supported (only bit 4 is writable) */
#define RP23XX_PIO_INTR_OFFSET               0x00016c                /* Raw Interrupts */
#define RP23XX_PIO_IRQ0_INTE_OFFSET          0x000170                /* Interrupt Enable for irq0 */
#define RP23XX_PIO_IRQ0_INTF_OFFSET          0x000174                /* Interrupt Force for irq0 */
#define RP23XX_PIO_IRQ0_INTS_OFFSET          0x000178                /* Interrupt status after masking & forcing for irq0 */
#define RP23XX_PIO_IRQ1_INTE_OFFSET          0x00017c                /* Interrupt Enable for irq1 */
#define RP23XX_PIO_IRQ1_INTF_OFFSET          0x000180                /* Interrupt Force for irq1 */
#define RP23XX_PIO_IRQ1_INTS_OFFSET          0x000184                /* Interrupt status after masking & forcing for irq1 */

/* Register definitions *****************************************************/

#define RP23XX_PIO_CTRL(n)               (RP23XX_PIO_BASE(n) + RP23XX_PIO_CTRL_OFFSET)
#define RP23XX_PIO_FSTAT(n)              (RP23XX_PIO_BASE(n) + RP23XX_PIO_FSTAT_OFFSET)
#define RP23XX_PIO_FDEBUG(n)             (RP23XX_PIO_BASE(n) + RP23XX_PIO_FDEBUG_OFFSET)
#define RP23XX_PIO_FLEVEL(n)             (RP23XX_PIO_BASE(n) + RP23XX_PIO_FLEVEL_OFFSET)
#define RP23XX_PIO_TXF(n, m)             (RP23XX_PIO_BASE(n) + RP23XX_PIO_TXF_OFFSET(m))
#define RP23XX_PIO_RXF(n, m)             (RP23XX_PIO_BASE(n) + RP23XX_PIO_RXF_OFFSET(m))
#define RP23XX_PIO_IRQ(n)                (RP23XX_PIO_BASE(n) + RP23XX_PIO_IRQ_OFFSET)
#define RP23XX_PIO_IRQ_FORCE(n)          (RP23XX_PIO_BASE(n) + RP23XX_PIO_IRQ_FORCE_OFFSET)
#define RP23XX_PIO_INPUT_SYNC_BYPASS(n)  (RP23XX_PIO_BASE(n) + RP23XX_PIO_INPUT_SYNC_BYPASS_OFFSET)
#define RP23XX_PIO_DBG_PADOUT(n)         (RP23XX_PIO_BASE(n) + RP23XX_PIO_DBG_PADOUT_OFFSET)
#define RP23XX_PIO_DBG_PADOE(n)          (RP23XX_PIO_BASE(n) + RP23XX_PIO_DBG_PADOE_OFFSET)
#define RP23XX_PIO_DBG_CFGINFO(n)        (RP23XX_PIO_BASE(n) + RP23XX_PIO_DBG_CFGINFO_OFFSET)
#define RP23XX_PIO_INSTR_MEM(n, m)       (RP23XX_PIO_BASE(n) + RP23XX_PIO_INSTR_MEM_OFFSET(m))

#define RP23XX_PIO_SM_CLKDIV(n, m)       (RP23XX_PIO_BASE(n) + RP23XX_PIO_SM_CLKDIV_OFFSET(m))
#define RP23XX_PIO_SM_EXECCTRL(n, m)     (RP23XX_PIO_BASE(n) + RP23XX_PIO_SM_EXECCTRL_OFFSET(m))
#define RP23XX_PIO_SM_SHIFTCTRL(n, m)    (RP23XX_PIO_BASE(n) + RP23XX_PIO_SM_SHIFTCTRL_OFFSET(m))
#define RP23XX_PIO_SM_ADDR(n, m)         (RP23XX_PIO_BASE(n) + RP23XX_PIO_SM_ADDR_OFFSET(m))
#define RP23XX_PIO_SM_INSTR(n, m)        (RP23XX_PIO_BASE(n) + RP23XX_PIO_SM_INSTR_OFFSET(m))
#define RP23XX_PIO_SM_PINCTRL(n, m)      (RP23XX_PIO_BASE(n) + RP23XX_PIO_SM_PINCTRL_OFFSET(m))
#define RP23XX_PIO_RXF_PUTGET0(n, m)     (RP23XX_PIO_BASE(n) + RP23XX_PIO_RXF_PUTGET0_OFFSET(m))
#define RP23XX_PIO_RXF_PUTGET1(n, m)     (RP23XX_PIO_BASE(n) + RP23XX_PIO_RXF_PUTGET1_OFFSET(m))
#define RP23XX_PIO_RXF_PUTGET2(n, m)     (RP23XX_PIO_BASE(n) + RP23XX_PIO_RXF_PUTGET2_OFFSET(m))
#define RP23XX_PIO_RXF_PUTGET3(n, m)     (RP23XX_PIO_BASE(n) + RP23XX_PIO_RXF_PUTGET3_OFFSET(m))
#define RP23XX_PIO_GPIOBASE(n)           (RP23XX_PIO_BASE(n) + RP23XX_PIO_GPIOBASE_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_PIO_CTRL_NEXTPREV_CLKDIV_RESTART     (1 << 26)  /* Write 1 to restart the clock dividers of state machines in neighbouring PIO blocks, as specified by NEXT_PIO_MASK and PREV_PIO_MASK in the same write. This is equivalent to writing 1 to the corresponding CLKDIV_RESTART bits in those PIOs' CTRL registers */
#define RP23XX_PIO_CTRL_NEXTPREV_SM_DISABLE         (1 << 25)  /* Write 1 to disable state machines in neighbouring PIO blocks, as specified by NEXT_PIO_MASK and PREV_PIO_MASK in the same write. This is equivalent to clearing the corresponding SM_ENABLE bits in those PIOs' CTRL registers */
#define RP23XX_PIO_CTRL_NEXTPREV_SM_ENABLE          (1 << 24)  /* Write 1 to enable state machines in neighbouring PIO blocks, as specified by NEXT_PIO_MASK and PREV_PIO_MASK in the same write. This is equivalent to clearing the corresponding SM_ENABLE bits in those PIOs' CTRL registers. If both OTHERS_SM_ENABLE and OTHERS_SM_DISABLE are set, the disable takes precedence */
#define RP23XX_PIO_NEXT_PIO_MASK_SHIFT              (20)       /* A mask of state machines in the neighbouring highernumbered PIO block in the system (or PIO block 0 if this is the highestnumbered PIO block) to which to apply the operations specified by NEXTPREV_CLKDIV_RESTART, NEXTPREV_SM_ENABLE, and NEXTPREV_SM_DISABLE in the same write */
#define RP23XX_PIO_NEXT_PIO_MASK_MASK               (0xf)
#define RP23XX_PIO_PREV_PIO_MASK_SHIFT              (16)       /* A mask of state machines in the neighbouring lowernumbered PIO block in the system (or the highest-numbered PIO block if this is PIO block 0) to which to apply the operations specified by OP_CLKDIV_RESTART, OP_ENABLE, OP_DISABLE in the same write */
#define RP23XX_PIO_PREV_PIO_MASK_MASK               (0xf)

#define RP23XX_PIO_CTRL_CLKDIV_RESTART_SHIFT        (8)     /* Force clock dividers to restart their count and clear fractional accumulators. Restart multiple dividers to synchronise them. */
#define RP23XX_PIO_CTRL_CLKDIV_RESTART_MASK         (0x0f << RP23XX_PIO_CTRL_CLKDIV_RESTART_SHIFT)
#define RP23XX_PIO_CTRL_SM_RESTART_SHIFT            (4)     /* Clear internal SM state which is otherwise difficult to access (e.g. shift counters). Self-clearing. */
#define RP23XX_PIO_CTRL_SM_RESTART_MASK             (0x0f << RP23XX_PIO_CTRL_SM_RESTART_SHIFT)
#define RP23XX_PIO_CTRL_SM_ENABLE_MASK              (0x0f)  /* Enable state machine */

#define RP23XX_PIO_FSTAT_TXEMPTY_SHIFT              (24)    /* State machine TX FIFO is empty */
#define RP23XX_PIO_FSTAT_TXEMPTY_MASK               (0x0f << RP23XX_PIO_FSTAT_TXEMPTY_SHIFT)
#define RP23XX_PIO_FSTAT_TXFULL_SHIFT               (16)    /* State machine TX FIFO is full */
#define RP23XX_PIO_FSTAT_TXFULL_MASK                (0x0f << RP23XX_PIO_FSTAT_TXFULL_SHIFT)
#define RP23XX_PIO_FSTAT_RXEMPTY_SHIFT              (8)     /* State machine RX FIFO is empty */
#define RP23XX_PIO_FSTAT_RXEMPTY_MASK               (0x0f << RP23XX_PIO_FSTAT_RXEMPTY_SHIFT)
#define RP23XX_PIO_FSTAT_RXFULL_SHIFT               (0)     /* State machine RX FIFO is full */
#define RP23XX_PIO_FSTAT_RXFULL_MASK                (0x0f)  /* State machine RX FIFO is full */

#define RP23XX_PIO_FDEBUG_TXSTALL_SHIFT             (24)    /* State machine has stalled on empty TX FIFO. Write 1 to clear. */
#define RP23XX_PIO_FDEBUG_TXSTALL_MASK              (0x0f << RP23XX_PIO_FDEBUG_TXSTALL_SHIFT)
#define RP23XX_PIO_FDEBUG_TXOVER_SHIFT              (16)    /* TX FIFO overflow has occurred. Write 1 to clear. */
#define RP23XX_PIO_FDEBUG_TXOVER_MASK               (0x0f << RP23XX_PIO_FDEBUG_TXOVER_SHIFT)
#define RP23XX_PIO_FDEBUG_RXUNDER_SHIFT             (8)     /* RX FIFO underflow has occurred. Write 1 to clear. */
#define RP23XX_PIO_FDEBUG_RXUNDER_MASK              (0x0f << RP23XX_PIO_FDEBUG_RXUNDER_SHIFT)
#define RP23XX_PIO_FDEBUG_RXSTALL_SHIFT             (0)     /* State machine has stalled on full RX FIFO. Write 1 to clear. */
#define RP23XX_PIO_FDEBUG_RXSTALL_MASK              (0x0f)  /* State machine has stalled on full RX FIFO. Write 1 to clear. */

#define RP23XX_PIO_FLEVEL_RX3_SHIFT                 (28)
#define RP23XX_PIO_FLEVEL_RX3_MASK                  (0x0f << RP23XX_PIO_FLEVEL_RX3_SHIFT)
#define RP23XX_PIO_FLEVEL_TX3_SHIFT                 (24)
#define RP23XX_PIO_FLEVEL_TX3_MASK                  (0x0f << RP23XX_PIO_FLEVEL_TX3_SHIFT)
#define RP23XX_PIO_FLEVEL_RX2_SHIFT                 (20)
#define RP23XX_PIO_FLEVEL_RX2_MASK                  (0x0f << RP23XX_PIO_FLEVEL_RX2_SHIFT)
#define RP23XX_PIO_FLEVEL_TX2_SHIFT                 (16)
#define RP23XX_PIO_FLEVEL_TX2_MASK                  (0x0f << RP23XX_PIO_FLEVEL_TX2_SHIFT)
#define RP23XX_PIO_FLEVEL_RX1_SHIFT                 (12)
#define RP23XX_PIO_FLEVEL_RX1_MASK                  (0x0f << RP23XX_PIO_FLEVEL_RX1_SHIFT)
#define RP23XX_PIO_FLEVEL_TX1_SHIFT                 (8)
#define RP23XX_PIO_FLEVEL_TX1_MASK                  (0x0f << RP23XX_PIO_FLEVEL_TX1_SHIFT)
#define RP23XX_PIO_FLEVEL_RX0_SHIFT                 (4)
#define RP23XX_PIO_FLEVEL_RX0_MASK                  (0x0f << RP23XX_PIO_FLEVEL_RX0_SHIFT)
#define RP23XX_PIO_FLEVEL_TX0_SHIFT                 (0)
#define RP23XX_PIO_FLEVEL_TX0_MASK                  (0x0f)

#define RP23XX_PIO_FLEVEL_TX_MASK(n)                (0x0f << 8*n)
#define RP23XX_PIO_FLEVEL_RX_MASK(n)                (0xf0 << 8*n)

#define RP23XX_PIO_IRQ_MASK                         (0xff)

#define RP23XX_PIO_IRQ_FORCE_MASK                   (0xff)

#define RP23XX_PIO_DBG_CFGINFO_VERSION_SHIFT        (28)    /* Version of the core PIO hardware */
#define RP23XX_PIO_DBG_CFGINFO_VERSION_MASK         (0xf << RP23XX_PIO_DBG_CFGINFO_VERSION_SHIFT)
#define RP23XX_PIO_DBG_CFGINFO_IMEM_SIZE_SHIFT      (16)    /* The size of the instruction memory, measured in units of one instruction */
#define RP23XX_PIO_DBG_CFGINFO_IMEM_SIZE_MASK       (0x3f << RP23XX_PIO_DBG_CFGINFO_IMEM_SIZE_SHIFT)
#define RP23XX_PIO_DBG_CFGINFO_SM_COUNT_SHIFT       (8)     /* The number of state machines this PIO instance is equipped with. */
#define RP23XX_PIO_DBG_CFGINFO_SM_COUNT_MASK        (0x0f << RP23XX_PIO_DBG_CFGINFO_SM_COUNT_SHIFT)
#define RP23XX_PIO_DBG_CFGINFO_FIFO_DEPTH_MASK      (0x3f)  /* The depth of the state machine TX/RX FIFOs, measured in words. Joining fifos via SHIFTCTRL_FJOIN gives one FIFO with double this depth. */

#define RP23XX_PIO_INSTR_MEM_MASK                   (0xffff)

#define RP23XX_PIO_SM_CLKDIV_INT_SHIFT             (16)  /* Effective frequency is sysclk/int. Value of 0 is interpreted as max possible value */
#define RP23XX_PIO_SM_CLKDIV_INT_MASK              (0xffff << RP23XX_PIO_SM_CLKDIV_INT_SHIFT)
#define RP23XX_PIO_SM_CLKDIV_FRAC_SHIFT            (8)   /* Fractional part of clock divider */
#define RP23XX_PIO_SM_CLKDIV_FRAC_MASK             (0xff << RP23XX_PIO_SM_CLKDIV_FRAC_SHIFT)

#define RP23XX_PIO_SM_EXECCTRL_EXEC_STALLED        (1 << 31)  /* An instruction written to SMx_INSTR is stalled, and latched by the state machine. Will clear once the instruction completes. */
#define RP23XX_PIO_SM_EXECCTRL_SIDE_EN             (1 << 30)  /* If 1, the delay MSB is used as side-set enable, rather than a side-set data bit. This allows instructions to perform side-set optionally, rather than on every instruction. */
#define RP23XX_PIO_SM_EXECCTRL_SIDE_PINDIR         (1 << 29)  /* Side-set data is asserted to pin OEs instead of pin values */
#define RP23XX_PIO_SM_EXECCTRL_JMP_PIN_SHIFT       (24)       /* The GPIO number to use as condition for JMP PIN. Unaffected by input mapping. */
#define RP23XX_PIO_SM_EXECCTRL_JMP_PIN_MASK        (0x1f << RP23XX_PIO_SM_EXECCTRL_JMP_PIN_SHIFT)
#define RP23XX_PIO_SM_EXECCTRL_OUT_EN_SEL_SHIFT    (19)       /* Which data bit to use for inline OUT enable */
#define RP23XX_PIO_SM_EXECCTRL_OUT_EN_SEL_MASK     (0x1f << RP23XX_PIO_SM_EXECCTRL_OUT_EN_SEL_SHIFT)
#define RP23XX_PIO_SM_EXECCTRL_INLINE_OUT_EN       (1 << 18)  /* If 1, use a bit of OUT data as an auxiliary write enable When used in conjunction with OUT_STICKY, writes with an enable of 0 will deassert the latest pin write. This can create useful masking/override behaviour due to the priority ordering of state machine pin writes (SM0 < SM1 < ...) */
#define RP23XX_PIO_SM_EXECCTRL_OUT_STICKY          (1 << 17)  /* Continuously assert the most recent OUT/SET to the pins */
#define RP23XX_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT      (12)       /* After reaching this address, execution is wrapped to wrap_bottom. If the instruction is a jump, and the jump condition is true, the jump takes priority. */
#define RP23XX_PIO_SM_EXECCTRL_WRAP_TOP_MASK       (0x1f << RP23XX_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT)
#define RP23XX_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT   (7)        /* After reaching wrap_top, execution is wrapped to this address. */
#define RP23XX_PIO_SM_EXECCTRL_WRAP_BOTTOM_MASK    (0x1f << RP23XX_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT)
#define RP23XX_PIO_SM_EXECCTRL_STATUS_SEL_SHIFT    (5)        /* Comparison used for the MOV x, STATUS instruction */
#define RP23XX_PIO_SM_EXECCTRL_STATUS_SEL_MASK     (0x3 << RP23XX_PIO_SM_EXECCTRL_STATUS_SEL_SHIFT)
#define RP23XX_PIO_SM_EXECCTRL_STATUS_N_MASK       (0x0f)     /* : Comparison level or IRQ index for the MOV x, STATUS instruction */

#define RP23XX_PIO_SM_SHIFTCTRL_FJOIN_RX           (1 << 31)  /* When 1, RX FIFO steals the TX FIFO's storage, and becomes twice as deep. TX FIFO is disabled as a result (always reads as both full and empty). FIFOs are flushed when this bit is changed. */
#define RP23XX_PIO_SM_SHIFTCTRL_FJOIN_TX           (1 << 30)  /* When 1, TX FIFO steals the RX FIFO's storage, and becomes twice as deep. RX FIFO is disabled as a result (always reads as both full and empty). FIFOs are flushed when this bit is changed. */
#define RP23XX_PIO_SM_SHIFTCTRL_PULL_THRESH_SHIFT  (25)       /* Number of bits shifted out of TXSR before autopull or conditional pull. Write 0 for value of 32. */
#define RP23XX_PIO_SM_SHIFTCTRL_PULL_THRESH_MASK   (0x1f << RP23XX_PIO_SM_SHIFTCTRL_PULL_THRESH_SHIFT)
#define RP23XX_PIO_SM_SHIFTCTRL_PUSH_THRESH_SHIFT  (20)       /* Number of bits shifted into RXSR before autopush or conditional push. Write 0 for value of 32. */
#define RP23XX_PIO_SM_SHIFTCTRL_PUSH_THRESH_MASK   (0x1f << RP23XX_PIO_SM_SHIFTCTRL_PUSH_THRESH_SHIFT)
#define RP23XX_PIO_SM_SHIFTCTRL_OUT_SHIFTDIR       (1 << 19)  /* 1 = shift out of output shift register to right. 0 = to left. */
#define RP23XX_PIO_SM_SHIFTCTRL_IN_SHIFTDIR        (1 << 18)  /* 1 = shift input shift register to right (data enters from left). 0 = to left. */
#define RP23XX_PIO_SM_SHIFTCTRL_AUTOPULL           (1 << 17)  /* Pull automatically when the output shift register is emptied */
#define RP23XX_PIO_SM_SHIFTCTRL_AUTOPUSH           (1 << 16)  /* Push automatically when the input shift register is filled */
#define RP23XX_PIO_SM_SHIFTCTRL_FJOIN_RX_PUT       (1 << 15)  /* If 1, disable this state machine’s RX FIFO, make its storage available for random write access by the state machine (using the put instruction) and, unless FJOIN_RX_GET is also set, random read access by the processor (through the RXFx_PUTGETy registers). */
#define RP23XX_PIO_SM_SHIFTCTRL_FJOIN_TX_GET       (1 << 14)  /* If 1, disable this state machine’s RX FIFO, make its storage available for random read access by the state machine (using the get instruction) and, unless FJOIN_RX_PUT is also set, random write access by the processor (through the RXFx_PUTGETy registers) */
#define RP23XX_PIO_SM_ADDR_MASK                    (0x1f)

#define RP23XX_PIO_SM_INSTR_MASK                   (0xffff)

#define RP23XX_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT  (29)    /* The number of delay bits co-opted for side-set. Inclusive of the enable bit, if present. */
#define RP23XX_PIO_SM_PINCTRL_SIDESET_COUNT_MASK   (0x07 << RP23XX_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT)
#define RP23XX_PIO_SM_PINCTRL_SET_COUNT_SHIFT      (26)    /* The number of pins asserted by a SET. Max of 5 */
#define RP23XX_PIO_SM_PINCTRL_SET_COUNT_MASK       (0x07 << RP23XX_PIO_SM_PINCTRL_SET_COUNT_SHIFT)
#define RP23XX_PIO_SM_PINCTRL_OUT_COUNT_SHIFT      (20)    /* The number of pins asserted by an OUT. Value of 0 -> 32 pins */
#define RP23XX_PIO_SM_PINCTRL_OUT_COUNT_MASK       (0x3f << RP23XX_PIO_SM_PINCTRL_OUT_COUNT_SHIFT)
#define RP23XX_PIO_SM_PINCTRL_IN_BASE_SHIFT        (15)    /* The virtual pin corresponding to IN bit 0 */
#define RP23XX_PIO_SM_PINCTRL_IN_BASE_MASK         (0x1f << RP23XX_PIO_SM_PINCTRL_IN_BASE_SHIFT)
#define RP23XX_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT   (10)    /* The virtual pin corresponding to delay field bit 0 */
#define RP23XX_PIO_SM_PINCTRL_SIDESET_BASE_MASK    (0x1f << RP23XX_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT)
#define RP23XX_PIO_SM_PINCTRL_SET_BASE_SHIFT       (5)     /* The virtual pin corresponding to SET bit 0 */
#define RP23XX_PIO_SM_PINCTRL_SET_BASE_MASK        (0x1f << RP23XX_PIO_SM_PINCTRL_SET_BASE_SHIFT)
#define RP23XX_PIO_SM_PINCTRL_OUT_BASE_SHIFT       (0)     /* The virtual pin corresponding to OUT bit 0 */
#define RP23XX_PIO_SM_PINCTRL_OUT_BASE_MASK        (0x1f)  /* The virtual pin corresponding to OUT bit 0 */

#define RP23XX_PIO_GPIOBASE_MASK                   (1 << 4) /* Relocate GPIO 0 (from PIO’s point of view) in the system GPIO numbering, to access more than 32 GPIOs from PIO */

#define RP23XX_PIO_INTR_SM7                        (1 << 15)
#define RP23XX_PIO_INTR_SM6                        (1 << 14)
#define RP23XX_PIO_INTR_SM5                        (1 << 13)
#define RP23XX_PIO_INTR_SM4                        (1 << 12)
#define RP23XX_PIO_INTR_SM3                        (1 << 11)
#define RP23XX_PIO_INTR_SM2                        (1 << 10)
#define RP23XX_PIO_INTR_SM1                        (1 << 9)
#define RP23XX_PIO_INTR_SM0                        (1 << 8)
#define RP23XX_PIO_INTR_SM3_TXNFULL                (1 << 7)
#define RP23XX_PIO_INTR_SM2_TXNFULL                (1 << 6)
#define RP23XX_PIO_INTR_SM1_TXNFULL                (1 << 5)
#define RP23XX_PIO_INTR_SM0_TXNFULL                (1 << 4)
#define RP23XX_PIO_INTR_SM3_RXNEMPTY               (1 << 3)
#define RP23XX_PIO_INTR_SM2_RXNEMPTY               (1 << 2)
#define RP23XX_PIO_INTR_SM1_RXNEMPTY               (1 << 1)
#define RP23XX_PIO_INTR_SM0_RXNEMPTY               (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PIO_H */
