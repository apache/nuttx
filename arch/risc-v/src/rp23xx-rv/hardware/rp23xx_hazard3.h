/****************************************************************************
 * arch/risc-v/src/rp23xx-rv/hardware/rp23xx_hazard3.h
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
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_RP23XX_HARDWARE_RP23XX_HAZARD3_H
#define __ARCH_RISCV_SRC_RP23XX_HARDWARE_RP23XX_HAZARD3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define hazard3_irqarray_read(csr, index) (READ_AND_SET_CSR(csr, (index)) >> 16)
#define hazard3_irqarray_write(csr, index, data) (WRITE_CSR(csr, (index) | ((uint32_t)(data) << 16)))
#define hazard3_irqarray_set(csr, index, data) (SET_CSR(csr, (index) | ((uint32_t)(data) << 16)))
#define hazard3_irqarray_clear(csr, index, data) (CLEAR_CSR(csr, (index) | ((uint32_t)(data) << 16)))

/****************************************************************************
 * This is a Hazard3 custom CSR.
 ****************************************************************************/
#define RVCSR_PMPCFGM0_OFFSET 0x00000bd0
#define RVCSR_PMPCFGM0_BITS   0x0000ffff
#define RVCSR_PMPCFGM0_RESET  0x00000000
#define RVCSR_PMPCFGM0_MSB    15
#define RVCSR_PMPCFGM0_LSB    0
#define RVCSR_PMPCFGM0_ACCESS "RW"

/****************************************************************************
 * Register    : RVCSR_MEIEA
 * Description : External interrupt enable array.
 *
 *          The array contains a read-write bit for each external interrupt
 *          request: a `1` bit indicates that interrupt is currently
 *          enabled. At reset, all external interrupts are disabled.
 *
 *          If enabled, an external interrupt can cause assertion of the
 *          standard RISC-V machine external interrupt pending flag
 *          (`mip.meip`), and therefore cause the processor to enter the
 *          external interrupt vector. See `meipa`.
 *
 *          There are up to 512 external interrupts. The upper half of this
 *          register contains a 16-bit window into the full 512-bit vector.
 *          The window is indexed by the 5 LSBs of the write data.
 ****************************************************************************/
#define RVCSR_MEIEA_OFFSET 0x00000be0
#define RVCSR_MEIEA_BITS   0xffff001f
#define RVCSR_MEIEA_RESET  0x00000000

/****************************************************************************
 * Field       : RVCSR_MEIEA_WINDOW
 * Description : 16-bit read/write window into the external interrupt enable
 *               array
 ****************************************************************************/
#define RVCSR_MEIEA_WINDOW_RESET  0x0000
#define RVCSR_MEIEA_WINDOW_BITS   0xffff0000
#define RVCSR_MEIEA_WINDOW_MSB    31
#define RVCSR_MEIEA_WINDOW_LSB    16
#define RVCSR_MEIEA_WINDOW_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEIEA_INDEX
 * Description : Write-only self-clearing field (no value is stored) used to
 *               control which window of the array appears in `window`.
 ****************************************************************************/
#define RVCSR_MEIEA_INDEX_RESET  0x00
#define RVCSR_MEIEA_INDEX_BITS   0x0000001f
#define RVCSR_MEIEA_INDEX_MSB    4
#define RVCSR_MEIEA_INDEX_LSB    0
#define RVCSR_MEIEA_INDEX_ACCESS "WO"

/****************************************************************************
 * Register    : RVCSR_MEIPA
 * Description : External interrupt pending array
 *
 *           Contains a read-only bit for each external interrupt request.
 *           Similarly to `meiea`, this register is a window into an array
 *           of up to 512 external interrupt flags. The status appears in
 *           the upper 16 bits of the value read from `meipa`, and the lower
 *           5 bits of the value _written_ by the same CSR instruction (or 0
 *           if no write takes place) select a 16-bit window of the full
 *           interrupt pending array.
 *
 *           A `1` bit indicates that interrupt is currently asserted. IRQs
 *           are assumed to be level-sensitive, and the relevant `meipa` bit
 *           is cleared by servicing the requester so that it deasserts its
 *           interrupt request.
 *
 *           When any interrupt of sufficient priority is both set in
 *           `meipa` and enabled in `meiea`, the standard RISC-V external
 *           interrupt pending bit `mip.meip` is asserted. In other words,
 *           `meipa` is filtered by `meiea` to generate the standard
 *           `mip.meip` flag.
 ****************************************************************************/
#define RVCSR_MEIPA_OFFSET 0x00000be1
#define RVCSR_MEIPA_BITS   0xffff001f
#define RVCSR_MEIPA_RESET  0x00000000
/****************************************************************************
 * Field       : RVCSR_MEIPA_WINDOW
 * Description : 16-bit read-only window into the external interrupt pending
 *               array
 ****************************************************************************/
#define RVCSR_MEIPA_WINDOW_RESET  "-"
#define RVCSR_MEIPA_WINDOW_BITS   0xffff0000
#define RVCSR_MEIPA_WINDOW_MSB    31
#define RVCSR_MEIPA_WINDOW_LSB    16
#define RVCSR_MEIPA_WINDOW_ACCESS "RO"

/****************************************************************************
 * Field       : RVCSR_MEIPA_INDEX
 * Description : Write-only, self-clearing field (no value is stored) used to
 *               control which window of the array appears in `window`.
 ****************************************************************************/
#define RVCSR_MEIPA_INDEX_RESET  0x00
#define RVCSR_MEIPA_INDEX_BITS   0x0000001f
#define RVCSR_MEIPA_INDEX_MSB    4
#define RVCSR_MEIPA_INDEX_LSB    0
#define RVCSR_MEIPA_INDEX_ACCESS "WO"

/****************************************************************************
 * Register    : RVCSR_MEIFA
 * Description : External interrupt force array
 *
 *            Contains a read-write bit for every interrupt request. Writing
 *            a 1 to a bit in the interrupt force array causes the
 *            corresponding bit to become pending in `meipa`. Software can
 *            use this feature to manually trigger a particular interrupt.
 *
 *            There are no restrictions on using `meifa` inside of an
 *            interrupt. The more useful case here is to schedule some lower-
 *            priority handler from within a high-priority interrupt, so that
 *            it will execute before the core returns to the foreground code.
 *            Implementers may wish to reserve some external IRQs with their
 *            external inputs tied to 0 for this purpose.
 *
 *            Bits can be cleared by software, and are cleared automatically
 *            by hardware upon a read of `meinext` which returns the
 *            corresponding IRQ number in `meinext.irq` with `mienext.noirq`
 *            clear (no matter whether `meinext.update` is written).
 *
 *            `meifa` implements the same array window indexing scheme as
 *            `meiea` and `meipa`.
 ****************************************************************************/
#define RVCSR_MEIFA_OFFSET 0x00000be2
#define RVCSR_MEIFA_BITS   0xffff001f
#define RVCSR_MEIFA_RESET  0x00000000

/****************************************************************************
 * Field       : RVCSR_MEIFA_WINDOW
 * Description : 16-bit read/write window into the external interrupt force
 *               array
 ****************************************************************************/
#define RVCSR_MEIFA_WINDOW_RESET  0x0000
#define RVCSR_MEIFA_WINDOW_BITS   0xffff0000
#define RVCSR_MEIFA_WINDOW_MSB    31
#define RVCSR_MEIFA_WINDOW_LSB    16
#define RVCSR_MEIFA_WINDOW_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEIFA_INDEX
 * Description : Write-only, self-clearing field (no value is stored) used to
 *               control which window of the array appears in `window`.
 ****************************************************************************/
#define RVCSR_MEIFA_INDEX_RESET  0x00
#define RVCSR_MEIFA_INDEX_BITS   0x0000001f
#define RVCSR_MEIFA_INDEX_MSB    4
#define RVCSR_MEIFA_INDEX_LSB    0
#define RVCSR_MEIFA_INDEX_ACCESS "WO"

/****************************************************************************
 * Register    : RVCSR_MEIPRA
 * Description : External interrupt priority array
 *
 *           Each interrupt has an (up to) 4-bit priority value associated
 *           with it, and each access to this register reads and/or writes a
 *           16-bit window containing four such priority values. When less
 *           than 16 priority levels are available, the LSBs of the priority
 *           fields are hardwired to 0.
 *
 *           When an interrupt's priority is lower than the current
 *           preemption priority `meicontext.preempt`, it is treated as not
 *           being pending for the purposes of `mip.meip`. The pending bit
 *           in `meipa` will still assert, but the machine external
 *           interrupt pending bit `mip.meip` will not, so the processor
 *           will ignore this interrupt. See `meicontext`.
 ****************************************************************************/
#define RVCSR_MEIPRA_OFFSET 0x00000be3
#define RVCSR_MEIPRA_BITS   0xffff001f
#define RVCSR_MEIPRA_RESET  0x00000000

/****************************************************************************
 * Field       : RVCSR_MEIPRA_WINDOW
 * Description : 16-bit read/write window into the external interrupt
 *               priority array, containing four 4-bit priority values.
 ****************************************************************************/
#define RVCSR_MEIPRA_WINDOW_RESET  0x0000
#define RVCSR_MEIPRA_WINDOW_BITS   0xffff0000
#define RVCSR_MEIPRA_WINDOW_MSB    31
#define RVCSR_MEIPRA_WINDOW_LSB    16
#define RVCSR_MEIPRA_WINDOW_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEIPRA_INDEX
 * Description : Write-only, self-clearing field (no value is stored) used to
 *               control which window of the array appears in `window`.
 ****************************************************************************/
#define RVCSR_MEIPRA_INDEX_RESET  0x00
#define RVCSR_MEIPRA_INDEX_BITS   0x0000001f
#define RVCSR_MEIPRA_INDEX_MSB    4
#define RVCSR_MEIPRA_INDEX_LSB    0
#define RVCSR_MEIPRA_INDEX_ACCESS "WO"

/****************************************************************************
 * Register    : RVCSR_MEINEXT
 * Description : Get next external interrupt
 *
 *           Contains the index of the highest-priority external interrupt
 *           which is both asserted in `meipa` and enabled in `meiea`, left-
 *           shifted by 2 so that it can be used to index an array of 32-bit
 *           function pointers. If there is no such interrupt, the MSB is
 *           set.
 *
 *           When multiple interrupts of the same priority are both pending
 *           and enabled, the lowest-numbered wins. Interrupts with priority
 *           less than `meicontext.ppreempt` -- the _previous_ preemption
 *           priority -- are treated as though they are not pending. This is
 *           to ensure that a preempting interrupt frame does not service
 *           interrupts which may be in progress in the frame that was
 *           preempted.
 ****************************************************************************/
#define RVCSR_MEINEXT_OFFSET 0x00000be4
#define RVCSR_MEINEXT_BITS   0x800007fd
#define RVCSR_MEINEXT_RESET  0x00000000

/****************************************************************************
 * Field       : RVCSR_MEINEXT_NOIRQ
 * Description : Set when there is no external interrupt which is enabled,
 *             pending, and has priority greater than or equal to
 *             `meicontext.ppreempt`. Can be efficiently tested with a `bltz`
 *             or `bgez` instruction.
 ****************************************************************************/
#define RVCSR_MEINEXT_NOIRQ_RESET  0x0
#define RVCSR_MEINEXT_NOIRQ_BITS   0x80000000
#define RVCSR_MEINEXT_NOIRQ_MSB    31
#define RVCSR_MEINEXT_NOIRQ_LSB    31
#define RVCSR_MEINEXT_NOIRQ_ACCESS "RO"

/****************************************************************************
 * Field       : RVCSR_MEINEXT_IRQ
 * Description : Index of the highest-priority active external interrupt.
 *               Zero when no external interrupts with sufficient priority
 *               are both pending and enabled.
 ****************************************************************************/
#define RVCSR_MEINEXT_IRQ_RESET  0x000
#define RVCSR_MEINEXT_IRQ_BITS   0x000007fc
#define RVCSR_MEINEXT_IRQ_MSB    10
#define RVCSR_MEINEXT_IRQ_LSB    2
#define RVCSR_MEINEXT_IRQ_ACCESS "RO"

/****************************************************************************
 * Field       : RVCSR_MEINEXT_UPDATE
 * Description : Writing 1 (self-clearing) causes hardware to update
 *            `meicontext` according to the IRQ number and preemption
 *            priority of the interrupt indicated in `noirq`/`irq`. This
 *            should be done in a single atomic operation, i.e. `csrrsi a0,
 *            meinext, 0x1`.
 ****************************************************************************/
#define RVCSR_MEINEXT_UPDATE_RESET  0x0
#define RVCSR_MEINEXT_UPDATE_BITS   0x00000001
#define RVCSR_MEINEXT_UPDATE_MSB    0
#define RVCSR_MEINEXT_UPDATE_LSB    0
#define RVCSR_MEINEXT_UPDATE_ACCESS "SC"

/****************************************************************************
 * Register    : RVCSR_MEICONTEXT
 * Description : External interrupt context register
 *
 *         Configures the priority level for interrupt preemption, and
 *         helps software track which interrupt it is currently in. The
 *         latter is useful when a common interrupt service routine
 *         handles interrupt requests from multiple instances of the same
 *         peripheral.
 *
 *         A three-level stack of preemption priorities is maintained in
 *         the `preempt`, `ppreempt` and `pppreempt` fields. The priority
 *         stack is saved when hardware enters the external interrupt
 *         vector, and restored by an `mret` instruction if
 *         `meicontext.mreteirq` is set.
 *
 *         The top entry of the priority stack, `preempt`, is used by
 *         hardware to ensure that only higher-priority interrupts can
 *         preempt the current interrupt. The next entry, `ppreempt`, is
 *         used to avoid servicing interrupts which may already be in
 *         progress in a frame that was preempted. The third entry,
 *         `pppreempt`, has no hardware effect, but ensures that `preempt`
 *         and `ppreempt` can be correctly saved/restored across arbitrary
 *         levels of preemption.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_OFFSET 0x00000be5
#define RVCSR_MEICONTEXT_BITS   0xff1f9fff
#define RVCSR_MEICONTEXT_RESET  0x00008000

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_PPPREEMPT
 * Description : Previous `ppreempt`. Set to `ppreempt` on priority save,
 *         set to zero on priority restore.  Has no hardware effect,
 *         but ensures that when `meicontext` is saved/restored correctly,
 *        `preempt` and `ppreempt` stack correctly through arbitrarily many
 *         preemption frames.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_PPPREEMPT_RESET  0x0
#define RVCSR_MEICONTEXT_PPPREEMPT_BITS   0xf0000000
#define RVCSR_MEICONTEXT_PPPREEMPT_MSB    31
#define RVCSR_MEICONTEXT_PPPREEMPT_LSB    28
#define RVCSR_MEICONTEXT_PPPREEMPT_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_PPREEMPT
 * Description : Previous `preempt`. Set to `preempt` on priority save,
 *               restored to to `pppreempt` on priority restore.
 *
 *               IRQs of lower priority than `ppreempt` are not visible in
 *               `meinext`, so that a preemptee is not re-taken in the
 *               preempting frame.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_PPREEMPT_RESET  0x0
#define RVCSR_MEICONTEXT_PPREEMPT_BITS   0x0f000000
#define RVCSR_MEICONTEXT_PPREEMPT_MSB    27
#define RVCSR_MEICONTEXT_PPREEMPT_LSB    24
#define RVCSR_MEICONTEXT_PPREEMPT_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_PREEMPT
 * Description : Minimum interrupt priority to preempt the current interrupt.
 *            Interrupts with lower priority than `preempt` do not cause the
 *            core to transfer to an interrupt handler. Updated by hardware
 *            when when `meinext.update` is written, or when hardware enters
 *            the external interrupt vector.
 *
 *            If an interrupt is present in `meinext` when this field is
 *            updated, then `preempt` is set to one level greater than that
 *            interrupt's priority. Otherwise, `ppreempt` is set to one level
 *            greater than the maximum interrupt priority, disabling
 *            preemption.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_PREEMPT_RESET  0x00
#define RVCSR_MEICONTEXT_PREEMPT_BITS   0x001f0000
#define RVCSR_MEICONTEXT_PREEMPT_MSB    20
#define RVCSR_MEICONTEXT_PREEMPT_LSB    16
#define RVCSR_MEICONTEXT_PREEMPT_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_NOIRQ
 * Description : Not in interrupt (read/write). Set to 1 at reset. Set to
 *               `meinext.noirq` when `meinext.update` is written.
 *               No hardware effect.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_NOIRQ_RESET  0x1
#define RVCSR_MEICONTEXT_NOIRQ_BITS   0x00008000
#define RVCSR_MEICONTEXT_NOIRQ_MSB    15
#define RVCSR_MEICONTEXT_NOIRQ_LSB    15
#define RVCSR_MEICONTEXT_NOIRQ_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_IRQ
 * Description : Current IRQ number (read/write). Set to `meinext.irq` when
 *               `meinext.update` is written. No hardware effect.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_IRQ_RESET  0x000
#define RVCSR_MEICONTEXT_IRQ_BITS   0x00001ff0
#define RVCSR_MEICONTEXT_IRQ_MSB    12
#define RVCSR_MEICONTEXT_IRQ_LSB    4
#define RVCSR_MEICONTEXT_IRQ_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_MTIESAVE
 * Description : Reads as the current value of `mie.mtie`, if `clearts`
 *           is set by the same CSR access instruction. Otherwise reads as 0.
 *           Writes are ORed into `mie.mtie`.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_MTIESAVE_RESET  0x0
#define RVCSR_MEICONTEXT_MTIESAVE_BITS   0x00000008
#define RVCSR_MEICONTEXT_MTIESAVE_MSB    3
#define RVCSR_MEICONTEXT_MTIESAVE_LSB    3
#define RVCSR_MEICONTEXT_MTIESAVE_ACCESS "RO"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_MSIESAVE
 * Description : Reads as the current value of `mie.msie`, if `clearts`
 *           is set by the same CSR access instruction. Otherwise reads as 0.
 *           Writes are ORed into `mie.msie`.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_MSIESAVE_RESET  0x0
#define RVCSR_MEICONTEXT_MSIESAVE_BITS   0x00000004
#define RVCSR_MEICONTEXT_MSIESAVE_MSB    2
#define RVCSR_MEICONTEXT_MSIESAVE_LSB    2
#define RVCSR_MEICONTEXT_MSIESAVE_ACCESS "RO"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_CLEARTS
 * Description : Write-1 self-clearing field. Writing 1 will clear `mie.mtie`
 *           and `mie.msie`, and present their prior values in the
 *           `mtiesave` and `msiesave` of this register. This makes it safe
 *           to re-enable IRQs (via `mstatus.mie`) without the possibility
 *           of being preempted by the standard timer and soft interrupt
 *           handlers, which may not be aware of Hazard3's interrupt
 *           hardware.
 *
 *           The clear due to `clearts` takes precedence over the set due to
 *           `mtiesave`/`msiesave`, although it would be unusual for
 *           software to write both on the same cycle.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_CLEARTS_RESET  0x0
#define RVCSR_MEICONTEXT_CLEARTS_BITS   0x00000002
#define RVCSR_MEICONTEXT_CLEARTS_MSB    1
#define RVCSR_MEICONTEXT_CLEARTS_LSB    1
#define RVCSR_MEICONTEXT_CLEARTS_ACCESS "SC"

/****************************************************************************
 * Field       : RVCSR_MEICONTEXT_MRETEIRQ
 * Description : If 1, enable restore of the preemption priority stack on
 *               `mret`. This bit is set on entering the external interrupt
 *               vector, cleared by `mret`, and cleared upon taking any trap
 *               other than an external interrupt.
 *
 *            Provided `meicontext` is saved on entry to the external
 *            interrupt vector (before enabling preemption), is restored
 *            before exiting, and the standard software/timer IRQs are
 *            prevented from preempting (e.g. by using `clearts`), this flag
 *            allows the hardware to safely manage the preemption priority
 *            stack even when an external interrupt handler may take
 *            exceptions.
 ****************************************************************************/
#define RVCSR_MEICONTEXT_MRETEIRQ_RESET  0x0
#define RVCSR_MEICONTEXT_MRETEIRQ_BITS   0x00000001
#define RVCSR_MEICONTEXT_MRETEIRQ_MSB    0
#define RVCSR_MEICONTEXT_MRETEIRQ_LSB    0
#define RVCSR_MEICONTEXT_MRETEIRQ_ACCESS "RW"

/****************************************************************************
 * Register    : RVCSR_MSLEEP
 * Description : M-mode sleep control register
 ****************************************************************************/
#define RVCSR_MSLEEP_OFFSET 0x00000bf0
#define RVCSR_MSLEEP_BITS   0x00000007
#define RVCSR_MSLEEP_RESET  0x00000000

/****************************************************************************
 * Field       : RVCSR_MSLEEP_SLEEPONBLOCK
 * Description : Enter the deep sleep state configured by
 *            msleep.deepsleep/msleep.powerdown on a `h3.block` instruction,
 *            as well as a standard `wfi`. If this bit is clear, a `h3.block`
 *            is always implemented as a simple pipeline stall.
 ****************************************************************************/
#define RVCSR_MSLEEP_SLEEPONBLOCK_RESET  0x0
#define RVCSR_MSLEEP_SLEEPONBLOCK_BITS   0x00000004
#define RVCSR_MSLEEP_SLEEPONBLOCK_MSB    2
#define RVCSR_MSLEEP_SLEEPONBLOCK_LSB    2
#define RVCSR_MSLEEP_SLEEPONBLOCK_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MSLEEP_POWERDOWN
 * Description : Release the external power request when going to sleep. The
 *          function of this is platform-defined -- it may do nothing, it
 *          may do something simple like clock-gating the fabric, or it may
 *          be tied to some complex system-level power controller.
 *
 *          When waking, the processor reasserts its external power-up
 *          request, and will not fetch any instructions until the request
 *          is acknowledged. This may add considerable latency to the
 *          wakeup.
 ****************************************************************************/
#define RVCSR_MSLEEP_POWERDOWN_RESET  0x0
#define RVCSR_MSLEEP_POWERDOWN_BITS   0x00000002
#define RVCSR_MSLEEP_POWERDOWN_MSB    1
#define RVCSR_MSLEEP_POWERDOWN_LSB    1
#define RVCSR_MSLEEP_POWERDOWN_ACCESS "RW"

/****************************************************************************
 * Field       : RVCSR_MSLEEP_DEEPSLEEP
 * Description : Deassert the processor clock enable when entering the sleep
 *          state. If a clock gate is instantiated, this allows most of the
 *          processor (everything except the power state machine and the
 *          interrupt and halt input registers) to be clock gated whilst
 *          asleep, which may reduce the sleep current. This adds one cycle
 *          to the wakeup latency.
 ****************************************************************************/
#define RVCSR_MSLEEP_DEEPSLEEP_RESET  0x0
#define RVCSR_MSLEEP_DEEPSLEEP_BITS   0x00000001
#define RVCSR_MSLEEP_DEEPSLEEP_MSB    0
#define RVCSR_MSLEEP_DEEPSLEEP_LSB    0
#define RVCSR_MSLEEP_DEEPSLEEP_ACCESS "RW"

/****************************************************************************
 * Register    : RVCSR_DMDATA0
 * Description : The Debug Module's DATA0 register is mapped into
 *           Hazard3's CSR space so that the Debug Module can exchange
 *           data with the core by executing CSR access instructions
 *           (this is used to implementthe Abstract Access Register command).
 *           Only accessible in Debug Mode.
 ****************************************************************************/
#define RVCSR_DMDATA0_OFFSET 0x00000bff
#define RVCSR_DMDATA0_BITS   0xffffffff
#define RVCSR_DMDATA0_RESET  0x00000000
#define RVCSR_DMDATA0_MSB    31
#define RVCSR_DMDATA0_LSB    0
#define RVCSR_DMDATA0_ACCESS "RW"

/****************************************************************************
 * Register    : RVCSR_CYCLE
 * Description : Read-only U-mode alias of mcycle, accessible when
 *               `mcounteren.cy` is set
 ****************************************************************************/
#define RVCSR_CYCLE_OFFSET 0x00000c00
#define RVCSR_CYCLE_BITS   0xffffffff
#define RVCSR_CYCLE_RESET  0x00000000
#define RVCSR_CYCLE_MSB    31
#define RVCSR_CYCLE_LSB    0
#define RVCSR_CYCLE_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_INSTRET
 * Description : Read-only U-mode alias of minstret, accessible when
 *               `mcounteren.ir` is set
 ****************************************************************************/
#define RVCSR_INSTRET_OFFSET 0x00000c02
#define RVCSR_INSTRET_BITS   0xffffffff
#define RVCSR_INSTRET_RESET  0x00000000
#define RVCSR_INSTRET_MSB    31
#define RVCSR_INSTRET_LSB    0
#define RVCSR_INSTRET_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_CYCLEH
 * Description : Read-only U-mode alias of mcycleh, accessible when
 *               `mcounteren.cy` is set
 ****************************************************************************/
#define RVCSR_CYCLEH_OFFSET 0x00000c80
#define RVCSR_CYCLEH_BITS   0xffffffff
#define RVCSR_CYCLEH_RESET  0x00000000
#define RVCSR_CYCLEH_MSB    31
#define RVCSR_CYCLEH_LSB    0
#define RVCSR_CYCLEH_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_INSTRETH
 * Description : Read-only U-mode alias of minstreth, accessible when
 *               `mcounteren.ir` is set
 ****************************************************************************/
#define RVCSR_INSTRETH_OFFSET 0x00000c82
#define RVCSR_INSTRETH_BITS   0xffffffff
#define RVCSR_INSTRETH_RESET  0x00000000
#define RVCSR_INSTRETH_MSB    31
#define RVCSR_INSTRETH_LSB    0
#define RVCSR_INSTRETH_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_MVENDORID
 * Description : Vendor ID
 ****************************************************************************/
#define RVCSR_MVENDORID_OFFSET 0x00000f11
#define RVCSR_MVENDORID_BITS   0xffffffff
#define RVCSR_MVENDORID_RESET  0x00000000

/****************************************************************************
 * Field       : RVCSR_MVENDORID_BANK
 ****************************************************************************/
#define RVCSR_MVENDORID_BANK_RESET  "-"
#define RVCSR_MVENDORID_BANK_BITS   0xffffff80
#define RVCSR_MVENDORID_BANK_MSB    31
#define RVCSR_MVENDORID_BANK_LSB    7
#define RVCSR_MVENDORID_BANK_ACCESS "RO"

/****************************************************************************
 * Field       : RVCSR_MVENDORID_OFFSET
 ****************************************************************************/
#define RVCSR_MVENDORID_OFFSET_RESET  "-"
#define RVCSR_MVENDORID_OFFSET_BITS   0x0000007f
#define RVCSR_MVENDORID_OFFSET_MSB    6
#define RVCSR_MVENDORID_OFFSET_LSB    0
#define RVCSR_MVENDORID_OFFSET_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_MARCHID
 * Description : Architecture ID (Hazard3
 ****************************************************************************/
#define RVCSR_MARCHID_OFFSET 0x00000f12
#define RVCSR_MARCHID_BITS   0xffffffff
#define RVCSR_MARCHID_RESET  0x0000001b
#define RVCSR_MARCHID_MSB    31
#define RVCSR_MARCHID_LSB    0
#define RVCSR_MARCHID_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_MIMPID
 * Description : Implementation ID
 ****************************************************************************/
#define RVCSR_MIMPID_OFFSET 0x00000f13
#define RVCSR_MIMPID_BITS   0xffffffff
#define RVCSR_MIMPID_RESET  "-"
#define RVCSR_MIMPID_MSB    31
#define RVCSR_MIMPID_LSB    0
#define RVCSR_MIMPID_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_MHARTID
 * Description : Hardware thread ID
 *             On RP2350, core 0 has a hart ID of 0, and core 1 has a hart ID
 *             of 1.
 ****************************************************************************/
#define RVCSR_MHARTID_OFFSET 0x00000f14
#define RVCSR_MHARTID_BITS   0xffffffff
#define RVCSR_MHARTID_RESET  "-"
#define RVCSR_MHARTID_MSB    31
#define RVCSR_MHARTID_LSB    0
#define RVCSR_MHARTID_ACCESS "RO"

/****************************************************************************
 * Register    : RVCSR_MCONFIGPTR
 * Description : Pointer to configuration data structure (hardwired to 0
 ****************************************************************************/
#define RVCSR_MCONFIGPTR_OFFSET 0x00000f15
#define RVCSR_MCONFIGPTR_BITS   0xffffffff
#define RVCSR_MCONFIGPTR_RESET  0x00000000
#define RVCSR_MCONFIGPTR_MSB    31
#define RVCSR_MCONFIGPTR_LSB    0
#define RVCSR_MCONFIGPTR_ACCESS "RO"

#endif /* __ARCH_RISCV_SRC_RP23XX_HARDWARE_RP23XX_HAZARD3_H */
