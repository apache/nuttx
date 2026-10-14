/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_adc.h
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

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_ADC_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/rm57_adc_ioctl.h>

#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MibADC register map, TRM SPNU562A Table 22-6.  Offsets are relative to
 * RM57_ADC1_BASE / RM57_ADC2_BASE.  All registers are 32-bit and word
 * aligned; 8-, 16- and 32-bit accesses are allowed.
 *
 * Three conversion groups share the register frame: the Event group (which
 * the interrupt header calls "GROUP0"), Group1 and Group2.  Most group
 * registers come as an EV/G1/G2 triple with a 4-byte stride, so the driver
 * indexes them with the RM57_ADC_*_OFFSET(g) helpers at the end of this
 * block rather than naming all three.
 */

/* Register offsets *********************************************************/

#define RM57_ADC_RSTCR_OFFSET        0x0000 /* Reset control */
#define RM57_ADC_OPMODECR_OFFSET     0x0004 /* Operating mode control */
#define RM57_ADC_CLOCKCR_OFFSET      0x0008 /* Clock control (prescaler) */
#define RM57_ADC_CALCR_OFFSET        0x000c /* Calibration mode control */

#define RM57_ADC_EVMODECR_OFFSET     0x0010 /* Event group operating mode */
#define RM57_ADC_G1MODECR_OFFSET     0x0014 /* Group1 operating mode */
#define RM57_ADC_G2MODECR_OFFSET     0x0018 /* Group2 operating mode */

#define RM57_ADC_EVSRC_OFFSET        0x001c /* Event group trigger source */
#define RM57_ADC_G1SRC_OFFSET        0x0020 /* Group1 trigger source */
#define RM57_ADC_G2SRC_OFFSET        0x0024 /* Group2 trigger source */

#define RM57_ADC_EVINTENA_OFFSET     0x0028 /* Event group interrupt enable */
#define RM57_ADC_G1INTENA_OFFSET     0x002c /* Group1 interrupt enable */
#define RM57_ADC_G2INTENA_OFFSET     0x0030 /* Group2 interrupt enable */

#define RM57_ADC_EVINTFLG_OFFSET     0x0034 /* Event group interrupt flags */
#define RM57_ADC_G1INTFLG_OFFSET     0x0038 /* Group1 interrupt flags */
#define RM57_ADC_G2INTFLG_OFFSET     0x003c /* Group2 interrupt flags */

#define RM57_ADC_EVTHRINTCR_OFFSET   0x0040 /* Event group threshold count */
#define RM57_ADC_G1THRINTCR_OFFSET   0x0044 /* Group1 threshold count */
#define RM57_ADC_G2THRINTCR_OFFSET   0x0048 /* Group2 threshold count */

#define RM57_ADC_EVDMACR_OFFSET      0x004c /* Event group DMA control */
#define RM57_ADC_G1DMACR_OFFSET      0x0050 /* Group1 DMA control */
#define RM57_ADC_G2DMACR_OFFSET      0x0054 /* Group2 DMA control */

#define RM57_ADC_BNDCR_OFFSET        0x0058 /* Results memory boundaries */
#define RM57_ADC_BNDEND_OFFSET       0x005c /* Results memory size */

#define RM57_ADC_EVSAMP_OFFSET       0x0060 /* Event group sample window */
#define RM57_ADC_G1SAMP_OFFSET       0x0064 /* Group1 sample window */
#define RM57_ADC_G2SAMP_OFFSET       0x0068 /* Group2 sample window */

#define RM57_ADC_EVSR_OFFSET         0x006c /* Event group status */
#define RM57_ADC_G1SR_OFFSET         0x0070 /* Group1 status */
#define RM57_ADC_G2SR_OFFSET         0x0074 /* Group2 status */

#define RM57_ADC_EVSEL_OFFSET        0x0078 /* Event group channel select */
#define RM57_ADC_G1SEL_OFFSET        0x007c /* Group1 channel select */
#define RM57_ADC_G2SEL_OFFSET        0x0080 /* Group2 channel select */

#define RM57_ADC_CALR_OFFSET         0x0084 /* Calibration / offset error */
#define RM57_ADC_SMSTATE_OFFSET      0x0088 /* State machine status */
#define RM57_ADC_LASTCONV_OFFSET     0x008c /* Channel last conversion value */

/* Group results FIFOs.  Reading any address inside a group's range pops one
 * conversion result out of that group's results memory.
 */

#define RM57_ADC_EVBUFFER_OFFSET     0x0090 /* Event group FIFO (0x90-0xaf) */
#define RM57_ADC_G1BUFFER_OFFSET     0x00b0 /* Group1 FIFO (0xb0-0xcf) */
#define RM57_ADC_G2BUFFER_OFFSET     0x00d0 /* Group2 FIFO (0xd0-0xef) */

/* Emulation FIFOs return the next result without popping it or disturbing
 * any status flag.
 */

#define RM57_ADC_EVEMUBUFFER_OFFSET  0x00f0 /* Event group emulation FIFO */
#define RM57_ADC_G1EMUBUFFER_OFFSET  0x00f4 /* Group1 emulation FIFO */
#define RM57_ADC_G2EMUBUFFER_OFFSET  0x00f8 /* Group2 emulation FIFO */

/* ADEVT pin (AD1EVT/AD2EVT) general purpose I/O control */

#define RM57_ADC_EVTDIR_OFFSET       0x00fc /* ADEVT direction */
#define RM57_ADC_EVTOUT_OFFSET       0x0100 /* ADEVT output value */
#define RM57_ADC_EVTIN_OFFSET        0x0104 /* ADEVT input value */
#define RM57_ADC_EVTSET_OFFSET       0x0108 /* ADEVT set */
#define RM57_ADC_EVTCLR_OFFSET       0x010c /* ADEVT clear */
#define RM57_ADC_EVTPDR_OFFSET       0x0110 /* ADEVT open drain enable */
#define RM57_ADC_EVTPDIS_OFFSET      0x0114 /* ADEVT pull control disable */
#define RM57_ADC_EVTPSEL_OFFSET      0x0118 /* ADEVT pull select */

#define RM57_ADC_EVSAMPDISEN_OFFSET  0x011c /* Event grp sample cap discharge */
#define RM57_ADC_G1SAMPDISEN_OFFSET  0x0120 /* Group1 sample cap discharge */
#define RM57_ADC_G2SAMPDISEN_OFFSET  0x0124 /* Group2 sample cap discharge */

/* Magnitude compare interrupts.  Three comparators per module; the control
 * and mask registers alternate on an 8-byte stride.
 */

#define RM57_ADC_MAGINT1CR_OFFSET    0x0128 /* Magnitude compare 1 control */
#define RM57_ADC_MAGINT1MASK_OFFSET  0x012c /* Magnitude compare 1 mask */
#define RM57_ADC_MAGINT2CR_OFFSET    0x0130 /* Magnitude compare 2 control */
#define RM57_ADC_MAGINT2MASK_OFFSET  0x0134 /* Magnitude compare 2 mask */
#define RM57_ADC_MAGINT3CR_OFFSET    0x0138 /* Magnitude compare 3 control */
#define RM57_ADC_MAGINT3MASK_OFFSET  0x013c /* Magnitude compare 3 mask */

#define RM57_ADC_MAGINTENASET_OFFSET 0x0158 /* Magnitude int enable set */
#define RM57_ADC_MAGINTENACLR_OFFSET 0x015c /* Magnitude int enable clear */
#define RM57_ADC_MAGINTFLG_OFFSET    0x0160 /* Magnitude int flags */
#define RM57_ADC_MAGINTOFF_OFFSET    0x0164 /* Magnitude int offset (index) */

#define RM57_ADC_EVFIFORESETCR_OFFSET 0x0168 /* Event group FIFO reset */
#define RM57_ADC_G1FIFORESETCR_OFFSET 0x016c /* Group1 FIFO reset */
#define RM57_ADC_G2FIFORESETCR_OFFSET 0x0170 /* Group2 FIFO reset */

#define RM57_ADC_EVRAMWRADDR_OFFSET  0x0174 /* Event group RAM write pointer */
#define RM57_ADC_G1RAMWRADDR_OFFSET  0x0178 /* Group1 RAM write pointer */
#define RM57_ADC_G2RAMWRADDR_OFFSET  0x017c /* Group2 RAM write pointer */

#define RM57_ADC_PARCR_OFFSET        0x0180 /* Parity control */
#define RM57_ADC_PARADDR_OFFSET      0x0184 /* Parity error address */
#define RM57_ADC_PWRUPDLYCTRL_OFFSET 0x0188 /* Power-up delay control */

/* Enhanced channel selection mode (ADC1 only) */

#define RM57_ADC_EVCHNSELMODE_OFFSET 0x0190 /* Event grp channel select mode */
#define RM57_ADC_G1CHNSELMODE_OFFSET 0x0194 /* Group1 channel select mode */
#define RM57_ADC_G2CHNSELMODE_OFFSET 0x0198 /* Group2 channel select mode */

#define RM57_ADC_EVCURRCOUNT_OFFSET  0x019c /* Event group LUT index */
#define RM57_ADC_EVMAXCOUNT_OFFSET   0x01a0 /* Event group LUT max count */
#define RM57_ADC_G1CURRCOUNT_OFFSET  0x01a4 /* Group1 LUT index */
#define RM57_ADC_G1MAXCOUNT_OFFSET   0x01a8 /* Group1 LUT max count */
#define RM57_ADC_G2CURRCOUNT_OFFSET  0x01ac /* Group2 LUT index */
#define RM57_ADC_G2MAXCOUNT_OFFSET   0x01b0 /* Group2 LUT max count */

/* RM57_ADC_GROUP_EV/_G1/_G2 and RM57_ADC_NGROUPS match this EV/G1/G2
 * register ordering; they are part of the application interface, so they are
 * defined in <arch/chip/rm57_adc_ioctl.h> above.
 */

/* Per-group register offset helpers.  Every group register triple starts at
 * the Event group offset and advances 4 bytes per group, except the result
 * FIFOs (0x20 apart) and the LUT count registers (8 apart).
 */

#define RM57_ADC_MODECR_OFFSET(g)    (RM57_ADC_EVMODECR_OFFSET + ((g) << 2))
#define RM57_ADC_SRC_OFFSET(g)       (RM57_ADC_EVSRC_OFFSET + ((g) << 2))
#define RM57_ADC_INTENA_OFFSET(g)    (RM57_ADC_EVINTENA_OFFSET + ((g) << 2))
#define RM57_ADC_INTFLG_OFFSET(g)    (RM57_ADC_EVINTFLG_OFFSET + ((g) << 2))
#define RM57_ADC_THRINTCR_OFFSET(g)  (RM57_ADC_EVTHRINTCR_OFFSET + ((g) << 2))
#define RM57_ADC_DMACR_OFFSET(g)     (RM57_ADC_EVDMACR_OFFSET + ((g) << 2))
#define RM57_ADC_SAMP_OFFSET(g)      (RM57_ADC_EVSAMP_OFFSET + ((g) << 2))
#define RM57_ADC_SR_OFFSET(g)        (RM57_ADC_EVSR_OFFSET + ((g) << 2))
#define RM57_ADC_SEL_OFFSET(g)       (RM57_ADC_EVSEL_OFFSET + ((g) << 2))
#define RM57_ADC_BUFFER_OFFSET(g)    (RM57_ADC_EVBUFFER_OFFSET + ((g) << 5))
#define RM57_ADC_EMUBUFFER_OFFSET(g) (RM57_ADC_EVEMUBUFFER_OFFSET + ((g) << 2))
#define RM57_ADC_SAMPDISEN_OFFSET(g) (RM57_ADC_EVSAMPDISEN_OFFSET + ((g) << 2))
#define RM57_ADC_FIFORESETCR_OFFSET(g) \
                                     (RM57_ADC_EVFIFORESETCR_OFFSET + ((g) << 2))
#define RM57_ADC_RAMWRADDR_OFFSET(g) (RM57_ADC_EVRAMWRADDR_OFFSET + ((g) << 2))
#define RM57_ADC_CHNSELMODE_OFFSET(g) \
                                     (RM57_ADC_EVCHNSELMODE_OFFSET + ((g) << 2))
#define RM57_ADC_CURRCOUNT_OFFSET(g) (RM57_ADC_EVCURRCOUNT_OFFSET + ((g) << 3))
#define RM57_ADC_MAXCOUNT_OFFSET(g)  (RM57_ADC_EVMAXCOUNT_OFFSET + ((g) << 3))

/* Magnitude compare registers are indexed 0-2 on an 8-byte stride; the count
 * itself is RM57_ADC_NMAGCOMP in the exported header.
 */

#define RM57_ADC_MAGINTCR_OFFSET(n)  (RM57_ADC_MAGINT1CR_OFFSET + ((n) << 3))
#define RM57_ADC_MAGINTMASK_OFFSET(n) \
                                     (RM57_ADC_MAGINT1MASK_OFFSET + ((n) << 3))

/* Register bit definitions *************************************************/

/* ADC Reset Control Register (ADRSTCR), offset 0x0000.  Privileged write. */

#define ADC_RSTCR_RESET              (1 << 0)  /* Bit 0:  Module reset */

/* ADC Operating Mode Control Register (ADOPMODECR), offset 0x0004 */

#define ADC_OPMODECR_ADC_EN          (1 << 0)  /* Bit 0:  ADC enable */
#define ADC_OPMODECR_IDLE_PWRDN      (1 << 4)  /* Bit 4:  Power down when idle */
#define ADC_OPMODECR_POWERDOWN       (1 << 8)  /* Bit 8:  ADC core power down */
#define ADC_OPMODECR_RAM_TEST_EN     (1 << 16) /* Bit 16: Results RAM test mode */
#define ADC_OPMODECR_CHN_TEST_SHIFT  (17)      /* Bits 17-20: Channel impedance test */
#define ADC_OPMODECR_CHN_TEST_MASK   (15 << ADC_OPMODECR_CHN_TEST_SHIFT)
#  define ADC_OPMODECR_CHN_TEST_DIS  (10 << ADC_OPMODECR_CHN_TEST_SHIFT)
#  define ADC_OPMODECR_CHN_TEST_ENA  (5 << ADC_OPMODECR_CHN_TEST_SHIFT)
#define ADC_OPMODECR_COS             (1 << 24) /* Bit 24: Continue on suspend */
#define ADC_OPMODECR_12BIT           (1 << 31) /* Bit 31: 0=10-bit, 1=12-bit */

/* ADC Clock Control Register (ADCLOCKCR), offset 0x0008.
 * ADCLK period = VCLK period x (PS + 1).
 */

#define ADC_CLOCKCR_PS_SHIFT         (0)       /* Bits 0-4: Clock prescaler */
#define ADC_CLOCKCR_PS_MASK          (31 << ADC_CLOCKCR_PS_SHIFT)
#  define ADC_CLOCKCR_PS(n)          ((uint32_t)(n) << ADC_CLOCKCR_PS_SHIFT)

/* ADC Calibration Mode Control Register (ADCALCR), offset 0x000c */

#define ADC_CALCR_CAL_EN             (1 << 0)  /* Bit 0:  Calibration enable */
#define ADC_CALCR_HILO               (1 << 8)  /* Bit 8:  Reference voltage select */
#define ADC_CALCR_BRIDGE_EN          (1 << 9)  /* Bit 9:  Bridge enable */
#define ADC_CALCR_CAL_ST             (1 << 16) /* Bit 16: Start; self-clears */
#define ADC_CALCR_SELF_TEST          (1 << 24) /* Bit 24: Self test enable */

/* Calibration reference voltage selection (TRM Table 22-2).  Valid only
 * while CAL_EN is set and SELF_TEST is clear.
 */

#define ADC_CALCR_REF_MIDHI          (0)                  /* (HI*R1+LO*R2)/(R1+R2) */
#define ADC_CALCR_REF_MIDLO          (ADC_CALCR_HILO)     /* (LO*R1+HI*R2)/(R1+R2) */
#define ADC_CALCR_REF_LO             (ADC_CALCR_BRIDGE_EN)
#define ADC_CALCR_REF_HI             (ADC_CALCR_BRIDGE_EN | ADC_CALCR_HILO)

/* Group Operating Mode Control Registers (ADEVMODECR/ADG1MODECR/ADG2MODECR),
 * offsets 0x0010/0x0014/0x0018.  DATA_FMT is 12-bit-mode only; 8BIT is
 * 10-bit-mode only.
 *
 * TRM Table 22-12 gives no bit numbers for these fields, because the layout
 * differs between the 10- and 12-bit configurations, so the positions come
 * from Figures 22-24 to 22-27.  Bit 3 is HW_TRIG for Group1 and Group2 but
 * reserved for the Event group, which is always started by its trigger
 * source.
 */

#define ADC_MODECR_FRZ               (1 << 0)  /* Bit 0:  Freeze enable */
#define ADC_MODECR_MODE_CONT         (1 << 1)  /* Bit 1:  Continuous conversion */
#define ADC_MODECR_8BIT              (1 << 2)  /* Bit 2:  8-bit read (10-bit mode) */
#define ADC_MODECR_HW_TRIG           (1 << 3)  /* Bit 3:  Hardware triggered */
#define ADC_MODECR_OVR_RAM_IGN       (1 << 4)  /* Bit 4:  Overwrite on overrun */
#define ADC_MODECR_CHID              (1 << 5)  /* Bit 5:  Channel id in FIFO read */
#define ADC_MODECR_DATA_FMT_SHIFT    (8)       /* Bits 8-9: Read data format */
#define ADC_MODECR_DATA_FMT_MASK     (3 << ADC_MODECR_DATA_FMT_SHIFT)
#  define ADC_MODECR_DATA_FMT_12BIT  (0 << ADC_MODECR_DATA_FMT_SHIFT)
#  define ADC_MODECR_DATA_FMT_10BIT  (1 << ADC_MODECR_DATA_FMT_SHIFT)
#  define ADC_MODECR_DATA_FMT_8BIT   (2 << ADC_MODECR_DATA_FMT_SHIFT)
#define ADC_MODECR_NORESET_CHNSEL    (1 << 16) /* Bit 16: No RAM reset on chnsel */

/* Group Trigger Source Select Registers (ADEVSRC/ADG1SRC/ADG2SRC),
 * offsets 0x001c/0x0020/0x0024.
 */

#define ADC_SRC_SRC_SHIFT            (0)       /* Bits 0-2: Trigger source */
#define ADC_SRC_SRC_MASK             (7 << ADC_SRC_SRC_SHIFT)
#  define ADC_SRC_SRC(n)             ((uint32_t)(n) << ADC_SRC_SRC_SHIFT)
#define ADC_SRC_EDG_SEL              (1 << 3)  /* Bit 3:  1=rising, 0=falling */
#define ADC_SRC_EDG_BOTH             (1 << 4)  /* Bit 4:  Trigger on both edges */

/* The ADC_TRIGSRC_* source values that go in this field are part of the
 * application interface and live in <arch/chip/rm57_adc_ioctl.h>.
 */

/* Group Interrupt Enable Registers (ADEVINTENA/ADG1INTENA/ADG2INTENA),
 * offsets 0x0028/0x002c/0x0030.
 */

#define ADC_INTENA_THR               (1 << 0)  /* Bit 0:  Threshold interrupt */
#define ADC_INTENA_OVR               (1 << 1)  /* Bit 1:  Memory overrun interrupt */
#define ADC_INTENA_END               (1 << 3)  /* Bit 3:  Conversion end interrupt */

#define ADC_INTENA_ALL               (ADC_INTENA_THR | ADC_INTENA_OVR | \
                                      ADC_INTENA_END)

/* Group Interrupt Flag Registers (ADEVINTFLG/ADG1INTFLG/ADG2INTFLG),
 * offsets 0x0034/0x0038/0x003c.  THR and END are write-1-to-clear; EMPTY
 * and OVERRUN are read-only status.
 */

#define ADC_INTFLG_THR               (1 << 0)  /* Bit 0:  Threshold reached (W1C) */
#define ADC_INTFLG_MEM_OVERRUN       (1 << 1)  /* Bit 1:  Memory overrun (RO) */
#define ADC_INTFLG_MEM_EMPTY         (1 << 2)  /* Bit 2:  Memory empty (RO) */
#define ADC_INTFLG_END               (1 << 3)  /* Bit 3:  Conversion end (W1C) */

#define ADC_INTFLG_CLRALL            (ADC_INTFLG_THR | ADC_INTFLG_END)

/* Group Threshold Interrupt Control Registers
 * (ADEVTHRINTCR/ADG1THRINTCR/ADG2THRINTCR), offsets 0x0040/0x0044/0x0048.
 * The threshold is a signed 9-bit down-counter that may go negative.
 */

#define ADC_THRINTCR_THR_SHIFT       (0)       /* Bits 0-8: Threshold counter */
#define ADC_THRINTCR_THR_MASK        (0x1ff << ADC_THRINTCR_THR_SHIFT)
#  define ADC_THRINTCR_THR(n)        ((uint32_t)(n) << ADC_THRINTCR_THR_SHIFT)

/* Group DMA Control Registers (ADEVDMACR/ADG1DMACR/ADG2DMACR),
 * offsets 0x004c/0x0050/0x0054.
 */

#define ADC_DMACR_DMA_EN             (1 << 0)  /* Bit 0:  DMA request enable */
#define ADC_DMACR_BLK_XFER           (1 << 2)  /* Bit 2:  Block transfer mode */
#define ADC_DMACR_DMA_END            (1 << 3)  /* Bit 3:  Request on group end */
#define ADC_DMACR_BLOCKS_SHIFT       (16)      /* Bits 16-24: Block size */
#define ADC_DMACR_BLOCKS_MASK        (0x1ff << ADC_DMACR_BLOCKS_SHIFT)
#  define ADC_DMACR_BLOCKS(n)        ((uint32_t)(n) << ADC_DMACR_BLOCKS_SHIFT)

/* ADC Results Memory Configuration Register (ADBNDCR), offset 0x0058.
 * Boundaries are expressed in *pairs* of result buffers: the Event group
 * gets 2 x BNDA buffers and Group1 gets 2 x (BNDB - BNDA); Group2 takes the
 * remainder up to the size selected by ADBNDEND.
 */

#define ADC_BNDCR_BNDB_SHIFT         (0)       /* Bits 0-8: Event+Group1 pairs */
#define ADC_BNDCR_BNDB_MASK          (0x1ff << ADC_BNDCR_BNDB_SHIFT)
#  define ADC_BNDCR_BNDB(n)          ((uint32_t)(n) << ADC_BNDCR_BNDB_SHIFT)
#define ADC_BNDCR_BNDA_SHIFT         (16)      /* Bits 16-24: Event group pairs */
#define ADC_BNDCR_BNDA_MASK          (0x1ff << ADC_BNDCR_BNDA_SHIFT)
#  define ADC_BNDCR_BNDA(n)          ((uint32_t)(n) << ADC_BNDCR_BNDA_SHIFT)

/* ADC Results Memory Size Configuration Register (ADBNDEND), offset 0x005c */

#define ADC_BNDEND_SHIFT             (0)       /* Bits 0-2: Memory size select */
#define ADC_BNDEND_MASK              (7 << ADC_BNDEND_SHIFT)
#  define ADC_BNDEND_64WORDS         (2 << ADC_BNDEND_SHIFT)
#define ADC_BNDEND_BUF_INIT_ACTIVE   (1 << 16) /* Bit 16: Auto-init in progress */

/* Total number of conversion result buffers in the ADC results RAM */

#define RM57_ADC_NRESULTBUFS         64

/* Group Sampling Time Configuration Registers (ADEVSAMP/ADG1SAMP/ADG2SAMP),
 * offsets 0x0060/0x0064/0x0068.  Sampling window = ACQ + 2 ADCLK cycles and
 * must be at least 3 ADCLK cycles.
 */

#define ADC_SAMP_ACQ_SHIFT           (0)       /* Bits 0-11: Acquisition time */
#define ADC_SAMP_ACQ_MASK            (0xfff << ADC_SAMP_ACQ_SHIFT)
#  define ADC_SAMP_ACQ(n)            ((uint32_t)(n) << ADC_SAMP_ACQ_SHIFT)

#define ADC_SAMP_ACQ_MIN             1         /* SW = ACQ + 2 >= 3 */

/* Group Status Registers (ADEVSR/ADG1SR/ADG2SR),
 * offsets 0x006c/0x0070/0x0074.
 */

#define ADC_SR_END                   (1 << 0)  /* Bit 0:  Conversion end (W1C) */
#define ADC_SR_STOP                  (1 << 1)  /* Bit 1:  Conversions frozen */
#define ADC_SR_BUSY                  (1 << 2)  /* Bit 2:  Conversion in progress */
#define ADC_SR_MEM_EMPTY             (1 << 3)  /* Bit 3:  Results memory empty */

/* ADC Calibration and Error Offset Correction Register (ADCALR),
 * offset 0x0084.  12 bits wide in 12-bit mode, 10 bits in 10-bit mode.
 */

#define ADC_CALR_12BIT_MASK          (0xfff)
#define ADC_CALR_10BIT_MASK          (0x3ff)

/* ADC State Machine Status Register (ADSMSTATE), offset 0x0088 */

#define ADC_SMSTATE_SHIFT            (0)       /* Bits 0-3: Current state */
#define ADC_SMSTATE_MASK             (15 << ADC_SMSTATE_SHIFT)

/* Conversion result format when read from a group FIFO (ADxxBUFFER).  The
 * layout differs between 12- and 10-bit module configurations (TRM Figures
 * 22-4 and 22-5).
 */

#define ADC_FIFO_12BIT_DR_MASK       (0xfff)   /* Bits 0-11:  Result */
#define ADC_FIFO_12BIT_CHID_SHIFT    (16)      /* Bits 16-20: Channel id */
#define ADC_FIFO_12BIT_CHID_MASK     (0x1f << ADC_FIFO_12BIT_CHID_SHIFT)
#define ADC_FIFO_12BIT_EMPTY         (1 << 31) /* Bit 31: FIFO was empty */

#define ADC_FIFO_10BIT_DR_MASK       (0x3ff)   /* Bits 0-9:   Result */
#define ADC_FIFO_10BIT_CHID_SHIFT    (10)      /* Bits 10-14: Channel id */
#define ADC_FIFO_10BIT_CHID_MASK     (0x1f << ADC_FIFO_10BIT_CHID_SHIFT)

/* The empty flag moves with the rest of the word: it is the top bit of the
 * 32-bit result in 12-bit mode but the top bit of the 16-bit result in
 * 10-bit mode, where bits 31-16 read as zero (TRM Figures 22-60 and 22-61).
 * Testing bit 31 unconditionally would therefore never see an empty FIFO on
 * a 10-bit module.
 */

#define ADC_FIFO_10BIT_EMPTY         (1 << 15) /* Bit 15: FIFO was empty */

/* ADEVT pin control registers, offsets 0x00fc-0x0118.  Each holds a single
 * bit 0 controlling the module's ADEVT pin.
 */

#define ADC_EVT_BIT                  (1 << 0)  /* Bit 0: ADEVT pin */

/* Group Sample Cap Discharge Control Registers
 * (ADEVSAMPDISEN/ADG1SAMPDISEN/ADG2SAMPDISEN), offsets 0x011c/0x0120/0x0124.
 */

#define ADC_SAMPDISEN_EN             (1 << 0)  /* Bit 0: Discharge enable */
#define ADC_SAMPDISEN_CYC_SHIFT      (8)       /* Bits 8-15: Discharge cycles */
#define ADC_SAMPDISEN_CYC_MASK       (0xff << ADC_SAMPDISEN_CYC_SHIFT)
#  define ADC_SAMPDISEN_CYC(n)       ((uint32_t)(n) << ADC_SAMPDISEN_CYC_SHIFT)

/* Magnitude Compare Interrupt Control Registers (ADMAGINTxCR), offsets
 * 0x0128/0x0130/0x0138.  The field layout differs between 12- and 10-bit
 * module configurations (TRM Figures 22-83 and 22-84).
 */

#define ADC_MAGINTCR12_MAG_CHID_SHIFT  (0)       /* Bits 0-4: Monitored chan */
#define ADC_MAGINTCR12_MAG_CHID_MASK   (0x1f << ADC_MAGINTCR12_MAG_CHID_SHIFT)
#define ADC_MAGINTCR12_COMP_CHID_SHIFT (8)       /* Bits 8-12: Compare chan */
#define ADC_MAGINTCR12_COMP_CHID_MASK  (0x1f << ADC_MAGINTCR12_COMP_CHID_SHIFT)
#define ADC_MAGINTCR12_CMP_GE_LT       (1 << 14) /* Bit 14: 1=>=, 0=< */
#define ADC_MAGINTCR12_CHN_THR_COMP    (1 << 15) /* Bit 15: Compare vs channel */
#define ADC_MAGINTCR12_MAG_THR_SHIFT   (16)      /* Bits 16-27: Threshold */
#define ADC_MAGINTCR12_MAG_THR_MASK    (0xfff << ADC_MAGINTCR12_MAG_THR_SHIFT)

#define ADC_MAGINTCR10_CMP_GE_LT       (1 << 0)  /* Bit 0:  1=>=, 0=< */
#define ADC_MAGINTCR10_CHN_THR_COMP    (1 << 1)  /* Bit 1:  Compare vs channel */
#define ADC_MAGINTCR10_COMP_CHID_SHIFT (8)       /* Bits 8-12: Compare chan */
#define ADC_MAGINTCR10_COMP_CHID_MASK  (0x1f << ADC_MAGINTCR10_COMP_CHID_SHIFT)
#define ADC_MAGINTCR10_MAG_THR_SHIFT   (16)      /* Bits 16-25: Threshold */
#define ADC_MAGINTCR10_MAG_THR_MASK    (0x3ff << ADC_MAGINTCR10_MAG_THR_SHIFT)
#define ADC_MAGINTCR10_MAG_CHID_SHIFT  (26)      /* Bits 26-30: Monitored ch */
#define ADC_MAGINTCR10_MAG_CHID_MASK   (0x1f << ADC_MAGINTCR10_MAG_CHID_SHIFT)

/* Magnitude Compare Interrupt Enable Set/Clear, Flag and Offset registers,
 * offsets 0x0158/0x015c/0x0160/0x0164.  One bit per comparator.
 */

#define ADC_MAGINT(n)                (1 << (n)) /* Comparator n, 0-2 */
#define ADC_MAGINT_ALL               (7)

#define ADC_MAGINTOFF_SHIFT          (0)       /* Bits 0-3: Highest priority */
#define ADC_MAGINTOFF_MASK           (15 << ADC_MAGINTOFF_SHIFT)

/* Group FIFO Reset Control Registers, offsets 0x0168/0x016c/0x0170.  The bit
 * self-clears and always reads as zero.
 */

#define ADC_FIFORESETCR_RESET        (1 << 0)  /* Bit 0: Reset group FIFO */

/* Group RAM Write Address Registers, offsets 0x0174/0x0178/0x017c.  Holds
 * the buffer number the next conversion result will be written to.
 */

#define ADC_RAMWRADDR_SHIFT          (0)       /* Bits 0-8: Buffer number */
#define ADC_RAMWRADDR_MASK           (0x1ff << ADC_RAMWRADDR_SHIFT)

/* ADC Parity Control Register (ADPARCR), offset 0x0180.  Privileged write.
 * PARITY_ENA is inverted logic: 5h disables the check, every other value
 * enables it.  Reset value is 5h, i.e. parity checking off.
 */

#define ADC_PARCR_PARITY_SHIFT       (0)       /* Bits 0-3: Parity enable */
#define ADC_PARCR_PARITY_MASK        (15 << ADC_PARCR_PARITY_SHIFT)
#  define ADC_PARCR_PARITY_DISABLE   (5 << ADC_PARCR_PARITY_SHIFT)
#  define ADC_PARCR_PARITY_ENABLE    (10 << ADC_PARCR_PARITY_SHIFT)
#define ADC_PARCR_TEST               (1 << 8)  /* Bit 8: Map parity bits */

/* ADC Power-Up Delay Control Register (ADPWRUPDLYCTRL), offset 0x0188 */

#define ADC_PWRUPDLY_SHIFT           (0)       /* Bits 0-9: VCLK cycles */
#define ADC_PWRUPDLY_MASK            (0x3ff << ADC_PWRUPDLY_SHIFT)
#  define ADC_PWRUPDLY(n)            ((uint32_t)(n) << ADC_PWRUPDLY_SHIFT)

/* Group Channel Selection Mode Control Registers, offsets
 * 0x0190/0x0194/0x0198.  Enhanced mode is implemented on ADC1 only.
 */

#define ADC_CHNSELMODE_SHIFT         (0)       /* Bits 0-3: Enhanced mode */
#define ADC_CHNSELMODE_MASK          (15 << ADC_CHNSELMODE_SHIFT)
#  define ADC_CHNSELMODE_SEQUENTIAL  (5 << ADC_CHNSELMODE_SHIFT)
#  define ADC_CHNSELMODE_ENHANCED    (10 << ADC_CHNSELMODE_SHIFT)

/* Group LUT index and maximum count registers, offsets 0x019c-0x01b0 */

#define ADC_COUNT_SHIFT              (0)       /* Bits 0-4: LUT index/limit */
#define ADC_COUNT_MASK               (0x1f << ADC_COUNT_SHIFT)

/* Enhanced channel selection look-up table.  Each conversion group owns 32
 * word-aligned entries; the Event group's table comes first, then Group1's,
 * then Group2's (TRM Figure 22-11).
 */

#define RM57_ADC_LUT_GROUP_OFFSET(g) ((g) * RM57_ADC_LUT_NENTRIES * 4)

#define ADC_LUT_INT_CHN_SHIFT        (0)       /* Bits 0-4: Internal mux select */
#define ADC_LUT_INT_CHN_MASK         (0x1f << ADC_LUT_INT_CHN_SHIFT)
#  define ADC_LUT_INT_CHN(n)         ((uint32_t)(n) << ADC_LUT_INT_CHN_SHIFT)
#define ADC_LUT_EXT_CHN_SHIFT        (8)       /* Bits 8-12: External mux select */
#define ADC_LUT_EXT_CHN_MASK         (0x1f << ADC_LUT_EXT_CHN_SHIFT)
#  define ADC_LUT_EXT_CHN(n)         ((uint32_t)(n) << ADC_LUT_EXT_CHN_SHIFT)

/* Channel counts.  ADC1 has 32 inputs; ADC2 has 25, of which 16 are shared
 * with ADC1 (TRM Figure 22-1).
 */

#define RM57_ADC1_NCHANNELS          32
#define RM57_ADC2_NCHANNELS          25

/* Selectable channels, as a mask rather than a count.  ADC2 needs this
 * because its channel select register is not simply the low 25 bits: the top
 * two bits address the on-die temperature sensors, which have no external
 * ball at all.  ADC1's channel 31 doubles as temperature sensor 1 and is a
 * real ball (T13), selected between the two by the IOMM.
 */

#define RM57_ADC1_VALIDMASK          0xffffffff
#define RM57_ADC2_VALIDMASK          0xc1ffffff

/* On-die temperature sensors (TRM SPNU562A section 6.5.13).  Three sensors,
 * each multiplexed onto one ADC channel.  Selecting the sensor rather than
 * the ball, and releasing all three from power down, is done through the
 * IOMM - see the PINMUX_TEMP* entries in rm57l843_pinmux.h.
 */

#define RM57_ADC_TEMP1_MODULE        0   /* ADC1 */
#define RM57_ADC_TEMP1_CHANNEL       31
#define RM57_ADC_TEMP2_MODULE        1   /* ADC2 */
#define RM57_ADC_TEMP2_CHANNEL       31
#define RM57_ADC_TEMP3_MODULE        1   /* ADC2 */
#define RM57_ADC_TEMP3_CHANNEL       30

/* Factory calibration for the temperature sensors, recorded in TI OTP bank 0
 * during device test (TRM SPNU562A section 7.5.2.5).  Each sensor gets three
 * (temperature, ADC reading) points, every field 16 bits wide, measured with
 * ADREFHI at 3.3V, and each point is one 32-bit word.
 *
 * Two things about the layout differ from what TRM Figure 7-7 suggests, and
 * both were established by reading a real device rather than deduced:
 *
 *   - The temperature is the *low* half of the word and the ADC reading the
 *     high half, the opposite way round from the figure's byte assignment.
 *     Most likely the image is laid out for the big-endian configuration
 *     TI's own tools default to; whatever the reason, a little-endian
 *     32-bit load yields the two fields in this order.
 *   - The three points are not in temperature order.  Silicon records them
 *     as +30C, -40C and +125C, in that sequence, so they have to be sorted
 *     before anything can interpolate between them.
 *
 * For reference, one RM57L843 reads (Kelvin, code): sensor 1 (303, 1879),
 * (233, 1427), (398, 2461); sensors 2 and 3 the same temperatures with their
 * own codes.  That is about 6 codes per degree, rising with temperature.
 *
 * Reading OTP is a plain load - there is no flash mode to enter - but the
 * region must not be mapped as "normal" memory or errata L2FMC#5 returns the
 * instruction opcode instead of the data.  rm57_periph_region() maps this
 * address range strongly-ordered, so the workaround is already in place.
 */

#define RM57_ADC_TEMPCAL_BASE(n)     (0xf0080310 + ((n) * 0x10))

#define RM57_ADC_TEMPCAL_KELVIN_MASK  (0xffff)  /* Bits 0-15:  Kelvin */
#define RM57_ADC_TEMPCAL_VAL_SHIFT    (16)      /* Bits 16-31: ADC reading */
#define RM57_ADC_TEMPCAL_VAL_MASK     (0xffff << RM57_ADC_TEMPCAL_VAL_SHIFT)

#define RM57_ADC_TEMPCAL_BLANK       0xffff /* Unprogrammed OTP reads as this */

/* Plausibility bounds on a calibration temperature.  The device is tested
 * between -40C and 125C, so anything far outside that is not a temperature -
 * which is the signature of OTP being read through errata L2FMC#5.
 */

#define RM57_ADC_TEMPCAL_MINK        200   /* -73C */
#define RM57_ADC_TEMPCAL_MAXK        500   /* 227C */

/* Absolute zero in milli-Celsius, for the Kelvin to Celsius conversion */

#define RM57_ADC_ZEROC_MILLIK        273150

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_ADC_H */
