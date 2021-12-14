/****************************************************************************
 * drivers/sensors/ad5940_def.h
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

#ifndef __DRIVERS_SENSORS_AD5940_DEF_H
#define __DRIVERS_SENSORS_AD5940_DEF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Part1. Registers addresses, bit positions and masks, and enumerations */

/* Part1.1 AGPIO registers (16bits) */

/* AGPIO GPIO0 Configuration register address, bit positions and masks */

#define AD5940_REG_AGPIO_GP0CON           0x0000 /* Address */
#define AD5940_REG_AGPIO_GP0CON_RST       0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0CON_PIN7CFG  14     /* P0.7 cfg bits position */
#define AD5940_BITP_AGPIO_GP0CON_PIN6CFG  12     /* P0.6 cfg bits position */
#define AD5940_BITP_AGPIO_GP0CON_PIN5CFG  10     /* P0.5 cfg bits position */
#define AD5940_BITP_AGPIO_GP0CON_PIN4CFG  8      /* P0.4 cfg bits position */
#define AD5940_BITP_AGPIO_GP0CON_PIN3CFG  6      /* P0.3 cfg bits position */
#define AD5940_BITP_AGPIO_GP0CON_PIN2CFG  4      /* P0.2 cfg bits position */
#define AD5940_BITP_AGPIO_GP0CON_PIN1CFG  2      /* P0.1 cfg bits position */
#define AD5940_BITP_AGPIO_GP0CON_PIN0CFG  0      /* P0.0 cfg bits position */
#define AD5940_BITM_AGPIO_GP0CON_PIN7CFG  0xc000 /* P0.7 cfg bits mask */
#define AD5940_BITM_AGPIO_GP0CON_PIN6CFG  0x3000 /* P0.6 cfg bits mask */
#define AD5940_BITM_AGPIO_GP0CON_PIN5CFG  0x0c00 /* P0.5 cfg bits mask */
#define AD5940_BITM_AGPIO_GP0CON_PIN4CFG  0x0300 /* P0.4 cfg bits mask */
#define AD5940_BITM_AGPIO_GP0CON_PIN3CFG  0x00c0 /* P0.3 cfg bits mask */
#define AD5940_BITM_AGPIO_GP0CON_PIN2CFG  0x0030 /* P0.2 cfg bits mask */
#define AD5940_BITM_AGPIO_GP0CON_PIN1CFG  0x000c /* P0.1 cfg bits mask */
#define AD5940_BITM_AGPIO_GP0CON_PIN0CFG  0x0003 /* P0.0 cfg bits mask */

/* AGPIO GPIO0 Output Enable register address, bit positions and masks */

#define AD5940_REG_AGPIO_GP0OEN           0x0004 /* Address */
#define AD5940_REG_AGPIO_GP0OEN_RST       0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0OEN_OEN      0      /* POS: output enable */
#define AD5940_BITM_AGPIO_GP0OEN_OEN      0x00ff /* Mask: output enable */

/* AGPIO GPIO0 Pullup/down Enable register addr, bit positions and masks */

#define AD5940_REG_AGPIO_GP0PE            0x0008 /* Address */
#define AD5940_REG_AGPIO_GP0PE_RST        0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0PE_PE        0      /* POS: pin pull enable */
#define AD5940_BITM_AGPIO_GP0PE_PE        0x00ff /* Mask: pin pull enable */

/* AGPIO GPIO0 Input Path Enable register address, bit positions and masks */

#define AD5940_REG_AGPIO_GP0IEN           0x000c /* Address */
#define AD5940_REG_AGPIO_GP0IEN_RST       0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0IEN_IEN      0      /* POS: input enable */
#define AD5940_BITM_AGPIO_GP0IEN_IEN      0x00ff /* Mask: input enable */

/* AGPIO GPIO0 Registered Data Input register addr, bit positions and masks */

#define AD5940_REG_AGPIO_GP0IN            0x0010 /* Address */
#define AD5940_REG_AGPIO_GP0IN_RST        0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0IN_IN        0      /* POS: data input */
#define AD5940_BITM_AGPIO_GP0IN_IN        0x00ff /* Mask: data input */

/* AGPIO GPIO0 Data Output register address, bit positions and masks */

#define AD5940_REG_AGPIO_GP0OUT           0x0014 /* Address */
#define AD5940_REG_AGPIO_GP0OUT_RST       0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0OUT_OUT      0      /* POS: data out */
#define AD5940_BITM_AGPIO_GP0OUT_OUT      0x00ff /* Mask: data out */

/* AGPIO GPIO0 Data Out Set register address, bit positions and masks */

#define AD5940_REG_AGPIO_GP0SET           0x0018 /* Address */
#define AD5940_REG_AGPIO_GP0SET_RST       0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0SET_SET      0      /* POS: set the output 1 */
#define AD5940_BITM_AGPIO_GP0SET_SET      0x00ff /* Mask: set the output 1 */

/* AGPIO GPIO0 Data Out Clear register address, bit positions and masks */

#define AD5940_REG_AGPIO_GP0CLR           0x001c /* Address */
#define AD5940_REG_AGPIO_GP0CLR_RST       0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0CLR_CLR      0      /* POS: set the output 0 */
#define AD5940_BITM_AGPIO_GP0CLR_CLR      0x00ff /* Mask: set the output 0 */

/* AGPIO GPIO0 Pin Toggle register address, bit positions and masks */

#define AD5940_REG_AGPIO_GP0TGL           0x0020 /* Address */
#define AD5940_REG_AGPIO_GP0TGL_RST       0x0000 /* Reset value */
#define AD5940_BITP_AGPIO_GP0TGL_TGL      0      /* POS: toggle output */
#define AD5940_BITM_AGPIO_GP0TGL_TGL      0x00ff /* Mask: toggle output */

/* Part1.2 AFECON registers (16bit) */

/* AFECON ADI identification register address and reset value */

#define AD5940_REG_AFECON_ADIID           0x0400 /* Address */
#define AD5940_ADIID                      0x4144 /* Fixed value */

/* AFECON Chip identification register address, bit positions and masks */

#define AD5940_REG_AFECON_CHIPID          0x0404 /* Address */
#define AD5940_CHIPID                     0x5502 /* Read-only value */
#define AD5940_BITP_CHIPID_PARTID         4      /* POS: part Identifier */
#define AD5940_BITP_CHIPID_REVISION       0      /* POS: silicon revision */
#define AD5940_BITM_CHIPID_PARTID         0xfff0 /* Mask: part Identifier */
#define AD5940_BITM_CHIPID_REVISION       0x000f /* Mask: silicon revision */

/* AFECON Clock Divider Config register address, bit positions and masks */

#define AD5940_REG_AFECON_CLKCON0                0x0408 /* Address */
#define AD5940_REG_AFECON_CLKCON0_RESET          0x0441 /* Reset value */
#define AD5940_BITP_AFECON_CLKCON0_SFFTCLKDIVCNT 10     /* POS: SFFT CLK */
#define AD5940_BITP_AFECON_CLKCON0_ADCCLKDIV     6      /* POS: ADC CLK */
#define AD5940_BITP_AFECON_CLKCON0_SYSCLKDIV     0      /* POS: SYS CLK*/
#define AD5940_BITM_AFECON_CLKCON0_SFFTCLKDIVCNT 0xfc00 /* Mask: SFFT CLK */
#define AD5940_BITM_AFECON_CLKCON0_ADCCLKDIV     0x03c0 /* Mask: ADC CLK */
#define AD5940_BITM_AFECON_CLKCON0_SYSCLKDIV     0x003f /* Mask: SYS CLK */

/* AFECON Clock Gate Enable register address, bit positions and masks */

#define AD5940_REG_AFECON_CLKEN1                 0x0410 /* Address */
#define AD5940_REG_AFECON_CLKEN1_RESET           0x02c0 /* Reset value */
#define AD5940_BITP_AFECON_CLKEN1_GPT1DIS        7      /* POS: GPT1 CLK */
#define AD5940_BITP_AFECON_CLKEN1_GPT0DIS        6      /* POS: GPT0 CLK */
#define AD5940_BITP_AFECON_CLKEN1_ACLKDIS        5      /* POS: ACLK CLK */
#define AD5940_BITM_AFECON_CLKEN1_GPT1DIS        0x0080 /* Mask: GPT1 CLK */
#define AD5940_BITM_AFECON_CLKEN1_GPT0DIS        0x0040 /* Mask: GPT0 CLK */
#define AD5940_BITM_AFECON_CLKEN1_ACLKDIS        0x0020 /* Mask: ACLK CLK */

/* AFECON Clock Select register address, bit positions and masks */

#define AD5940_REG_AFECON_CLKSEL                 0x0414 /* Address */
#define AD5940_REG_AFECON_CLKSEL_RESET           0x0000 /* Reset value */
#define AD5940_BITP_AFECON_CLKSEL_ADCCLKSEL      2      /* POS: ADC CLK */
#define AD5940_BITP_AFECON_CLKSEL_SYSCLKSEL      0      /* POS: System CLK */
#define AD5940_BITM_AFECON_CLKSEL_ADCCLKSEL      0x000c /* Mask: ADC CLK */
#define AD5940_BITM_AFECON_CLKSEL_SYSCLKSEL      0x0003 /* Mask: System CLK */

/* AFECON Key protection for CLKCON0 register address, bit positions and
 * masks. Writing correct value to this register enables clock division to
 * 8Mhz, 4Mhz and 2Mhz.
 */

#define AD5940_REG_AFECON_CLKCON0KEY             0x0420 /* Address */
#define AD5940_REG_AFECON_CLKCON0KEY_RESET       0x0000 /* Reset VAL */
#define AD5940_BITP_AFECON_CLKCON0KEY_ULP_EN     0      /* Position */
#define AD5940_BITM_AFECON_CLKCON0KEY_ULP_EN     0xffff /* Mask */

/* AFECON Software Reset register address, bit positions and masks */

#define AD5940_REG_AFECON_SWRSTCON               0x0424 /* Address */
#define AD5940_REG_AFECON_SWRSTCON_RESET         0x0001 /* RST value */
#define AD5940_BITP_AFECON_SWRSTCON_SWRSTL       0      /* Softreset */
#define AD5940_BITM_AFECON_SWRSTCON_SWRSTL       0xffff /* Softreset */

/* AFECON Trigger Sequence register address, bit positions and masks */

#define AD5940_REG_AFECON_TRIGSEQ                0x0430 /* Address */
#define AD5940_REG_AFECON_TRIGSEQ_RESET          0x0000 /* Reset value */
#define AD5940_BITP_AFECON_TRIGSEQ_TRIG3         3      /* Trigger seq 3 */
#define AD5940_BITP_AFECON_TRIGSEQ_TRIG2         2      /* Trigger seq 2 */
#define AD5940_BITP_AFECON_TRIGSEQ_TRIG1         1      /* Trigger seq 1 */
#define AD5940_BITP_AFECON_TRIGSEQ_TRIG0         0      /* Trigger seq 0 */
#define AD5940_BITM_AFECON_TRIGSEQ_TRIG3         0x0008 /* Trigger seq 3 */
#define AD5940_BITM_AFECON_TRIGSEQ_TRIG2         0x0004 /* Trigger seq 2 */
#define AD5940_BITM_AFECON_TRIGSEQ_TRIG1         0x0002 /* Trigger seq 1 */
#define AD5940_BITM_AFECON_TRIGSEQ_TRIG0         0x0001 /* Trigger seq 0 */

/* Part1.3 AFEWDT registers (16bit) */

/* AFEWDT Watchdog Timer Load Value register addr, bit positions and masks */

#define AD5940_REG_AFEWDT_WDTLD                  0x0900 /* Address */
#define AD5940_BITP_AFEWDT_WDTLD_LOAD            0      /* WDT load value */
#define AD5940_BITM_AFEWDT_WDTLD_LOAD            0xffff /* WDT load value */

/* AFEWDT Current Count Value register address, bit positions and masks */

#define AD5940_REG_AFEWDT_WDTVALS                0x0904 /* Address */
#define AD5940_BITP_AFEWDT_WDTVALS_CCOUNT        0      /* Current WDT cnt */
#define AD5940_BITM_AFEWDT_WDTVALS_CCOUNT        0xffff /* Current WDT cnt */

/* AFEWDT Watchdog Timer Control register address, bit positions and masks.
 * Bit 15~11: Reserved
 * Bit    10: WDT interrupt ennable
 * Bit     9: Timer window control
 * Bit     8: Clock Source
 * Bit     7: Reserved
 * Bit     6: Timer mode select
 * Bit     5: Timer enable
 * Bit     2: Prescaler
 * Bit     1: WDT interrupt enable
 *            If set 0, watchdog timer timeout creates a reset.
 *            If set 1, watchdog timer timeout creates an interrupt.
 * Bit     0: Power down stop enable.
 *            If set 0, continue counting when in hibernate
 *            If set 1, stop counter when in hibernate
 */

#define AD5940_REG_AFEWDT_WDTCON                 0x0908 /* Address */
#define AD5940_BITP_AFEWDT_WDTCON_WDTIRQEN       10     /* Poisition */
#define AD5940_BITP_AFEWDT_WDTCON_MINLOAD_EN     9      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTCON_CLKDIV2        8      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTCON_MDE            6      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTCON_EN             5      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTCON_PRE            2      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTCON_IRQ            1      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTCON_PDSTOP         0      /* Poisition */
#define AD5940_BITM_AFEWDT_WDTCON_WDTIRQEN       0x0400 /* Mask */
#define AD5940_BITM_AFEWDT_WDTCON_MINLOAD_EN     0x0200 /* Mask */
#define AD5940_BITM_AFEWDT_WDTCON_CLKDIV2        0x0100 /* Mask */
#define AD5940_BITM_AFEWDT_WDTCON_MDE            0x0040 /* Mask */
#define AD5940_BITM_AFEWDT_WDTCON_EN             0x0020 /* Mask */
#define AD5940_BITM_AFEWDT_WDTCON_PRE            0x000c /* Mask */
#define AD5940_BITM_AFEWDT_WDTCON_IRQ            0x0002 /* Mask */
#define AD5940_BITM_AFEWDT_WDTCON_PDSTOP         0x0001 /* Mask */
#define AD5940_ENUM_AFEWDT_WDTCON_RESET          0x0000 /* IRQ = 0 */
#define AD5940_ENUM_AFEWDT_WDTCON_INTERRUPT      0x0002 /* IRQ = 1 */
#define AD5940_ENUM_AFEWDT_WDTCON_CONTINUE       0x0000 /* PDSTOP = 0 */
#define AD5940_ENUM_AFEWDT_WDTCON_STOP           0x0001 /* PDSTOP = 1 */

/* AFEWDT Refresh Watchdog register address, bit positions and masks */

#define AD5940_REG_AFEWDT_WDTCLRI                0x090C /* Address */
#define AD5940_BITP_AFEWDT_WDTCLRI_CLRWDG        0      /* Position */
#define AD5940_BITM_AFEWDT_WDTCLRI_CLRWDG        0xffff /* Mask */

/* AFEWDT Timer Status register address, bit positions and masks.
 * Bit 15~7: Reserved.
 * Bit    6: Writing status of register AD5940_REG_AFEWDT_WDTMINLD
 * Bit    5: Reset type status
 * Bit    4: Lock status
 *           If it's 0, timer operation is not locked.
 *           If it's 1, timer is enabled and locked.
 * Bit    3: Writing status of register AD5940_REG_AFEWDT_WDTCON
 * Bit    2: Writing status of register AD5940_REG_AFEWDT_WDTVAL
 *           If it's 0, arm and AFE Watchdog clock domains WDTLD values match
 *           If it's 1, synchronization is in progress
 * Bit    1: Writing status of register AD5940_REG_AFEWDT_WDTCLRI
 * Bit    0: WDT interrupt status
 *           If it's 0, watchdog timer interrupt is not pending
 *           If it's 1, watchdog timer interrupt is pending
 */

#define AD5940_REG_AFEWDT_WDTSTA                 0x0918 /* Address */
#define AD5940_BITP_AFEWDT_WDTSTA_TMINLD         6      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTSTA_OTPWRDONE      5      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTSTA_LOCK           4      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTSTA_CON            3      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTSTA_TLD            2      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTSTA_CLRI           1      /* Poisition */
#define AD5940_BITP_AFEWDT_WDTSTA_IRQ            0      /* Poisition */
#define AD5940_BITM_AFEWDT_WDTSTA_TMINLD         0x0040 /* Mask */
#define AD5940_BITM_AFEWDT_WDTSTA_OTPWRDONE      0x0020 /* Mask */
#define AD5940_BITM_AFEWDT_WDTSTA_LOCK           0x0010 /* Mask */
#define AD5940_BITM_AFEWDT_WDTSTA_CON            0x0008 /* Mask */
#define AD5940_BITM_AFEWDT_WDTSTA_TLD            0x0004 /* Mask */
#define AD5940_BITM_AFEWDT_WDTSTA_CLRI           0x0002 /* Mask */
#define AD5940_BITM_AFEWDT_WDTSTA_IRQ            0x0001 /* Mask */
#define AD5940_ENUM_AFEWDT_WDTSTA_OPEN           0x0000 /* WDTSTA_LOCK = 0 */
#define AD5940_ENUM_AFEWDT_WDTSTA_LOCKED         0x0010 /* WDTSTA_LOCK = 1 */
#define AD5940_ENUM_AFEWDT_WDTSTA_SYNC_COMPLETE  0x0000 /* WDTSTA_TLD = 0 */
#define AD5940_ENUM_AFEWDT_WDTSTA_SYNC_PROGRESS  0x0004 /* WDTSTA_TLD = 1 */
#define AD5940_ENUM_AFEWDT_WDTSTA_CLEARED        0x0000 /* WDTSTA_IRQ = 0 */
#define AD5940_ENUM_AFEWDT_WDTSTA_PENDING        0x0001 /* WDTSTA_IRQ = 1 */

/* AFEWDT Minimum Load Value register address, bit positions and masks */

#define AD5940_REG_AFEWDT_WDTMINLD               0x091c /* Address */
#define AD5940_BITP_AFEWDT_WDTMINLD_MIN_LOAD     0      /* Position */
#define AD5940_BITM_AFEWDT_WDTMINLD_MIN_LOAD     0xffff /* Mask */

/* Part1.4 Wakeup timer registers (16bits) */

/* WUPTMR (wakeup timer) registers address, bit positions and masks.
 * Bit   6: Mark sequence trigger from sleep wakeup timer
 * Bit 5:4: Clock selection
 *          If it's 0/2, clock comes from internal 32kHz OSC
 *          If it's 1/3, clock comes from external clock
 * Bit 3:1: End sequence
 *          (0~7)<<1 represent SeqA~H respectively, the sleep wakeup timer
 *          will stop at the set SeqX and then go back to SeqA
 * Bit   0: Sleep wake timer enable bit
 *          If it's 0, enables sleep wakeup timer.
 *          If it's 1, disables sleep wakeup timer.
 */

#define AD5940_REG_WUPTMR_CON             0x0800 /* Address */
#define AD5940_REG_WUPTMR_CON_RESET       0x0000 /* Reset value */
#define AD5940_BITP_WUPTMR_CON_MSKTRG     6      /* Poisition */
#define AD5940_BITP_WUPTMR_CON_CLKSEL     4      /* Poisition */
#define AD5940_BITP_WUPTMR_CON_ENDSEQ     1      /* Poisition */
#define AD5940_BITP_WUPTMR_CON_EN         0      /* Poisition */
#define AD5940_BITM_WUPTMR_CON_MSKTRG     0x0040 /* Mask */
#define AD5940_BITM_WUPTMR_CON_CLKSEL     0x0030 /* Mask */
#define AD5940_BITM_WUPTMR_CON_ENDSEQ     0x000e /* Mask */
#define AD5940_BITM_WUPTMR_CON_EN         0x0001 /* Mask */
#define AD5940_ENUM_WUPTMR_CON_SWT32K0    0x0000 /* CON_CLKSEL = 0 << 4 */
#define AD5940_ENUM_WUPTMR_CON_SWTEXT0    0x0010 /* CON_CLKSEL = 1 << 4 */
#define AD5940_ENUM_WUPTMR_CON_SWT32K     0x0020 /* CON_CLKSEL = 2 << 4 */
#define AD5940_ENUM_WUPTMR_CON_SWTEXT     0x0030 /* CON_CLKSEL = 3 << 4 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQA    0x0000 /* CON_ENDSEQ = 0 << 1 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQB    0x0002 /* CON_ENDSEQ = 1 << 1 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQC    0x0004 /* CON_ENDSEQ = 2 << 1 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQD    0x0006 /* CON_ENDSEQ = 3 << 1 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQE    0x0008 /* CON_ENDSEQ = 4 << 1 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQF    0x000a /* CON_ENDSEQ = 5 << 1 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQG    0x000c /* CON_ENDSEQ = 6 << 1 */
#define AD5940_ENUM_WUPTMR_CON_ENDSEQH    0x000e /* CON_ENDSEQ = 7 << 1 */
#define AD5940_ENUM_WUPTMR_CON_SWTEN      0x0000 /* CON_EN = 0 */
#define AD5940_ENUM_WUPTMR_CON_SWTDIS     0x0001 /* CON_EN = 1 */

/* WUPTMR Order Control registers address, bit positions and masks.
 * Bits: 15 14 | 13 12 | 11 10 | 9  8 | 7  6 | 5  4 | 3  2 | 1  0
 * Seq:   SeqH | SeqG  | SeqF  | SeqE | SeqD | SeqC | SeqB | SeqA
 */

#define AD5940_REG_WUPTMR_SEQORDER        0x0804 /* Address */
#define AD5940_REG_WUPTMR_SEQORDER_RESET  0x0000 /* Reset value */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQH  14     /* Poisition */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQG  12     /* Poisition */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQF  10     /* Poisition */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQE  8      /* Poisition */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQD  6      /* Poisition */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQC  4      /* Poisition */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQB  2      /* Poisition */
#define AD5940_BITP_WUPTMR_SEQORDER_SEQA  0      /* Poisition */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQH  0xc000 /* Mask */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQG  0x3000 /* Mask */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQF  0x0c00 /* Mask */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQE  0x0300 /* Mask */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQD  0x00c0 /* Mask */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQC  0x0030 /* Mask */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQB  0x000c /* Mask */
#define AD5940_BITM_WUPTMR_SEQORDER_SEQA  0x0003 /* Mask */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQH0 0x0000 /* SEQH: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQH1 0x4000 /* SEQH: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQH2 0x8000 /* SEQH: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQH3 0xC000 /* SEQH: Fill SEQ3 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQG0 0x0000 /* SEQG: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQG1 0x1000 /* SEQG: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQG2 0x2000 /* SEQG: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQG3 0x3000 /* SEQG: Fill SEQ3 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQF0 0x0000 /* SEQF: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQF1 0x0400 /* SEQF: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQF2 0x0800 /* SEQF: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQF3 0x0c00 /* SEQF: Fill SEQ3 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQE0 0x0000 /* SEQE: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQE1 0x0100 /* SEQE: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQE2 0x0200 /* SEQE: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQE3 0x0300 /* SEQE: Fill SEQ3 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQD0 0x0000 /* SEQD: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQD1 0x0040 /* SEQD: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQD2 0x0080 /* SEQD: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQD3 0x00c0 /* SEQD: Fill SEQ3 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQC0 0x0000 /* SEQC: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQC1 0x0010 /* SEQC: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQC2 0x0020 /* SEQC: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQC3 0x0030 /* SEQC: Fill SEQ3 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQB0 0x0000 /* SEQB: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQB1 0x0004 /* SEQB: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQB2 0x0008 /* SEQB: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQB3 0x000c /* SEQB: Fill SEQ3 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQA0 0x0000 /* SEQA: Fill SEQ0 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQA1 0x0001 /* SEQA: Fill SEQ1 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQA2 0x0002 /* SEQA: Fill SEQ2 in */
#define AD5940_ENUM_WUPTMR_SEQORDER_SEQA3 0x0003 /* SEQA: Fill SEQ3 in */

/* WUPTMR SEQ0 Wakeup timer (LSB) register address, bit positions and masks.
 * Its value represents Seq0 sleep period.
 */

#define AD5940_REG_WUPTMR_SEQ0WUPL               0x0808 /* Address */
#define AD5940_REG_WUPTMR_SEQ0WUPL_RESET         0xffff /* Reset value */
#define AD5940_BITP_WUPTMR_SEQXWUPL_WAKEUPTIMEX  0      /* Position */
#define AD5940_BITM_WUPTMR_SEQXWUPL_WAKEUPTIMEX  0xffff /* Mask */

/* WUPTMR SEQ0 Wakeup timer (MSB) register address, bit positions and masks */

#define AD5940_REG_WUPTMR_SEQ0WUPH               0x080c /* Address */
#define AD5940_REG_WUPTMR_SEQ0WUPH_RESET         0x000f /* Reset value */
#define AD5940_BITP_WUPTMR_SEQXWUPH_WAKEUPTIMEX  0      /* Position */
#define AD5940_BITM_WUPTMR_SEQXWUPH_WAKEUPTIMEX  0x000f /* Mask */

/* WUPTMR SEQ0 Sleep timer (LSB) register address, bit positions and masks.
 * Its value represents Seq0 active period.
 */

#define AD5940_REG_WUPTMR_SEQ0SLEEPL             0x0810 /* Address */
#define AD5940_REG_WUPTMR_SEQ0SLEEPL_RESET       0xffff /* Reset value */
#define AD5940_BITP_WUPTMR_SEQXSLEEPL_SLEEPTIMEX 0      /* Position */
#define AD5940_BITM_WUPTMR_SEQXSLEEPL_SLEEPTIMEX 0xffff /* Mask */

/* WUPTMR SEQ0 Sleep timer (MSB) register address, bit positions and masks */

#define AD5940_REG_WUPTMR_SEQ0SLEEPH             0x0814 /* Address */
#define AD5940_REG_WUPTMR_SEQ0SLEEPH_RESET       0x000f /* Reset value */
#define AD5940_BITP_WUPTMR_SEQXSLEEPH_SLEEPTIMEX 0      /* Position */
#define AD5940_BITM_WUPTMR_SEQXSLEEPH_SLEEPTIMEX 0x000f /* Mask */

/* WUPTMR SEQ1/2/3 wakeup/sleep timers (LSB/MSB) registers address and reset
 * values. The bit postions and masks are as the same as SEQ0 registers
 */

#define AD5940_REG_WUPTMR_SEQ1WUPL               0x0818 /* Wakeup timer LSB */
#define AD5940_REG_WUPTMR_SEQ1WUPL_RESET         0xffff /* Reset value */

#define AD5940_REG_WUPTMR_SEQ1WUPH               0x081c /* Wakeup timer MSB */
#define AD5940_REG_WUPTMR_SEQ1WUPH_RESET         0x000f /* Reset value */

#define AD5940_REG_WUPTMR_SEQ1SLEEPL             0x0820 /* Sleep timer LSB */
#define AD5940_REG_WUPTMR_SEQ1SLEEPL_RESET       0xffff /* Reset value */

#define AD5940_REG_WUPTMR_SEQ1SLEEPH             0x0824 /* Sleep timer MSB */
#define AD5940_REG_WUPTMR_SEQ1SLEEPH_RESET       0x000f /* Reset value */

#define AD5940_REG_WUPTMR_SEQ2WUPL               0x0828 /* Wakeup timer LSB */
#define AD5940_REG_WUPTMR_SEQ2WUPL_RESET         0xffff /* Reset value */

#define AD5940_REG_WUPTMR_SEQ2WUPH               0x082c /* Wakeup timer MSB */
#define AD5940_REG_WUPTMR_SEQ2WUPH_RESET         0x000f /* Reset value */

#define AD5940_REG_WUPTMR_SEQ2SLEEPL             0x0830 /* Sleep timer LSB */
#define AD5940_REG_WUPTMR_SEQ2SLEEPL_RESET       0xffff /* Reset value */

#define AD5940_REG_WUPTMR_SEQ2SLEEPH             0x0834 /* Sleep timer MSB */
#define AD5940_REG_WUPTMR_SEQ2SLEEPH_RESET       0x000f /* Reset value */

#define AD5940_REG_WUPTMR_SEQ3WUPL               0x0838 /* Wakeup timer LSB */
#define AD5940_REG_WUPTMR_SEQ3WUPL_RESET         0xffff /* Reset value */

#define AD5940_REG_WUPTMR_SEQ3WUPH               0x083c /* Wakeup timer MSB */
#define AD5940_REG_WUPTMR_SEQ3WUPH_RESET         0x000f /* Reset value */

#define AD5940_REG_WUPTMR_SEQ3SLEEPL             0x0840 /* Sleep timer LSB */
#define AD5940_REG_WUPTMR_SEQ3SLEEPL_RESET       0xffff /* Reset value */

#define AD5940_REG_WUPTMR_SEQ3SLEEPH             0x0844 /* Sleep timer MSB */
#define AD5940_REG_WUPTMR_SEQ3SLEEPH_RESET       0x000f /* Reset value */

/* Part1.5 ALLON (Always On) registers (16bit) */

/* ALLON Power Modes register.
 * Bit 15: Retention for RAM
 * Bit 14: Keep ADC Power Switch on in Hibernate
 * Bit  3: Auto Sleep by Sequencer Command
 * Bit  2: Auto Sleep by Sleep Wakeup Timer
 * Bit  0: Power Mode Control Bits
 */

#define AD5940_REG_ALLON_PWRMOD           0x0a00 /* Address */
#define AD5940_REG_ALLON_PWRMOD_RESET     0x0001 /* Reset value */
#define AD5940_BITP_ALLON_PWRMOD_RAMRETEN 15     /* Position */
#define AD5940_BITP_ALLON_PWRMOD_ADCRETEN 14     /* Position */
#define AD5940_BITP_ALLON_PWRMOD_SEQSLPEN 3      /* Position */
#define AD5940_BITP_ALLON_PWRMOD_TMRSLPEN 2      /* Position */
#define AD5940_BITP_ALLON_PWRMOD_PWRMOD   0      /* Position */
#define AD5940_BITM_ALLON_PWRMOD_RAMRETEN 0x8000 /* Mask */
#define AD5940_BITM_ALLON_PWRMOD_ADCRETEN 0x4000 /* Mask */
#define AD5940_BITM_ALLON_PWRMOD_SEQSLPEN 0x0008 /* Mask */
#define AD5940_BITM_ALLON_PWRMOD_TMRSLPEN 0x0004 /* Mask */
#define AD5940_BITM_ALLON_PWRMOD_PWRMOD   0x0003 /* Mask */
#define AD5940_ENUM_ALLON_PWRMOD_SLPMOD   0x0002 /* Sleep mode */
#define AD5940_ENUM_ALLON_PWRMOD_ACTIVMOD 0x0001 /* Active mode */

/* ALLON Key Protection for PWRMOD register */

#define AD5940_REG_ALLON_PWRKEY           0x0a04 /* Address */
#define AD5940_REG_ALLON_PWRKEY_RESET     0x0000 /* Reset value */
#define AD5940_BITP_ALLON_PWRKEY_PWRKEY   0      /* Position */
#define AD5940_BITM_ALLON_PWRKEY_PWRKEY   0xffff /* Mask */

/* ALLON Key Protection for OSCCON register */

#define AD5940_REG_ALLON_OSCKEY           0x0a0c /* Address */
#define AD5940_REG_ALLON_OSCKEY_RESET     0x0000 /* Reset value */
#define AD5940_BITP_ALLON_OSCKEY_OSCKEY   0      /* Position */
#define AD5940_BITM_ALLON_OSCKEY_OSCKEY   0xffff /* Mask */

/* ALLON Oscillator Control register.
 * Bit 10: Status of HFXTAL oscillator
 * Bit  9: Status of HFOSC oscillator
 * Bit  8: Status of LFOSC oscillator
 * Bit  2: High frequency crystal oscillator enable
 * Bit  1: High frequency internal oscillator enable
 * Bit  0: Low frequency internal oscillator enable
 */

#define AD5940_REG_ALLON_OSCCON           0x0a10 /* Address */
#define AD5940_REG_ALLON_OSCCON_RESET     0x0003 /* Reset value */
#define AD5940_BITP_ALLON_OSCCON_HFXTALOK 10     /* Position */
#define AD5940_BITP_ALLON_OSCCON_HFOSCOK  9      /* Position */
#define AD5940_BITP_ALLON_OSCCON_LFOSCOK  8      /* Position */
#define AD5940_BITP_ALLON_OSCCON_HFXTALEN 2      /* Position */
#define AD5940_BITP_ALLON_OSCCON_HFOSCEN  1      /* Position */
#define AD5940_BITP_ALLON_OSCCON_LFOSCEN  0      /* Position */
#define AD5940_BITM_ALLON_OSCCON_HFXTALOK 0x0400 /* Mask */
#define AD5940_BITM_ALLON_OSCCON_HFOSCOK  0x0200 /* Mask */
#define AD5940_BITM_ALLON_OSCCON_LFOSCOK  0x0100 /* Mask */
#define AD5940_BITM_ALLON_OSCCON_HFXTALEN 0x0004 /* Mask */
#define AD5940_BITM_ALLON_OSCCON_HFOSCEN  0x0002 /* Mask */
#define AD5940_BITM_ALLON_OSCCON_LFOSCEN  0x0001 /* Mask */

/* ALLON Timer Wakeup Configuration register */

#define AD5940_REG_ALLON_TMRCON           0x0a1c /* Address */
#define AD5940_REG_ALLON_TMRCON_RESET     0x0000 /* Reset value for TMRCON */
#define AD5940_BITP_ALLON_TMRCON_TMRINTEN 0      /* Enable wakeup timer */
#define AD5940_BITM_ALLON_TMRCON_TMRINTEN 0x0001 /* Enable wakeup timer */

/* ALLON External Interrupt Configuration 0 register
 * Bit    15: External interrupt 3 enable bit
 * Bit 14:12: External interrupt 3 mode registers
 * Bit    11: External interrupt 2 enable bit
 * Bit  10:8: External interrupt 2 mode registers
 * Bit     7: External interrupt 1 enable bit
 * Bit   6:4: External interrupt 1 mode registers
 * Bit     3: External interrupt 0 enable bit
 * Bit   2:0: External interrupt 0 mode registers
 */

#define AD5940_REG_ALLON_EI0CON           0x0a20 /* Address */
#define AD5940_REG_ALLON_EI0CON_RESET     0x0000 /* Reset value for EI0CON */
#define AD5940_BITP_ALLON_EI0CON_IRQ3EN   15     /* Interrupt 3 enable */
#define AD5940_BITP_ALLON_EI0CON_IRQ3MDE  12     /* Interrupt 3 mode */
#define AD5940_BITP_ALLON_EI0CON_IRQ2EN   11     /* Interrupt 2 enable */
#define AD5940_BITP_ALLON_EI0CON_IRQ2MDE  8      /* Interrupt 2 mode */
#define AD5940_BITP_ALLON_EI0CON_IRQ1EN   7      /* Interrupt 1 enable */
#define AD5940_BITP_ALLON_EI0CON_IRQ1MDE  4      /* Interrupt 1 mode */
#define AD5940_BITP_ALLON_EI0CON_IRQ0EN   3      /* Interrupt 0 enable */
#define AD5940_BITP_ALLON_EI0CON_IRQ0MDE  0      /* Interrupt 0 mode */
#define AD5940_BITM_ALLON_EI0CON_IRQ3EN   0x8000 /* Interrupt 3 enable */
#define AD5940_BITM_ALLON_EI0CON_IRQ3MDE  0x7000 /* Interrupt 3 mode */
#define AD5940_BITM_ALLON_EI0CON_IRQ2EN   0x0800 /* Interrupt 2 enable */
#define AD5940_BITM_ALLON_EI0CON_IRQ2MDE  0x0700 /* Interrupt 2 mode */
#define AD5940_BITM_ALLON_EI0CON_IRQ1EN   0x0080 /* Interrupt 1 enable */
#define AD5940_BITM_ALLON_EI0CON_IRQ1MDE  0x0070 /* Interrupt 1 mode */
#define AD5940_BITM_ALLON_EI0CON_IRQ0EN   0x0008 /* Interrupt 0 enable */
#define AD5940_BITM_ALLON_EI0CON_IRQ0MDE  0x0007 /* Interrupt 0 mode */

/* ALLON External Interrupt Configuration 1 register
 * Bit    15: External interrupt 7 enable bit
 * Bit 14:12: External interrupt 7 mode registers
 * Bit    11: External interrupt 6 enable bit
 * Bit  10:8: External interrupt 6 mode registers
 * Bit     7: External interrupt 5 enable bit
 * Bit   6:4: External interrupt 5 mode registers
 * Bit     3: External interrupt 4 enable bit
 * Bit   2:0: External interrupt 4 mode registers
 */

#define AD5940_REG_ALLON_EI1CON           0x0a24 /* Address */
#define AD5940_REG_ALLON_EI1CON_RESET     0x0000 /* Reset value for EI1CON */
#define AD5940_BITP_ALLON_EI1CON_IRQ7EN   15     /* Interrupt 7 enable */
#define AD5940_BITP_ALLON_EI1CON_IRQ7MDE  12     /* Interrupt 7 mode */
#define AD5940_BITP_ALLON_EI1CON_IRQ6EN   11     /* Interrupt 6 enable */
#define AD5940_BITP_ALLON_EI1CON_IRQ6MDE  8      /* Interrupt 6 mode */
#define AD5940_BITP_ALLON_EI1CON_IRQ5EN   7      /* Interrupt 5 enable */
#define AD5940_BITP_ALLON_EI1CON_IRQ5MDE  4      /* Interrupt 5 mode */
#define AD5940_BITP_ALLON_EI1CON_IRQ4EN   3      /* Interrupt 4 enable */
#define AD5940_BITP_ALLON_EI1CON_IRQ4MDE  0      /* Interrupt 4 mode */
#define AD5940_BITM_ALLON_EI1CON_IRQ7EN   0x8000 /* Interrupt 7 enable */
#define AD5940_BITM_ALLON_EI1CON_IRQ7MDE  0x7000 /* Interrupt 7 mode */
#define AD5940_BITM_ALLON_EI1CON_IRQ6EN   0x0800 /* Interrupt 6 enable */
#define AD5940_BITM_ALLON_EI1CON_IRQ6MDE  0x0700 /* Interrupt 6 mode */
#define AD5940_BITM_ALLON_EI1CON_IRQ5EN   0x0080 /* Interrupt 5 enable */
#define AD5940_BITM_ALLON_EI1CON_IRQ5MDE  0x0070 /* Interrupt 5 mode */
#define AD5940_BITM_ALLON_EI1CON_IRQ4EN   0x0008 /* Interrupt 4 enable */
#define AD5940_BITM_ALLON_EI1CON_IRQ4MDE  0x0007 /* Interrupt 4 mode */

/* ALLON External Interrupt Configuration 2 register
 * Bit   3: Bus interrupt detection enable bit
 * Bit 2:0: Bus interrupt detection mode registers
 */

#define AD5940_REG_ALLON_EI2CON           0x0a28 /* Address */
#define AD5940_REG_ALLON_EI2CON_RESET     0x0000 /* Reset value */
#define AD5940_BITP_ALLON_EI2CON_BINTEN   3      /* Position */
#define AD5940_BITP_ALLON_EI2CON_BINTMDE  0      /* Position */
#define AD5940_BITM_ALLON_EI2CON_BINTEN   0x0008 /* Mask */
#define AD5940_BITM_ALLON_EI2CON_BINTMDE  0x0007 /* Mask */

/* ALLON External Interrupt Clear register
 * Bit 15: Enable auto clear of bus interrupt
 * BIt  8: BUS interrupt
 */

#define AD5940_REG_ALLON_EICLR            0x0a30 /* Address */
#define AD5940_REG_ALLON_EICLR_RESET      0xc000 /* Reset value */
#define AD5940_BITP_ALLON_EICLR_ACLRBUSEN 15     /* Enable auto CLR */
#define AD5940_BITP_ALLON_EICLR_BUSINT    8      /* BUS interrupt */
#define AD5940_BITM_ALLON_EICLR_ACLRBUSEN 0x8000 /* Enable auto CLR */
#define AD5940_BITM_ALLON_EICLR_BUSINT    0x0100 /* BUS interrupt */

/* ALLON Reset Status register */

#define AD5940_REG_ALLON_RSTSTA           0x0a40 /* ALLON reset status */
#define AD5940_REG_ALLON_RSTSTA_RESET     0x0000 /* Reset value for RSTSTA */
#define AD5940_BITP_ALLON_RSTSTA_PINSWRST 4      /* Software reset pin */
#define AD5940_BITP_ALLON_RSTSTA_MMRSWRST 3      /* Mmr software reset */
#define AD5940_BITP_ALLON_RSTSTA_WDRST    2      /* Watchdog timeout */
#define AD5940_BITP_ALLON_RSTSTA_EXTRST   1      /* External reset */
#define AD5940_BITP_ALLON_RSTSTA_POR      0      /* Power on reset */
#define AD5940_BITM_ALLON_RSTSTA_PINSWRST 0x0010 /* Software reset pin */
#define AD5940_BITM_ALLON_RSTSTA_MMRSWRST 0x0008 /* Mmr software reset */
#define AD5940_BITM_ALLON_RSTSTA_WDRST    0x0004 /* Watchdog timeout */
#define AD5940_BITM_ALLON_RSTSTA_EXTRST   0x0002 /* External reset */
#define AD5940_BITM_ALLON_RSTSTA_POR      0x0001 /* Power-on reset */

/* ALLON Key Protection for RSTCON Register */

#define AD5940_REG_ALLON_RSTCONKEY        0x0a5c /* Address */
#define AD5940_REG_ALLON_RSTCONKEY_RESET  0x0000 /* Reset value */
#define AD5940_BITP_ALLON_RSTCONKEY_KEY   0      /* Reset Control Key */
#define AD5940_BITM_ALLON_RSTCONKEY_KEY   0xffff /* Reset Control Key */

/* ALLON Internal LF Oscillator Test register
 * Bit 3:0: Trim capacitances to adjust frequency.
 */

#define AD5940_REG_ALLON_LOSCTST          0x0a6c /* Address */
#define AD5940_REG_ALLON_LOSCTST_RESET    0x008f /* Reset value for LOSCTST */
#define AD5940_BITP_ALLON_LOSCTST_TRIM    0      /* Trim cap to adjust freq */
#define AD5940_BITM_ALLON_LOSCTST_TRIM    0x000f /* Trim cap to adjust freq */

/* ALLON 32KHz Peripheral Clock Enable register
 * Bit 2: TIA chop clock disable.
 * Bit 1: Sleep/wakeup timer clock disable.
 * Bit 0: Watch dog timer clock disable.
 */

#define AD5940_REG_ALLON_CLKEN0           0x0a70 /* Address */
#define AD5940_REG_ALLON_CLKEN0_RESET     0x0004 /* Reset value */
#define AD5940_BITP_ALLON_CLKEN0_TIACPDIS 2      /* Position */
#define AD5940_BITP_ALLON_CLKEN0_SLPWTDIS 1      /* Position */
#define AD5940_BITP_ALLON_CLKEN0_WDTDIS   0      /* Position */
#define AD5940_BITM_ALLON_CLKEN0_TIACPDIS 0x0004 /* Mask */
#define AD5940_BITM_ALLON_CLKEN0_SLPWTDIS 0x0002 /* Mask */
#define AD5940_BITM_ALLON_CLKEN0_WDTDIS   0x0001 /* Mask */

/* Part1.6 General purpose timer - AGPT0/1 registers (16bit) */

/* AGPT0 16 bit load value register. */

#define AD5940_REG_AGPT0_LD0              0x0d00 /* Address */
#define AD5940_BITP_AGPTX_LDX_LOAD        0      /* Poistion: load value */
#define AD5940_BITM_AGPTX_LDX_LOAD        0xffff /* Mask: load value */

/* AGPT0 16-bit timer value register. */

#define AD5940_REG_AGPT0_VAL0             0x0d04 /* Address */
#define AD5940_BITP_AGPTX_VALX_VAL        0      /* Current Count */
#define AD5940_BITM_AGPTX_VALX_VAL        0xffff /* Current Count */

/* AGPT0 control register. */

#define AD5940_REG_AGPT0_CON0             0x0d08 /* Address */
#define AD5940_BITP_AGPTX_CONX_SYNCBYP    15     /* Synchronization bypass */
#define AD5940_BITP_AGPTX_CONX_RSTEN      14     /* Counter & Prescale RST */
#define AD5940_BITP_AGPTX_CONX_EVTEN      13     /* Event Select */
#define AD5940_BITP_AGPTX_CONX_EVENT      8      /* Event Select Range */
#define AD5940_BITP_AGPTX_CONX_RLD        7      /* Reload Control */
#define AD5940_BITP_AGPTX_CONX_CLK        5      /* Clock Select */
#define AD5940_BITP_AGPTX_CONX_ENABLE     4      /* Timer Enable */
#define AD5940_BITP_AGPTX_CONX_MOD        3      /* Timer Mode */
#define AD5940_BITP_AGPTX_CONX_UP         2      /* Count up */
#define AD5940_BITP_AGPTX_CONX_PRE        0      /* Prescaler */
#define AD5940_BITM_AGPTX_CONX_SYNCBYP    0x8000 /* Synchronization Bypass */
#define AD5940_BITM_AGPTX_CONX_RSTEN      0x4000 /* Counter & Prescale RST */
#define AD5940_BITM_AGPTX_CONX_EVTEN      0x2000 /* Event select */
#define AD5940_BITM_AGPTX_CONX_EVENT      0x1f00 /* Event select range */
#define AD5940_BITM_AGPTX_CONX_RLD        0x0080 /* Reload control */
#define AD5940_BITM_AGPTX_CONX_CLK        0x0060 /* Clock select */
#define AD5940_BITM_AGPTX_CONX_ENABLE     0x0010 /* Timer enable */
#define AD5940_BITM_AGPTX_CONX_MOD        0x0008 /* Timer mode */
#define AD5940_BITM_AGPTX_CONX_UP         0x0004 /* Count up */
#define AD5940_BITM_AGPTX_CONX_PRE        0x0003 /* Prescaler */

/* AGPT0 clear interrupt register.
 * Bit 1: Clear captured event interrupt.
 * Bit 0: Clear timeout interrupt.
 */

#define AD5940_REG_AGPT0_CLRI0            0x0d0c /* Address */
#define AD5940_BITP_AGPTX_CLRIX_CAP       1      /* Clear captured event */
#define AD5940_BITP_AGPTX_CLRIX_TMOUT     0      /* Clear timeout interrupt */
#define AD5940_BITM_AGPTX_CLRIX_CAP       0x0002 /* Clear captured event */
#define AD5940_BITM_AGPTX_CLRIX_TMOUT     0x0001 /* Clear timeout interrupt */

/* AGPT0 capture register. */

#define AD5940_REG_AGPT0_CAP0             0x0d10 /* Address */
#define AD5940_BITP_AGPTX_CAPX_CAP        0      /* 16-bit captured value */
#define AD5940_BITM_AGPTX_CAPX_CAP        0xffff /* 16-bit captured value */

/* AGPT0 16-bit load value, asynchronous register. */

#define AD5940_REG_AGPT0_ALD0             0x0d14 /* Address */
#define AD5940_BITP_AGPTX_ALDX_ALOAD      0      /* Load value asynchronous */
#define AD5940_BITM_AGPTX_ALDX_ALOAD      0xffff /* Load value asynchronous */

/* AGPT0 16-bit timer value, asynchronous register. */

#define AD5940_REG_AGPT0_AVAL0            0x0d18 /* Address */
#define AD5940_BITP_AGPTX_AVALX_AVAL      0      /* Counter value */
#define AD5940_BITM_AGPTX_AVALX_AVAL      0xffff /* Counter value */

/* AGPT0 status register.
 * Bit 8: Counter reset occurring
 * Bit 7: Clear interrupt register synchronization
 * Bit 6: Timer busy
 * Bit 1: Capture event pending
 * Bit 0: Timeout event occurred
 */

#define AD5940_REG_AGPT0_STA0             0x0d1c /* Address */
#define AD5940_BITP_AGPTX_STAX_RSTCNT     8      /* Counter reset occurring */
#define AD5940_BITP_AGPTX_STAX_PDOK       7      /* Clear interrupt REG */
#define AD5940_BITP_AGPTX_STAX_BUSY       6      /* Timer busy */
#define AD5940_BITP_AGPTX_STAX_CAP        1      /* Capture event pending */
#define AD5940_BITP_AGPTX_STAX_TMOUT      0      /* Timeout event occurred */
#define AD5940_BITM_AGPTX_STAX_RSTCNT     0x0100 /* Counter reset occurring */
#define AD5940_BITM_AGPTX_STAX_PDOK       0x0080 /* Clear interrupt REG */
#define AD5940_BITM_AGPTX_STAX_BUSY       0x0040 /* Timer busy */
#define AD5940_BITM_AGPTX_STAX_CAP        0x0002 /* Capture event pending */
#define AD5940_BITM_AGPTX_STAX_TMOUT      0x0001 /* Timeout event occurred */

/* AGPT0 PWM control register. */

#define AD5940_REG_AGPT0_PWMCON0          0x0d20 /* Address */
#define AD5940_BITP_AGPTX_PWMCONX_IDLE    1      /* PWM Idle State */
#define AD5940_BITP_AGPTX_PWMCONX_MATCHEN 0      /* PWM Match Enabled */
#define AD5940_BITM_AGPTX_PWMCONX_IDLE    0x0002 /* PWM Idle State */
#define AD5940_BITM_AGPTX_PWMCONX_MATCHEN 0x0001 /* PWM Match Enabled */

/* AGPT0 PWM match value register. */

#define AD5940_REG_AGPT0_PWMMAT0          0x0d24 /* Address */
#define AD5940_BITP_AGPTX_PWMMATX_MTCHVAL 0      /* PWM match value */
#define AD5940_BITM_AGPTX_PWMMATX_MTCHVAL 0xffff /* PWM match value */

/* AGPT0 Interrupt Enable */

#define AD5940_REG_AGPT0_INTEN            0x0d28 /* Address */
#define AD5940_BITP_AGPTX_INTEN_INTEN     0      /* Interrupt Enable */
#define AD5940_BITM_AGPTX_INTEN_INTEN     0x0001 /* Interrupt Enable */

/* General purpose timer - AGPT1 registers addresses.
 * The bit positions and masks of each register, are as the same as those
 * ones of corresponding AGPT0 register. See AGPT registers for detail.
 */

#define AD5940_REG_AGPT1_LD1              0x0e00 /* Load value */
#define AD5940_REG_AGPT1_VAL1             0x0e04 /* Timer value */
#define AD5940_REG_AGPT1_CON1             0x0e08 /* Control */
#define AD5940_REG_AGPT1_CLRI1            0x0e0c /* Clear interrupt */
#define AD5940_REG_AGPT1_CAP1             0x0e10 /* Capture */
#define AD5940_REG_AGPT1_ALD1             0x0e14 /* Load value (async) */
#define AD5940_REG_AGPT1_AVAL1            0x0e18 /* Timer value (async) */
#define AD5940_REG_AGPT1_STA1             0x0e1c /* Status */
#define AD5940_REG_AGPT1_PWMCON1          0x0e20 /* PWM control */
#define AD5940_REG_AGPT1_PWMMAT1          0x0e24 /* PWM match value */
#define AD5940_REG_AGPT1_INTEN1           0x0e28 /* Interrupt enable */

/* Part1.7 AFECRC - CRC accelerator registers (32bit) */

/* AFECRC CRC control register
 * Bit 31:28: Revision ID
 * Bit     9: Enable Apb32/Apb16 to get address/data for CRC calculation
 * Bit     4: Word16 swap enabled
 * Bit     3: Byte mirroring
 * Bit     2: Bit mirroring
 * Bit     1: LSB first calculation order
 * Bit     0: CRC peripheral enable
 */

#define AD5940_REG_AFECRC_CTL             0x1000        /* Address */
#define AD5940_BITP_AFECRC_CTL_REVID      28            /* Revision ID */
#define AD5940_BITP_AFECRC_CTL_MON_EN     9             /* Enable Apb32/16 */
#define AD5940_BITP_AFECRC_CTL_W16SWP     4             /* Word16 swap */
#define AD5940_BITP_AFECRC_CTL_BYTMIRR    3             /* Byte mirroring */
#define AD5940_BITP_AFECRC_CTL_BITMIRR    2             /* Bit mirroring */
#define AD5940_BITP_AFECRC_CTL_LSBFIRST   1             /* LSB first */
#define AD5940_BITP_AFECRC_CTL_EN         0             /* CRC peripheral */
#define AD5940_BITM_AFECRC_CTL_REVID      0xf0000000    /* Revision ID */
#define AD5940_BITM_AFECRC_CTL_MON_EN     0x00000200    /* Enable Apb32/16 */
#define AD5940_BITM_AFECRC_CTL_W16SWP     0x00000010    /* Word16 swap */
#define AD5940_BITM_AFECRC_CTL_BYTMIRR    0x00000008    /* Byte mirroring */
#define AD5940_BITM_AFECRC_CTL_BITMIRR    0x00000004    /* Bit mirroring */
#define AD5940_BITM_AFECRC_CTL_LSBFIRST   0x00000002    /* LSB first */
#define AD5940_BITM_AFECRC_CTL_EN         0x00000001    /* CRC peripheral */

/* AFECRC data input */

#define AD5940_REG_AFECRC_IPDATA          0x1004        /* Address */
#define AD5940_BITP_AFECRC_IPDATA_VALUE   0             /* Data input */
#define AD5940_BITM_AFECRC_IPDATA_VALUE   0xffffffff    /* Data input */

/* AFECRC CRC residue */

#define AD5940_REG_AFECRC_RESULT          0x1008        /* Address */
#define AD5940_BITP_AFECRC_RESULT_VALUE   0             /* CRC residue */
#define AD5940_BITM_AFECRC_RESULT_VALUE   0xffffffff    /* CRC residue */

/* AFECRC CRC reduction polynomial */

#define AD5940_REG_AFECRC_POLY            0x100c        /* Address */
#define AD5940_BITP_AFECRC_POLY_VALUE     0             /* Reduction poly */
#define AD5940_BITM_AFECRC_POLY_VALUE     0xffffffff    /* Reduction poly */

/* AFECRC input data bits */

#define AD5940_REG_AFECRC_IPBITS                 0x1010 /* Address */
#define AD5940_BITP_AFECRC_IPBITS_DATA_BITS      0      /* Input data bits */
#define AD5940_BITM_AFECRC_IPBITS_DATA_BITS      0xff   /* Input data bits */

/* AFECRC input data byte */

#define AD5940_REG_AFECRC_IPBYTE                 0x1014 /* Address */
#define AD5940_BITP_AFECRC_IPBYTE_DATA_BYTE      0      /* Input data byte */
#define AD5940_BITM_AFECRC_IPBYTE_DATA_BYTE      0xff   /* Input data byte */

/* AFECRC CRC signature compare data input. */

#define AD5940_REG_AFECRC_CRC_SIG_COMP           0x1020        /* Address */
#define AD5940_BITP_AFECRC_CRC_SIG_COMP_CRC_SIG  0             /* Position */
#define AD5940_BITM_AFECRC_CRC_SIG_COMP_CRC_SIG  0xffffffff    /* Mask */

/* AFECRC CRC error interrupt enable register
 * Bit 0: CRC error interrupt enable bit
 */

#define AD5940_REG_AFECRC_CRCINTEN               0x1024        /* Address */
#define AD5940_BITP_AFECRC_CRCINTEN_RESERVED     1             /* Reserved */
#define AD5940_BITP_AFECRC_CRCINTEN_CRC_ERR_EN   0             /* Position */
#define AD5940_BITM_AFECRC_CRCINTEN_RESERVED     0xfffffffe    /* Reserved */
#define AD5940_BITM_AFECRC_CRCINTEN_CRC_ERR_EN   0x00000001    /* Mask */

/* AFECRC CRC error interrupt status register
 * Bit 0: CRC error interrupt status bit
 */

#define AD5940_REG_AFECRC_INTSTA                 0x1028        /* Address */
#define AD5940_BITP_AFECRC_INTSTA_CRC_ERR_ST     0             /* Position */
#define AD5940_BITM_AFECRC_INTSTA_CRC_ERR_ST     0x00000001    /* Mask */

/* Part1.8 AFE registers (32bit) */

/* AFE AFE configuration
 * Bit 21: Enable DC DAC buffer
 * Bit 20: High speed DAC reference enable
 * Bit 19: Analog LDO current limiting enable
 * Bit 16: ADC output 50/60Hz filter enable
 * Bit 15: DFT hardware accelerator enable
 * Bit 14: Waveform generator enable
 * Bit 13: ADC temperature sensor convert enable
 * Bit 12: ADC temperature sensor channel enable
 * Bit 11: High power TIA enable
 * Bit 10: Enable excitation amplifier
 * Bit  9: Enable excitation buffer
 * Bit  8: ADC conversion start enable
 * Bit  7: ADC power enable
 * Bit  6: High power DAC enable
 * Bit  5: Disable high power reference
 */

#define AD5940_REG_AFE_AFECON                    0x2000     /* Address */
#define AD5940_REG_AFE_AFECON_RESET              0x00080000 /* Reset value */
#define AD5940_BITP_AFE_AFECON_DACBUFEN          21         /* Poistion */
#define AD5940_BITP_AFE_AFECON_DACREFEN          20         /* Poistion */
#define AD5940_BITP_AFE_AFECON_ALDOILIMITEN      19         /* Poistion */
#define AD5940_BITP_AFE_AFECON_SINC2EN           16         /* Poistion */
#define AD5940_BITP_AFE_AFECON_DFTEN             15         /* Poistion */
#define AD5940_BITP_AFE_AFECON_WAVEGENEN         14         /* Poistion */
#define AD5940_BITP_AFE_AFECON_TEMPCONVEN        13         /* Poistion */
#define AD5940_BITP_AFE_AFECON_TEMPSENSEN        12         /* Poistion */
#define AD5940_BITP_AFE_AFECON_TIAEN             11         /* Poistion */
#define AD5940_BITP_AFE_AFECON_INAMPEN           10         /* Poistion */
#define AD5940_BITP_AFE_AFECON_EXBUFEN           9          /* Poistion */
#define AD5940_BITP_AFE_AFECON_ADCCONVEN         8          /* Poistion */
#define AD5940_BITP_AFE_AFECON_ADCEN             7          /* Poistion */
#define AD5940_BITP_AFE_AFECON_DACEN             6          /* Poistion */
#define AD5940_BITP_AFE_AFECON_HPREFDIS          5          /* Poistion */
#define AD5940_BITM_AFE_AFECON_DACBUFEN          0x00200000 /* Mask */
#define AD5940_BITM_AFE_AFECON_DACREFEN          0x00100000 /* Mask */
#define AD5940_BITM_AFE_AFECON_ALDOILIMITEN      0x00080000 /* Mask */
#define AD5940_BITM_AFE_AFECON_SINC2EN           0x00010000 /* Mask */
#define AD5940_BITM_AFE_AFECON_DFTEN             0x00008000 /* Mask */
#define AD5940_BITM_AFE_AFECON_WAVEGENEN         0x00004000 /* Mask */
#define AD5940_BITM_AFE_AFECON_TEMPCONVEN        0x00002000 /* Mask */
#define AD5940_BITM_AFE_AFECON_TEMPSENSEN        0x00001000 /* Mask */
#define AD5940_BITM_AFE_AFECON_TIAEN             0x00000800 /* Mask */
#define AD5940_BITM_AFE_AFECON_INAMPEN           0x00000400 /* Mask */
#define AD5940_BITM_AFE_AFECON_EXBUFEN           0x00000200 /* Mask */
#define AD5940_BITM_AFE_AFECON_ADCCONVEN         0x00000100 /* Mask */
#define AD5940_BITM_AFE_AFECON_ADCEN             0x00000080 /* Mask */
#define AD5940_BITM_AFE_AFECON_DACEN             0x00000040 /* Mask */
#define AD5940_BITM_AFE_AFECON_HPREFDIS          0x00000020 /* Mask */
#define AD5940_ENUM_AFE_AFECON_OFF               0x00000000 /* DACEN = 0 */
#define AD5940_ENUM_AFE_AFECON_ON                0x00000040 /* DACEN = 1 */

/* AFE sequencer configuration
 * Bit 15:8: Timer for sequencer write commands
 * Bit    4: Halt sequencer
 * Bit    1: Halt sequencer if empty
 * Bit    0: Enable sequencer
 */

#define AD5940_REG_AFE_SEQCON                    0x2004     /* Address */
#define AD5940_REG_AFE_SEQCON_RESET              0x00000002 /* Reset value */
#define AD5940_BITP_AFE_SEQCON_SEQWRTMR          8          /* Timer for write */
#define AD5940_BITP_AFE_SEQCON_SEQHALT           4          /* Halt */
#define AD5940_BITP_AFE_SEQCON_SEQHALTFIFOEMPTY  1          /* Halt if empty */
#define AD5940_BITP_AFE_SEQCON_SEQEN             0          /* Enable sequencer */
#define AD5940_BITM_AFE_SEQCON_SEQWRTMR          0x0000ff00 /* Timer for write */
#define AD5940_BITM_AFE_SEQCON_SEQHALT           0x00000010 /* Halt */
#define AD5940_BITM_AFE_SEQCON_SEQHALTFIFOEMPTY  0x00000002 /* Halt if empty */
#define AD5940_BITM_AFE_SEQCON_SEQEN             0x00000001 /* Enable sequencer */

/* AFE FIFOs configuration
 * Bit 15:13: Selects the source for the data FIFO
 * Bit    11: Data FIFO enable
 */

#define AD5940_REG_AFE_FIFOCON                   0x2008     /* Address */
#define AD5940_REG_AFE_FIFOCON_RESET             0x00001010 /* Reset value */
#define AD5940_BITP_AFE_FIFOCON_DATAFIFOSRCSEL   13         /* Selects SRC */
#define AD5940_BITP_AFE_FIFOCON_DATAFIFOEN       11         /* Data FIFO EN */
#define AD5940_BITM_AFE_FIFOCON_DATAFIFOSRCSEL   0x0000e000 /* Selects SRC */
#define AD5940_BITM_AFE_FIFOCON_DATAFIFOEN       0x00000800 /* Data FIFO EN */

/* AFE switch matrix configuration
 * Bit    19: Control of T[11]
 * Bit    18: Control of T[10]
 * Bit    17: Control of T[9]
 * Bit    16: Switch control select
 * Bit 15:12: Control of T switch MUX
 * Bit  11:8: Control of N switch MUX
 * Bit   7:4: Control of P switch MUX
 * Bit   3:0: Control of D switch MUX
 */

#define AD5940_REG_AFE_SWCON                     0x200c     /* Address */
#define AD5940_REG_AFE_SWCON_RESET               0x0000ffff /* Reset value*/
#define AD5940_BITP_AFE_SWCON_T11CON             19         /* Ctrl T[11] */
#define AD5940_BITP_AFE_SWCON_T10CON             18         /* Ctrl T[10] */
#define AD5940_BITP_AFE_SWCON_T9CON              17         /* Ctrl T[9] */
#define AD5940_BITP_AFE_SWCON_SWSOURCESEL        16         /* Switch SEL */
#define AD5940_BITP_AFE_SWCON_TMUXCON            12         /* T Switch MUX */
#define AD5940_BITP_AFE_SWCON_NMUXCON            8          /* N Switch MUX */
#define AD5940_BITP_AFE_SWCON_PMUXCON            4          /* P Switch MUX */
#define AD5940_BITP_AFE_SWCON_DMUXCON            0          /* D Switch MUX */
#define AD5940_BITM_AFE_SWCON_T11CON             0x00080000 /* Ctrl T[11] */
#define AD5940_BITM_AFE_SWCON_T10CON             0x00040000 /* Ctrl T[10] */
#define AD5940_BITM_AFE_SWCON_T9CON              0x00020000 /* Ctrl T[9] */
#define AD5940_BITM_AFE_SWCON_SWSOURCESEL        0x00010000 /* Switch SEL */
#define AD5940_BITM_AFE_SWCON_TMUXCON            0x0000f000 /* T Switch MUX. */
#define AD5940_BITM_AFE_SWCON_NMUXCON            0x00000f00 /* N Switch MUX */
#define AD5940_BITM_AFE_SWCON_PMUXCON            0x000000f0 /* P Switch MUX */
#define AD5940_BITM_AFE_SWCON_DMUXCON            0x0000000f /* D Switch MUX */

/* AFE high speed DAC configuration
 * Bit  12: Excitation amplifier gain control
 * Bit 8:1: DAC update rate
 * Bit   0: PGA stage gain attenuation
 */

#define AD5940_REG_AFE_HSDACCON                  0x2010     /* Address */
#define AD5940_REG_AFE_HSDACCON_RESET            0x0000001e /* Reset value */
#define AD5940_BITP_AFE_HSDACCON_INAMPGNMDE      12         /* Amp gain */
#define AD5940_BITP_AFE_HSDACCON_RATE            1          /* DAC rate */
#define AD5940_BITP_AFE_HSDACCON_ATTENEN         0          /* PGA gain */
#define AD5940_BITM_AFE_HSDACCON_INAMPGNMDE      0x00001000 /* Amp gain */
#define AD5940_BITM_AFE_HSDACCON_RATE            0x000001fe /* DAC rate */
#define AD5940_BITM_AFE_HSDACCON_ATTENEN         0x00000001 /* PGA gain */

/* AFE waveform generator configuration
 * Bit   5: Bypass DAC gain
 * Bit   4: Bypass DAC offset
 * Bit 2:1: Selects the type of waveform
 * Bit   0: Resets the trapezoid waveform generator
 */

#define AD5940_REG_AFE_WGCON                     0x2014     /* Address */
#define AD5940_REG_AFE_WGCON_RESET               0x00000030 /* Reset value */
#define AD5940_BITP_AFE_WGCON_DACGAINCAL         5          /* Position */
#define AD5940_BITP_AFE_WGCON_DACOFFSETCAL       4          /* Position */
#define AD5940_BITP_AFE_WGCON_TYPESEL            1          /* Position */
#define AD5940_BITP_AFE_WGCON_TRAPRSTEN          0          /* Position */
#define AD5940_BITM_AFE_WGCON_DACGAINCAL         0x00000020 /* Mask */
#define AD5940_BITM_AFE_WGCON_DACOFFSETCAL       0x00000010 /* Mask */
#define AD5940_BITM_AFE_WGCON_TYPESEL            0x00000006 /* Mask */
#define AD5940_BITM_AFE_WGCON_TRAPRSTEN          0x00000001 /* Mask */

/* AFE waveform generator - trapezoid DC level register 1
 * Bit 11:0:  DC level 1 value for trapezoid waveform generation
 */

#define AD5940_REG_AFE_WGDCLEVEL1                0x2018     /* Address */
#define AD5940_REG_AFE_WGDCLEVEL1_RESET          0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGDCLEVEL1_TRAPDCLEVEL1  0          /* DC level 1 */
#define AD5940_BITM_AFE_WGDCLEVEL1_TRAPDCLEVEL1  0x00000fff /* DC level 1 */

/* AFE waveform generator - trapezoid DC level register 2
 * Bit 11:0:  DC level 2 value for trapezoid waveform generation
 */

#define AD5940_REG_AFE_WGDCLEVEL2                0x201c     /* Address */
#define AD5940_REG_AFE_WGDCLEVEL2_RESET          0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGDCLEVEL2_TRAPDCLEVEL2  0          /* DC level 2 */
#define AD5940_BITM_AFE_WGDCLEVEL2_TRAPDCLEVEL2  0x00000fff /* DC level 2 */

/* AFE waveform generator - trapezoid delay 1 time register
 * Bit 15:0: Delay 1 value for trapezoid waveform generation
 */

#define AD5940_REG_AFE_WGDELAY1                  0x2020     /* Address */
#define AD5940_REG_AFE_WGDELAY1_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGDELAY1_DELAY1          0          /* Delay 1 VAL */
#define AD5940_BITM_AFE_WGDELAY1_DELAY1          0x0000ffff /* Delay 1 VAL */

/* AFE waveform generator - trapezoid slope 1 time register
 * Bit 15:0: Slope 1 value for trapezoid waveform generation
 */

#define AD5940_REG_AFE_WGSLOPE1                  0x2024     /* Address */
#define AD5940_REG_AFE_WGSLOPE1_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGSLOPE1_SLOPE1          0          /* Slope 1 VAL */
#define AD5940_BITM_AFE_WGSLOPE1_SLOPE1          0x0000ffff /* Slope 1 VAL */

/* AFE waveform generator - trapezoid delay 2 time register
 * Bit 15:0: Delay 1 value for trapezoid waveform generation
 */

#define AD5940_REG_AFE_WGDELAY2                  0x2028     /* Address */
#define AD5940_REG_AFE_WGDELAY2_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGDELAY2_DELAY2          0          /* Delay 2 VAL */
#define AD5940_BITM_AFE_WGDELAY2_DELAY2          0x0000ffff /* Delay 2 VAL */

/* AFE waveform generator - trapezoid slope 2 time register
 * Bit 15:0: Slope 2 value for trapezoid waveform generation
 */

#define AD5940_REG_AFE_WGSLOPE2                  0x202c     /* Address */
#define AD5940_REG_AFE_WGSLOPE2_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGSLOPE2_SLOPE2          0          /* Slope 2 VAL */
#define AD5940_BITM_AFE_WGSLOPE2_SLOPE2          0x0000ffff /* Slope 2 VAL */

/* AFE waveform generator - sinusoid frequency control word register
 * Bit 23:0: Sinusoid generator frequency control word
 */

#define AD5940_REG_AFE_WGFCW                     0x2030     /* Address */
#define AD5940_REG_AFE_WGFCW_RESET               0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGFCW_SINEFCW            0          /* Freq ctrl */
#define AD5940_BITM_AFE_WGFCW_SINEFCW            0x00ffffff /* Freq ctrl */

/* AFE Waveform generator - sinusoid phase offset register
 * Bit 19:0: Sinusoid phase offset
 */

#define AD5940_REG_AFE_WGPHASE                   0x2034     /* Address */
#define AD5940_REG_AFE_WGPHASE_RESET             0x00000000 /* RST value */
#define AD5940_BITP_AFE_WGPHASE_SINEOFFSET       0          /* Position */
#define AD5940_BITM_AFE_WGPHASE_SINEOFFSET       0x000fffff /* Mask */

/* AFE waveform generator - sinusoid offset register
 * Bit 11:0: Sinusoid offset
 */

#define AD5940_REG_AFE_WGOFFSET                  0x2038     /* Address */
#define AD5940_REG_AFE_WGOFFSET_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_WGOFFSET_SINEOFFSET      0          /* Sin Offset */
#define AD5940_BITM_AFE_WGOFFSET_SINEOFFSET      0x00000fff /* Sin Offset */

/* AFE waveform generator - sinusoid amplitude */

#define AD5940_REG_AFE_WGAMPLITUDE               0x203c     /* Address */
#define AD5940_REG_AFE_WGAMPLITUDE_RESET         0x00000000 /* RST value */
#define AD5940_BITP_AFE_WGAMPLITUDE_SINAMPLITUDE 0          /* Amplitude */
#define AD5940_BITM_AFE_WGAMPLITUDE_SINAMPLITUDE 0x000007ff /* Amplitude */

/* AFE ADC output filters configuration
 * Bit 15:14: Number of samples averaged
 * Bit 13:12: SINC3 OSR
 * Bit  11:8: SINC2 OSR
 * Bit     7: Average function enable
 * Bit     6: SINC3 filter bypass
 * Bit     4: 50/60Hz low pass filter
 * Bit     0: ADC data rate
 */

#define AD5940_REG_AFE_ADCFILTERCON              0x2044     /* Address */
#define AD5940_REG_AFE_ADCFILTERCON_RESET        0x00000301 /* Reset value */
#define AD5940_BITP_AFE_ADCFILTERCON_AVRGNUM     14         /* AVEG samples */
#define AD5940_BITP_AFE_ADCFILTERCON_SINC3OSR    12         /* SINC3 OSR */
#define AD5940_BITP_AFE_ADCFILTERCON_SINC2OSR    8          /* SINC2 OSR */
#define AD5940_BITP_AFE_ADCFILTERCON_AVRGEN      7          /* AVEG enable */
#define AD5940_BITP_AFE_ADCFILTERCON_SINC3BYP    6          /* SINC3 bypass */
#define AD5940_BITP_AFE_ADCFILTERCON_LPFBYPEN    4          /* 50/60Hz LPF */
#define AD5940_BITP_AFE_ADCFILTERCON_ADCCLK      0          /* ADC Rate */
#define AD5940_BITM_AFE_ADCFILTERCON_AVRGNUM     0x0000c000 /* AVEG samples */
#define AD5940_BITM_AFE_ADCFILTERCON_SINC3OSR    0x00003000 /* SINC3 OSR */
#define AD5940_BITM_AFE_ADCFILTERCON_SINC2OSR    0x00000f00 /* SINC2 OSR */
#define AD5940_BITM_AFE_ADCFILTERCON_AVRGEN      0x00000080 /* AVEG enable */
#define AD5940_BITM_AFE_ADCFILTERCON_SINC3BYP    0x00000040 /* SINC3 bypass */
#define AD5940_BITM_AFE_ADCFILTERCON_LPFBYPEN    0x00000010 /* 50/60Hz LPF */
#define AD5940_BITM_AFE_ADCFILTERCON_ADCCLK      0x00000001 /* ADC Rate */

/* AFE HS DAC code */

#define AD5940_REG_AFE_HSDACDAT                  0x2048     /* Address */
#define AD5940_REG_AFE_HSDACDAT_RESET            0x00000800 /* Reset value */
#define AD5940_BITP_AFE_HSDACDAT_DACDAT          0          /* DAC code */
#define AD5940_BITM_AFE_HSDACDAT_DACDAT          0x00000fff /* DAC code */

/* AFE low power reference control register
 * Bit 2: Set: drive 2 DACs; Unset: drive 1 DAC, and save power
 * Bit 1: Low power bandgap's output buffer
 * Bit 0: Set this bit will power down low power bandgap
 */

#define AD5940_REG_AFE_LPREFBUFCON               0x2050     /* Address */
#define AD5940_REG_AFE_LPREFBUFCON_RESET         0x00000000 /* Reset value */
#define AD5940_BITP_AFE_LPREFBUFCON_BOOSTCURRENT 2          /* 1 or 2 DAC */
#define AD5940_BITP_AFE_LPREFBUFCON_LPBUF2P5DIS  1          /* LP buffer */
#define AD5940_BITP_AFE_LPREFBUFCON_LPREFDIS     0          /* Ctrl bandgap */
#define AD5940_BITM_AFE_LPREFBUFCON_BOOSTCURRENT 0x00000004 /* 1 or 2 DAC */
#define AD5940_BITM_AFE_LPREFBUFCON_LPBUF2P5DIS  0x00000002 /* LP buffer */
#define AD5940_BITM_AFE_LPREFBUFCON_LPREFDIS     0x00000001 /* Ctrl bandgap */

/* AFE SYNC external devices
 * Bit [7:0] are output data of the GPIOx. Writing 1 to the corresponding bit
 * sets the corresponding GPIOx high. Writing 0 sets the corresponding GPIOx
 * to 0.
 */

#define AD5940_REG_AFE_SYNCEXTDEVICE             0x2054     /* Address */
#define AD5940_REG_AFE_SYNCEXTDEVICE_RESET       0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SYNCEXTDEVICE_SYNC       0          /* GPIOx output */
#define AD5940_BITM_AFE_SYNCEXTDEVICE_SYNC       0x000000ff /* GPIOx output */

/* AFE Sequencer CRC Value register
 * Bit 0: Sequencer command CRC value.
 */

#define AD5940_REG_AFE_SEQCRC                    0x2060     /* SEQ CRC */
#define AD5940_REG_AFE_SEQCRC_RESET              0x00000001 /* Reset value */
#define AD5940_BITP_AFE_SEQCRC_CRC               0          /* SEQ cmd CRC */
#define AD5940_BITM_AFE_SEQCRC_CRC               0x000000ff /* SEQ cmd CRC */

/* AFE sequencer command count register
 * Bit 15:0: Sequencer command count
 */

#define AD5940_REG_AFE_SEQCNT                    0x2064     /* SEQ CMD cnt */
#define AD5940_REG_AFE_SEQCNT_RESET              0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SEQCNT_COUNT             0          /* SEQ CMD cnt */
#define AD5940_BITM_AFE_SEQCNT_COUNT             0x0000ffff /* SEQ CMD cnt */

/* AFE Sequencer timeout counter register
 * Bit 30:0: Current value of the sequencer timeout counter.
 */

#define AD5940_REG_AFE_SEQTIMEOUT                0x2068     /* Address */
#define AD5940_REG_AFE_SEQTIMEOUT_RESET          0x00000000 /* RST value */
#define AD5940_BITP_AFE_SEQTIMEOUT_TIMEOUT       0          /* Position */
#define AD5940_BITM_AFE_SEQTIMEOUT_TIMEOUT       0x3fffffff /* Mask */

/* AFE data FIFO read register
 * Bit 15:0: Data FIFO read
 */

#define AD5940_REG_AFE_DATAFIFORD                0x206cU    /* Data FIFO RD */
#define AD5940_REG_AFE_DATAFIFORD_RESET          0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DATAFIFORD_DATAFIFOOUT   0          /* Data FIFO RD */
#define AD5940_BITM_AFE_DATAFIFORD_DATAFIFOOUT   0x0000ffff /* Data FIFO RD */

/* AFE command FIFO write register
 * Bit 31:0:  Command FIFO Write.
 */

#define AD5940_REG_AFE_CMDFIFOWRITE              0x2070     /* Address */
#define AD5940_REG_AFE_CMDFIFOWRITE_RESET        0x00000000 /* RST value */
#define AD5940_BITP_AFE_CMDFIFOWRITE_CMDFIFOIN   0          /* Position */
#define AD5940_BITM_AFE_CMDFIFOWRITE_CMDFIFOIN   0xffffffff /* CMD FIFO WR */

/* AFE ADC raw result */

#define AD5940_REG_AFE_ADCDAT                    0x2074     /* Address */
#define AD5940_REG_AFE_ADCDAT_RESET              0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCDAT_DATA              0          /* ADC Result */
#define AD5940_BITM_AFE_ADCDAT_DATA              0x0000ffff /* ADC Result */

/* AFE DFT result, real part */

#define AD5940_REG_AFE_DFTREAL                   0x2078     /* DFT result r */
#define AD5940_REG_AFE_DFTREAL_RESET             0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DFTREAL_DATA             0          /* DFT real */
#define AD5940_BITM_AFE_DFTREAL_DATA             0x0003ffff /* DFT real */

/* AFE DFT result, imaginary part */

#define AD5940_REG_AFE_DFTIMAG                   0x207c     /* DFT result i */
#define AD5940_REG_AFE_DFTIMAG_RESET             0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DFTIMAG_DATA             0          /* DFT imag */
#define AD5940_BITM_AFE_DFTIMAG_DATA             0x0003ffff /* DFT imag */

/* AFE supply rejection filter result */

#define AD5940_REG_AFE_SINC2DAT                  0x2080     /* Address */
#define AD5940_REG_AFE_SINC2DAT_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SINC2DAT_DATA            0          /* LPF result */
#define AD5940_BITM_AFE_SINC2DAT_DATA            0x0000ffff /* LPF result */

/* AFE temperature sensor result */

#define AD5940_REG_AFE_TEMPSENSDAT               0x2084     /* Address */
#define AD5940_REG_AFE_TEMPSENSDAT_RESET         0x00000000 /* Reset value */
#define AD5940_BITP_AFE_TEMPSENSDAT_DATA         0          /* Temp sensor */
#define AD5940_BITM_AFE_TEMPSENSDAT_DATA         0x0000ffff /* Temp sensor */

/* AFE analog generation interrupt */

#define AD5940_REG_AFE_AFEGENINTSTA              0x209c     /* Address */
#define AD5940_REG_AFE_AFEGENINTSTA_RESET        0x00000000 /* Reset value */
#define AD5940_BITP_AFE_AFEGENINTSTA_CUSTOMIRQ3  3          /* Custom IRQ 3 */
#define AD5940_BITP_AFE_AFEGENINTSTA_CUSTOMIRQ2  2          /* Custom IRQ 2 */
#define AD5940_BITP_AFE_AFEGENINTSTA_CUSTOMIRQ1  1          /* Custom IRQ 1 */
#define AD5940_BITP_AFE_AFEGENINTSTA_CUSTOMIRQ0  0          /* Custom IRQ 0 */
#define AD5940_BITM_AFE_AFEGENINTSTA_CUSTOMIRQ3  0x00000008 /* Custom IRQ 3 */
#define AD5940_BITM_AFE_AFEGENINTSTA_CUSTOMIRQ2  0x00000004 /* Custom IRQ 2 */
#define AD5940_BITM_AFE_AFEGENINTSTA_CUSTOMIRQ1  0x00000002 /* Custom IRQ 1 */
#define AD5940_BITM_AFE_AFEGENINTSTA_CUSTOMIRQ0  0x00000001 /* Custom IRQ 0 */

/* AFE ADC minimum value check register
 * Bit 15:0: ADC minimum value threshold
 */

#define AD5940_REG_AFE_ADCMIN                    0x20a8     /* Address */
#define AD5940_REG_AFE_ADCMIN_RESET              0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCMIN_MINVAL            0          /* Min. threshd */
#define AD5940_BITM_AFE_ADCMIN_MINVAL            0x0000ffff /* Min. threshd */

/* AFE ADCMIN hysteresis value register
 * Bit 15:0: Hysteresis value
 */

#define AD5940_REG_AFE_ADCMINSM                  0x20ac     /* Address */
#define AD5940_REG_AFE_ADCMINSM_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCMINSM_MINCLRVAL       0          /* Hysteresis */
#define AD5940_BITM_AFE_ADCMINSM_MINCLRVAL       0x0000ffff /* Hysteresis */

/* AFE ADC maximum value check register
 * Bit 15:0: ADC maximum Threshold
 */

#define AD5940_REG_AFE_ADCMAX                    0x20b0     /* Address */
#define AD5940_REG_AFE_ADCMAX_RESET              0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCMAX_MAXVAL            0          /* Max. threshd */
#define AD5940_BITM_AFE_ADCMAX_MAXVAL            0x0000ffff /* Max. threshd */

/* AFE ADCMAX hysteresis value
 * Bit 15:0: ADCMAX hysteresis value
 */

#define AD5940_REG_AFE_ADCMAXSMEN                0x20b4     /* Address */
#define AD5940_REG_AFE_ADCMAXSMEN_RESET          0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCMAXSMEN_MAXSWEN       0          /* Hysteresis */
#define AD5940_BITM_AFE_ADCMAXSMEN_MAXSWEN       0x0000ffff /* Hysteresis */

/* AFE ADC delta value
 * Bit 15:0: ADCDAT code differences limit option
 */

#define AD5940_REG_AFE_ADCDELTA                  0x20b8     /* ADC delta */
#define AD5940_REG_AFE_ADCDELTA_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCDELTA_DELTAVAL        0          /* Diff limit */
#define AD5940_BITM_AFE_ADCDELTA_DELTAVAL        0x0000ffff /* diff limit */

/* AFE HPOSC Configuration
 * Bit 2: 16M/32M output selector signal.
 */

#define AD5940_REG_AFE_HPOSCCON                  0x20bc     /* Address */
#define AD5940_REG_AFE_HPOSCCON_RESET            0x00000024 /* Reset value */
#define AD5940_BITP_AFE_HPOSCCON_CLK32MHZEN      2          /* 16M/32M SEL */
#define AD5940_BITM_AFE_HPOSCCON_CLK32MHZEN      0x00000004 /* 16M/32M SEL */

/* AFE DSP configuration register */

#define AD5940_REG_AFE_DFTCON                    0x20d0     /* Address */
#define AD5940_REG_AFE_DFTCON_RESET              0x00000090 /* Reset value */
#define AD5940_BITP_AFE_DFTCON_DFTINSEL          20         /* DFT input */
#define AD5940_BITP_AFE_DFTCON_DFTNUM            4          /* ADC samples */
#define AD5940_BITP_AFE_DFTCON_HANNINGEN         0          /* Hanning EN */
#define AD5940_BITM_AFE_DFTCON_DFTINSEL          0x00300000 /* DFT input */
#define AD5940_BITM_AFE_DFTCON_DFTNUM            0x000000f0 /* ADC samples */
#define AD5940_BITM_AFE_DFTCON_HANNINGEN         0x00000001 /* Hanning EN */

/* AFE ULPTIA switch configuration for channel 0/1 registers
 * Bit X controls SWX switch, active high. For bit[11:0]:
 * 0x014: CAPA test with LP TIA
 * 0x02c: Normal work mode with back-back diode enabled
 * 0x02d: Normal work mode with back-back diode enabled.
 * 0x02e: Work mode with short switch protection
 * 0x06c: Work mode, vzero-vbias = 0.
 * 0x094: CAPA test or Ramp test with HP TIA
 * 0x180: Set PA/TIA as unity gain buffer.
 * 0x1a4: Set PA/TIA as unity gain buffer. Connect amp's output to CE0 & RC01
 * 0x42c: Two lead sensor, set PA as unity gain buffer.
 * 0x4a4: Set PA/TIA as unity gain buffer.
 * 0x800: Close SW11 - Short SE0 to RE0.
 */

#define AD5940_REG_AFE_LPTIASW0                  0x20e4     /* SW0 ADDR */
#define AD5940_REG_AFE_LPTIASW1                  0x20e0     /* SW1 ADDR */
#define AD5940_REG_AFE_LPTIASW0_RESET            0x00000000 /* RST value */
#define AD5940_BITP_AFE_LPTIASW0_RECAL           15         /* TIA SW15 */
#define AD5940_BITP_AFE_LPTIASW0_VZEROSHARE      14         /* TIA SW14 */
#define AD5940_BITP_AFE_LPTIASW0_TIABIASSEL      13         /* TIA SW13 */
#define AD5940_BITP_AFE_LPTIASW0_PABIASSEL       12         /* TIA SW12 */
#define AD5940_BITP_AFE_LPTIASW0_TIASWCON        0          /* SW[11:0] */
#define AD5940_BITM_AFE_LPTIASW0_RECAL           0x00008000 /* TIA SW15 */
#define AD5940_BITM_AFE_LPTIASW0_VZEROSHARE      0x00004000 /* TIA SW14 */
#define AD5940_BITM_AFE_LPTIASW0_TIABIASSEL      0x00002000 /* TIA SW13 */
#define AD5940_BITM_AFE_LPTIASW0_PABIASSEL       0x00001000 /* TIA SW12 */
#define AD5940_BITM_AFE_LPTIASW0_TIASWCON        0x00000fff /* SW[11:0] */
#define AD5940_ENUM_AFE_LPTIASW0_11              0x00000014 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_NORM            0x0000002c /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_DIO             0x0000002d /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_SHORTSW         0x0000002e /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_LOWNOISE        0x0000006c /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_1               0x00000094 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_BUFDIS          0x00000180 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_BUFEN           0x000001a4 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_TWOLEAD         0x0000042c /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_BUFEN2          0x000004a4 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW0_SESHORTRE       0x00000800 /* TIASWCON */
#define AD5940_BITP_AFE_LPTIASW1_TIABIASSEL      13         /* TIA SW13 */
#define AD5940_BITP_AFE_LPTIASW1_PABIASSEL       12         /* TIA SW12 */
#define AD5940_BITP_AFE_LPTIASW1_TIASWCON        0          /* SW[11:0] */
#define AD5940_BITM_AFE_LPTIASW1_TIABIASSEL      0x00002000 /* TIA SW13 */
#define AD5940_BITM_AFE_LPTIASW1_PABIASSEL       0x00001000 /* TIA SW12 */
#define AD5940_BITM_AFE_LPTIASW1_TIASWCON        0x00000fff /* SW[11:0] */
#define AD5940_ENUM_AFE_LPTIASW1_CAPA_LP         0x00000014 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_NORM            0x0000002c /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_DIO             0x0000002d /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_SHORTSW         0x0000002e /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_LOWNOISE        0x0000006c /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_CAPA_RAMP_H     0x00000094 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_BUFDIS          0x00000180 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_BUFEN           0x000001a4 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_TWOLEAD         0x0000042c /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_BUFEN2          0x000004a4 /* TIASWCON */
#define AD5940_ENUM_AFE_LPTIASW1_SESHORTRE       0x00000800 /* TIASWCON */

/* AFE ULPTIA Control Bits Channel 0/1 registers
 * Bit    16: Chopping enable
 * Bit 15:13: Set LPF resistor (TIARF)
 *            0 << 13 - Disconnect TIA output from LPF pin
 *            1 << 13 - Bypass resistor
 *            2 << 13 - 20k Ohm  |  5 << 13 - 400k Ohm
 *            3 << 13 - 100k Ohm |  6 << 13 - 600k Ohm
 *            4 << 13 - 200k Ohm |  7 << 13 - 1Meg Ohm
 * Bit 12:10: Set RLOAD (TIARL)
 *            0 << 10 - 0 Ohm    |  4 << 10 - 100 Ohm
 *            1 << 10 - 10 Ohm   |  5 << 10 - 1.6K Ohm
 *            2 << 10 - 30 Ohm   |  6 << 10 - 3.1 Ohm
 *            3 << 10 - 50 Ohm   |  7 << 10 - 3.6 Ohm
 * Bit   9:5: Set RTIA gain resistor (TIAGAIN)
 *            0 << 5 - Disconnect TIA Gain resistor
 *            1 << 5 - 200 Ohm  |  10 << 5 - 16K Ohm   |  19 << 5 - 96K Ohm
 *            2 << 5 - 1K Ohm   |  11 << 5 - 20K Ohm   |  20 << 5 - 100K Ohm
 *            3 << 5 - 2K Ohm   |  12 << 5 - 24K Ohm   |  21 << 5 - 120K Ohm
 *            4 << 5 - 3K Ohm   |  13 << 5 - 30K Ohm   |  22 << 5 - 128K Ohm
 *            5 << 5 - 4K Ohm   |  14 << 5 - 32K Ohm   |  23 << 5 - 160K Ohm
 *            6 << 5 - 6K Ohm   |  15 << 5 - 40K Ohm   |  24 << 5 - 196K Ohm
 *            7 << 5 - 8K Ohm   |  16 << 5 - 48K Ohm   |  25 << 5 - 256K Ohm
 *            8 << 5 - 10K Ohm  |  17 << 5 - 64K Ohm   |  26 << 5 - 512K Ohm
 *            9 << 5 - 12K Ohm  |  18 << 5 - 85K Ohm   |
 * Bit   4:3: Current boost control
 * Bit     2: Half power mode select
 * Bit     1: PA power down
 * Bit     0: TIA power down
 */

#define AD5940_REG_AFE_LPTIACON0                 0x20ec     /* Address 0 */
#define AD5940_REG_AFE_LPTIACON1                 0x20e8     /* Address 1 */
#define AD5940_REG_AFE_LPTIACON0_RESET           0x00000003 /* Reset value */
#define AD5940_BITP_AFE_LPTIACONX_CHOPEN         16         /* Chopping */
#define AD5940_BITP_AFE_LPTIACONX_TIARF          13         /* LPF resistor */
#define AD5940_BITP_AFE_LPTIACONX_TIARL          10         /* Set RLOAD */
#define AD5940_BITP_AFE_LPTIACONX_TIAGAIN        5          /* Set RTIA */
#define AD5940_BITP_AFE_LPTIACONX_IBOOST         3          /* Current */
#define AD5940_BITP_AFE_LPTIACONX_HALFPWR        2          /* Power mode */
#define AD5940_BITP_AFE_LPTIACONX_PAPDEN         1          /* PA PWDN */
#define AD5940_BITP_AFE_LPTIACONX_TIAPDEN        0          /* TIA PWDN */
#define AD5940_BITM_AFE_LPTIACONX_CHOPEN         0x00030000 /* Chopping */
#define AD5940_BITM_AFE_LPTIACONX_TIARF          0x0000e000 /* LPF resistor */
#define AD5940_BITM_AFE_LPTIACONX_TIARL          0x00001c00 /* Set RLOAD */
#define AD5940_BITM_AFE_LPTIACONX_TIAGAIN        0x000003e0 /* Set RTIA */
#define AD5940_BITM_AFE_LPTIACONX_IBOOST         0x00000018 /* Current */
#define AD5940_BITM_AFE_LPTIACONX_HALFPWR        0x00000004 /* Power mode */
#define AD5940_BITM_AFE_LPTIACONX_PAPDEN         0x00000002 /* PA PWDN */
#define AD5940_BITM_AFE_LPTIACONX_TIAPDEN        0x00000001 /* TIA PWDN */
#define AD5940_ENUM_AFE_LPTIACONX_DISCONRF       0x00000000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_BYPRF          0x00002000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_RF20K          0x00004000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_RF100K         0x00006000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_RF200K         0x00008000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_RF400K         0x0000a000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_RF600K         0x0000c000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_RF1MOHM        0x0000e000 /* TIARF */
#define AD5940_ENUM_AFE_LPTIACONX_RL0            0x00000000 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_RL10           0x00000400 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_RL30           0x00000800 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_RL50           0x00000c00 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_RL100          0x00001000 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_RL1p6K         0x00001400 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_RL3p1K         0x00001800 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_RL3p5K         0x00001c00 /* TIARL */
#define AD5940_ENUM_AFE_LPTIACONX_DISCONTIA      0x00000000 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN200     0x00000020 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN1K      0x00000040 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN2K      0x00000060 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN3K      0x00000080 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN4K      0x000000a0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN6K      0x000000c0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN8K      0x000000e0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN10K     0x00000100 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN12K     0x00000120 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN16K     0x00000140 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN20K     0x00000160 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN24K     0x00000180 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN30K     0x000001a0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN32K     0x000001c0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN40K     0x000001e0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN48K     0x00000200 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN64K     0x00000220 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN85K     0x00000240 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN96K     0x00000260 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN100K    0x00000280 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN120K    0x000002a0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN128K    0x000002c0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN160K    0x000002e0 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN196K    0x00000300 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN256K    0x00000320 /* TIAGAIN */
#define AD5940_ENUM_AFE_LPTIACONX_TIAGAIN512K    0x00000340 /* TIAGAIN */

/* AFE high power RTIA configuration
 * Bit 12:5: Configure capacitor in parallel with RTIA
 * Bit    4: SW6 control
 * Bit  3:0: Configure general RTIA value
 */

#define AD5940_REG_AFE_HSRTIACON                 0x20f0     /* Address */
#define AD5940_REG_AFE_HSRTIACON_RESET           0x0000000f /* Reset value */
#define AD5940_BITP_AFE_HSRTIACON_CTIACON        5          /* Capacitor */
#define AD5940_BITP_AFE_HSRTIACON_TIASW6CON      4          /* SW6 Control */
#define AD5940_BITP_AFE_HSRTIACON_RTIACON        0          /* RTIA Value */
#define AD5940_BITM_AFE_HSRTIACON_CTIACON        0x00001fe0 /* Capacitor */
#define AD5940_BITM_AFE_HSRTIACON_TIASW6CON      0x00000010 /* SW6 Control */
#define AD5940_BITM_AFE_HSRTIACON_RTIACON        0x0000000f /* RTIA Value */

/* AFE DE0/1 HSTIA resistors configuration */

#define AD5940_REG_AFE_DE0RESCON                 0x20f8     /* Address 0*/
#define AD5940_REG_AFE_DE1RESCON                 0x20f4     /* Address 1 */
#define AD5940_REG_AFE_DE0RESCON_RESET           0x000000ff /* Reset value */
#define AD5940_BITP_AFE_DEXRESCON_DE0RCON        0          /* RLOAD RTIA */
#define AD5940_BITM_AFE_DEXRESCON_DE0RCON        0x000000ff /* RLOAD RTIA */

/* AFE HSTIA amplifier configuration
 * Bit 1:0: Select HSTIA positive input
 */

#define AD5940_REG_AFE_HSTIACON                  0x20fc     /* Address */
#define AD5940_REG_AFE_HSTIACON_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_HSTIACON_VBIASSEL        0          /* SEL positive */
#define AD5940_BITM_AFE_HSTIACON_VBIASSEL        0x00000003 /* SEL positive */

/* AFE LP mode AFE control lock */

#define AD5940_REG_AFE_LPMODEKEY                 0x210c     /* Address */
#define AD5940_REG_AFE_LPMODEKEY_RESET           0x00000000 /* Reset value */
#define AD5940_BITP_AFE_LPMODEKEY_KEY            0          /* LP Key */
#define AD5940_BITM_AFE_LPMODEKEY_KEY            0x000fffff /* LP Key */

/* AFE low power mode clock selection register.
 * It's protected by LPMODEKEY register.
 * Bit 0: Enable switching system clock to 32kHz by sequencer.
 */

#define AD5940_REG_AFE_LPMODECLKSEL              0x2110     /* Address */
#define AD5940_REG_AFE_LPMODECLKSEL_RESET        0x00000000 /* Reset value */
#define AD5940_BITP_AFE_LPMODECLKSEL_LFSYSCLKEN  0          /* Position */
#define AD5940_BITM_AFE_LPMODECLKSEL_LFSYSCLKEN  0x00000001 /* Mask */

/* AFE low power mode configuration register.
 * It's protected by LPMODEKEY register.
 * Bit 8: Set high to power down of analog LDO
 * Bit 7: Set high to enable 1.1V HP CM buffer
 * Bit 6: Set high to enable HP 1.8V reference buffer
 * Bit 5: Set high to generate Ptat current bias
 * Bit 4: Set high to generate Ztat current bias
 * Bit 3: Set high to enable repeat ADC conversion
 * Bit 2: Set high to enable ADC conversion
 * Bit 1: Set high to power down HP reference
 * Bit 0: Set high to power down HP power oscillator
 */

#define AD5940_REG_AFE_LPMODECON                 0x2114     /* Address */
#define AD5940_REG_AFE_LPMODECON_RESET           0x00000102 /* RST value */
#define AD5940_BITP_AFE_LPMODECON_ALDOEN         8          /* Analog LDO */
#define AD5940_BITP_AFE_LPMODECON_V1P1HPADCEN    7          /* HP CM BUFF */
#define AD5940_BITP_AFE_LPMODECON_V1P8HPADCEN    6          /* HP 1.8 REF */
#define AD5940_BITP_AFE_LPMODECON_PTATEN         5          /* Ptat bias */
#define AD5940_BITP_AFE_LPMODECON_ZTATEN         4          /* Ztat bias */
#define AD5940_BITP_AFE_LPMODECON_REPTADCCNVEN_P 3          /* Repeat ADC */
#define AD5940_BITP_AFE_LPMODECON_ADCCONVEN      2          /* Enable ADC */
#define AD5940_BITP_AFE_LPMODECON_HPREFDIS       1          /* DIS HP REF */
#define AD5940_BITP_AFE_LPMODECON_HFOSCPD        0          /* DIS HP OSC */
#define AD5940_BITM_AFE_LPMODECON_ALDOEN         0x00000100 /* Analog LDO */
#define AD5940_BITM_AFE_LPMODECON_V1P1HPADCEN    0x00000080 /* HP CM BUFF */
#define AD5940_BITM_AFE_LPMODECON_V1P8HPADCEN    0x00000040 /* HP 1.8 REF */
#define AD5940_BITM_AFE_LPMODECON_PTATEN         0x00000020 /* Ptat bias */
#define AD5940_BITM_AFE_LPMODECON_ZTATEN         0x00000010 /* Ztat bias */
#define AD5940_BITM_AFE_LPMODECON_REPTADCCNVEN_P 0x00000008 /* Repeat ADC */
#define AD5940_BITM_AFE_LPMODECON_ADCCONVEN      0x00000004 /* Enable ADC */
#define AD5940_BITM_AFE_LPMODECON_HPREFDIS       0x00000002 /* DIS HP REF */
#define AD5940_BITM_AFE_LPMODECON_HFOSCPD        0x00000001 /* DIS HP OSC */

/* AFE sequencer sleep control lock register
 * Bit 15:0: Password for SLPBYSEQ register
 */

#define AD5940_REG_AFE_SEQSLPLOCK                0x2118     /* Address */
#define AD5940_REG_AFE_SEQSLPLOCK_RESET          0x0000     /* Reset value */
#define AD5940_BITP_AFE_SEQSLPLOCK_SEQ_SLP_PW    0          /* Password */
#define AD5940_BITM_AFE_SEQSLPLOCK_SEQ_SLP_PW    0x000fffff /* Password  */

/* AFE sequencer trigger sleep register
 * Bit 0: Trigger sleep by sequencer
 */

#define AD5940_REG_AFE_SEQTRGSLP                 0x211c     /* Address */
#define AD5940_REG_AFE_SEQTRGSLP_RESET           0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SEQTRGSLP_TRGSLP         0          /* Trigger SLP */
#define AD5940_BITM_AFE_SEQTRGSLP_TRGSLP         0x00000001 /* Trigger SLP */

/* AFE LPDAC data-out register
 * Bit 17:12: Low power DAC 6-bit output data
 * Bit  11:0: Low power DAC 12-bit output data
 */

#define AD5940_REG_AFE_LPDACDAT0                 0x2120     /* Address */
#define AD5940_REG_AFE_LPDACDAT0_RESET           0x00000000 /* Reset value */
#define AD5940_BITP_AFE_LPDACDATX_DACIN6         12         /* LSB=34.375mV */
#define AD5940_BITP_AFE_LPDACDATX_DACIN12        0          /* LSB=537uV */
#define AD5940_BITM_AFE_LPDACDATX_DACIN6         0x0003f000 /* LSB=34.375mV */
#define AD5940_BITM_AFE_LPDACDATX_DACIN12        0x00000fff /* LSB=537uV */

/* AFE LPDAC0 switch control register
 * Bit   5: Switch control
 * Bit 4:0: LPDAC0 switches matrix (LPMODEDIS)
 *          0 - AD5940_REG_AFE_LPDACDAT0 switch controlled by bit 5 of
 *              AD5940_REG_AFE_LPDACCON0
 *          2 - AD5940_REG_AFE_LPDACDAT0 switches override
 * The bit positions and masks also applies to AD5940_REG_AFE_LPDACDAT1
 */

#define AD5940_REG_AFE_LPDACSW0                  0x2124     /* Address */
#define AD5940_REG_AFE_LPDACSW0_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_LPDACSWX_LPMODEDIS       5          /* Switch ctrl */
#define AD5940_BITP_AFE_LPDACSWX_LPDACSW         0          /* SW matrix */
#define AD5940_BITM_AFE_LPDACSWX_LPMODEDIS       0x00000020 /* Switch ctrl */
#define AD5940_BITM_AFE_LPDACSWX_LPDACSW         0x0000001f /* SW matrix */
#define AD5940_ENUM_AFE_LPDACSWX_DACCONBIT5      0x00000000 /* LPMODEDIS */
#define AD5940_ENUM_AFE_LPDACSWX_OVRRIDE         0x00000020 /* LPMODEDIS */

/* AFE LPDAC control bits register
 * Bit 6: LPDAC data source
 *        0 - Direct from AD5940_REG_AFE_LPDACDATx
 *        1 - Waveform generator
 * Bit 5: LPDACx switch settings
 *        0 - AD5940_REG_AFE_LPDACDATx switches set for normal mode
 *        1 - AD5940_REG_AFE_LPDACDATx switches set for diagnostic mode
 * Bit 4: VZERO MUX select
 *        0 - VZERO 6bit
 *        1 - VZERO 12bit
 * Bit 3: VBIAS MUX select
 *        0 - Output 12bit
 *        1 - Output 6bit
 * Bit 2: Reference select bit
 *        0 - ULP2P5V reference
 *        1 - AVDD reference
 * Bit 1: LPDACx power down
 *        0 - AD5940_REG_AFE_LPDACDATx powered on
 *        1 - AD5940_REG_AFE_LPDACDATx powered off
 * Bit 0: Enable writes to AD5940_REG_AFE_LPDACDATx
 *        0 - Disable AD5940_REG_AFE_LPDACDATx writes
 *        1 - Enable AD5940_REG_AFE_LPDACDATx writes
 */

#define AD5940_REG_AFE_LPDACCON0                 0x2128     /* Address */
#define AD5940_REG_AFE_LPDACCON0_RESET           0x00000002 /* Reset value */
#define AD5940_BITP_AFE_LPDACCONX_WAVETYPE       6          /* Data source */
#define AD5940_BITP_AFE_LPDACCONX_DACMDE         5          /* Switch */
#define AD5940_BITP_AFE_LPDACCONX_VZEROMUX       4          /* VZERO MUX */
#define AD5940_BITP_AFE_LPDACCONX_VBIASMUX       3          /* VBIAS MUX */
#define AD5940_BITP_AFE_LPDACCONX_REFSEL         2          /* Reference */
#define AD5940_BITP_AFE_LPDACCONX_PWDEN          1          /* LPDACx PWDN */
#define AD5940_BITP_AFE_LPDACCONX_RSTEN          0          /* Enable write */
#define AD5940_BITM_AFE_LPDACCONX_WAVETYPE       0x00000040 /* Data source */
#define AD5940_BITM_AFE_LPDACCONX_DACMDE         0x00000020 /* Switch */
#define AD5940_BITM_AFE_LPDACCONX_VZEROMUX       0x00000010 /* VZERO MUX */
#define AD5940_BITM_AFE_LPDACCONX_VBIASMUX       0x00000008 /* VBIAS MUX */
#define AD5940_BITM_AFE_LPDACCONX_REFSEL         0x00000004 /* Reference */
#define AD5940_BITM_AFE_LPDACCONX_PWDEN          0x00000002 /* LPDACx PWDN */
#define AD5940_BITM_AFE_LPDACCONX_RSTEN          0x00000001 /* Enable write */
#define AD5940_ENUM_AFE_LPDACCONX_MMR            0x00000000 /* WAVETYPE */
#define AD5940_ENUM_AFE_LPDACCONX_WAVEGEN        0x00000040 /* WAVETYPE */
#define AD5940_ENUM_AFE_LPDACCONX_NORM           0x00000000 /* DACMDE */
#define AD5940_ENUM_AFE_LPDACCONX_DIAG           0x00000020 /* DACMDE */
#define AD5940_ENUM_AFE_LPDACCONX_BITS6          0x00000000 /* VZEROMUX */
#define AD5940_ENUM_AFE_LPDACCONX_BITS12         0x00000010 /* VZEROMUX */
#define AD5940_ENUM_AFE_LPDACCONX_12BIT          0x00000000 /* VBIASMUX */
#define AD5940_ENUM_AFE_LPDACCONX_EN             0x00000008 /* VBIASMUX */
#define AD5940_ENUM_AFE_LPDACCONX_ULPREF         0x00000000 /* REFSEL */
#define AD5940_ENUM_AFE_LPDACCONX_AVDD           0x00000004 /* REFSEL */
#define AD5940_ENUM_AFE_LPDACCONX_PWREN          0x00000000 /* PWDEN */
#define AD5940_ENUM_AFE_LPDACCONX_PWRDIS         0x00000002 /* PWDEN */
#define AD5940_ENUM_AFE_LPDACCONX_WRITEDIS       0x00000000 /* RSTEN */
#define AD5940_ENUM_AFE_LPDACCONX_WRITEEN        0x00000001 /* RSTEN */

/* AFE low power DAC1 data register.
 * See AD5940_REG_AFE_LPDACDAT0 fot detail.
 */

#define AD5940_REG_AFE_LPDACDAT1                 0x212c     /* Address */

/* AFE control register for switches to LPDAC1.
 * See AD5940_REG_AFE_LPDACSW0 for detail.
 */

#define AD5940_REG_AFE_LPDACSW1                  0x2130     /* Address */

/* AFE LPDAC control bits register. See AD5940_REG_AFE_LPDACCON0 for detial */

#define AD5940_REG_AFE_LPDACCON1                 0x2134     /* Address */

/* AFE switch matrix full configuration (D) register */

#define AD5940_REG_AFE_DSWFULLCON         0x2150     /* Address */
#define AD5940_REG_AFE_DSWFULLCON_RESET   0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DSWFULLCON_D8     7          /* Control D8 switch */
#define AD5940_BITP_AFE_DSWFULLCON_D7     6          /* Control D7 switch */
#define AD5940_BITP_AFE_DSWFULLCON_D6     5          /* Control D6 switch */
#define AD5940_BITP_AFE_DSWFULLCON_D5     4          /* Control D5 switch */
#define AD5940_BITP_AFE_DSWFULLCON_D4     3          /* Control D4 switch */
#define AD5940_BITP_AFE_DSWFULLCON_D3     2          /* Control D3 switch */
#define AD5940_BITP_AFE_DSWFULLCON_D2     1          /* Control D2 switch */
#define AD5940_BITP_AFE_DSWFULLCON_DR0    0          /* Control DR0 switch */
#define AD5940_BITM_AFE_DSWFULLCON_D8     0x00000080 /* Control D8 switch */
#define AD5940_BITM_AFE_DSWFULLCON_D7     0x00000040 /* Control D7 switch */
#define AD5940_BITM_AFE_DSWFULLCON_D6     0x00000020 /* Control D6 switch */
#define AD5940_BITM_AFE_DSWFULLCON_D5     0x00000010 /* Control D5 switch */
#define AD5940_BITM_AFE_DSWFULLCON_D4     0x00000008 /* Control D4 switch */
#define AD5940_BITM_AFE_DSWFULLCON_D3     0x00000004 /* Control D3 switch */
#define AD5940_BITM_AFE_DSWFULLCON_D2     0x00000002 /* Control D2 switch */
#define AD5940_BITM_AFE_DSWFULLCON_DR0    0x00000001 /* Control DR0 switch */

/* AFE switch matrix full configuration (N) register
 * Bit 11~0: Control of each switch.
 *           0 - Open
 *           1 - Close
 */

#define AD5940_REG_AFE_NSWFULLCON         0x2154     /* Address */
#define AD5940_REG_AFE_NSWFULLCON_RESET   0x00000000 /* Reset value */
#define AD5940_BITP_AFE_NSWFULLCON_NL2    11         /* Control NL2 switch. */
#define AD5940_BITP_AFE_NSWFULLCON_NL     10         /* Control NL switch. */
#define AD5940_BITP_AFE_NSWFULLCON_NR1    9          /* Control NR1 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N9     8          /* Control N9 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N8     7          /* Control N8 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N7     6          /* Control N7 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N6     5          /* Control N6 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N5     4          /* Control N5 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N4     3          /* Control N4 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N3     2          /* Control N3 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N2     1          /* Control N2 switch */
#define AD5940_BITP_AFE_NSWFULLCON_N1     0          /* Control N1 switch */
#define AD5940_BITM_AFE_NSWFULLCON_NL2    0x00000800 /* Control NL2 switch */
#define AD5940_BITM_AFE_NSWFULLCON_NL     0x00000400 /* Control NL switch */
#define AD5940_BITM_AFE_NSWFULLCON_NR1    0x00000200 /* Control Nr1 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N9     0x00000100 /* Control N9 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N8     0x00000080 /* Control N8 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N7     0x00000040 /* Control N7 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N6     0x00000020 /* Control N6 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N5     0x00000010 /* Control N5 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N4     0x00000008 /* Control N4 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N3     0x00000004 /* Control N3 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N2     0x00000002 /* Control N2 switch */
#define AD5940_BITM_AFE_NSWFULLCON_N1     0x00000001 /* Control N1 switch */

/* AFE switch matrix full configuration (P) register
 * Bit 14~0: Control of each switch.
 *           0 - Open
 *           1 - Close
 */

#define AD5940_REG_AFE_PSWFULLCON         0x2158     /* Address */
#define AD5940_REG_AFE_PSWFULLCON_RESET   0x00000000 /* Reset value */
#define AD5940_BITP_AFE_PSWFULLCON_PL2    14         /* PL2 switch control */
#define AD5940_BITP_AFE_PSWFULLCON_PL     13         /* PL switch Control */
#define AD5940_BITP_AFE_PSWFULLCON_P12    11         /* Control P12 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P11    10         /* Control P11 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P10    9          /* P10 switch control */
#define AD5940_BITP_AFE_PSWFULLCON_P9     8          /* Control P9 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P8     7          /* Control P8 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P7     6          /* Control P7 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P6     5          /* Control P6 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P5     4          /* Control P5 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P4     3          /* Control P4 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P3     2          /* Control P3 switch */
#define AD5940_BITP_AFE_PSWFULLCON_P2     1          /* Control P2 switch */
#define AD5940_BITP_AFE_PSWFULLCON_PR0    0          /* PR0 switch control */
#define AD5940_BITM_AFE_PSWFULLCON_PL2    0x00004000 /* PL2 switch control */
#define AD5940_BITM_AFE_PSWFULLCON_PL     0x00002000 /* PL switch control */
#define AD5940_BITM_AFE_PSWFULLCON_P12    0x00000800 /* Control P12 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P11    0x00000400 /* Control P11 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P10    0x00000200 /* P10 switch control */
#define AD5940_BITM_AFE_PSWFULLCON_P9     0x00000100 /* Control P9 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P8     0x00000080 /* Control P8 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P7     0x00000040 /* Control P7 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P6     0x00000020 /* Control P6 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P5     0x00000010 /* Control P5 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P4     0x00000008 /* Control P4 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P3     0x00000004 /* Control P3 switch */
#define AD5940_BITM_AFE_PSWFULLCON_P2     0x00000002 /* Control P2 switch */
#define AD5940_BITM_AFE_PSWFULLCON_PR0    0x00000001 /* PR0 switch control */

/* AFE switch matrix full configuration (T)
 * Bit 14~0: Control of each switch.
 *           0 - Open
 *           1 - Close
 */

#define AD5940_REG_AFE_TSWFULLCON         0x215c     /* Address */
#define AD5940_REG_AFE_TSWFULLCON_RESET   0x00000000 /* Reset value */
#define AD5940_BITP_AFE_TSWFULLCON_TR1    11         /* Control TR1 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T11    10         /* Control T11 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T10    9          /* Control T10 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T9     8          /* Control T9 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T7     6          /* Control T7 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T5     4          /* Control T5 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T4     3          /* Control T4 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T3     2          /* Control T3 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T2     1          /* Control T2 switch */
#define AD5940_BITP_AFE_TSWFULLCON_T1     0          /* Control T1 switch */
#define AD5940_BITM_AFE_TSWFULLCON_TR1    0x00000800 /* Control TR1 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T11    0x00000400 /* Control T11 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T10    0x00000200 /* Control T10 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T9     0x00000100 /* Control T9 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T7     0x00000040 /* Control T7 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T5     0x00000010 /* Control T5 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T4     0x00000008 /* Control T4 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T3     0x00000004 /* Control T3 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T2     0x00000002 /* Control T2 switch */
#define AD5940_BITM_AFE_TSWFULLCON_T1     0x00000001 /* Control T1 switch */

/* AFE temperature sensor configuration register
 * Bit 3:2: Chop mode frequency setting
 * Bit   1: Temperatur sensor chop mode
 *          0 - Disable chop mode
 *          1 - Enable chop mode
 * Bit   0: Unused
 */

#define AD5940_REG_AFE_TEMPSENS                  0x2174     /* Address */
#define AD5940_REG_AFE_TEMPSENS_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_TEMPSENS_CHOPFRESEL      2          /* Frequency */
#define AD5940_BITP_AFE_TEMPSENS_CHOPCON         1          /* Chop mode */
#define AD5940_BITP_AFE_TEMPSENS_ENABLE          0          /* Unused */
#define AD5940_BITM_AFE_TEMPSENS_CHOPFRESEL      0x0000000c /* Frequency */
#define AD5940_BITM_AFE_TEMPSENS_CHOPCON         0x00000002 /* Chop mode */
#define AD5940_BITM_AFE_TEMPSENS_ENABLE          0x00000001 /* Unused */
#define AD5940_ENUM_AFE_TEMPSENS_DIS             0x00000000 /* CHOPCON */
#define AD5940_ENUM_AFE_TEMPSENS_EN              0x00000002 /* CHOPCON */

/* AFE HP and LP buffer control register
 * Bit 8: Buffered reference output
 *        0 - Disable 1.8V buffered reference output
 *        1 - Enable 1.8V buffered reference output
 * Bit 6: Controls decoupling cap discharge switch
 *        0 - Open switch
 *        1 - Close switch
 * Bit 5: ADC 1.1V LP buffer
 *        0 - Disable ADC 1.1V LP reference buffer
 *        1 - Enable ADC 1.1V LP reference buffer
 * Bit 4: Enable 1.1V HP CM buffer
 *        0 - Disable 1.1V HP common mode buffer
 *        1 - Enable 1.1V HP common mode buffer
 * Bit 3: Controls decoupling cap discharge switch
 *        0 - Open switch
 *        1 - Close switch
 * Bit 2: ADC 1.8V LP reference buffer
 *        0 - Disable LP 1.8V reference buffer
 *        1 - Enable LP 1.8V reference buffer
 * Bit 1: HP ADC input current limit
 *        0 - Disable buffer current limit
 *        1 - Enable buffer current limit
 * Bit 0: HP 1.8V reference buffer
 *        0 - Disable 1.8V HP ADC reference buffer
 *        1 - Enable 1.8V HP ADC reference buffer
 */

#define AD5940_REG_AFE_BUFCON                    0x2180     /* Address */
#define AD5940_REG_AFE_BUFCON_RESET              0x00000037 /* RST value */
#define AD5940_BITP_AFE_BUFCON_V1P8THERMSTEN     8          /* Position */
#define AD5940_BITP_AFE_BUFCON_V1P1LPADCCHGDIS   6          /* Position */
#define AD5940_BITP_AFE_BUFCON_V1P1LPADCEN       5          /* Position */
#define AD5940_BITP_AFE_BUFCON_V1P1HPADCEN       4          /* Position */
#define AD5940_BITP_AFE_BUFCON_V1P8HPADCCHGDIS   3          /* Position */
#define AD5940_BITP_AFE_BUFCON_V1P8LPADCEN       2          /* Position */
#define AD5940_BITP_AFE_BUFCON_V1P8HPADCILIMITEN 1          /* Position */
#define AD5940_BITP_AFE_BUFCON_V1P8HPADCEN       0          /* Position */
#define AD5940_BITM_AFE_BUFCON_V1P8THERMSTEN     0x00000100 /* Mask */
#define AD5940_BITM_AFE_BUFCON_V1P1LPADCCHGDIS   0x00000040 /* Mask */
#define AD5940_BITM_AFE_BUFCON_V1P1LPADCEN       0x00000020 /* Mask */
#define AD5940_BITM_AFE_BUFCON_V1P1HPADCEN       0x00000010 /* Mask */
#define AD5940_BITM_AFE_BUFCON_V1P8HPADCCHGDIS   0x00000008 /* Mask */
#define AD5940_BITM_AFE_BUFCON_V1P8LPADCEN       0x00000004 /* Mask */
#define AD5940_BITM_AFE_BUFCON_V1P8HPADCILIMITEN 0x00000002 /* Mask */
#define AD5940_BITM_AFE_BUFCON_V1P8HPADCEN       0x00000001 /* Mask */
#define AD5940_ENUM_AFE_BUFCON_DIS               0x00000000 /* Bit 8 */
#define AD5940_ENUM_AFE_BUFCON_EN                0x00000100 /* Bit 8 */
#define AD5940_ENUM_AFE_BUFCON_ENCHRG            0x00000000 /* Bit 6 */
#define AD5940_ENUM_AFE_BUFCON_DISCHRG           0x00000040 /* Bit 6 */
#define AD5940_ENUM_AFE_BUFCON_DISABLE           0x00000000 /* Bit 5 */
#define AD5940_ENUM_AFE_BUFCON_ENABLE            0x00000020 /* Bit 5 */
#define AD5940_ENUM_AFE_BUFCON_OFF               0x00000000 /* Bit 4 */
#define AD5940_ENUM_AFE_BUFCON_ON                0x00000010 /* Bit 4 */
#define AD5940_ENUM_AFE_BUFCON_OPEN              0x00000000 /* Bit 3 */
#define AD5940_ENUM_AFE_BUFCON_CLOSED            0x00000008 /* Bit 3 */
#define AD5940_ENUM_AFE_BUFCON_LPADCREF_DIS      0x00000000 /* Bit 2 */
#define AD5940_ENUM_AFE_BUFCON_LPADCREF_EN       0x00000004 /* Bit 2 */
#define AD5940_ENUM_AFE_BUFCON_LIMIT_DIS         0x00000000 /* Bit 1 */
#define AD5940_ENUM_AFE_BUFCON_LIMIT_EN          0x00000002 /* Bit 1 */
#define AD5940_ENUM_AFE_BUFCON_HPBUF_DIS         0x00000000 /* Bit 0 */
#define AD5940_ENUM_AFE_BUFCON_HPBUF_EN          0x00000001 /* Bit 0 */

/* AFE ADC configuration register
 * Bit 18:16: PGA gain setup
 * Bit    15: Internal offset/gain cancellation
 * Bit 14:13: Obsolete
 * Bit  12:8: Select negative input
 * Bit   5:0: Select positive input
 */

#define AD5940_REG_AFE_ADCCON             0x21a8     /* ADC Configuration */
#define AD5940_REG_AFE_ADCCON_RESET       0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCCON_GNPGA      16         /* PGA Gain Setup */
#define AD5940_BITP_AFE_ADCCON_GNOFSELPGA 15         /* Offset/gain Disable */
#define AD5940_BITP_AFE_ADCCON_GNOFFSEL   13         /* Obsolete */
#define AD5940_BITP_AFE_ADCCON_MUXSELN    8          /* Negative input */
#define AD5940_BITP_AFE_ADCCON_MUXSELP    0          /* Positive input */
#define AD5940_BITM_AFE_ADCCON_GNPGA      0x00070000 /* PGA Gain Setup */
#define AD5940_BITM_AFE_ADCCON_GNOFSELPGA 0x00008000 /* Offset/gain Disable */
#define AD5940_BITM_AFE_ADCCON_GNOFFSEL   0x00006000 /* Obsolete */
#define AD5940_BITM_AFE_ADCCON_MUXSELN    0x00001f00 /* Negative input */
#define AD5940_BITM_AFE_ADCCON_MUXSELP    0x0000003f /* Positive input */
#define AD5940_ENUM_AFE_ADCCON_RESERVED   0x00000011 /* MUXSELP: Reserved */

/* AFE switch Matrix Status (D) register */

#define AD5940_REG_AFE_DSWSTA             0x21B0     /* Address */
#define AD5940_REG_AFE_DSWSTA_RESET       0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DSWSTA_D8STA      7          /* Status of D8 switch */
#define AD5940_BITP_AFE_DSWSTA_D7STA      6          /* Status of D7 switch */
#define AD5940_BITP_AFE_DSWSTA_D6STA      5          /* Status of D6 switch */
#define AD5940_BITP_AFE_DSWSTA_D5STA      4          /* Status of D5 switch */
#define AD5940_BITP_AFE_DSWSTA_D4STA      3          /* Status of D4 switch */
#define AD5940_BITP_AFE_DSWSTA_D3STA      2          /* Status of D3 switch */
#define AD5940_BITP_AFE_DSWSTA_D2STA      1          /* Status of D2 switch */
#define AD5940_BITP_AFE_DSWSTA_D1STA      0          /* DR0 switch status */
#define AD5940_BITM_AFE_DSWSTA_D8STA      0x00000080 /* Status of D8 switch */
#define AD5940_BITM_AFE_DSWSTA_D7STA      0x00000040 /* Status of D7 switch*/
#define AD5940_BITM_AFE_DSWSTA_D6STA      0x00000020 /* Status of D6 switch */
#define AD5940_BITM_AFE_DSWSTA_D5STA      0x00000010 /* Status of D5 switch */
#define AD5940_BITM_AFE_DSWSTA_D4STA      0x00000008 /* Status of D4 switch */
#define AD5940_BITM_AFE_DSWSTA_D3STA      0x00000004 /* Status of D3 switch */
#define AD5940_BITM_AFE_DSWSTA_D2STA      0x00000002 /* Status of D2 switch */
#define AD5940_BITM_AFE_DSWSTA_D1STA      0x00000001 /* DR0 switch status */

/* AFE switch Matrix Status (P) register */

#define AD5940_REG_AFE_PSWSTA             0x21b4     /* Address */
#define AD5940_REG_AFE_PSWSTA_RESET       0x00006000 /* Reset value */
#define AD5940_BITP_AFE_PSWSTA_PL2STA     14         /* PL2 switch status */
#define AD5940_BITP_AFE_PSWSTA_PLSTA      13         /* PL switch status */
#define AD5940_BITP_AFE_PSWSTA_P13STA     12         /* P13 switch status */
#define AD5940_BITP_AFE_PSWSTA_P12STA     11         /* P12 switch status */
#define AD5940_BITP_AFE_PSWSTA_P11STA     10         /* P11 switch status */
#define AD5940_BITP_AFE_PSWSTA_P10STA     9          /* P10 switch status */
#define AD5940_BITP_AFE_PSWSTA_P9STA      8          /* P9 switch status */
#define AD5940_BITP_AFE_PSWSTA_P8STA      7          /* Status of P8 switch */
#define AD5940_BITP_AFE_PSWSTA_P7STA      6          /* Status of P7 switch */
#define AD5940_BITP_AFE_PSWSTA_P6STA      5          /* Status of P6 switch */
#define AD5940_BITP_AFE_PSWSTA_P5STA      4          /* Status of P5 switch */
#define AD5940_BITP_AFE_PSWSTA_P4STA      3          /* Status of P4 switch */
#define AD5940_BITP_AFE_PSWSTA_P3STA      2          /* Status of P3 switch */
#define AD5940_BITP_AFE_PSWSTA_P2STA      1          /* Status of P2 switch */
#define AD5940_BITP_AFE_PSWSTA_PR0STA     0          /* PR0 switch status */
#define AD5940_BITM_AFE_PSWSTA_PL2STA     0x00004000 /* PL2 switch status */
#define AD5940_BITM_AFE_PSWSTA_PLSTA      0x00002000 /* PL switch status */
#define AD5940_BITM_AFE_PSWSTA_P13STA     0x00001000 /* P13 switch status */
#define AD5940_BITM_AFE_PSWSTA_P12STA     0x00000800 /* P12 switch status */
#define AD5940_BITM_AFE_PSWSTA_P11STA     0x00000400 /* P11 switch status */
#define AD5940_BITM_AFE_PSWSTA_P10STA     0x00000200 /* P10 switch status */
#define AD5940_BITM_AFE_PSWSTA_P9STA      0x00000100 /* Status of P9 switch */
#define AD5940_BITM_AFE_PSWSTA_P8STA      0x00000080 /* Status of P8 switch */
#define AD5940_BITM_AFE_PSWSTA_P7STA      0x00000040 /* Status of P7 switch */
#define AD5940_BITM_AFE_PSWSTA_P6STA      0x00000020 /* Status of P6 switch */
#define AD5940_BITM_AFE_PSWSTA_P5STA      0x00000010 /* Status of P5 switch */
#define AD5940_BITM_AFE_PSWSTA_P4STA      0x00000008 /* Status of P4 switch */
#define AD5940_BITM_AFE_PSWSTA_P3STA      0x00000004 /* Status of P3 switch */
#define AD5940_BITM_AFE_PSWSTA_P2STA      0x00000002 /* Status of P2 switch */
#define AD5940_BITM_AFE_PSWSTA_PR0STA     0x00000001 /* PR0 switch status */

/* AFE switch matrix status (N) register */

#define AD5940_REG_AFE_NSWSTA             0x21b8     /* Address */
#define AD5940_REG_AFE_NSWSTA_RESET       0x00000c00 /* Reset value */
#define AD5940_BITP_AFE_NSWSTA_NL2STA     11         /* NL2 switch status */
#define AD5940_BITP_AFE_NSWSTA_NLSTA      10         /* NL switch status */
#define AD5940_BITP_AFE_NSWSTA_NR1STA     9          /* NR1 switch status */
#define AD5940_BITP_AFE_NSWSTA_N9STA      8          /* Status of N9 switch */
#define AD5940_BITP_AFE_NSWSTA_N8STA      7          /* Status of N8 switch */
#define AD5940_BITP_AFE_NSWSTA_N7STA      6          /* Status of N7 switch */
#define AD5940_BITP_AFE_NSWSTA_N6STA      5          /* Status of N6 switch */
#define AD5940_BITP_AFE_NSWSTA_N5STA      4          /* Status of N5 switch */
#define AD5940_BITP_AFE_NSWSTA_N4STA      3          /* Status of N4 switch */
#define AD5940_BITP_AFE_NSWSTA_N3STA      2          /* Status of N3 switch */
#define AD5940_BITP_AFE_NSWSTA_N2STA      1          /* Status of N2 switch */
#define AD5940_BITP_AFE_NSWSTA_N1STA      0          /* Status of N1 switch */
#define AD5940_BITM_AFE_NSWSTA_NL2STA     0x00000800 /* NL2 switch status */
#define AD5940_BITM_AFE_NSWSTA_NLSTA      0x00000400 /* NL switch status */
#define AD5940_BITM_AFE_NSWSTA_NR1STA     0x00000200 /* NR1 switch status */
#define AD5940_BITM_AFE_NSWSTA_N9STA      0x00000100 /* Status of N9 switch */
#define AD5940_BITM_AFE_NSWSTA_N8STA      0x00000080 /* Status of N8 switch */
#define AD5940_BITM_AFE_NSWSTA_N7STA      0x00000040 /* Status of N7 switch */
#define AD5940_BITM_AFE_NSWSTA_N6STA      0x00000020 /* Status of N6 switch */
#define AD5940_BITM_AFE_NSWSTA_N5STA      0x00000010 /* Status of N5 switch */
#define AD5940_BITM_AFE_NSWSTA_N4STA      0x00000008 /* Status of N4 switch */
#define AD5940_BITM_AFE_NSWSTA_N3STA      0x00000004 /* Status of N3 switch */
#define AD5940_BITM_AFE_NSWSTA_N2STA      0x00000002 /* Status of N2 switch */
#define AD5940_BITM_AFE_NSWSTA_N1STA      0x00000001 /* Status of N1 switch */

/* AFE switch matrix status (T) register */

#define AD5940_REG_AFE_TSWSTA             0x21bc     /* Address */
#define AD5940_REG_AFE_TSWSTA_RESET       0x00000000 /* Reset value */
#define AD5940_BITP_AFE_TSWSTA_TR1STA     11         /* TR1 switch status */
#define AD5940_BITP_AFE_TSWSTA_T11STA     10         /* T11 switch status */
#define AD5940_BITP_AFE_TSWSTA_T10STA     9          /* T10 switch status */
#define AD5940_BITP_AFE_TSWSTA_T9STA      8          /* Status of T9 switch */
#define AD5940_BITP_AFE_TSWSTA_T8STA      7          /* Status of T8 switch */
#define AD5940_BITP_AFE_TSWSTA_T7STA      6          /* Status of T7 switch */
#define AD5940_BITP_AFE_TSWSTA_T6STA      5          /* Status of T6 switch */
#define AD5940_BITP_AFE_TSWSTA_T5STA      4          /* Status of T5 switch */
#define AD5940_BITP_AFE_TSWSTA_T4STA      3          /* Status of T4 switch */
#define AD5940_BITP_AFE_TSWSTA_T3STA      2          /* Status of T3 switch */
#define AD5940_BITP_AFE_TSWSTA_T2STA      1          /* Status of T2 switch */
#define AD5940_BITP_AFE_TSWSTA_T1STA      0          /* Status of T1 switch */
#define AD5940_BITM_AFE_TSWSTA_TR1STA     0x00000800 /* TR1 switch status */
#define AD5940_BITM_AFE_TSWSTA_T11STA     0x00000400 /* T11 switch status */
#define AD5940_BITM_AFE_TSWSTA_T10STA     0x00000200 /* T10 switch status */
#define AD5940_BITM_AFE_TSWSTA_T9STA      0x00000100 /* Status of T9 switch */
#define AD5940_BITM_AFE_TSWSTA_T8STA      0x00000080 /* Status of T8 switch */
#define AD5940_BITM_AFE_TSWSTA_T7STA      0x00000040 /* Status of T7 switch */
#define AD5940_BITM_AFE_TSWSTA_T6STA      0x00000020 /* Status of T6 switch */
#define AD5940_BITM_AFE_TSWSTA_T5STA      0x00000010 /* Status of T5 switch */
#define AD5940_BITM_AFE_TSWSTA_T4STA      0x00000008 /* Status of T4 switch */
#define AD5940_BITM_AFE_TSWSTA_T3STA      0x00000004 /* Status of T3 switch */
#define AD5940_BITM_AFE_TSWSTA_T2STA      0x00000002 /* Status of T2 switch */
#define AD5940_BITM_AFE_TSWSTA_T1STA      0x00000001 /* Status of T1 switch */

/* AFE variance output register
 * Bit 30:0: Statistical variance value
 */

#define AD5940_REG_AFE_STATSVAR           0x21c0     /* AFE Variance Output */
#define AD5940_REG_AFE_STATSVAR_RESET     0x00000000 /* Reset value */
#define AD5940_BITP_AFE_STATSVAR_VARIANCE 0          /* Variance value */
#define AD5940_BITM_AFE_STATSVAR_VARIANCE 0x7fffffff /* Variance value */

/* AFE statistics control register
 * Bit 11:7: Standard deviation configuration
 * Bit  6:4: Sample size
 * Bit  3:1: Reserved
 * Bit    0: Statistics enable
 *           0 - Disable statistics
 *           1 - Enable statistics
 */

#define AD5940_REG_AFE_STATSCON           0x21c4     /* Address */
#define AD5940_REG_AFE_STATSCON_RESET     0x00000000 /* Reset value */
#define AD5940_BITP_AFE_STATSCON_STDDEV   7          /* Deviation */
#define AD5940_BITP_AFE_STATSCON_SMPLENUM 4          /* Sample size */
#define AD5940_BITP_AFE_STATSCON_RESRVED  1          /* Reserved */
#define AD5940_BITP_AFE_STATSCON_STATSEN  0          /* Enable */
#define AD5940_BITM_AFE_STATSCON_STDDEV   0x00000F80 /* Deviation */
#define AD5940_BITM_AFE_STATSCON_SPLENUM  0x00000070 /* Sample size */
#define AD5940_BITM_AFE_STATSCON_RESRVED  0x0000000E /* Reserved */
#define AD5940_BITM_AFE_STATSCON_STATSEN  0x00000001 /* Enable */
#define AD5940_ENUM_AFE_STATSCON_DIS      0x00000000 /* STATSEN */
#define AD5940_ENUM_AFE_STATSCON_EN       0x00000001 /* STATSEN */

/* AFE statistics mean output register */

#define AD5940_REG_AFE_STATSMEAN          0x21c8     /* Address */
#define AD5940_REG_AFE_STATSMEAN_RESET    0x00000000 /* Reset value */
#define AD5940_BITP_AFE_STATSMEAN_MEAN    0          /* Mean Output */
#define AD5940_BITM_AFE_STATSMEAN_MEAN    0x0000ffff /* Mean Output */

/* AFE Sequence 0 Info register
 * Bit 26:16: SEQ0 instruction number
 * Bit  10:0: SEQ0 start address
 */

#define AD5940_REG_AFE_SEQ0INFO           0x21cc     /* AFE Sequence 0 Info */
#define AD5940_REG_AFE_SEQ0INFO_RESET     0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SEQ0INFO_LEN      16         /* SEQ0 instruction No */
#define AD5940_BITP_AFE_SEQ0INFO_ADDR     0          /* SEQ0 start address */
#define AD5940_BITM_AFE_SEQ0INFO_LEN      0x07ff0000 /* SEQ0 instruction No */
#define AD5940_BITM_AFE_SEQ0INFO_ADDR     0x000007ff /* SEQ0 start address */

/* AFE Sequence 2 Info register
 * Bit 26:16: SEQ2 instruction number
 * Bit  10:0: SEQ2 start address
 */

#define AD5940_REG_AFE_SEQ2INFO           0x21d0     /* AFE Sequence 2 Info */
#define AD5940_REG_AFE_SEQ2INFO_RESET     0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SEQ2INFO_LEN      16         /* SEQ2 instruction No */
#define AD5940_BITP_AFE_SEQ2INFO_ADDR     0          /* SEQ2 start address */
#define AD5940_BITM_AFE_SEQ2INFO_LEN      0x07ff0000 /* SEQ2 instruction No */
#define AD5940_BITM_AFE_SEQ2INFO_ADDR     0x000007ff /* SEQ2 start address */

/* AFE command FIFO write address register */

#define AD5940_REG_AFE_CMDFIFOWADDR       0x21d4     /* Address */
#define AD5940_REG_AFE_CMDFIFOWADDR_RESET 0x00000000 /* Reset value */
#define AD5940_BITP_AFE_CMDFIFOWADDR_ADDR 0          /* Write addr. */
#define AD5940_BITM_AFE_CMDFIFOWADDR_ADDR 0x000007ff /* Write addr. */

/* AFE command data control register
 * Bit 11:9: Data FIFO mode select
 *           2 - FIFO mode
 *           3 - Stream mode
 * Bit  8:6: Data FIFO size select
 *           0 - 32B_1 local memory
 *           1 - 2K_2 SRAM
 *           2 - 2K_2~1 SRAM
 *           3 - 2K_2~0 SRAM
 * Bit  5:3: Command FIFO mode select
 *           1 - Memory mode
 *           2 - FIFO mode
 *           3 - Stream mode
 * BIt  2:0: Command FIFO size select
 *           0 - 32B_0 local memory
 *           1 - 2K_0 SRAM
 *           2 - 2K_0~1 SRAM
 *           3 - 2K_0~2 SRAM
 */

#define AD5940_REG_AFE_CMDDATACON                0x21d8     /* Address */
#define AD5940_REG_AFE_CMDDATACON_RESET          0x00000410 /* Reset value */
#define AD5940_BITP_AFE_CMDDATACON_DATAMEMMDE    9          /* Position */
#define AD5940_BITP_AFE_CMDDATACON_DATA_MEM_SEL  6          /* Position */
#define AD5940_BITP_AFE_CMDDATACON_CMDMEMMDE     3          /* Position */
#define AD5940_BITP_AFE_CMDDATACON_CMD_MEM_SEL   0          /* Position */
#define AD5940_BITM_AFE_CMDDATACON_DATAMEMMDE    0x00000e00 /* Mask */
#define AD5940_BITM_AFE_CMDDATACON_DATA_MEM_SEL  0x000001c0 /* Mask */
#define AD5940_BITM_AFE_CMDDATACON_CMDMEMMDE     0x00000038 /* Mask */
#define AD5940_BITM_AFE_CMDDATACON_CMD_MEM_SEL   0x00000007 /* Mask */
#define AD5940_ENUM_AFE_CMDDATACON_DFIFO         0x00000400 /* DATAMEMMDE */
#define AD5940_ENUM_AFE_CMDDATACON_DSTM          0x00000600 /* DATAMEMMDE */
#define AD5940_ENUM_AFE_CMDDATACON_DMEM32B       0x00000000 /* DATA_MEM_SEL */
#define AD5940_ENUM_AFE_CMDDATACON_DMEM2K        0x00000040 /* DATA_MEM_SEL */
#define AD5940_ENUM_AFE_CMDDATACON_DMEM4K        0x00000080 /* DATA_MEM_SEL */
#define AD5940_ENUM_AFE_CMDDATACON_DMEM6K        0x000000c0 /* DATA_MEM_SEL */
#define AD5940_ENUM_AFE_CMDDATACON_CMEM          0x00000008 /* CMDMEMMDE */
#define AD5940_ENUM_AFE_CMDDATACON_CFIFO         0x00000010 /* CMDMEMMDE*/
#define AD5940_ENUM_AFE_CMDDATACON_CSTM          0x00000018 /* CMDMEMMDE */
#define AD5940_ENUM_AFE_CMDDATACON_CMEM32B       0x00000000 /* CMD_MEM_SEL */
#define AD5940_ENUM_AFE_CMDDATACON_CMEM2K        0x00000001 /* CMD_MEM_SEL */
#define AD5940_ENUM_AFE_CMDDATACON_CMEM4K        0x00000002 /* CMD_MEM_SEL */
#define AD5940_ENUM_AFE_CMDDATACON_CMEM6K        0x00000003 /* CMD_MEM_SEL */

/* AFE data FIFO threshold register */

#define AD5940_REG_AFE_DATAFIFOTHRES             0x21e0     /* Address */
#define AD5940_REG_AFE_DATAFIFOTHRES_RESET       0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DATAFIFOTHRES_HIGHTHRES  16         /* High threshd */
#define AD5940_BITM_AFE_DATAFIFOTHRES_HIGHTHRES  0x07ff0000 /* High threshd */

/* AFE sequence 3 info register */

#define AD5940_REG_AFE_SEQ3INFO                  0x21e4     /* Address */
#define AD5940_REG_AFE_SEQ3INFO_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SEQ3INFO_LEN             16         /* Instructions */
#define AD5940_BITP_AFE_SEQ3INFO_ADDR            0          /* Start addr */
#define AD5940_BITM_AFE_SEQ3INFO_LEN             0x07ff0000 /* Instructions */
#define AD5940_BITM_AFE_SEQ3INFO_ADDR            0x000007ff /* Start addr */

/* AFE sequence 1 info register */

#define AD5940_REG_AFE_SEQ1INFO                  0x21e8     /* Address */
#define AD5940_REG_AFE_SEQ1INFO_RESET            0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SEQ1INFO_LEN             16         /* Instructions */
#define AD5940_BITP_AFE_SEQ1INFO_ADDR            0          /* Start addr */
#define AD5940_BITM_AFE_SEQ1INFO_LEN             0x07ff0000 /* Instructions */
#define AD5940_BITM_AFE_SEQ1INFO_ADDR            0x000007ff /* Start addr */

/* AFE repeat ADC conversions register
 * Bit 11:4: Repeat value
 * Bit    0: Enable repeat ADC conversions
 *           0 - Disable repeat ADC conversions
 *           1 - Enable repeat ADC conversions
 */

#define AD5940_REG_AFE_REPEATADCCNV              0x21f0     /* Address */
#define AD5940_REG_AFE_REPEATADCCNV_RESET        0x00000160 /* Reset value */
#define AD5940_BITP_AFE_REPEATADCCNV_NUM         4          /* Repeat value */
#define AD5940_BITP_AFE_REPEATADCCNV_EN          0          /* Repeat ADC */
#define AD5940_BITM_AFE_REPEATADCCNV_NUM         0x00000ff0 /* Repeat Value */
#define AD5940_BITM_AFE_REPEATADCCNV_EN          0x00000001 /* Repeat ADC */
#define AD5940_ENUM_AFE_REPEATADCCNV_DIS         0x00000000 /* Disable */
#define AD5940_ENUM_AFE_REPEATADCCNV_EN          0x00000001 /* Enable */

/* AFE CMD and DATA FIFO internal data count register
 * Bit 26:16: Current number of words in the data FIFO
 */

#define AD5940_REG_AFE_FIFOCNTSTA                0x2200     /* Addr */
#define AD5940_REG_AFE_FIFOCNTSTA_RESET          0x00000000 /* RST */
#define AD5940_BITP_AFE_FIFOCNTSTA_DATAFIFOCNT   16         /* POS */
#define AD5940_BITM_AFE_FIFOCNTSTA_DATAFIFOCNT   0x07ff0000 /* Mask */

/* AFE calibration data lock register.
 * Bit: 31:0: Password for calibration data registers
 */

#define AD5940_REG_AFE_CALDATLOCK                0x2230     /* Address */
#define AD5940_REG_AFE_CALDATLOCK_RESET          0x00000000 /* Reset value */
#define AD5940_BITP_AFE_CALDATLOCK_KEY           0          /* Password */
#define AD5940_BITM_AFE_CALDATLOCK_KEY           0xffffffff /* Password */

/* AFE ADC offset calibration high speed TIA channel register
 * Bit 14:0: HSTIA offset calibration
 */

#define AD5940_REG_AFE_ADCOFFSETHSTIA            0x2234      /* Address */
#define AD5940_REG_AFE_ADCOFFSETHSTIA_RESET      0x000000000 /* Reset value */
#define AD5940_BITP_AFE_ADCOFFSETHSTIA_VALUE     0           /* Offset CAL */
#define AD5940_BITM_AFE_ADCOFFSETHSTIA_VALUE     0x00007fff  /* Offset CAL */

/* AFE ADC gain calibration temp sensor channel register
 * Bit 14:0: Gain calibration temp sensor channel
 */

#define AD5940_REG_AFE_ADCGAINTEMPSENS0          0x2238     /* Address */
#define AD5940_REG_AFE_ADCGAINTEMPSENS0_RESET    0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGAINTEMPSENS0_VALUE   0          /* Temp gain */
#define AD5940_BITM_AFE_ADCGAINTEMPSENS0_VALUE   0x00007fff /* Temp gain */

/* AFE ADC offset calibration temp sensor channel 0 register
 * Bit 14:0: Offset calibration temp sensor channel
 */

#define AD5940_REG_AFE_ADCOFFSETTEMPSENS0        0x223c     /* Address */
#define AD5940_REG_AFE_ADCOFFSETTEMPSENS0_RESET  0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCOFFSETTEMPSENS0_VALUE 0          /* Temp offset */
#define AD5940_BITM_AFE_ADCOFFSETTEMPSENS0_VALUE 0x00007fff /* Temp offset */

/* AFE ADC gain calibration auxiliary input channel register
 * Bit: 14:0: Gain calibration PGA gain 1x
 */

#define AD5940_REG_AFE_ADCGAINGN1                0x2240     /* Address */
#define AD5940_REG_AFE_ADCGAINGN1_RESET          0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGAINGN1_VALUE         0          /* Gain calibr */
#define AD5940_BITM_AFE_ADCGAINGN1_VALUE         0x00007fff /* Gain calibr */

/* AFE ADC offset calibration auxiliary channel (PGA Gain = 1) register
 * Bit 14:0: Offset calibration gain1
 */

#define AD5940_REG_AFE_ADCOFFSETGN1              0x2244     /* Address */
#define AD5940_REG_AFE_ADCOFFSETGN1_RESET        0x00000000 /* RST value */
#define AD5940_BITP_AFE_ADCOFFSETGN1_VALUE       0          /* Position */
#define AD5940_BITM_AFE_ADCOFFSETGN1_VALUE       0x00007fff /* Mask */

/* AFE DACGAIN register
 * Bit 11:0: HS DAC gain correction factor
 */

#define AD5940_REG_AFE_DACGAIN                   0x2260     /* AFE DACGAIN */
#define AD5940_REG_AFE_DACGAIN_RESET             0x00000800 /* Reset value */
#define AD5940_BITP_AFE_DACGAIN_VALUE            0          /* Gain factor */
#define AD5940_BITM_AFE_DACGAIN_VALUE            0x00000fff /* Gain factor */

/* AFE DAC offset with attenuator enabled (LP mode) register
 * Bit 11:0: DAC offset correction factor
 */

#define AD5940_REG_AFE_DACOFFSETATTEN            0x2264     /* Address */
#define AD5940_REG_AFE_DACOFFSETATTEN_RESET      0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DACOFFSETATTEN_VALUE     0          /* Position */
#define AD5940_BITM_AFE_DACOFFSETATTEN_VALUE     0x00000fff /* Mask */

/* AFE DAC offset with attenuator disabled (LP mode) register
 * Bit 11:0: DAC offset correction factor
 */

#define AD5940_REG_AFE_DACOFFSET                 0x2268     /* Address */
#define AD5940_REG_AFE_DACOFFSET_RESET           0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DACOFFSET_VALUE          0          /* OFFST factor */
#define AD5940_BITM_AFE_DACOFFSET_VALUE          0x00000fff /* OFFST factor */

/* AFE ADC gain calibration auxiliary input channel (PGA Gain = 1.5) register
 * Bit 14:0: Gain calibration PGA gain 1.5x
 */

#define AD5940_REG_AFE_ADCGAINGN1P5              0x2270     /* Address */
#define AD5940_REG_AFE_ADCGAINGN1P5_RESET        0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGAINGN1P5_VALUE       0          /* Gain CAL */
#define AD5940_BITM_AFE_ADCGAINGN1P5_VALUE       0x00007fff /* Gain CAL */

/* AFE ADC gain calibration auxiliary input channel (PGA Gain = 2) register
 *  Bit 14:0: Gain calibration PGA gain 2x
 */

#define AD5940_REG_AFE_ADCGAINGN2                0x2274     /* Address */
#define AD5940_REG_AFE_ADCGAINGN2_RESET          0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGAINGN2_VALUE         0          /* Gain calibr */
#define AD5940_BITM_AFE_ADCGAINGN2_VALUE         0x00007fff /* Gain calibr */

/* AFE ADC gain calibration auxiliary input channel (PGA Gain = 4) register
 *  Bit 14:0: Gain calibration PGA gain 4x
 */

#define AD5940_REG_AFE_ADCGAINGN4                0x2278     /* Address */
#define AD5940_REG_AFE_ADCGAINGN4_RESET          0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGAINGN4_VALUE         0          /* Gain calibr */
#define AD5940_BITM_AFE_ADCGAINGN4_VALUE         0x00007fff /* Gain calibr */

/* AFE ADC Offset Cancellation (Optional)
 * Bit 14:0: Offset cancellation
 */

#define AD5940_REG_AFE_ADCPGAOFFSETCANCEL        0x2280     /* Addr */
#define AD5940_REG_AFE_ADCPGAOFFSETCANCEL_RESET  0x0000     /* RST */
#define AD5940_BITP_AFE_ADCPGAOFFSETCANCEL_CANCL 0          /* POS */
#define AD5940_BITM_AFE_ADCPGAOFFSETCANCEL_CANCL 0x00007fff /* Mask */

/* AFE ADC gain calibration for HS TIA channel register
 * Bit 14:0: Gain error calibration HS TIA channel
 */

#define AD5940_REG_AFE_ADCGNHSTIA                0x2284     /* Address */
#define AD5940_REG_AFE_ADCGNHSTIA_RESET          0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGNHSTIA_VALUE         0          /* Gain Calibr */
#define AD5940_BITM_AFE_ADCGNHSTIA_VALUE         0x00007fff /* Gain Calibr */

/* AFE ADC offset calibration ULP-TIA0/1 channel registers
 * Bit 14:0: Offset calibration for ULP-TIA0/1
 */

#define AD5940_REG_AFE_ADCOFFSETLPTIA0           0x2288     /* Address 0 */
#define AD5940_REG_AFE_ADCOFFSETLPTIA0_RESET     0x00000000 /* Reset value */
#define AD5940_REG_AFE_ADCOFFSETLPTIA1           0x22c0     /* Address 1 */
#define AD5940_BITP_AFE_ADCOFFSETLPTIA0_VALUE    0          /* Offset CAL0 */
#define AD5940_BITM_AFE_ADCOFFSETLPTIA0_VALUE    0x00007fff /* Offset CAL0 */
#define AD5940_BITP_AFE_ADCOFFSETLPTIA1_VALUE    0          /* Offset CAL1 */
#define AD5940_BITM_AFE_ADCOFFSETLPTIA1_VALUE    0x00007fff /* Offset CAL1 */

/* AFE ADC gain calibration for LP TIA0 channel
 * Bit 14:0: Gain error calibration ULPTIA0
 */

#define AD5940_REG_AFE_ADCGNLPTIA0               0x228c     /* Address */
#define AD5940_REG_AFE_ADCGNLPTIA0_RESET         0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGNLPTIA0_VALUE        0          /* Gain calibr */
#define AD5940_BITM_AFE_ADCGNLPTIA0_VALUE        0x00007fff /* Gain calibr */

/* AFE ADC gain calibration with DC cancellation (PGA G = 4) register
 * Bit 14:0: DC calibration gain = 4
 */

#define AD5940_REG_AFE_ADCPGAGN4OFCAL            0x2294     /* ADDR */
#define AD5940_REG_AFE_ADCPGAGN4OFCAL_RESET      0x00004000 /* RST */
#define AD5940_BITP_AFE_ADCPGAGN4OFCAL_GAINAUX   0          /* POS */
#define AD5940_BITM_AFE_ADCPGAGN4OFCAL_GAINAUX   0x00007fff /* Mask */

/* AFE ADC gain calibration auxiliary input channel (PGA gain = 9) register
 * Bit 14:0: Gain calibration PGA gain 9x
 */

#define AD5940_REG_AFE_ADCGAINGN9                0x2298     /* Address */
#define AD5940_REG_AFE_ADCGAINGN9_RESET          0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGAINGN9_VALUE         0          /* Gain calibr */
#define AD5940_BITM_AFE_ADCGAINGN9_VALUE         0x00007fff /* Gain calibr */

/* AFE ADC offset calibration temp sensor channel 1 register
 * Bit 14:0: Offset calibration temp sensor
 */

#define AD5940_REG_AFE_ADCOFFSETEMPSENS1         0x22a8     /* Address */
#define AD5940_REG_AFE_ADCOFFSETEMPSENS1_RESET   0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCOFFSETEMPSENS1_VALUE  0          /* Offset CAL */
#define AD5940_BITM_AFE_ADCOFFSETEMPSENS1_VALUE  0x00007fff /* Offset CAL */

/* AFE ADC gain calibration diode temperature sensor channel register
 * Bit 14:0: Gain calibration for diode temp sensor
 */

#define AD5940_REG_AFE_ADCGAINDIOTEMPSENS        0x22ac     /* Address */
#define AD5940_REG_AFE_ADCGAINDIOTEMPSENS_RESET  0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGAINDIOTEMPSENS_VALUE 0          /* Gain CALIBR */
#define AD5940_BITM_AFE_ADCGAINDIOTEMPSENS_VALUE 0x00007fff /* Gain CALIBR */

/* AFE DAC offset with attenuator enabled (HP mode) register
 * Bit 14:0: DAC Offset correction factor
 */

#define AD5940_REG_AFE_DACOFFSETATTENHP          0x22B8     /* Address */
#define AD5940_REG_AFE_DACOFFSETATTENHP_RESET    0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DACOFFSETATTENHP_VALUE   0          /* Offset CAL */
#define AD5940_BITM_AFE_DACOFFSETATTENHP_VALUE   0x00000fff /* Offset CAL */

/* AFE DAC offset with attenuator disabled (HP mode) register
 * Bit 11:0: DAC offset correction factor
 */

#define AD5940_REG_AFE_DACOFFSETHP               0x22bc     /* Address */
#define AD5940_REG_AFE_DACOFFSETHP_RESET         0x00000000 /* Reset value */
#define AD5940_BITP_AFE_DACOFFSETHP_VALUE        0          /* Offset CAL */
#define AD5940_BITM_AFE_DACOFFSETHP_VALUE        0x00000fff /* Offset CAL */

/* AFE ADC gain calibration for LP TIA1 channel register
 * Bit 14:0: Gain Calibration ULP-TIA1
 */

#define AD5940_REG_AFE_ADCGNLPTIA1               0x22c4     /* Address */
#define AD5940_REG_AFE_ADCGNLPTIA1_RESET         0x00004000 /* Reset value */
#define AD5940_BITP_AFE_ADCGNLPTIA1_ULPTIA1GN    0          /* Gain CALIBR */
#define AD5940_BITM_AFE_ADCGNLPTIA1_ULPTIA1GN    0x00007fff /* Gain CALIBR */

/* AFE offset calibration auxiliary channel (PGA gain = 2)
 * Bit 14:0: offset calibration auxiliary channel (PGA gain = 2)
 */

#define AD5940_REG_AFE_ADCOFFSETGN2              0x22c8     /* Address */
#define AD5940_REG_AFE_ADCOFFSETGN2_RESET        0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCOFFSETGN2_VALUE       0          /* Offset CAL */
#define AD5940_BITM_AFE_ADCOFFSETGN2_VALUE       0x00007fff /* Offset CAL */

/* AFE offset calibration auxiliary channel (PGA gain = 1.5)
 * Bit 14:0: offset calibration auxiliary channel (PGA gain = 1.5)
 */

#define AD5940_REG_AFE_ADCOFFSETGN1P5            0x22cc     /* Address */
#define AD5940_REG_AFE_ADCOFFSETGN1P5_RESET      0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCOFFSETGN1P5_VALUE     0          /* Offset CAL */
#define AD5940_BITM_AFE_ADCOFFSETGN1P5_VALUE     0x00007fff /* Offset CAL */

/* AFE offset calibration auxiliary channel (PGA gain = 9)
 * Bit 14:0: offset calibration auxiliary channel (PGA gain = 9)
 */

#define AD5940_REG_AFE_ADCOFFSETGN9              0x22d0     /* Address */
#define AD5940_REG_AFE_ADCOFFSETGN9_RESET        0x00000000 /* Reset value */
#define AD5940_BITP_AFE_ADCOFFSETGN9_VALUE       0          /* Offset CAL */
#define AD5940_BITM_AFE_ADCOFFSETGN9_VALUE       0x00007fff /* Offset CAL */

/* AFE offset calibration auxiliary channel (PGA gain = 4)
 * Bit 14:0: offset calibration auxiliary channel (PGA gain = 4)
 */

#define AD5940_REG_AFE_ADCOFFSETGN4              0x22d4      /* Address */
#define AD5940_REG_AFE_ADCOFFSETGN4_RESET        0x000000000 /* Reset value */
#define AD5940_BITP_AFE_ADCOFFSETGN4_VALUE       0           /* Offset CAL */
#define AD5940_BITM_AFE_ADCOFFSETGN4_VALUE       0x00007fff  /* Offset CAL */

/* AFE Power Mode Configuration
 * Bit 3:2: Configure system bandwidth
 *          0 - No action for system configuration
 *          1 - 50kHz -3dB bandwidth
 *          2 - 100kHz -3dB bandwidth
 *          3 - 250kHz -3dB bandwidth
 * Bit   0: Set high speed DAC and ADC in high power mode
 *          0 - Low power mode
 *          1 - High power mode
 */

#define AD5940_REG_AFE_PMBW                      0x22f0     /* Address */
#define AD5940_REG_AFE_PMBW_RESET                0x00088800 /* Reset value */
#define AD5940_BITP_AFE_PMBW_SYSBW               2          /* System BW */
#define AD5940_BITP_AFE_PMBW_SYSHP               0          /* H-power mode */
#define AD5940_BITM_AFE_PMBW_SYSBW               0x0000000c /* System BW */
#define AD5940_BITM_AFE_PMBW_SYSHP               0x00000001 /* H-power mode */
#define AD5940_ENUM_AFE_PMBW_BWNA                0x00000000 /* BW: None */
#define AD5940_ENUM_AFE_PMBW_BW50                0x00000004 /* BW: 50kHz */
#define AD5940_ENUM_AFE_PMBW_BW100               0x00000008 /* BW: 100kHz */
#define AD5940_ENUM_AFE_PMBW_BW250               0x0000000c /* BW: 250kHz */
#define AD5940_ENUM_AFE_PMBW_LP                  0x00000000 /* HP: LP mode */
#define AD5940_ENUM_AFE_PMBW_HP                  0x00000001 /* HP: HP mode */

/* AFE switch MUX for ECG
 * Bit 3: CM resistor select for Ain2, Ain3
 */

#define AD5940_REG_AFE_SWMUX                     0x235c     /* Address */
#define AD5940_REG_AFE_SWMUX_RESET               0x00000000 /* Reset value */
#define AD5940_BITP_AFE_SWMUX_CMMUX              3          /* CM resistor */
#define AD5940_BITM_AFE_SWMUX_CMMUX              0x00000008 /* CM resistor */

/* AFE AFE_TEMPSEN_DIO register
 * Bit   17: Power down control
 * Bit   16: Test signal enable
 * Bit 15:0: Bias current selection
 */

#define AD5940_REG_AFE_AFE_TEMPSEN_DIO           0x2374     /* Address */
#define AD5940_REG_AFE_AFE_TEMPSEN_DIO_RESET     0x00020000 /* RST */
#define AD5940_BITP_AFE_AFE_TEMPSEN_DIO_TSDIOPD  17         /* POS */
#define AD5940_BITP_AFE_AFE_TEMPSEN_DIO_TSDIOEN  16         /* POS */
#define AD5940_BITP_AFE_AFE_TEMPSEN_DIO_TSDIOCON 0          /* POS */
#define AD5940_BITM_AFE_AFE_TEMPSEN_DIO_TSDIOPD  0x00020000 /* Mask */
#define AD5940_BITM_AFE_AFE_TEMPSEN_DIO_TSDIOEN  0x00010000 /* Mask*/
#define AD5940_BITM_AFE_AFE_TEMPSEN_DIO_TSDIOCON 0x0000ffff /* Mask */

/* AFE Configure ADC Input Buffer */

#define AD5940_REG_AFE_ADCBUFCON                 0x238c     /* Address */
#define AD5940_REG_AFE_ADCBUFCON_RESET           0x005f3d00 /* Reset value */
#define AD5940_BITP_AFE_ADCBUFCON_AMPDIS         4          /* Disable OpAmp */
#define AD5940_BITP_AFE_ADCBUFCON_CHOPDIS        0          /* Disable Chop */
#define AD5940_BITM_AFE_ADCBUFCON_AMPDIS         0x000001f0 /* Disable OpAmp */
#define AD5940_BITM_AFE_ADCBUFCON_CHOPDIS        0x0000000f /* Disable Chop */

/* Part1.9 Interrupt Controller Register Map registers (32 bit) */

/* INTC interrupt polarity register
 * Bit 0: Interrupt polarity
 *        0 - Output negative edge interrupt
 *        1 - Output positive edge interrupt
 */

#define AD5940_REG_INTC_INTCPOL           0x3000     /* Address*/
#define AD5940_REG_INTC_INTCPOL_RESET     0x00000000 /* Reset value */
#define AD5940_BITP_INTC_INTCPOL_INTPOL   0          /* Interrupt polarity */
#define AD5940_BITM_INTC_INTCPOL_INTPOL   0x00000001 /* Interrupt polarity */
#define AD5940_ENUM_INTC_INTCPOL_FALLING  0x00000000 /* Negative edge */
#define AD5940_ENUM_INTC_INTCPOL_RISING   0x00000001 /* Positive edge */

/* INTC interrupt clear register. Write 1 to clear. */

#define AD5940_REG_INTC_INTCCLR           0x3004     /* Address */
#define AD5940_REG_INTC_INTCCLR_RESET     0x00000000 /* Reset value */
#define AD5940_BITP_INTC_INTCCLR_INTCLR31 31         /* Attempt to break */
#define AD5940_BITP_INTC_INTCCLR_INTCLR30 30         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR29 29         /* Outlier IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR28 28         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR27 27         /* Data FIFO underflow */
#define AD5940_BITP_INTC_INTCCLR_INTCLR26 26         /* Data FIFO overflow */
#define AD5940_BITP_INTC_INTCCLR_INTCLR25 25         /* Data FIFO threshold */
#define AD5940_BITP_INTC_INTCCLR_INTCLR24 24         /* Data FIFO empty IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR23 23         /* Data FIFO full IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR22 22         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR21 21         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR20 20         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR19 19         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR18 18         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR17 17         /* SEQ timeout error */
#define AD5940_BITP_INTC_INTCCLR_INTCLR16 16         /* SEQ timeout finish */
#define AD5940_BITP_INTC_INTCCLR_INTCLR15 15         /* Sequencer end IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR14 14         /* Reserved */
#define AD5940_BITP_INTC_INTCCLR_INTCLR13 13         /* Boot load done IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR12 12         /* Custom IRQ 3 */
#define AD5940_BITP_INTC_INTCCLR_INTCLR11 11         /* Custom IRQ 2 */
#define AD5940_BITP_INTC_INTCCLR_INTCLR10 10         /* Custom IRQ 1 */
#define AD5940_BITP_INTC_INTCCLR_INTCLR9  9          /* Custom IRQ 0 */
#define AD5940_BITP_INTC_INTCCLR_INTCLR8  8          /* Varience IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR7  7          /* Mean IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR6  6          /* ADC delta fail IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR5  5          /* ADC max. fail IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR4  4          /* ADC min. fail IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR3  3          /* Temp result IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR2  2          /* SINC2 result ready */
#define AD5940_BITP_INTC_INTCCLR_INTCLR1  1          /* DFT result IRQ */
#define AD5940_BITP_INTC_INTCCLR_INTCLR0  0          /* ADC result IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR31 0x80000000 /* Attempt to break */
#define AD5940_BITM_INTC_INTCCLR_INTCLR30 0x40000000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR29 0x20000000 /* Outlier IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR28 0x10000000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR27 0x08000000 /* Data FIFO underflow */
#define AD5940_BITM_INTC_INTCCLR_INTCLR26 0x04000000 /* Data FIFO overflow */
#define AD5940_BITM_INTC_INTCCLR_INTCLR25 0x02000000 /* Data FIFO threshold */
#define AD5940_BITM_INTC_INTCCLR_INTCLR24 0x01000000 /* Data FIFO empty IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR23 0x00800000 /* Data FIFO full IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR22 0x00400000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR21 0x00200000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR20 0x00100000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR19 0x00080000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR18 0x00040000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR17 0x00020000 /* SEQ timeout error */
#define AD5940_BITM_INTC_INTCCLR_INTCLR16 0x00010000 /* SEQ timeout finish */
#define AD5940_BITM_INTC_INTCCLR_INTCLR15 0x00008000 /* Sequencer end IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR14 0x00004000 /* Reserved */
#define AD5940_BITM_INTC_INTCCLR_INTCLR13 0x00002000 /* Boot load done IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR12 0x00001000 /* Custom IRQ 3 */
#define AD5940_BITM_INTC_INTCCLR_INTCLR11 0x00000800 /* Custom IRQ 2 */
#define AD5940_BITM_INTC_INTCCLR_INTCLR10 0x00000400 /* Custom IRQ 1 */
#define AD5940_BITM_INTC_INTCCLR_INTCLR9  0x00000200 /* Custom IRQ 0 */
#define AD5940_BITM_INTC_INTCCLR_INTCLR8  0x00000100 /* Varience IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR7  0x00000080 /* Mean IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR6  0x00000040 /* ADC delta fail IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR5  0x00000020 /* ADC max. fail IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR4  0x00000010 /* ADC min. fail IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR3  0x00000008 /* Temp result IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR2  0x00000004 /* SINC2 result ready */
#define AD5940_BITM_INTC_INTCCLR_INTCLR1  0x00000002 /* DFT result IRQ */
#define AD5940_BITM_INTC_INTCCLR_INTCLR0  0x00000001 /* ADC result IRQ */

/* INTC INT0 select register. Write 1 to enable. */

#define AD5940_REG_INTC_INTCSEL0          0x3008     /* Address */
#define AD5940_REG_INTC_INTCSEL0_RESET    0x00002000 /* Reset value */
#define AD5940_BITP_INTC_INTCSEL_INTSEL31 31         /* Attempt to break */
#define AD5940_BITP_INTC_INTCSEL_INTSEL30 30         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL29 29         /* Outlier IRQ enable */
#define AD5940_BITP_INTC_INTCSEL_INTSEL28 28         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL27 27         /* DataFIFO underflow */
#define AD5940_BITP_INTC_INTCSEL_INTSEL26 26         /* DataFIFO overflow */
#define AD5940_BITP_INTC_INTCSEL_INTSEL25 25         /* DataFIFO threshold */
#define AD5940_BITP_INTC_INTCSEL_INTSEL24 24         /* DataFIFO empty IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL23 23         /* DataFIFO full IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL22 22         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL21 21         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL20 20         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL19 19         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL18 18         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL17 17         /* SEQ timeout error */
#define AD5940_BITP_INTC_INTCSEL_INTSEL16 16         /* SEQ timeout finish */
#define AD5940_BITP_INTC_INTCSEL_INTSEL15 15         /* Sequencer end IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL14 14         /* Reserved */
#define AD5940_BITP_INTC_INTCSEL_INTSEL13 13         /* Boot load done IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL12 12         /* Custom IRQ3 enable */
#define AD5940_BITP_INTC_INTCSEL_INTSEL11 11         /* Custom IRQ2 enable */
#define AD5940_BITP_INTC_INTCSEL_INTSEL10 10         /* Custom IRQ1 enable */
#define AD5940_BITP_INTC_INTCSEL_INTSEL9  9          /* Custom IRQ0 enable */
#define AD5940_BITP_INTC_INTCSEL_INTSEL8  8          /* Varience IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL7  7          /* Mean IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL6  6          /* ADC delta fail IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL5  5          /* ADC max. fail IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL4  4          /* ADC min. fail IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL3  3          /* Temp result IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL2  2          /* SINC2 result ready */
#define AD5940_BITP_INTC_INTCSEL_INTSEL1  1          /* DFT result IRQ */
#define AD5940_BITP_INTC_INTCSEL_INTSEL0  0          /* ADC result IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL31 0x80000000 /* Attempt to break */
#define AD5940_BITM_INTC_INTCSEL_INTSEL30 0x40000000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL29 0x20000000 /* Outlier IRQ enable */
#define AD5940_BITM_INTC_INTCSEL_INTSEL28 0x10000000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL27 0x08000000 /* DataFIFO underflow */
#define AD5940_BITM_INTC_INTCSEL_INTSEL26 0x04000000 /* DataFIFO overflow */
#define AD5940_BITM_INTC_INTCSEL_INTSEL25 0x02000000 /* DataFIFO threshold */
#define AD5940_BITM_INTC_INTCSEL_INTSEL24 0x01000000 /* DataFIFO empty IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL23 0x00800000 /* DataFIFO full IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL22 0x00400000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL21 0x00200000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL20 0x00100000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL19 0x00080000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL18 0x00040000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL17 0x00020000 /* SEQ timeout error */
#define AD5940_BITM_INTC_INTCSEL_INTSEL16 0x00010000 /* SEQ timeout finish */
#define AD5940_BITM_INTC_INTCSEL_INTSEL15 0x00008000 /* Sequencer end IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL14 0x00004000 /* Reserved */
#define AD5940_BITM_INTC_INTCSEL_INTSEL13 0x00002000 /* Boot load done IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL12 0x00001000 /* Custom IRQ3 enable */
#define AD5940_BITM_INTC_INTCSEL_INTSEL11 0x00000800 /* Custom IRQ2 eEnable */
#define AD5940_BITM_INTC_INTCSEL_INTSEL10 0x00000400 /* Custom IRQ1 enable */
#define AD5940_BITM_INTC_INTCSEL_INTSEL9  0x00000200 /* Custom IRQ0 enable */
#define AD5940_BITM_INTC_INTCSEL_INTSEL8  0x00000100 /* Varience IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL7  0x00000080 /* Mean IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL6  0x00000040 /* ADC delta fail IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL5  0x00000020 /* ADC max. fail IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL4  0x00000010 /* ADC min. fail IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL3  0x00000008 /* Temp result IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL2  0x00000004 /* SINC2 result ready */
#define AD5940_BITM_INTC_INTCSEL_INTSEL1  0x00000002 /* DFT result IRQ */
#define AD5940_BITM_INTC_INTCSEL_INTSEL0  0x00000001 /* ADC result IRQ */

/* INTC INT1 select register. Write 1 to enable. See INTTCSEL0 for detail */

#define AD5940_REG_INTC_INTCSEL1          0x300c     /* Address */
#define AD5940_REG_INTC_INTCSEL1_RESET    0x00000000 /* Reset value */

/* INTC INT0 flag register. 1 indicates that an interrupt is asserted. */

#define AD5940_REG_INTC_INTCFLAG0         0x3010     /* Address */
#define AD5940_REG_INTC_INTCFLAG0_RESET   0x00000000 /* Reset value */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG31 31         /* Attempt to break */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG30 30         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG29 29         /* Outlier IRQ status */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG28 28         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG27 27         /* Data FIFO underflow */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG26 26         /* Data FIFO overflow */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG25 25         /* Data FIFO threshold */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG24 24         /* Data FIFO empty IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG23 23         /* Data FIFO full IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG22 22         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG21 21         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG20 20         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG19 19         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG18 18         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG17 17         /* SEQ timeout error */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG16 16         /* SEQ timeout finish */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG15 15         /* Sequencer end IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG14 14         /* Reserved */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG13 13         /* Boot load done IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG12 12         /* Custom IRQ 3 Status */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG11 11         /* Custom IRQ 2 Status */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG10 10         /* Custom IRQ 1 Status */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG9  9          /* Custom IRQ 0 Status */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG8  8          /* Variance IRQ status */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG7  7          /* Mean IRQ status */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG6  6          /* ADC delta fail IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG5  5          /* ADC max. fail IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG4  4          /* ADC min. fail IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG3  3          /* Temp result IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG2  2          /* SINC2 result ready */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG1  1          /* DFT result IRQ */
#define AD5940_BITP_INTC_INTCFLAGX_FLAG0  0          /* ADC result IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG31 0x80000000 /* Attempt to break */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG30 0x40000000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG29 0x20000000 /* Outlier IRQ status */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG28 0x10000000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG27 0x08000000 /* Data FIFO underflow */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG26 0x04000000 /* Data FIFO overflow */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG25 0x02000000 /* Data FIFO threshold */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG24 0x01000000 /* Data FIFO empty IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG23 0x00800000 /* Data FIFO full IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG22 0x00400000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG21 0x00200000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG20 0x00100000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG19 0x00080000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG18 0x00040000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG17 0x00020000 /* SEQ timeout error */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG16 0x00010000 /* SEQ timeout finish */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG15 0x00008000 /* Sequencer end IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG14 0x00004000 /* Reserved */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG13 0x00002000 /* Boot load done IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG12 0x00001000 /* Custom IRQ3 Status */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG11 0x00000800 /* Custom IRQ2 Status */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG10 0x00000400 /* Custom IRQ1 Status */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG9  0x00000200 /* Custom IRQ0 Status */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG8  0x00000100 /* Variance IRQ status */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG7  0x00000080 /* Mean IRQ status */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG6  0x00000040 /* ADC delta fail IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG5  0x00000020 /* ADC max. fail IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG4  0x00000010 /* ADC min. fail IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG3  0x00000008 /* Temp result IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG2  0x00000004 /* SINC2 result ready */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG1  0x00000002 /* DFT result IRQ */
#define AD5940_BITM_INTC_INTCFLAGX_FLAG0  0x00000001 /* ADC result IRQ */

/* INTC INT1 flag register. See INTCFLAG0 for detail */

#define AD5940_REG_INTC_INTCFLAG1         0x3014     /* Address */
#define AD5940_REG_INTC_INTCFLAG1_RESET   0x00000000 /* Reset value */

/* Part2. Device natives */

/* Part2.1 Register address natives */

#define AD5940_REGADDR_ENDOF16            0x0ffe /* <=0x0ffe, a REG has 16b */
#define AD5940_REGADDR_STARTOF32          0x1000 /* >=0x1000, a REG has 32b */
#define AD5940_REGADDR_ENDOF32            0x3014 /* The last 32b REG addr */
#define AD5940_REGADDR_ENDOFSEQ           0x21ff /* SEQ can't deal >0x3000 */

/* Part2.2 FIFO natives */

#define AD5940_BYTES_PER_RESULT           4      /* Each result  has 4bytes */
#define AD5940_RESULTS_PER_MEAS           4      /* Each MEAS has 4 data */

/* Last 2 results should be read out using non-zero offset on SPI */

#define AD5940_RDFIFO_TAIL_SIZE           2
#define AD5940_RDFIFO_CRITIC              3      /* If <3 data, read as REG */

/* Part2.3 Chip natives */

#define AD5940_SWRST_TIME                 200    /* Softreset time < 200 us */
#define AD5940_RST_LOW_TIME               2000   /* In reset: 2000us */
#define AD5940_RST_HIGH_TIME              5000   /* After reset: 5000us */

/* Higher ADC CLK frequency needs a 32MHz source instead of a 16MHz source. */

#define AD5940_ADCCLK_THRESHD             25600000

/* Part2.4 SPI commands and natives */

#define AD5940_SPI_MAX_FREQ               16000000   /* Maximum SCLK 16MHz */
#define AD5940_SPI_MIN_FREQ               100000     /* Minimum SCLK 100kHz */
#define AD5940_SPI_NBITS                  8          /* Only 8b/word is OK */

#define AD5940_SPICMD_SETADDR             0x20   /* Set address of register */
#define AD5940_SPICMD_READREG             0x6d   /* Read register */
#define AD5940_SPICMD_WRITEREG            0x2d   /* Write register */
#define AD5940_SPICMD_READFIFO            0x5f   /* Read FIFO */

#define AD5940_SPICNT_SETADDR             3      /* Set address: trans 3B */
#define AD5940_SPICNT_PREREAD             2      /* Before read: trans 2B */
#define AD5940_SPICNT_READREG16           4      /* Read 16b REG: trans 4B */
#define AD5940_SPICNT_READREG32           6      /* Read 32b REG: trans 6B */
#define AD5940_SPICNT_WRITEREG16          3      /* Write 16b REG: trans 3B */
#define AD5940_SPICNT_WRITEREG32          5      /* Write 32b REG: trans 5B */
#define AD5940_SPICNT_PREFIFO             7      /* Pre-read fifo: trans 7B */
#define AD5940_SPIOFFSET_NONZERO          0x44   /* Non-zero trans offset */

/* Part2.5 Misc natives */

/* 1 s = 1000000 us. */

#define AD5940_ONE_SECOND                 1000000.0f

/* Ceiling operation */

#define AD5940_CEILING(x, y)              ((x) + ((y) - 1)) / (y)

/* Part3. Enumerations/constants for AFE */

/* Part3.1 AFE control */

/* AFE Interrupt controller selection.
 * AD5940 has two interrupt controller INTC0 and INTC1. Both of them have
 * ability to generate interrupt signal from GPIO.
 */

#define AD5940_AFEINTC_0                  0      /* Interrupt controller 0 */
#define AD5940_AFEINTC_1                  1      /* Interrupt controller 1 */

/* AFE Interrupt source selection.
 * These sources are defined as bit mask. They are available for register
 * INTCCLR, INTCSEL0/1, INTCFLAG0/1
 */

/* Bit0, ADC result ready status */

#define AD5940_AFEINTSRC_ADCRDY           0x00000001

/* Bit1, DFT result ready status */

#define AD5940_AFEINTSRC_DFTRDY           0x00000002

/* Bit2, SINC2/low pass filter result status */

#define AD5940_AFEINTSRC_SINC2RDY         0x00000004

/* Bit3, temperature sensor result ready */

#define AD5940_AFEINTSRC_TEMPRDY          0x00000008

/* Bit4, ADC minimum value */

#define AD5940_AFEINTSRC_ADCMINERR        0x00000010

/* Bit5, ADC maximum value */

#define AD5940_AFEINTSRC_ADCMAXERR        0x00000020

/* Bit6, ADC delta ready */

#define AD5940_AFEINTSRC_ADCDIFFERR       0x00000040

/* Bit7, mean result ready */

#define AD5940_AFEINTSRC_MEANRDY          0x00000080

/* Bit8, variance result ready */

#define AD5940_AFEINTSRC_VARRDY           0x00000100

/* Bit9, custom interrupt source 0. It happens when sequencer writes 1 to
 * register AD5940_REG_AFE_AFEGENINTSTA.bit0
 */

#define AD5940_AFEINTSRC_CUSTOMINT0       0x00000200

/* Bit10, custom interrupt source 1. It happens when sequencer writes 1 to
 * register AD5940_REG_AFE_AFEGENINTSTA.bit1
 */

#define AD5940_AFEINTSRC_CUSTOMINT1       0x00000400

/* Bit11, custom interrupt source 2. It happens when sequencer writes 1 to
 * register AD5940_REG_AFE_AFEGENINTSTA.bit2
 */

#define AD5940_AFEINTSRC_CUSTOMINT2       0x00000800

/* Bit12, custom interrupt source 3. It happens when sequencer writes 1 to
 * register AD5940_REG_AFE_AFEGENINTSTA.bit3
 */

#define AD5940_AFEINTSRC_CUSTOMINT3       0x00001000

/* Bit13, OTP boot loading done */

#define AD5940_AFEINTSRC_BOOTLDDONE       0x00002000

/* Bit14, AFE woken up */

#define AD5940_AFEINTSRC_WAKEUP           0x00004000

/* Bit15, end of sequence interrupt */

#define AD5940_AFEINTSRC_ENDSEQ           0x00008000

/* Bit16, sequencer timeout command finished */

#define AD5940_AFEINTSRC_SEQTIMEOUT       0x00010000

/* Bit17, sequencer timeout command error */

#define AD5940_AFEINTSRC_SEQTIMEOUTERR    0x00020000

/* Bit18, command fifo full interrupt */

#define AD5940_AFEINTSRC_CMDFIFOFULL      0x00040000

/* Bit19, command fifo empty */

#define AD5940_AFEINTSRC_CMDFIFOEMPTY     0x00080000

/* Bit20, command fifo threshold interrupt */

#define AD5940_AFEINTSRC_CMDFIFOTHRESH    0x00100000

/* Bit21, command fifo overflow interrupt */

#define AD5940_AFEINTSRC_CMDFIFOOF        0x00200000

/* Bit22, command fifo underflow interrupt. */

#define AD5940_AFEINTSRC_CMDFIFOUF        0x00400000

/* Bit23, data fifo full interrupt. */

#define AD5940_AFEINTSRC_DATAFIFOFULL     0x00800000

/* Bit24, data fifo empty */

#define AD5940_AFEINTSRC_DATAFIFOEMPTY    0x01000000

/* Bit25, data fifo threshold interrupt. */

#define AD5940_AFEINTSRC_DATAFIFOTHRESH   0x02000000

/* Bit26, data fifo overflow interrupt. */

#define AD5940_AFEINTSRC_DATAFIFOOF       0x04000000

/* Bit27, data fifo underflow interrupt. */

#define AD5940_AFEINTSRC_DATAFIFOUF       0x08000000

/* Bit28, WDT timeout interrupt. */

#define AD5940_AFEINTSRC_WDTIRQ           0x10000000

/* Bit29, outlier int for AD5940 */

#define AD5940_AFEINTSRC_CRC_OUTLIER      0x20000000

/* Bit30, sleep or wakeup timer timeout for AD5940 */

#define AD5940_AFEINTSRC_GPT0INT_SLPWUT   0x40000000

/* Bit31, tried to break IRQ for AD5940 */

#define AD5940_AFEINTSRC_GPT1INT_TRYBRK   0x80000000

/* Mask of all interrupts */

#define AD5940_AFEINTSRC_ALLINT           0xffffffff

/* AFE power mode.
 * It will set the whole analog system power mode include HSDAC, excitation
 * buffer, HSTIA, ADC front-buffer etc. For signal <80kHz, use low power
 * mode; for signal >80kHz, use high power mode.
 */

#define AD5940_AFEPWR_LP                  0      /* Set to low power mode */
#define AD5940_AFEPWR_HP                  1      /* Set to high power mode */

/* AFE system bandwidth.
 * It will set the whole analog bandwidth include HSDAC, excitation buffer,
 * HSTIA, ADC front-buffer etc.
 * 0 - Set the bandwidth automatically based on WGFCW frequency word.
 * 1 - 50kHZ system bandwidth(DAC/ADC)
 * 2 - 100kHZ system bandwidth(DAC/ADC)
 * 3 - 250kHZ system bandwidth(DAC/ADC)
 */

#define AD5940_AFEBW_AUTOSET              0      /* Bandwidth automatically */
#define AD5940_AFEBW_50KHZ                1      /* 50kHZ system bandwidth */
#define AD5940_AFEBW_100KHZ               2      /* 100kHZ system bandwidth */
#define AD5940_AFEBW_250KHZ               3      /* 250kHZ system bandwidth */

/* AFE Control signal set. Bit masks for register AFECON.
 * This's all the available control signal for register AD5940_REG_AFE_AFECON
 * Bit field in register AFECON has some opposite meaning as below
 * definitions. We use all positive word here like HPREF instead of HPREFDIS.
 * This set is only used for some BSP functions, where the parameter decides
 * whether enable it or disable it.
 * Bit  5: High power reference on-off control
 * Bit  6: High speed DAC on-off control
 * Bit  7: ADC power on-off control
 * Bit  8: Start ADC convert enable
 * Bit  9: Excitation buffer power control
 * Bit 10: Excitation loop input amplifier before P/N node power control
 * Bit 11: High speed TIA amplifier power control
 * Bit 12: Temperature sensor power
 * Bit 13: Start temperature sensor convert
 * Bit 14: Waveform generator on-off control
 * Bit 15: DFT engine on-off control
 * Bit 16: SIN2+Notch block on-off control
 * Bit 19: ALDO current limit on-off control
 * Bit 20: DAC reference buffer power control
 * Bit 21: Excitation loop DC offset buffer sourced from LPDAC power control
 */

#define AD5940_AFECTRL_HPREFPWR        (1 << 5)  /* High power reference */
#define AD5940_AFECTRL_HSDACPWR        (1 << 6)  /* High speed DAC */
#define AD5940_AFECTRL_ADCPWR          (1 << 7)  /* ADC power on-off */
#define AD5940_AFECTRL_ADCCNV          (1 << 8)  /* Start ADC convert */
#define AD5940_AFECTRL_EXTBUFPWR       (1 << 9)  /* Excitation buffer power */
#define AD5940_AFECTRL_INAMPPWR        (1 << 10) /* Excitation input amp */
#define AD5940_AFECTRL_HSTIAPWR        (1 << 11) /* High speed TIA amp */
#define AD5940_AFECTRL_TEMPSPWR        (1 << 12) /* Temperature sensor */
#define AD5940_AFECTRL_TEMPCNV         (1 << 13) /* Start temp sensor */
#define AD5940_AFECTRL_WG              (1 << 14) /* Waveform generator */
#define AD5940_AFECTRL_DFT             (1 << 15) /* DFT engine on-off */
#define AD5940_AFECTRL_SINC2NOTCH      (1 << 16) /* SIN2+Notch block */
#define AD5940_AFECTRL_ALDOLIMIT       (1 << 19) /* ALDO current limit */
#define AD5940_AFECTRL_DACREFPWR       (1 << 20) /* DAC reference buffer */
#define AD5940_AFECTRL_DCBUFPWR        (1 << 21) /* Excitation offsst buff */
#define AD5940_AFECTRL_ALL             0x39ffe0  /* All control signals */

/* LP Control signal(bit mask) for register LPMODECON
 * This is all the available control signal for some function to control
 * low power mode Bit field in register LPMODECON has some opposite meaning
 * as below definitions. We use all positive word here like HPREFPWR instead
 * of HPREFDIS. This set is only used in the mentioned functions, where the
 * parameter decides whether enable or disable selected block(s).
 * Bit 0: Enable internal HFOSC. Note: the register defination is set this
 *        bit to 1 to disable it.
 * Bit 1: High power reference power EN. Note: the register defination is set
 *        this bit to 1 to disable it.
 * Bit 2: Start ADC convert enable
 * Bit 3: Enable repeat convert function. This will enable ADC power
 *        automatically.
 * Bit 4: Enable Global ZTAT bias. Disable it to save more power.
 * Bit 5: Enable Global PTAT bias. Disable it to save more power.
 * Bit 6: High power 1.8V reference buffer
 * Bit 7: High power 1.1V reference buffer
 * Bit 8: Enable ALDO. Note: register defination is set this bit to 1 to
 *        disable ALDO.
 */

#define AD5940_LPMODECTRL_HFOSCEN      (1 << 0)  /* Enable internal HFOSC */
#define AD5940_LPMODECTRL_HPREFPWR     (1 << 1)  /* High power ref power EN */
#define AD5940_LPMODECTRL_ADCCNV       (1 << 2)  /* Start ADC convert EN */
#define AD5940_LPMODECTRL_REPEATEN     (1 << 3)  /* Enable repeat convert */
#define AD5940_LPMODECTRL_GLBBIASZ     (1 << 4)  /* Enable Global ZTAT bias */
#define AD5940_LPMODECTRL_GLBBIASP     (1 << 5)  /* Enable Global PTAT bias */
#define AD5940_LPMODECTRL_BUFHP1P8V    (1 << 6)  /* High power 1V8 ref buff */
#define AD5940_LPMODECTRL_BUFHP1P1V    (1 << 7)  /* High power 1V1 ref buff */
#define AD5940_LPMODECTRL_ALDOPWR      (1 << 8)  /* Enable ALDO */
#define AD5940_LPMODECTRL_ALL          0x01ff    /* All Control signal */
#define AD5940_LPMODECTRL_NONE         0         /* No blocks selected */

/* The available AFE results type. */

#define AD5940_AFERESULT_SINC3         0         /* SINC3 result */
#define AD5940_AFERESULT_SINC2         1         /* SINC2+NOTCH result */
#define AD5940_AFERESULT_TEMPSENSOR    2         /* Temp sensor result */
#define AD5940_AFERESULT_DFTREAL       3         /* DFT real result */
#define AD5940_AFERESULT_DFTIMAGE      4         /* DFT imaginary result */
#define AD5940_AFERESULT_STATSMEAN     5         /* Statistic mean result */
#define AD5940_AFERESULT_STATSVAR      6         /* Statistic var result */

/* Part3.2 High speed loop constants */

/* Switch matrix D set. This is bit mask for register DSWFULLCON. It's used
 * to initialize structure. The bit masks can be OR'ed together. For example
 * - `AD5940_SWD_AIN1|AD5940_SWD_RCAL0` means close AD5940_SWD_AIN1 and
 *   AD5940_SWD_RCAL0 in same time, and open all other D switches.
 * - `AD5940_SWD_AIN2` means close AD5940_SWD_AIN2 and open all other D
 *   switches.
 */

#define AD5940_SWD_OPEN                 (0 << 0)  /* Open all D switch. */
#define AD5940_SWD_RCAL0                (1 << 0)  /* pin RCAL0 */
#define AD5940_SWD_AIN1                 (1 << 1)  /* Pin AIN1 */
#define AD5940_SWD_AIN2                 (1 << 2)  /* Pin AIN2 */
#define AD5940_SWD_AIN3                 (1 << 3)  /* Pin AIN3 */
#define AD5940_SWD_CE0                  (1 << 4)  /* Pin CE0 */
#define AD5940_SWD_CE1                  (1 << 5)  /* CE1 in ADuCM355 */
#define AD5940_SWD_AFE1                 (1 << 5)  /* AFE1 in AD594x */
#define AD5940_SWD_SE0                  (1 << 6)  /* Pin SE0 */
#define AD5940_SWD_SE1                  (1 << 7)  /* SE1 in ADuCM355 */
#define AD5940_SWD_AFE3                 (1 << 7)  /* AFE3 in AD594x */

/* Switch P set. This is bit mask for register PSWFULLCON. It's used to
 * initialize structure. The bit masks can be OR'ed together. For example
 * - `AD5940_SWP_RCAL0|AD5940_SWP_AIN1` means close AD5940_SWP_RCAL0 and
 *   AD5940_SWP_AIN1 in same time, and open all other P switches.
 * - `AD5940_SWP_SE0` means close AD5940_SWP_SE0 and open all other P
 *   switches.
 */

#define AD5940_SWP_OPEN                 0         /* Open all P switches */
#define AD5940_SWP_RCAL0                (1 << 0)  /* Pin RCAL0 */
#define AD5940_SWP_AIN1                 (1 << 1)  /* Pin AIN1 */
#define AD5940_SWP_AIN2                 (1 << 2)  /* Pin AIN2 */
#define AD5940_SWP_AIN3                 (1 << 3)  /* Pin AIN3 */
#define AD5940_SWP_RE0                  (1 << 4)  /* Pin RE0 */
#define AD5940_SWP_RE1                  (1 << 5)  /* RE1 in ADuCM355 */
#define AD5940_SWP_AFE2                 (1 << 5)  /* AFE2 in AD5940 */
#define AD5940_SWP_SE0                  (1 << 6)  /* Pin SE0 */
#define AD5940_SWP_DE0                  (1 << 7)  /* Pin DE0 */
#define AD5940_SWP_SE1                  (1 << 8)  /* SE1 in ADuCM355 */
#define AD5940_SWP_AFE3                 (1 << 8)  /* AFE3 in AD5940 */
#define AD5940_SWP_DE1                  (1 << 9)  /* ADuCM355 Only. */
#define AD5940_SWP_CE0                  (1 << 10) /* Pin CE0 */
#define AD5940_SWP_CE1                  (1 << 11) /* CE1 in ADuCM355 */
#define AD5940_SWP_AFE1                 (1 << 11) /* AFE1 in AD5940 */
#define AD5940_SWP_PL                   (1 << 13) /* Internal PL switch */
#define AD5940_SWP_PL2                  (1 << 14) /* Internal PL2 switch */

/* Switch N set. This is bit mask for register NSWFULLCON. It's used to
 * initialize structure. The bit masks can be OR'ed together. For example
 * - `AD5940_SWN_RCAL0|AD5940_SWN_AIN1` means close AD5940_SWN_RCAL0 and
 *   AD5940_SWN_AIN1 in same time, and open all other N switches.
 * - `AD5940_SWN_SE0` means close AD5940_SWN_SE0 and open all other N
 *   switches.
 * Note that SE0_LOAD is different from PIN SE0. It's the point after 100Ohm
 * load resistor.
 */

#define AD5940_SWN_OPEN                 0         /* Open all N switches */
#define AD5940_SWN_RCAL1                (1 << 9)  /* Pin RCAL1 */
#define AD5940_SWN_AIN0                 (1 << 0)  /* Pin AIN0 */
#define AD5940_SWN_AIN1                 (1 << 1)  /* Pin AIN1 */
#define AD5940_SWN_AIN2                 (1 << 2)  /* Pin AIN2 */
#define AD5940_SWN_AIN3                 (1 << 3)  /* Pin AIN3 */
#define AD5940_SWN_SE0LOAD              (1 << 4)  /* SE0_LOAD after load R */
#define AD5940_SWN_DE0LOAD              (1 << 5)  /* DE0_Load after Rload R */
#define AD5940_SWN_SE1LOAD              (1 << 6)  /* SE1_LOAD in ADuCM355 */
#define AD5940_SWN_AFE3LOAD             (1 << 6)  /* AFE3LOAD in ADuCM355 */
#define AD5940_SWN_DE1LOAD              (1 << 7)  /* ADuCM355 Only*/
#define AD5940_SWN_SE0                  (1 << 8)  /* SE0 means the PIN SE0. */
#define AD5940_SWN_NL                   (1 << 10) /* Internal NL switch */
#define AD5940_SWN_NL2                  (1 << 11) /* Internal NL2 switch */

/* Switch T set. This is bit mask for register TSWFULLCON. It's used to
 * initialize structure. The bit masks can be OR'ed together. For example
 * - AD5940_SWT_RCAL0|AD5940_SWT_AIN1 means close AD5940_SWT_RCAL0 and
 *   AD5940_SWT_AIN1 in same time, and open all other T switches.
 * - AD5940_SWT_SE0LOAD means close AD5940_SWT_SE0LOAD and open all other T
 *   switches.
 * Note that SE0_LOAD is different from PIN SE0. It's the point after 100Ohm
 * load resistor.
 */

#define AD5940_SWT_OPEN                0         /* Open all T switches */
#define AD5940_SWT_RCAL1               (1 << 11) /* Pin RCAL1 */
#define AD5940_SWT_AIN0                (1 << 0)  /* Pin AIN0 */
#define AD5940_SWT_AIN1                (1 << 1)  /* Pin AIN1 */
#define AD5940_SWT_AIN2                (1 << 2)  /* Pin AIN2 */
#define AD5940_SWT_AIN3                (1 << 3)  /* Pin AIN3 */
#define AD5940_SWT_SE0LOAD             (1 << 4)  /* SE0_LOAD after load R */
#define AD5940_SWT_DE0                 (1 << 5)  /* DE0 pin. */
#define AD5940_SWT_SE1LOAD             (1 << 6)  /* SE1_LOAD on ADuCM355*/
#define AD5940_SWT_AFE3LOAD            (1 << 6)  /* AFE3_LOAD on ADuCM355*/
#define AD5940_SWT_DE1                 (1 << 7)  /* ADuCM355 Only */
#define AD5940_SWT_TRTIA               (1 << 8)  /* T9. Connect RTIA to T */
#define AD5940_SWT_DE0LOAD             (1 << 9)  /* DE0Load after Rload */
#define AD5940_SWT_DE1LOAD             (1 << 10) /* DE1Load after Rload */

/* Waveform generator signal type */

#define AD5940_WGTYPE_MMR              0         /* Direct write to DAC */
#define AD5940_WGTYPE_SIN              2         /* Sine wave generator */
#define AD5940_WGTYPE_TRAPZ            3         /* Trapezoid generator */

/* Excitation buffer gain selection */

#define AD5940_EXCITBUFGAIN_2          0         /* Gain is x2 */
#define AD5940_EXCITBUFGAIN_0P25       1         /* gain is x1/4 */

/* HSDAC PGA Gain selection(DACCON.bit0) */

#define AD5940_HSDACGAIN_1             0         /* Gain is x1 */
#define AD5940_HSDACGAIN_0P2           1         /* Gain is x1/5 */

/* HSTIA Amplifier Positive Input selection */

/* HSTIABIAS_Constant
 * When select Vzero0 as bias, close LPDAC switch<xxx>.
 * 0 -  Internal 1.1V common voltage from internal 1.1V reference buffer
 * 1 - From LPDAC0 Vzero0 output
 * 2 - From LPDAC1 Vzero1 output. Only available on ADuCM355.
 */

#define AD5940_HSTIABIAS_1P1       0      /* Internal 1.1V common voltage */
#define AD5940_HSTIABIAS_VZERO0    1      /* From LPDAC0 Vzero0 output */
#define AD5940_HSTIABIAS_VZERO1    2      /* From LPDAC1 Vzero1 output */

/* HSTIA Internal RTIA selection */

#define AD5940_HSTIARTIA_200       0      /* Internal RTIA resistor 200 */
#define AD5940_HSTIARTIA_1K        1      /* Internal RTIA resistor 1K  */
#define AD5940_HSTIARTIA_5K        2      /* Internal RTIA resistor 5K  */
#define AD5940_HSTIARTIA_10K       3      /* Internal RTIA resistor 10K */
#define AD5940_HSTIARTIA_20K       4      /* Internal RTIA resistor 20K */
#define AD5940_HSTIARTIA_40K       5      /* Internal RTIA resistor 40K */
#define AD5940_HSTIARTIA_80K       6      /* Internal RTIA resistor 80K */
#define AD5940_HSTIARTIA_160K      7      /* Internal RTIA resistor 160K */
#define AD5940_HSTIARTIA_OPEN      8      /* Open internal resistor */

/* AD5940_HSTIADERTIA_Const */

#define AD5940_HSTIADERTIA_50      0      /* 50Ohm  | Settings depends on */
#define AD5940_HSTIADERTIA_100     1      /* 100Ohm | RLOAD resistor. */
#define AD5940_HSTIADERTIA_200     2      /* 200Ohm | */
#define AD5940_HSTIADERTIA_1K      3      /* set bit[7:3] to 0x0b(11) */
#define AD5940_HSTIADERTIA_5K      4      /* set bit[7:3] to 0x0c(12) */
#define AD5940_HSTIADERTIA_10K     5      /* set bit[7:3] to 0x0d(13) */
#define AD5940_HSTIADERTIA_20K     6      /* set bit[7:3] to 0x0e(14) */
#define AD5940_HSTIADERTIA_40K     7      /* set bit[7:3] to 0x0f(15) */
#define AD5940_HSTIADERTIA_80K     8      /* set bit[7:3] to 0x10(16) */
#define AD5940_HSTIADERTIA_160K    9      /* set bit[7:3] to 0x11(17) */

/* Offset between the value and bit[7:3] value for above 7 items */

#define AD5940_HSTIADERTIA_OFST    8      /* Offset for above 7 items */

/* short HSTIA output to DE0 pin. set bit[7:3] to 0x12(18) */

#define AD5940_HSTIADERTIA_TODE    10     /* set bit[7:3] to 0x12(18) */

/* Default state is set to OPEN RTIA by setting bit[7:3] to 0x1f */

#define AD5940_HSTIADERTIA_OPEN    11     /* set bit[7:3] to 0x1f */

/* HSTIA DE0 Terminal internal RLOAD selection */

#define AD5940_HSTIADERLOAD_0R     0      /* set bit[2:0] to 0x00 */
#define AD5940_HSTIADERLOAD_10R    1      /* set bit[2:0] to 0x01 */
#define AD5940_HSTIADERLOAD_30R    2      /* set bit[2:0] to 0x02 */
#define AD5940_HSTIADERLOAD_50R    3      /* set bit[2:0] to 0x03 */
#define AD5940_HSTIADERLOAD_100R   4      /* set bit[2:0] to 0x04 */

/* RLOAD open means open switch between HSTIA negative input and Rload
 * resistor(<S1>).Default state is OPEN RLOAD by setting HSTIARES03CON[2:0]
 * to 0x5, 0x6 or 0x7
 */

#define AD5940_HSTIADERLOAD_OPEN   5      /* RLOAD open */

/* AD5940_HSTIADERLOAD_Const */

#define AD5940_HSTIADERLOAD_LP     0      /* HSTIA in LP mode */
#define AD5940_HSTIADERLOAD_HP     1      /* HSTIA in HP mode */

/* AD5940 HSTIA DE pin */

#define AD5940_HSTIADE0PIN         0      /* DE0 pin */
#define AD5940_HSTIADE1PIN         0      /* DE1 pin */

/* Part3.3 Low_Power_Loop.
 * Low power includes low power DAC and two low power amplifiers(PA and TIA).
 */

/* Select which LPDAC is accessing. This parameter must be configured
 * correctly.
 */

#define AD5940_LPDAC0              0      /* AD5940_LPDAC0 */
#define AD5940_LPDAC1              1      /* AD5940_LPDAC1, ADuCM355 Only */

/* LPDAC data source selection. Either from MMR or from waveform generator. */

#define AD5940_LPDACSRC_MMR        0      /* From REG_AFE_LPDACDAT0DATA0 */
#define AD5940_LPDACSRC_WG         1      /* From waveform generator */

/* LPDAC switch settings
 * 0x10: Switch between LPDAC Vbias output and LPPA(low power PA(Potential
 *       Amplifier))
 * 0x08: Switch between LPDAC Vbias output and Vbias pin
 * 0x04: Switch between LPDAC Vzero output and LPTIA positive input
 * 0x02: Switch between LPDAC Vzero output and Vzero pin
 * 0x01: Switch between LPDAC Vzero output and HSTIA positive input MUX
 */

#define AD5940_LPDACSW_VBIAS2LPPA  0x10   /* LPDAC Vbias and LPPA */
#define AD5940_LPDACSW_VBIAS2PIN   0x08   /* LPDAC Vbias and Vbias pin */
#define AD5940_LPDACSW_VZERO2LPTIA 0x04   /* LPDAC Vzero and LPTIA IN+ */
#define AD5940_LPDACSW_VZERO2PIN   0x02   /* LPDAC Vzero and Vzero pin */
#define AD5940_LPDACSW_VZERO2HSTIA 0x01   /* LPDAC Vzero and HSTIA IN+ MUX */

/* Vzero MUX selection */

#define AD5940_LPDACVZERO_6BIT     0      /* Vzero to 6bit LPDAC output */
#define AD5940_LPDACVZERO_12BIT    1      /* Vzero to 12bit LPDAC output */

/* Vbias MUX selection */

#define AD5940_LPDACVBIAS_6BIT     1      /* Vbias to 6bit LPDAC output */
#define AD5940_LPDACVBIAS_12BIT    0      /* Vbias to 12bit LPDAC output */

/* LPDAC reference selection */

#define AD5940_LPDACREF_2P5        0      /* Internal 2.5V reference */
#define AD5940_LPDACREF_AVDD       1      /* Use AVDD as reference */

/* LPTIA selecion */

#define AD5940_LPTIA0              0      /* AD5940_LPTIA0 */
#define AD5940_LPTIA1              1      /* AD5940_LPTIA1, ADuCM355 Only */

/* LPTIA LPF Resistor selection */

#define AD5940_LPTIARF_OPEN        0      /* Disconnect Rf resistor */
#define AD5940_LPTIARF_SHORT       1      /* Bypass Rf resistor */
#define AD5940_LPTIARF_20K         2      /* 20kOhm Rf */
#define AD5940_LPTIARF_100K        3      /* Rf resistor 100kOhm */
#define AD5940_LPTIARF_200K        4      /* Rf resistor 200kOhm */
#define AD5940_LPTIARF_400K        5      /* Rf resistor 400kOhm */
#define AD5940_LPTIARF_600K        6      /* Rf resistor 600kOhm */
#define AD5940_LPTIARF_1M          7      /* Rf resistor 1MOhm */

/* LPTIA Rload Selection */

#define AD5940_LPTIARLOAD_SHORT    0      /* 0Ohm Rload */
#define AD5940_LPTIARLOAD_10R      1      /* 10Ohm Rload */
#define AD5940_LPTIARLOAD_30R      2      /* Rload resistor 30Ohm */
#define AD5940_LPTIARLOAD_50R      3      /* Rload resistor 50Ohm */
#define AD5940_LPTIARLOAD_100R     4      /* Rload resistor 100Ohm */
#define AD5940_LPTIARLOAD_1K6      5      /* Only available when RTIA >= 2K */
#define AD5940_LPTIARLOAD_3K1      6      /* Only available when RTIA >= 4K */
#define AD5940_LPTIARLOAD_3K6      7      /* Only available when RTIA >= 4K */

/* LPTIA RTIA Selection. Note that the real RTIA resistor value dependents on
 * Rload settings.
 */

#define AD5940_LPTIARTIA_OPEN      0      /* Disconnect LPTIA Internal RTIA */
#define AD5940_LPTIARTIA_200R      1      /* 200Ohm Internal RTIA */
#define AD5940_LPTIARTIA_1K        2      /* 1KOHM */
#define AD5940_LPTIARTIA_2K        3      /* 2KOHM */
#define AD5940_LPTIARTIA_3K        4      /* 3KOHM */
#define AD5940_LPTIARTIA_4K        5      /* 4KOHM */
#define AD5940_LPTIARTIA_6K        6      /* 6KOHM */
#define AD5940_LPTIARTIA_8K        7      /* 8KOHM */
#define AD5940_LPTIARTIA_10K       8      /* 10KOHM */
#define AD5940_LPTIARTIA_12K       9      /* 12KOHM */
#define AD5940_LPTIARTIA_16K       10     /* 16KOHM */
#define AD5940_LPTIARTIA_20K       11     /* 20KOHM */
#define AD5940_LPTIARTIA_24K       12     /* 24KOHM */
#define AD5940_LPTIARTIA_30K       13     /* 30KOHM */
#define AD5940_LPTIARTIA_32K       14     /* 32KOHM */
#define AD5940_LPTIARTIA_40K       15     /* 40KOHM */
#define AD5940_LPTIARTIA_48K       16     /* 48KOHM */
#define AD5940_LPTIARTIA_64K       17     /* 64KOHM */
#define AD5940_LPTIARTIA_85K       18     /* 85KOHM */
#define AD5940_LPTIARTIA_96K       19     /* 96KOHM */
#define AD5940_LPTIARTIA_100K      20     /* 100KOHM */
#define AD5940_LPTIARTIA_120K      21     /* 120KOHM */
#define AD5940_LPTIARTIA_128K      22     /* 128KOHM */
#define AD5940_LPTIARTIA_160K      23     /* 160KOHM */
#define AD5940_LPTIARTIA_196K      24     /* 196KOHM */
#define AD5940_LPTIARTIA_256K      25     /* 256KOHM */
#define AD5940_LPTIARTIA_512K      26     /* 512KOHM */

/* LPAMP selecion. On AD594x, only AD5940_LPAMP0 is available. This parameter
 * must be configured correctly.
 * 0 - AMP include both LPTIA and Potentio-stat amplifiers
 * 1 - ADuCM355 Only
 */

#define AD5940_LPAMP0              0   /* AD5940 */
#define AD5940_LPAMP1              1   /* ADuCM355 Only */

/* Low power amplifier(PA and TIA) power mode selection. */

#define AD5940_LPAMPPWR_NORM       0   /* Normal Power mode */
#define AD5940_LPAMPPWR_BOOST1     1   /* Boost power to level 1 */
#define AD5940_LPAMPPWR_BOOST2     2   /* Boost power to level 2 */
#define AD5940_LPAMPPWR_BOOST3     3   /* Boost power to level 3 */
#define AD5940_LPAMPPWR_HALF       4   /* Put PA and TIA in half power mode */

/* LPTIA switch control. Use this to set LpTiaSW field of LPAmpCfg_Type */

#define AD5940_LPTIASW(n)          (1 << n)

/* Part3.4 DSP block
 * DSP block include signal chain from raw ADC data to various filters, DFT
 * engine and Statistic Functions etc.
 */

/* ADC PGA Selection. Only gain 1.5 is factory calibrated. */

#define AD5940_ADCPGA_1            0      /* ADC PGA Gain of 1 */
#define AD5940_ADCPGA_1P5          1      /* ADC PGA Gain of 1.5 */
#define AD5940_ADCPGA_2            2      /* ADC PGA Gain of 2 */
#define AD5940_ADCPGA_4            3      /* ADC PGA Gain of 4 */
#define AD5940_ADCPGA_9            4      /* ADC PGA Gain of 9 */

/* ADC Channel P Configuration */

#define AD5940_ADCMUXP_FLOAT       0x0    /* float */
#define AD5940_ADCMUXP_HSTIA_P     0x1    /* output of HSTIA */
#define AD5940_ADCMUXP_AIN0        0x4    /* pin AIN0 */
#define AD5940_ADCMUXP_AIN1        0x5    /* pin AIN1 */
#define AD5940_ADCMUXP_AIN2        0x6    /* pin AIN2 */
#define AD5940_ADCMUXP_AIN3        0x7    /* pin AIN3 */
#define AD5940_ADCMUXP_AVDD_2      0x8    /* AVDD/2 */
#define AD5940_ADCMUXP_DVDD_2      0x9    /* DVDD/2 */
#define AD5940_ADCMUXP_AVDDREG     0xa    /* AVDD internal regulator output */
#define AD5940_ADCMUXP_TEMPP       0xb    /* Internal temperature output+ */
#define AD5940_ADCMUXP_VSET1P1     0xc    /* Internal 1.1V bias voltage */
#define AD5940_ADCMUXP_VDE0        0xd    /* Voltage of DE0 pin */
#define AD5940_ADCMUXP_VSE0        0xe    /* Voltage of SE0 pin */
#define AD5940_ADCMUXP_VAFE3       0xf    /* Voltage of AFE3 pin on AD5940. */
#define AD5940_ADCMUXP_VREF2P5     0x10   /* 1.25V. Internal 2.5V ref/2. */
#define AD5940_ADCMUXP_VREF1P8DAC  0x12   /* HSDAC 1.8V internal reference. */
#define AD5940_ADCMUXP_TEMPN       0x13   /* Internal temperature output- */
#define AD5940_ADCMUXP_AIN4        0x14   /* Voltage of AIN4/LPF0 pin */
#define AD5940_ADCMUXP_AIN5        0x15   /* Voltage of AIN5 pin */
#define AD5940_ADCMUXP_AIN6        0x16   /* Voltage of AIN6 pin */
#define AD5940_ADCMUXP_VZERO0      0x17   /* Voltage of Vzero0 pin */
#define AD5940_ADCMUXP_VBIAS0      0x18   /* Voltage of Vbias0 pin */
#define AD5940_ADCMUXP_VCE0        0x19   /* Pin CE0 */
#define AD5940_ADCMUXP_VRE0        0x1a   /* Pin RE0 */
#define AD5940_ADCMUXP_VAFE4       0x1b   /* Voltage of AFE4 pin on AD5940. */
#define AD5940_ADCMUXP_VBIAS1      0x1c   /* Voltage of Vbias1 pin */
#define AD5940_ADCMUXP_VAFE1       0x1d   /* Voltage of AFE1 pin on AD5940. */
#define AD5940_ADCMUXP_VAFE2       0x1e   /* Voltage of AFE2 pin on AD5940. */
#define AD5940_ADCMUXP_VCE0_2      0x1f   /* VCE0 divide by 2 */
#define AD5940_ADCMUXP_VCE1_2      0x20   /* VCE1 divide by 2 */
#define AD5940_ADCMUXP_LPTIA0_P    0x21   /* Output of LPTIA0 */
#define AD5940_ADCMUXP_LPTIA1_P    0x22   /* Output of LPTIA1 */
#define AD5940_ADCMUXP_AGND        0x23   /* Internal AGND node */
#define AD5940_ADCMUXP_P_NODE      0x24   /* Voltage of excitation buffer P */
#define AD5940_ADCMUXP_IOVDD_2     0x27   /* IOVDD/2 */

/* ADC Channel N Configuration */

#define AD5940_ADCMUXN_FLOAT       0x0    /* float */
#define AD5940_ADCMUXN_HSTIA_N     0x1    /* HSTIA negative input node. */
#define AD5940_ADCMUXN_LPTIA0_N    0x2    /* LPTIA0 negative input node. */
#define AD5940_ADCMUXN_LPTIA1_N    0x3    /* LPTIA1 negative input node. */
#define AD5940_ADCMUXN_AIN0        0x4    /* Pin AIN0 */
#define AD5940_ADCMUXN_AIN1        0x5    /* Pin AIN1 */
#define AD5940_ADCMUXN_AIN2        0x6    /* Pin AIN2 */
#define AD5940_ADCMUXN_AIN3        0x7    /* Pin AIN3 */
#define AD5940_ADCMUXN_VSET1P1     0x8    /* Internal 1.11V reference */
#define AD5940_ADCMUXN_VREF1P1     0x8    /* i.e. AD5940_ADCMUXN_VSET1P1 */
#define AD5940_ADCMUXN_TEMPN       0xb    /* Temperature sensor output. */
#define AD5940_ADCMUXN_AIN4        0xc    /* AIN4 */
#define AD5940_ADCMUXN_AIN5        0xd    /* AIN5 */
#define AD5940_ADCMUXN_AIN6        0xe    /* AIN6 */
#define AD5940_ADCMUXN_VZERO0      0x10   /* pin Vzero0 */
#define AD5940_ADCMUXN_VBIAS0      0x11   /* pin Vbias0 */
#define AD5940_ADCMUXN_VZERO1      0x12   /* pin Vzero1 */
#define AD5940_ADCMUXN_AFE4        0x12   /* Pin AFE4 on AD5940. */
#define AD5940_ADCMUXN_VBIAS1      0x13   /* pin Vbias1 */
#define AD5940_ADCMUXN_N_NODE      0x14   /* Voltage of excitation buffer N */

/* ADC Current Sample Rate. If ADC clock is 32MHz, set it to
 * AD5940_ADCRATE_1P6MHZ. Otherwise, set it to AD5940_ADCRATE_800KHZ.
 */

#define AD5940_ADCRATE_800KHZ      1     /* Input CLK = 16MHz, SR is 800kHz */
#define AD5940_ADCRATE_1P6MHZ      0     /* Input CLK = 32MHz, SR is 1.6MHz */

/* ADC SINC3 Filter OSR. 2, 4 is recommended value. 5 is not recommended. */

#define AD5940_ADCSINC3OSR_2       2     /* ADC SINC3 OSR 2 */
#define AD5940_ADCSINC3OSR_4       1     /* ADC SINC3 OSR 4 */
#define AD5940_ADCSINC3OSR_5       0     /* ADC SINC3 OSR 5 */

/* ADC SINC2 Filter OSR. */

#define AD5940_ADCSINC2OSR_22      0     /* ADC SINC2 OSR 22  */
#define AD5940_ADCSINC2OSR_44      1     /* ADC SINC2 OSR 44  */
#define AD5940_ADCSINC2OSR_89      2     /* ADC SINC2 OSR 89  */
#define AD5940_ADCSINC2OSR_178     3     /* ADC SINC2 OSR 178 */
#define AD5940_ADCSINC2OSR_267     4     /* ADC SINC2 OSR 267 */
#define AD5940_ADCSINC2OSR_533     5     /* ADC SINC2 OSR 533 */
#define AD5940_ADCSINC2OSR_640     6     /* ADC SINC2 OSR 640 */
#define AD5940_ADCSINC2OSR_667     7     /* ADC SINC2 OSR 667 */
#define AD5940_ADCSINC2OSR_800     8     /* ADC SINC2 OSR 800 */
#define AD5940_ADCSINC2OSR_889     9     /* ADC SINC2 OSR 889 */
#define AD5940_ADCSINC2OSR_1067    10    /* ADC SINC2 OSR 1067 */
#define AD5940_ADCSINC2OSR_1333    11    /* ADC SINC2 OSR 1333 */

/* ADC Average filter for DFT. The average block locates after SINC3 filter.
 * The output of average filter is directly feed into DFT block. Once average
 * filter is enabled, DFT source is automatically changed to averaged data.
 */

#define AD5940_ADCAVGNUM_2         0      /* Take 2 input to do average. */
#define AD5940_ADCAVGNUM_4         1      /* Take 4 input to do average. */
#define AD5940_ADCAVGNUM_8         2      /* Take 8 input to do average. */
#define AD5940_ADCAVGNUM_16        3      /* Take 16 input to do average. */

/* DFT source selection. When average function is enabled, DFT source
 * automatically switch to average output.
 */

/* SINC2+Notch filter block output. Bypass Notch to use SINC2 data */

#define AD5940_DFTSRC_SINC2NOTCH   0   /* SINC2+Notch filter block output. */
#define AD5940_DFTSRC_SINC3        1   /* SINC3 filter */
#define AD5940_DFTSRC_ADCRAW       2   /* Raw ADC data */
#define AD5940_DFTSRC_AVG          3   /* Average output of SINC3. */

/* DFT number selection. */

#define AD5940_DFTNUM_4            0      /* 4     Points */
#define AD5940_DFTNUM_8            1      /* 8     Points */
#define AD5940_DFTNUM_16           2      /* 16    Points */
#define AD5940_DFTNUM_32           3      /* 32    Points */
#define AD5940_DFTNUM_64           4      /* 64    Points */
#define AD5940_DFTNUM_128          5      /* 128   Points */
#define AD5940_DFTNUM_256          6      /* 256   Points */
#define AD5940_DFTNUM_512          7      /* 512   Points */
#define AD5940_DFTNUM_1024         8      /* 1024  Points */
#define AD5940_DFTNUM_2048         9      /* 2048  Points */
#define AD5940_DFTNUM_4096         10     /* 4096  Points */
#define AD5940_DFTNUM_8192         11     /* 8192  Points */
#define AD5940_DFTNUM_16384        12     /* 16384 Points */

/* The statistic module sample size. It decides how much data is used to do
 * calculation.
 */

#define AD5940_STATSAMPLE_128      0      /* Sample size 128 */
#define AD5940_STATSAMPLE_64       1      /* Sample size 64 */
#define AD5940_STATSAMPLE_32       2      /* Sample size 32 */
#define AD5940_STATSAMPLE_16       3      /* Sample size 16 */
#define AD5940_STATSAMPLE_8        4      /* Sample size 8 */

/*  The standard deviation configure */

#define AD5940_STATDEV_1           1      /* For check ADC result outlier */
#define AD5940_STATDEV_4           4      /* For check ADC result outlier */
#define AD5940_STATDEV_9           9      /* For check ADC result outlier */
#define AD5940_STATDEV_16          16     /* For check ADC result outlier */
#define AD5940_STATDEV_25          25     /* For check ADC result outlier */

/* Part3.5 Sequencer and FIFO */

/* AD5940_SEQID_Const */

#define AD5940_SEQID_0             0      /* Sequence0 */
#define AD5940_SEQID_1             1      /* Sequence1 */
#define AD5940_SEQID_2             2      /* Sequence2 */
#define AD5940_SEQID_3             3      /* Sequence3 */

/* Sequencer memory size. SRAM is shared between FIFO and Sequencer. The
 * total available SRAM is 6kB. It's shared by FIFO and sequencer.
 */

/* The selfbuild in 32Byte for sequencer. All 6kB SRAM is for data FIFO */

#define AD5940_SEQMEMSIZE_32B      0

/* Sequencer use 2kB. The reset 4kB can be used for data FIFO */

#define AD5940_SEQMEMSIZE_2KB      1

/* 4kB for Sequencer. 2kB for data FIFO */

#define AD5940_SEQMEMSIZE_4KB      2

/* All 6kB for Sequencer. Build in 32Bytes memory can be used for data FIFO */

#define AD5940_SEQMEMSIZE_6KB      3

/* SEQPINTRIGMODE_Const.
 * Mode of GPIO detecting used for triggering sequence
 */

#define AD5940_SEQTRIGMDE_RISING   0      /* Rising edge */
#define AD5940_SEQTRIGMDE_FALLING  1      /* Falling edge */
#define AD5940_SEQTRIGMDE_BOTHEDGE 2      /* Rising or falling */
#define AD5940_SEQTRIGMDE_HIGHL    3      /* High level */
#define AD5940_SEQTRIGMDE_LOWL     4      /* Low level */

/* Sequencer command - wait: wait some clocks-code. Command code is 'b00 @bit
 * [31:30]. Maximum wait time is 0x3fff_ffff (system clock).
 */

#define AD5940_SEQ_WAIT(ClkNum)    (0x00000000 | \
                                   ((uint32_t)(ClkNum) & 0x3fffffff))

/* Sequencer command - time-out: set time-out count down value. Command code
 * is 'b01 @bit[31:30]. Maximum time-out timer value is 0x3fffffff
 */

#define AD5940_SEQ_TOUT(ClkNum)    (0x40000000 | \
                                   ((uint32_t)(ClkNum) & 0x3fffffff))

/* Sequencer command - write register. Command code is 'b10 or 'b11
 * @bit[31:30]. Address range is 0x2000 to 0x21FF. Data is limited to 24bits.
 */

#define AD5940_SEQ_WR(addr,data)   (0x80000000 | \
                                   (((((uint32_t)(addr)) >> 2)&0x7f) << 24) \
                                   | (((uint32_t)(data)) & 0xffffff))

/* Some commands used frequently
 * AD5940_SEQ_NOP()  is just a simple wait command that wait one system clock
 * AD5940_SEQ_HALT() can halt sequencer. Used for debug
 * AD5940_SEQ_STOP() disables sequencer. This will generate an
 *                   end-of-sequence interrupt.
 * AD5940_SEQ_SLP()  triggers sleep. If sleep is allowed, AFE will go to
 *                   sleep/hibernate mode
 * AD5940_SEQ_INTx() generates custom interrupt x
 */

#define AD5940_SEQ_NOP()           AD5940_SEQ_WAIT(0)
#define AD5940_SEQ_HALT()          AD5940_SEQ_WR(AD5940_REG_AFE_SEQCON,0x12)
#define AD5940_SEQ_STOP()          AD5940_SEQ_WR(AD5940_REG_AFE_SEQCON,0x00)
#define AD5940_SEQ_SLP()           AD5940_SEQ_WR(AD5940_REG_AFE_SEQTRGSLP, 1)
#define AD5940_SEQ_INT0()          AD5940_SEQ_WR( \
                                     AD5940_REG_AFE_AFEGENINTSTA, (1 << 0))
#define AD5940_SEQ_INT1()          AD5940_SEQ_WR( \
                                     AD5940_REG_AFE_AFEGENINTSTA, (1 << 1))
#define AD5940_SEQ_INT2()          AD5940_SEQ_WR( \
                                     AD5940_REG_AFE_AFEGENINTSTA, (1 << 2))
#define AD5940_SEQ_INT3()          AD5940_SEQ_WR( \
                                     AD5940_REG_AFE_AFEGENINTSTA, (1 << 3))

/* Calculate how many commands are in sepecified array. */

#define AD5940_SEQ_LEN(n)          (sizeof(n) / 4)

/* FIFO mode
 * 2 - Standard FIFO mode. If FIFO is full, reject all comming data and put
 *     FIFO to fault state, report interrupt if enabled.
 * 3 - Stream mode. If FIFO is full, discard older data. Report FIFO full
 *     interrupt if enabled.
 */

#define AD5940_FIFOMODE_FIFO        2     /* Standard FIFO mode */
#define AD5940_FIFOMODE_STREAM      3     /* Stream mode */

/* FIFO source */

#define AD5940_FIFOSRC_SINC3        0     /* SINC3 data */
#define AD5940_FIFOSRC_DFT          2     /* DFT real and imaginary part */
#define AD5940_FIFOSRC_SINC2NOTCH   3     /* SINC2+NOTCH block */
#define AD5940_FIFOSRC_VAR          4     /* Statistic variarance output */
#define AD5940_FIFOSRC_MEAN         5     /* Statistic mean output */

/* FIFO helper.
 * Method to identify FIFO channel ID:
 * [31:25][24:23][22:16][15:0]
 * [ ECC ][SEQID][CH_ID][DATA]
 *
 * CH_ID: [22:16] 7bit in total:
 *        xxxxx_xx
 *        11111_xx    : DFT results.
 *        11110_xx    : Mean of statistic block.
 *        11101_xx    : Variance of statistic block.
 *        1xxxx_xx    : Notch filter result, where xxx_xx is the ADC MUX P
 *                      settings(6bits of reg ADCCON[5:0]).
 *        0xxxx_xx    : SINC3 filter result, where xxx_xx is the ADC MUX P
 *                      settings(6bits of reg ADCCON[5:0]).
 * AD5940_FIFO_SEQID  - Return seqid of this FIFO result.
 * AD5940_FIFO_ECC    - Return ECC of this FIFO result.
 * AD5940_FIFO_CHANID - Return Channel ID.
 * AD5940_FIFO_MUXP   - Return the ADC MUXP selection.
 */

#define AD5940_FIFO_SEQID(data)    ((((uint32_t)(data)) >> 23) & 0x03)
#define AD5940_FIFO_ECC(data)      ((((uint32_t)(data)) >> 25) & 0x7f)
#define AD5940_FIFO_CHANID(data)   ((((uint32_t)(data)) >> 16) & 0x7f)
#define AD5940_FIFO_MUXP(data)     ((((uint32_t)(data)) >> 16) & 0x3f)

/* Set FIFO size FIFOSIZE_Const */

#define AD5940_FIFOSIZE_32B        0      /* DATA FIFO 32B. Sequencer 6KB */
#define AD5940_FIFOSIZE_2KB        1      /* DATA FIFO 2KB. Sequencer 4kB */
#define AD5940_FIFOSIZE_4KB        2      /* Data FIFO 4KB. Sequencer 2KB */
#define AD5940_FIFOSIZE_6KB        3      /* Data FIFO 6KB. Sequencer 32B */

/* Wake up timer WUPTENDSEQ_Const */

#define AD5940_WUPTENDSEQ_A        0      /* End at slot A */
#define AD5940_WUPTENDSEQ_B        1      /* End at slot B */
#define AD5940_WUPTENDSEQ_C        2      /* End at slot C */
#define AD5940_WUPTENDSEQ_D        3      /* End at slot D */
#define AD5940_WUPTENDSEQ_E        4      /* End at slot E */
#define AD5940_WUPTENDSEQ_F        5      /* End at slot F */
#define AD5940_WUPTENDSEQ_G        6      /* End at slot G */
#define AD5940_WUPTENDSEQ_H        7      /* End at slot H */

/* Part3.6 MISC part including clock, GPIO, configuration. */

/* DATATYPE_Const */

#define AD5940_DATATYPE_ADCRAW     0      /* ADC raw data */
#define AD5940_DATATYPE_SINC3      1      /* SINC3 data */
#define AD5940_DATATYPE_SINC2      2      /* SINC2 Data */
#define AD5940_DATATYPE_DFT        3      /* DFT */
#define AD5940_DATATYPE_NOTCH      4      /* Notch filter output */

/*  SLPKEY_Const */

#define AD5940_SLPKEY_LOCK         0             /* Incorrect value */
#define AD5940_SLPKEY_UNLOCK       0xa47e5       /* The correct key */

/* Power mode key */
#define AD5940_PWRKEY_UNLOCK1      0x4859        /* Correct key 1 */
#define AD5940_PWRKEY_UNLOCK2      0xf27b        /* Correct key 1 */

/* Set HPOSC output clock frequency, 16MHz or 32MHz. */

#define AD5940_HPOSCOUT_32MHZ      0            /* HFOSC output 32MHz clock */
#define AD5940_HPOSCOUT_16MHZ      1            /* 16MHz Clock */

/* The pin masks for register GP0OEN, GP0PE, GP0IEN,..., GP0TGL */

#define AD5940_AGPIO_PIN0          0x01          /* AFE GPIO0 */
#define AD5940_AGPIO_PIN1          0x02          /* AFE GPIO1 */
#define AD5940_AGPIO_PIN2          0x04          /* AFE GPIO2 */
#define AD5940_AGPIO_PIN3          0x08          /* AFE GPIO3 */
#define AD5940_AGPIO_PIN4          0x10          /* AFE GPIO4 */
#define AD5940_AGPIO_PIN5          0x20          /* AFE GPIO5 */
#define AD5940_AGPIO_PIN6          0x40          /* AFE GPIO6 */
#define AD5940_AGPIO_PIN7          0x80          /* AFE GPIO7 */

/* GP0FUNC_Const */

#define AD5940_GP0_INT0            0             /* Interrupt 0 output */
#define AD5940_GP0_TRIG            1             /* Sequence0 trigger */
#define AD5940_GP0_SYNC            2             /* Sequencer output level */
#define AD5940_GP0_GPIO            3             /* Normal GPIO function */

/* GP1FUNC_Const */

#define AD5940_GP1_GPIO            (0<<2)        /* Normal GPIO function */
#define AD5940_GP1_TRIG            (1<<2)        /* Sequence1 trigger */
#define AD5940_GP1_SYNC            (2<<2)        /* Sequencer output level */
#define AD5940_GP1_SLEEP           (3<<2)        /* Internal Sleep Signal */

/* GP2FUNC_Const */

#define AD5940_GP2_PORB            (0<<4)        /* Internal POR signal */
#define AD5940_GP2_TRIG            (1<<4)        /* Sequence1 trigger */
#define AD5940_GP2_SYNC            (2<<4)        /* Sequencer output level */
#define AD5940_GP2_EXTCLK          (3<<4)        /* External Clock input */

/* GP3FUNC_Const */

#define AD5940_GP3_GPIO            (0<<6)        /* Normal GPIO function */
#define AD5940_GP3_TRIG            (1<<6)        /* Sequence3 trigger */
#define AD5940_GP3_SYNC            (2<<6)        /* Sequencer output level */
#define AD5940_GP3_INT0            (3<<6)        /* Interrupt 0 output */

/* GP4FUNC_Const */

#define AD5940_GP4_GPIO            (0<<8)        /* Normal GPIO function */
#define AD5940_GP4_TRIG            (1<<8)        /* Sequence0 trigger */
#define AD5940_GP4_SYNC            (2<<8)        /* Sequencer output level */
#define AD5940_GP4_INT1            (3<<8)        /* Interrupt 1 output */

/* GP5FUNC_Const */

#define AD5940_GP5_GPIO            (0<<10)       /* Internal POR signal */
#define AD5940_GP5_TRIG            (1<<10)       /* Sequence1 trigger */
#define AD5940_GP5_SYNC            (2<<10)       /* Sequencer output level */
#define AD5940_GP5_EXTCLK          (3<<10)       /* External clock input */

/* GP6FUNC_Const */

#define AD5940_GP6_GPIO            (0<<12)       /* Normal GPIO function */
#define AD5940_GP6_TRIG            (1<<12)       /* Sequence2 trigger */
#define AD5940_GP6_SYNC            (2<<12)       /* Sequencer output level */
#define AD5940_GP6_INT0            (3<<12)       /* Interrupt 0 output */

/* GP7FUNC_Const */

#define AD5940_GP7_GPIO            (0<<14)       /* Normal GPIO function */
#define AD5940_GP7_TRIG            (1<<14)       /* Sequence2 trigger */
#define AD5940_GP7_SYNC            (2<<14)       /* Sequencer output level */
#define AD5940_GP7_INT             (3<<14)       /* Interrupt 1 output */

/* Low power mode clock */

#define AD5940_LPMODECLK_HFOSC     0             /* HFOSC as SYS clock */
#define AD5940_LPMODECLK_LFOSC     1             /* LFOSC as SYS clock */

/* SYSCLKSRC_Const
 * Select system clock source. The clock must be available. If unavailable
 * clock is selected, we can reset AD5940. The system clock should be limited
 * to 32MHz. If external clock or XTAL is faster than 16MHz, we use system
 * clock divider to ensure it's always in range of 16MHz. Maximum SPI clock
 * has relation with system clock. Limit the SPI clock to ensure SPI clock is
 * slower than system clock.
 * 0 - Internal HFOSC. CLock is 16MHz or 32MHz configurable. Set clock
 *     divider to ensure system clock is always 16MHz.
 * 1 - External crystal. It can be 16MHz or 32MHz.Set clock divider to ensure
 *     system clock is always 16MHz.
 * 2 - Internal 32kHz clock. Note the SPI clock also sourced with 32kHz so
 *     the register read/write frequency is lower down.
 * 3 - External clock from GPIO.
 */

#define AD5940_SYSCLKSRC_HFOSC     0             /* Internal HFOSC. */
#define AD5940_SYSCLKSRC_XTAL      1             /* External crystal. */
#define AD5940_SYSCLKSRC_LFOSC     2             /* Internal 32kHz clock. */
#define AD5940_SYSCLKSRC_EXT       3             /* External clock from pin */

/* ADCCLKSRC_Const
 * Select ADC clock source. The maximum clock is 32MHz. The ADC raw data
 * update rate is equal to ADCClock/20. When ADC clock is 32MHz, sample rate
 * is 1.6MSPS. The SINC3 filter clock are sourced from ADC clock and should
 * be limited to 16MHz. When ADC clock is set to 32MHz, clear bit
 * ADCFILTERCON.bit0 to enable the SINC3 clock divider.
 */

#define AD5940_ADCCLKSRC_HFOSC     0             /* Internal HFOSC. */
#define AD5940_ADCCLKSRC_XTAL      1             /* External crystal. */
#define AD5940_ADCCLKSRC_EXT       3             /* External clock from pin */

/* ADCCLKDIV_Const
 * The divider for ADC clock. ADC clock = ClockSrc/Divider.
 */

#define AD5940_ADCCLKDIV_1         1             /* ADCClk = ClkSrc/1 */
#define AD5940_ADCCLKDIV_2         2             /* ADCClk = ClkSrc/2 */

/* SYSCLKDV_Const
 * The divider for system clock. System clock = ClockSrc/Divider.
 */

#define AD5940_SYSCLKDIV_1         1             /* SysClk = ClkSrc/1 */
#define AD5940_SYSCLKDIV_2         2             /* SysClk = ClkSrc/2 */

/* Calibration Type */

#define AD5940_PGACALTYPE_OFFSET   0             /* Calibrate offset */
#define AD5940_PGACALTYPE_GAIN     1             /* Calibrate gain */
#define AD5940_PGACALTYPE_OFSTGAIN 2             /* Calibrate offset & gain */

/* Special register values */

#define AD5940_SWRST               0xa158        /* Trigger a soft-reset */
#define AD5940_KEY_OSCCON          0xcb14        /* Key of register OSCCON. */
#define AD5940_KEY_CALDATLOCK      0xde87a5af    /* Calibration key. */
#define AD5940_KEY_LPMODEKEY       0xc59d6       /* LP mode key */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* AD5940 references configuration */

struct ad5940_aferefcfg_s
{
  /* ADC/DAC/TIA reference and buffer */

  bool hpbandgapen;     /* Enable high power bandgap */
  bool hp1v8buffen;     /* High power 1.8V reference buffer enable */
  bool hp1v1buffen;     /* High power 1.1V reference buffer enable */
  bool lp1v8buffen;     /* Low power 1.8V reference buffer enable */
  bool lp1v1buffen;     /* Low power 1.1V reference buffer enable */

  /* Low bandwidth loop reference and buffer */

  bool lpbandgapen;     /* Enable low power band-gap. */
  bool lprefbufen;      /* Enable the 2.5V low power reference buffer */
  bool lprefboosten;    /* Boost buffer current */

  /* DAC reference buffer */

  bool hsdacrefen;      /* Enable DAC reference buffer from HP bandgap */

  /* Misc. control  */

  bool hp1v8thembuff;   /* Thermal buffer for internal 1.8V ref to AIN3 pin */
  bool hp1v8ilimit;     /* Current limit for high power 1.8V ref buffer */
  bool disc1v8cap;      /* Discharge 1.8V capacitor. */
  bool disc1v1cap;      /* Discharge 1.1V capacitor. */
};

/* Structure for AD5940 ADC Basic settings include MUX and PGA. */

struct ad5940_adcbasecfg_s
{
  uint32_t adcmuxp;     /* ADC Positive input channel selection (ADCMUXP) */
  uint32_t adcmuxn;     /* ADC negative input channel selection (ADCMUXN) */
  uint32_t adcpga;      /* ADC PGA settings, select from ADCPGA */
};

/* Structure for AD5940 ADC filter settings. */

struct ad5940_adcfiltercfg_s
{
  uint32_t adcsinc3osr;      /* ADC SINC3 oversample rate */
  uint32_t adcsinc2osr;      /* ADC SINC2 oversample rate */
  uint32_t adcavgnum;        /* An average filter only used by DFT engine. */
  uint32_t adcrate;          /* ADC Core sample rate */
  bool     bpnotch;          /* Bypass Notch filter in SINC2+Notch block */
  bool     bpsinc3;          /* Bypass SINC3 Module */
  bool     sinc2notchenable; /* Enable SINC2+Notch block */
};

/* Structure for calculating how many SYSCLKs needed for N data */

struct ad5940_clkscalinfo_s
{
  uint32_t datatype;         /* The final data output selection. */
  uint32_t datacount;        /* How many data you want. */
  uint32_t adcsinc3osr;      /* ADC SINC3 filter OSR setting */
  uint32_t adcsinc2osr;      /* ADC SINC2 filter OSR setting */
  uint32_t adcavgnum;        /* Average number for DFT engine. */
  uint32_t dftsrc;           /* The DFT source. */
  float    ratiosys2adcclk;  /* Ratio of system clock to ADC clock freq */
  uint8_t  adcrate;          /* ADCRate @ref ADCRATE_Const. */
  bool     bpnotch;          /* Bypass notch filter or not. */
};

/* AD5940 ADC digital comparator */

struct ad5940_adcdigcomp_s
{
  uint16_t adcmin;           /* The ADC code minimum limit value */
  uint16_t adcminhys;        /* The ADC code min. limit hysteresis */
  uint16_t adcmax;           /* The ADC code maximum limit value */
  uint16_t adcmaxhys;        /* The ADC code max. limit hysteresis */
};

/* AD5940 DFT configuration structure. */

struct ad5940_dftcfg_s
{
  uint32_t dftnum;           /* DFT number */
  uint32_t dftsrc;           /* DFT Source */
  bool     hanwinen;         /* Enable Hanning window */
};

/* AD5940 Statistic function */

struct ad5940_statcfg_s
{
  uint32_t statdev;          /* Statistic standard deviation configure */
  uint32_t statsample;       /* Sample size */
  bool     statenable;       /* Set true to enable statistic block */
};

/* AD5940 DSP Configure */

struct ad5940_dspcfg_s
{
  struct ad5940_adcbasecfg_s   adcbasecfg;       /* ADC base config */
  struct ad5940_adcfiltercfg_s adcfiltercfg;     /* ADC filter config  */
  struct ad5940_adcdigcomp_s   adcdigcompcfg;    /* ADC digital comparator */
  struct ad5940_dftcfg_s       dftcfg;           /* DFT configuration */
  struct ad5940_statcfg_s      statcfg;          /* Statistic block */
};

/* AD5940 FIFO configuration */

struct ad5940_fifocfg_s
{
  uint32_t fifomode;         /* Stream mode or standard FIFO mode */
  uint32_t fifosize;         /* How to allocate the internal 6kB SRAM */
  uint32_t fifosrc;          /* Select which data source stored in FIFO */
  uint32_t fifothreshd;      /* FIFO threshold value, 0 to 1023. */
  bool     fifoen;           /* Enable DATAFIFO. Disable FIFO will reset it */
};

/* AD5940 HSTIA Configure */

struct ad5940_hstiacfg_s
{
  uint32_t hstiabias;        /* If select Vzero, VZERO2HSTIA must be closed */
  uint32_t hstiartiasel;     /* RTIA selection @ref HSTIARTIA_Const */
  uint32_t hstiactia;        /* Set internal CTIA value from 1 to 32 pF */
  uint32_t hstiadertia;      /* DE0 node RTIA selection (HSTIADERTIA_Const) */
  uint32_t hstiaderload;     /* DE0 node Rload SEL (HSTIADERLOAD_Const) */
  uint32_t hstiade1rtia;     /* Ignored on AD594x. DE1 node RTIA selection */
  uint32_t hstiade1rload;    /* Ignored on AD594x. DE1 node Rload selection */
  bool     diodeclose;       /* Close switch for internal back-back diode */
};

/* HSDAC Configure */

struct ad5940_hsdaccfg_s
{
  uint32_t excitbufgain;     /* EXCITBUFGAIN_2 or EXCITBUFGAIN_0P25 */
  uint32_t hsdacgain;        /* HSDACGAIN_1 or HSDACGAIN_0P2 */
  uint32_t hsdacupdaterate;  /* Divider for DAC update. 7~255. */
};

/* Switch matrix configure */

struct ad5940_swmatrixcfg_s
{
  uint32_t dswitch;          /* D switch settings. Select from SWD_Const */
  uint32_t pswitch;          /* P switch settings. Select from SWP_Const */
  uint32_t nswitch;          /* N switch settings. Select from SWN_Const */
  uint32_t tswitch;          /* T switch settings. Select from SWT_Const */
};

/* Sin wave generator parameters */

struct ad5940_wgsincfg_s
{
  uint32_t sinfreqword;      /* Frequency word */
  uint32_t sinamplitudeword; /* Amplitude word, range is 0 to 2047 */
  uint32_t sinoffsetword;    /* Offset word, range is 0 to 4095 */
  uint32_t sinphaseword;     /* The start phase of sine wave. */
};

/* Trapezoid Generator parameters
 * The definition of the Trapezoid waveform is shown below. Note the Delay
 * and slope are all in clock unit.
 * DCLevel2         _________
 *                 /         \
 *                /           \
 * DCLevel1 _____/             \______
 *         |     |  |       |  |
 *         Delay1|S1|Delay2 |S2| Delay1 repeat...
 * Where S1 is slope1 and S2 is slop2
 *
 * The DAC update rate from Trapezoid generator is SystemClock/50. The
 * default SystemClock is internal HFOSC 16MHz. So the update rate is 320kHz.
 * The time parameter specifies in clock number. For example, if Delay1 is
 * set to 10, S1 is set 20, the time for Delay1 period is 10/320kHz = 31.25us
 * and time for S1 period is 20/320kHz = 62.5us.
 */

struct ad5940_wgtrapzcfg_s
{
  uint32_t wgtrapzdclevel1;  /* Trapezoid generator DC level1 */
  uint32_t wgtrapzdclevel2;  /* DC level2, similar to DCLevel1 */
  uint32_t wgtrapzdelay1;    /* Trapezoid generator delay 1 */
  uint32_t wgtrapzdelay2;    /* Trapezoid generator delay 2 */
  uint32_t wgtrapzslope1;    /* Trapezoid generator Slope 1 */
  uint32_t wgtrapzslope2;    /* Trapezoid generator Slope 2 */
};

/* Waveform generator configuration */

struct ad5940_wgcfg_s
{
  struct ad5940_wgtrapzcfg_s trapzcfg;    /* Trapezoid generator cfg */
  struct ad5940_wgsincfg_s   sincfg;      /* Sine wave generator cfg */
  uint32_t                   wgtype;      /* WGTYPE_MMR, _SIN or _TRAPZ */
  uint32_t                   wgcode;      /* It'll move to DAC data REG */
  bool                       gaincalen;   /* Enable Gain calibration */
  bool                       offsetcalen; /* Enable offset calibration */
};

/* High speed loop configuration */

struct ad5940_hsloopcfg_s
{
  struct ad5940_swmatrixcfg_s swmatcfg;   /* switch matrix configuration */
  struct ad5940_hsdaccfg_s    hsdaccfg;   /* HSDAC configuration. */
  struct ad5940_wgcfg_s       wgcfg;      /* Waveform generator config */
  struct ad5940_hstiacfg_s    hstiacfg;   /* HSTIA configuration. */
};

/* AD5940 HSTIA internal RTIA calibration structure.
 * ADC filter and DFT should be configured properly based on signal frequency
 */

struct ad5940_hsrtiacal_s
{
  float                    ffreq;         /* Calibration frequency */
  float                    frcal;         /* Rcal resistor value in Ohm */
  float                    sysclkfreq;    /* The real frequency of SYSCLK */
  float                    adcclkfreq;    /* The real frequency of ADCCLK */
  struct ad5940_hstiacfg_s hstiacfg;      /* HSTIA configuration */
  uint32_t                 adcsinc3osr;   /* SINC3OSR_5, _4 or _2 */
  uint32_t                 adcsinc2osr;   /* SINC3OSR_5, _4 or _2 */
  struct ad5940_dftcfg_s   dftcfg;        /* DFT configuration */
  bool                     bpolarresult;  /* True: Polar; false: Cartesian */
};

/* AD5940 LPDAC configuration */

struct ad5940_lpdaccfg_s
{
  uint32_t lpdacsel;         /* Selectr from LPDAC0 or LPDAC1 */
  uint32_t lpdacsrc;         /* LPDACSRC_MMR or LPDACSRC_WG. */
  uint32_t vzeromux;         /* Select which DAC output connects to Vzero */
  uint32_t vbiasmux;         /* Select which DAC output connects to Vbias */
  uint32_t lpdacsw;          /* LPDAC switch set */
  uint32_t lpdacref;         /* Reference selection: internal 2.5V or AVDD */
  uint16_t dacdata12bit;     /* Data for 12bit DAC */
  uint16_t dacdata6bit;      /* Data for 6bit DAC */
  bool     datarst;          /* Keep reset register AFE_LPDACDAT0DATA */
  bool     poweren;          /* Power up AFE_LPDACDAT0 */
};

/* AD5940 low power amplifiers(PA and TIA) configuration */

struct ad5940_lpampcfg_s
{
  uint32_t lpampsel;        /* Select from LPAMP0 and LPAMP1. */
  uint32_t lptiarf;         /* The one order RC filter resistor selection. */
  uint32_t lptiarload;      /* The Rload in front of LPTIA negative input. */
  uint32_t lptiartia;       /* LPTIA RTIA resistor selection. */
  uint32_t lpamppwrmod;     /* Power mode for LP PA and LPTIA */
  uint32_t lptiasw;         /* Set of switches */
  bool     lppapwren;       /* Enable/disable power of PA(potential amp) */
  bool     lptiapwren;      /* Enable/disable power of LPTIA amplifier */
};

/* AD5940 low power loop configuration */

struct ad5940_lploopcfg_s
{
  struct ad5940_lpdaccfg_s lpdaccfg;      /* LPDAC configuration */
  struct ad5940_lpampcfg_s lpampcfg;      /* LPAMP configuration */
};

/* Sequencer configuration */

struct ad5940_seqcfg_s
{
  uint32_t seqmemsize;       /* Sequencer memory size. */
  uint32_t seqwrtimer;       /* Set wait time after every commands executed */
  bool     seqenable;        /* Enable sequencer. It runs only with trigger */
  bool     seqbreaken;       /* Do not use it */
  bool     seqignoreen;      /* Do not use it */
  bool     seqcntcrcclr;     /* Clear sequencer count and CRC */
};

/* AD5940 sequence info structure */

struct ad5940_seqinfo_s
{
  uint32_t           seqid;      /* The sequence ID (4 sequence totally) */
  uint32_t           seqramaddr; /* The start address that in AF5940 SRAM */
  uint32_t           seqlen;     /* Sequence length */
  FAR const uint32_t *pseqcmd;   /* Point to the sequencer commands in MCU */
  bool               writesram;  /* Write command to SRAM or not */
};

/* ad5940 software controlled sweep function */

struct ad5940_softsweepcfg_s
{
  float    sweepstart;           /* Sweep start frequency */
  float    sweepstop;            /* Sweep end frequency */
  uint32_t sweeppoints;          /* Points from START to STOP frequency */
  uint32_t sweepindex;           /* Current position of sweep */
  bool     sweeplog;             /* The step is linear(0) or logarithmic(1) */
  bool     sweepen;              /* Automatically sweep frequency. 1 - EN */
};

/* Wakeup Timer Configure */

struct ad5940_wuptcfg_s
{
  uint32_t wuptendseq;           /* End sequence selection WUPTENDSEQ_Const */
  uint32_t wuptorder[8];         /* The 8 slots for WakeupTimer. */
  uint32_t seqxsleeptime[4];     /* Time before sleep. 0 to 0x000fffff */
  uint32_t seqxwakeuptime[4];    /* Time before wakeup AFE.  */
  bool     wupten;               /* Timer enable. Once enabled, it starts */
};

/* Impedance result in Polar coordinate */

struct ad5940_fimppol_s
{
  float magnitude;    /* The magnitude in polar coordinate */
  float phase;        /* The phase in polar coordinate */
};

/* Impedance result in Cartesian coordinate */

struct ad5940_fimpcar_s
{
  float real;         /* The real part in Cartesian coordinate */
  float image;        /* The imaginary in Cartesian coordinate */
};

/* int32_t type Impedance result in Cartesian coordinate */

struct ad5940_iimpcar_s
{
  int32_t real;       /* The real part in Cartesian coordinate */
  int32_t image;      /* The real imaginary in Cartesian coordinate */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_SENSORS_AD5940_DEF_H */
