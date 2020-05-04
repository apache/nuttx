/****************************************************************************
 * arch/arm/src/armv8-m/fpb.h
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

#ifndef __ARCH_ARM_SRC_ARMV8-M_FPB_H
#define __ARCH_ARM_SRC_ARMV8-M_FPB_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash Patch and Breakpoint Unit FPB **************************************/

/* FPB Register Base Address ************************************************/

#define FPB_BASE                  0xe0002000

/* FPB Register Offsets *****************************************************/

#define FPB_CTRL_OFFSET       0x0000  /* Control */
#define FPB_REMAP_OFFSET      0x0004  /* Remap */
#define FPB_COMP0_OFFSET      0x0008  /* Comparator 0 */
#define FPB_COMP1_OFFSET      0x000c  /* Comparator 1 */
#define FPB_COMP2_OFFSET      0x0010  /* Comparator 2 */
#define FPB_COMP3_OFFSET      0x0014  /* Comparator 3 */
#define FPB_COMP4_OFFSET      0x0018  /* Comparator 4 */
#define FPB_COMP5_OFFSET      0x001C  /* Comparator 5 */
#define FPB_COMP6_OFFSET      0x0020  /* Comparator 6 */
#define FPB_COMP7_OFFSET      0x0024  /* Comparator 7 */

/* FPB Register Addresses ***************************************************/

#define FPB_CTRL              (FPB_BASE + FPB_CTRL_OFFSET)
#define FPB_REMAP             (FPB_BASE + FPB_REMAP_OFFSET)
#define FPB_COMP0             (FPB_BASE + FPB_COMP0_OFFSET)
#define FPB_COMP1             (FPB_BASE + FPB_COMP1_OFFSET)
#define FPB_COMP2             (FPB_BASE + FPB_COMP2_OFFSET)
#define FPB_COMP3             (FPB_BASE + FPB_COMP3_OFFSET)
#define FPB_COMP4             (FPB_BASE + FPB_COMP4_OFFSET)
#define FPB_COMP5             (FPB_BASE + FPB_COMP5_OFFSET)
#define FPB_COMP6             (FPB_BASE + FPB_COMP6_OFFSET)
#define FPB_COMP7             (FPB_BASE + FPB_COMP7_OFFSET

/* FPB Register Bitfield Definitions ****************************************/

/* FPB_CTRL */

/* NUM_CODE2
 *
 * Number of full banks of code comparators, sixteen comparators per bank.
 * Where less than sixteen code comparators are provided, the bank count is
 * zero, and the number present indicated by NUM_CODE1. This read only field
 * contains 3'b000 to indicate 0 banks for Cortex-M processor.
 */

#define FPB_CTRL_NUM_CODE2_SHIFT  12
#define FPB_CTRL_NUM_CODE2_MASK   0x00003000

/* NUM_LIT
 *
 * Number of literal slots field.
 *
 * 0: No literal slots
 * 2: Two literal slots
 */

#define FPB_CTRL_NUM_LIT_SHIFT    8
#define FPB_CTRL_NUM_LIT_MASK     0x00000f00

/* NUM_CODE1
 *
 * Number of code slots field.
 *
 * 0: No code slots
 * 2: Two code slots
 * 6: Six code slots
 */

#define FPB_CTRL_NUM_CODE1_SHIFT  4
#define FPB_CTRL_NUM_CODE1_MASK   0x000000f0

/* KEY
 *
 * Key field. In order to write to this register, this bit-field must be
 * written to '1'. This bit always reads 0.
 */

#define FPB_CTRL_KEY_SHIFT        1
#define FPB_CTRL_KEY_MASK         0x00000002
#  define FPB_CTRL_KEY            0x00000002

/* ENABLE
 *
 * Flash patch unit enable bit
 *
 * 0: Flash patch unit disabled
 * 1: Flash patch unit enabled
 */

#define FPB_CTRL_ENABLE_SHIFT     0
#define FPB_CTRL_ENABLE_MASK      0x00000001
#  define FPB_CTRL_ENABLE         0x00000001

/* FPB_REMAP */

/* REMAP
 *
 * Remap base address field.
 */

#define FPB_REMAP_REMAP_SHIFT     5
#define FPB_REMAP_REMAP_MASK      0x1fffffe0

/* FPB_COMP0 - FPB_COMP7 */

/* REPLACE
 *
 * This selects what happens when the COMP address is matched. Address
 * remapping only takes place for the 0x0 setting.
 *
 * 0: Remap to remap address. See REMAP.REMAP
 * 1: Set BKPT on lower halfword, upper is unaffected
 * 2: Set BKPT on upper halfword, lower is unaffected
 * 3: Set BKPT on both lower and upper halfwords.
 */

#define FPB_COMP0_REPLACE_SHIFT   30
#define FPB_COMP0_REPLACE_MASK    0xc0000000

/* COMP
 *
 * Comparison address.
 */

#define FPB_COMP0_COMP_SHIFT      2
#define FPB_COMP0_COMP_MASK       0x1ffffffc

/* ENABLE
 *
 * Compare and remap enable comparator. CTRL.ENABLE must also be set to
 * enable comparisons.
 *
 * 0: Compare and remap for comparator 0 disabled
 * 1: Compare and remap for comparator 0 enabled
 */

#define FPB_COMP0_ENABLE_MASK     0x00000001
#define FPB_COMP0_ENABLE_SHIFT    0
#  define FPB_COMP0_ENABLE        0x00000001

#endif /* __ARCH_ARM_SRC_ARMV8-M_FPB_H */
