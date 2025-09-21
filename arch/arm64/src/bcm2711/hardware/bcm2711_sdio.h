/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_sdio.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_SDIO_H
#define __ARCH_ARM64_SRC_BCM2711_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDIO (EMMC) register offsets, taken from BCM2835 datasheet */

#define BCM_SDIO_ARG2_OFFSET 0x0
#define BCM_SDIO_BLKSIZECNT_OFFSET 0x4
#define BCM_SDIO_ARG1_OFFSET 0x8
#define BCM_SDIO_CMDTM_OFFSET 0xc
#define BCM_SDIO_RESP0_OFFSET 0x10
#define BCM_SDIO_RESP1_OFFSET 0x14
#define BCM_SDIO_RESP2_OFFSET 0x18
#define BCM_SDIO_RESP3_OFFSET 0x1c
#define BCM_SDIO_DATA_OFFSET 0x20
#define BCM_SDIO_STATUS_OFFSET 0x24
#define BCM_SDIO_CONTROL0_OFFSET 0x28
#define BCM_SDIO_CONTROL1_OFFSET 0x2c
#define BCM_SDIO_INTERRUPT_OFFSET 0x30
#define BCM_SDIO_IRPT_MASK_OFFSET 0x34
#define BCM_SDIO_IRPT_EN_OFFSET 0x38
#define BCM_SDIO_CONTROL2_OFFSET 0x3c
#define BCM_SDIO_FORCE_IRPT_OFFSET 0x50
#define BCM_SDIO_BOOT_TIMEOUT_OFFSET 0x70
#define BCM_SDIO_DBG_SEL_OFFSET 0x74
#define BCM_SDIO_EXRDFIFO_CFG_OFFSET 0x80
#define BCM_SDIO_EXRDFIFO_EN_OFFSET 0x84
#define BCM_SDIO_TUNE_STEP_OFFSET 0x88
#define BCM_SDIO_TUNE_STEPS_STD_OFFSET 0x8c
#define BCM_SDIO_TUNE_STEPS_DDR_OFFSET 0x90
#define BCM_SDIO_SPI_INT_SPT_OFFSET 0xf0
#define BCM_SDIO_SLOTISR_VER_OFFSET 0xfc

/* SDIO (EMMC) register addresses */

#define BCM_SDIO_ARG2(base) ((base) + BCM_SDIO_ARG2_OFFSET)
#define BCM_SDIO_BLKSIZECNT(base) ((base) + BCM_SDIO_BLKSIZECNT_OFFSET)
#define BCM_SDIO_ARG1(base) ((base) + BCM_SDIO_ARG1_OFFSET)
#define BCM_SDIO_CMDTM(base) ((base) + BCM_SDIO_CMDTM_OFFSET)
#define BCM_SDIO_RESP0(base) ((base) + BCM_SDIO_RESP0_OFFSET)
#define BCM_SDIO_RESP1(base) ((base) + BCM_SDIO_RESP1_OFFSET)
#define BCM_SDIO_RESP2(base) ((base) + BCM_SDIO_RESP2_OFFSET)
#define BCM_SDIO_RESP3(base) ((base) + BCM_SDIO_RESP3_OFFSET)
#define BCM_SDIO_DATA(base) ((base) + BCM_SDIO_DATA_OFFSET)
#define BCM_SDIO_STATUS(base) ((base) + BCM_SDIO_STATUS_OFFSET)
#define BCM_SDIO_CONTROL0(base) ((base) + BCM_SDIO_CONTROL0_OFFSET)
#define BCM_SDIO_CONTROL1(base) ((base) + BCM_SDIO_CONTROL1_OFFSET)
#define BCM_SDIO_INTERRUPT(base) ((base) + BCM_SDIO_INTERRUPT_OFFSET)
#define BCM_SDIO_IRPT_MASK(base) ((base) + BCM_SDIO_IRPT_MASK_OFFSET)
#define BCM_SDIO_IRPT_EN(base) ((base) + BCM_SDIO_IRPT_EN_OFFSET)
#define BCM_SDIO_CONTROL2(base) ((base) + BCM_SDIO_CONTROL2_OFFSET)
#define BCM_SDIO_FORCE_IRPT(base) ((base) + BCM_SDIO_FORCE_IRPT_OFFSET)
#define BCM_SDIO_BOOT_TIMEOUT(base) ((base) + BCM_SDIO_BOOT_TIMEOUT_OFFSET)
#define BCM_SDIO_DBG_SEL(base) ((base) + BCM_SDIO_DBG_SEL_OFFSET)
#define BCM_SDIO_EXRDFIFO_CFG(base) ((base) + BCM_SDIO_EXRDFIFO_CFG_OFFSET)
#define BCM_SDIO_EXRDFIFO_EN(base) ((base) + BCM_SDIO_EXRDFIFO_EN_OFFSET)
#define BCM_SDIO_TUNE_STEP(base) ((base) + BCM_SDIO_TUNE_STEP_OFFSET)
#define BCM_SDIO_TUNE_STEPS_STD(base)                                        \
  ((base) + BCM_SDIO_TUNE_STEPS_STD_OFFSET)
#define BCM_SDIO_TUNE_STEPS_DDR(base)                                        \
  ((base) + BCM_SDIO_TUNE_STEPS_DDR_OFFSET)
#define BCM_SDIO_SPI_INT_SPT(base) ((base) + BCM_SDIO_SPI_INT_SPT_OFFSET)
#define BCM_SDIO_SLOTISR_VER(base) ((base) + BCM_SDIO_SLOTISR_VER_OFFSET)

/* SDIO (EMMC) register bit definitions.
 *
 * NOTE: some of these definitions are from the BCM2835 datasheet, while
 * others (which you won't find in the datasheet) are from the SD Host
 * Controller Simplified Specification.
 */

#define BCM_SDIO_BLKSIZECNT_BLKSIZE_MASK (0x3ff)
#define BCM_SDIO_BLKSIZECNT_BLKCNT_SHIFTLEN (16)
#define BCM_SDIO_BLKSIZECNT_BLKCNT_MASK (0xffff)

#define BCM_SDIO_CMDTM_TM_BLKCNT_EN (1 << 1)
#define BCM_SDIO_CMDTM_TM_AUTO_CMD_EN (0x3 << 2)
#define BCM_SDIO_CMDTM_TM_DAT_DIR (1 << 4)
#define BCM_SDIO_CMDTM_TM_MULTI_BLOCK (1 << 5)

#define BCM_SDIO_CMDTM_CMD_RSPNS_TYPE (0x3 << 16)
#define BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_NONE (0x0 << 16) /* No response */
#define BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_136 (0x1 << 16)  /* 136 bits */
#define BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_48 (0x2 << 16)   /* 48 bits */
#define BCM_SDIO_CMDTM_CMD_RSPNS_TYPE_48B (0x3 << 16)  /* 48 bits busy */

#define BCM_SDIO_CMDTM_CMD_CRCCHK_EN (1 << 19)
#define BCM_SDIO_CMDTM_CMD_IXCHK_EN (1 << 20)
#define BCM_SDIO_CMDTM_CMD_ISDATA (1 << 21)
#define BCM_SDIO_CMDTM_CMD_TYPE (0x3 << 22)
#define BCM_SDIO_CMDTM_CMD_INDEX_SHIFT (24)
#define BCM_SDIO_CMDTM_CMD_INDEX (0x3f << BCM_SDIO_CMDTM_CMD_INDEX_SHIFT)

#define BCM_SDIO_STATUS_CMD_INHIBIT (1 << 0)
#define BCM_SDIO_STATUS_DAT_INHIBIT (1 << 1)
#define BCM_SDIO_STATUS_DAT_ACTIVE (1 << 2)
#define BCM_SDIO_STATUS_WRITE_TRANSFER (1 << 8)
#define BCM_SDIO_STATUS_READ_TRANSFER (1 << 9)
#define BCM_SDIO_STATUS_CARD_INSERTED (1 << 16)
#define BCM_SDIO_STATUS_CARDDET_STABLE (1 << 17)
#define BCM_SDIO_STATUS_CARDDET_LEVEL (1 << 18)
#define BCM_SDIO_STATUS_WRPROT_LEVEL (1 << 19)
#define BCM_SDIO_STATUS_DAT_LEVEL0 (0xf << 20)
#define BCM_SDIO_STATUS_CMD_LEVEL (1 << 24)
#define BCM_SDIO_STATUS_DAT_LEVEL1 (0xf << 25)

#define BCM_SDIO_CONTROL0_HCTL_DWIDTH (1 << 1)
#define BCM_SDIO_CONTROL0_HCTL_HS_EN (1 << 2)
#define BCM_SDIO_CONTROL0_HCTL_8BIT (1 << 5)
#define BCM_SDIO_CONTROL0_VDD1_ON (1 << 8)
#define BCM_SDIO_CONTROL0_VDD1_SHIFT (9)
#define BCM_SDIO_CONTROL0_VDD1_MASK (0x7)
#define BCM_SDIO_CONTROL0_VDD1_3V3 (0x7 << BCM_SDIO_CONTROL0_VDD1_SHIFT)
#define BCM_SDIO_CONTROL0_VDD1_3V (0x6 << BCM_SDIO_CONTROL0_VDD1_SHIFT)
#define BCM_SDIO_CONTROL0_VDD1_1V8 (0x5 << BCM_SDIO_CONTROL0_VDD1_SHIFT)
#define BCM_SDIO_CONTROL0_GAP_STOP (1 << 16)
#define BCM_SDIO_CONTROL0_GAP_RESTART (1 << 17)
#define BCM_SDIO_CONTROL0_READWAIT_EN (1 << 18)
#define BCM_SDIO_CONTROL0_GAP_IEN (1 << 19)
#define BCM_SDIO_CONTROL0_SPI_MODE (1 << 20)
#define BCM_SDIO_CONTROL0_BOOT_EN (1 << 21)
#define BCM_SDIO_CONTROL0_ALT_BOOT_EN (1 << 22)
#define BCM_SDIO_CONTROL0_WKUP_INSERT (1 << 25)
#define BCM_SDIO_CONTROL0_WKUP_REMOVAL (1 << 26)

#define BCM_SDIO_CONTROL1_CLK_INTLEN (1 << 0)
#define BCM_SDIO_CONTROL1_CLK_STABLE (1 << 1)
#define BCM_SDIO_CONTROL1_CLK_EN (1 << 2)
#define BCM_SDIO_CONTROL1_CLK_GENSEL (1 << 5)
#define BCM_SDIO_CONTROL1_CLK_FREQ_MS2 (0x3 << 6)
#define BCM_SDIO_CONTROL1_CLK_FREQ8 (0xff << 8)
#define BCM_SDIO_CONTROL1_DATA_TOUNIT_MASK (0xf)
#define BCM_SDIO_CONTROL1_DATA_TOUNIT_SHIFT (16)
#define BCM_SDIO_CONTROL1_DATA_TOUNIT                                        \
  (BCM_SDIO_CONTROL1_DATA_TOUNIT_MASK << BCM_SDIO_CONTROL1_DATA_TOUNIT_SHIFT)
#define BCM_SDIO_CONTROL1_SRST_HC (1 << 24)
#define BCM_SDIO_CONTROL1_SRST_CMD (1 << 25)
#define BCM_SDIO_CONTROL1_SRST_DATA (1 << 26)

#define BCM_SDIO_INTERRUPT_CMD_DONE (1 << 0)
#define BCM_SDIO_INTERRUPT_DATA_DONE (1 << 1)
#define BCM_SDIO_INTERRUPT_BLOCK_GAP (1 << 2)
#define BCM_SDIO_INTERRUPT_WRITE_RDY (1 << 4)
#define BCM_SDIO_INTERRUPT_READ_RDY (1 << 5)
#define BCM_SDIO_INTERRUPT_INSERTION (1 << 6)
#define BCM_SDIO_INTERRUPT_REMOVAL (1 << 7)
#define BCM_SDIO_INTERRUPT_CARD (1 << 8)
#define BCM_SDIO_INTERRUPT_RETUNE (1 << 12)
#define BCM_SDIO_INTERRUPT_BOOTACK (1 << 13)
#define BCM_SDIO_INTERRUPT_ENDBOOT (1 << 14)
#define BCM_SDIO_INTERRUPT_ERR (1 << 15)
#define BCM_SDIO_INTERRUPT_CTO_ERR (1 << 16)
#define BCM_SDIO_INTERRUPT_CCRC_ERR (1 << 17)
#define BCM_SDIO_INTERRUPT_CEND_ERR (1 << 18)
#define BCM_SDIO_INTERRUPT_CBAD_ERR (1 << 19)
#define BCM_SDIO_INTERRUPT_DTO_ERR (1 << 20)
#define BCM_SDIO_INTERRUPT_DCRC_ERR (1 << 21)
#define BCM_SDIO_INTERRUPT_DEND_ERR (1 << 22)
#define BCM_SDIO_INTERRUPT_ACMD_ERR (1 << 24)
#define BCM_SDIO_INTERRUPT_ALL                                               \
  (BCM_SDIO_INTERRUPT_CMD_DONE | BCM_SDIO_INTERRUPT_DATA_DONE |              \
   BCM_SDIO_INTERRUPT_BLOCK_GAP | BCM_SDIO_INTERRUPT_WRITE_RDY |             \
   BCM_SDIO_INTERRUPT_READ_RDY | BCM_SDIO_INTERRUPT_INSERTION |              \
   BCM_SDIO_INTERRUPT_REMOVAL | BCM_SDIO_INTERRUPT_CARD |                    \
   BCM_SDIO_INTERRUPT_RETUNE | BCM_SDIO_INTERRUPT_BOOTACK |                  \
   BCM_SDIO_INTERRUPT_ENDBOOT | BCM_SDIO_INTERRUPT_CTO_ERR |                 \
   BCM_SDIO_INTERRUPT_CCRC_ERR | BCM_SDIO_INTERRUPT_CEND_ERR |               \
   BCM_SDIO_INTERRUPT_CBAD_ERR | BCM_SDIO_INTERRUPT_DTO_ERR |                \
   BCM_SDIO_INTERRUPT_DCRC_ERR | BCM_SDIO_INTERRUPT_DEND_ERR |               \
   BCM_SDIO_INTERRUPT_ACMD_ERR)

#define BCM_SDIO_IRPT_MASK_CMD_DONE (1 << 0)
#define BCM_SDIO_IRPT_MASK_DATA_DONE (1 << 1)
#define BCM_SDIO_IRPT_MASK_BLOCK_GAP (1 << 2)
#define BCM_SDIO_IRPT_MASK_WRITE_RDY (1 << 4)
#define BCM_SDIO_IRPT_MASK_READ_RDY (1 << 5)
#define BCM_SDIO_IRPT_MASK_INSERTION (1 << 6)
#define BCM_SDIO_IRPT_MASK_REMOVAL (1 << 7)
#define BCM_SDIO_IRPT_MASK_CARD (1 << 8)
#define BCM_SDIO_IRPT_MASK_RETUNE (1 << 12)
#define BCM_SDIO_IRPT_MASK_BOOTACK (1 << 13)
#define BCM_SDIO_IRPT_MASK_ENDBOOT (1 << 14)
#define BCM_SDIO_IRPT_MASK_CTO_ERR (1 << 16)
#define BCM_SDIO_IRPT_MASK_CCRC_ERR (1 << 17)
#define BCM_SDIO_IRPT_MASK_CEND_ERR (1 << 18)
#define BCM_SDIO_IRPT_MASK_CBAD_ERR (1 << 19)
#define BCM_SDIO_IRPT_MASK_DTO_ERR (1 << 20)
#define BCM_SDIO_IRPT_MASK_DCRC_ERR (1 << 21)
#define BCM_SDIO_IRPT_MASK_DEND_ERR (1 << 22)
#define BCM_SDIO_IRPT_MASK_ACMD_ERR (1 << 24)
#define BCM_SDIO_IRPT_MASK_ALL                                               \
  (BCM_SDIO_IRPT_MASK_CMD_DONE | BCM_SDIO_IRPT_MASK_DATA_DONE |              \
   BCM_SDIO_IRPT_MASK_BLOCK_GAP | BCM_SDIO_IRPT_MASK_WRITE_RDY |             \
   BCM_SDIO_IRPT_MASK_READ_RDY | BCM_SDIO_IRPT_MASK_INSERTION |              \
   BCM_SDIO_IRPT_MASK_REMOVAL | BCM_SDIO_IRPT_MASK_CARD |                    \
   BCM_SDIO_IRPT_MASK_RETUNE | BCM_SDIO_IRPT_MASK_BOOTACK |                  \
   BCM_SDIO_IRPT_MASK_ENDBOOT | BCM_SDIO_IRPT_MASK_CTO_ERR |                 \
   BCM_SDIO_IRPT_MASK_CCRC_ERR | BCM_SDIO_IRPT_MASK_CEND_ERR |               \
   BCM_SDIO_IRPT_MASK_CBAD_ERR | BCM_SDIO_IRPT_MASK_DTO_ERR |                \
   BCM_SDIO_IRPT_MASK_DCRC_ERR | BCM_SDIO_IRPT_MASK_DEND_ERR |               \
   BCM_SDIO_IRPT_MASK_ACMD_ERR)

#define BCM_SDIO_IRPT_EN_CMD_DONE (1 << 0)
#define BCM_SDIO_IRPT_EN_DATA_DONE (1 << 1)
#define BCM_SDIO_IRPT_EN_BLOCK_GAP (1 << 2)
#define BCM_SDIO_IRPT_EN_WRITE_RDY (1 << 4)
#define BCM_SDIO_IRPT_EN_READ_RDY (1 << 5)
#define BCM_SDIO_IRPT_EN_INSERTION (1 << 6)
#define BCM_SDIO_IRPT_EN_REMOVAL (1 << 7)
#define BCM_SDIO_IRPT_EN_CARD (1 << 8)
#define BCM_SDIO_IRPT_EN_RETUNE (1 << 12)
#define BCM_SDIO_IRPT_EN_BOOTACK (1 << 13)
#define BCM_SDIO_IRPT_EN_ENDBOOT (1 << 14)
#define BCM_SDIO_IRPT_EN_CTO_ERR (1 << 16)
#define BCM_SDIO_IRPT_EN_CCRC_ERR (1 << 17)
#define BCM_SDIO_IRPT_EN_CEND_ERR (1 << 18)
#define BCM_SDIO_IRPT_EN_CBAD_ERR (1 << 19)
#define BCM_SDIO_IRPT_EN_DTO_ERR (1 << 20)
#define BCM_SDIO_IRPT_EN_DCRC_ERR (1 << 21)
#define BCM_SDIO_IRPT_EN_DEND_ERR (1 << 22)
#define BCM_SDIO_IRPT_EN_ACMD_ERR (1 << 24)
#define BCM_SDIO_IRPT_EN_ALL                                                 \
  (BCM_SDIO_IRPT_EN_CMD_DONE | BCM_SDIO_IRPT_EN_DATA_DONE |                  \
   BCM_SDIO_IRPT_EN_BLOCK_GAP | BCM_SDIO_IRPT_EN_WRITE_RDY |                 \
   BCM_SDIO_IRPT_EN_READ_RDY | BCM_SDIO_IRPT_EN_INSERTION |                  \
   BCM_SDIO_IRPT_EN_REMOVAL | BCM_SDIO_IRPT_EN_CARD |                        \
   BCM_SDIO_IRPT_EN_RETUNE | BCM_SDIO_IRPT_EN_BOOTACK |                      \
   BCM_SDIO_IRPT_EN_ENDBOOT | BCM_SDIO_IRPT_EN_CTO_ERR |                     \
   BCM_SDIO_IRPT_EN_CCRC_ERR | BCM_SDIO_IRPT_EN_CEND_ERR |                   \
   BCM_SDIO_IRPT_EN_CBAD_ERR | BCM_SDIO_IRPT_EN_DTO_ERR |                    \
   BCM_SDIO_IRPT_EN_DCRC_ERR | BCM_SDIO_IRPT_EN_DEND_ERR |                   \
   BCM_SDIO_IRPT_EN_ACMD_ERR)

#define BCM_SDIO_CONTROL2_ACNOX_ERR (1 << 0)
#define BCM_SDIO_CONTROL2_ACTO_ERR (1 << 1)
#define BCM_SDIO_CONTROL2_ACCRC_ERR (1 << 2)
#define BCM_SDIO_CONTROL2_ACEND_ERR (1 << 3)
#define BCM_SDIO_CONTROL2_ACBAD_ERR (1 << 4)
#define BCM_SDIO_CONTROL2_NOTC12_ERR (1 << 7)
#define BCM_SDIO_CONTROL2_UHSMODE (0x7 << 16)
#define BCM_SDIO_CONTROL2_TUNEON (1 << 22)
#define BCM_SDIO_CONTROL2_TUNED (1 << 23)

#define BCM_SDIO_FORCE_IRPT_CMD_DONE (1 << 0)
#define BCM_SDIO_FORCE_IRPT_DATA_DONE (1 << 1)
#define BCM_SDIO_FORCE_IRPT_BLOCK_GAP (1 << 2)
#define BCM_SDIO_FORCE_IRPT_WRITE_RDY (1 << 4)
#define BCM_SDIO_FORCE_IRPT_READ_RDY (1 << 5)
#define BCM_SDIO_FORCE_IRPT_CARD (1 << 8)
#define BCM_SDIO_FORCE_IRPT_RETUNE (1 << 12)
#define BCM_SDIO_FORCE_IRPT_BOOTACK (1 << 13)
#define BCM_SDIO_FORCE_IRPT_ENDBOOT (1 << 14)
#define BCM_SDIO_FORCE_IRPT_CTO_ERR (1 << 16)
#define BCM_SDIO_FORCE_IRPT_CCRC_ERR (1 << 17)
#define BCM_SDIO_FORCE_IRPT_CEND_ERR (1 << 18)
#define BCM_SDIO_FORCE_IRPT_CBAD_ERR (1 << 19)
#define BCM_SDIO_FORCE_IRPT_DTO_ERR (1 << 20)
#define BCM_SDIO_FORCE_IRPT_DCRC_ERR (1 << 21)
#define BCM_SDIO_FORCE_IRPT_DEND_ERR (1 << 22)
#define BCM_SDIO_FORCE_IRPT_ACMD_ERR (1 << 24)
#define BCM_SDIO_FORCE_IRPT_ALL (0x17ff137)

#define BCM_SDIO_DBG_SEL_SELECT (1 << 0)

#define BCM_SDIO_EXRDFIFO_CFG_RD_THRSH (0x7 << 0)

#define BCM_SDIO_EXRDFIFO_EN_ENABLE (1 << 0)

#define BCM_SDIO_TUNE_STEP_DELAY (0x7 << 0)

#define BCM_SDIO_TUNE_STEPS_STD_STEPS (0x3f << 0)

#define BCM_SDIO_TUNE_STEPS_DDR_STEPS (0x3f << 0)

#define BCM_SDIO_SPI_INT_SPT_SELECT (0xff << 0)

#define BCM_SDIO_SLOTISR_VER_SLOT_STATUS (0xff << 0)
#define BCM_SDIO_SLOTISR_VER_SDVERSION (0xff << 16)
#define BCM_SDIO_SLOTISR_VER_VENDOR (0xff << 24)

#endif /* __ARCH_ARM64_SRC_BCM2711_SDIO_H */
