/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_sf.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SF_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_SF_0_OFFSET                0x000000  /* sf_ctrl_0 */
#define BL602_SF_1_OFFSET                0x000004  /* sf_ctrl_1 */
#define BL602_SF_IF_SAHB_0_OFFSET        0x000008  /* sf_if_sahb_0 */
#define BL602_SF_IF_SAHB_1_OFFSET        0x00000c  /* sf_if_sahb_1 */
#define BL602_SF_IF_SAHB_2_OFFSET        0x000010  /* sf_if_sahb_2 */
#define BL602_SF_IF_IAHB_0_OFFSET        0x000014  /* sf_if_iahb_0 */
#define BL602_SF_IF_IAHB_1_OFFSET        0x000018  /* sf_if_iahb_1 */
#define BL602_SF_IF_IAHB_2_OFFSET        0x00001c  /* sf_if_iahb_2 */
#define BL602_SF_IF_STATUS_0_OFFSET      0x000020  /* sf_if_status_0 */
#define BL602_SF_IF_STATUS_1_OFFSET      0x000024  /* sf_if_status_1 */
#define BL602_SF_AES_OFFSET              0x000028  /* sf_aes */
#define BL602_SF_AHB2SIF_STATUS_OFFSET   0x00002c  /* sf_ahb2sif_status */
#define BL602_SF_IF_IO_DLY_0_OFFSET      0x000030  /* sf_if_io_dly_0 */
#define BL602_SF_IF_IO_DLY_1_OFFSET      0x000034  /* sf_if_io_dly_1 */
#define BL602_SF_IF_IO_DLY_2_OFFSET      0x000038  /* sf_if_io_dly_2 */
#define BL602_SF_IF_IO_DLY_3_OFFSET      0x00003c  /* sf_if_io_dly_3 */
#define BL602_SF_IF_IO_DLY_4_OFFSET      0x000040  /* sf_if_io_dly_4 */
#define BL602_SF_RESERVED_OFFSET         0x000044  /* sf_reserved */
#define BL602_SF2_IF_IO_DLY_0_OFFSET     0x000048  /* sf2_if_io_dly_0 */
#define BL602_SF2_IF_IO_DLY_1_OFFSET     0x00004c  /* sf2_if_io_dly_1 */
#define BL602_SF2_IF_IO_DLY_2_OFFSET     0x000050  /* sf2_if_io_dly_2 */
#define BL602_SF2_IF_IO_DLY_3_OFFSET     0x000054  /* sf2_if_io_dly_3 */
#define BL602_SF2_IF_IO_DLY_4_OFFSET     0x000058  /* sf2_if_io_dly_4 */
#define BL602_SF3_IF_IO_DLY_0_OFFSET     0x00005c  /* sf3_if_io_dly_0 */
#define BL602_SF3_IF_IO_DLY_1_OFFSET     0x000060  /* sf3_if_io_dly_1 */
#define BL602_SF3_IF_IO_DLY_2_OFFSET     0x000064  /* sf3_if_io_dly_2 */
#define BL602_SF3_IF_IO_DLY_3_OFFSET     0x000068  /* sf3_if_io_dly_3 */
#define BL602_SF3_IF_IO_DLY_4_OFFSET     0x00006c  /* sf3_if_io_dly_4 */
#define BL602_SF_2_OFFSET                0x000070  /* sf_ctrl_2 */
#define BL602_SF_3_OFFSET                0x000074  /* sf_ctrl_3 */
#define BL602_SF_IF_IAHB_3_OFFSET        0x000078  /* sf_if_iahb_3 */
#define BL602_SF_IF_IAHB_4_OFFSET        0x00007c  /* sf_if_iahb_4 */
#define BL602_SF_IF_IAHB_5_OFFSET        0x000080  /* sf_if_iahb_5 */
#define BL602_SF_IF_IAHB_6_OFFSET        0x000084  /* sf_if_iahb_6 */
#define BL602_SF_IF_IAHB_7_OFFSET        0x000088  /* sf_if_iahb_7 */
#define BL602_SF_PROT_EN_RD_OFFSET       0x000100  /* sf_ctrl_prot_en_rd */
#define BL602_SF_PROT_EN_OFFSET          0x000104  /* sf_ctrl_prot_en */
#define BL602_SF_AES_KEY_R0_0_OFFSET     0x000200  /* sf_aes_key_r0_0 */
#define BL602_SF_AES_KEY_R0_1_OFFSET     0x000204  /* sf_aes_key_r0_1 */
#define BL602_SF_AES_KEY_R0_2_OFFSET     0x000208  /* sf_aes_key_r0_2 */
#define BL602_SF_AES_KEY_R0_3_OFFSET     0x00020c  /* sf_aes_key_r0_3 */
#define BL602_SF_AES_KEY_R0_4_OFFSET     0x000210  /* sf_aes_key_r0_4 */
#define BL602_SF_AES_KEY_R0_5_OFFSET     0x000214  /* sf_aes_key_r0_5 */
#define BL602_SF_AES_KEY_R0_6_OFFSET     0x000218  /* sf_aes_key_r0_6 */
#define BL602_SF_AES_KEY_R0_7_OFFSET     0x00021c  /* sf_aes_key_r0_7 */
#define BL602_SF_AES_IV_R0_W0_OFFSET     0x000220  /* sf_aes_iv_r0_w0 */
#define BL602_SF_AES_IV_R0_W1_OFFSET     0x000224  /* sf_aes_iv_r0_w1 */
#define BL602_SF_AES_IV_R0_W2_OFFSET     0x000228  /* sf_aes_iv_r0_w2 */
#define BL602_SF_AES_IV_R0_W3_OFFSET     0x00022c  /* sf_aes_iv_r0_w3 */
#define BL602_SF_AES_CFG_R0_OFFSET       0x000230  /* sf_aes_cfg_r0 */
#define BL602_SF_AES_KEY_R1_0_OFFSET     0x000300  /* sf_aes_key_r1_0 */
#define BL602_SF_AES_KEY_R1_1_OFFSET     0x000304  /* sf_aes_key_r1_1 */
#define BL602_SF_AES_KEY_R1_2_OFFSET     0x000308  /* sf_aes_key_r1_2 */
#define BL602_SF_AES_KEY_R1_3_OFFSET     0x00030c  /* sf_aes_key_r1_3 */
#define BL602_SF_AES_KEY_R1_4_OFFSET     0x000310  /* sf_aes_key_r1_4 */
#define BL602_SF_AES_KEY_R1_5_OFFSET     0x000314  /* sf_aes_key_r1_5 */
#define BL602_SF_AES_KEY_R1_6_OFFSET     0x000318  /* sf_aes_key_r1_6 */
#define BL602_SF_AES_KEY_R1_7_OFFSET     0x00031c  /* sf_aes_key_r1_7 */
#define BL602_SF_AES_IV_R1_W0_OFFSET     0x000320  /* sf_aes_iv_r1_w0 */
#define BL602_SF_AES_IV_R1_W1_OFFSET     0x000324  /* sf_aes_iv_r1_w1 */
#define BL602_SF_AES_IV_R1_W2_OFFSET     0x000328  /* sf_aes_iv_r1_w2 */
#define BL602_SF_AES_IV_R1_W3_OFFSET     0x00032c  /* sf_aes_iv_r1_w3 */
#define BL602_SF_AES_R1_OFFSET           0x000330  /* sf_aes_r1 */
#define BL602_SF_AES_KEY_R2_0_OFFSET     0x000400  /* sf_aes_key_r2_0 */
#define BL602_SF_AES_KEY_R2_1_OFFSET     0x000404  /* sf_aes_key_r2_1 */
#define BL602_SF_AES_KEY_R2_2_OFFSET     0x000408  /* sf_aes_key_r2_2 */
#define BL602_SF_AES_KEY_R2_3_OFFSET     0x00040c  /* sf_aes_key_r2_3 */
#define BL602_SF_AES_KEY_R2_4_OFFSET     0x000410  /* sf_aes_key_r2_4 */
#define BL602_SF_AES_KEY_R2_5_OFFSET     0x000414  /* sf_aes_key_r2_5 */
#define BL602_SF_AES_KEY_R2_6_OFFSET     0x000418  /* sf_aes_key_r2_6 */
#define BL602_SF_AES_KEY_R2_7_OFFSET     0x00041c  /* sf_aes_key_r2_7 */
#define BL602_SF_AES_IV_R2_W0_OFFSET     0x000420  /* sf_aes_iv_r2_w0 */
#define BL602_SF_AES_IV_R2_W1_OFFSET     0x000424  /* sf_aes_iv_r2_w1 */
#define BL602_SF_AES_IV_R2_W2_OFFSET     0x000428  /* sf_aes_iv_r2_w2 */
#define BL602_SF_AES_IV_R2_W3_OFFSET     0x00042c  /* sf_aes_iv_r2_w3 */
#define BL602_SF_AES_R2_OFFSET           0x000430  /* sf_aes_r2 */
#define BL602_SF_ID0_OFFSET_OFFSET       0x000434  /* sf_id0_offset */
#define BL602_SF_ID1_OFFSET_OFFSET       0x000438  /* sf_id1_offset */

/* Register definitions *****************************************************/

#define BL602_SF_0                (BL602_SF_BASE + BL602_SF_0_OFFSET)
#define BL602_SF_1                (BL602_SF_BASE + BL602_SF_1_OFFSET)
#define BL602_SF_IF_SAHB_0        (BL602_SF_BASE + BL602_SF_IF_SAHB_0_OFFSET)
#define BL602_SF_IF_SAHB_1        (BL602_SF_BASE + BL602_SF_IF_SAHB_1_OFFSET)
#define BL602_SF_IF_SAHB_2        (BL602_SF_BASE + BL602_SF_IF_SAHB_2_OFFSET)
#define BL602_SF_IF_IAHB_0        (BL602_SF_BASE + BL602_SF_IF_IAHB_0_OFFSET)
#define BL602_SF_IF_IAHB_1        (BL602_SF_BASE + BL602_SF_IF_IAHB_1_OFFSET)
#define BL602_SF_IF_IAHB_2        (BL602_SF_BASE + BL602_SF_IF_IAHB_2_OFFSET)
#define BL602_SF_IF_STATUS_0      (BL602_SF_BASE + BL602_SF_IF_STATUS_0_OFFSET)
#define BL602_SF_IF_STATUS_1      (BL602_SF_BASE + BL602_SF_IF_STATUS_1_OFFSET)
#define BL602_SF_AES              (BL602_SF_BASE + BL602_SF_AES_OFFSET)
#define BL602_SF_AHB2SIF_STATUS   (BL602_SF_BASE + BL602_SF_AHB2SIF_STATUS_OFFSET)
#define BL602_SF_IF_IO_DLY_0      (BL602_SF_BASE + BL602_SF_IF_IO_DLY_0_OFFSET)
#define BL602_SF_IF_IO_DLY_1      (BL602_SF_BASE + BL602_SF_IF_IO_DLY_1_OFFSET)
#define BL602_SF_IF_IO_DLY_2      (BL602_SF_BASE + BL602_SF_IF_IO_DLY_2_OFFSET)
#define BL602_SF_IF_IO_DLY_3      (BL602_SF_BASE + BL602_SF_IF_IO_DLY_3_OFFSET)
#define BL602_SF_IF_IO_DLY_4      (BL602_SF_BASE + BL602_SF_IF_IO_DLY_4_OFFSET)
#define BL602_SF_RESERVED         (BL602_SF_BASE + BL602_SF_RESERVED_OFFSET)
#define BL602_SF2_IF_IO_DLY_0     (BL602_SF_BASE + BL602_SF2_IF_IO_DLY_0_OFFSET)
#define BL602_SF2_IF_IO_DLY_1     (BL602_SF_BASE + BL602_SF2_IF_IO_DLY_1_OFFSET)
#define BL602_SF2_IF_IO_DLY_2     (BL602_SF_BASE + BL602_SF2_IF_IO_DLY_2_OFFSET)
#define BL602_SF2_IF_IO_DLY_3     (BL602_SF_BASE + BL602_SF2_IF_IO_DLY_3_OFFSET)
#define BL602_SF2_IF_IO_DLY_4     (BL602_SF_BASE + BL602_SF2_IF_IO_DLY_4_OFFSET)
#define BL602_SF3_IF_IO_DLY_0     (BL602_SF_BASE + BL602_SF3_IF_IO_DLY_0_OFFSET)
#define BL602_SF3_IF_IO_DLY_1     (BL602_SF_BASE + BL602_SF3_IF_IO_DLY_1_OFFSET)
#define BL602_SF3_IF_IO_DLY_2     (BL602_SF_BASE + BL602_SF3_IF_IO_DLY_2_OFFSET)
#define BL602_SF3_IF_IO_DLY_3     (BL602_SF_BASE + BL602_SF3_IF_IO_DLY_3_OFFSET)
#define BL602_SF3_IF_IO_DLY_4     (BL602_SF_BASE + BL602_SF3_IF_IO_DLY_4_OFFSET)
#define BL602_SF_2                (BL602_SF_BASE + BL602_SF_2_OFFSET)
#define BL602_SF_3                (BL602_SF_BASE + BL602_SF_3_OFFSET)
#define BL602_SF_IF_IAHB_3        (BL602_SF_BASE + BL602_SF_IF_IAHB_3_OFFSET)
#define BL602_SF_IF_IAHB_4        (BL602_SF_BASE + BL602_SF_IF_IAHB_4_OFFSET)
#define BL602_SF_IF_IAHB_5        (BL602_SF_BASE + BL602_SF_IF_IAHB_5_OFFSET)
#define BL602_SF_IF_IAHB_6        (BL602_SF_BASE + BL602_SF_IF_IAHB_6_OFFSET)
#define BL602_SF_IF_IAHB_7        (BL602_SF_BASE + BL602_SF_IF_IAHB_7_OFFSET)
#define BL602_SF_PROT_EN_RD       (BL602_SF_BASE + BL602_SF_PROT_EN_RD_OFFSET)
#define BL602_SF_PROT_EN          (BL602_SF_BASE + BL602_SF_PROT_EN_OFFSET)
#define BL602_SF_AES_KEY_R0_0     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_0_OFFSET)
#define BL602_SF_AES_KEY_R0_1     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_1_OFFSET)
#define BL602_SF_AES_KEY_R0_2     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_2_OFFSET)
#define BL602_SF_AES_KEY_R0_3     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_3_OFFSET)
#define BL602_SF_AES_KEY_R0_4     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_4_OFFSET)
#define BL602_SF_AES_KEY_R0_5     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_5_OFFSET)
#define BL602_SF_AES_KEY_R0_6     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_6_OFFSET)
#define BL602_SF_AES_KEY_R0_7     (BL602_SF_BASE + BL602_SF_AES_KEY_R0_7_OFFSET)
#define BL602_SF_AES_IV_R0_W0     (BL602_SF_BASE + BL602_SF_AES_IV_R0_W0_OFFSET)
#define BL602_SF_AES_IV_R0_W1     (BL602_SF_BASE + BL602_SF_AES_IV_R0_W1_OFFSET)
#define BL602_SF_AES_IV_R0_W2     (BL602_SF_BASE + BL602_SF_AES_IV_R0_W2_OFFSET)
#define BL602_SF_AES_IV_R0_W3     (BL602_SF_BASE + BL602_SF_AES_IV_R0_W3_OFFSET)
#define BL602_SF_AES_CFG_R0       (BL602_SF_BASE + BL602_SF_AES_CFG_R0_OFFSET)
#define BL602_SF_AES_KEY_R1_0     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_0_OFFSET)
#define BL602_SF_AES_KEY_R1_1     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_1_OFFSET)
#define BL602_SF_AES_KEY_R1_2     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_2_OFFSET)
#define BL602_SF_AES_KEY_R1_3     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_3_OFFSET)
#define BL602_SF_AES_KEY_R1_4     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_4_OFFSET)
#define BL602_SF_AES_KEY_R1_5     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_5_OFFSET)
#define BL602_SF_AES_KEY_R1_6     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_6_OFFSET)
#define BL602_SF_AES_KEY_R1_7     (BL602_SF_BASE + BL602_SF_AES_KEY_R1_7_OFFSET)
#define BL602_SF_AES_IV_R1_W0     (BL602_SF_BASE + BL602_SF_AES_IV_R1_W0_OFFSET)
#define BL602_SF_AES_IV_R1_W1     (BL602_SF_BASE + BL602_SF_AES_IV_R1_W1_OFFSET)
#define BL602_SF_AES_IV_R1_W2     (BL602_SF_BASE + BL602_SF_AES_IV_R1_W2_OFFSET)
#define BL602_SF_AES_IV_R1_W3     (BL602_SF_BASE + BL602_SF_AES_IV_R1_W3_OFFSET)
#define BL602_SF_AES_R1           (BL602_SF_BASE + BL602_SF_AES_R1_OFFSET)
#define BL602_SF_AES_KEY_R2_0     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_0_OFFSET)
#define BL602_SF_AES_KEY_R2_1     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_1_OFFSET)
#define BL602_SF_AES_KEY_R2_2     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_2_OFFSET)
#define BL602_SF_AES_KEY_R2_3     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_3_OFFSET)
#define BL602_SF_AES_KEY_R2_4     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_4_OFFSET)
#define BL602_SF_AES_KEY_R2_5     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_5_OFFSET)
#define BL602_SF_AES_KEY_R2_6     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_6_OFFSET)
#define BL602_SF_AES_KEY_R2_7     (BL602_SF_BASE + BL602_SF_AES_KEY_R2_7_OFFSET)
#define BL602_SF_AES_IV_R2_W0     (BL602_SF_BASE + BL602_SF_AES_IV_R2_W0_OFFSET)
#define BL602_SF_AES_IV_R2_W1     (BL602_SF_BASE + BL602_SF_AES_IV_R2_W1_OFFSET)
#define BL602_SF_AES_IV_R2_W2     (BL602_SF_BASE + BL602_SF_AES_IV_R2_W2_OFFSET)
#define BL602_SF_AES_IV_R2_W3     (BL602_SF_BASE + BL602_SF_AES_IV_R2_W3_OFFSET)
#define BL602_SF_AES_R2           (BL602_SF_BASE + BL602_SF_AES_R2_OFFSET)
#define BL602_SF_ID0_OFFSET       (BL602_SF_BASE + BL602_SF_ID0_OFFSET_OFFSET)
#define BL602_SF_ID1_OFFSET       (BL602_SF_BASE + BL602_SF_ID1_OFFSET_OFFSET)

/* Register bit definitions *************************************************/

#define SF_0_SF_ID_SHIFT                       (24)
#define SF_0_SF_ID_MASK                        (0xff << SF_0_SF_ID_SHIFT)
#define SF_0_SF_AES_IV_ENDIAN                  (1 << 23)
#define SF_0_SF_AES_KEY_ENDIAN                 (1 << 22)
#define SF_0_SF_AES_CTR_PLUS_EN                (1 << 21)
#define SF_0_SF_AES_DOUT_ENDIAN                (1 << 20)
#define SF_0_SF_AES_DLY_MODE                   (1 << 19)
#define SF_0_SF_IF_INT_SET                     (1 << 18)
#define SF_0_SF_IF_INT_CLR                     (1 << 17)
#define SF_0_SF_IF_INT                         (1 << 16)
#define SF_0_SF_IF_READ_DLY_EN                 (1 << 11)
#define SF_0_SF_IF_READ_DLY_N_SHIFT            (8)
#define SF_0_SF_IF_READ_DLY_N_MASK             (0x07 << SF_0_SF_IF_READ_DLY_N_SHIFT)
#define SF_0_SF_CLK_SAHB_SRAM_SEL              (1 << 5)
#define SF_0_SF_CLK_OUT_INV_SEL                (1 << 4)
#define SF_0_SF_CLK_OUT_GATE_EN                (1 << 3)
#define SF_0_SF_CLK_SF_RX_INV_SEL              (1 << 2)

#define SF_1_SF_AHB2SRAM_EN                    (1 << 31)
#define SF_1_SF_AHB2SIF_EN                     (1 << 30)
#define SF_1_IF_EN                             (1 << 29)
#define SF_1_IF_FN_SEL                         (1 << 28)
#define SF_1_SF_AHB2SIF_STOP                   (1 << 27)
#define SF_1_SF_AHB2SIF_STOPPED                (1 << 26)
#define SF_1_IF_REG_WP                         (1 << 25)
#define SF_1_IF_REG_HOLD                       (1 << 24)
#define SF_1_IF_0_ACK_LAT_SHIFT                (20)
#define SF_1_IF_0_ACK_LAT_MASK                 (0x07 << SF_1_IF_0_ACK_LAT_SHIFT)
#define SF_1_IF_SR_INT_SET                     (1 << 18)
#define SF_1_IF_SR_INT_EN                      (1 << 17)
#define SF_1_IF_SR_INT                         (1 << 16)
#define SF_1_IF_SR_PAT_SHIFT                   (8)
#define SF_1_IF_SR_PAT_MASK                    (0xff << SF_1_IF_SR_PAT_SHIFT)
#define SF_1_IF_SR_PAT_MASK_MASK               (0xff)

#define SF_IF_SAHB_0_SF_IF_0_QPI_MODE_EN            (1 << 31)
#define SF_IF_SAHB_0_SF_IF_0_SPI_MODE_SHIFT         (28)
#define SF_IF_SAHB_0_SF_IF_0_SPI_MODE_MASK          (0x07 << SF_IF_SAHB_0_SF_IF_0_SPI_MODE_SHIFT)
#define SF_IF_SAHB_0_SF_IF_0_CMD_EN                 (1 << 27)
#define SF_IF_SAHB_0_SF_IF_0_ADR_EN                 (1 << 26)
#define SF_IF_SAHB_0_SF_IF_0_DMY_EN                 (1 << 25)
#define SF_IF_SAHB_0_SF_IF_0_DAT_EN                 (1 << 24)
#define SF_IF_SAHB_0_SF_IF_0_DAT_RW                 (1 << 23)
#define SF_IF_SAHB_0_SF_IF_0_CMD_BYTE_SHIFT         (20)
#define SF_IF_SAHB_0_SF_IF_0_CMD_BYTE_MASK          (0x07 << SF_IF_SAHB_0_SF_IF_0_CMD_BYTE_SHIFT)
#define SF_IF_SAHB_0_SF_IF_0_ADR_BYTE_SHIFT         (17)
#define SF_IF_SAHB_0_SF_IF_0_ADR_BYTE_MASK          (0x07 << SF_IF_SAHB_0_SF_IF_0_ADR_BYTE_SHIFT)
#define SF_IF_SAHB_0_SF_IF_0_DMY_BYTE_SHIFT         (12)
#define SF_IF_SAHB_0_SF_IF_0_DMY_BYTE_MASK          (0x1f << SF_IF_SAHB_0_SF_IF_0_DMY_BYTE_SHIFT)
#define SF_IF_SAHB_0_SF_IF_0_DAT_BYTE_SHIFT         (2)
#define SF_IF_SAHB_0_SF_IF_0_DAT_BYTE_MASK          (0x3ff << SF_IF_SAHB_0_SF_IF_0_DAT_BYTE_SHIFT)
#define SF_IF_SAHB_0_SF_IF_0_TRIG                   (1 << 1)
#define SF_IF_SAHB_0_SF_IF_BUSY                     (1 << 0)

#define SF_IF_IAHB_0_SF_IF_1_QPI_MODE_EN            (1 << 31)
#define SF_IF_IAHB_0_SF_IF_1_SPI_MODE_SHIFT         (28)
#define SF_IF_IAHB_0_SF_IF_1_SPI_MODE_MASK          (0x07 << SF_IF_IAHB_0_SF_IF_1_SPI_MODE_SHIFT)
#define SF_IF_IAHB_0_SF_IF_1_CMD_EN                 (1 << 27)
#define SF_IF_IAHB_0_SF_IF_1_ADR_EN                 (1 << 26)
#define SF_IF_IAHB_0_SF_IF_1_DMY_EN                 (1 << 25)
#define SF_IF_IAHB_0_SF_IF_1_DAT_EN                 (1 << 24)
#define SF_IF_IAHB_0_SF_IF_1_DAT_RW                 (1 << 23)
#define SF_IF_IAHB_0_SF_IF_1_CMD_BYTE_SHIFT         (20)
#define SF_IF_IAHB_0_SF_IF_1_CMD_BYTE_MASK          (0x07 << SF_IF_IAHB_0_SF_IF_1_CMD_BYTE_SHIFT)
#define SF_IF_IAHB_0_SF_IF_1_ADR_BYTE_SHIFT         (17)
#define SF_IF_IAHB_0_SF_IF_1_ADR_BYTE_MASK          (0x07 << SF_IF_IAHB_0_SF_IF_1_ADR_BYTE_SHIFT)
#define SF_IF_IAHB_0_SF_IF_1_DMY_BYTE_SHIFT         (12)
#define SF_IF_IAHB_0_SF_IF_1_DMY_BYTE_MASK          (0x1f << SF_IF_IAHB_0_SF_IF_1_DMY_BYTE_SHIFT)

#define SF_AES_SF_AES_STATUS_SHIFT                  (5)
#define SF_AES_SF_AES_STATUS_MASK                   (0x7ffffff << SF_AES_SF_AES_STATUS_SHIFT)
#define SF_AES_SF_AES_PREF_BUSY                     (1 << 4)
#define SF_AES_SF_AES_PREF_TRIG                     (1 << 3)
#define SF_AES_SF_AES_MODE_SHIFT                    (1)
#define SF_AES_SF_AES_MODE_MASK                     (0x03 << SF_AES_SF_AES_MODE_SHIFT)
#define SF_AES_SF_AES_EN                            (1 << 0)

#define SF_IF_IO_DLY_0_SF_DQS_DO_DLY_SEL_SHIFT      (30)
#define SF_IF_IO_DLY_0_SF_DQS_DO_DLY_SEL_MASK       (0x03 << SF_IF_IO_DLY_0_SF_DQS_DO_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_0_SF_DQS_DI_DLY_SEL_SHIFT      (28)
#define SF_IF_IO_DLY_0_SF_DQS_DI_DLY_SEL_MASK       (0x03 << SF_IF_IO_DLY_0_SF_DQS_DI_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_0_SF_DQS_OE_DLY_SEL_SHIFT      (26)
#define SF_IF_IO_DLY_0_SF_DQS_OE_DLY_SEL_MASK       (0x03 << SF_IF_IO_DLY_0_SF_DQS_OE_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_0_SF_CLK_OUT_DLY_SEL_SHIFT     (8)
#define SF_IF_IO_DLY_0_SF_CLK_OUT_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_0_SF_CLK_OUT_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_0_SF_CS_DLY_SEL_MASK           (0x03)

#define SF_IF_IO_DLY_1_SF_IO_0_DO_DLY_SEL_SHIFT     (16)
#define SF_IF_IO_DLY_1_SF_IO_0_DO_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_1_SF_IO_0_DO_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_1_SF_IO_0_DI_DLY_SEL_SHIFT     (8)
#define SF_IF_IO_DLY_1_SF_IO_0_DI_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_1_SF_IO_0_DI_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_1_SF_IO_0_OE_DLY_SEL_MASK      (0x03)

#define SF_IF_IO_DLY_2_SF_IO_1_DO_DLY_SEL_SHIFT     (16)
#define SF_IF_IO_DLY_2_SF_IO_1_DO_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_2_SF_IO_1_DO_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_2_SF_IO_1_DI_DLY_SEL_SHIFT     (8)
#define SF_IF_IO_DLY_2_SF_IO_1_DI_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_2_SF_IO_1_DI_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_2_SF_IO_1_OE_DLY_SEL_MASK      (0x03)

#define SF_IF_IO_DLY_3_SF_IO_2_DO_DLY_SEL_SHIFT     (16)
#define SF_IF_IO_DLY_3_SF_IO_2_DO_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_3_SF_IO_2_DO_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_3_SF_IO_2_DI_DLY_SEL_SHIFT     (8)
#define SF_IF_IO_DLY_3_SF_IO_2_DI_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_3_SF_IO_2_DI_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_3_SF_IO_2_OE_DLY_SEL_MASK      (0x03)

#define SF_IF_IO_DLY_4_SF_IO_3_DO_DLY_SEL_SHIFT     (16)
#define SF_IF_IO_DLY_4_SF_IO_3_DO_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_4_SF_IO_3_DO_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_4_SF_IO_3_DI_DLY_SEL_SHIFT     (8)
#define SF_IF_IO_DLY_4_SF_IO_3_DI_DLY_SEL_MASK      (0x03 << SF_IF_IO_DLY_4_SF_IO_3_DI_DLY_SEL_SHIFT)
#define SF_IF_IO_DLY_4_SF_IO_3_OE_DLY_SEL_MASK      (0x03)

#define SF2_IF_IO_DLY_0_SF2_DQS_DO_DLY_SEL_SHIFT    (30)
#define SF2_IF_IO_DLY_0_SF2_DQS_DO_DLY_SEL_MASK     (0x03 << SF2_IF_IO_DLY_0_SF2_DQS_DO_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_0_SF2_DQS_DI_DLY_SEL_SHIFT    (28)
#define SF2_IF_IO_DLY_0_SF2_DQS_DI_DLY_SEL_MASK     (0x03 << SF2_IF_IO_DLY_0_SF2_DQS_DI_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_0_SF2_DQS_OE_DLY_SEL_SHIFT    (26)
#define SF2_IF_IO_DLY_0_SF2_DQS_OE_DLY_SEL_MASK     (0x03 << SF2_IF_IO_DLY_0_SF2_DQS_OE_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_0_SF2_CLK_OUT_DLY_SEL_SHIFT   (8)
#define SF2_IF_IO_DLY_0_SF2_CLK_OUT_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_0_SF2_CLK_OUT_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_0_SF2_CS_DLY_SEL_MASK         (0x03)

#define SF2_IF_IO_DLY_1_SF2_IO_0_DO_DLY_SEL_SHIFT   (16)
#define SF2_IF_IO_DLY_1_SF2_IO_0_DO_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_1_SF2_IO_0_DO_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_1_SF2_IO_0_DI_DLY_SEL_SHIFT   (8)
#define SF2_IF_IO_DLY_1_SF2_IO_0_DI_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_1_SF2_IO_0_DI_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_1_SF2_IO_0_OE_DLY_SEL_MASK    (0x03)

#define SF2_IF_IO_DLY_2_SF2_IO_1_DO_DLY_SEL_SHIFT   (16)
#define SF2_IF_IO_DLY_2_SF2_IO_1_DO_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_2_SF2_IO_1_DO_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_2_SF2_IO_1_DI_DLY_SEL_SHIFT   (8)
#define SF2_IF_IO_DLY_2_SF2_IO_1_DI_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_2_SF2_IO_1_DI_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_2_SF2_IO_1_OE_DLY_SEL_MASK    (0x03)

#define SF2_IF_IO_DLY_3_SF2_IO_2_DO_DLY_SEL_SHIFT   (16)
#define SF2_IF_IO_DLY_3_SF2_IO_2_DO_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_3_SF2_IO_2_DO_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_3_SF2_IO_2_DI_DLY_SEL_SHIFT   (8)
#define SF2_IF_IO_DLY_3_SF2_IO_2_DI_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_3_SF2_IO_2_DI_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_3_SF2_IO_2_OE_DLY_SEL_MASK    (0x03)

#define SF2_IF_IO_DLY_4_SF2_IO_3_DO_DLY_SEL_SHIFT   (16)
#define SF2_IF_IO_DLY_4_SF2_IO_3_DO_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_4_SF2_IO_3_DO_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_4_SF2_IO_3_DI_DLY_SEL_SHIFT   (8)
#define SF2_IF_IO_DLY_4_SF2_IO_3_DI_DLY_SEL_MASK    (0x03 << SF2_IF_IO_DLY_4_SF2_IO_3_DI_DLY_SEL_SHIFT)
#define SF2_IF_IO_DLY_4_SF2_IO_3_OE_DLY_SEL_MASK    (0x03)

#define SF3_IF_IO_DLY_0_SF3_DQS_DO_DLY_SEL_SHIFT    (30)
#define SF3_IF_IO_DLY_0_SF3_DQS_DO_DLY_SEL_MASK     (0x03 << SF3_IF_IO_DLY_0_SF3_DQS_DO_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_0_SF3_DQS_DI_DLY_SEL_SHIFT    (28)
#define SF3_IF_IO_DLY_0_SF3_DQS_DI_DLY_SEL_MASK     (0x03 << SF3_IF_IO_DLY_0_SF3_DQS_DI_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_0_SF3_DQS_OE_DLY_SEL_SHIFT    (26)
#define SF3_IF_IO_DLY_0_SF3_DQS_OE_DLY_SEL_MASK     (0x03 << SF3_IF_IO_DLY_0_SF3_DQS_OE_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_0_SF3_CLK_OUT_DLY_SEL_SHIFT   (8)
#define SF3_IF_IO_DLY_0_SF3_CLK_OUT_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_0_SF3_CLK_OUT_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_0_SF3_CS_DLY_SEL_MASK         (0x03)

#define SF3_IF_IO_DLY_1_SF3_IO_0_DO_DLY_SEL_SHIFT   (16)
#define SF3_IF_IO_DLY_1_SF3_IO_0_DO_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_1_SF3_IO_0_DO_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_1_SF3_IO_0_DI_DLY_SEL_SHIFT   (8)
#define SF3_IF_IO_DLY_1_SF3_IO_0_DI_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_1_SF3_IO_0_DI_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_1_SF3_IO_0_OE_DLY_SEL_MASK    (0x03)

#define SF3_IF_IO_DLY_2_SF3_IO_1_DO_DLY_SEL_SHIFT   (16)
#define SF3_IF_IO_DLY_2_SF3_IO_1_DO_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_2_SF3_IO_1_DO_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_2_SF3_IO_1_DI_DLY_SEL_SHIFT   (8)
#define SF3_IF_IO_DLY_2_SF3_IO_1_DI_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_2_SF3_IO_1_DI_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_2_SF3_IO_1_OE_DLY_SEL_MASK    (0x03)

#define SF3_IF_IO_DLY_3_SF3_IO_2_DO_DLY_SEL_SHIFT   (16)
#define SF3_IF_IO_DLY_3_SF3_IO_2_DO_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_3_SF3_IO_2_DO_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_3_SF3_IO_2_DI_DLY_SEL_SHIFT   (8)
#define SF3_IF_IO_DLY_3_SF3_IO_2_DI_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_3_SF3_IO_2_DI_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_3_SF3_IO_2_OE_DLY_SEL_MASK    (0x03)

#define SF3_IF_IO_DLY_4_SF3_IO_3_DO_DLY_SEL_SHIFT   (16)
#define SF3_IF_IO_DLY_4_SF3_IO_3_DO_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_4_SF3_IO_3_DO_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_4_SF3_IO_3_DI_DLY_SEL_SHIFT   (8)
#define SF3_IF_IO_DLY_4_SF3_IO_3_DI_DLY_SEL_MASK    (0x03 << SF3_IF_IO_DLY_4_SF3_IO_3_DI_DLY_SEL_SHIFT)
#define SF3_IF_IO_DLY_4_SF3_IO_3_OE_DLY_SEL_MASK    (0x03)

#define SF_2_IF_DQS_EN                         (1 << 5)
#define SF_2_IF_DTR_EN                         (1 << 4)
#define SF_2_IF_PAD_SEL_LOCK                   (1 << 3)
#define SF_2_IF_PAD_SEL_MASK                   (0x03)

#define SF_3_IF_1_ACK_LAT_SHIFT                (29)
#define SF_3_IF_1_ACK_LAT_MASK                 (0x07 << SF_3_IF_1_ACK_LAT_SHIFT)
#define SF_3_SF_CMDS_WRAP_MODE                 (1 << 10)
#define SF_3_SF_CMDS_WRAP_Q_INI                (1 << 9)
#define SF_3_SF_CMDS_BT_EN                     (1 << 8)
#define SF_3_SF_CMDS_BT_DLY_SHIFT              (5)
#define SF_3_SF_CMDS_BT_DLY_MASK               (0x07 << SF_3_SF_CMDS_BT_DLY_SHIFT)
#define SF_3_SF_CMDS_EN                        (1 << 4)
#define SF_3_SF_CMDS_WRAP_LEN_MASK             (0x0f)

#define SF_IF_IAHB_3_IF_2_QPI_MODE_EN          (1 << 31)
#define SF_IF_IAHB_3_IF_2_SPI_MODE_SHIFT       (28)
#define SF_IF_IAHB_3_IF_2_SPI_MODE_MASK        (0x07 << SF_IF_IAHB_3_IF_2_SPI_MODE_SHIFT)
#define SF_IF_IAHB_3_IF_2_CMD_EN               (1 << 27)
#define SF_IF_IAHB_3_IF_2_ADR_EN               (1 << 26)
#define SF_IF_IAHB_3_IF_2_DMY_EN               (1 << 25)
#define SF_IF_IAHB_3_IF_2_DAT_EN               (1 << 24)
#define SF_IF_IAHB_3_IF_2_DAT_RW               (1 << 23)
#define SF_IF_IAHB_3_IF_2_CMD_BYTE_SHIFT       (20)
#define SF_IF_IAHB_3_IF_2_CMD_BYTE_MASK        (0x07 << SF_IF_IAHB_3_IF_2_CMD_BYTE_SHIFT)
#define SF_IF_IAHB_3_IF_2_ADR_BYTE_SHIFT       (17)
#define SF_IF_IAHB_3_IF_2_ADR_BYTE_MASK        (0x07 << SF_IF_IAHB_3_IF_2_ADR_BYTE_SHIFT)
#define SF_IF_IAHB_3_IF_2_DMY_BYTE_SHIFT       (12)
#define SF_IF_IAHB_3_IF_2_DMY_BYTE_MASK        (0x1f << SF_IF_IAHB_3_IF_2_DMY_BYTE_SHIFT)

#define SF_IF_IAHB_6_IF_3_QPI_MODE_EN          (1 << 31)
#define SF_IF_IAHB_6_IF_3_SPI_MODE_SHIFT       (28)
#define SF_IF_IAHB_6_IF_3_SPI_MODE_MASK        (0x07 << SF_IF_IAHB_6_IF_3_SPI_MODE_SHIFT)
#define SF_IF_IAHB_6_IF_3_CMD_BYTE_SHIFT       (20)
#define SF_IF_IAHB_6_IF_3_CMD_BYTE_MASK        (0x07 << SF_IF_IAHB_6_IF_3_CMD_BYTE_SHIFT)

#define SF_PROT_EN_RD_SF_DBG_DIS               (1 << 31)
#define SF_PROT_EN_RD_SF_IF_0_TRIG_WR_LOCK     (1 << 30)
#define SF_PROT_EN_RD_SF_ID1_EN_RD             (1 << 2)
#define SF_PROT_EN_RD_SF_ID0_EN_RD             (1 << 1)
#define SF_PROT_EN_RD                          (1 << 0)

#define SF_PROT_EN_SF_ID1_EN                   (1 << 2)
#define SF_PROT_EN_SF_ID0_EN                   (1 << 1)
#define SF_PROT_EN                             (1 << 0)

#define SF_AES_CFG_R0_SF_AES_REGION_R0_LOCK         (1 << 31)
#define SF_AES_CFG_R0_SF_AES_REGION_R0_EN           (1 << 30)
#define SF_AES_CFG_R0_SF_AES_REGION_R0_HW_KEY_EN    (1 << 29)
#define SF_AES_CFG_R0_SF_AES_REGION_R0_START_SHIFT  (14)
#define SF_AES_CFG_R0_SF_AES_REGION_R0_START_MASK   (0x3fff << SF_AES_CFG_R0_SF_AES_REGION_R0_START_SHIFT)
#define SF_AES_CFG_R0_SF_AES_REGION_R0_END_MASK     (0x3fff)

#define SF_AES_R1_SF_AES_R1_LOCK                    (1 << 31)
#define SF_AES_R1_SF_AES_R1_EN                      (1 << 30)
#define SF_AES_R1_SF_AES_R1_HW_KEY_EN               (1 << 29)
#define SF_AES_R1_SF_AES_R1_START_SHIFT             (14)
#define SF_AES_R1_SF_AES_R1_START_MASK              (0x3fff << SF_AES_R1_SF_AES_R1_START_SHIFT)
#define SF_AES_R1_SF_AES_R1_END_MASK                (0x3fff)

#define SF_AES_R2_SF_AES_R2_LOCK                    (1 << 31)
#define SF_AES_R2_SF_AES_R2_EN                      (1 << 30)
#define SF_AES_R2_SF_AES_R2_HW_KEY_EN               (1 << 29)
#define SF_AES_R2_SF_AES_R2_START_SHIFT             (14)
#define SF_AES_R2_SF_AES_R2_START_MASK              (0x3fff << SF_AES_R2_SF_AES_R2_START_SHIFT)
#define SF_AES_R2_SF_AES_R2_END_MASK                (0x3fff)

#define SF_ID0_OFFSET_MASK                          (0xffffff)

#define SF_ID1_OFFSET_MASK                          (0xffffff)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SF_H */
