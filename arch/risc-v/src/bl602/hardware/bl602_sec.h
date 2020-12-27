/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_sec.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SEC_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_SD_CHIP_ID_LOW_OFFSET        0x000000  /* sd_chip_id_low */
#define BL602_SD_CHIP_ID_HIGH_OFFSET       0x000004  /* sd_chip_id_high */
#define BL602_SD_WIFI_MAC_LOW_OFFSET       0x000008  /* sd_wifi_mac_low */
#define BL602_SD_WIFI_MAC_HIGH_OFFSET      0x00000c  /* sd_wifi_mac_high */
#define BL602_SD_DBG_PWD_LOW_OFFSET        0x000010  /* sd_dbg_pwd_low */
#define BL602_SD_DBG_PWD_HIGH_OFFSET       0x000014  /* sd_dbg_pwd_high */
#define BL602_SD_STATUS_OFFSET             0x000018  /* sd_status */
#define BL602_SD_DBG_RESERVED_OFFSET       0x00001c  /* sd_dbg_reserved */

#define BL602_SE_SHA_CTRL_OFFSET           0x000000  /* se_sha_0_ctrl */
#define BL602_SE_SHA_MSA_OFFSET            0x000004  /* se_sha_0_msa */
#define BL602_SE_SHA_STATUS_OFFSET         0x000008  /* se_sha_0_status */
#define BL602_SE_SHA_ENDIAN_OFFSET         0x00000c  /* se_sha_0_endian */
#define BL602_SE_SHA_HASH_L_0_OFFSET       0x000010  /* se_sha_0_hash_l_0 */
#define BL602_SE_SHA_HASH_L_1_OFFSET       0x000014  /* se_sha_0_hash_l_1 */
#define BL602_SE_SHA_HASH_L_2_OFFSET       0x000018  /* se_sha_0_hash_l_2 */
#define BL602_SE_SHA_HASH_L_3_OFFSET       0x00001c  /* se_sha_0_hash_l_3 */
#define BL602_SE_SHA_HASH_L_4_OFFSET       0x000020  /* se_sha_0_hash_l_4 */
#define BL602_SE_SHA_HASH_L_5_OFFSET       0x000024  /* se_sha_0_hash_l_5 */
#define BL602_SE_SHA_HASH_L_6_OFFSET       0x000028  /* se_sha_0_hash_l_6 */
#define BL602_SE_SHA_HASH_L_7_OFFSET       0x00002c  /* se_sha_0_hash_l_7 */
#define BL602_SE_SHA_HASH_H_0_OFFSET       0x000030  /* se_sha_0_hash_h_0 */
#define BL602_SE_SHA_HASH_H_1_OFFSET       0x000034  /* se_sha_0_hash_h_1 */
#define BL602_SE_SHA_HASH_H_2_OFFSET       0x000038  /* se_sha_0_hash_h_2 */
#define BL602_SE_SHA_HASH_H_3_OFFSET       0x00003c  /* se_sha_0_hash_h_3 */
#define BL602_SE_SHA_HASH_H_4_OFFSET       0x000040  /* se_sha_0_hash_h_4 */
#define BL602_SE_SHA_HASH_H_5_OFFSET       0x000044  /* se_sha_0_hash_h_5 */
#define BL602_SE_SHA_HASH_H_6_OFFSET       0x000048  /* se_sha_0_hash_h_6 */
#define BL602_SE_SHA_HASH_H_7_OFFSET       0x00004c  /* se_sha_0_hash_h_7 */
#define BL602_SE_SHA_LINK_OFFSET           0x000050  /* se_sha_0_link */
#define BL602_SE_SHA_CTRL_PROT_OFFSET      0x0000fc  /* se_sha_0_ctrl_prot */
#define BL602_SE_AES_CTRL_OFFSET           0x000100  /* se_aes_0_ctrl */
#define BL602_SE_AES_MSA_OFFSET            0x000104  /* se_aes_0_msa */
#define BL602_SE_AES_MDA_OFFSET            0x000108  /* se_aes_0_mda */
#define BL602_SE_AES_STATUS_OFFSET         0x00010c  /* se_aes_0_status */
#define BL602_SE_AES_IV_0_OFFSET           0x000110  /* se_aes_0_iv_0 */
#define BL602_SE_AES_IV_1_OFFSET           0x000114  /* se_aes_0_iv_1 */
#define BL602_SE_AES_IV_2_OFFSET           0x000118  /* se_aes_0_iv_2 */
#define BL602_SE_AES_IV_3_OFFSET           0x00011c  /* se_aes_0_iv_3 */
#define BL602_SE_AES_KEY_0_OFFSET          0x000120  /* se_aes_0_key_0 */
#define BL602_SE_AES_KEY_1_OFFSET          0x000124  /* se_aes_0_key_1 */
#define BL602_SE_AES_KEY_2_OFFSET          0x000128  /* se_aes_0_key_2 */
#define BL602_SE_AES_KEY_3_OFFSET          0x00012c  /* se_aes_0_key_3 */
#define BL602_SE_AES_KEY_4_OFFSET          0x000130  /* se_aes_0_key_4 */
#define BL602_SE_AES_KEY_5_OFFSET          0x000134  /* se_aes_0_key_5 */
#define BL602_SE_AES_KEY_6_OFFSET          0x000138  /* se_aes_0_key_6 */
#define BL602_SE_AES_KEY_7_OFFSET          0x00013c  /* se_aes_0_key_7 */
#define BL602_SE_AES_KEY_SEL_0_OFFSET      0x000140  /* se_aes_0_key_sel_0 */
#define BL602_SE_AES_KEY_SEL_1_OFFSET      0x000144  /* se_aes_0_key_sel_1 */
#define BL602_SE_AES_ENDIAN_OFFSET         0x000148  /* se_aes_0_endian */
#define BL602_SE_AES_SBOOT_OFFSET          0x00014c  /* se_aes_0_sboot */
#define BL602_SE_AES_LINK_OFFSET           0x000150  /* se_aes_0_link */
#define BL602_SE_AES_CTRL_PROT_OFFSET      0x0001fc  /* se_aes_0_ctrl_prot */
#define BL602_SE_TRNG_CTRL_0_OFFSET        0x000200  /* se_trng_0_ctrl_0 */
#define BL602_SE_TRNG_STATUS_OFFSET        0x000204  /* se_trng_0_status */
#define BL602_SE_TRNG_DOUT_0_OFFSET        0x000208  /* se_trng_0_dout_0 */
#define BL602_SE_TRNG_DOUT_1_OFFSET        0x00020c  /* se_trng_0_dout_1 */
#define BL602_SE_TRNG_DOUT_2_OFFSET        0x000210  /* se_trng_0_dout_2 */
#define BL602_SE_TRNG_DOUT_3_OFFSET        0x000214  /* se_trng_0_dout_3 */
#define BL602_SE_TRNG_DOUT_4_OFFSET        0x000218  /* se_trng_0_dout_4 */
#define BL602_SE_TRNG_DOUT_5_OFFSET        0x00021c  /* se_trng_0_dout_5 */
#define BL602_SE_TRNG_DOUT_6_OFFSET        0x000220  /* se_trng_0_dout_6 */
#define BL602_SE_TRNG_DOUT_7_OFFSET        0x000224  /* se_trng_0_dout_7 */
#define BL602_SE_TRNG_TEST_OFFSET          0x000228  /* se_trng_0_test */
#define BL602_SE_TRNG_CTRL_1_OFFSET        0x00022c  /* se_trng_0_ctrl_1 */
#define BL602_SE_TRNG_CTRL_2_OFFSET        0x000230  /* se_trng_0_ctrl_2 */
#define BL602_SE_TRNG_CTRL_3_OFFSET        0x000234  /* se_trng_0_ctrl_3 */
#define BL602_SE_TRNG_TEST_OUT_0_OFFSET    0x000240  /* se_trng_0_test_out_0 */
#define BL602_SE_TRNG_TEST_OUT_1_OFFSET    0x000244  /* se_trng_0_test_out_1 */
#define BL602_SE_TRNG_TEST_OUT_2_OFFSET    0x000248  /* se_trng_0_test_out_2 */
#define BL602_SE_TRNG_TEST_OUT_3_OFFSET    0x00024c  /* se_trng_0_test_out_3 */
#define BL602_SE_TRNG_CTRL_PROT_OFFSET     0x0002fc  /* se_trng_0_ctrl_prot */
#define BL602_SE_PKA_CTRL_0_OFFSET         0x000300  /* se_pka_0_ctrl_0 */
#define BL602_SE_PKA_SEED_OFFSET           0x00030c  /* se_pka_0_seed */
#define BL602_SE_PKA_CTRL_1_OFFSET         0x000310  /* se_pka_0_ctrl_1 */
#define BL602_SE_PKA_RW_OFFSET             0x000340  /* se_pka_0_rw */
#define BL602_SE_PKA_RW_BURST_OFFSET       0x000360  /* se_pka_0_rw_burst */
#define BL602_SE_PKA_CTRL_PROT_OFFSET      0x0003fc  /* se_pka_0_ctrl_prot */
#define BL602_SE_CDET_CTRL_0_OFFSET        0x000400  /* se_cdet_0_ctrl_0 */
#define BL602_SE_CDET_CTRL_1_OFFSET        0x000404  /* se_cdet_0_ctrl_1 */
#define BL602_SE_CDET_CTRL_PROT_OFFSET     0x0004fc  /* se_cdet_0_ctrl_prot */
#define BL602_SE_GMAC_CTRL_0_OFFSET        0x000500  /* se_gmac_0_ctrl_0 */
#define BL602_SE_GMAC_LCA_OFFSET           0x000504  /* se_gmac_0_lca */
#define BL602_SE_GMAC_STATUS_OFFSET        0x000508  /* se_gmac_0_status */
#define BL602_SE_GMAC_CTRL_PROT_OFFSET     0x0005fc  /* se_gmac_0_ctrl_prot */
#define BL602_SE_CTRL_PROT_RD_OFFSET       0x000f00  /* se_ctrl_prot_rd */
#define BL602_SE_CTRL_RESERVED_0_OFFSET    0x000f04  /* se_ctrl_reserved_0 */
#define BL602_SE_CTRL_RESERVED_1_OFFSET    0x000f08  /* se_ctrl_reserved_1 */
#define BL602_SE_CTRL_RESERVED_2_OFFSET    0x000f0c  /* se_ctrl_reserved_2 */

/* Register definitions *****************************************************/

#define BL602_SD_CHIP_ID_LOW        (BL602_SEC_DBG_BASE + BL602_SD_CHIP_ID_LOW_OFFSET)
#define BL602_SD_CHIP_ID_HIGH       (BL602_SEC_DBG_BASE + BL602_SD_CHIP_ID_HIGH_OFFSET)
#define BL602_SD_WIFI_MAC_LOW       (BL602_SEC_DBG_BASE + BL602_SD_WIFI_MAC_LOW_OFFSET)
#define BL602_SD_WIFI_MAC_HIGH      (BL602_SEC_DBG_BASE + BL602_SD_WIFI_MAC_HIGH_OFFSET)
#define BL602_SD_DBG_PWD_LOW        (BL602_SEC_DBG_BASE + BL602_SD_DBG_PWD_LOW_OFFSET)
#define BL602_SD_DBG_PWD_HIGH       (BL602_SEC_DBG_BASE + BL602_SD_DBG_PWD_HIGH_OFFSET)
#define BL602_SD_STATUS             (BL602_SEC_DBG_BASE + BL602_SD_STATUS_OFFSET)
#define BL602_SD_DBG_RESERVED       (BL602_SEC_DBG_BASE + BL602_SD_DBG_RESERVED_OFFSET)

#define BL602_SE_SHA_CTRL           (BL602_SEC_ENG_BASE + BL602_SE_SHA_CTRL_OFFSET)
#define BL602_SE_SHA_MSA            (BL602_SEC_ENG_BASE + BL602_SE_SHA_MSA_OFFSET)
#define BL602_SE_SHA_STATUS         (BL602_SEC_ENG_BASE + BL602_SE_SHA_STATUS_OFFSET)
#define BL602_SE_SHA_ENDIAN         (BL602_SEC_ENG_BASE + BL602_SE_SHA_ENDIAN_OFFSET)
#define BL602_SE_SHA_HASH_L_0       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_0_OFFSET)
#define BL602_SE_SHA_HASH_L_1       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_1_OFFSET)
#define BL602_SE_SHA_HASH_L_2       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_2_OFFSET)
#define BL602_SE_SHA_HASH_L_3       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_3_OFFSET)
#define BL602_SE_SHA_HASH_L_4       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_4_OFFSET)
#define BL602_SE_SHA_HASH_L_5       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_5_OFFSET)
#define BL602_SE_SHA_HASH_L_6       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_6_OFFSET)
#define BL602_SE_SHA_HASH_L_7       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_L_7_OFFSET)
#define BL602_SE_SHA_HASH_H_0       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_0_OFFSET)
#define BL602_SE_SHA_HASH_H_1       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_1_OFFSET)
#define BL602_SE_SHA_HASH_H_2       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_2_OFFSET)
#define BL602_SE_SHA_HASH_H_3       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_3_OFFSET)
#define BL602_SE_SHA_HASH_H_4       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_4_OFFSET)
#define BL602_SE_SHA_HASH_H_5       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_5_OFFSET)
#define BL602_SE_SHA_HASH_H_6       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_6_OFFSET)
#define BL602_SE_SHA_HASH_H_7       (BL602_SEC_ENG_BASE + BL602_SE_SHA_HASH_H_7_OFFSET)
#define BL602_SE_SHA_LINK           (BL602_SEC_ENG_BASE + BL602_SE_SHA_LINK_OFFSET)
#define BL602_SE_SHA_CTRL_PROT      (BL602_SEC_ENG_BASE + BL602_SE_SHA_CTRL_PROT_OFFSET)
#define BL602_SE_AES_CTRL           (BL602_SEC_ENG_BASE + BL602_SE_AES_CTRL_OFFSET)
#define BL602_SE_AES_MSA            (BL602_SEC_ENG_BASE + BL602_SE_AES_MSA_OFFSET)
#define BL602_SE_AES_MDA            (BL602_SEC_ENG_BASE + BL602_SE_AES_MDA_OFFSET)
#define BL602_SE_AES_STATUS         (BL602_SEC_ENG_BASE + BL602_SE_AES_STATUS_OFFSET)
#define BL602_SE_AES_IV_0           (BL602_SEC_ENG_BASE + BL602_SE_AES_IV_0_OFFSET)
#define BL602_SE_AES_IV_1           (BL602_SEC_ENG_BASE + BL602_SE_AES_IV_1_OFFSET)
#define BL602_SE_AES_IV_2           (BL602_SEC_ENG_BASE + BL602_SE_AES_IV_2_OFFSET)
#define BL602_SE_AES_IV_3           (BL602_SEC_ENG_BASE + BL602_SE_AES_IV_3_OFFSET)
#define BL602_SE_AES_KEY_0          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_0_OFFSET)
#define BL602_SE_AES_KEY_1          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_1_OFFSET)
#define BL602_SE_AES_KEY_2          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_2_OFFSET)
#define BL602_SE_AES_KEY_3          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_3_OFFSET)
#define BL602_SE_AES_KEY_4          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_4_OFFSET)
#define BL602_SE_AES_KEY_5          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_5_OFFSET)
#define BL602_SE_AES_KEY_6          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_6_OFFSET)
#define BL602_SE_AES_KEY_7          (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_7_OFFSET)
#define BL602_SE_AES_KEY_SEL_0      (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_SEL_0_OFFSET)
#define BL602_SE_AES_KEY_SEL_1      (BL602_SEC_ENG_BASE + BL602_SE_AES_KEY_SEL_1_OFFSET)
#define BL602_SE_AES_ENDIAN         (BL602_SEC_ENG_BASE + BL602_SE_AES_ENDIAN_OFFSET)
#define BL602_SE_AES_SBOOT          (BL602_SEC_ENG_BASE + BL602_SE_AES_SBOOT_OFFSET)
#define BL602_SE_AES_LINK           (BL602_SEC_ENG_BASE + BL602_SE_AES_LINK_OFFSET)
#define BL602_SE_AES_CTRL_PROT      (BL602_SEC_ENG_BASE + BL602_SE_AES_CTRL_PROT_OFFSET)
#define BL602_SE_TRNG_CTRL_0        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_CTRL_0_OFFSET)
#define BL602_SE_TRNG_STATUS        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_STATUS_OFFSET)
#define BL602_SE_TRNG_DOUT_0        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_0_OFFSET)
#define BL602_SE_TRNG_DOUT_1        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_1_OFFSET)
#define BL602_SE_TRNG_DOUT_2        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_2_OFFSET)
#define BL602_SE_TRNG_DOUT_3        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_3_OFFSET)
#define BL602_SE_TRNG_DOUT_4        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_4_OFFSET)
#define BL602_SE_TRNG_DOUT_5        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_5_OFFSET)
#define BL602_SE_TRNG_DOUT_6        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_6_OFFSET)
#define BL602_SE_TRNG_DOUT_7        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_DOUT_7_OFFSET)
#define BL602_SE_TRNG_TEST          (BL602_SEC_ENG_BASE + BL602_SE_TRNG_TEST_OFFSET)
#define BL602_SE_TRNG_CTRL_1        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_CTRL_1_OFFSET)
#define BL602_SE_TRNG_CTRL_2        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_CTRL_2_OFFSET)
#define BL602_SE_TRNG_CTRL_3        (BL602_SEC_ENG_BASE + BL602_SE_TRNG_CTRL_3_OFFSET)
#define BL602_SE_TRNG_TEST_OUT_0    (BL602_SEC_ENG_BASE + BL602_SE_TRNG_TEST_OUT_0_OFFSET)
#define BL602_SE_TRNG_TEST_OUT_1    (BL602_SEC_ENG_BASE + BL602_SE_TRNG_TEST_OUT_1_OFFSET)
#define BL602_SE_TRNG_TEST_OUT_2    (BL602_SEC_ENG_BASE + BL602_SE_TRNG_TEST_OUT_2_OFFSET)
#define BL602_SE_TRNG_TEST_OUT_3    (BL602_SEC_ENG_BASE + BL602_SE_TRNG_TEST_OUT_3_OFFSET)
#define BL602_SE_TRNG_CTRL_PROT     (BL602_SEC_ENG_BASE + BL602_SE_TRNG_CTRL_PROT_OFFSET)
#define BL602_SE_PKA_CTRL_0         (BL602_SEC_ENG_BASE + BL602_SE_PKA_CTRL_0_OFFSET)
#define BL602_SE_PKA_SEED           (BL602_SEC_ENG_BASE + BL602_SE_PKA_SEED_OFFSET)
#define BL602_SE_PKA_CTRL_1         (BL602_SEC_ENG_BASE + BL602_SE_PKA_CTRL_1_OFFSET)
#define BL602_SE_PKA_RW             (BL602_SEC_ENG_BASE + BL602_SE_PKA_RW_OFFSET)
#define BL602_SE_PKA_RW_BURST       (BL602_SEC_ENG_BASE + BL602_SE_PKA_RW_BURST_OFFSET)
#define BL602_SE_PKA_CTRL_PROT      (BL602_SEC_ENG_BASE + BL602_SE_PKA_CTRL_PROT_OFFSET)
#define BL602_SE_CDET_CTRL_0        (BL602_SEC_ENG_BASE + BL602_SE_CDET_CTRL_0_OFFSET)
#define BL602_SE_CDET_CTRL_1        (BL602_SEC_ENG_BASE + BL602_SE_CDET_CTRL_1_OFFSET)
#define BL602_SE_CDET_CTRL_PROT     (BL602_SEC_ENG_BASE + BL602_SE_CDET_CTRL_PROT_OFFSET)
#define BL602_SE_GMAC_CTRL_0        (BL602_SEC_ENG_BASE + BL602_SE_GMAC_CTRL_0_OFFSET)
#define BL602_SE_GMAC_LCA           (BL602_SEC_ENG_BASE + BL602_SE_GMAC_LCA_OFFSET)
#define BL602_SE_GMAC_STATUS        (BL602_SEC_ENG_BASE + BL602_SE_GMAC_STATUS_OFFSET)
#define BL602_SE_GMAC_CTRL_PROT     (BL602_SEC_ENG_BASE + BL602_SE_GMAC_CTRL_PROT_OFFSET)
#define BL602_SE_CTRL_PROT_RD       (BL602_SEC_ENG_BASE + BL602_SE_CTRL_PROT_RD_OFFSET)
#define BL602_SE_CTRL_RESERVED_0    (BL602_SEC_ENG_BASE + BL602_SE_CTRL_RESERVED_0_OFFSET)
#define BL602_SE_CTRL_RESERVED_1    (BL602_SEC_ENG_BASE + BL602_SE_CTRL_RESERVED_1_OFFSET)
#define BL602_SE_CTRL_RESERVED_2    (BL602_SEC_ENG_BASE + BL602_SE_CTRL_RESERVED_2_OFFSET)

/* Register bit definitions *************************************************/

#define SD_STATUS_ENA_SHIFT      (28)
#define SD_STATUS_ENA_MASK       (0x0f << SD_STATUS_ENA_SHIFT)
#define SD_STATUS_MODE_SHIFT     (24)
#define SD_STATUS_MODE_MASK      (0x0f << SD_STATUS_MODE_SHIFT)
#define SD_STATUS_PWD_CNT_SHIFT  (4)
#define SD_STATUS_PWD_CNT_MASK   (0xfffff << SD_STATUS_PWD_CNT_SHIFT)
#define SD_STATUS_CCI_CLK_SEL    (1 << 3)
#define SD_STATUS_CCI_READ_EN    (1 << 2)
#define SD_STATUS_PWD_TRIG       (1 << 1)
#define SD_STATUS_PWD_BUSY       (1 << 0)

#define SE_SHA_CTRL_MSG_LEN_SHIFT          (16)
#define SE_SHA_CTRL_MSG_LEN_MASK           (0xffff << SE_SHA_CTRL_MSG_LEN_SHIFT)
#define SE_SHA_CTRL_LINK_MODE              (1 << 15)
#define SE_SHA_CTRL_INT_MASK               (1 << 11)
#define SE_SHA_CTRL_INT_SET_1T             (1 << 10)
#define SE_SHA_CTRL_INT_CLR_1T             (1 << 9)
#define SE_SHA_CTRL_INT                    (1 << 8)
#define SE_SHA_CTRL_HASH_SEL               (1 << 6)
#define SE_SHA_CTRL_EN                     (1 << 5)
#define SE_SHA_CTRL_MODE_SHIFT             (2)
#define SE_SHA_CTRL_MODE_MASK              (0x07 << SE_SHA_CTRL_MODE_SHIFT)
#define SE_SHA_CTRL_TRIG_1T                (1 << 1)
#define SE_SHA_CTRL_BUSY                   (1 << 0)

#define SE_SHA_ENDIAN_DOUT_ENDIAN          (1 << 0)

#define SE_SHA_CTRL_PROT_ID1_EN            (1 << 2)
#define SE_SHA_CTRL_PROT_ID0_EN            (1 << 1)
#define SE_SHA_CTRL_PROT_PROT_EN           (1 << 0)

#define SE_AES_CTRL_MSG_LEN_SHIFT          (16)
#define SE_AES_CTRL_MSG_LEN_MASK           (0xffff << SE_AES_CTRL_MSG_LEN_SHIFT)
#define SE_AES_CTRL_LINK_MODE              (1 << 15)
#define SE_AES_CTRL_IV_SEL                 (1 << 14)
#define SE_AES_CTRL_BLOCK_MODE_SHIFT       (12)
#define SE_AES_CTRL_BLOCK_MODE_MASK        (0x03 << SE_AES_CTRL_BLOCK_MODE_SHIFT)
#define SE_AES_CTRL_INT_MASK               (1 << 11)
#define SE_AES_CTRL_INT_SET_1T             (1 << 10)
#define SE_AES_CTRL_INT_CLR_1T             (1 << 9)
#define SE_AES_CTRL_INT                    (1 << 8)
#define SE_AES_CTRL_HW_KEY_EN              (1 << 7)
#define SE_AES_CTRL_DEC_KEY_SEL            (1 << 6)
#define SE_AES_CTRL_DEC_EN                 (1 << 5)
#define SE_AES_CTRL_MODE_SHIFT             (3)
#define SE_AES_CTRL_MODE_MASK              (0x03 << SE_AES_CTRL_MODE_SHIFT)
#define SE_AES_CTRL_EN                     (1 << 2)
#define SE_AES_CTRL_TRIG_1T                (1 << 1)
#define SE_AES_CTRL_BUSY                   (1 << 0)

#define SE_AES_KEY_SEL_0_MASK              (0x03)

#define SE_AES_KEY_SEL_1_MASK              (0x03)

#define SE_AES_ENDIAN_CTR_LEN_SHIFT        (30)
#define SE_AES_ENDIAN_CTR_LEN_MASK         (0x03 << SE_AES_ENDIAN_CTR_LEN_SHIFT)
#define SE_AES_ENDIAN_IV_ENDIAN            (1 << 3)
#define SE_AES_ENDIAN_KEY_ENDIAN           (1 << 2)
#define SE_AES_ENDIAN_DIN_ENDIAN           (1 << 1)
#define SE_AES_ENDIAN_DOUT_ENDIAN          (1 << 0)

#define SE_AES_SBOOT_SBOOT_KEY_SEL         (1 << 0)

#define SE_AES_CTRL_PROT_ID1_EN            (1 << 2)
#define SE_AES_CTRL_PROT_ID0_EN            (1 << 1)
#define SE_AES_CTRL_PROT_PROT_EN           (1 << 0)

#define SE_TRNG_CTRL_0_MANUAL_EN           (1 << 15)
#define SE_TRNG_CTRL_0_MANUAL_RESEED       (1 << 14)
#define SE_TRNG_CTRL_0_MANUAL_FUN_SEL      (1 << 13)
#define SE_TRNG_CTRL_0_INT_MASK            (1 << 11)
#define SE_TRNG_CTRL_0_INT_SET_1T          (1 << 10)
#define SE_TRNG_CTRL_0_INT_CLR_1T          (1 << 9)
#define SE_TRNG_CTRL_0_INT                 (1 << 8)
#define SE_TRNG_CTRL_0_HT_ERROR            (1 << 4)
#define SE_TRNG_CTRL_0_DOUT_CLR_1T         (1 << 3)
#define SE_TRNG_CTRL_0_EN                  (1 << 2)
#define SE_TRNG_CTRL_0_TRIG_1T             (1 << 1)
#define SE_TRNG_CTRL_0_BUSY                (1 << 0)

#define SE_TRNG_TEST_HT_ALARM_N_SHIFT      (4)
#define SE_TRNG_TEST_HT_ALARM_N_MASK       (0xff << SE_TRNG_TEST_HT_ALARM_N_SHIFT)
#define SE_TRNG_TEST_HT_DIS                (1 << 3)
#define SE_TRNG_TEST_CP_BYPASS             (1 << 2)
#define SE_TRNG_TEST_CP_TEST_EN            (1 << 1)
#define SE_TRNG_TEST_TEST_EN               (1 << 0)

#define SE_TRNG_CTRL_2_SERESEED_N_MSB_MASK  (0xffff)

#define SE_TRNG_CTRL_3_ROSC_EN             (1 << 31)
#define SE_TRNG_CTRL_3_HT_OD_EN            (1 << 26)
#define SE_TRNG_CTRL_3_HT_APT_C_SHIFT      (16)
#define SE_TRNG_CTRL_3_HT_APT_C_MASK       (0x3ff << SE_TRNG_CTRL_3_HT_APT_C_SHIFT)
#define SE_TRNG_CTRL_3_HT_RCT_C_SHIFT      (8)
#define SE_TRNG_CTRL_3_HT_RCT_C_MASK       (0xff << SE_TRNG_CTRL_3_HT_RCT_C_SHIFT)
#define SE_TRNG_CTRL_3_CP_RATIO_MASK       (0xff)

#define SE_TRNG_CTRL_PROT_ID1_EN           (1 << 2)
#define SE_TRNG_CTRL_PROT_ID0_EN           (1 << 1)
#define SE_TRNG_CTRL_PROT_PROT_EN          (1 << 0)

#define SE_PKA_CTRL_0_STATUS_SHIFT         (17)
#define SE_PKA_CTRL_0_STATUS_MASK          (0x7fff << SE_PKA_CTRL_0_STATUS_SHIFT)
#define SE_PKA_CTRL_0_STATUS_CLR_1T        (1 << 16)
#define SE_PKA_CTRL_0_RAM_CLR_MD           (1 << 13)
#define SE_PKA_CTRL_0_ENDIAN               (1 << 12)
#define SE_PKA_CTRL_0_INT_MASK             (1 << 11)
#define SE_PKA_CTRL_0_INT_SET              (1 << 10)
#define SE_PKA_CTRL_0_INT_CLR_1T           (1 << 9)
#define SE_PKA_CTRL_0_INT                  (1 << 8)
#define SE_PKA_CTRL_0_PROT_MD_SHIFT        (4)
#define SE_PKA_CTRL_0_PROT_MD_MASK         (0x0f << SE_PKA_CTRL_0_PROT_MD_SHIFT)
#define SE_PKA_CTRL_0_EN                   (1 << 3)
#define SE_PKA_CTRL_0_BUSY                 (1 << 2)
#define SE_PKA_CTRL_0_DONE_CLR_1T          (1 << 1)
#define SE_PKA_CTRL_0_DONE                 (1 << 0)

#define SE_PKA_CTRL_1_HBYPASS              (1 << 3)
#define SE_PKA_CTRL_1_HBURST_MASK          (0x07)

#define SE_PKA_CTRL_PROT_ID1_EN            (1 << 2)
#define SE_PKA_CTRL_PROT_ID0_EN            (1 << 1)
#define SE_PKA_CTRL_PROT_PROT_EN           (1 << 0)

#define SE_CDET_CTRL_0_G_LOOP_MIN_SHIFT    (24)
#define SE_CDET_CTRL_0_G_LOOP_MIN_MASK     (0xff << SE_CDET_CTRL_0_G_LOOP_MIN_SHIFT)
#define SE_CDET_CTRL_0_G_LOOP_MAX_SHIFT    (16)
#define SE_CDET_CTRL_0_G_LOOP_MAX_MASK     (0xff << SE_CDET_CTRL_0_G_LOOP_MAX_SHIFT)
#define SE_CDET_CTRL_0_STATUS_SHIFT        (2)
#define SE_CDET_CTRL_0_STATUS_MASK         (0x3fff << SE_CDET_CTRL_0_STATUS_SHIFT)
#define SE_CDET_CTRL_0_ERROR               (1 << 1)
#define SE_CDET_CTRL_0_EN                  (1 << 0)

#define SE_CDET_CTRL_1_G_SLP_N_SHIFT       (16)
#define SE_CDET_CTRL_1_G_SLP_N_MASK        (0xff << SE_CDET_CTRL_1_G_SLP_N_SHIFT)
#define SE_CDET_CTRL_1_T_DLY_N_SHIFT       (8)
#define SE_CDET_CTRL_1_T_DLY_N_MASK        (0xff << SE_CDET_CTRL_1_T_DLY_N_SHIFT)
#define SE_CDET_CTRL_1_T_LOOP_N_MASK       (0xff)

#define SE_CDET_CTRL_PROT_ID1_EN           (1 << 2)
#define SE_CDET_CTRL_PROT_ID0_EN           (1 << 1)
#define SE_CDET_CTRL_PROT_PROT_EN          (1 << 0)

#define SE_GMAC_CTRL_0_X_ENDIAN            (1 << 14)
#define SE_GMAC_CTRL_0_H_ENDIAN            (1 << 13)
#define SE_GMAC_CTRL_0_T_ENDIAN            (1 << 12)
#define SE_GMAC_CTRL_0_INT_MASK            (1 << 11)
#define SE_GMAC_CTRL_0_INT_SET_1T          (1 << 10)
#define SE_GMAC_CTRL_0_INT_CLR_1T          (1 << 9)
#define SE_GMAC_CTRL_0_INT                 (1 << 8)
#define SE_GMAC_CTRL_0_EN                  (1 << 2)
#define SE_GMAC_CTRL_0_TRIG_1T             (1 << 1)
#define SE_GMAC_CTRL_0_BUSY                (1 << 0)

#define SE_GMAC_CTRL_PROT_ID1_EN           (1 << 2)
#define SE_GMAC_CTRL_PROT_ID0_EN           (1 << 1)
#define SE_GMAC_CTRL_PROT_PROT_EN          (1 << 0)

#define SE_CTRL_PROT_RD_DBG_DIS            (1 << 31)
#define SE_CTRL_PROT_RD_GMAC_ID1_EN_RD     (1 << 22)
#define SE_CTRL_PROT_RD_GMAC_ID0_EN_RD     (1 << 21)
#define SE_CTRL_PROT_RD_GMAC_PROT_EN_RD    (1 << 20)
#define SE_CTRL_PROT_RD_CDET_ID1_EN_RD     (1 << 18)
#define SE_CTRL_PROT_RD_CDET_ID0_EN_RD     (1 << 17)
#define SE_CTRL_PROT_RD_CDET_PROT_EN_RD    (1 << 16)
#define SE_CTRL_PROT_RD_PKA_ID1_EN_RD      (1 << 14)
#define SE_CTRL_PROT_RD_PKA_ID0_EN_RD      (1 << 13)
#define SE_CTRL_PROT_RD_PKA_PROT_EN_RD     (1 << 12)
#define SE_CTRL_PROT_RD_TRNG_ID1_EN_RD     (1 << 10)
#define SE_CTRL_PROT_RD_TRNG_ID0_EN_RD     (1 << 9)
#define SE_CTRL_PROT_RD_TRNG_PROT_EN_RD    (1 << 8)
#define SE_CTRL_PROT_RD_AES_ID1_EN_RD      (1 << 6)
#define SE_CTRL_PROT_RD_AES_ID0_EN_RD      (1 << 5)
#define SE_CTRL_PROT_RD_AES_PROT_EN_RD     (1 << 4)
#define SE_CTRL_PROT_RD_SHA_ID1_EN_RD      (1 << 2)
#define SE_CTRL_PROT_RD_SHA_ID0_EN_RD      (1 << 1)
#define SE_CTRL_PROT_RD_SHA_PROT_EN_RD     (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_SEC_H */
