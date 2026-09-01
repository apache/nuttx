/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/include/imxrt118x_trdc_config.h
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

#ifndef __BOARDS_ARM_IMXRT_IMXRT1180_EVK_INCLUDE_IMXRT118X_TRDC_CONFIG_H
#define __BOARDS_ARM_IMXRT_IMXRT1180_EVK_INCLUDE_IMXRT118X_TRDC_CONFIG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SP(X)           ((X) << 12)
#define SU(X)           ((X) << 8)
#define NP(X)           ((X) << 4)
#define NU(X)           ((X) << 0)
#define LK              (1 << 31)
#define RWX             7
#define RW              6
#define RX              5
#define R               4
#define X               1

#define TRDC_FULL_ACCESS_GLBAC(id, mbc_mrc) \
  { mbc_mrc, id, SP(RWX) | SU(RWX) | NP(RWX) | NU(RWX) }

/* Domain identifiers used by the TRDC setup. */

#define DID_CM33         BOARD_CCM_DOMAIN_CM33
#define DID_EDMA3_RESET  4
#define DID_CM7          BOARD_CCM_DOMAIN_CM7
#define DID_USDHC1       5
#define DID_USDHC2       6
#define DID_DMA          7
#define DID_EDMA3        DID_DMA
#define DID_EDMA4        DID_DMA
#define DID_CORESIGHT    8
#define DID_DAP          9
#define DID_NETC         10
#define DID_FLEXSPI_FLR  DID_NETC
#define DID_USB          11

#define TRDC_VALID_DOMAINS_MBC(mbc, mem) \
  { mbc, DID_CM33,       mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_CM7,        mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_USDHC1,     mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_USDHC2,     mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_DMA,        mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_CORESIGHT,  mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_DAP,        mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_NETC,       mem, MBC_BLK_ALL, 0, true }, \
  { mbc, DID_USB,        mem, MBC_BLK_ALL, 0, true }

#define TRDC_VALID_DOMAINS_MRC(mrc, base, size) \
  { mrc, DID_CM33,       0, base, size, 0, true }, \
  { mrc, DID_CM7,        0, base, size, 0, true }, \
  { mrc, DID_USDHC1,     0, base, size, 0, true }, \
  { mrc, DID_USDHC2,     0, base, size, 0, true }, \
  { mrc, DID_DMA,        0, base, size, 0, true }, \
  { mrc, DID_CORESIGHT,  0, base, size, 0, true }, \
  { mrc, DID_DAP,        0, base, size, 0, true }, \
  { mrc, DID_NETC,       0, base, size, 0, true }, \
  { mrc, DID_USB,        0, base, size, 0, true }

#define TRDC_CM33_ROM_BASE       0x00000000
#define TRDC_CM33_ROM_SIZE       0x00028000
#define TRDC_FLEXSPI2_BASE       0x04000000
#define TRDC_FLEXSPI2_SIZE       0x04000000
#define TRDC_FLEXSPI1_BASE       0x28000000
#define TRDC_FLEXSPI1_SIZE       0x08000000
#define TRDC_CM7_TCM_BASE        0x203c0000
#define TRDC_CM7_TCM_SIZE        0x00080000
#define TRDC_OCRAM1_BASE         0x20480000
#define TRDC_OCRAM1_SIZE         0x00080000
#define TRDC_OCRAM2_BASE         0x20500000
#define TRDC_OCRAM2_SIZE         0x00040000
#define TRDC_SEMC_BASE           0x80000000
#define TRDC_SEMC_SIZE           0x10000000
#define TRDC_NETC_BASE           0x60000000
#define TRDC_NETC_SIZE           0x01000000

/* TRDC1 MBC access policy. */

struct trdc_glbac_config trdc_a_mbc_glbac[] =
{
  TRDC_FULL_ACCESS_GLBAC(0, 0), /* MBC0 */
  TRDC_FULL_ACCESS_GLBAC(1, 0),
  TRDC_FULL_ACCESS_GLBAC(2, 0),
  TRDC_FULL_ACCESS_GLBAC(3, 0),
  TRDC_FULL_ACCESS_GLBAC(4, 0),
  TRDC_FULL_ACCESS_GLBAC(5, 0),
  TRDC_FULL_ACCESS_GLBAC(6, 0),
  TRDC_FULL_ACCESS_GLBAC(7, 0),
  TRDC_FULL_ACCESS_GLBAC(0, 1), /* MBC1 */
  TRDC_FULL_ACCESS_GLBAC(1, 1),
  TRDC_FULL_ACCESS_GLBAC(2, 1),
  TRDC_FULL_ACCESS_GLBAC(3, 1),
  TRDC_FULL_ACCESS_GLBAC(4, 1),
  TRDC_FULL_ACCESS_GLBAC(5, 1),
  TRDC_FULL_ACCESS_GLBAC(6, 1),
  TRDC_FULL_ACCESS_GLBAC(7, 1),
};

/* TRDC1 MBC block assignments. */

struct trdc_mbc_config trdc_a_mbc[] =
{
  TRDC_VALID_DOMAINS_MBC(0, 0), /* MBC_A0 AIPS1 */
  TRDC_VALID_DOMAINS_MBC(0, 2), /* MBC_A0 GPIO1 */
  TRDC_VALID_DOMAINS_MBC(1, 0), /* MBC_A1 CM33 Code-TCM */
  TRDC_VALID_DOMAINS_MBC(1, 1), /* MBC_A1 CM33 System-TCM */
};

struct trdc_glbac_config trdc_a_mrc_glbac[] =
{
  TRDC_FULL_ACCESS_GLBAC(0, 0), /* MRC0 */
  TRDC_FULL_ACCESS_GLBAC(1, 0),
  TRDC_FULL_ACCESS_GLBAC(2, 0),
  TRDC_FULL_ACCESS_GLBAC(3, 0),
  TRDC_FULL_ACCESS_GLBAC(4, 0),
  TRDC_FULL_ACCESS_GLBAC(5, 0),
  TRDC_FULL_ACCESS_GLBAC(6, 0),
  TRDC_FULL_ACCESS_GLBAC(7, 0),
  TRDC_FULL_ACCESS_GLBAC(0, 1), /* MRC1 */
  TRDC_FULL_ACCESS_GLBAC(1, 1),
  TRDC_FULL_ACCESS_GLBAC(2, 1),
  TRDC_FULL_ACCESS_GLBAC(3, 1),
  TRDC_FULL_ACCESS_GLBAC(4, 1),
  TRDC_FULL_ACCESS_GLBAC(5, 1),
  TRDC_FULL_ACCESS_GLBAC(6, 1),
  TRDC_FULL_ACCESS_GLBAC(7, 1),
};

struct trdc_mrc_config trdc_a_mrc[] =
{
  TRDC_VALID_DOMAINS_MRC(0, TRDC_CM33_ROM_BASE, TRDC_CM33_ROM_SIZE),
  TRDC_VALID_DOMAINS_MRC(1, TRDC_FLEXSPI2_BASE, TRDC_FLEXSPI2_SIZE),
};

/* TRDC2 MBC access policy. */

struct trdc_glbac_config trdc_w_mbc_glbac[] =
{
  TRDC_FULL_ACCESS_GLBAC(0, 0), /* MBC0 */
  TRDC_FULL_ACCESS_GLBAC(1, 0),
  TRDC_FULL_ACCESS_GLBAC(2, 0),
  TRDC_FULL_ACCESS_GLBAC(3, 0),
  TRDC_FULL_ACCESS_GLBAC(4, 0),
  TRDC_FULL_ACCESS_GLBAC(5, 0),
  TRDC_FULL_ACCESS_GLBAC(6, 0),
  TRDC_FULL_ACCESS_GLBAC(7, 0),
  TRDC_FULL_ACCESS_GLBAC(0, 1), /* MBC1 */
  TRDC_FULL_ACCESS_GLBAC(1, 1),
  TRDC_FULL_ACCESS_GLBAC(2, 1),
  TRDC_FULL_ACCESS_GLBAC(3, 1),
  TRDC_FULL_ACCESS_GLBAC(4, 1),
  TRDC_FULL_ACCESS_GLBAC(5, 1),
  TRDC_FULL_ACCESS_GLBAC(6, 1),
  TRDC_FULL_ACCESS_GLBAC(7, 1),
};

/* TRDC2 MBC block assignments. */

struct trdc_mbc_config trdc_w_mbc[] =
{
  TRDC_VALID_DOMAINS_MBC(0, 0), /* MBC_W0 AIPS2 */
  TRDC_VALID_DOMAINS_MBC(0, 1), /* MBC_W0 GPIO2/4/6 */
  TRDC_VALID_DOMAINS_MBC(0, 2), /* MBC_W0 GPIO3/5 */
  TRDC_VALID_DOMAINS_MBC(0, 3), /* MBC_W0 DAP */
  TRDC_VALID_DOMAINS_MBC(1, 0), /* MBC_W1 AIPS3 */
  TRDC_VALID_DOMAINS_MBC(1, 1), /* MBC_W1 AHB_ISPAP */
  TRDC_VALID_DOMAINS_MBC(1, 2), /* MBC_W1 NIC_MAIN GPV */
  TRDC_VALID_DOMAINS_MBC(1, 3), /* MBC_W1 SRAMC */
};

struct trdc_glbac_config trdc_w_mrc_glbac[] =
{
  TRDC_FULL_ACCESS_GLBAC(0, 1), /* MRC1 */
  TRDC_FULL_ACCESS_GLBAC(1, 1),
  TRDC_FULL_ACCESS_GLBAC(2, 1),
  TRDC_FULL_ACCESS_GLBAC(3, 1),
  TRDC_FULL_ACCESS_GLBAC(4, 1),
  TRDC_FULL_ACCESS_GLBAC(5, 1),
  TRDC_FULL_ACCESS_GLBAC(6, 1),
  TRDC_FULL_ACCESS_GLBAC(7, 1),
  TRDC_FULL_ACCESS_GLBAC(0, 2), /* MRC2 */
  TRDC_FULL_ACCESS_GLBAC(1, 2),
  TRDC_FULL_ACCESS_GLBAC(2, 2),
  TRDC_FULL_ACCESS_GLBAC(3, 2),
  TRDC_FULL_ACCESS_GLBAC(4, 2),
  TRDC_FULL_ACCESS_GLBAC(5, 2),
  TRDC_FULL_ACCESS_GLBAC(6, 2),
  TRDC_FULL_ACCESS_GLBAC(7, 2),
  TRDC_FULL_ACCESS_GLBAC(0, 3), /* MRC3 */
  TRDC_FULL_ACCESS_GLBAC(1, 3),
  TRDC_FULL_ACCESS_GLBAC(2, 3),
  TRDC_FULL_ACCESS_GLBAC(3, 3),
  TRDC_FULL_ACCESS_GLBAC(4, 3),
  TRDC_FULL_ACCESS_GLBAC(5, 3),
  TRDC_FULL_ACCESS_GLBAC(6, 3),
  TRDC_FULL_ACCESS_GLBAC(7, 3),
  TRDC_FULL_ACCESS_GLBAC(0, 4), /* MRC4 */
  TRDC_FULL_ACCESS_GLBAC(1, 4),
  TRDC_FULL_ACCESS_GLBAC(2, 4),
  TRDC_FULL_ACCESS_GLBAC(3, 4),
  TRDC_FULL_ACCESS_GLBAC(4, 4),
  TRDC_FULL_ACCESS_GLBAC(5, 4),
  TRDC_FULL_ACCESS_GLBAC(6, 4),
  TRDC_FULL_ACCESS_GLBAC(7, 4),
  TRDC_FULL_ACCESS_GLBAC(0, 5), /* MRC5 */
  TRDC_FULL_ACCESS_GLBAC(1, 5),
  TRDC_FULL_ACCESS_GLBAC(2, 5),
  TRDC_FULL_ACCESS_GLBAC(3, 5),
  TRDC_FULL_ACCESS_GLBAC(4, 5),
  TRDC_FULL_ACCESS_GLBAC(5, 5),
  TRDC_FULL_ACCESS_GLBAC(6, 5),
  TRDC_FULL_ACCESS_GLBAC(7, 5),
  TRDC_FULL_ACCESS_GLBAC(0, 6), /* MRC6 */
  TRDC_FULL_ACCESS_GLBAC(1, 6),
  TRDC_FULL_ACCESS_GLBAC(2, 6),
  TRDC_FULL_ACCESS_GLBAC(3, 6),
  TRDC_FULL_ACCESS_GLBAC(4, 6),
  TRDC_FULL_ACCESS_GLBAC(5, 6),
  TRDC_FULL_ACCESS_GLBAC(6, 6),
  TRDC_FULL_ACCESS_GLBAC(7, 6),
};

/* TRDC2 MRC region assignments. */

struct trdc_mrc_config trdc_w_mrc[] =
{
  TRDC_VALID_DOMAINS_MRC(1, TRDC_FLEXSPI1_BASE, TRDC_FLEXSPI1_SIZE),
  TRDC_VALID_DOMAINS_MRC(2, TRDC_CM7_TCM_BASE, TRDC_CM7_TCM_SIZE),
  TRDC_VALID_DOMAINS_MRC(3, TRDC_OCRAM1_BASE, TRDC_OCRAM1_SIZE),
  TRDC_VALID_DOMAINS_MRC(4, TRDC_OCRAM2_BASE, TRDC_OCRAM2_SIZE),
  TRDC_VALID_DOMAINS_MRC(5, TRDC_SEMC_BASE, TRDC_SEMC_SIZE),
  TRDC_VALID_DOMAINS_MRC(6, TRDC_NETC_BASE, TRDC_NETC_SIZE),
};

#endif /* __BOARDS_ARM_IMXRT_IMXRT1180_EVK_INCLUDE_IMXRT118X_TRDC_CONFIG_H */
