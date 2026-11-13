/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_netc_switch.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_SWITCH_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_SWITCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IMXRT1180RM Rev. 10, sections 53.4.2.2.2, 53.4.2.4.5 and 53.4.6.5. */

#define NETC_SW_CCAR                0x0080
#define NETC_SW_CBDRMR              0x0800
#define NETC_SW_CBDRBAR0            0x0810
#define NETC_SW_CBDRBAR1            0x0814
#define NETC_SW_CBDRPIR             0x0818
#define NETC_SW_CBDRCIR             0x081c
#define NETC_SW_CBDRLENR            0x0820
#define NETC_SW_VFHTDECR0           0x2010
#define NETC_SW_CPU                 (1 << 4)
#define NETC_SW_VFHTDECR1           0x2014
#define NETC_SW_SHARED_FID          ((0xffffu << 16) | (1 << 12))
#define NETC_SW_VFHTDECR2           0x2018
#define NETC_SW_LEARN_FLOOD         ((2 << 24) | (2 << 27))
#define NETC_SW_FDBHTMCR            0x2024

#define NETC_SW_PTAR                0x0050
#define NETC_SW_STANDARD_TPIDS      0x33
#define NETC_SW_PQOSMR              0x0054
#define NETC_SW_PIPFCR              0x0084
#define NETC_SW_PDGSR               0x0138
#define NETC_SW_PCTFCR              0x0450
#define NETC_SW_STORE_FORWARD       3
#define NETC_SW_PISIDCR             0x0460
#define NETC_SW_NULL_STREAM         (0xffffu << 16)
#define NETC_SW_PIPV2QMR0           0x0470
#define NETC_SW_BPCR                0x0500
#define NETC_SW_BPDVR               0x0510
#define NETC_SW_DEFAULT_VLAN        (1 | (1 << 24) | (15 << 20))
#define NETC_SW_BPSTGSR             0x0520
#define NETC_SW_STG_FORWARD         2

#define NETC_NTMP_FDB               15
#define NETC_NTMP_DELETE            1
#define NETC_NTMP_ADDQUERYUPDATE    14
#define NETC_NTMP_EXACT             (1 << 12)
#define NETC_NTMP_VERSION2          (2 << 24)
#define NETC_NTMP_NPF               (1 << 15)
#define NETC_NTMP_READY             (1u << 31)
#define NETC_NTMP_ERROR_MASK        (0xfff << 16)
#define NETC_NTMP_FDB_CFGEU         (1 << 0)
#define NETC_NTMP_FDB_REQUEST       48
#define NETC_NTMP_FDB_DELETE        36
#define NETC_NTMP_FDB_RESPONSE      36

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct imxrt_netc_cmd_s
{
  uint32_t address;
  uint32_t address_hi;
  uint32_t lengths;
  uint32_t command;
  uint32_t reserved[3];
  uint32_t format;
};

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_SWITCH_H */
