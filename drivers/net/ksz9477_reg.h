/****************************************************************************
 * drivers/net/ksz9477_reg.h
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

#ifndef __DRIVERS_NET_KSZ9477_REG_H
#define __DRIVERS_NET_KSZ9477_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These define only a subset of full register map. New registers can added
 * as needed
 */

#define KSZ9477_I2C_ADDRESS                0x5f
#define KSZ9477_I2C_SPEED                  I2C_SPEED_FAST

/* Global registers */

#define KSZ9477_ID0                        0x0
#define KSZ9477_ID1                        0x1
#define KSZ9477_ID2                        0x2
#define KSZ9477_ID3                        0x3

/* Port registers */

#define KSZ9477_PORT_REG(p,r)              ((((uint16_t)(p)) << 12) | (r))

#define KSZ9477_PORT_DEFAULT_TAG0(p)       KSZ9477_PORT_REG(p, 0x0)
#define KSZ9477_PORT_DEFAULT_TAG1(p)       KSZ9477_PORT_REG(p, 0x1)

#define KSZ9477_PHY_CONTROL(p)             KSZ9477_PORT_REG(p, 0x100)
#define KSZ9477_PHY_STATUS(p)              KSZ9477_PORT_REG(p, 0x102)

/* Note! Unlike in data sheet, the indirect data register reads and
 * writes must be done with 32-bit accesses and the address is
 * 0x204
 */

#define KSZ9477_PORT_ADDRESS(p)            KSZ9477_PORT_REG(p, 0x200)
#define KSZ9477_PORT_DATA(p)               KSZ9477_PORT_REG(p, 0x204)

/* Switch queue management registers */

#define KSZ9477_Q_MGMT_CONTROL0            0x0390
#define KSZ9477_Q_MGMT_PORT_VLAN_ENABLE    (1 << 1)

#define KSZ9477_Q_MGMT_PORT_CONTROL0(p)    KSZ9477_PORT_REG(p, 0xA00)
#define KSZ9477_Q_MGMT_PORT_CONTROL1(p)    KSZ9477_PORT_REG(p, 0xA04)

#define KSZ9477_SGMII_PORT_ADDRESS         KSZ9477_PORT_ADDRESS(7)
#define KSZ9477_SGMII_PORT_DATA            KSZ9477_PORT_DATA(7)

/* SGMII indirect registers */

#define KSZ9477_SGMII_CONTROL              0x1F0000
#define KSZ9477_SGMII_STATUS               0x1F0001
#define KSZ9477_SGMII_ID1                  0x1F0002
#define KSZ9477_SGMII_ID2                  0x1F0003
#define KSZ9477_SGMII_AUTONEG_ADVERTISE    0x1F0004
#define KSZ9477_SGMII_LINK_PARTNER_ABILITY 0x1F0005
#define KSZ9477_SGMII_AUTONEG_EXPANSION    0x1F0006
#define KSZ9477_SGMII_DIGITAL_CONTROL      0x1F8000
#define KSZ9477_SGMII_AUTONEG_CONTROL      0x1F8001
#define KSZ9477_SGMII_AUTONEG_STATUS       0x1F8002

/* Register bit definitions */

/* KSZ9477_ID2, KSZ9477_ID1 */

#define KSZ9477_ID                         0x9477

/* KSZ9477_SGMII_ID2, KSZ9477_SGMII_ID1 */

#define SGMII_PHY_ID                       0xced07996

/* KSZ9477_SGMII_PORT_ADDRESS */

#define SGMII_PORT_ADDRESS_AUTO_INC_ENB    (1 << 23)

/* KSZ9477_SGMII_AUTONEG_ADVERTISE */

#define SGMII_AUTONEG_ADVERTISE_FD         (1 << 5)
#define SGMII_AUTONEG_ADVERTISE_HD         (1 << 5)

/* KSZ9477_SGMII_AUTONEG_CONTROL */

#define SGMII_AUTONEG_CONTROL_IE           (1 << 0)
#define SGMII_AUTONEG_CONTROL_PCS_SERDES   (0 << 1)
#define SGMII_AUTONEG_CONTROL_PCS_SGMII    (2 << 1)
#define SGMII_AUTONEG_CONTROL_TC_MASTER    (1 << 3)
#define SGMII_AUTONEG_CONTROL_LINK_STATUS  (1 << 4)

/* Port Mirroring Control Register */

#define KSZ9477_PORT_MIRROR_CONTROL(p)     KSZ9477_PORT_REG(p, 0x800)
#define KSZ9477_PORT_MIRROR_SNIFFER_PORT   (1 << 1)
#define KSZ9477_PORT_MIRROR_TX_SNIFF       (1 << 5)
#define KSZ9477_PORT_MIRROR_RX_SNIFF       (1 << 6)

/* Global Port Mirroring and Snooping Control Register */

#define KSZ9477_GLOBAL_PORT_MIRROR_CONTROL 0x0370
#define KSZ9477_GLOBAL_PORT_SNIFF_MODE     (1 << 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct ksz9477_transfer_s
{
  uint16_t len;
  uint16_t reg;
  uint32_t data;
} end_packed_struct;

#ifndef __ASSEMBLY__

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
 * Public Function Prototypes
 ****************************************************************************/

int ksz9477_init(ksz9477_port_t master_port);

int ksz9477_read(struct ksz9477_transfer_s *read_msg);

int ksz9477_write(struct ksz9477_transfer_s *write_msg);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __DRIVERS_NET_KSZ9477_REG_H */
