/****************************************************************************
 * drivers/net/oa_tc6/oa_tc6_ncv7410.h
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

#ifndef __DRIVERS_NET_OA_TC6_NCV7410_H
#define __DRIVERS_NET_OA_TC6_NCV7410_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "oa_tc6.h"

/****************************************************************************
 * Preprocessor Macros
 ****************************************************************************/

/* The PHYID of the NCN26010 chip is identical */

#define OA_TC6_NCV7410_PHYID  0xBC0189A1U

/* Registers specific to the NCV7410 */

#define NCV_PHY_CONTROL_LCTL_POS   12

#define NCV_MAC_CONTROL0_MMS       1
#define NCV_MAC_CONTROL0_ADDR      0x0U
#define NCV_MAC_CONTROL0_REGID     OA_TC6_MAKE_REGID(NCV_MAC_CONTROL0_MMS, NCV_MAC_CONTROL0_ADDR)
#define NCV_MAC_CONTROL0_ADRF_POS  16
#define NCV_MAC_CONTROL0_FCSA_POS  8
#define NCV_MAC_CONTROL0_TXEN_POS  1
#define NCV_MAC_CONTROL0_RXEN_POS  0

#define NCV_ADDRFILT0L_MMS         1
#define NCV_ADDRFILT0L_ADDR        0x10U
#define NCV_ADDRFILT0L_REGID       OA_TC6_MAKE_REGID(NCV_ADDRFILT0L_MMS, NCV_ADDRFILT0L_ADDR)

#define NCV_ADDRFILT0H_MMS         1
#define NCV_ADDRFILT0H_ADDR        0x11U
#define NCV_ADDRFILT0H_REGID       OA_TC6_MAKE_REGID(NCV_ADDRFILT0H_MMS, NCV_ADDRFILT0H_ADDR)

#define NCV_ADDRMASK0L_MMS         1
#define NCV_ADDRMASK0L_ADDR        0x20U
#define NCV_ADDRMASK0L_REGID       OA_TC6_MAKE_REGID(NCV_ADDRMASK0L_MMS, NCV_ADDRMASK0L_ADDR)

#define NCV_ADDRMASK0H_MMS         1
#define NCV_ADDRMASK0H_ADDR        0x21U
#define NCV_ADDRMASK0H_REGID       OA_TC6_MAKE_REGID(NCV_ADDRMASK0H_MMS, NCV_ADDRMASK0H_ADDR)

#define NCV_ADDRFILTL_REGID(i)     OA_TC6_MAKE_REGID(NCV_ADDRFILT0L_MMS, NCV_ADDRFILT0L_ADDR + 4 * i)
#define NCV_ADDRFILTH_REGID(i)     OA_TC6_MAKE_REGID(NCV_ADDRFILT0H_MMS, NCV_ADDRFILT0H_ADDR + 4 * i)
#define NCV_ADDRMASKL_REGID(i)     OA_TC6_MAKE_REGID(NCV_ADDRMASK0L_MMS, NCV_ADDRMASK0L_ADDR + 4 * i)
#define NCV_ADDRMASKH_REGID(i)     OA_TC6_MAKE_REGID(NCV_ADDRMASK0H_MMS, NCV_ADDRMASK0H_ADDR + 4 * i)

#define NCV_DIO_CONFIG_MMS         12
#define NCV_DIO_CONFIG_ADDR        0x0012U
#define NCV_DIO_CONFIG_REGID       OA_TC6_MAKE_REGID(NCV_DIO_CONFIG_MMS, NCV_DIO_CONFIG_ADDR)
#define NCV_DIO_CONFIG_DEF         0x6060
#define NCV_DIO0_FUNC_POS          1
#define NCV_DIO1_FUNC_POS          9
#define NCV_DIO0_OUT_VAL_POS       0
#define NCV_DIO1_OUT_VAL_POS       8
#define NCV_DIO_TRISTATE_FUNC      0x0
#define NCV_DIO_GPIO_FUNC          0x1
#define NCV_DIO_SFD_TX_FUNC        0x2
#define NCV_DIO_SFD_RX_FUNC        0x3
#define NCV_DIO_LINK_CTRL_FUNC     0x4
#define NCV_DIO_SFD_TXRX_FUNC      0xB
#define NCV_DIO_TXRX_FUNC          0xF

#define NCV_MACID0_MMS             12
#define NCV_MACID0_ADDR            0x1002U
#define NCV_MACID0_REGID           OA_TC6_MAKE_REGID(NCV_MACID0_MMS, NCV_MACID0_ADDR)
#define NCV_MACID0_MASK            GENMASK(15, 0)
#define NCV_MACID0_POS             0

#define NCV_MACID1_MMS             12
#define NCV_MACID1_ADDR            0x1003U
#define NCV_MACID1_REGID           OA_TC6_MAKE_REGID(NCV_MACID1_MMS, NCV_MACID1_ADDR)
#define NCV_MACID1_MASK            GENMASK(7, 0)
#define NCV_MACID1_POS             0

#endif /* __DRIVERS_NET_OA_TC6_NCV7410_H */
