/****************************************************************************
 * drivers/net/ncv7410.h
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

#ifndef __DRIVERS_NET_NCV7410_H
#define __DRIVERS_NET_NCV7410_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/bits.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NuttX SPI mode number for SPI config as defined in OpenAlliance TC6 */

#define OA_SPI_MODE 0

/* Number of bits in a SPI word */

#define OA_SPI_NBITS 8

#define NCV_CHUNK_DEFAULT_PAYLOAD_SIZE 64
#define NCV_CHUNK_DEFAULT_SIZE (NCV_CHUNK_DEFAULT_PAYLOAD_SIZE + 4)

typedef uint32_t oa_regid_t;

#define OA_MAKE_REGID(mms, addr) \
    (((uint32_t)(mms) << 16) | ((uint32_t)(addr) & 0xFFFF))

#define OA_REGID_GET_MMS(regid) ((uint8_t)((regid >> 16) & 0xF))
#define OA_REGID_GET_ADDR(regid) ((uint16_t)(regid & 0xFFFF))

#define OA_IDVER_MMS              0
#define OA_IDVER_ADDR             0x0U
#define OA_IDVER_REGID            OA_MAKE_REGID(OA_IDVER_MMS, OA_IDVER_ADDR)

#define OA_PHYID_MMS              0
#define OA_PHYID_ADDR             0x1U
#define OA_PHYID_REGID            OA_MAKE_REGID(OA_PHYID_MMS, OA_PHYID_ADDR)
#define OA_PHYID_OUI_MASK         GENMASK(31, 10)
#define OA_PHYID_OUI_POS          10
#define OA_PHYID_MODEL_MASK       GENMASK(9, 4)
#define OA_PHYID_MODEL_POS        4
#define OA_PHYID_REV_MASK         GENMASK(3, 0)
#define OA_PHYID_REV_POS          0

#define OA_STDCAP_MMS             0
#define OA_STDCAP_ADDR            0x2U
#define OA_STDCAP_REGID           OA_MAKE_REGID(OA_STDCAP_MMS, OA_STDCAP_ADDR)

#define OA_RESET_MMS              0
#define OA_RESET_ADDR             0x3U
#define OA_RESET_REGID            OA_MAKE_REGID(OA_RESET_MMS, OA_RESET_ADDR)
#define OA_RESET_SWRESET_MASK     BIT(0)
#define OA_RESET_SWRESET_POS      0

#define OA_CONFIG0_MMS            0
#define OA_CONFIG0_ADDR           0x4U
#define OA_CONFIG0_REGID          OA_MAKE_REGID(OA_CONFIG0_MMS, OA_CONFIG0_ADDR)
#define OA_CONFIG0_SYNC_MASK      BIT(15)
#define OA_CONFIG0_SYNC_POS       15
#define OA_CONFIG0_TXFCSVE_MASK   BIT(14)
#define OA_CONFIG0_TXFCSVE_POS    14
#define OA_CONFIG0_CSARFE_MASK    BIT(13)
#define OA_CONFIG0_CSARFE_POS     13
#define OA_CONFIG0_ZARFE_MASK     BIT(12)
#define OA_CONFIG0_ZARFE_POS      12
#define OA_CONFIG0_TXCTHRESH_MASK GENMASK(11, 10)
#define OA_CONFIG0_TXCTHRESH_POS  10
#define OA_CONFIG0_TXCTE_MASK     BIT(9)
#define OA_CONFIG0_TXCTE_POS      9
#define OA_CONFIG0_RXCTE_MASK     BIT(8)
#define OA_CONFIG0_RXCTE_POS      8
#define OA_CONFIG0_FTSE_MASK      BIT(7)
#define OA_CONFIG0_FTSE_POS       7
#define OA_CONFIG0_FTSS_MASK      BIT(6)
#define OA_CONFIG0_FTSS_POS       6
#define OA_CONFIG0_PROTE_MASK     BIT(5)
#define OA_CONFIG0_PROTE_POS      5
#define OA_CONFIG0_SEQE_MASK      BIT(4)
#define OA_CONFIG0_SEQE_POS       4
#define OA_CONFIG0_CPS_MASK       GENMASK(2, 0)
#define OA_CONFIG0_CPS_POS        0

#define OA_STATUS0_MMS            0
#define OA_STATUS0_ADDR           0x8U
#define OA_STATUS0_REGID          OA_MAKE_REGID(OA_STATUS0_MMS, OA_STATUS0_ADDR)
#define OA_STATUS0_RESETC_MASK    BIT(6)
#define OA_STATUS0_RESETC_POS     6
#define OA_STATUS0_HDRE_MASK      BIT(5)
#define OA_STATUS0_HDRE_POS       5

#define OA_BUFSTS_MMS             0
#define OA_BUFSTS_ADDR            0xBU
#define OA_BUFSTS_REGID           OA_MAKE_REGID(OA_BUFSTS_MMS, OA_BUFSTS_ADDR)

#define OA_IMSK0_MMS              0
#define OA_IMSK0_ADDR             0xCU
#define OA_IMSK0_REGID            OA_MAKE_REGID(OA_IMSK0_MMS, OA_IMSK0_ADDR)
#define OA_IMSK0_DEF              0x1FBFU
#define OA_IMSK0_PHYINTM_MASK     BIT(7)
#define OA_IMSK0_PHYINTM_POS      7
#define OA_IMSK0_RXBOEM_MASK      BIT(3)
#define OA_IMSK0_RXBOEM_POS       3

#define OA_PHY_CONTROL_MMS        0
#define OA_PHY_CONTROL_ADDR       0xFF00U
#define OA_PHY_CONTROL_REGID      OA_MAKE_REGID(OA_PHY_CONTROL_MMS, OA_PHY_CONTROL_ADDR)
#define OA_PHY_CONTROL_LCTL_POS   12

#define OA_PHY_STATUS_MMS         0
#define OA_PHY_STATUS_ADDR        0xFF01U
#define OA_PHY_STATUS_REGID       OA_MAKE_REGID(OA_PHY_STATUS_MMS, OA_PHY_STATUS_ADDR)

/* registers specific to ncv7410 */

#define NCV_MAC_CONTROL0_MMS      1
#define NCV_MAC_CONTROL0_ADDR     0x0U
#define NCV_MAC_CONTROL0_REGID    OA_MAKE_REGID(NCV_MAC_CONTROL0_MMS, NCV_MAC_CONTROL0_ADDR)
#define NCV_MAC_CONTROL0_ADRF_POS 16
#define NCV_MAC_CONTROL0_FCSA_POS 8
#define NCV_MAC_CONTROL0_TXEN_POS 1
#define NCV_MAC_CONTROL0_RXEN_POS 0

#define NCV_ADDRFILT0L_MMS        1
#define NCV_ADDRFILT0L_ADDR       0x10U
#define NCV_ADDRFILT0L_REGID      OA_MAKE_REGID(NCV_ADDRFILT0L_MMS, NCV_ADDRFILT0L_ADDR)

#define NCV_ADDRFILT0H_MMS        1
#define NCV_ADDRFILT0H_ADDR       0x11U
#define NCV_ADDRFILT0H_REGID      OA_MAKE_REGID(NCV_ADDRFILT0H_MMS, NCV_ADDRFILT0H_ADDR)

#define NCV_ADDRMASK0L_MMS        1
#define NCV_ADDRMASK0L_ADDR       0x20U
#define NCV_ADDRMASK0L_REGID      OA_MAKE_REGID(NCV_ADDRMASK0L_MMS, NCV_ADDRMASK0L_ADDR)

#define NCV_ADDRMASK0H_MMS        1
#define NCV_ADDRMASK0H_ADDR       0x21U
#define NCV_ADDRMASK0H_REGID      OA_MAKE_REGID(NCV_ADDRMASK0H_MMS, NCV_ADDRMASK0H_ADDR)

#define NCV_DIO_CONFIG_MMS        12
#define NCV_DIO_CONFIG_ADDR       0x0012U
#define NCV_DIO_CONFIG_REGID      OA_MAKE_REGID(NCV_DIO_CONFIG_MMS, NCV_DIO_CONFIG_ADDR)
#define NCV_DIO_CONFIG_DEF        0x6060
#define NCV_DIO0_FUNC_POS         1
#define NCV_DIO1_FUNC_POS         9
#define NCV_DIO0_OUT_VAL_POS      0
#define NCV_DIO1_OUT_VAL_POS      8
#define NCV_DIO_TRISTATE_FUNC     0x0
#define NCV_DIO_GPIO_FUNC         0x1
#define NCV_DIO_SFD_TX_FUNC       0x2
#define NCV_DIO_SFD_RX_FUNC       0x3
#define NCV_DIO_LINK_CTRL_FUNC    0x4
#define NCV_DIO_SFD_TXRX_FUNC     0xB
#define NCV_DIO_TXRX_FUNC         0xF

#define NCV_MACID0_MMS            12
#define NCV_MACID0_ADDR           0x1002U
#define NCV_MACID0_REGID          OA_MAKE_REGID(NCV_MACID0_MMS, NCV_MACID0_ADDR)
#define NCV_MACID0_MASK           GENMASK(15, 0)
#define NCV_MACID0_POS            0

#define NCV_MACID1_MMS            12
#define NCV_MACID1_ADDR           0x1003U
#define NCV_MACID1_REGID          OA_MAKE_REGID(NCV_MACID1_MMS, NCV_MACID1_ADDR)
#define NCV_MACID1_MASK           GENMASK(7, 0)
#define NCV_MACID1_POS            0

/* OA Data Transaction and Control Transaction protocols bitfields */

/* Common bitfields */

#define OA_DNC_MASK  BIT(31)
#define OA_DNC_POS   31

#define OA_HDRB_MASK BIT(30)
#define OA_HDRB_POS  30

#define OA_VS_MASK   GENMASK(23, 22)
#define OA_VS_POS    22

#define OA_DV_MASK   BIT(21)
#define OA_DV_POS    21

#define OA_SV_MASK   BIT(20)
#define OA_SV_POS    20

#define OA_SWO_MASK  GENMASK(19, 16)
#define OA_SWO_POS   16

#define OA_EV_MASK   BIT(14)
#define OA_EV_POS    14

#define OA_EBO_MASK  GENMASK(13, 8)
#define OA_EBO_POS   8

#define OA_P_MASK    BIT(0)
#define OA_P_POS     0

/* Control Transaction Protocol header bitfields */

#define OA_WNR_MASK  BIT(29)
#define OA_WNR_POS   29

#define OA_AID_MASK  BIT(28)
#define OA_AID_POS   28

#define OA_MMS_MASK  GENMASK(27, 24)
#define OA_MMS_POS   24

#define OA_ADDR_MASK GENMASK(23, 8)
#define OA_ADDR_POS  8

#define OA_LEN_MASK  GENMASK(7, 1)
#define OA_LEN_POS   1

/* Transmit data header bitfields */

#define OA_SEQ_MASK  BIT(30)
#define OA_SEQ_POS   30

#define OA_NORX_MASK BIT(29)
#define OA_NORX_POS  29

#define OA_TSC_MASK  GENMASK(7, 6)
#define OA_TSC_POS   6

/* Receive data footer bitfields */

#define OA_EXST_MASK BIT(31)
#define OA_EXST_POS  31

#define OA_SYNC_MASK BIT(29)
#define OA_SYNC_POS  29

#define OA_RCA_MASK  GENMASK(28, 24)
#define OA_RCA_POS   24

#define OA_FD_MASK   BIT(15)
#define OA_FD_POS    15

#define OA_RTSA_MASK BIT(7)
#define OA_RTSA_POS  7

#define OA_RTSP_MASK BIT(6)
#define OA_RTSP_POS  6

#define OA_TXC_MASK  GENMASK(5, 1)
#define OA_TXC_POS   1

#define _oa_control_field(f, fieldname) \
    ((int) ((f & OA_##fieldname##_MASK) >> OA_##fieldname##_POS))

#define oa_tx_credits(f)                _oa_control_field(f, TXC)
#define oa_rx_available(f)              _oa_control_field(f, RCA)
#define oa_header_bad(f)                _oa_control_field(f, HDRB)
#define oa_ext_status(f)                _oa_control_field(f, EXST)
#define oa_data_valid(f)                _oa_control_field(f, DV)
#define oa_start_valid(f)               _oa_control_field(f, SV)
#define oa_start_word_offset(f)         _oa_control_field(f, SWO)
#define oa_end_valid(f)                 _oa_control_field(f, EV)
#define oa_end_byte_offset(f)           _oa_control_field(f, EBO)
#define oa_frame_drop(f)                _oa_control_field(f, FD)
#define oa_rx_frame_timestamp_added(f)  _oa_control_field(f, RTSA)
#define oa_rx_frame_timestamp_parity(f) _oa_control_field(f, RTSP)
#define oa_mac_phy_sync(f)              _oa_control_field(f, SYNC)

#endif /* __DRIVERS_NET_NCV7410_H */
