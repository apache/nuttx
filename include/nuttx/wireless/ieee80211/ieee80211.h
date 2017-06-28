/****************************************************************************
 * include/nuttx/wireless/ieee80211/ieee80211.h
 * 802.11 protocol definitions.
 *
 * Copyright (c) 2002, 2003 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE80211_IEEE80211_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE80211_IEEE80211_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IEEE80211_ADDR_LEN          6  /* size of 802.11 address */

/* is 802.11 address multicast/broadcast? */

#define IEEE80211_IS_MULTICAST(_a)  (*(_a) & 0x01)

/* htframe */

#define IEEE80211_FC0_VERSION_MASK          0x03
#define IEEE80211_FC0_VERSION_SHIFT         0
#define IEEE80211_FC0_VERSION_0             0x00
#define IEEE80211_FC0_TYPE_MASK             0x0c
#define IEEE80211_FC0_TYPE_SHIFT            2
#define IEEE80211_FC0_TYPE_MGT              0x00
#define IEEE80211_FC0_TYPE_CTL              0x04
#define IEEE80211_FC0_TYPE_DATA             0x08

#define IEEE80211_FC0_SUBTYPE_MASK          0xf0
#define IEEE80211_FC0_SUBTYPE_SHIFT         4

/* for TYPE_MGT */

#define IEEE80211_FC0_SUBTYPE_ASSOC_REQ     0x00
#define IEEE80211_FC0_SUBTYPE_ASSOC_RESP    0x10
#define IEEE80211_FC0_SUBTYPE_REASSOC_REQ   0x20
#define IEEE80211_FC0_SUBTYPE_REASSOC_RESP  0x30
#define IEEE80211_FC0_SUBTYPE_PROBE_REQ     0x40
#define IEEE80211_FC0_SUBTYPE_PROBE_RESP    0x50
#define IEEE80211_FC0_SUBTYPE_BEACON        0x80
#define IEEE80211_FC0_SUBTYPE_ATIM          0x90
#define IEEE80211_FC0_SUBTYPE_DISASSOC      0xa0
#define IEEE80211_FC0_SUBTYPE_AUTH          0xb0
#define IEEE80211_FC0_SUBTYPE_DEAUTH        0xc0
#define IEEE80211_FC0_SUBTYPE_ACTION        0xd0
#define IEEE80211_FC0_SUBTYPE_ACTION_NOACK  0xe0  /* 11n */

/* for TYPE_CTL */

#define IEEE80211_FC0_SUBTYPE_WRAPPER       0x70  /* 11n */
#define IEEE80211_FC0_SUBTYPE_BAR           0x80
#define IEEE80211_FC0_SUBTYPE_BA            0x90
#define IEEE80211_FC0_SUBTYPE_PS_POLL       0xa0
#define IEEE80211_FC0_SUBTYPE_RTS           0xb0
#define IEEE80211_FC0_SUBTYPE_CTS           0xc0
#define IEEE80211_FC0_SUBTYPE_ACK           0xd0
#define IEEE80211_FC0_SUBTYPE_CF_END        0xe0
#define IEEE80211_FC0_SUBTYPE_CF_END_ACK    0xf0

/* for TYPE_DATA (bit combination) */

#define IEEE80211_FC0_SUBTYPE_DATA          0x00
#define IEEE80211_FC0_SUBTYPE_CF_ACK        0x10
#define IEEE80211_FC0_SUBTYPE_CF_POLL       0x20
#define IEEE80211_FC0_SUBTYPE_CF_ACPL       0x30
#define IEEE80211_FC0_SUBTYPE_NODATA        0x40
#define IEEE80211_FC0_SUBTYPE_CFACK         0x50
#define IEEE80211_FC0_SUBTYPE_CFPOLL        0x60
#define IEEE80211_FC0_SUBTYPE_CF_ACK_CF_ACK 0x70
#define IEEE80211_FC0_SUBTYPE_QOS           0x80

#define IEEE80211_FC1_DIR_MASK              0x03
#define IEEE80211_FC1_DIR_NODS              0x00  /* STA->STA */
#define IEEE80211_FC1_DIR_TODS              0x01  /* STA->AP */
#define IEEE80211_FC1_DIR_FROMDS            0x02
                                                  /* AP ->STA */
#define IEEE80211_FC1_DIR_DSTODS            0x03
                                                  /* AP ->AP */

#define IEEE80211_FC1_MORE_FRAG             0x04
#define IEEE80211_FC1_RETRY                 0x08
#define IEEE80211_FC1_PWR_MGT               0x10
#define IEEE80211_FC1_MORE_DATA             0x20
#define IEEE80211_FC1_PROTECTED             0x40
#define IEEE80211_FC1_WEP                   0x40  /* pre-RSNA compat */
#define IEEE80211_FC1_ORDER                 0x80
#define IEEE80211_FC1_BITS \
    "\20\03MORE_FRAG\04RETRY\05PWR_MGT\06MORE_DATA" \
    "\07PROTECTED\08ORDER"

/* Sequence Control field (see 7.1.3.4). */

#define IEEE80211_SEQ_FRAG_MASK            0x000f
#define IEEE80211_SEQ_FRAG_SHIFT           0
#define IEEE80211_SEQ_SEQ_MASK             0xfff0
#define IEEE80211_SEQ_SEQ_SHIFT            4

#define IEEE80211_NWID_LEN                 32
#define IEEE80211_MMIE_LEN                 18  /* 11w */

/* QoS Control field (see 7.1.3.5). */

#define IEEE80211_QOS_TXOP                 0xff00
#define IEEE80211_QOS_AMSDU                0x0080  /* 11n */
#define IEEE80211_QOS_ACK_POLICY_NORMAL    0x0000
#define IEEE80211_QOS_ACK_POLICY_NOACK     0x0020
#define IEEE80211_QOS_ACK_POLICY_NOEXPLACK 0x0040
#define IEEE80211_QOS_ACK_POLICY_BA        0x0060
#define IEEE80211_QOS_ACK_POLICY_MASK      0x0060
#define IEEE80211_QOS_ACK_POLICY_SHIFT     5
#define IEEE80211_QOS_EOSP                 0x0010
#define IEEE80211_QOS_TID                  0x000f

/* Capability Information field (see 7.3.1.4) */

#define IEEE80211_CAPINFO_ESS              0x0001
#define IEEE80211_CAPINFO_IBSS             0x0002
#define IEEE80211_CAPINFO_CF_POLLABLE      0x0004
#define IEEE80211_CAPINFO_CF_POLLREQ       0x0008
#define IEEE80211_CAPINFO_PRIVACY          0x0010
#define IEEE80211_CAPINFO_SHORT_PREAMBLE   0x0020
#define IEEE80211_CAPINFO_PBCC             0x0040
#define IEEE80211_CAPINFO_CHNL_AGILITY     0x0080
#define IEEE80211_CAPINFO_SPECTRUM_MGMT    0x0100
#define IEEE80211_CAPINFO_QOS              0x0200
#define IEEE80211_CAPINFO_SHORT_SLOTTIME   0x0400
#define IEEE80211_CAPINFO_APSD             0x0800

/* bit 12 is reserved */

#define IEEE80211_CAPINFO_DSSSOFDM         0x2000
#define IEEE80211_CAPINFO_DELAYED_B_ACK    0x4000
#define IEEE80211_CAPINFO_IMMEDIATE_B_ACK  0x8000
#define IEEE80211_CAPINFO_BITS \
    "\10\01ESS\02IBSS\03CF_POLLABLE\04CF_POLLREQ" \
    "\05PRIVACY\06SHORT_PREAMBLE\07PBCC\10CHNL_AGILITY" \
    "\11SPECTRUM_MGMT\12QOS\13SHORT_SLOTTIME\14APSD" \
    "\16DSSSOFDM\17DELAYED_B_ACK\20IMMEDIATE_B_ACK"

/* Block Ack Action field values (see Table 7-54). */

#define IEEE80211_ACTION_ADDBA_REQ         0
#define IEEE80211_ACTION_ADDBA_RESP        1
#define IEEE80211_ACTION_DELBA             2

/* SA Query Action field values (see Table 7-57l). */

#define IEEE80211_ACTION_SA_QUERY_REQ      0
#define IEEE80211_ACTION_SA_QUERY_RESP     1

/* HT Action field values (see Table 7-57m). */

#define IEEE80211_ACTION_NOTIFYCW          0

#define IEEE80211_RATE_BASIC               0x80
#define IEEE80211_RATE_VAL                 0x7f
#define IEEE80211_RATE_SIZE                8    /* 802.11 standard */
#define IEEE80211_RATE_MAXSIZE             15   /* max rates we'll handle */

/* BlockAck/BlockAckReq Control field (see Figure 7-13). */

#define IEEE80211_BA_ACK_POLICY            0x0001
#define IEEE80211_BA_MULTI_TID             0x0002
#define IEEE80211_BA_COMPRESSED            0x0004
#define IEEE80211_BA_TID_INFO_MASK         0xf000
#define IEEE80211_BA_TID_INFO_SHIFT        12

/* DELBA Parameter Set field (see Figure 7-34). */

#define IEEE80211_DELBA_INITIATOR          0x0800

/* ERP information element (see 7.3.2.13). */

#define IEEE80211_ERP_NON_ERP_PRESENT      0x01
#define IEEE80211_ERP_USE_PROTECTION       0x02
#define IEEE80211_ERP_BARKER_MODE          0x04

/* RSN capabilities (see 7.3.2.25.3). */

#define IEEE80211_RSNCAP_PREAUTH           0x0001
#define IEEE80211_RSNCAP_NOPAIRWISE        0x0002
#define IEEE80211_RSNCAP_PTKSA_RCNT_MASK   0x000c
#define IEEE80211_RSNCAP_PTKSA_RCNT_SHIFT  2
#define IEEE80211_RSNCAP_GTKSA_RCNT_MASK   0x0030
#define IEEE80211_RSNCAP_GTKSA_RCNT_SHIFT  4
#define IEEE80211_RSNCAP_RCNT1             0
#define IEEE80211_RSNCAP_RCNT2             1
#define IEEE80211_RSNCAP_RCNT4             2
#define IEEE80211_RSNCAP_RCNT16            3
#define IEEE80211_RSNCAP_MFPR              0x0040  /* 11w */
#define IEEE80211_RSNCAP_MFPC              0x0080  /* 11w */
#define IEEE80211_RSNCAP_PEERKEYENA        0x0200
#define IEEE80211_RSNCAP_SPPAMSDUC         0x0400  /* 11n */
#define IEEE80211_RSNCAP_SPPAMSDUR         0x0800  /* 11n */
#define IEEE80211_RSNCAP_PBAC              0x1000  /* 11n */

/* HT Capabilities Info (see 7.3.2.57.2). */

#define IEEE80211_HTCAP_LDPC               0x00000001
#define IEEE80211_HTCAP_CBW20_40           0x00000002
#define IEEE80211_HTCAP_SMPS_MASK          0x0000000c
#define IEEE80211_HTCAP_SMPS_SHIFT         2
#define IEEE80211_HTCAP_SMPS_STA           0
#define IEEE80211_HTCAP_SMPS_DYN           1
#define IEEE80211_HTCAP_SMPS_DIS           3
#define IEEE80211_HTCAP_GF                 0x00000010
#define IEEE80211_HTCAP_SGI20              0x00000020
#define IEEE80211_HTCAP_SGI40              0x00000040
#define IEEE80211_HTCAP_TXSTBC             0x00000080
#define IEEE80211_HTCAP_RXSTBC_MASK        0x00000300
#define IEEE80211_HTCAP_RXSTBC_SHIFT       8
#define IEEE80211_HTCAP_DELAYEDBA          0x00000400
#define IEEE80211_HTCAP_AMSDU7935          0x00000800
#define IEEE80211_HTCAP_DSSSCCK40          0x00001000
#define IEEE80211_HTCAP_PSMP               0x00002000
#define IEEE80211_HTCAP_40INTOLERANT       0x00004000
#define IEEE80211_HTCAP_LSIGTXOPPROT       0x00008000

/* HT Extended Capabilities (see 7.3.2.57.5).*/

#define IEEE80211_HTXCAP_PCO               0x0001
#define IEEE80211_HTXCAP_PCOTT_MASK        0x0006
#define IEEE80211_HTXCAP_PCOTT_SHIFT       1
#define IEEE80211_HTXCAP_PCOTT_400         1
#define IEEE80211_HTXCAP_PCOTT_1500        2
#define IEEE80211_HTXCAP_PCOTT_5000        3

/* Bits 3-7 are reserved. */

#define IEEE80211_HTXCAP_MFB_MASK          0x0300
#define IEEE80211_HTXCAP_MFB_SHIFT         8
#define IEEE80211_HTXCAP_MFB_NONE          0
#define IEEE80211_HTXCAP_MFB_UNSOL         2
#define IEEE80211_HTXCAP_MFB_BOTH          3
#define IEEE80211_HTXCAP_HTC               0x0400
#define IEEE80211_HTXCAP_RDRESP            0x0800

/* Bits 12-15 are reserved. */

/* Transmit Beamforming (TxBF) Capabilities (see 7.3.2.57.6). */

#define IEEE80211_TXBFCAP_IMPLICIT_RX      0x00000001
#define IEEE80211_TXBFCAP_RSSC             0x00000002
#define IEEE80211_TXBFCAP_TSSC             0x00000004
#define IEEE80211_TXBFCAP_RNDP             0x00000008
#define IEEE80211_TXBFCAP_TNDP             0x00000010
#define IEEE80211_TXBFCAP_IMPLICIT_TX      0x00000020
#define IEEE80211_TXBFCAP_CALIB_MASK       0x000000c0
#define IEEE80211_TXBFCAP_CALIB_SHIFT      6
#define IEEE80211_TXBFCAP_TX_CSI           0x00000100

/* Antenna Selection (ASEL) Capability (see 7.3.2.57.7). */

#define IEEE80211_ASELCAP_ASEL             0x01
#define IEEE80211_ASELCAP_CSIFB            0x02

/* Bit 7 is reserved. */

/* HT Operation element (see 7.3.2.58). */

/* Byte 1. */

#define IEEE80211_HTOP0_SCO_MASK           0x03
#define IEEE80211_HTOP0_SCO_SHIFT          0
#define IEEE80211_HTOP0_SCO_SCN            0
#define IEEE80211_HTOP0_SCO_SCA            1
#define IEEE80211_HTOP0_SCO_SCB            3
#define IEEE80211_HTOP0_CHW                0x04
#define IEEE80211_HTOP0_RIFS               0x08
#define IEEE80211_HTOP0_SPSMP              0x10
#define IEEE80211_HTOP0_SIG_MASK           0xe0
#define IEEE80211_HTOP0_SIG_SHIFT          5

/* Bytes 2-3. */

#define IEEE80211_HTOP1_PROT_MASK          0x0003
#define IEEE80211_HTOP1_PROT_SHIFT         0
#define IEEE80211_HTOP1_NONGTSTA           0x0004

/* Bit 3 is reserved. */

#define IEEE80211_HTOP1_OBSS_NONHTSTA      0x0010

/* Bits 5-15 are reserved. */

/* Bytes 4-5. */

/* Bits 0-5 are reserved. */

#define IEEE80211_HTOP2_DUALBEACON         0x0040
#define IEEE80211_HTOP2_DUALCTSPROT        0x0080
#define IEEE80211_HTOP2_STBCBEACON         0x0100
#define IEEE80211_HTOP2_LSIGTXOP           0x0200
#define IEEE80211_HTOP2_PCOACTIVE          0x0400
#define IEEE80211_HTOP2_PCOPHASE40         0x0800

/* Bits 12-15 are reserved. */

/* EDCA Access Categories. */

#define EDCA_NUM_AC                        4

/* Number of TID values (traffic identifier) */

#define IEEE80211_NUM_TID                  16

/* Atheros private advanced capabilities info */

#define ATHEROS_CAP_TURBO_PRIME            0x01
#define ATHEROS_CAP_COMPRESSION            0x02
#define ATHEROS_CAP_FAST_FRAME             0x04

/* bits 3-6 reserved */

#define ATHEROS_CAP_BOOST                  0x80

/* Organizationally Unique Identifiers.
 * See http://standards.ieee.org/regauth/oui/oui.txt for a list.
 */

#define ATHEROS_OUI                        ((const uint8_t[]){ 0x00, 0x03, 0x7f })
#define BROADCOM_OUI                       ((const uint8_t[]){ 0x00, 0x90, 0x4c })
#define IEEE80211_OUI                      ((const uint8_t[]){ 0x00, 0x0f, 0xac })
#define MICROSOFT_OUI                     ((const uint8_t[]){ 0x00, 0x50, 0xf2 })

#define IEEE80211_AUTH_ALGORITHM(auth) \
    ((auth)[0] | ((auth)[1] << 8))
#define IEEE80211_AUTH_TRANSACTION(auth) \
    ((auth)[2] | ((auth)[3] << 8))
#define IEEE80211_AUTH_STATUS(auth) \
    ((auth)[4] | ((auth)[5] << 8))

/* Authentication Algorithm Number field (see 7.3.1.1). */

#define IEEE80211_AUTH_ALG_OPEN            0x0000
#define IEEE80211_AUTH_ALG_SHARED          0x0001
#define IEEE80211_AUTH_ALG_LEAP            0x0080

/* WEP */

#define IEEE80211_WEP_KEYLEN               5   /* 40bit */
#define IEEE80211_WEP_NKID                 4   /* number of key ids */
#define IEEE80211_CHALLENGE_LEN            128

/* WEP header constants */

#define IEEE80211_WEP_IVLEN                3   /* 24bit */
#define IEEE80211_WEP_KIDLEN               1   /* 1 octet */
#define IEEE80211_WEP_CRCLEN               4   /* CRC-32 */
#define IEEE80211_CRC_LEN                  4
#define IEEE80211_WEP_TOTLEN \
    (IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN + IEEE80211_WEP_CRCLEN)

/* 802.11i defines an extended IV for use with non-WEP ciphers.
 * When the EXTIV bit is set in the key id byte an additional
 * 4 bytes immediately follow the IV for TKIP.  For CCMP the
 * EXTIV bit is likewise set but the 8 bytes represent the
 * CCMP header rather than IV+extended-IV.
 */

#define IEEE80211_WEP_EXTIV                0x20
#define IEEE80211_WEP_EXTIVLEN             4    /* extended IV length */
#define IEEE80211_WEP_MICLEN               8    /* trailing MIC */

/* Maximum acceptable MTU is:
 *    IEEE80211_MAX_LEN - WEP overhead - CRC -
 *        QoS overhead - RSN/WPA overhead
 * Min is arbitrarily chosen > IEEE80211_MIN_LEN.  The default
 * mtu is Ethernet-compatible.
 */

#define IEEE80211_MTU_MAX                  2290
#define IEEE80211_MTU_MIN                  32

#define IEEE80211_MAX_LEN \
    (2300 + IEEE80211_CRC_LEN + \
    (IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN + IEEE80211_WEP_CRCLEN))
#define IEEE80211_ACK_LEN \
    (sizeof(struct ieee80211_frame_ack) + IEEE80211_CRC_LEN)
#define IEEE80211_MIN_LEN \
    (sizeof(struct ieee80211_frame_min) + IEEE80211_CRC_LEN)

/* The 802.11 spec says at most 2007 stations may be
 * associated at once.  For most AP's this is way more
 * than is feasible so we use a default of 1800. This
 * number may be overridden by the driver and/or by
 * user configuration.
 */

#define IEEE80211_AID_MAX                  2007
#define IEEE80211_AID_DEF                  1800
#define IEEE80211_AID(b)                   ((b) &~ 0xc000)

/* RTS frame length parameters.  The default is specified in
 * the 802.11 spec.  The max may be wrong for jumbo frames.
 */

#define IEEE80211_RTS_DEFAULT              512
#define IEEE80211_RTS_MIN                  1
#define IEEE80211_RTS_MAX                  IEEE80211_MAX_LEN

#define IEEE80211_PLCP_SERVICE             0x00
#define IEEE80211_PLCP_SERVICE_PBCC        0x08   /* PBCC encoded */
#define IEEE80211_PLCP_SERVICE_LENEXT      0x80 /* length extension bit */

/* One Time Unit (TU) is 1Kus = 1024 microseconds. */

#define IEEE80211_DUR_TU                   1024

/* IEEE 802.11b durations for DSSS PHY in microseconds */

#define IEEE80211_DUR_DS_LONG_PREAMBLE     144
#define IEEE80211_DUR_DS_SHORT_PREAMBLE    72
#define IEEE80211_DUR_DS_PREAMBLE_DIFFERENCE \
    (IEEE80211_DUR_DS_LONG_PREAMBLE - IEEE80211_DUR_DS_SHORT_PREAMBLE)
#define IEEE80211_DUR_DS_FAST_PLCPHDR      24
#define IEEE80211_DUR_DS_SLOW_PLCPHDR      48
#define IEEE80211_DUR_DS_PLCPHDR_DIFFERENCE \
    (IEEE80211_DUR_DS_SLOW_PLCPHDR - IEEE80211_DUR_DS_FAST_PLCPHDR)
#define IEEE80211_DUR_DS_SLOW_ACK          112
#define IEEE80211_DUR_DS_FAST_ACK          56
#define IEEE80211_DUR_DS_SLOW_CTS          112
#define IEEE80211_DUR_DS_FAST_CTS          56
#define IEEE80211_DUR_DS_SLOT              20
#define IEEE80211_DUR_DS_SIFS              10
#define IEEE80211_DUR_DS_PIFS \
    (IEEE80211_DUR_DS_SIFS + IEEE80211_DUR_DS_SLOT)
#define IEEE80211_DUR_DS_DIFS \
    (IEEE80211_DUR_DS_SIFS + 2 * IEEE80211_DUR_DS_SLOT)
#define IEEE80211_DUR_DS_EIFS    (IEEE80211_DUR_DS_SIFS + \
    IEEE80211_DUR_DS_SLOW_ACK + \
    IEEE80211_DUR_DS_LONG_PREAMBLE + \
    IEEE80211_DUR_DS_SLOW_PLCPHDR + \
    IEEE80211_DUR_DIFS)

/* The RSNA key descriptor used by IEEE 802.11 does not use the IEEE 802.1X
 * key descriptor.  Instead, it uses the key descriptor described in 8.5.2.
 */

#define EAPOL_KEY_NONCE_LEN                32
#define EAPOL_KEY_IV_LEN                   16
#define EAPOL_KEY_MIC_LEN                  16

/* Pairwise Transient Key (see 8.5.1.2) */

#define IEEE80211_PMKID_LEN    16
#define IEEE80211_SMKID_LEN    16

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Generic definitions for IEEE 802.11 frames */

begin_packed_struct struct ieee80211_frame
{
  uint8_t i_fc[2];
  uint8_t i_dur[2];
  uint8_t i_addr1[IEEE80211_ADDR_LEN];
  uint8_t i_addr2[IEEE80211_ADDR_LEN];
  uint8_t i_addr3[IEEE80211_ADDR_LEN];
  uint8_t i_seq[2];
} end_packed_struct;

begin_packed_struct struct ieee80211_qosframe
{
  uint8_t i_fc[2];
  uint8_t i_dur[2];
  uint8_t i_addr1[IEEE80211_ADDR_LEN];
  uint8_t i_addr2[IEEE80211_ADDR_LEN];
  uint8_t i_addr3[IEEE80211_ADDR_LEN];
  uint8_t i_seq[2];
  uint8_t i_qos[2];
} end_packed_struct;

begin_packed_struct struct ieee80211_htframe  /* 11n */
{
  uint8_t i_fc[2];
  uint8_t i_dur[2];
  uint8_t i_addr1[IEEE80211_ADDR_LEN];
  uint8_t i_addr2[IEEE80211_ADDR_LEN];
  uint8_t i_addr3[IEEE80211_ADDR_LEN];
  uint8_t i_seq[2];
  uint8_t i_qos[2];
  uint8_t i_ht[4];
} end_packed_struct;

begin_packed_struct struct ieee80211_frame_addr4
{
  uint8_t i_fc[2];
  uint8_t i_dur[2];
  uint8_t i_addr1[IEEE80211_ADDR_LEN];
  uint8_t i_addr2[IEEE80211_ADDR_LEN];
  uint8_t i_addr3[IEEE80211_ADDR_LEN];
  uint8_t i_seq[2];
  uint8_t i_addr4[IEEE80211_ADDR_LEN];
} end_packed_struct;

begin_packed_struct struct ieee80211_qosframe_addr4
{
  uint8_t i_fc[2];
  uint8_t i_dur[2];
  uint8_t i_addr1[IEEE80211_ADDR_LEN];
  uint8_t i_addr2[IEEE80211_ADDR_LEN];
  uint8_t i_addr3[IEEE80211_ADDR_LEN];
  uint8_t i_seq[2];
  uint8_t i_addr4[IEEE80211_ADDR_LEN];
  uint8_t i_qos[2];
} end_packed_struct;

begin_packed_struct struct ieee80211_htframe_addr4  /* 11n */
{
  uint8_t i_fc[2];
  uint8_t i_dur[2];
  uint8_t i_addr1[IEEE80211_ADDR_LEN];
  uint8_t i_addr2[IEEE80211_ADDR_LEN];
  uint8_t i_addr3[IEEE80211_ADDR_LEN];
  uint8_t i_seq[2];
  uint8_t i_addr4[IEEE80211_ADDR_LEN];
  uint8_t i_qos[2];
  uint8_t i_ht[4];
} end_packed_struct;

/* Control frames. */

begin_packed_struct struct ieee80211_frame_min
{
    uint8_t i_fc[2];
    uint8_t i_dur[2];
    uint8_t i_addr1[IEEE80211_ADDR_LEN];
    uint8_t i_addr2[IEEE80211_ADDR_LEN];

    /* FCS */

} end_packed_struct;

begin_packed_struct struct ieee80211_frame_rts
{
    uint8_t i_fc[2];
    uint8_t i_dur[2];
    uint8_t i_ra[IEEE80211_ADDR_LEN];
    uint8_t i_ta[IEEE80211_ADDR_LEN];

    /* FCS */

} end_packed_struct;

struct ieee80211_frame_cts
{
    uint8_t i_fc[2];
    uint8_t i_dur[2];
    uint8_t i_ra[IEEE80211_ADDR_LEN];

    /* FCS */

} end_packed_struct;

struct ieee80211_frame_ack
{
    uint8_t i_fc[2];
    uint8_t i_dur[2];
    uint8_t i_ra[IEEE80211_ADDR_LEN];

    /* FCS */

} end_packed_struct;

struct ieee80211_frame_pspoll
{
    uint8_t i_fc[2];
    uint8_t i_aid[2];
    uint8_t i_bssid[IEEE80211_ADDR_LEN];
    uint8_t i_ta[IEEE80211_ADDR_LEN];

    /* FCS */

} end_packed_struct;

struct ieee80211_frame_cfend
{                             /* NB: also CF-End+CF-Ack */
  uint8_t i_fc[2];
  uint8_t i_dur[2];           /* should be zero */
  uint8_t i_ra[IEEE80211_ADDR_LEN];
  uint8_t i_bssid[IEEE80211_ADDR_LEN];

  /* FCS */

} end_packed_struct;

/* Information elements (see Table 7-26). */

enum
{
  IEEE80211_ELEMID_SSID      = 0,
  IEEE80211_ELEMID_RATES     = 1,
  IEEE80211_ELEMID_FHPARMS   = 2,
  IEEE80211_ELEMID_DSPARMS   = 3,
  IEEE80211_ELEMID_CFPARMS   = 4,
  IEEE80211_ELEMID_TIM       = 5,
  IEEE80211_ELEMID_IBSSPARMS = 6,
  IEEE80211_ELEMID_COUNTRY   = 7,
  IEEE80211_ELEMID_QBSS_LOAD = 11,
  IEEE80211_ELEMID_EDCAPARMS = 12,
  IEEE80211_ELEMID_CHALLENGE = 16,

  /* 17-31 reserved for challenge text extension */

  IEEE80211_ELEMID_ERP       = 42,
  IEEE80211_ELEMID_HTCAPS    = 45,  /* 11n */
  IEEE80211_ELEMID_QOS_CAP   = 46,
  IEEE80211_ELEMID_RSN       = 48,
  IEEE80211_ELEMID_XRATES    = 50,
  IEEE80211_ELEMID_TIE       = 56,  /* 11r */
  IEEE80211_ELEMID_HTOP      = 61,  /* 11n */
  IEEE80211_ELEMID_MMIE      = 76,  /* 11w */
  IEEE80211_ELEMID_TPC       = 150,
  IEEE80211_ELEMID_CCKM      = 156,
  IEEE80211_ELEMID_VENDOR    = 221  /* vendor private */
};

/* Action field category values (see Table 7-24). */

enum
{
  IEEE80211_CATEG_SPECTRUM   = 0,
  IEEE80211_CATEG_QOS        = 1,
  IEEE80211_CATEG_DLS        = 2,
  IEEE80211_CATEG_BA         = 3,
  IEEE80211_CATEG_HT         = 7,  /* 11n */
  IEEE80211_CATEG_SA_QUERY   = 8   /* 11w */
};

/* EDCA Access Categories. */

enum ieee80211_edca_ac
{
  EDCA_AC_BK                 = 1,  /* Background */
  EDCA_AC_BE                 = 0,  /* Best Effort */
  EDCA_AC_VI                 = 2,  /* Video */
  EDCA_AC_VO                 = 3   /* Voice */
};

/* Authentication Transaction Sequence Number field (see 7.3.1.2). */

enum
{
  IEEE80211_AUTH_OPEN_REQUEST  = 1,
  IEEE80211_AUTH_OPEN_RESPONSE = 2
};

enum
{
  IEEE80211_AUTH_SHARED_REQUEST   = 1,
  IEEE80211_AUTH_SHARED_CHALLENGE = 2,
  IEEE80211_AUTH_SHARED_RESPONSE  = 3,
  IEEE80211_AUTH_SHARED_PASS      = 4
};

/* Reason codes (see Table 22). */

enum
{
  IEEE80211_REASON_UNSPECIFIED         = 1,
  IEEE80211_REASON_AUTH_EXPIRE         = 2,
  IEEE80211_REASON_AUTH_LEAVE          = 3,
  IEEE80211_REASON_ASSOC_EXPIRE        = 4,
  IEEE80211_REASON_ASSOC_TOOMANY       = 5,
  IEEE80211_REASON_NOT_AUTHED          = 6,
  IEEE80211_REASON_NOT_ASSOCED         = 7,
  IEEE80211_REASON_ASSOC_LEAVE         = 8,
  IEEE80211_REASON_ASSOC_NOT_AUTHED    = 9,

  /* XXX the following two reason codes are not correct */

  IEEE80211_REASON_RSN_REQUIRED        = 11,
  IEEE80211_REASON_RSN_INCONSISTENT    = 12,

  IEEE80211_REASON_IE_INVALID          = 13,
  IEEE80211_REASON_MIC_FAILURE         = 14,
  IEEE80211_REASON_4WAY_TIMEOUT        = 15,
  IEEE80211_REASON_GROUP_TIMEOUT       = 16,
  IEEE80211_REASON_RSN_DIFFERENT_IE    = 17,
  IEEE80211_REASON_BAD_GROUP_CIPHER    = 18,
  IEEE80211_REASON_BAD_PAIRWISE_CIPHER = 19,
  IEEE80211_REASON_BAD_AKMP            = 20,
  IEEE80211_REASON_RSN_IE_VER_UNSUP    = 21,
  IEEE80211_REASON_RSN_IE_BAD_CAP      = 22,

  IEEE80211_REASON_CIPHER_REJ_POLICY   = 24,

  IEEE80211_REASON_SETUP_REQUIRED      = 38,
  IEEE80211_REASON_TIMEOUT             = 39
};

/* Status codes (see Table 23). */

enum
{
  IEEE80211_STATUS_SUCCESS             = 0,
  IEEE80211_STATUS_UNSPECIFIED         = 1,
  IEEE80211_STATUS_CAPINFO             = 10,
  IEEE80211_STATUS_NOT_ASSOCED         = 11,
  IEEE80211_STATUS_OTHER               = 12,
  IEEE80211_STATUS_ALG                 = 13,
  IEEE80211_STATUS_SEQUENCE            = 14,
  IEEE80211_STATUS_CHALLENGE           = 15,
  IEEE80211_STATUS_TIMEOUT             = 16,
  IEEE80211_STATUS_TOOMANY             = 17,
  IEEE80211_STATUS_BASIC_RATE          = 18,
  IEEE80211_STATUS_SP_REQUIRED         = 19,
  IEEE80211_STATUS_PBCC_REQUIRED       = 20,
  IEEE80211_STATUS_CA_REQUIRED         = 21,
  IEEE80211_STATUS_TOO_MANY_STATIONS   = 22,
  IEEE80211_STATUS_RATES               = 23,
  IEEE80211_STATUS_SHORTSLOT_REQUIRED  = 25,
  IEEE80211_STATUS_DSSSOFDM_REQUIRED   = 26,

  IEEE80211_STATUS_TRY_AGAIN_LATER     = 30,
  IEEE80211_STATUS_MFP_POLICY          = 31,

  IEEE80211_STATUS_REFUSED             = 37,
  IEEE80211_STATUS_INVALID_PARAM       = 38,

  IEEE80211_STATUS_IE_INVALID          = 40,
  IEEE80211_STATUS_BAD_GROUP_CIPHER    = 41,
  IEEE80211_STATUS_BAD_PAIRWISE_CIPHER = 42,
  IEEE80211_STATUS_BAD_AKMP            = 43,
  IEEE80211_STATUS_RSN_IE_VER_UNSUP    = 44,

  IEEE80211_STATUS_CIPHER_REJ_POLICY   = 46,
};

struct ieee80211_eapol_key
{
  uint8_t version;

#define EAPOL_VERSION               1

  uint8_t type;

/* IEEE Std 802.1X-2004, 7.5.4 (only type EAPOL-Key is used here) */

#define EAP_PACKET                  0
#define EAPOL_START                 1
#define EAPOL_LOGOFF                2
#define EAPOL_KEY                   3
#define EAPOL_ASF_ALERT             4

  uint8_t len[2];
  uint8_t desc;

/* IEEE Std 802.1X-2004, 7.6.1 */

#define EAPOL_KEY_DESC_RC4          1   /* Deprecated */
#define EAPOL_KEY_DESC_IEEE80211    2
#define EAPOL_KEY_DESC_WPA          254 /* Non-standard WPA */

  uint8_t info[2];

#define EAPOL_KEY_VERSION_MASK      0x7
#define EAPOL_KEY_DESC_V1           1
#define EAPOL_KEY_DESC_V2           2
#define EAPOL_KEY_DESC_V3           3         /* 11r */
#define EAPOL_KEY_PAIRWISE          (1 <<  3)
#define EAPOL_KEY_INSTALL           (1 <<  6) /* I */
#define EAPOL_KEY_KEYACK            (1 <<  7) /* A */
#define EAPOL_KEY_KEYMIC            (1 <<  8) /* M */
#define EAPOL_KEY_SECURE            (1 <<  9) /* S */
#define EAPOL_KEY_ERROR             (1 << 10)
#define EAPOL_KEY_REQUEST           (1 << 11)
#define EAPOL_KEY_ENCRYPTED         (1 << 12)
#define EAPOL_KEY_SMK               (1 << 13)

/* WPA compatibility */

#define EAPOL_KEY_WPA_KID_MASK      0x3
#define EAPOL_KEY_WPA_KID_SHIFT     4
#define EAPOL_KEY_WPA_TX            EAPOL_KEY_INSTALL

  uint8_t keylen[2];
  uint8_t replaycnt[8];
  uint8_t nonce[EAPOL_KEY_NONCE_LEN];
  uint8_t iv[EAPOL_KEY_IV_LEN];
  uint8_t rsc[8];
  uint8_t reserved[8];
  uint8_t mic[EAPOL_KEY_MIC_LEN];
  uint8_t paylen[2];
} end_packed_struct;

/* Pairwise Transient Key (see 8.5.1.2) */

struct ieee80211_ptk
{
  uint8_t kck[16];            /* Key Confirmation Key */
  uint8_t kek[16];            /* Key Encryption Key */
  uint8_t tk[32];             /* Temporal Key */
} end_packed_struct;

/* Key Data Encapsulation (see Table 62) */

enum
{
  IEEE80211_KDE_GTK        = 1,
  IEEE80211_KDE_MACADDR    = 3,
  IEEE80211_KDE_PMKID      = 4,
  IEEE80211_KDE_SMK        = 5,
  IEEE80211_KDE_NONCE      = 6,
  IEEE80211_KDE_LIFETIME   = 7,
  IEEE80211_KDE_ERROR      = 8,
  IEEE80211_KDE_IGTK       = 9  /* 11w */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline int ieee80211_has_seq(FAR const struct ieee80211_frame *wh)
{
  return (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_CTL;
}

static inline int ieee80211_has_addr4(FAR const struct ieee80211_frame *wh)
{
  return (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_DSTODS;
}

static inline int ieee80211_has_qos(FAR const struct ieee80211_frame *wh)
{
  return (wh->i_fc[0] &
          (IEEE80211_FC0_TYPE_MASK | IEEE80211_FC0_SUBTYPE_QOS)) ==
          (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS);
}

static inline int ieee80211_has_htc(FAR const struct ieee80211_frame *wh)
{
  return (wh->i_fc[1] & IEEE80211_FC1_ORDER) &&
          (ieee80211_has_qos(wh) ||
         (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_MGT);
}

static inline uint16_t ieee80211_get_qos(FAR const struct ieee80211_frame *wh)
{
  FAR const uint8_t *frm;

  if (ieee80211_has_addr4(wh))
    {
      frm = ((FAR const struct ieee80211_qosframe_addr4 *)wh)->i_qos;
    }
  else
    {
      frm = ((FAR const struct ieee80211_qosframe *)wh)->i_qos;
    }

  return letoh16(*(FAR const uint16_t *)frm);
}

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE80211_IEEE80211_H */
