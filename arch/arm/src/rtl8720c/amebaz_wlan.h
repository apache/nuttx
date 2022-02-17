/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_wlan.h
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

#ifndef __ARCH_ARM_SRC_RTL8720C_AMEBAZ_WLAN_H
#define __ARCH_ARM_SRC_RTL8720C_AMEBAZ_WLAN_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WEP_ENABLED             0x0001
#define TKIP_ENABLED            0x0002
#define AES_ENABLED             0x0004
#define WSEC_SWFLAG             0x0008
#define AES_CMAC_ENABLED        0x0010
#define SHARED_ENABLED          0x00008000
#define WPA_SECURITY            0x00200000
#define WPA2_SECURITY           0x00400000
#define WPS_ENABLED             0x10000000
#define PSCAN_ENABLE            0x01 /* enable for partial channel scan */
#define PSCAN_FAST_SURVEY       0x02 /* set to select scan time to FAST_SURVEY_TO, otherwise SURVEY_TO */
#define PSCAN_SIMPLE_CONFIG     0x04 /* set to select scan time to FAST_SURVEY_TO and resend probe request */

enum
{
  /* CHANNEL PLAN */

  RTW_COUNTRY_WORLD1,  /* 0x20 */
  RTW_COUNTRY_ETSI1,   /* 0x21 */
  RTW_COUNTRY_FCC1,    /* 0x22 */
  RTW_COUNTRY_MKK1,    /* 0x23 */
  RTW_COUNTRY_ETSI2,   /* 0x24 */
  RTW_COUNTRY_FCC2,    /* 0x2a */
  RTW_COUNTRY_WORLD2,  /* 0x47 */
  RTW_COUNTRY_MKK2,    /* 0x58 */
  RTW_COUNTRY_GLOBAL,  /* 0x41 */

  /* SPECIAL */

  RTW_COUNTRY_WORLD, /* WORLD1 */
  RTW_COUNTRY_EU,    /* ETSI1 */

  /* JAPANESE */

  RTW_COUNTRY_JP, /* MKK1 */

  /* FCC , 19 countries */

  RTW_COUNTRY_AS, /* FCC2 */
  RTW_COUNTRY_BM,
  RTW_COUNTRY_CA,
  RTW_COUNTRY_DM,
  RTW_COUNTRY_DO,
  RTW_COUNTRY_FM,
  RTW_COUNTRY_GD,
  RTW_COUNTRY_GT,
  RTW_COUNTRY_GU,
  RTW_COUNTRY_HT,
  RTW_COUNTRY_MH,
  RTW_COUNTRY_MP,
  RTW_COUNTRY_NI,
  RTW_COUNTRY_PA,
  RTW_COUNTRY_PR,
  RTW_COUNTRY_PW,
  RTW_COUNTRY_TW,
  RTW_COUNTRY_US,
  RTW_COUNTRY_VI,

  /* others,  ETSI */

  RTW_COUNTRY_AD, /* ETSI1 */
  RTW_COUNTRY_AE,
  RTW_COUNTRY_AF,
  RTW_COUNTRY_AI,
  RTW_COUNTRY_AL,
  RTW_COUNTRY_AM,
  RTW_COUNTRY_AN,
  RTW_COUNTRY_AR,
  RTW_COUNTRY_AT,
  RTW_COUNTRY_AU,
  RTW_COUNTRY_AW,
  RTW_COUNTRY_AZ,
  RTW_COUNTRY_BA,
  RTW_COUNTRY_BB,
  RTW_COUNTRY_BD,
  RTW_COUNTRY_BE,
  RTW_COUNTRY_BF,
  RTW_COUNTRY_BG,
  RTW_COUNTRY_BH,
  RTW_COUNTRY_BL,
  RTW_COUNTRY_BN,
  RTW_COUNTRY_BO,
  RTW_COUNTRY_BR,
  RTW_COUNTRY_BS,
  RTW_COUNTRY_BT,
  RTW_COUNTRY_BY,
  RTW_COUNTRY_BZ,
  RTW_COUNTRY_CF,
  RTW_COUNTRY_CH,
  RTW_COUNTRY_CI,
  RTW_COUNTRY_CL,
  RTW_COUNTRY_CN,
  RTW_COUNTRY_CO,
  RTW_COUNTRY_CR,
  RTW_COUNTRY_CX,
  RTW_COUNTRY_CY,
  RTW_COUNTRY_CZ,
  RTW_COUNTRY_DE,
  RTW_COUNTRY_DK,
  RTW_COUNTRY_DZ,
  RTW_COUNTRY_EC,
  RTW_COUNTRY_EE,
  RTW_COUNTRY_EG,
  RTW_COUNTRY_ES,
  RTW_COUNTRY_ET,
  RTW_COUNTRY_FI,
  RTW_COUNTRY_FR,
  RTW_COUNTRY_GB,
  RTW_COUNTRY_GE,
  RTW_COUNTRY_GF,
  RTW_COUNTRY_GH,
  RTW_COUNTRY_GL,
  RTW_COUNTRY_GP,
  RTW_COUNTRY_GR,
  RTW_COUNTRY_GY,
  RTW_COUNTRY_HK,
  RTW_COUNTRY_HN,
  RTW_COUNTRY_HR,
  RTW_COUNTRY_HU,
  RTW_COUNTRY_ID,
  RTW_COUNTRY_IE,
  RTW_COUNTRY_IL,
  RTW_COUNTRY_IN,
  RTW_COUNTRY_IQ,
  RTW_COUNTRY_IR,
  RTW_COUNTRY_IS,
  RTW_COUNTRY_IT,
  RTW_COUNTRY_JM,
  RTW_COUNTRY_JO,
  RTW_COUNTRY_KE,
  RTW_COUNTRY_KH,
  RTW_COUNTRY_KN,
  RTW_COUNTRY_KP,
  RTW_COUNTRY_KR,
  RTW_COUNTRY_KW,
  RTW_COUNTRY_KY,
  RTW_COUNTRY_KZ,
  RTW_COUNTRY_LA,
  RTW_COUNTRY_LB,
  RTW_COUNTRY_LC,
  RTW_COUNTRY_LI,
  RTW_COUNTRY_LK,
  RTW_COUNTRY_LR,
  RTW_COUNTRY_LS,
  RTW_COUNTRY_LT,
  RTW_COUNTRY_LU,
  RTW_COUNTRY_LV,
  RTW_COUNTRY_MA,
  RTW_COUNTRY_MC,
  RTW_COUNTRY_MD,
  RTW_COUNTRY_ME,
  RTW_COUNTRY_MF,
  RTW_COUNTRY_MK,
  RTW_COUNTRY_MN,
  RTW_COUNTRY_MO,
  RTW_COUNTRY_MQ,
  RTW_COUNTRY_MR,
  RTW_COUNTRY_MT,
  RTW_COUNTRY_MU,
  RTW_COUNTRY_MV,
  RTW_COUNTRY_MW,
  RTW_COUNTRY_MX,
  RTW_COUNTRY_MY,
  RTW_COUNTRY_NG,
  RTW_COUNTRY_NL,
  RTW_COUNTRY_NO,
  RTW_COUNTRY_NP,
  RTW_COUNTRY_NZ,
  RTW_COUNTRY_OM,
  RTW_COUNTRY_PE,
  RTW_COUNTRY_PF,
  RTW_COUNTRY_PG,
  RTW_COUNTRY_PH,
  RTW_COUNTRY_PK,
  RTW_COUNTRY_PL,
  RTW_COUNTRY_PM,
  RTW_COUNTRY_PT,
  RTW_COUNTRY_PY,
  RTW_COUNTRY_QA,
  RTW_COUNTRY_RS,
  RTW_COUNTRY_RU,
  RTW_COUNTRY_RW,
  RTW_COUNTRY_SA,
  RTW_COUNTRY_SE,
  RTW_COUNTRY_SG,
  RTW_COUNTRY_SI,
  RTW_COUNTRY_SK,
  RTW_COUNTRY_SN,
  RTW_COUNTRY_SR,
  RTW_COUNTRY_SV,
  RTW_COUNTRY_SY,
  RTW_COUNTRY_TC,
  RTW_COUNTRY_TD,
  RTW_COUNTRY_TG,
  RTW_COUNTRY_TH,
  RTW_COUNTRY_TN,
  RTW_COUNTRY_TR,
  RTW_COUNTRY_TT,
  RTW_COUNTRY_TZ,
  RTW_COUNTRY_UA,
  RTW_COUNTRY_UG,
  RTW_COUNTRY_UY,
  RTW_COUNTRY_UZ,
  RTW_COUNTRY_VC,
  RTW_COUNTRY_VE,
  RTW_COUNTRY_VN,
  RTW_COUNTRY_VU,
  RTW_COUNTRY_WF,
  RTW_COUNTRY_WS,
  RTW_COUNTRY_YE,
  RTW_COUNTRY_YT,
  RTW_COUNTRY_ZA,
  RTW_COUNTRY_ZW,
  RTW_COUNTRY_MAX
};

enum
{
  RTW_BSS_TYPE_INFRASTRUCTURE = 0, /* *< Denotes infrastructure network */
  RTW_BSS_TYPE_ADHOC          = 1, /* *< Denotes an 802.11 ad-hoc IBSS network */
  RTW_BSS_TYPE_ANY            = 2, /* *< Denotes either infrastructure or ad-hoc network */
  RTW_BSS_TYPE_UNKNOWN        = -1 /* *< May be returned by scan function if BSS type is unknown. Do not pass this to the Join function */
};

enum
{
  RTW_SECURITY_OPEN           = 0,                                              /* *< Open security */
  RTW_SECURITY_WEP_PSK        = WEP_ENABLED,                                    /* *< WEP Security with open authentication */
  RTW_SECURITY_WEP_SHARED     = (WEP_ENABLED | SHARED_ENABLED),                 /* *< WEP Security with shared authentication */
  RTW_SECURITY_WPA_TKIP_PSK   = (WPA_SECURITY  | TKIP_ENABLED),                 /* *< WPA Security with TKIP */
  RTW_SECURITY_WPA_AES_PSK    = (WPA_SECURITY  | AES_ENABLED),                  /* *< WPA Security with AES */
  RTW_SECURITY_WPA2_AES_PSK   = (WPA2_SECURITY | AES_ENABLED),                  /* *< WPA2 Security with AES */
  RTW_SECURITY_WPA2_TKIP_PSK  = (WPA2_SECURITY | TKIP_ENABLED),                 /* *< WPA2 Security with TKIP */
  RTW_SECURITY_WPA2_MIXED_PSK = (WPA2_SECURITY | AES_ENABLED | TKIP_ENABLED),   /* *< WPA2 Security with AES & TKIP */
  RTW_SECURITY_WPA_WPA2_MIXED = (WPA_SECURITY  | WPA2_SECURITY),                /* *< WPA/WPA2 Security */
  RTW_SECURITY_WPA2_AES_CMAC  = (WPA2_SECURITY | AES_CMAC_ENABLED),             /* *< WPA2 Security with AES and Management Frame Protection */
  RTW_SECURITY_WPS_OPEN       = WPS_ENABLED,                                    /* *< WPS with open security */
  RTW_SECURITY_WPS_SECURE     = (WPS_ENABLED | AES_ENABLED),                    /* *< WPS with AES security */
  RTW_SECURITY_UNKNOWN        = -1,                                             /* *< May be returned by scan function if security is unknown. Do not pass this to the join function! */
  RTW_SECURITY_FORCE_32_BIT   = 0x7fffffff                                      /* *< Exists only to force rtw_security_t type to 32 bits */
};

enum
{
  RTW_WPS_TYPE_DEFAULT              = 0x0000,
  RTW_WPS_TYPE_USER_SPECIFIED       = 0x0001,
  RTW_WPS_TYPE_MACHINE_SPECIFIED    = 0x0002,
  RTW_WPS_TYPE_REKEY                = 0x0003,
  RTW_WPS_TYPE_PUSHBUTTON           = 0x0004,
  RTW_WPS_TYPE_REGISTRAR_SPECIFIED  = 0x0005,
  RTW_WPS_TYPE_NONE                 = 0x0006,
  RTW_WPS_TYPE_WSC                  = 0x0007
};

enum
{
  RTW_802_11_BAND_5GHZ              = 0, /* *< Denotes 5GHz radio band */
  RTW_802_11_BAND_2_4GHZ            = 1  /* *< Denotes 2.4GHz radio band */
};

begin_packed_struct struct rtw_ssid
{
  unsigned char                 len;     /* *< SSID length */
  unsigned char                 val[33]; /* *< SSID name (AP name) */
} end_packed_struct;

begin_packed_struct struct rtw_mac
{
  unsigned char                 octet[6]; /* *< Unique 6-byte MAC address */
} end_packed_struct;

typedef unsigned long             rtw_bss_type_t;
typedef unsigned long             rtw_security_t;
typedef unsigned long             rtw_wps_type_t;
typedef unsigned long             rtw_802_11_band_t;
typedef struct rtw_ssid           rtw_ssid_t;
typedef struct rtw_mac            rtw_mac_t;
typedef struct rtw_scan_result    rtw_scan_result_t;
typedef struct rtw_network_info   rtw_network_info_t;
typedef struct rtw_scan_ap_result rtw_scan_ap_result_t;

begin_packed_struct struct rtw_network_info
{
  rtw_ssid_t              ssid;
  rtw_mac_t               bssid;
  rtw_security_t          security_type;
  unsigned char          *password;
  int password_len;
  int key_id;
} end_packed_struct;

begin_packed_struct struct rtw_scan_result
{
  rtw_ssid_t        SSID;             /* *< Service Set Identification (i.e. Name of Access Point) */
  rtw_mac_t         BSSID;            /* *< Basic Service Set Identification (i.e. MAC address of Access Point) */
  signed short      signal_strength;  /* *< Receive Signal Strength Indication in dBm. <-90=Very poor, >-30=Excellent */
  rtw_bss_type_t    bss_type;         /* *< Network type */
  rtw_security_t    security;         /* *< Security type */
  rtw_wps_type_t    wps_type;         /* *< WPS type */
  unsigned int      channel;          /* *< Radio channel that the AP beacon was received on */
  rtw_802_11_band_t band;             /* *< Radio band */
} end_packed_struct;

begin_packed_struct struct rtw_scan_ap_result
{
  char                    len;
  uint8_t                 BSSID[IFHWADDRLEN];
  int                     signal_strength;
  char                    security;
  char                    wps_type;
  char                    channel;
  char                    SSID[0];
} end_packed_struct;

enum
{
  RTW_MODE_NONE = 0,
  RTW_MODE_STA,
  RTW_MODE_AP,
  RTW_MODE_STA_AP,
  RTW_MODE_PROMISC,
  RTW_MODE_P2P
};

struct sk_buff_head
{
  struct list_head    *next;
  struct list_head    *prev;
  unsigned int        qlen;
};

struct sk_buff
{
  struct sk_buff      *next;
  struct sk_buff      *prev;
  struct sk_buff_head *list;
  unsigned char       *head;
  unsigned char       *data;
  unsigned char       *tail;
  unsigned char       *end;
  void                *dev;
  unsigned int        len;
  int                 dyalloc_flag;
};

unsigned char  *skb_put(struct sk_buff *skb,
                          unsigned int len);
unsigned char  *skb_pull(struct sk_buff *skb,
                           unsigned int len);
int             rltk_wlan_init(int index, unsigned long mode);
void            rltk_wlan_deinit(void);
void            rltk_wlan_deinit_fastly(void);
int             rltk_wlan_start(int index);
void            rltk_wlan_statistic(unsigned char index);
unsigned char   rltk_wlan_running(unsigned char index);
int             rltk_wlan_control(unsigned long cmd, void *data);
int             rltk_wlan_handshake_done(void);
int             rltk_wlan_rf_on(void);
int             rltk_wlan_rf_off(void);
int             rltk_wlan_check_bus(void);
int             rltk_wlan_wireless_mode(unsigned char mode);
int             rltk_wlan_get_wireless_mode(unsigned char *pmode);
int             rltk_wlan_set_wps_phase(unsigned char is_trigger_wps);
int             rtw_ps_enable(int enable);
int             rltk_wlan_is_connected_to_ap(void);
int             rltk_set_mode_prehandle(unsigned char curr_mode,
                                               unsigned char next_mode,
                                               const char *ifname);
int             rltk_set_mode_posthandle(unsigned char curr_mode,
                                                unsigned char next_mode,
                                                const char *ifname);
int             rltk_remove_softap_in_concurrent_mode(const char *ifname);
unsigned char   rltk_wlan_check_isup(int index);
void            rltk_wlan_tx_inc(int index);
void            rltk_wlan_tx_dec(int index);
struct sk_buff *rltk_wlan_get_recv_skb(int index);
struct sk_buff *rltk_wlan_alloc_skb(unsigned int len);
void            rltk_wlan_set_netif_info(int index_wlan, void *dev,
                                                unsigned char *addr);
void            rltk_wlan_send_skb(int index, struct sk_buff *skb);
unsigned char  *rltk_wlan_get_ip(int index);
int             netif_is_valid_ip(int index, unsigned char *ip_dest);
unsigned char  *netif_get_hwaddr(int index);
void            netif_rx(int index, unsigned int len);
void            netif_post_sleep_processing(void);
void            netif_pre_sleep_processing(void);

#endif /* __ARCH_ARM_SRC_RTL8720C_AMEBAZ_WLAN_H */
