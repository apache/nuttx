/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_ioctl.h
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Broadcom nor the names of other contributors to
 * this software may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * 4. This software may not be used as a standalone product, and may only
 * be used as incorporated in your product or device that incorporates
 * Broadcom wireless connectivity products and solely for the purpose of
 * enabling the functionalities of such Broadcom products.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY WARRANTIES OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT, ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/* Custom OID/ioctl definitions for
 * Broadcom 802.11abg Networking Device Driver
 */

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_IOCTL_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <net/ethernet.h>

typedef uint16_t wl_chanspec_t;
typedef uint16_t chanspec_t;

#define ACTION_FRAME_SIZE 1040

typedef struct wl_action_frame
{
  struct ether_addr da;
  uint16_t          len;
  uint32_t          packet_id;
  uint8_t           data[ACTION_FRAME_SIZE];
} wl_action_frame_t;

typedef struct ssid_info
{
  uint8_t ssid_len;
  uint8_t ssid[32];
} ssid_info_t;

typedef struct cnt_rx
{
  uint32_t cnt_rxundec;
  uint32_t cnt_rxframe;
} cnt_rx_t;

#define MCSSET_LEN 16

typedef struct wl_bss_info
{
  uint32_t      version;                /* version field */
  uint32_t      length;                 /* byte length of data in this record,   */
                                        /* starting at version and including IEs */
  struct ether_addr BSSID;
  uint16_t      beacon_period;          /* units are Kusec */
  uint16_t      capability;             /* Capability information */
  uint8_t       ssid_len;
  uint8_t       SSID[32];
  struct
  {
    uint32_t  count;                    /* # rates in this set */
    uint8_t   rates[16];                /* rates in 500kbps units w/hi bit set if basic */
  } rateset;                            /* supported rates */
  wl_chanspec_t chanspec;               /* chanspec for bss */
  uint16_t      atim_window;            /* units are Kusec */
  uint8_t       dtim_period;            /* DTIM period */
  int16_t       RSSI;                   /* receive signal strength (in dBm) */
  int8_t        phy_noise;              /* noise (in dBm) */

  uint8_t       n_cap;                  /* BSS is 802.11N Capable */
  uint32_t      nbss_cap;               /* 802.11N BSS Capabilities (based on HT_CAP_*) */
  uint8_t       ctl_ch;                 /* 802.11N BSS control channel number */
  uint32_t      reserved32[1];          /* Reserved for expansion of BSS properties */
  uint8_t       flags;                  /* flags */
  uint8_t       reserved[3];            /* Reserved for expansion of BSS properties */
  uint8_t       basic_mcs[MCSSET_LEN];  /* 802.11N BSS required MCS set */

  uint16_t      ie_offset;              /* offset at which IEs start, from beginning */
  uint32_t      ie_length;              /* byte length of Information Elements */
  int16_t       SNR;                    /* average SNR of during frame reception */

  /* Add new fields here */

  /* variable length Information Elements */
} wl_bss_info_t;

#define DOT11_CAP_ESS     0x0001
#define DOT11_CAP_IBSS    0x0002
#define DOT11_CAP_PRIVACY 0x0010

typedef struct wlc_ssid
{
  uint32_t ssid_len;
  uint8_t  SSID[32];
} wlc_ssid_t;

#define WL_BSSTYPE_INFRA 1
#define WL_BSSTYPE_INDEP 0
#define WL_BSSTYPE_ANY   2
#define WL_SCANFLAGS_PASSIVE 0x01
#define WL_SCANFLAGS_PROHIBITED    0x04

typedef struct wl_scan_params
{
  wlc_ssid_t         ssid;
  struct ether_addr  bssid;
  int8_t             bss_type;
  int8_t             scan_type;
  int32_t            nprobes;
  int32_t            active_time;
  int32_t            passive_time;
  int32_t            home_time;
  int32_t            channel_num;
  uint16_t           channel_list[1];
} wl_scan_params_t;

#define WL_SCAN_PARAMS_FIXED_SIZE          (64)
#define WL_SCAN_PARAMS_COUNT_MASK  (0x0000ffff)
#define WL_SCAN_PARAMS_NSSID_SHIFT         (16)
#define WL_SCAN_ACTION_START                (1)
#define WL_SCAN_ACTION_CONTINUE             (2)
#define WL_SCAN_ACTION_ABORT                (3)
#define ISCAN_REQ_VERSION                   (1)

typedef struct wl_iscan_params
{
  uint32_t          version;
  uint16_t          action;
  uint16_t          scan_duration;
  wl_scan_params_t  params;
} wl_iscan_params_t;

#define WL_ISCAN_PARAMS_FIXED_SIZE (offsetof(wl_iscan_params_t, params) + sizeof(wlc_ssid_t))

typedef struct wl_scan_results
{
  uint32_t       buflen;
  uint32_t       version;
  uint32_t       count;
  wl_bss_info_t  bss_info[1];
} wl_scan_results_t;

#define WL_SCAN_RESULTS_FIXED_SIZE  (12)
#define WL_SCAN_RESULTS_SUCCESS         (0)
#define WL_SCAN_RESULTS_PARTIAL         (1)
#define WL_SCAN_RESULTS_PENDING         (2)
#define WL_SCAN_RESULTS_ABORTED         (3)
#define WL_SCAN_RESULTS_NO_MEM         (4)
#define ESCAN_REQ_VERSION 1

typedef struct wl_escan_params
{
  uint32_t          version;
  uint16_t          action;
  uint16_t          sync_id;
  wl_scan_params_t  params;
} wl_escan_params_t;

#define WL_ESCAN_PARAMS_FIXED_SIZE (offsetof(wl_escan_params_t, params) + sizeof(wlc_ssid_t))

typedef struct wl_escan_result
{
  uint32_t       buflen;
  uint32_t       version;
  uint16_t       sync_id;
  uint16_t       bss_count;
  wl_bss_info_t  bss_info[1];
} wl_escan_result_t;

#define WL_ESCAN_RESULTS_FIXED_SIZE (sizeof(wl_escan_result_t) - sizeof(wl_bss_info_t))

typedef struct wl_iscan_results
{
  uint32_t           status;
  wl_scan_results_t  results;
} wl_iscan_results_t;

#define WL_ISCAN_RESULTS_FIXED_SIZE \
  (WL_SCAN_RESULTS_FIXED_SIZE + offsetof(wl_iscan_results_t, results))
#define WL_MAXRATES_IN_SET      16  /* max # of rates in a rateset */

typedef struct wl_rateset
{
  uint32_t  count;                      /* # rates in this set */
  uint8_t   rates[WL_MAXRATES_IN_SET];  /* rates in 500kbps units w/hi bit set if basic */
} wl_rateset_t;

typedef struct wl_uint32_list
{
  uint32_t count;
  uint32_t element[1];
} wl_uint32_list_t;

/* scan params for extended join */

typedef struct wl_join_scan_params
{
  uint8_t scan_type;    /* 0 use default, active or passive scan */
  uint8_t pad[3];
  int32_t nprobes;      /* -1 use default, number of probes per channel */
  int32_t active_time;  /* -1 use default, dwell time per channel for
                         * active scanning */
  int32_t passive_time; /* -1 use default, dwell time per channel
                         * for passive scanning */
  int32_t home_time;    /* -1 use default, dwell time for the home channel
                         * between channel scans */
} wl_join_scan_params_t;

/** used for association with a specific BSSID and chanspec list */

typedef struct wl_assoc_params
{
  struct ether_addr bssid;            /* 00:00:00:00:00:00: broadcast scan */
  uint16_t          bssid_cnt;        /* 0: use chanspec_num, and the single bssid,
                                       * otherwise count of chanspecs in chanspec_list
                                       * AND paired bssids following chanspec_list
                                       * also, chanspec_num has to be set to zero
                                       * for bssid list to be used
                                       */
  int32_t           chanspec_num;     /* 0: all available channels,
                                       * otherwise count of chanspecs in chanspec_list */
  chanspec_t        chanspec_list[1]; /* list of chanspecs */
} wl_assoc_params_t;

/** used for association to a specific BSSID and channel */

typedef wl_assoc_params_t wl_join_assoc_params_t;

/** extended join params */

typedef struct wl_extjoin_params
{
  wlc_ssid_t             ssid;  /* {0, ""}: wildcard scan */
  wl_join_scan_params_t  scan;
  wl_join_assoc_params_t assoc; /* optional field, but it must include the fixed portion
                                 * of the wl_join_assoc_params_t struct when it does
                                 * present. */
} wl_extjoin_params_t;

#define WL_EXTJOIN_PARAMS_FIXED_SIZE \
  (offset(wl_extjoin_params_t, assoc) + WL_JOIN_ASSOC_PARAMS_FIXED_SIZE)

#define NRATE_MCS_INUSE            (0x00000080)
#define NRATE_RATE_MASK         (0x0000007f)
#define NRATE_STF_MASK            (0x0000ff00)
#define NRATE_STF_SHIFT                     (8)
#define NRATE_OVERRIDE            (0x80000000)
#define NRATE_OVERRIDE_MCS_ONLY   (0x40000000)
#define NRATE_SGI_MASK          (0x00800000)
#define NRATE_SGI_SHIFT                 (23)
#define NRATE_LDPC_CODING       (0x00400000)
#define NRATE_LDPC_SHIFT                (22)
#define NRATE_BCMC_OVERRIDE     (0x00200000)
#define NRATE_BCMC_SHIFT                (21)
#define NRATE_STF_SISO                     (0)
#define NRATE_STF_CDD                     (1)
#define NRATE_STF_STBC                     (2)
#define NRATE_STF_SDM                     (3)
#define ANTENNA_NUM_1                     (1)
#define ANTENNA_NUM_2                     (2)
#define ANTENNA_NUM_3                     (3)
#define ANTENNA_NUM_4                     (4)
#define ANT_SELCFG_AUTO                  (0x80)
#define ANT_SELCFG_MASK                  (0x33)
#define ANT_SELCFG_MAX                     (4)
#define ANT_SELCFG_TX_UNICAST             (0)
#define ANT_SELCFG_RX_UNICAST             (1)
#define ANT_SELCFG_TX_DEF                 (2)
#define ANT_SELCFG_RX_DEF                 (3)

typedef struct
{
  uint8_t ant_config[ANT_SELCFG_MAX];
  uint8_t num_antcfg;
} wlc_antselcfg_t;

#define HIGHEST_SINGLE_STREAM_MCS    (7)
#define WLC_CNTRY_BUF_SZ    (4)

typedef struct wl_country
{
  int8_t    country_abbrev[WLC_CNTRY_BUF_SZ];
  int32_t rev;
  int8_t ccode[WLC_CNTRY_BUF_SZ];
} wl_country_t;

typedef struct wl_channels_in_country
{
  uint32_t buflen;
  uint32_t band;
  int8_t country_abbrev[WLC_CNTRY_BUF_SZ];
  uint32_t count;
  uint32_t channel[1];
} wl_channels_in_country_t;

typedef struct wl_country_list
{
  uint32_t buflen;
  uint32_t band_set;
  uint32_t band;
  uint32_t count;
  int8_t country_abbrev[1];
} wl_country_list_t;

#define WL_NUM_RPI_BINS      8
#define WL_RM_TYPE_BASIC     1
#define WL_RM_TYPE_CCA       2
#define WL_RM_TYPE_RPI       3
#define WL_RM_FLAG_PARALLEL  (1<<0)
#define WL_RM_FLAG_LATE      (1<<1)
#define WL_RM_FLAG_INCAPABLE (1<<2)
#define WL_RM_FLAG_REFUSED   (1<<3)

typedef struct wl_rm_req_elt
{
  int8_t type;
  int8_t flags;
  wl_chanspec_t chanspec;
  uint32_t token;
  uint32_t tsf_h;
  uint32_t tsf_l;
  uint32_t dur;
} wl_rm_req_elt_t;

typedef struct wl_rm_req
{
  uint32_t token;
  uint32_t count;
  void *cb;
  void *cb_arg;
  wl_rm_req_elt_t req[1];
} wl_rm_req_t;

#define WL_RM_REQ_FIXED_LEN    offsetof(wl_rm_req_t, req)

typedef struct wl_rm_rep_elt
{
  int8_t type;
  int8_t flags;
  wl_chanspec_t chanspec;
  uint32_t token;
  uint32_t tsf_h;
  uint32_t tsf_l;
  uint32_t dur;
  uint32_t len;
  uint8_t data[1];
} wl_rm_rep_elt_t;

#define WL_RM_REP_ELT_FIXED_LEN    24
#define WL_RPI_REP_BIN_NUM 8

typedef struct wl_rm_rpi_rep
{
  uint8_t rpi[WL_RPI_REP_BIN_NUM];
  int8_t rpi_max[WL_RPI_REP_BIN_NUM];
} wl_rm_rpi_rep_t;
typedef struct wl_rm_rep
{
  uint32_t token;
  uint32_t len;
  wl_rm_rep_elt_t rep[1];
} wl_rm_rep_t;

#define WL_RM_REP_FIXED_LEN    8

#define CRYPTO_ALGO_OFF          0
#define CRYPTO_ALGO_WEP1         1
#define CRYPTO_ALGO_TKIP         2
#define CRYPTO_ALGO_WEP128       3
#define CRYPTO_ALGO_AES_CCM      4
#define CRYPTO_ALGO_AES_OCB_MSDU 5
#define CRYPTO_ALGO_AES_OCB_MPDU 6
#define CRYPTO_ALGO_NALG         7

#define WSEC_GEN_MIC_ERROR 0x0001
#define WSEC_GEN_REPLAY    0x0002
#define WSEC_GEN_ICV_ERROR 0x0004

#define WL_SOFT_KEY            (1 << 0)
#define WL_PRIMARY_KEY         (1 << 1)
#define WL_KF_RES_4            (1 << 4)
#define WL_KF_RES_5            (1 << 5)
#define WL_IBSS_PEER_GROUP_KEY (1 << 6)
#define DOT11_MAX_KEY_SIZE     32

typedef struct wl_wsec_key
{
  uint32_t index;
  uint32_t len;
  uint8_t data[DOT11_MAX_KEY_SIZE];
  uint32_t pad_1[18];
  uint32_t algo;
  uint32_t flags;
  uint32_t pad_2[2];
  int32_t pad_3;
  int32_t iv_initialized;
  int32_t pad_4;
  struct
  {
    uint32_t hi;
    uint16_t lo;
  } rxiv;
  uint32_t pad_5[2];
  struct ether_addr ea;
} wl_wsec_key_t;

#define WSEC_MIN_PSK_LEN    8
#define WSEC_MAX_PSK_LEN    64
#define WSEC_PASSPHRASE        (1<<0)

typedef struct
{
  uint16_t key_len;
  uint16_t flags;
  uint8_t key[WSEC_MAX_PSK_LEN];
} wsec_pmk_t;

#define OPEN_AUTH                   0x0000
#define SHARED_AUTH                 0x0001
#define WEP_ENABLED                 0x0001
#define TKIP_ENABLED                0x0002
#define AES_ENABLED                 0x0004
#define WSEC_SWFLAG                 0x0008
#define SES_OW_ENABLED              0x0040
#define WPA_AUTH_DISABLED           0x0000
#define WPA_AUTH_NONE               0x0001
#define WPA_AUTH_UNSPECIFIED        0x0002
#define WPA_AUTH_PSK                0x0004
#define WPA2_AUTH_UNSPECIFIED       0x0040
#define WPA2_AUTH_PSK               0x0080
#define BRCM_AUTH_PSK               0x0100
#define BRCM_AUTH_DPT               0x0200
#define WPA_AUTH_PFN_ANY            0xffffffff
#define MAXPMKID                    16
#define WPA2_PMKID_LEN              16

typedef struct _pmkid
{
  struct ether_addr BSSID;
  uint8_t PMKID[WPA2_PMKID_LEN];
} pmkid_t;

typedef struct _pmkid_list
{
  uint32_t npmkid;
  pmkid_t pmkid[1];
} pmkid_list_t;

typedef struct _pmkid_cand
{
  struct ether_addr BSSID;
  uint8_t preauth;
} pmkid_cand_t;

typedef struct _pmkid_cand_list
{
  uint32_t npmkid_cand;
  pmkid_cand_t pmkid_cand[1];
} pmkid_cand_list_t;

typedef struct wl_led_info
{
  uint32_t index;
  uint32_t behavior;
  uint8_t activehi;
} wl_led_info_t;

struct wl_dot11_assoc_req
{
  uint16_t capability;
  uint16_t listen;
};

struct wl_dot11_assoc_resp
{
  uint16_t capability;
  uint16_t status;
  uint16_t aid;
};

typedef struct wl_assoc_info
{
  uint32_t req_len;
  uint32_t resp_len;
  uint32_t flags;
  struct wl_dot11_assoc_req req;
  struct ether_addr reassoc_bssid;
  struct wl_dot11_assoc_resp resp;
} wl_assoc_info_t;

#define WLC_ASSOC_REQ_IS_REASSOC 0x01

typedef struct
{
  uint32_t byteoff;
  uint32_t nbytes;
  uint16_t buf[1];
} srom_rw_t;

typedef struct
{
  uint32_t source;
  uint32_t byteoff;
  uint32_t nbytes;
} cis_rw_t;

#define WLC_CIS_DEFAULT 0
#define WLC_CIS_SROM    1
#define WLC_CIS_OTP     2

typedef struct
{
  uint32_t byteoff;
  uint32_t val;
  uint32_t size;
  uint32_t band;
} rw_reg_t;

#define WL_ATTEN_APP_INPUT_PCL_OFF 0
#define WL_ATTEN_PCL_ON            1
#define WL_ATTEN_PCL_OFF           2

typedef struct
{
  uint16_t auto_ctrl;
  uint16_t bb;
  uint16_t radio;
  uint16_t txctl1;
} atten_t;

struct wme_tx_params_s
{
  uint8_t short_retry;
  uint8_t short_fallback;
  uint8_t long_retry;
  uint8_t long_fallback;
  uint16_t max_rate;
};
typedef struct wme_tx_params_s wme_tx_params_t;

#define WL_WME_TX_PARAMS_IO_BYTES (sizeof(wme_tx_params_t) * AC_COUNT)
#define WL_PWRIDX_PCL_OFF    -2
#define WL_PWRIDX_PCL_ON    -1
#define WL_PWRIDX_LOWER_LIMIT    -2
#define WL_PWRIDX_UPPER_LIMIT    63

typedef struct
{
  uint32_t val;
  struct ether_addr ea;
  uint16_t pad;
} scb_val_t;

#define BCM_MAC_STATUS_INDICATION    (0x40010200L)

typedef struct
{
  uint16_t ver;
  uint16_t len;
  uint16_t cap;
  uint32_t flags;
  uint32_t idle;
  struct ether_addr ea;
  wl_rateset_t rateset;
  uint32_t in;
  uint32_t listen_interval_inms;
  uint32_t tx_pkts;
  uint32_t tx_failures;
  uint32_t rx_ucast_pkts;
  uint32_t rx_mcast_pkts;
  uint32_t tx_rate;
  uint32_t rx_rate;
} sta_info_t;

#define WL_OLD_STAINFO_SIZE    offsetof(sta_info_t, tx_pkts)
#define WL_STA_VER             2
#define WL_STA_BRCM            0x1
#define WL_STA_WME             0x2
#define WL_STA_ABCAP           0x4
#define WL_STA_AUTHE        0x8
#define WL_STA_ASSOC        0x10
#define WL_STA_AUTHO        0x20
#define WL_STA_WDS        0x40
#define WL_STA_WDS_LINKUP    0x80
#define WL_STA_PS        0x100
#define WL_STA_APSD_BE        0x200
#define WL_STA_APSD_BK        0x400
#define WL_STA_APSD_VI        0x800
#define WL_STA_APSD_VO        0x1000
#define WL_STA_N_CAP        0x2000
#define WL_STA_SCBSTATS        0x4000
#define WL_WDS_LINKUP        WL_STA_WDS_LINKUP

typedef struct channel_info
{
  int32_t hw_channel;
  int32_t target_channel;
  int32_t scan_channel;
} channel_info_t;
struct mac_list
{
  uint32_t count;
  struct ether_addr ea[1];
};
typedef struct get_pktcnt
{
  uint32_t rx_good_pkt;
  uint32_t rx_bad_pkt;
  uint32_t tx_good_pkt;
  uint32_t tx_bad_pkt;
  uint32_t rx_ocast_good_pkt;
} get_pktcnt_t;
typedef struct wl_ioctl
{
  uint32_t cmd;
  void *buf;
  uint32_t len;
  uint8_t set;
  uint32_t used;
  uint32_t needed;
} wl_ioctl_t;

typedef struct wlc_rev_info
{
  uint32_t vendorid;
  uint32_t deviceid;
  uint32_t radiorev;
  uint32_t chiprev;
  uint32_t corerev;
  uint32_t boardid;
  uint32_t boardvendor;
  uint32_t boardrev;
  uint32_t driverrev;
  uint32_t ucoderev;
  uint32_t bus;
  uint32_t chipnum;
  uint32_t phytype;
  uint32_t phyrev;
  uint32_t anarev;
} wlc_rev_info_t;

#define WL_REV_INFO_LEGACY_LENGTH    48
#define WL_BRAND_MAX 10

typedef struct wl_instance_info
{
  uint32_t instance;
  int8_t brand[WL_BRAND_MAX];
} wl_instance_info_t;

typedef struct wl_txfifo_sz
{
  uint8_t fifo;
  uint8_t size;
} wl_txfifo_sz_t;

#define WLC_IOV_NAME_LEN 30

typedef struct wlc_iov_trx_s
{
  uint8_t module;
  uint8_t type;
  int8_t name[WLC_IOV_NAME_LEN];
} wlc_iov_trx_t;

#define IOVAR_STR_BSSCFG_WPA_AUTH        "bsscfg:wpa_auth"
#define IOVAR_STR_BSSCFG_WSEC            "bsscfg:wsec"
#define IOVAR_STR_BSSCFG_SSID            "bsscfg:ssid"
#define IOVAR_STR_BSSCFG_EVENT_MSGS      "bsscfg:event_msgs"
#define IOVAR_STR_BSSCFG_CUR_ETHERADDR   "bsscfg:cur_etheraddr"
#define IOVAR_STR_BSSCFG_ACTFRAME        "bsscfg:actframe"
#define IOVAR_STR_BSSCFG_ACTION_FRAME    "bsscfg:wifiaction"
#define IOVAR_STR_BSSCFG_VENDOR_IE       "bsscfg:vndr_ie"
#define IOVAR_STR_BSS                    "bss"
#define IOVAR_STR_BSS_RATESET            "bss_rateset"
#define IOVAR_STR_CSA                    "csa"
#define IOVAR_STR_AMPDU_TID              "ampdu_tid"
#define IOVAR_STR_APSTA                  "apsta"
#define IOVAR_STR_ALLMULTI               "allmulti"
#define IOVAR_STR_COUNTRY                "country"
#define IOVAR_STR_EVENT_MSGS             "event_msgs"
#define IOVAR_STR_ESCAN                  "escan"
#define IOVAR_STR_SUP_WPA                "sup_wpa"
#define IOVAR_STR_CUR_ETHERADDR          "cur_etheraddr"
#define IOVAR_STR_QTXPOWER               "qtxpower"
#define IOVAR_STR_MCAST_LIST             "mcast_list"
#define IOVAR_STR_PM2_SLEEP_RET          "pm2_sleep_ret"
#define IOVAR_STR_PM_LIMIT               "pm_limit"
#define IOVAR_STR_LISTEN_INTERVAL_BEACON "bcn_li_bcn"
#define IOVAR_STR_LISTEN_INTERVAL_DTIM   "bcn_li_dtim"
#define IOVAR_STR_LISTEN_INTERVAL_ASSOC  "assoc_listen"
#define IOVAR_PSPOLL_PERIOD              "pspoll_prd"
#define IOVAR_STR_VENDOR_IE              "vndr_ie"
#define IOVAR_STR_TX_GLOM                "bus:txglom"
#define IOVAR_STR_ACTION_FRAME           "wifiaction"
#define IOVAR_STR_AC_PARAMS_STA          "wme_ac_sta"
#define IOVAR_STR_COUNTERS               "counters"
#define IOVAR_STR_PKT_FILTER_ADD         "pkt_filter_add"
#define IOVAR_STR_PKT_FILTER_DELETE      "pkt_filter_delete"
#define IOVAR_STR_PKT_FILTER_ENABLE      "pkt_filter_enable"
#define IOVAR_STR_PKT_FILTER_MODE        "pkt_filter_mode"
#define IOVAR_STR_PKT_FILTER_LIST        "pkt_filter_list"
#define IOVAR_STR_PKT_FILTER_STATS       "pkt_filter_stats"
#define IOVAR_STR_PKT_FILTER_CLEAR_STATS "pkt_filter_clear_stats"
#define IOVAR_STR_DUTY_CYCLE_CCK         "dutycycle_cck"
#define IOVAR_STR_DUTY_CYCLE_OFDM        "dutycycle_ofdm"
#define IOVAR_STR_MKEEP_ALIVE            "mkeep_alive"
#define IOVAR_STR_VERSION                "ver"
#define IOVAR_STR_SUP_WPA2_EAPVER        "sup_wpa2_eapver"
#define IOVAR_STR_ROAM_OFF               "roam_off"
#define IOVAR_STR_CLOSEDNET              "closednet"
#define IOVAR_STR_P2P_DISC               "p2p_disc"
#define IOVAR_STR_P2P_DEV                "p2p_dev"
#define IOVAR_STR_P2P_IFADD              "p2p_ifadd"
#define IOVAR_STR_P2P_IFDEL              "p2p_ifdel"
#define IOVAR_STR_P2P_IFUPD              "p2p_ifupd"
#define IOVAR_STR_P2P_SCAN               "p2p_scan"
#define IOVAR_STR_P2P_STATE              "p2p_state"
#define IOVAR_STR_P2P_SSID               "p2p_ssid"
#define IOVAR_STR_NRATE                  "nrate"
#define IOVAR_STR_BGRATE                 "bg_rate"
#define IOVAR_STR_ARATE                  "a_rate"
#define IOVAR_STR_NMODE                  "nmode"
#define IOVAR_STR_MAX_ASSOC              "maxassoc"
#define IOVAR_STR_2G_MULTICAST_RATE      "2g_mrate"
#define IOVAR_STR_MPC                    "mpc"
#define IOVAR_STR_AMPDU_BA_WINDOW_SIZE   "ampdu_ba_wsize"
#define IOVAR_STR_AMPDU_MPDU             "ampdu_mpdu"
#define IOVAR_STR_AMPDU_RX_FACTOR        "ampdu_rx_factor"
#define IOVAR_STR_MIMO_BW_CAP            "mimo_bw_cap"
#define IOVAR_STR_CLMLOAD                "clmload"
#define IOVAR_STR_JOIN                   "join"

#define WLC_IOCTL_MAGIC                    ( 0x14e46c77 )
#define WLC_IOCTL_VERSION                  (          1 )
#define WLC_IOCTL_SMLEN                    (        256 )
#define WLC_IOCTL_MEDLEN                   (       1536 )
#define WLC_IOCTL_MAXLEN                   (       8192 )

#define WLC_GET_MAGIC                      ( (uint32_t)   0 )
#define WLC_GET_VERSION                    ( (uint32_t)   1 )
#define WLC_UP                             ( (uint32_t)   2 )
#define WLC_DOWN                           ( (uint32_t)   3 )
#define WLC_GET_LOOP                       ( (uint32_t)   4 )
#define WLC_SET_LOOP                       ( (uint32_t)   5 )
#define WLC_DUMP                           ( (uint32_t)   6 )
#define WLC_GET_MSGLEVEL                   ( (uint32_t)   7 )
#define WLC_SET_MSGLEVEL                   ( (uint32_t)   8 )
#define WLC_GET_PROMISC                    ( (uint32_t)   9 )
#define WLC_SET_PROMISC                    ( (uint32_t)  10 )
#define WLC_GET_RATE                       ( (uint32_t)  12 )
#define WLC_GET_INSTANCE                   ( (uint32_t)  14 )
#define WLC_GET_INFRA                      ( (uint32_t)  19 )
#define WLC_SET_INFRA                      ( (uint32_t)  20 )
#define WLC_GET_AUTH                       ( (uint32_t)  21 )
#define WLC_SET_AUTH                       ( (uint32_t)  22 )
#define WLC_GET_BSSID                      ( (uint32_t)  23 )
#define WLC_SET_BSSID                      ( (uint32_t)  24 )
#define WLC_GET_SSID                       ( (uint32_t)  25 )
#define WLC_SET_SSID                       ( (uint32_t)  26 )
#define WLC_RESTART                        ( (uint32_t)  27 )
#define WLC_GET_CHANNEL                    ( (uint32_t)  29 )
#define WLC_SET_CHANNEL                    ( (uint32_t)  30 )
#define WLC_GET_SRL                        ( (uint32_t)  31 )
#define WLC_SET_SRL                        ( (uint32_t)  32 )
#define WLC_GET_LRL                        ( (uint32_t)  33 )
#define WLC_SET_LRL                        ( (uint32_t)  34 )
#define WLC_GET_PLCPHDR                    ( (uint32_t)  35 )
#define WLC_SET_PLCPHDR                    ( (uint32_t)  36 )
#define WLC_GET_RADIO                      ( (uint32_t)  37 )
#define WLC_SET_RADIO                      ( (uint32_t)  38 )
#define WLC_GET_PHYTYPE                    ( (uint32_t)  39 )
#define WLC_DUMP_RATE                      ( (uint32_t)  40 )
#define WLC_SET_RATE_PARAMS                ( (uint32_t)  41 )
#define WLC_GET_KEY                        ( (uint32_t)  44 )
#define WLC_SET_KEY                        ( (uint32_t)  45 )
#define WLC_GET_REGULATORY                 ( (uint32_t)  46 )
#define WLC_SET_REGULATORY                 ( (uint32_t)  47 )
#define WLC_GET_PASSIVE_SCAN               ( (uint32_t)  48 )
#define WLC_SET_PASSIVE_SCAN               ( (uint32_t)  49 )
#define WLC_SCAN                           ( (uint32_t)  50 )
#define WLC_SCAN_RESULTS                   ( (uint32_t)  51 )
#define WLC_DISASSOC                       ( (uint32_t)  52 )
#define WLC_REASSOC                        ( (uint32_t)  53 )
#define WLC_GET_ROAM_TRIGGER               ( (uint32_t)  54 )
#define WLC_SET_ROAM_TRIGGER               ( (uint32_t)  55 )
#define WLC_GET_ROAM_DELTA                 ( (uint32_t)  56 )
#define WLC_SET_ROAM_DELTA                 ( (uint32_t)  57 )
#define WLC_GET_ROAM_SCAN_PERIOD           ( (uint32_t)  58 )
#define WLC_SET_ROAM_SCAN_PERIOD           ( (uint32_t)  59 )
#define WLC_EVM                            ( (uint32_t)  60 )
#define WLC_GET_TXANT                      ( (uint32_t)  61 )
#define WLC_SET_TXANT                      ( (uint32_t)  62 )
#define WLC_GET_ANTDIV                     ( (uint32_t)  63 )
#define WLC_SET_ANTDIV                     ( (uint32_t)  64 )
#define WLC_GET_CLOSED                     ( (uint32_t)  67 )
#define WLC_SET_CLOSED                     ( (uint32_t)  68 )
#define WLC_GET_MACLIST                    ( (uint32_t)  69 )
#define WLC_SET_MACLIST                    ( (uint32_t)  70 )
#define WLC_GET_RATESET                    ( (uint32_t)  71 )
#define WLC_SET_RATESET                    ( (uint32_t)  72 )
#define WLC_LONGTRAIN                      ( (uint32_t)  74 )
#define WLC_GET_BCNPRD                     ( (uint32_t)  75 )
#define WLC_SET_BCNPRD                     ( (uint32_t)  76 )
#define WLC_GET_DTIMPRD                    ( (uint32_t)  77 )
#define WLC_SET_DTIMPRD                    ( (uint32_t)  78 )
#define WLC_GET_SROM                       ( (uint32_t)  79 )
#define WLC_SET_SROM                       ( (uint32_t)  80 )
#define WLC_GET_WEP_RESTRICT               ( (uint32_t)  81 )
#define WLC_SET_WEP_RESTRICT               ( (uint32_t)  82 )
#define WLC_GET_COUNTRY                    ( (uint32_t)  83 )
#define WLC_SET_COUNTRY                    ( (uint32_t)  84 )
#define WLC_GET_PM                         ( (uint32_t)  85 )
#define WLC_SET_PM                         ( (uint32_t)  86 )
#define WLC_GET_WAKE                       ( (uint32_t)  87 )
#define WLC_SET_WAKE                       ( (uint32_t)  88 )
#define WLC_GET_FORCELINK                  ( (uint32_t)  90 )
#define WLC_SET_FORCELINK                  ( (uint32_t)  91 )
#define WLC_FREQ_ACCURACY                  ( (uint32_t)  92 )
#define WLC_CARRIER_SUPPRESS               ( (uint32_t)  93 )
#define WLC_GET_PHYREG                     ( (uint32_t)  94 )
#define WLC_SET_PHYREG                     ( (uint32_t)  95 )
#define WLC_GET_RADIOREG                   ( (uint32_t)  96 )
#define WLC_SET_RADIOREG                   ( (uint32_t)  97 )
#define WLC_GET_REVINFO                    ( (uint32_t)  98 )
#define WLC_GET_UCANTDIV                   ( (uint32_t)  99 )
#define WLC_SET_UCANTDIV                   ( (uint32_t) 100 )
#define WLC_R_REG                          ( (uint32_t) 101 )
#define WLC_W_REG                          ( (uint32_t) 102 )
#define WLC_GET_MACMODE                    ( (uint32_t) 105 )
#define WLC_SET_MACMODE                    ( (uint32_t) 106 )
#define WLC_GET_MONITOR                    ( (uint32_t) 107 )
#define WLC_SET_MONITOR                    ( (uint32_t) 108 )
#define WLC_GET_GMODE                      ( (uint32_t) 109 )
#define WLC_SET_GMODE                      ( (uint32_t) 110 )
#define WLC_GET_LEGACY_ERP                 ( (uint32_t) 111 )
#define WLC_SET_LEGACY_ERP                 ( (uint32_t) 112 )
#define WLC_GET_RX_ANT                     ( (uint32_t) 113 )
#define WLC_GET_CURR_RATESET               ( (uint32_t) 114 )
#define WLC_GET_SCANSUPPRESS               ( (uint32_t) 115 )
#define WLC_SET_SCANSUPPRESS               ( (uint32_t) 116 )
#define WLC_GET_AP                         ( (uint32_t) 117 )
#define WLC_SET_AP                         ( (uint32_t) 118 )
#define WLC_GET_EAP_RESTRICT               ( (uint32_t) 119 )
#define WLC_SET_EAP_RESTRICT               ( (uint32_t) 120 )
#define WLC_SCB_AUTHORIZE                  ( (uint32_t) 121 )
#define WLC_SCB_DEAUTHORIZE                ( (uint32_t) 122 )
#define WLC_GET_WDSLIST                    ( (uint32_t) 123 )
#define WLC_SET_WDSLIST                    ( (uint32_t) 124 )
#define WLC_GET_ATIM                       ( (uint32_t) 125 )
#define WLC_SET_ATIM                       ( (uint32_t) 126 )
#define WLC_GET_RSSI                       ( (uint32_t) 127 )
#define WLC_GET_PHYANTDIV                  ( (uint32_t) 128 )
#define WLC_SET_PHYANTDIV                  ( (uint32_t) 129 )
#define WLC_AP_RX_ONLY                     ( (uint32_t) 130 )
#define WLC_GET_TX_PATH_PWR                ( (uint32_t) 131 )
#define WLC_SET_TX_PATH_PWR                ( (uint32_t) 132 )
#define WLC_GET_WSEC                       ( (uint32_t) 133 )
#define WLC_SET_WSEC                       ( (uint32_t) 134 )
#define WLC_GET_PHY_NOISE                  ( (uint32_t) 135 )
#define WLC_GET_BSS_INFO                   ( (uint32_t) 136 )
#define WLC_GET_PKTCNTS                    ( (uint32_t) 137 )
#define WLC_GET_LAZYWDS                    ( (uint32_t) 138 )
#define WLC_SET_LAZYWDS                    ( (uint32_t) 139 )
#define WLC_GET_BANDLIST                   ( (uint32_t) 140 )
#define WLC_GET_BAND                       ( (uint32_t) 141 )
#define WLC_SET_BAND                       ( (uint32_t) 142 )
#define WLC_SCB_DEAUTHENTICATE             ( (uint32_t) 143 )
#define WLC_GET_SHORTSLOT                  ( (uint32_t) 144 )
#define WLC_GET_SHORTSLOT_OVERRIDE         ( (uint32_t) 145 )
#define WLC_SET_SHORTSLOT_OVERRIDE         ( (uint32_t) 146 )
#define WLC_GET_SHORTSLOT_RESTRICT         ( (uint32_t) 147 )
#define WLC_SET_SHORTSLOT_RESTRICT         ( (uint32_t) 148 )
#define WLC_GET_GMODE_PROTECTION           ( (uint32_t) 149 )
#define WLC_GET_GMODE_PROTECTION_OVERRIDE  ( (uint32_t) 150 )
#define WLC_SET_GMODE_PROTECTION_OVERRIDE  ( (uint32_t) 151 )
#define WLC_UPGRADE                        ( (uint32_t) 152 )
#define WLC_GET_IGNORE_BCNS                ( (uint32_t) 155 )
#define WLC_SET_IGNORE_BCNS                ( (uint32_t) 156 )
#define WLC_GET_SCB_TIMEOUT                ( (uint32_t) 157 )
#define WLC_SET_SCB_TIMEOUT                ( (uint32_t) 158 )
#define WLC_GET_ASSOCLIST                  ( (uint32_t) 159 )
#define WLC_GET_CLK                        ( (uint32_t) 160 )
#define WLC_SET_CLK                        ( (uint32_t) 161 )
#define WLC_GET_UP                         ( (uint32_t) 162 )
#define WLC_OUT                            ( (uint32_t) 163 )
#define WLC_GET_WPA_AUTH                   ( (uint32_t) 164 )
#define WLC_SET_WPA_AUTH                   ( (uint32_t) 165 )
#define WLC_GET_UCFLAGS                    ( (uint32_t) 166 )
#define WLC_SET_UCFLAGS                    ( (uint32_t) 167 )
#define WLC_GET_PWRIDX                     ( (uint32_t) 168 )
#define WLC_SET_PWRIDX                     ( (uint32_t) 169 )
#define WLC_GET_TSSI                       ( (uint32_t) 170 )
#define WLC_GET_SUP_RATESET_OVERRIDE       ( (uint32_t) 171 )
#define WLC_SET_SUP_RATESET_OVERRIDE       ( (uint32_t) 172 )
#define WLC_GET_PROTECTION_CONTROL         ( (uint32_t) 178 )
#define WLC_SET_PROTECTION_CONTROL         ( (uint32_t) 179 )
#define WLC_GET_PHYLIST                    ( (uint32_t) 180 )
#define WLC_ENCRYPT_STRENGTH               ( (uint32_t) 181 )
#define WLC_DECRYPT_STATUS                 ( (uint32_t) 182 )
#define WLC_GET_KEY_SEQ                    ( (uint32_t) 183 )
#define WLC_GET_SCAN_CHANNEL_TIME          ( (uint32_t) 184 )
#define WLC_SET_SCAN_CHANNEL_TIME          ( (uint32_t) 185 )
#define WLC_GET_SCAN_UNASSOC_TIME          ( (uint32_t) 186 )
#define WLC_SET_SCAN_UNASSOC_TIME          ( (uint32_t) 187 )
#define WLC_GET_SCAN_HOME_TIME             ( (uint32_t) 188 )
#define WLC_SET_SCAN_HOME_TIME             ( (uint32_t) 189 )
#define WLC_GET_SCAN_NPROBES               ( (uint32_t) 190 )
#define WLC_SET_SCAN_NPROBES               ( (uint32_t) 191 )
#define WLC_GET_PRB_RESP_TIMEOUT           ( (uint32_t) 192 )
#define WLC_SET_PRB_RESP_TIMEOUT           ( (uint32_t) 193 )
#define WLC_GET_ATTEN                      ( (uint32_t) 194 )
#define WLC_SET_ATTEN                      ( (uint32_t) 195 )
#define WLC_GET_SHMEM                      ( (uint32_t) 196 )
#define WLC_SET_SHMEM                      ( (uint32_t) 197 )
#define WLC_SET_WSEC_TEST                  ( (uint32_t) 200 )
#define WLC_SCB_DEAUTHENTICATE_FOR_REASON  ( (uint32_t) 201 )
#define WLC_TKIP_COUNTERMEASURES           ( (uint32_t) 202 )
#define WLC_GET_PIOMODE                    ( (uint32_t) 203 )
#define WLC_SET_PIOMODE                    ( (uint32_t) 204 )
#define WLC_SET_ASSOC_PREFER               ( (uint32_t) 205 )
#define WLC_GET_ASSOC_PREFER               ( (uint32_t) 206 )
#define WLC_SET_ROAM_PREFER                ( (uint32_t) 207 )
#define WLC_GET_ROAM_PREFER                ( (uint32_t) 208 )
#define WLC_SET_LED                        ( (uint32_t) 209 )
#define WLC_GET_LED                        ( (uint32_t) 210 )
#define WLC_GET_INTERFERENCE_MODE          ( (uint32_t) 211 )
#define WLC_SET_INTERFERENCE_MODE          ( (uint32_t) 212 )
#define WLC_GET_CHANNEL_QA                 ( (uint32_t) 213 )
#define WLC_START_CHANNEL_QA               ( (uint32_t) 214 )
#define WLC_GET_CHANNEL_SEL                ( (uint32_t) 215 )
#define WLC_START_CHANNEL_SEL              ( (uint32_t) 216 )
#define WLC_GET_VALID_CHANNELS             ( (uint32_t) 217 )
#define WLC_GET_FAKEFRAG                   ( (uint32_t) 218 )
#define WLC_SET_FAKEFRAG                   ( (uint32_t) 219 )
#define WLC_GET_PWROUT_PERCENTAGE          ( (uint32_t) 220 )
#define WLC_SET_PWROUT_PERCENTAGE          ( (uint32_t) 221 )
#define WLC_SET_BAD_FRAME_PREEMPT          ( (uint32_t) 222 )
#define WLC_GET_BAD_FRAME_PREEMPT          ( (uint32_t) 223 )
#define WLC_SET_LEAP_LIST                  ( (uint32_t) 224 )
#define WLC_GET_LEAP_LIST                  ( (uint32_t) 225 )
#define WLC_GET_CWMIN                      ( (uint32_t) 226 )
#define WLC_SET_CWMIN                      ( (uint32_t) 227 )
#define WLC_GET_CWMAX                      ( (uint32_t) 228 )
#define WLC_SET_CWMAX                      ( (uint32_t) 229 )
#define WLC_GET_WET                        ( (uint32_t) 230 )
#define WLC_SET_WET                        ( (uint32_t) 231 )
#define WLC_GET_PUB                        ( (uint32_t) 232 )
#define WLC_GET_KEY_PRIMARY                ( (uint32_t) 235 )
#define WLC_SET_KEY_PRIMARY                ( (uint32_t) 236 )
#define WLC_GET_ACI_ARGS                   ( (uint32_t) 238 )
#define WLC_SET_ACI_ARGS                   ( (uint32_t) 239 )
#define WLC_UNSET_CALLBACK                 ( (uint32_t) 240 )
#define WLC_SET_CALLBACK                   ( (uint32_t) 241 )
#define WLC_GET_RADAR                      ( (uint32_t) 242 )
#define WLC_SET_RADAR                      ( (uint32_t) 243 )
#define WLC_SET_SPECT_MANAGMENT            ( (uint32_t) 244 )
#define WLC_GET_SPECT_MANAGMENT            ( (uint32_t) 245 )
#define WLC_WDS_GET_REMOTE_HWADDR          ( (uint32_t) 246 )
#define WLC_WDS_GET_WPA_SUP                ( (uint32_t) 247 )
#define WLC_SET_CS_SCAN_TIMER              ( (uint32_t) 248 )
#define WLC_GET_CS_SCAN_TIMER              ( (uint32_t) 249 )
#define WLC_MEASURE_REQUEST                ( (uint32_t) 250 )
#define WLC_INIT                           ( (uint32_t) 251 )
#define WLC_SEND_QUIET                     ( (uint32_t) 252 )
#define WLC_KEEPALIVE                      ( (uint32_t) 253 )
#define WLC_SEND_PWR_CONSTRAINT            ( (uint32_t) 254 )
#define WLC_UPGRADE_STATUS                 ( (uint32_t) 255 )
#define WLC_CURRENT_PWR                    ( (uint32_t) 256 )
#define WLC_GET_SCAN_PASSIVE_TIME          ( (uint32_t) 257 )
#define WLC_SET_SCAN_PASSIVE_TIME          ( (uint32_t) 258 )
#define WLC_LEGACY_LINK_BEHAVIOR           ( (uint32_t) 259 )
#define WLC_GET_CHANNELS_IN_COUNTRY        ( (uint32_t) 260 )
#define WLC_GET_COUNTRY_LIST               ( (uint32_t) 261 )
#define WLC_GET_VAR                        ( (uint32_t) 262 )
#define WLC_SET_VAR                        ( (uint32_t) 263 )
#define WLC_NVRAM_GET                      ( (uint32_t) 264 )
#define WLC_NVRAM_SET                      ( (uint32_t) 265 )
#define WLC_NVRAM_DUMP                     ( (uint32_t) 266 )
#define WLC_REBOOT                         ( (uint32_t) 267 )
#define WLC_SET_WSEC_PMK                   ( (uint32_t) 268 )
#define WLC_GET_AUTH_MODE                  ( (uint32_t) 269 )
#define WLC_SET_AUTH_MODE                  ( (uint32_t) 270 )
#define WLC_GET_WAKEENTRY                  ( (uint32_t) 271 )
#define WLC_SET_WAKEENTRY                  ( (uint32_t) 272 )
#define WLC_NDCONFIG_ITEM                  ( (uint32_t) 273 )
#define WLC_NVOTPW                         ( (uint32_t) 274 )
#define WLC_OTPW                           ( (uint32_t) 275 )
#define WLC_IOV_BLOCK_GET                  ( (uint32_t) 276 )
#define WLC_IOV_MODULES_GET                ( (uint32_t) 277 )
#define WLC_SOFT_RESET                     ( (uint32_t) 278 )
#define WLC_GET_ALLOW_MODE                 ( (uint32_t) 279 )
#define WLC_SET_ALLOW_MODE                 ( (uint32_t) 280 )
#define WLC_GET_DESIRED_BSSID              ( (uint32_t) 281 )
#define WLC_SET_DESIRED_BSSID              ( (uint32_t) 282 )
#define WLC_DISASSOC_MYAP                  ( (uint32_t) 283 )
#define WLC_GET_NBANDS                     ( (uint32_t) 284 )
#define WLC_GET_BANDSTATES                 ( (uint32_t) 285 )
#define WLC_GET_WLC_BSS_INFO               ( (uint32_t) 286 )
#define WLC_GET_ASSOC_INFO                 ( (uint32_t) 287 )
#define WLC_GET_OID_PHY                    ( (uint32_t) 288 )
#define WLC_SET_OID_PHY                    ( (uint32_t) 289 )
#define WLC_SET_ASSOC_TIME                 ( (uint32_t) 290 )
#define WLC_GET_DESIRED_SSID               ( (uint32_t) 291 )
#define WLC_GET_CHANSPEC                   ( (uint32_t) 292 )
#define WLC_GET_ASSOC_STATE                ( (uint32_t) 293 )
#define WLC_SET_PHY_STATE                  ( (uint32_t) 294 )
#define WLC_GET_SCAN_PENDING               ( (uint32_t) 295 )
#define WLC_GET_SCANREQ_PENDING            ( (uint32_t) 296 )
#define WLC_GET_PREV_ROAM_REASON           ( (uint32_t) 297 )
#define WLC_SET_PREV_ROAM_REASON           ( (uint32_t) 298 )
#define WLC_GET_BANDSTATES_PI              ( (uint32_t) 299 )
#define WLC_GET_PHY_STATE                  ( (uint32_t) 300 )
#define WLC_GET_BSS_WPA_RSN                ( (uint32_t) 301 )
#define WLC_GET_BSS_WPA2_RSN               ( (uint32_t) 302 )
#define WLC_GET_BSS_BCN_TS                 ( (uint32_t) 303 )
#define WLC_GET_INT_DISASSOC               ( (uint32_t) 304 )
#define WLC_SET_NUM_PEERS                  ( (uint32_t) 305 )
#define WLC_GET_NUM_BSS                    ( (uint32_t) 306 )
#define WLC_GET_WSEC_PMK                   ( (uint32_t) 318 )
#define WLC_GET_RANDOM_BYTES               ( (uint32_t) 319 )
#define WLC_LAST                           ( (uint32_t) 320 )

#define CMN_IOCTL_OFF 0x180
#define WL_OID_BASE        0xFFE41420
#define OID_WL_GETINSTANCE    (WL_OID_BASE + WLC_GET_INSTANCE)
#define OID_WL_GET_FORCELINK    (WL_OID_BASE + WLC_GET_FORCELINK)
#define OID_WL_SET_FORCELINK    (WL_OID_BASE + WLC_SET_FORCELINK)
#define    OID_WL_ENCRYPT_STRENGTH    (WL_OID_BASE + WLC_ENCRYPT_STRENGTH)
#define OID_WL_DECRYPT_STATUS    (WL_OID_BASE + WLC_DECRYPT_STATUS)
#define OID_LEGACY_LINK_BEHAVIOR (WL_OID_BASE + WLC_LEGACY_LINK_BEHAVIOR)
#define OID_WL_NDCONFIG_ITEM (WL_OID_BASE + WLC_NDCONFIG_ITEM)
#define OID_STA_CHANSPEC    (WL_OID_BASE + WLC_GET_CHANSPEC)
#define OID_STA_NBANDS        (WL_OID_BASE + WLC_GET_NBANDS)
#define OID_STA_GET_PHY        (WL_OID_BASE + WLC_GET_OID_PHY)
#define OID_STA_SET_PHY        (WL_OID_BASE + WLC_SET_OID_PHY)
#define OID_STA_ASSOC_TIME    (WL_OID_BASE + WLC_SET_ASSOC_TIME)
#define OID_STA_DESIRED_SSID (WL_OID_BASE + WLC_GET_DESIRED_SSID)
#define OID_STA_SET_PHY_STATE (WL_OID_BASE + WLC_SET_PHY_STATE)
#define OID_STA_SCAN_PENDING    (WL_OID_BASE + WLC_GET_SCAN_PENDING)
#define OID_STA_SCANREQ_PENDING (WL_OID_BASE + WLC_GET_SCANREQ_PENDING)
#define OID_STA_GET_ROAM_REASON (WL_OID_BASE + WLC_GET_PREV_ROAM_REASON)
#define OID_STA_SET_ROAM_REASON (WL_OID_BASE + WLC_SET_PREV_ROAM_REASON)
#define OID_STA_GET_PHY_STATE (WL_OID_BASE + WLC_GET_PHY_STATE)
#define OID_STA_INT_DISASSOC    (WL_OID_BASE + WLC_GET_INT_DISASSOC)
#define OID_STA_SET_NUM_PEERS    (WL_OID_BASE + WLC_SET_NUM_PEERS)
#define OID_STA_GET_NUM_BSS        (WL_OID_BASE + WLC_GET_NUM_BSS)
#define WL_DECRYPT_STATUS_SUCCESS    1
#define WL_DECRYPT_STATUS_FAILURE    2
#define WL_DECRYPT_STATUS_UNKNOWN    3
#define WLC_UPGRADE_SUCCESS            0
#define WLC_UPGRADE_PENDING            1
#define WL_RADIO_SW_DISABLE        (1<<0)
#define WL_RADIO_HW_DISABLE        (1<<1)
#define WL_RADIO_MPC_DISABLE       (1<<2)
#define WL_RADIO_COUNTRY_DISABLE   (1<<3)
#define WL_TXPWR_OVERRIDE          (1U<<31)
#define WL_PHY_PAVARS_LEN          (6)
#define WL_DIAG_INTERRUPT          (1)
#define WL_DIAG_LOOPBACK           (2)
#define WL_DIAG_MEMORY             (3)
#define WL_DIAG_LED                (4)
#define WL_DIAG_REG                (5)
#define WL_DIAG_SROM               (6)
#define WL_DIAG_DMA                (7)
#define WL_DIAGERR_SUCCESS            (0)
#define WL_DIAGERR_FAIL_TO_RUN        (1)
#define WL_DIAGERR_NOT_SUPPORTED      (2)
#define WL_DIAGERR_INTERRUPT_FAIL     (3)
#define WL_DIAGERR_LOOPBACK_FAIL      (4)
#define WL_DIAGERR_SROM_FAIL          (5)
#define WL_DIAGERR_SROM_BADCRC        (6)
#define WL_DIAGERR_REG_FAIL           (7)
#define WL_DIAGERR_MEMORY_FAIL        (8)
#define WL_DIAGERR_NOMEM              (9)
#define WL_DIAGERR_DMA_FAIL           (10)
#define WL_DIAGERR_MEMORY_TIMEOUT     (11)
#define WL_DIAGERR_MEMORY_BADPATTERN  (12)
#define    WLC_BAND_AUTO                (0)
#define    WLC_BAND_5G                  (1)
#define    WLC_BAND_2G                  (2)
#define    WLC_BAND_ALL                 (3)
#define WL_CHAN_FREQ_RANGE_2G           (0)
#define WL_CHAN_FREQ_RANGE_5GL          (1)
#define WL_CHAN_FREQ_RANGE_5GM          (2)
#define WL_CHAN_FREQ_RANGE_5GH          (3)
#define WLC_PHY_TYPE_A               (0)
#define WLC_PHY_TYPE_B               (1)
#define WLC_PHY_TYPE_G               (2)
#define WLC_PHY_TYPE_N               (4)
#define WLC_PHY_TYPE_LP              (5)
#define WLC_PHY_TYPE_SSN             (6)
#define WLC_PHY_TYPE_NULL          (0xf)
#define WLC_MACMODE_DISABLED         (0)
#define WLC_MACMODE_DENY             (1)
#define WLC_MACMODE_ALLOW            (2)
#define GMODE_LEGACY_B               (0)
#define GMODE_AUTO                   (1)
#define GMODE_ONLY                   (2)
#define GMODE_B_DEFERRED             (3)
#define GMODE_PERFORMANCE            (4)
#define GMODE_LRS                    (5)
#define GMODE_MAX                    (6)
#define WLC_PLCP_AUTO               (-1)
#define WLC_PLCP_SHORT               (0)
#define WLC_PLCP_LONG                (1)
#define WLC_PROTECTION_AUTO         (-1)
#define WLC_PROTECTION_OFF           (0)
#define WLC_PROTECTION_ON            (1)
#define WLC_PROTECTION_MMHDR_ONLY    (2)
#define WLC_PROTECTION_CTS_ONLY      (3)
#define WLC_PROTECTION_CTL_OFF       (0)
#define WLC_PROTECTION_CTL_LOCAL     (1)
#define WLC_PROTECTION_CTL_OVERLAP   (2)
#define WLC_N_PROTECTION_OFF         (0)
#define WLC_N_PROTECTION_OPTIONAL    (1)
#define WLC_N_PROTECTION_20IN40      (2)
#define WLC_N_PROTECTION_MIXEDMODE   (3)
#define WLC_N_PREAMBLE_MIXEDMODE     (0)
#define WLC_N_PREAMBLE_GF            (1)
#define WLC_N_BW_20ALL               (0)
#define WLC_N_BW_40ALL               (1)
#define WLC_N_BW_20IN2G_40IN5G       (2)
#define WLC_N_TXRX_CHAIN0            (0)
#define WLC_N_TXRX_CHAIN1            (1)
#define WLC_N_SGI_20              (0x01)
#define WLC_N_SGI_40              (0x02)
#define PM_OFF                       (0)
#define PM_MAX                       (1)
#define PM_FAST                      (2)
#define INTERFERE_NONE               (0)
#define NON_WLAN                     (1)
#define WLAN_MANUAL                  (2)
#define WLAN_AUTO                    (3)
#define AUTO_ACTIVE             (1 << 7)

typedef struct wl_aci_args
{
  int32_t enter_aci_thresh;
  int32_t exit_aci_thresh;
  int32_t usec_spin;
  int32_t glitch_delay;
  uint16_t nphy_adcpwr_enter_thresh;
  uint16_t nphy_adcpwr_exit_thresh;
  uint16_t nphy_repeat_ctr;
  uint16_t nphy_num_samples;
  uint16_t nphy_undetect_window_sz;
  uint16_t nphy_b_energy_lo_aci;
  uint16_t nphy_b_energy_md_aci;
  uint16_t nphy_b_energy_hi_aci;
} wl_aci_args_t;

#define WL_ACI_ARGS_LEGACY_LENGTH    16

typedef struct
{
  int32_t npulses;
  int32_t ncontig;
  int32_t min_pw;
  int32_t max_pw;
  uint16_t thresh0;
  uint16_t thresh1;
  uint16_t blank;
  uint16_t fmdemodcfg;
  int32_t npulses_lp;
  int32_t min_pw_lp;
  int32_t max_pw_lp;
  int32_t min_fm_lp;
  int32_t max_deltat_lp;
  int32_t min_deltat;
  int32_t max_deltat;
  uint16_t autocorr;
  uint16_t st_level_time;
  uint16_t t2_min;
  uint32_t version;
} wl_radar_args_t;

#define WL_RADAR_ARGS_VERSION 1
#define WL_RADAR_DETECTOR_OFF 0
#define WL_RADAR_DETECTOR_ON  1
#define WL_RADAR_SIMULATED    2
#define WL_RSSI_ANT_VERSION   1
#define WL_RSSI_ANT_MAX       4

typedef struct
{
  uint32_t version;
  uint32_t count;
  int8_t rssi_ant[WL_RSSI_ANT_MAX];
} wl_rssi_ant_t;

#define WL_DFS_CACSTATE_IDLE        0
#define WL_DFS_CACSTATE_PREISM_CAC  1
#define WL_DFS_CACSTATE_ISM         2
#define WL_DFS_CACSTATE_CSA         3
#define WL_DFS_CACSTATE_POSTISM_CAC 4
#define WL_DFS_CACSTATE_PREISM_OOC  5
#define WL_DFS_CACSTATE_POSTISM_OOC 6
#define WL_DFS_CACSTATES            7

typedef struct
{
  uint32_t state;
  uint32_t duration;
  wl_chanspec_t chanspec_cleared;
  uint16_t pad;
} wl_dfs_status_t;

#define NUM_PWRCTRL_RATES 12

typedef struct
{
  uint8_t txpwr_band_max[NUM_PWRCTRL_RATES];
  uint8_t txpwr_limit[NUM_PWRCTRL_RATES];
  uint8_t txpwr_local_max;
  uint8_t txpwr_local_constraint;
  uint8_t txpwr_chan_reg_max;
  uint8_t txpwr_target[2][NUM_PWRCTRL_RATES];
  uint8_t txpwr_est_pout[2];
  uint8_t txpwr_opo[NUM_PWRCTRL_RATES];
  uint8_t txpwr_bphy_cck_max[NUM_PWRCTRL_RATES];
  uint8_t txpwr_bphy_ofdm_max;
  uint8_t txpwr_aphy_max[NUM_PWRCTRL_RATES];
  int8_t txpwr_antgain[2];
  uint8_t txpwr_est_pout_gofdm;
} tx_power_legacy_t;

#define WL_TX_POWER_RATES         45
#define WL_TX_POWER_CCK_FIRST     0
#define WL_TX_POWER_CCK_NUM       4
#define WL_TX_POWER_OFDM_FIRST    4
#define WL_TX_POWER_OFDM_NUM      8
#define WL_TX_POWER_MCS_SISO_NUM  8
#define WL_TX_POWER_MCS20_FIRST   12
#define WL_TX_POWER_MCS20_NUM     16
#define WL_TX_POWER_MCS40_FIRST   28
#define WL_TX_POWER_MCS40_NUM     17
#define WL_TX_POWER_MCS20SISO_NUM 8
#define WL_TX_POWER_MCS40_LAST    44
#define WL_TX_POWER_F_ENABLED     1
#define WL_TX_POWER_F_HW          2
#define WL_TX_POWER_F_MIMO        4
#define WL_TX_POWER_F_SISO        8
#define WL_TX_POWER_F_40M_CAP     16

typedef struct
{
  uint32_t flags;
  wl_chanspec_t chanspec;
  wl_chanspec_t local_chanspec;
  uint8_t local_max;
  uint8_t local_constraint;
  int8_t antgain[2];
  uint8_t rf_cores;
  uint8_t est_pout[4];
  uint8_t est_pout_cck;
  uint8_t user_limit[WL_TX_POWER_RATES];
  uint8_t reg_limit[WL_TX_POWER_RATES];
  uint8_t board_limit[WL_TX_POWER_RATES];
  uint8_t target[WL_TX_POWER_RATES];
} tx_power_t;

typedef struct tx_inst_power
{
  uint8_t txpwr_est_pout[2];
  uint8_t txpwr_est_pout_gofdm;
} tx_inst_power_t;

#define WLC_MEASURE_TPC           1
#define WLC_MEASURE_CHANNEL_BASIC 2
#define WLC_MEASURE_CHANNEL_CCA   3
#define WLC_MEASURE_CHANNEL_RPI   4
#define SPECT_MNGMT_OFF         0
#define SPECT_MNGMT_LOOSE_11H   1
#define SPECT_MNGMT_STRICT_11H  2
#define SPECT_MNGMT_STRICT_11D  3
#define SPECT_MNGMT_LOOSE_11H_D 4
#define WL_CHAN_VALID_HW    (1 << 0)
#define WL_CHAN_VALID_SW    (1 << 1)
#define WL_CHAN_BAND_5G     (1 << 2)
#define WL_CHAN_RADAR       (1 << 3)
#define WL_CHAN_INACTIVE    (1 << 4)
#define WL_CHAN_PASSIVE     (1 << 5)
#define WL_CHAN_RESTRICTED  (1 << 6)
#define    WL_BTC_DISABLE   0
#define WL_BTC_ENABLE       (1 << 0)
#define WL_BTC_PREMPT       (1 << 1)
#define WL_BTC_PARTIAL      (1 << 2)
#define WL_BTC_DEFAULT      (1 << 3)
#define WL_BTC_HYBRID       (WL_BTC_ENABLE | WL_BTC_PARTIAL)
#define WL_INF_BTC_DISABLE  0
#define WL_INF_BTC_ENABLE   1
#define WL_INF_BTC_AUTO     3
#define WL_BTC_DEFWIRE      0
#define WL_BTC_2WIRE        2
#define WL_BTC_3WIRE        3
#define WL_BTC_4WIRE        4
#define WL_BTC_FLAG_PREMPT               (1 << 0)
#define WL_BTC_FLAG_BT_DEF               (1 << 1)
#define WL_BTC_FLAG_ACTIVE_PROT          (1 << 2)
#define WL_BTC_FLAG_SIM_RSP              (1 << 3)
#define WL_BTC_FLAG_PS_PROTECT           (1 << 4)
#define WL_BTC_FLAG_SIM_TX_LP            (1 << 5)
#define WL_BTC_FLAG_ECI                  (1 << 6)
#define WL_ERROR_VAL       0x00000001
#define WL_TRACE_VAL       0x00000002
#define WL_PRHDRS_VAL      0x00000004
#define WL_PRPKT_VAL       0x00000008
#define WL_INFORM_VAL      0x00000010
#define WL_TMP_VAL         0x00000020
#define WL_OID_VAL         0x00000040
#define WL_RATE_VAL        0x00000080
#define WL_ASSOC_VAL       0x00000100
#define WL_PRUSR_VAL       0x00000200
#define WL_PS_VAL          0x00000400
#define WL_TXPWR_VAL       0x00000800
#define WL_PORT_VAL        0x00001000
#define WL_DUAL_VAL        0x00002000
#define WL_WSEC_VAL        0x00004000
#define WL_WSEC_DUMP_VAL   0x00008000
#define WL_LOG_VAL         0x00010000
#define WL_NRSSI_VAL       0x00020000
#define WL_LOFT_VAL        0x00040000
#define WL_REGULATORY_VAL  0x00080000
#define WL_PHYCAL_VAL      0x00100000
#define WL_RADAR_VAL       0x00200000
#define WL_MPC_VAL         0x00400000
#define WL_APSTA_VAL       0x00800000
#define WL_DFS_VAL         0x01000000
#define WL_BA_VAL          0x02000000
#define WL_MBSS_VAL        0x04000000
#define WL_CAC_VAL         0x08000000
#define WL_AMSDU_VAL       0x10000000
#define WL_AMPDU_VAL       0x20000000
#define WL_FFPLD_VAL       0x40000000
#define WL_DPT_VAL         0x00000001
#define WL_SCAN_VAL        0x00000002
#define WL_WOWL_VAL        0x00000004
#define WL_COEX_VAL        0x00000008
#define WL_RTDC_VAL        0x00000010
#define WL_BTA_VAL         0x00000040
#define WL_LED_NUMGPIO        16
#define WL_LED_OFF            0
#define WL_LED_ON             1
#define WL_LED_ACTIVITY       2
#define WL_LED_RADIO          3
#define WL_LED_ARADIO         4
#define WL_LED_BRADIO         5
#define WL_LED_BGMODE         6
#define WL_LED_WI1            7
#define WL_LED_WI2            8
#define WL_LED_WI3            9
#define WL_LED_ASSOC          10
#define WL_LED_INACTIVE       11
#define WL_LED_ASSOCACT       12
#define WL_LED_NUMBEHAVIOR    13
#define WL_LED_BEH_MASK       0x7f
#define WL_LED_AL_MASK        0x80
#define WL_NUMCHANNELS        64
#define WL_NUMCHANSPECS       100
#define WL_WDS_WPA_ROLE_AUTH  0
#define WL_WDS_WPA_ROLE_SUP   1
#define WL_WDS_WPA_ROLE_AUTO  255
#define WL_EVENTING_MASK_LEN  ((WLC_E_LAST + 7) / 8)

#define VNDR_IE_CMD_LEN       4
#define VNDR_IE_BEACON_FLAG   0x1
#define VNDR_IE_PRBRSP_FLAG   0x2
#define VNDR_IE_ASSOCRSP_FLAG 0x4
#define VNDR_IE_AUTHRSP_FLAG  0x8
#define VNDR_IE_PRBREQ_FLAG   0x10
#define VNDR_IE_ASSOCREQ_FLAG 0x20
#define VNDR_IE_CUSTOM_FLAG   0x100
#define VNDR_IE_INFO_HDR_LEN  (sizeof(uint32_t))

struct wl_vndr_ie
{
  uint8_t id;
  uint8_t len;
  uint8_t oui[3];
  uint8_t data[1];
};
typedef struct wl_vndr_ie wl_vndr_ie_t;

typedef struct
{
  uint32_t pktflag;
  wl_vndr_ie_t vndr_ie_data;
} vndr_ie_info_t;

typedef struct
{
  int32_t iecount;
  vndr_ie_info_t vndr_ie_list[1];
} vndr_ie_buf_t;

typedef struct
{
  int8_t cmd[VNDR_IE_CMD_LEN];
  vndr_ie_buf_t vndr_ie_buffer;
} vndr_ie_setbuf_t;

#define WL_JOIN_PREF_RSSI    1
#define WL_JOIN_PREF_WPA     2
#define WL_JOIN_PREF_BAND    3
#define WLJP_BAND_ASSOC_PREF 255
#define WL_WPA_ACP_MCS_ANY   "\x00\x00\x00\x00"

struct tsinfo_arg
{
  uint8_t octets[3];
};

#define    NFIFO                6
#define    WL_CNT_T_VERSION     6
#define    WL_CNT_EXT_T_VERSION 1

typedef struct
{
  uint16_t  version;    /* see definition of WL_CNT_T_VERSION */
  uint16_t  length;     /* length of entire structure */

  /* transmit stat counters */

  uint32_t  txframe;    /* tx data frames */
  uint32_t  txbyte;     /* tx data bytes */
  uint32_t  txretrans;  /* tx mac retransmits */
  uint32_t  txerror;    /* tx data errors (derived: sum of others) */
  uint32_t  txctl;      /* tx management frames */
  uint32_t  txprshort;  /* tx short preamble frames */
  uint32_t  txserr;     /* tx status errors */
  uint32_t  txnobuf;    /* tx out of buffers errors */
  uint32_t  txnoassoc;  /* tx discard because we're not associated */
  uint32_t  txrunt;     /* tx runt frames */
  uint32_t  txchit;     /* tx header cache hit (fastpath) */
  uint32_t  txcmiss;    /* tx header cache miss (slowpath) */

  /* transmit chip error counters */

  uint32_t  txuflo;     /* tx fifo underflows */
  uint32_t  txphyerr;   /* tx phy errors (indicated in tx status) */
  uint32_t  txphycrs;   /* PR8861/8963 counter */

  /* receive stat counters */

  uint32_t  rxframe;      /* rx data frames */
  uint32_t  rxbyte;       /* rx data bytes */
  uint32_t  rxerror;      /* rx data errors (derived: sum of others) */
  uint32_t  rxctl;        /* rx management frames */
  uint32_t  rxnobuf;      /* rx out of buffers errors */
  uint32_t  rxnondata;    /* rx non data frames in the data channel errors */
  uint32_t  rxbadds;      /* rx bad DS errors */
  uint32_t  rxbadcm;      /* rx bad control or management frames */
  uint32_t  rxfragerr;    /* rx fragmentation errors */
  uint32_t  rxrunt;       /* rx runt frames */
  uint32_t  rxgiant;      /* rx giant frames */
  uint32_t  rxnoscb;      /* rx no scb error */
  uint32_t  rxbadproto;   /* rx invalid frames */
  uint32_t  rxbadsrcmac;  /* rx frames with Invalid Src Mac */
  uint32_t  rxbadda;      /* rx frames tossed for invalid da */
  uint32_t  rxfilter;     /* rx frames filtered out */

  /* receive chip error counters */

  uint32_t  rxoflo;         /* rx fifo overflow errors */
  uint32_t  rxuflo[NFIFO];  /* rx dma descriptor underflow errors */

  uint32_t  d11cnt_txrts_off;   /* d11cnt txrts value when reset d11cnt */
  uint32_t  d11cnt_rxcrc_off;   /* d11cnt rxcrc value when reset d11cnt */
  uint32_t  d11cnt_txnocts_off; /* d11cnt txnocts value when reset d11cnt */

  /* misc counters */

  uint32_t  dmade;                  /* tx/rx dma descriptor errors */
  uint32_t  dmada;                  /* tx/rx dma data errors */
  uint32_t  dmape;                  /* tx/rx dma descriptor protocol errors */
  uint32_t  reset;                  /* reset count */
  uint32_t  tbtt;                   /* cnts the TBTT int's */
  uint32_t  txdmawar;               /* # occurrences of PR15420 workaround */
  uint32_t  pkt_callback_reg_fail;  /* callbacks register failure */

  /* MAC counters: 32-bit version of d11.h's macstat_t */

  uint32_t  txallfrm;   /* total number of frames sent, incl. Data, ACK, RTS, CTS,
                         * Control Management (includes retransmissions) */
  uint32_t  txrtsfrm;   /* number of RTS sent out by the MAC */
  uint32_t  txctsfrm;   /* number of CTS sent out by the MAC */
  uint32_t  txackfrm;   /* number of ACK frames sent out */
  uint32_t  txdnlfrm;   /* Not used */
  uint32_t  txbcnfrm;   /* beacons transmitted */
  uint32_t  txfunfl[8]; /* per-fifo tx underflows */
  uint32_t  txtplunfl;  /* Template underflows (mac was too slow to transmit ACK/CTS
                         * or BCN) */
  uint32_t  txphyerror; /* Transmit phy error, type of error is reported in tx-status for
                         * driver enqueued frames */

  uint32_t  rxfrmtoolong;   /* Received frame longer than legal limit (2346 bytes) */
  uint32_t  rxfrmtooshrt;   /* Received frame did not contain enough bytes for its frame type */
  uint32_t  rxinvmachdr;    /* Either the protocol version != 0 or frame type not
                             * data/control/management */

  uint32_t  rxbadfcs;     /* number of frames for which the CRC check failed in the MAC */
  uint32_t  rxbadplcp;    /* parity check of the PLCP header failed */
  uint32_t  rxcrsglitch;  /* PHY was able to correlate the preamble but not the header */
  uint32_t  rxstrt;       /* Number of received frames with a good PLCP
                           * (i.e. passing parity check) */

  uint32_t  rxdfrmucastmbss; /* Number of received DATA frames with good FCS and matching RA */
  uint32_t  rxmfrmucastmbss; /* number of received mgmt frames with good FCS and matching RA */
  uint32_t  rxcfrmucast;     /* number of received CNTRL frames with good FCS and matching RA */
  uint32_t  rxrtsucast;      /* number of unicast RTS addressed to the MAC (good FCS) */
  uint32_t  rxctsucast;      /* number of unicast CTS addressed to the MAC (good FCS) */
  uint32_t  rxackucast;      /* number of ucast ACKS received (good FCS) */
  uint32_t  rxdfrmocast;     /* number of received DATA frames (good FCS and not matching RA) */
  uint32_t  rxmfrmocast;     /* number of received MGMT frames (good FCS and not matching RA) */
  uint32_t  rxcfrmocast;     /* number of received CNTRL frame (good FCS and not matching RA) */
  uint32_t  rxrtsocast;      /* number of received RTS not addressed to the MAC */
  uint32_t  rxctsocast;      /* number of received CTS not addressed to the MAC */
  uint32_t  rxdfrmmcast;     /* number of RX Data multicast frames received by the MAC */
  uint32_t  rxmfrmmcast;     /* number of RX Management multicast frames received by the MAC */
  uint32_t  rxcfrmmcast;     /* number of RX Control multicast frames received by the MAC
                              * (unlikely to see these) */

  uint32_t  rxbeaconmbss;    /* beacons received from member of BSS */
  uint32_t  rxdfrmucastobss; /* number of unicast frames addressed to the MAC from
                              * other BSS (WDS FRAME) */

  uint32_t  rxbeaconobss;   /* beacons received from other BSS */
  uint32_t  rxrsptmout;     /* Number of response timeouts for transmitted frames
                             * expecting a response */

  uint32_t  bcntxcancl; /* transmit beacons canceled due to receipt of beacon (IBSS) */
  uint32_t  rxf0ovfl;   /* Number of receive fifo 0 overflows */
  uint32_t  rxf1ovfl;   /* Number of receive fifo 1 overflows (obsolete) */
  uint32_t  rxf2ovfl;   /* Number of receive fifo 2 overflows (obsolete) */
  uint32_t  txsfovfl;   /* Number of transmit status fifo overflows (obsolete) */
  uint32_t  pmqovfl;    /* Number of PMQ overflows */
  uint32_t  rxcgprqfrm; /* Number of received Probe requests that made it into
                         * the PRQ fifo */

  uint32_t  rxcgprsqovfl;   /* Rx Probe Request Queue overflow in the AP */
  uint32_t  txcgprsfail;    /* Tx Probe Response Fail. AP sent probe response but did
                             * not get ACK */

  uint32_t  txcgprssuc;   /* Tx Probe Response Success (ACK was received) */
  uint32_t  prs_timeout;  /* Number of probe requests that were dropped from the PRQ
                           * fifo because a probe response could not be sent out within
                           * the time limit defined in M_PRS_MAXTIME */

  uint32_t  rxnack;     /* XXX Number of NACKS received (Afterburner) */
  uint32_t  frmscons;   /* XXX Number of frames completed without transmission because of an
                         * Afterburner re-queue */

  uint32_t  txnack;        /* XXX Number of NACKs transmitted (Afterburner) */
  uint32_t  txglitch_nack; /* obsolete */
  uint32_t  txburst;       /* obsolete */

  /* 802.11 MIB counters, pp. 614 of 802.11 reaff doc. */

  uint32_t  txfrag;     /* dot11TransmittedFragmentCount */
  uint32_t  txmulti;    /* dot11MulticastTransmittedFrameCount */
  uint32_t  txfail;     /* dot11FailedCount */
  uint32_t  txretry;    /* dot11RetryCount */
  uint32_t  txretrie;   /* dot11MultipleRetryCount */
  uint32_t  rxdup;      /* dot11FrameduplicateCount */
  uint32_t  txrts;      /* dot11RTSSuccessCount */
  uint32_t  txnocts;    /* dot11RTSFailureCount */
  uint32_t  txnoack;    /* dot11ACKFailureCount */
  uint32_t  rxfrag;     /* dot11ReceivedFragmentCount */
  uint32_t  rxmulti;    /* dot11MulticastReceivedFrameCount */
  uint32_t  rxcrc;      /* dot11FCSErrorCount */
  uint32_t  txfrmsnt;   /* dot11TransmittedFrameCount (bogus MIB?) */
  uint32_t  rxundec;    /* dot11WEPUndecryptableCount */

  /* WPA2 counters (see rxundec for DecryptFailureCount) */

  uint32_t  tkipmicfaill; /* TKIPLocalMICFailures */
  uint32_t  tkipcntrmsr;  /* TKIPCounterMeasuresInvoked */
  uint32_t  tkipreplay;   /* TKIPReplays */
  uint32_t  ccmpfmterr;   /* CCMPFormatErrors */
  uint32_t  ccmpreplay;   /* CCMPReplays */
  uint32_t  ccmpundec;    /* CCMPDecryptErrors */
  uint32_t  fourwayfail;  /* FourWayHandshakeFailures */
  uint32_t  wepundec;     /* dot11WEPUndecryptableCount */
  uint32_t  wepicverr;    /* dot11WEPICVErrorCount */
  uint32_t  decsuccess;   /* DecryptSuccessCount */
  uint32_t  tkipicverr;   /* TKIPICVErrorCount */
  uint32_t  wepexcluded;  /* dot11WEPExcludedCount */

  uint32_t  rxundec_mcst;   /* dot11WEPUndecryptableCount */

  /* WPA2 counters (see rxundec for DecryptFailureCount) */

  uint32_t  tkipmicfaill_mcst; /* TKIPLocalMICFailures */
  uint32_t  tkipcntrmsr_mcst;  /* TKIPCounterMeasuresInvoked */
  uint32_t  tkipreplay_mcst;   /* TKIPReplays */
  uint32_t  ccmpfmterr_mcst;   /* CCMPFormatErrors */
  uint32_t  ccmpreplay_mcst;   /* CCMPReplays */
  uint32_t  ccmpundec_mcst;    /* CCMPDecryptErrors */
  uint32_t  fourwayfail_mcst;  /* FourWayHandshakeFailures */
  uint32_t  wepundec_mcst;     /* dot11WEPUndecryptableCount */
  uint32_t  wepicverr_mcst;    /* dot11WEPICVErrorCount */
  uint32_t  decsuccess_mcst;   /* DecryptSuccessCount */
  uint32_t  tkipicverr_mcst;   /* TKIPICVErrorCount */
  uint32_t  wepexcluded_mcst;  /* dot11WEPExcludedCount */

  uint32_t  txchanrej;   /* Tx frames suppressed due to channel rejection */
  uint32_t  txexptime;   /* Tx frames suppressed due to timer expiration */
  uint32_t  psmwds;      /* Count PSM watchdogs */
  uint32_t  phywatchdog; /* Count Phy watchdogs (triggered by ucode) */

  /* MBSS counters, AP only */

  uint32_t  prq_entries_handled;         /* PRQ entries read in */
  uint32_t  prq_undirected_entries;      /*    which were bcast bss & ssid */
  uint32_t  prq_bad_entries;             /*    which could not be translated to info */
  uint32_t  atim_suppress_count;         /* TX suppressions on ATIM fifo */
  uint32_t  bcn_template_not_ready;      /* Template marked in use on send bcn ... */
  uint32_t  bcn_template_not_ready_done; /* ...but "DMA done" interrupt rcvd */
  uint32_t  late_tbtt_dpc;               /* TBTT DPC did not happen in time */

  /* per-rate receive stat counters */

  uint32_t  rx1mbps;    /* packets rx at 1Mbps */
  uint32_t  rx2mbps;    /* packets rx at 2Mbps */
  uint32_t  rx5mbps5;   /* packets rx at 5.5Mbps */
  uint32_t  rx6mbps;    /* packets rx at 6Mbps */
  uint32_t  rx9mbps;    /* packets rx at 9Mbps */
  uint32_t  rx11mbps;   /* packets rx at 11Mbps */
  uint32_t  rx12mbps;   /* packets rx at 12Mbps */
  uint32_t  rx18mbps;   /* packets rx at 18Mbps */
  uint32_t  rx24mbps;   /* packets rx at 24Mbps */
  uint32_t  rx36mbps;   /* packets rx at 36Mbps */
  uint32_t  rx48mbps;   /* packets rx at 48Mbps */
  uint32_t  rx54mbps;   /* packets rx at 54Mbps */
  uint32_t  rx108mbps;  /* packets rx at 108mbps */
  uint32_t  rx162mbps;  /* packets rx at 162mbps */
  uint32_t  rx216mbps;  /* packets rx at 216 mbps */
  uint32_t  rx270mbps;  /* packets rx at 270 mbps */
  uint32_t  rx324mbps;  /* packets rx at 324 mbps */
  uint32_t  rx378mbps;  /* packets rx at 378 mbps */
  uint32_t  rx432mbps;  /* packets rx at 432 mbps */
  uint32_t  rx486mbps;  /* packets rx at 486 mbps */
  uint32_t  rx540mbps;  /* packets rx at 540 mbps */

  /* pkteng rx frame stats */

  uint32_t  pktengrxducast; /* unicast frames rxed by the pkteng code */
  uint32_t  pktengrxdmcast; /* multicast frames rxed by the pkteng code */

  uint32_t  rfdisable;         /* count of radio disables */
  uint32_t  bphy_rxcrsglitch;  /* PHY count of bphy glitches */

  uint32_t  txmpdu_sgi;    /* count for sgi transmit */
  uint32_t  rxmpdu_sgi;    /* count for sgi received */
  uint32_t  txmpdu_stbc;   /* count for stbc transmit */
  uint32_t  rxmpdu_stbc;   /* count for stbc received */
} wl_cnt_ver_six_t;

typedef struct
{
  uint16_t  version;    /* see definition of WL_CNT_T_VERSION */
  uint16_t  length;     /* length of entire structure */

  /* transmit stat counters */

  uint32_t  txframe;    /* tx data frames */
  uint32_t  txbyte;     /* tx data bytes */
  uint32_t  txretrans;  /* tx mac retransmits */
  uint32_t  txerror;    /* tx data errors (derived: sum of others) */
  uint32_t  txctl;      /* tx management frames */
  uint32_t  txprshort;  /* tx short preamble frames */
  uint32_t  txserr;     /* tx status errors */
  uint32_t  txnobuf;    /* tx out of buffers errors */
  uint32_t  txnoassoc;  /* tx discard because we're not associated */
  uint32_t  txrunt;     /* tx runt frames */
  uint32_t  txchit;     /* tx header cache hit (fastpath) */
  uint32_t  txcmiss;    /* tx header cache miss (slowpath) */

  /* transmit chip error counters */

  uint32_t  txuflo;     /* tx fifo underflows */
  uint32_t  txphyerr;   /* tx phy errors (indicated in tx status) */
  uint32_t  txphycrs;   /* PR8861/8963 counter */

  /* receive stat counters */

  uint32_t  rxframe;     /* rx data frames */
  uint32_t  rxbyte;      /* rx data bytes */
  uint32_t  rxerror;     /* rx data errors (derived: sum of others) */
  uint32_t  rxctl;       /* rx management frames */
  uint32_t  rxnobuf;     /* rx out of buffers errors */
  uint32_t  rxnondata;   /* rx non data frames in the data channel errors */
  uint32_t  rxbadds;     /* rx bad DS errors */
  uint32_t  rxbadcm;     /* rx bad control or management frames */
  uint32_t  rxfragerr;   /* rx fragmentation errors */
  uint32_t  rxrunt;      /* rx runt frames */
  uint32_t  rxgiant;     /* rx giant frames */
  uint32_t  rxnoscb;     /* rx no scb error */
  uint32_t  rxbadproto;  /* rx invalid frames */
  uint32_t  rxbadsrcmac; /* rx frames with Invalid Src Mac */
  uint32_t  rxbadda;     /* rx frames tossed for invalid da */
  uint32_t  rxfilter;    /* rx frames filtered out */

  /* receive chip error counters */

  uint32_t  rxoflo;         /* rx fifo overflow errors */
  uint32_t  rxuflo[NFIFO];  /* rx dma descriptor underflow errors */

  uint32_t  d11cnt_txrts_off;   /* d11cnt txrts value when reset d11cnt */
  uint32_t  d11cnt_rxcrc_off;   /* d11cnt rxcrc value when reset d11cnt */
  uint32_t  d11cnt_txnocts_off; /* d11cnt txnocts value when reset d11cnt */

  /* misc counters */

  uint32_t  dmade;                  /* tx/rx dma descriptor errors */
  uint32_t  dmada;                  /* tx/rx dma data errors */
  uint32_t  dmape;                  /* tx/rx dma descriptor protocol errors */
  uint32_t  reset;                  /* reset count */
  uint32_t  tbtt;                   /* cnts the TBTT int's */
  uint32_t  txdmawar;               /* # occurrences of PR15420 workaround */
  uint32_t  pkt_callback_reg_fail;  /* callbacks register failure */

  /* MAC counters: 32-bit version of d11.h's macstat_t */

  uint32_t  txallfrm;   /* total number of frames sent, incl. Data, ACK, RTS, CTS,
                         * Control Management (includes retransmissions) */
  uint32_t  txrtsfrm;   /* number of RTS sent out by the MAC */
  uint32_t  txctsfrm;   /* number of CTS sent out by the MAC */
  uint32_t  txackfrm;   /* number of ACK frames sent out */
  uint32_t  txdnlfrm;   /* Not used */
  uint32_t  txbcnfrm;   /* beacons transmitted */
  uint32_t  txfunfl[8]; /* per-fifo tx underflows */
  uint32_t  txtplunfl;  /* Template underflows (mac was too slow to transmit ACK/CTS
                         * or BCN) */
  uint32_t  txphyerror; /* Transmit phy error, type of error is reported in tx-status for
                         * driver enqueued frames */

  uint32_t  rxfrmtoolong;   /* Received frame longer than legal limit (2346 bytes) */
  uint32_t  rxfrmtooshrt;   /* Received frame did not contain enough bytes for its frame type */
  uint32_t  rxinvmachdr;    /* Either the protocol version != 0 or frame type not
                             * data/control/management */

  uint32_t  rxbadfcs;     /* number of frames for which the CRC check failed in the MAC */
  uint32_t  rxbadplcp;    /* parity check of the PLCP header failed */
  uint32_t  rxcrsglitch;  /* PHY was able to correlate the preamble but not the header */
  uint32_t  rxstrt;       /* Number of received frames with a good PLCP
                           * (i.e. passing parity check) */

  uint32_t  rxdfrmucastmbss; /* Number of received DATA frames with good FCS and matching RA */
  uint32_t  rxmfrmucastmbss; /* number of received mgmt frames with good FCS and matching RA */
  uint32_t  rxcfrmucast;     /* number of received CNTRL frames with good FCS and matching RA */
  uint32_t  rxrtsucast;      /* number of unicast RTS addressed to the MAC (good FCS) */
  uint32_t  rxctsucast;      /* number of unicast CTS addressed to the MAC (good FCS) */
  uint32_t  rxackucast;      /* number of ucast ACKS received (good FCS) */
  uint32_t  rxdfrmocast;     /* number of received DATA frames (good FCS and not matching RA) */
  uint32_t  rxmfrmocast;     /* number of received MGMT frames (good FCS and not matching RA) */
  uint32_t  rxcfrmocast;     /* number of received CNTRL frame (good FCS and not matching RA) */
  uint32_t  rxrtsocast;      /* number of received RTS not addressed to the MAC */
  uint32_t  rxctsocast;      /* number of received CTS not addressed to the MAC */
  uint32_t  rxdfrmmcast;     /* number of RX Data multicast frames received by the MAC */
  uint32_t  rxmfrmmcast;     /* number of RX Management multicast frames received by the MAC */
  uint32_t  rxcfrmmcast;     /* number of RX Control multicast frames received by the MAC
                              * (unlikely to see these) */
  uint32_t  rxbeaconmbss;    /* beacons received from member of BSS */
  uint32_t  rxdfrmucastobss; /* number of unicast frames addressed to the MAC from
                              * other BSS (WDS FRAME) */
  uint32_t  rxbeaconobss;    /* beacons received from other BSS */
  uint32_t  rxrsptmout;      /* Number of response timeouts for transmitted frames
                              * expecting a response */

  uint32_t  bcntxcancl; /* transmit beacons canceled due to receipt of beacon (IBSS) */
  uint32_t  rxf0ovfl;   /* Number of receive fifo 0 overflows */
  uint32_t  rxf1ovfl;   /* Number of receive fifo 1 overflows (obsolete) */
  uint32_t  rxf2ovfl;   /* Number of receive fifo 2 overflows (obsolete) */
  uint32_t  txsfovfl;   /* Number of transmit status fifo overflows (obsolete) */
  uint32_t  pmqovfl;    /* Number of PMQ overflows */
  uint32_t  rxcgprqfrm; /* Number of received Probe requests that made it into
                         * the PRQ fifo */

  uint32_t  rxcgprsqovfl;   /* Rx Probe Request Queue overflow in the AP */
  uint32_t  txcgprsfail;    /* Tx Probe Response Fail. AP sent probe response but did
                             * not get ACK */
  uint32_t  txcgprssuc;     /* Tx Probe Response Success (ACK was received) */
  uint32_t  prs_timeout;    /* Number of probe requests that were dropped from the PRQ
                             * fifo because a probe response could not be sent out within
                             * the time limit defined in M_PRS_MAXTIME */
  uint32_t  rxnack;         /* obsolete */
  uint32_t  frmscons;       /* obsolete */
  uint32_t  txnack;         /* obsolete */
  uint32_t  txglitch_nack;  /* obsolete */
  uint32_t  txburst;        /* obsolete */

  /* 802.11 MIB counters, pp. 614 of 802.11 reaff doc. */

  uint32_t  txfrag;     /* dot11TransmittedFragmentCount */
  uint32_t  txmulti;    /* dot11MulticastTransmittedFrameCount */
  uint32_t  txfail;     /* dot11FailedCount */
  uint32_t  txretry;    /* dot11RetryCount */
  uint32_t  txretrie;   /* dot11MultipleRetryCount */
  uint32_t  rxdup;      /* dot11FrameduplicateCount */
  uint32_t  txrts;      /* dot11RTSSuccessCount */
  uint32_t  txnocts;    /* dot11RTSFailureCount */
  uint32_t  txnoack;    /* dot11ACKFailureCount */
  uint32_t  rxfrag;     /* dot11ReceivedFragmentCount */
  uint32_t  rxmulti;    /* dot11MulticastReceivedFrameCount */
  uint32_t  rxcrc;      /* dot11FCSErrorCount */
  uint32_t  txfrmsnt;   /* dot11TransmittedFrameCount (bogus MIB?) */
  uint32_t  rxundec;    /* dot11WEPUndecryptableCount */

  /* WPA2 counters (see rxundec for DecryptFailureCount) */

  uint32_t  tkipmicfaill; /* TKIPLocalMICFailures */
  uint32_t  tkipcntrmsr;  /* TKIPCounterMeasuresInvoked */
  uint32_t  tkipreplay;   /* TKIPReplays */
  uint32_t  ccmpfmterr;   /* CCMPFormatErrors */
  uint32_t  ccmpreplay;   /* CCMPReplays */
  uint32_t  ccmpundec;    /* CCMPDecryptErrors */
  uint32_t  fourwayfail;  /* FourWayHandshakeFailures */
  uint32_t  wepundec;     /* dot11WEPUndecryptableCount */
  uint32_t  wepicverr;    /* dot11WEPICVErrorCount */
  uint32_t  decsuccess;   /* DecryptSuccessCount */
  uint32_t  tkipicverr;   /* TKIPICVErrorCount */
  uint32_t  wepexcluded;  /* dot11WEPExcludedCount */

  uint32_t  txchanrej;    /* Tx frames suppressed due to channel rejection */
  uint32_t  psmwds;       /* Count PSM watchdogs */
  uint32_t  phywatchdog;  /* Count Phy watchdogs (triggered by ucode) */

  /* MBSS counters, AP only */

  uint32_t  prq_entries_handled;         /* PRQ entries read in */
  uint32_t  prq_undirected_entries;      /*    which were bcast bss & ssid */
  uint32_t  prq_bad_entries;             /*    which could not be translated to info */
  uint32_t  atim_suppress_count;         /* TX suppressions on ATIM fifo */
  uint32_t  bcn_template_not_ready;      /* Template marked in use on send bcn ... */
  uint32_t  bcn_template_not_ready_done; /* ...but "DMA done" interrupt rcvd */
  uint32_t  late_tbtt_dpc;               /* TBTT DPC did not happen in time */

  /* per-rate receive stat counters */

  uint32_t  rx1mbps;    /* packets rx at 1Mbps */
  uint32_t  rx2mbps;    /* packets rx at 2Mbps */
  uint32_t  rx5mbps5;   /* packets rx at 5.5Mbps */
  uint32_t  rx6mbps;    /* packets rx at 6Mbps */
  uint32_t  rx9mbps;    /* packets rx at 9Mbps */
  uint32_t  rx11mbps;   /* packets rx at 11Mbps */
  uint32_t  rx12mbps;   /* packets rx at 12Mbps */
  uint32_t  rx18mbps;   /* packets rx at 18Mbps */
  uint32_t  rx24mbps;   /* packets rx at 24Mbps */
  uint32_t  rx36mbps;   /* packets rx at 36Mbps */
  uint32_t  rx48mbps;   /* packets rx at 48Mbps */
  uint32_t  rx54mbps;   /* packets rx at 54Mbps */
  uint32_t  rx108mbps;  /* packets rx at 108mbps */
  uint32_t  rx162mbps;  /* packets rx at 162mbps */
  uint32_t  rx216mbps;  /* packets rx at 216 mbps */
  uint32_t  rx270mbps;  /* packets rx at 270 mbps */
  uint32_t  rx324mbps;  /* packets rx at 324 mbps */
  uint32_t  rx378mbps;  /* packets rx at 378 mbps */
  uint32_t  rx432mbps;  /* packets rx at 432 mbps */
  uint32_t  rx486mbps;  /* packets rx at 486 mbps */
  uint32_t  rx540mbps;  /* packets rx at 540 mbps */

  /* pkteng rx frame stats */

  uint32_t  pktengrxducast; /* unicast frames rxed by the pkteng code */
  uint32_t  pktengrxdmcast; /* multicast frames rxed by the pkteng code */

  uint32_t  rfdisable;        /* count of radio disables */
  uint32_t  bphy_rxcrsglitch; /* PHY count of bphy glitches */

  uint32_t  txexptime;  /* Tx frames suppressed due to timer expiration */

  uint32_t  txmpdu_sgi;   /* count for sgi transmit */
  uint32_t  rxmpdu_sgi;   /* count for sgi received */
  uint32_t  txmpdu_stbc;  /* count for stbc transmit */
  uint32_t  rxmpdu_stbc;  /* count for stbc received */

  uint32_t  rxundec_mcst; /* dot11WEPUndecryptableCount */

  /* WPA2 counters (see rxundec for DecryptFailureCount) */

  uint32_t  tkipmicfaill_mcst;  /* TKIPLocalMICFailures */
  uint32_t  tkipcntrmsr_mcst;   /* TKIPCounterMeasuresInvoked */
  uint32_t  tkipreplay_mcst;    /* TKIPReplays */
  uint32_t  ccmpfmterr_mcst;    /* CCMPFormatErrors */
  uint32_t  ccmpreplay_mcst;    /* CCMPReplays */
  uint32_t  ccmpundec_mcst;     /* CCMPDecryptErrors */
  uint32_t  fourwayfail_mcst;   /* FourWayHandshakeFailures */
  uint32_t  wepundec_mcst;      /* dot11WEPUndecryptableCount */
  uint32_t  wepicverr_mcst;     /* dot11WEPICVErrorCount */
  uint32_t  decsuccess_mcst;    /* DecryptSuccessCount */
  uint32_t  tkipicverr_mcst;    /* TKIPICVErrorCount */
  uint32_t  wepexcluded_mcst;   /* dot11WEPExcludedCount */

  uint32_t  dma_hang;   /* count for stbc received */
} wl_cnt_ver_seven_t;

typedef struct
{
  uint16_t  version;    /* see definition of WL_CNT_T_VERSION */
  uint16_t  length;     /* length of entire structure */

  /* transmit stat counters */

  uint32_t  txframe;    /* tx data frames */
  uint32_t  txbyte;     /* tx data bytes */
  uint32_t  txretrans;  /* tx mac retransmits */
  uint32_t  txerror;    /* tx data errors (derived: sum of others) */
  uint32_t  txctl;      /* tx management frames */
  uint32_t  txprshort;  /* tx short preamble frames */
  uint32_t  txserr;     /* tx status errors */
  uint32_t  txnobuf;    /* tx out of buffers errors */
  uint32_t  txnoassoc;  /* tx discard because we're not associated */
  uint32_t  txrunt;     /* tx runt frames */
  uint32_t  txchit;     /* tx header cache hit (fastpath) */
  uint32_t  txcmiss;    /* tx header cache miss (slowpath) */

  /* transmit chip error counters */

  uint32_t  txuflo;     /* tx fifo underflows */
  uint32_t  txphyerr;   /* tx phy errors (indicated in tx status) */
  uint32_t  txphycrs;   /* PR8861/8963 counter */

  /* receive stat counters */

  uint32_t  rxframe;     /* rx data frames */
  uint32_t  rxbyte;      /* rx data bytes */
  uint32_t  rxerror;     /* rx data errors (derived: sum of others) */
  uint32_t  rxctl;       /* rx management frames */
  uint32_t  rxnobuf;     /* rx out of buffers errors */
  uint32_t  rxnondata;   /* rx non data frames in the data channel errors */
  uint32_t  rxbadds;     /* rx bad DS errors */
  uint32_t  rxbadcm;     /* rx bad control or management frames */
  uint32_t  rxfragerr;   /* rx fragmentation errors */
  uint32_t  rxrunt;      /* rx runt frames */
  uint32_t  rxgiant;     /* rx giant frames */
  uint32_t  rxnoscb;     /* rx no scb error */
  uint32_t  rxbadproto;  /* rx invalid frames */
  uint32_t  rxbadsrcmac; /* rx frames with Invalid Src Mac */
  uint32_t  rxbadda;     /* rx frames tossed for invalid da */
  uint32_t  rxfilter;    /* rx frames filtered out */

  /* receive chip error counters */

  uint32_t  rxoflo;         /* rx fifo overflow errors */
  uint32_t  rxuflo[NFIFO];  /* rx dma descriptor underflow errors */

  uint32_t  d11cnt_txrts_off;   /* d11cnt txrts value when reset d11cnt */
  uint32_t  d11cnt_rxcrc_off;   /* d11cnt rxcrc value when reset d11cnt */
  uint32_t  d11cnt_txnocts_off; /* d11cnt txnocts value when reset d11cnt */

  /* misc counters */

  uint32_t  dmade;                  /* tx/rx dma descriptor errors */
  uint32_t  dmada;                  /* tx/rx dma data errors */
  uint32_t  dmape;                  /* tx/rx dma descriptor protocol errors */
  uint32_t  reset;                  /* reset count */
  uint32_t  tbtt;                   /* cnts the TBTT int's */
  uint32_t  txdmawar;               /* # occurrences of PR15420 workaround */
  uint32_t  pkt_callback_reg_fail;  /* callbacks register failure */

  /* MAC counters: 32-bit version of d11.h's macstat_t */

  uint32_t  txallfrm;   /* total number of frames sent, incl. Data, ACK, RTS, CTS,
                         * Control Management (includes retransmissions) */
  uint32_t  txrtsfrm;   /* number of RTS sent out by the MAC */
  uint32_t  txctsfrm;   /* number of CTS sent out by the MAC */
  uint32_t  txackfrm;   /* number of ACK frames sent out */
  uint32_t  txdnlfrm;   /* Not used */
  uint32_t  txbcnfrm;   /* beacons transmitted */
  uint32_t  txfunfl[6]; /* per-fifo tx underflows */
  uint32_t  rxtoolate;  /* receive too late */
  uint32_t  txfbw;      /* transmit at fallback bw (dynamic bw) */
  uint32_t  txtplunfl;  /* Template underflows (mac was too slow to transmit ACK/CTS
                         * or BCN) */
  uint32_t  txphyerror; /* Transmit phy error, type of error is reported in tx-status for
                         * driver enqueued frames */

  uint32_t  rxfrmtoolong;   /* Received frame longer than legal limit (2346 bytes) */
  uint32_t  rxfrmtooshrt;   /* Received frame did not contain enough bytes for its frame type */
  uint32_t  rxinvmachdr;    /* Either the protocol version != 0 or frame type not
                             * data/control/management */

  uint32_t  rxbadfcs;     /* number of frames for which the CRC check failed in the MAC */
  uint32_t  rxbadplcp;    /* parity check of the PLCP header failed */
  uint32_t  rxcrsglitch;  /* PHY was able to correlate the preamble but not the header */
  uint32_t  rxstrt;       /* Number of received frames with a good PLCP
                           * (i.e. passing parity check)
                           */

  uint32_t  rxdfrmucastmbss; /* Number of received DATA frames with good FCS and matching RA */
  uint32_t  rxmfrmucastmbss; /* number of received mgmt frames with good FCS and matching RA */
  uint32_t  rxcfrmucast;     /* number of received CNTRL frames with good FCS and matching RA */
  uint32_t  rxrtsucast;      /* number of unicast RTS addressed to the MAC (good FCS) */
  uint32_t  rxctsucast;      /* number of unicast CTS addressed to the MAC (good FCS) */
  uint32_t  rxackucast;      /* number of ucast ACKS received (good FCS) */
  uint32_t  rxdfrmocast;     /* number of received DATA frames (good FCS and not matching RA) */
  uint32_t  rxmfrmocast;     /* number of received MGMT frames (good FCS and not matching RA) */
  uint32_t  rxcfrmocast;     /* number of received CNTRL frame (good FCS and not matching RA) */
  uint32_t  rxrtsocast;      /* number of received RTS not addressed to the MAC */
  uint32_t  rxctsocast;      /* number of received CTS not addressed to the MAC */
  uint32_t  rxdfrmmcast;     /* number of RX Data multicast frames received by the MAC */
  uint32_t  rxmfrmmcast;     /* number of RX Management multicast frames received by the MAC */
  uint32_t  rxcfrmmcast;     /* number of RX Control multicast frames received by the MAC
                              * (unlikely to see these) */
  uint32_t  rxbeaconmbss;    /* beacons received from member of BSS */
  uint32_t  rxdfrmucastobss; /* number of unicast frames addressed to the MAC from
                              * other BSS (WDS FRAME) */

  uint32_t  rxbeaconobss; /* beacons received from other BSS */
  uint32_t  rxrsptmout;   /* Number of response timeouts for transmitted frames
                           * expecting a response */

  uint32_t  bcntxcancl; /* transmit beacons canceled due to receipt of beacon (IBSS) */
  uint32_t  rxf0ovfl;   /* Number of receive fifo 0 overflows */
  uint32_t  rxf1ovfl;   /* Number of receive fifo 1 overflows (obsolete) */
  uint32_t  rxf2ovfl;   /* Number of receive fifo 2 overflows (obsolete) */
  uint32_t  txsfovfl;   /* Number of transmit status fifo overflows (obsolete) */
  uint32_t  pmqovfl;    /* Number of PMQ overflows */
  uint32_t  rxcgprqfrm; /* Number of received Probe requests that made it into
                         * the PRQ fifo */

  uint32_t  rxcgprsqovfl; /* Rx Probe Request Queue overflow in the AP */
  uint32_t  txcgprsfail;  /* Tx Probe Response Fail. AP sent probe response but did
                           * not get ACK */

  uint32_t  txcgprssuc;  /* Tx Probe Response Success (ACK was received) */
  uint32_t  prs_timeout; /* Number of probe requests that were dropped from the PRQ
                          * fifo because a probe response could not be sent out within
                          * the time limit defined in M_PRS_MAXTIME */

  uint32_t  rxnack;     /* obsolete */
  uint32_t  frmscons;   /* obsolete */
  uint32_t  txnack;     /* obsolete */
  uint32_t  rxback;     /* blockack rxcnt */
  uint32_t  txback;     /* blockack txcnt */

  /* 802.11 MIB counters, pp. 614 of 802.11 reaff doc. */

  uint32_t  txfrag;     /* dot11TransmittedFragmentCount */
  uint32_t  txmulti;    /* dot11MulticastTransmittedFrameCount */
  uint32_t  txfail;     /* dot11FailedCount */
  uint32_t  txretry;    /* dot11RetryCount */
  uint32_t  txretrie;   /* dot11MultipleRetryCount */
  uint32_t  rxdup;      /* dot11FrameduplicateCount */
  uint32_t  txrts;      /* dot11RTSSuccessCount */
  uint32_t  txnocts;    /* dot11RTSFailureCount */
  uint32_t  txnoack;    /* dot11ACKFailureCount */
  uint32_t  rxfrag;     /* dot11ReceivedFragmentCount */
  uint32_t  rxmulti;    /* dot11MulticastReceivedFrameCount */
  uint32_t  rxcrc;      /* dot11FCSErrorCount */
  uint32_t  txfrmsnt;   /* dot11TransmittedFrameCount (bogus MIB?) */
  uint32_t  rxundec;    /* dot11WEPUndecryptableCount */

  /* WPA2 counters (see rxundec for DecryptFailureCount) */

  uint32_t  tkipmicfaill;   /* TKIPLocalMICFailures */
  uint32_t  tkipcntrmsr;    /* TKIPCounterMeasuresInvoked */
  uint32_t  tkipreplay;     /* TKIPReplays */
  uint32_t  ccmpfmterr;     /* CCMPFormatErrors */
  uint32_t  ccmpreplay;     /* CCMPReplays */
  uint32_t  ccmpundec;      /* CCMPDecryptErrors */
  uint32_t  fourwayfail;    /* FourWayHandshakeFailures */
  uint32_t  wepundec;       /* dot11WEPUndecryptableCount */
  uint32_t  wepicverr;      /* dot11WEPICVErrorCount */
  uint32_t  decsuccess;     /* DecryptSuccessCount */
  uint32_t  tkipicverr;     /* TKIPICVErrorCount */
  uint32_t  wepexcluded;    /* dot11WEPExcludedCount */

  uint32_t  txchanrej;      /* Tx frames suppressed due to channel rejection */
  uint32_t  psmwds;         /* Count PSM watchdogs */
  uint32_t  phywatchdog;    /* Count Phy watchdogs (triggered by ucode) */

  /* MBSS counters, AP only */

  uint32_t  prq_entries_handled;         /* PRQ entries read in */
  uint32_t  prq_undirected_entries;      /*    which were bcast bss & ssid */
  uint32_t  prq_bad_entries;             /*    which could not be translated to info */
  uint32_t  atim_suppress_count;         /* TX suppressions on ATIM fifo */
  uint32_t  bcn_template_not_ready;      /* Template marked in use on send bcn ... */
  uint32_t  bcn_template_not_ready_done; /* ...but "DMA done" interrupt rcvd */
  uint32_t  late_tbtt_dpc;               /* TBTT DPC did not happen in time */

  /* per-rate receive stat counters */

  uint32_t  rx1mbps;    /* packets rx at 1Mbps */
  uint32_t  rx2mbps;    /* packets rx at 2Mbps */
  uint32_t  rx5mbps5;   /* packets rx at 5.5Mbps */
  uint32_t  rx6mbps;    /* packets rx at 6Mbps */
  uint32_t  rx9mbps;    /* packets rx at 9Mbps */
  uint32_t  rx11mbps;   /* packets rx at 11Mbps */
  uint32_t  rx12mbps;   /* packets rx at 12Mbps */
  uint32_t  rx18mbps;   /* packets rx at 18Mbps */
  uint32_t  rx24mbps;   /* packets rx at 24Mbps */
  uint32_t  rx36mbps;   /* packets rx at 36Mbps */
  uint32_t  rx48mbps;   /* packets rx at 48Mbps */
  uint32_t  rx54mbps;   /* packets rx at 54Mbps */
  uint32_t  rx108mbps;  /* packets rx at 108mbps */
  uint32_t  rx162mbps;  /* packets rx at 162mbps */
  uint32_t  rx216mbps;  /* packets rx at 216 mbps */
  uint32_t  rx270mbps;  /* packets rx at 270 mbps */
  uint32_t  rx324mbps;  /* packets rx at 324 mbps */
  uint32_t  rx378mbps;  /* packets rx at 378 mbps */
  uint32_t  rx432mbps;  /* packets rx at 432 mbps */
  uint32_t  rx486mbps;  /* packets rx at 486 mbps */
  uint32_t  rx540mbps;  /* packets rx at 540 mbps */

  /* pkteng rx frame stats */

  uint32_t  pktengrxducast; /* unicast frames rxed by the pkteng code */
  uint32_t  pktengrxdmcast; /* multicast frames rxed by the pkteng code */

  uint32_t  rfdisable;          /* count of radio disables */
  uint32_t  bphy_rxcrsglitch;   /* PHY count of bphy glitches */
  uint32_t  bphy_badplcp;

  uint32_t  txexptime;  /* Tx frames suppressed due to timer expiration */

  uint32_t  txmpdu_sgi;     /* count for sgi transmit */
  uint32_t  rxmpdu_sgi;     /* count for sgi received */
  uint32_t  txmpdu_stbc;    /* count for stbc transmit */
  uint32_t  rxmpdu_stbc;    /* count for stbc received */

  uint32_t  rxundec_mcst;   /* dot11WEPUndecryptableCount */

  /* WPA2 counters (see rxundec for DecryptFailureCount) */

  uint32_t  tkipmicfaill_mcst;  /* TKIPLocalMICFailures */
  uint32_t  tkipcntrmsr_mcst;   /* TKIPCounterMeasuresInvoked */
  uint32_t  tkipreplay_mcst;    /* TKIPReplays */
  uint32_t  ccmpfmterr_mcst;    /* CCMPFormatErrors */
  uint32_t  ccmpreplay_mcst;    /* CCMPReplays */
  uint32_t  ccmpundec_mcst;     /* CCMPDecryptErrors */
  uint32_t  fourwayfail_mcst;   /* FourWayHandshakeFailures */
  uint32_t  wepundec_mcst;      /* dot11WEPUndecryptableCount */
  uint32_t  wepicverr_mcst;     /* dot11WEPICVErrorCount */
  uint32_t  decsuccess_mcst;    /* DecryptSuccessCount */
  uint32_t  tkipicverr_mcst;    /* TKIPICVErrorCount */
  uint32_t  wepexcluded_mcst;   /* dot11WEPExcludedCount */

  uint32_t  dma_hang;   /* count for dma hang */
  uint32_t  reinit;     /* count for reinit */

  uint32_t  pstatxucast;    /* count of ucast frames xmitted on all psta assoc */
  uint32_t  pstatxnoassoc;  /* count of txnoassoc frames xmitted on all psta assoc */
  uint32_t  pstarxucast;    /* count of ucast frames received on all psta assoc */
  uint32_t  pstarxbcmc;     /* count of bcmc frames received on all psta */
  uint32_t  pstatxbcmc;     /* count of bcmc frames transmitted on all psta */

  uint32_t  cso_passthrough; /* hw cso required but passthrough */
  uint32_t  cso_normal;      /* hw cso hdr for normal process */
  uint32_t  chained;         /* number of frames chained */
  uint32_t  chainedsz1;      /* number of chain size 1 frames */
  uint32_t  unchained;       /* number of frames not chained */
  uint32_t  maxchainsz;      /* max chain size so far */
  uint32_t  currchainsz;     /* current chain size */

  uint32_t  rxdrop20s;  /* drop secondary cnt */
} wl_cnt_ver_eight_t;

typedef struct
{
  uint16_t version;
  uint16_t length;
  uint32_t rxampdu_sgi;
  uint32_t rxampdu_stbc;
  uint32_t rxmpdu_sgi;
  uint32_t rxmpdu_stbc;
  uint32_t rxmcs0_40;
  uint32_t rxmcs1_40;
  uint32_t rxmcs2_40;
  uint32_t rxmcs3_40;
  uint32_t rxmcs4_40;
  uint32_t rxmcs5_40;
  uint32_t rxmcs6_40;
  uint32_t rxmcs7_40;
  uint32_t rxmcs32_40;
  uint32_t txfrmsnt_20lo;
  uint32_t txfrmsnt_20up;
  uint32_t txfrmsnt_40;
  uint32_t rx_20ul;
} wl_cnt_ext_t;
#define    WL_RXDIV_STATS_T_VERSION    1
typedef struct
{
  uint16_t version;
  uint16_t length;
  uint32_t rxant[4];
} wl_rxdiv_stats_t;
#define    WL_DELTA_STATS_T_VERSION    1
typedef struct
{
  uint16_t version;
  uint16_t length;
  uint32_t txframe;
  uint32_t txbyte;
  uint32_t txretrans;
  uint32_t txfail;
  uint32_t rxframe;
  uint32_t rxbyte;
  uint32_t rx1mbps;
  uint32_t rx2mbps;
  uint32_t rx5mbps5;
  uint32_t rx6mbps;
  uint32_t rx9mbps;
  uint32_t rx11mbps;
  uint32_t rx12mbps;
  uint32_t rx18mbps;
  uint32_t rx24mbps;
  uint32_t rx36mbps;
  uint32_t rx48mbps;
  uint32_t rx54mbps;
  uint32_t rx108mbps;
  uint32_t rx162mbps;
  uint32_t rx216mbps;
  uint32_t rx270mbps;
  uint32_t rx324mbps;
  uint32_t rx378mbps;
  uint32_t rx432mbps;
  uint32_t rx486mbps;
  uint32_t rx540mbps;
} wl_delta_stats_t;
#define WL_WME_CNT_VERSION    1
typedef struct
{
  uint32_t packets;
  uint32_t bytes;
} wl_traffic_stats_t;
#define AC_COUNT        4
typedef struct
{
  uint16_t version;
  uint16_t length;
  wl_traffic_stats_t tx[AC_COUNT];
  wl_traffic_stats_t tx_failed[AC_COUNT];
  wl_traffic_stats_t rx[AC_COUNT];
  wl_traffic_stats_t rx_failed[AC_COUNT];
  wl_traffic_stats_t forward[AC_COUNT];
  wl_traffic_stats_t tx_expired[AC_COUNT];
} wl_wme_cnt_t;

typedef struct wl_mkeep_alive_pkt
{
  uint16_t    version;        /* Version for mkeep_alive */
  uint16_t    length;         /* length of fixed parameters */
  uint32_t    period_msec;    /* repeat interval msecs */
  uint16_t    len_bytes;      /* packet length */
  uint8_t     keep_alive_id;  /* 0 - 3 for N = 4 */
  uint8_t     data[1];        /* Packet data */
} wl_mkeep_alive_pkt_t;

#define WL_MKEEP_ALIVE_VERSION      1
#define WL_MKEEP_ALIVE_FIXED_LEN    offsetof(wl_mkeep_alive_pkt_t, data)
#define WL_MKEEP_ALIVE_PRECISION    500

struct ampdu_tid_control
{
  uint8_t tid;
  uint8_t enable;
};

struct wl_msglevel2
{
  uint32_t low;
  uint32_t high;
};

struct ampdu_ea_tid
{
  struct ether_addr ea;
  uint8_t tid;
};

struct ampdu_retry_tid
{
  uint8_t tid;
  uint8_t retry;
};

struct ampdu_ba_sizes
{
  uint8_t ba_tx_wsize;
  uint8_t ba_rx_wsize;
};

#define    DPT_DISCOVERY_MANUAL    0x01
#define    DPT_DISCOVERY_AUTO    0x02
#define    DPT_DISCOVERY_SCAN    0x04
#define DPT_PATHSEL_AUTO    0
#define DPT_PATHSEL_DIRECT    1
#define DPT_PATHSEL_APPATH    2
#define DPT_DENY_LIST_ADD     1
#define DPT_DENY_LIST_REMOVE     2
#define DPT_MANUAL_EP_CREATE    1
#define DPT_MANUAL_EP_MODIFY    2
#define DPT_MANUAL_EP_DELETE    3
typedef struct dpt_iovar
{
  struct ether_addr ea;
  uint8_t mode;
  uint32_t pad;
} dpt_iovar_t;
#define    DPT_STATUS_ACTIVE    0x01
#define    DPT_STATUS_AES        0x02
#define    DPT_STATUS_FAILED    0x04
#define    DPT_FNAME_LEN        48
typedef struct dpt_status
{
  uint8_t status;
  uint8_t fnlen;
  uint8_t name[DPT_FNAME_LEN];
  uint32_t rssi;
  sta_info_t sta;
} dpt_status_t;
typedef struct dpt_list
{
  uint32_t num;
  dpt_status_t status[1];
} dpt_list_t;
typedef struct dpt_fname
{
  uint8_t len;
  uint8_t name[DPT_FNAME_LEN];
} dpt_fname_t;
#define    BDD_FNAME_LEN        32
typedef struct bdd_fname
{
  uint8_t len;
  uint8_t name[BDD_FNAME_LEN];
} bdd_fname_t;
struct ts_list
{
  int32_t count;
  struct tsinfo_arg tsinfo[1];
};
typedef struct tspec_arg
{
  uint16_t version;
  uint16_t length;
  uint32_t flag;
  struct tsinfo_arg tsinfo;
  uint16_t nom_msdu_size;
  uint16_t max_msdu_size;
  uint32_t min_srv_interval;
  uint32_t max_srv_interval;
  uint32_t inactivity_interval;
  uint32_t suspension_interval;
  uint32_t srv_start_time;
  uint32_t min_data_rate;
  uint32_t mean_data_rate;
  uint32_t peak_data_rate;
  uint32_t max_burst_size;
  uint32_t delay_bound;
  uint32_t min_phy_rate;
  uint16_t surplus_bw;
  uint16_t medium_time;
  uint8_t dialog_token;
} tspec_arg_t;
typedef struct tspec_per_sta_arg
{
  struct ether_addr ea;
  struct tspec_arg ts;
} tspec_per_sta_arg_t;
typedef struct wme_max_bandwidth
{
  uint32_t ac[AC_COUNT];
} wme_max_bandwidth_t;
#define WL_WME_MBW_PARAMS_IO_BYTES (sizeof(wme_max_bandwidth_t))
#define    TSPEC_ARG_VERSION        2
#define TSPEC_ARG_LENGTH        55
#define TSPEC_DEFAULT_DIALOG_TOKEN    42
#define TSPEC_DEFAULT_SBW_FACTOR    0x3000
#define TSPEC_PENDING        0
#define TSPEC_ACCEPTED        1
#define TSPEC_REJECTED        2
#define TSPEC_UNKNOWN        3
#define TSPEC_STATUS_MASK    7
#define WL_SWFL_NOHWRADIO    0x0004
#define WL_LIFETIME_MAX 0xFFFF
typedef struct wl_lifetime
{
  uint32_t ac;
  uint32_t lifetime;
} wl_lifetime_t;
typedef struct wl_chan_switch
{
  uint8_t mode;
  uint8_t count;
  wl_chanspec_t chspec;
  uint8_t reg;
} wl_chan_switch_t;

#define WLC_ROAM_TRIGGER_DEFAULT    0
#define WLC_ROAM_TRIGGER_BANDWIDTH    1
#define WLC_ROAM_TRIGGER_DISTANCE    2
#define WLC_ROAM_TRIGGER_MAX_VALUE    2

enum
{
  PFN_LIST_ORDER, PFN_RSSI
};

#define SORT_CRITERIA_BIT        0
#define AUTO_NET_SWITCH_BIT        1
#define ENABLE_BKGRD_SCAN_BIT    2
#define IMMEDIATE_SCAN_BIT        3
#define    AUTO_CONNECT_BIT        4
#define SORT_CRITERIA_MASK        0x01
#define AUTO_NET_SWITCH_MASK    0x02
#define ENABLE_BKGRD_SCAN_MASK    0x04
#define IMMEDIATE_SCAN_MASK        0x08
#define    AUTO_CONNECT_MASK        0x10
#define PFN_VERSION            1

typedef struct wl_pfn_param
{
  int32_t version;
  int32_t scan_freq;
  int32_t lost_network_timeout;
  int16_t flags;
  int16_t rssi_margin;
} wl_pfn_param_t;

typedef struct wl_pfn
{
  wlc_ssid_t ssid;
  int32_t bss_type;
  int32_t infra;
  int32_t auth;
  uint32_t wpa_auth;
  int32_t wsec;
} wl_pfn_t;

#define TOE_TX_CSUM_OL        0x00000001
#define TOE_RX_CSUM_OL        0x00000002
#define TOE_ERRTEST_TX_CSUM    0x00000001
#define TOE_ERRTEST_RX_CSUM    0x00000002
#define TOE_ERRTEST_RX_CSUM2    0x00000004

struct toe_ol_stats_t
{
  uint32_t tx_summed;
  uint32_t tx_iph_fill;
  uint32_t tx_tcp_fill;
  uint32_t tx_udp_fill;
  uint32_t tx_icmp_fill;
  uint32_t rx_iph_good;
  uint32_t rx_iph_bad;
  uint32_t rx_tcp_good;
  uint32_t rx_tcp_bad;
  uint32_t rx_udp_good;
  uint32_t rx_udp_bad;
  uint32_t rx_icmp_good;
  uint32_t rx_icmp_bad;
  uint32_t tx_tcp_errinj;
  uint32_t tx_udp_errinj;
  uint32_t tx_icmp_errinj;
  uint32_t rx_tcp_errinj;
  uint32_t rx_udp_errinj;
  uint32_t rx_icmp_errinj;
};

#define ARP_OL_AGENT        0x00000001
#define ARP_OL_SNOOP        0x00000002
#define ARP_OL_HOST_AUTO_REPLY    0x00000004
#define ARP_OL_PEER_AUTO_REPLY    0x00000008
#define ARP_ERRTEST_REPLY_PEER    0x1
#define ARP_ERRTEST_REPLY_HOST    0x2
#define ARP_MULTIHOMING_MAX    8

struct arp_ol_stats_t
{
  uint32_t host_ip_entries;
  uint32_t host_ip_overflow;
  uint32_t arp_table_entries;
  uint32_t arp_table_overflow;
  uint32_t host_request;
  uint32_t host_reply;
  uint32_t host_service;
  uint32_t peer_request;
  uint32_t peer_request_drop;
  uint32_t peer_reply;
  uint32_t peer_reply_drop;
  uint32_t peer_service;
};

typedef struct wl_keep_alive_pkt
{
  uint32_t period_msec;
  uint16_t len_bytes;
  uint8_t data[1];
} wl_keep_alive_pkt_t;

#define WL_KEEP_ALIVE_FIXED_LEN        offsetof(wl_keep_alive_pkt_t, data)

typedef enum wl_pkt_filter_type
{
  WL_PKT_FILTER_TYPE_PATTERN_MATCH
} wl_pkt_filter_type_t;

#define WL_PKT_FILTER_TYPE wl_pkt_filter_type_t

typedef struct wl_pkt_filter_pattern
{
  uint32_t offset;
  uint32_t size_bytes;
  uint8_t mask_and_pattern[1];
} wl_pkt_filter_pattern_t;

typedef struct wl_pkt_filter
{
  uint32_t id;
  uint32_t type;
  uint32_t negate_match;
  union
  {
      wl_pkt_filter_pattern_t pattern;
  } u;
} wl_pkt_filter_t;

#define WL_PKT_FILTER_FIXED_LEN              offsetof(wl_pkt_filter_t, u)
#define WL_PKT_FILTER_PATTERN_FIXED_LEN      offsetof(wl_pkt_filter_pattern_t, mask_and_pattern)

typedef struct wl_pkt_filter_enable
{
  uint32_t id;
  uint32_t enable;
} wl_pkt_filter_enable_t;

typedef struct wl_pkt_filter_list
{
  uint32_t num;
  wl_pkt_filter_t filter[1];
} wl_pkt_filter_list_t;

#define WL_PKT_FILTER_LIST_FIXED_LEN      offsetof(wl_pkt_filter_list_t, filter)

typedef struct wl_pkt_filter_stats
{
  uint32_t num_pkts_matched;
  uint32_t num_pkts_forwarded;
  uint32_t num_pkts_discarded;
} wl_pkt_filter_stats_t;

typedef struct wl_seq_cmd_ioctl
{
  uint32_t cmd;
  uint32_t len;
} wl_seq_cmd_ioctl_t;

#define WL_SEQ_CMD_ALIGN_BYTES    4
#define WL_SEQ_CMDS_GET_IOCTL_FILTER(cmd) \
  (((cmd) == WLC_GET_MAGIC)        || \
   ((cmd) == WLC_GET_VERSION)        || \
   ((cmd) == WLC_GET_AP)            || \
   ((cmd) == WLC_GET_INSTANCE))
#define WL_PKTENG_PER_TX_START            0x01
#define WL_PKTENG_PER_TX_STOP            0x02
#define WL_PKTENG_PER_RX_START            0x04
#define WL_PKTENG_PER_RX_WITH_ACK_START     0x05
#define WL_PKTENG_PER_TX_WITH_ACK_START     0x06
#define WL_PKTENG_PER_RX_STOP            0x08
#define WL_PKTENG_PER_MASK            0xff
#define WL_PKTENG_SYNCHRONOUS            0x100

typedef struct wl_pkteng
{
  uint32_t flags;
  uint32_t delay;
  uint32_t nframes;
  uint32_t length;
  uint8_t seqno;
  struct ether_addr dest;
  struct ether_addr src;
} wl_pkteng_t;

#define NUM_80211b_RATES    4
#define NUM_80211ag_RATES    8
#define NUM_80211n_RATES    32
#define NUM_80211_RATES        (NUM_80211b_RATES+NUM_80211ag_RATES+NUM_80211n_RATES)

typedef struct wl_pkteng_stats
{
  uint32_t lostfrmcnt;
  int32_t rssi;
  int32_t snr;
  uint16_t rxpktcnt[NUM_80211_RATES + 1];
} wl_pkteng_stats_t;

#define WL_WOWL_MAGIC    (1 << 0)
#define WL_WOWL_NET    (1 << 1)
#define WL_WOWL_DIS    (1 << 2)
#define WL_WOWL_RETR    (1 << 3)
#define WL_WOWL_BCN    (1 << 4)
#define WL_WOWL_TST    (1 << 5)
#define WL_WOWL_BCAST    (1 << 15)
#define MAGIC_PKT_MINLEN 102

typedef struct
{
  uint32_t masksize;
  uint32_t offset;
  uint32_t patternoffset;
  uint32_t patternsize;
} wl_wowl_pattern_t;

typedef struct
{
  uint32_t count;
  wl_wowl_pattern_t pattern[1];
} wl_wowl_pattern_list_t;

typedef struct
{
  uint8_t pci_wakeind;
  uint16_t ucode_wakeind;
} wl_wowl_wakeind_t;

typedef struct wl_txrate_class
{
  uint8_t init_rate;
  uint8_t min_rate;
  uint8_t max_rate;
} wl_txrate_class_t;

#define WLC_OBSS_SCAN_PASSIVE_DWELL_DEFAULT             100
#define WLC_OBSS_SCAN_PASSIVE_DWELL_MIN                 5
#define WLC_OBSS_SCAN_PASSIVE_DWELL_MAX                 1000
#define WLC_OBSS_SCAN_ACTIVE_DWELL_DEFAULT              20
#define WLC_OBSS_SCAN_ACTIVE_DWELL_MIN                  10
#define WLC_OBSS_SCAN_ACTIVE_DWELL_MAX                  1000
#define WLC_OBSS_SCAN_WIDTHSCAN_INTERVAL_DEFAULT        300
#define WLC_OBSS_SCAN_WIDTHSCAN_INTERVAL_MIN            10
#define WLC_OBSS_SCAN_WIDTHSCAN_INTERVAL_MAX            900
#define WLC_OBSS_SCAN_CHANWIDTH_TRANSITION_DLY_DEFAULT  5
#define WLC_OBSS_SCAN_CHANWIDTH_TRANSITION_DLY_MIN      5
#define WLC_OBSS_SCAN_CHANWIDTH_TRANSITION_DLY_MAX      100
#define WLC_OBSS_SCAN_PASSIVE_TOTAL_PER_CHANNEL_DEFAULT 200
#define WLC_OBSS_SCAN_PASSIVE_TOTAL_PER_CHANNEL_MIN     200
#define WLC_OBSS_SCAN_PASSIVE_TOTAL_PER_CHANNEL_MAX     10000
#define WLC_OBSS_SCAN_ACTIVE_TOTAL_PER_CHANNEL_DEFAULT  20
#define WLC_OBSS_SCAN_ACTIVE_TOTAL_PER_CHANNEL_MIN      20
#define WLC_OBSS_SCAN_ACTIVE_TOTAL_PER_CHANNEL_MAX      10000
#define WLC_OBSS_SCAN_ACTIVITY_THRESHOLD_DEFAULT        25
#define WLC_OBSS_SCAN_ACTIVITY_THRESHOLD_MIN            0
#define WLC_OBSS_SCAN_ACTIVITY_THRESHOLD_MAX            100

typedef struct wl_obss_scan_arg
{
  int16_t passive_dwell;
  int16_t active_dwell;
  int16_t bss_widthscan_interval;
  int16_t passive_total;
  int16_t active_total;
  int16_t chanwidth_transition_delay;
  int16_t activity_threshold;
} wl_obss_scan_arg_t;

#define WL_OBSS_SCAN_PARAM_LEN   sizeof(wl_obss_scan_arg_t)
#define WL_MIN_NUM_OBSS_SCAN_ARG 7
#define WL_COEX_INFO_MASK        0x07
#define WL_COEX_INFO_REQ         0x01
#define WL_COEX_40MHZ_INTOLERANT 0x02
#define WL_COEX_WIDTH20          0x04

typedef struct wl_action_obss_coex_req
{
  uint8_t info;
  uint8_t num;
  uint8_t ch_list[1];
} wl_action_obss_coex_req_t;

#define MAX_RSSI_LEVELS 8

typedef struct wl_rssi_event
{
  uint32_t rate_limit_msec;
  uint8_t num_rssi_levels;
  int8_t rssi_levels[MAX_RSSI_LEVELS];
} wl_rssi_event_t;

typedef struct wl_sta_rssi
{
  uint32_t          rssi;
  struct ether_addr sta_addr;
  uint16_t          foo;
} wl_sta_rssi_t;

#define WLFEATURE_DISABLE_11N          0x00000001
#define WLFEATURE_DISABLE_11N_STBC_TX  0x00000002
#define WLFEATURE_DISABLE_11N_STBC_RX  0x00000004
#define WLFEATURE_DISABLE_11N_SGI_TX   0x00000008
#define WLFEATURE_DISABLE_11N_SGI_RX   0x00000010
#define WLFEATURE_DISABLE_11N_AMPDU_TX 0x00000020
#define WLFEATURE_DISABLE_11N_AMPDU_RX 0x00000040
#define WLFEATURE_DISABLE_11N_GF       0x00000080

#pragma pack(1)

typedef struct sta_prbreq_wps_ie_hdr
{
  struct ether_addr sta_addr;
  uint16_t ie_len;
} sta_prbreq_wps_ie_hdr_t;

typedef struct sta_prbreq_wps_ie_data
{
  sta_prbreq_wps_ie_hdr_t hdr;
  uint8_t ie_data[1];
} sta_prbreq_wps_ie_data_t;

typedef  struct sta_prbreq_wps_ie_list
{
  uint32_t tot_len;
  uint8_t ie_data_list[1];
} sta_prbreq_wps_ie_list_t;

/* EDCF related items from 802.11.h */

/* ACI from 802.11.h */

#define EDCF_AIFSN_MIN               1           /* AIFSN minimum value */
#define EDCF_AIFSN_MAX               15          /* AIFSN maximum value */
#define EDCF_AIFSN_MASK              0x0f        /* AIFSN mask */
#define EDCF_ACM_MASK                0x10        /* ACM mask */
#define EDCF_ACI_MASK                0x60        /* ACI mask */
#define EDCF_ACI_SHIFT               5           /* ACI shift */
#define EDCF_AIFSN_SHIFT             12          /* 4 MSB(0xFFF) in ifs_ctl for AC idx */

/* ECW from 802.11.h */

#define EDCF_ECW_MIN                 0           /* cwmin/cwmax exponent minimum value */
#define EDCF_ECW_MAX                 15          /* cwmin/cwmax exponent maximum value */
#define EDCF_ECW2CW(exp)             ((1 << (exp)) - 1)
#define EDCF_ECWMIN_MASK             0x0f        /* cwmin exponent form mask */
#define EDCF_ECWMAX_MASK             0xf0        /* cwmax exponent form mask */
#define EDCF_ECWMAX_SHIFT            4           /* cwmax exponent form shift */

/* TXOP from 802.11.h */

#define EDCF_TXOP_MIN                0           /* TXOP minimum value */
#define EDCF_TXOP_MAX                65535       /* TXOP maximum value */
#define EDCF_TXOP2USEC(txop)         ((txop) << 5)

struct edcf_acparam
{
  uint8_t   ACI;
  uint8_t   ECW;
  uint16_t  TXOP;       /* stored in network order (ls octet first) */
};

typedef struct edcf_acparam edcf_acparam_t;

/* Stop packing structures */

#pragma pack()

/* Enumerated list of event types */

typedef enum
{
  WLC_E_NONE                           =  -1,
  WLC_E_SET_SSID                       =   0  /* indicates status of set SSID */,
  WLC_E_JOIN                           =   1, /* differentiates join IBSS from found (WLC_E_START) IBSS */
  WLC_E_START                          =   2, /* STA founded an IBSS or AP started a BSS */
  WLC_E_AUTH                           =   3, /* 802.11 AUTH request */
  WLC_E_AUTH_IND                       =   4, /* 802.11 AUTH indication */
  WLC_E_DEAUTH                         =   5, /* 802.11 DEAUTH request */
  WLC_E_DEAUTH_IND                     =   6, /* 802.11 DEAUTH indication */
  WLC_E_ASSOC                          =   7, /* 802.11 ASSOC request */
  WLC_E_ASSOC_IND                      =   8, /* 802.11 ASSOC indication */
  WLC_E_REASSOC                        =   9, /* 802.11 REASSOC request */
  WLC_E_REASSOC_IND                    =  10, /* 802.11 REASSOC indication */
  WLC_E_DISASSOC                       =  11, /* 802.11 DISASSOC request */
  WLC_E_DISASSOC_IND                   =  12, /* 802.11 DISASSOC indication */
  WLC_E_QUIET_START                    =  13, /* 802.11h Quiet period started */
  WLC_E_QUIET_END                      =  14, /* 802.11h Quiet period ended */
  WLC_E_BEACON_RX                      =  15, /* BEACONS received/lost indication */
  WLC_E_LINK                           =  16, /* generic link indication */
  WLC_E_MIC_ERROR                      =  17, /* TKIP MIC error occurred */
  WLC_E_NDIS_LINK                      =  18, /* NDIS style link indication */
  WLC_E_ROAM                           =  19, /* roam attempt occurred: indicate status & reason */
  WLC_E_TXFAIL                         =  20, /* change in dot11FailedCount (txfail) */
  WLC_E_PMKID_CACHE                    =  21, /* WPA2 pmkid cache indication */
  WLC_E_RETROGRADE_TSF                 =  22, /* current AP's TSF value went backward */
  WLC_E_PRUNE                          =  23, /* AP was pruned from join list for reason */
  WLC_E_AUTOAUTH                       =  24, /* report AutoAuth table entry match for join attempt */
  WLC_E_EAPOL_MSG                      =  25, /* Event encapsulating an EAPOL message */
  WLC_E_SCAN_COMPLETE                  =  26, /* Scan results are ready or scan was aborted */
  WLC_E_ADDTS_IND                      =  27, /* indicate to host addts fail/success */
  WLC_E_DELTS_IND                      =  28, /* indicate to host delts fail/success */
  WLC_E_BCNSENT_IND                    =  29, /* indicate to host of beacon transmit */
  WLC_E_BCNRX_MSG                      =  30, /* Send the received beacon up to the host */
  WLC_E_BCNLOST_MSG                    =  31, /* indicate to host loss of beacon */
  WLC_E_ROAM_PREP                      =  32, /* before attempting to roam */
  WLC_E_PFN_NET_FOUND                  =  33, /* PFN network found event */
  WLC_E_PFN_NET_LOST                   =  34, /* PFN network lost event */
  WLC_E_RESET_COMPLETE                 =  35,
  WLC_E_JOIN_START                     =  36,
  WLC_E_ROAM_START                     =  37,
  WLC_E_ASSOC_START                    =  38,
  WLC_E_IBSS_ASSOC                     =  39,
  WLC_E_RADIO                          =  40,
  WLC_E_PSM_WATCHDOG                   =  41, /* PSM microcode watchdog fired */
  WLC_E_CCX_ASSOC_START                =  42, /* CCX association start */
  WLC_E_CCX_ASSOC_ABORT                =  43, /* CCX association abort */
  WLC_E_PROBREQ_MSG                    =  44, /* probe request received */
  WLC_E_SCAN_CONFIRM_IND               =  45,
  WLC_E_PSK_SUP                        =  46, /* WPA Handshake */
  WLC_E_COUNTRY_CODE_CHANGED           =  47,
  WLC_E_EXCEEDED_MEDIUM_TIME           =  48, /* WMMAC exceeded medium time */
  WLC_E_ICV_ERROR                      =  49, /* WEP ICV error occurred */
  WLC_E_UNICAST_DECODE_ERROR           =  50, /* Unsupported unicast encrypted frame */
  WLC_E_MULTICAST_DECODE_ERROR         =  51, /* Unsupported multicast encrypted frame */
  WLC_E_TRACE                          =  52,
  WLC_E_BTA_HCI_EVENT                  =  53, /* BT-AMP HCI event */
  WLC_E_IF                             =  54, /* I/F change (for wlan host notification) */
  WLC_E_P2P_DISC_LISTEN_COMPLETE       =  55, /* P2P Discovery listen state expires */
  WLC_E_RSSI                           =  56, /* indicate RSSI change based on configured levels */
  WLC_E_PFN_SCAN_COMPLETE              =  57, /* PFN completed scan of network list */
  WLC_E_EXTLOG_MSG                     =  58,
  WLC_E_ACTION_FRAME                   =  59, /* Action frame reception */
  WLC_E_ACTION_FRAME_COMPLETE          =  60, /* Action frame Tx complete */
  WLC_E_PRE_ASSOC_IND                  =  61, /* assoc request received */
  WLC_E_PRE_REASSOC_IND                =  62, /* re-assoc request received */
  WLC_E_CHANNEL_ADOPTED                =  63, /* channel adopted (xxx: obsoleted) */
  WLC_E_AP_STARTED                     =  64, /* AP started */
  WLC_E_DFS_AP_STOP                    =  65, /* AP stopped due to DFS */
  WLC_E_DFS_AP_RESUME                  =  66, /* AP resumed due to DFS */
  WLC_E_WAI_STA_EVENT                  =  67, /* WAI stations event */
  WLC_E_WAI_MSG                        =  68, /* event encapsulating an WAI message */
  WLC_E_ESCAN_RESULT                   =  69, /* escan result event */
  WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE =  70, /* action frame off channel complete. NOTE - This used to be WLC_E_WAKE_EVENT */
  WLC_E_PROBRESP_MSG                   =  71, /* probe response received */
  WLC_E_P2P_PROBREQ_MSG                =  72, /* P2P Probe request received */
  WLC_E_DCS_REQUEST                    =  73,
  WLC_E_FIFO_CREDIT_MAP                =  74, /* credits for D11 FIFOs. [AC0,AC1,AC2,AC3,BC_MC,ATIM] */
  WLC_E_ACTION_FRAME_RX                =  75, /* Received action frame event WITH wl_event_rx_frame_data_t header */
  WLC_E_WAKE_EVENT                     =  76, /* Wake Event timer fired, used for wake WLAN test mode */
  WLC_E_RM_COMPLETE                    =  77, /* Radio measurement complete */
  WLC_E_HTSFSYNC                       =  78, /* Synchronize TSF with the host */
  WLC_E_OVERLAY_REQ                    =  79, /* request an overlay IOCTL/iovar from the host */
  WLC_E_CSA_COMPLETE_IND               =  80,
  WLC_E_EXCESS_PM_WAKE_EVENT           =  81, /* excess PM Wake Event to inform host  */
  WLC_E_PFN_SCAN_NONE                  =  82, /* no PFN networks around */
  WLC_E_PFN_SCAN_ALLGONE               =  83, /* last found PFN network gets lost */
  WLC_E_GTK_PLUMBED                    =  84,
  WLC_E_ASSOC_IND_NDIS                 =  85, /* 802.11 ASSOC indication for NDIS only */
  WLC_E_REASSOC_IND_NDIS               =  86, /* 802.11 REASSOC indication for NDIS only */
  WLC_E_ASSOC_REQ_IE                   =  87,
  WLC_E_ASSOC_RESP_IE                  =  88,
  WLC_E_ASSOC_RECREATED                =  89, /* association recreated on resume */
  WLC_E_ACTION_FRAME_RX_NDIS           =  90, /* rx action frame event for NDIS only */
  WLC_E_AUTH_REQ                       =  91, /* authentication request received */
  WLC_E_TDLS_PEER_EVENT                =  92, /* discovered peer, connected/disconnected peer */
  WLC_E_SPEEDY_RECREATE_FAIL           =  93, /* fast assoc recreation failed */
  WLC_E_NATIVE                         =  94, /* port-specific event and payload (e.g. NDIS) */
  WLC_E_PKTDELAY_IND                   =  95, /* event for tx pkt delay suddently jump */
  WLC_E_AWDL_AW                        =  96, /* AWDL AW period starts */
  WLC_E_AWDL_ROLE                      =  97, /* AWDL Master/Slave/NE master role event */
  WLC_E_AWDL_EVENT                     =  98, /* Generic AWDL event */
  WLC_E_NIC_AF_TXS                     =  99, /* NIC AF txstatus */
  WLC_E_NIC_NIC_REPORT                 = 100, /* NIC period report */
  WLC_E_BEACON_FRAME_RX                = 101,
  WLC_E_SERVICE_FOUND                  = 102, /* desired service found */
  WLC_E_GAS_FRAGMENT_RX                = 103, /* GAS fragment received */
  WLC_E_GAS_COMPLETE                   = 104, /* GAS sessions all complete */
  WLC_E_P2PO_ADD_DEVICE                = 105, /* New device found by p2p offload */
  WLC_E_P2PO_DEL_DEVICE                = 106, /* device has been removed by p2p offload */
  WLC_E_WNM_STA_SLEEP                  = 107, /* WNM event to notify STA enter sleep mode */
  WLC_E_TXFAIL_THRESH                  = 108, /* Indication of MAC tx failures (exhaustion of 802.11 retries) exceeding threshold(s) */
  WLC_E_PROXD                          = 109, /* Proximity Detection event */
  WLC_E_IBSS_COALESCE                  = 110, /* IBSS Coalescing */
  WLC_E_AWDL_RX_PRB_RESP               = 111, /* AWDL RX Probe response */
  WLC_E_AWDL_RX_ACT_FRAME              = 112, /* AWDL RX Action Frames */
  WLC_E_AWDL_WOWL_NULLPKT              = 113, /* AWDL Wowl nulls */
  WLC_E_AWDL_PHYCAL_STATUS             = 114, /* AWDL Phycal status */
  WLC_E_AWDL_OOB_AF_STATUS             = 115, /* AWDL OOB AF status */
  WLC_E_AWDL_SCAN_STATUS               = 116, /* Interleaved Scan status */
  WLC_E_AWDL_AW_START                  = 117, /* AWDL AW Start */
  WLC_E_AWDL_AW_END                    = 118, /* AWDL AW End */
  WLC_E_AWDL_AW_EXT                    = 119, /* AWDL AW Extensions */
  WLC_E_AWDL_PEER_CACHE_CONTROL        = 120,
  WLC_E_CSA_START_IND                  = 121,
  WLC_E_CSA_DONE_IND                   = 122,
  WLC_E_CSA_FAILURE_IND                = 123,
  WLC_E_CCA_CHAN_QUAL                  = 124, /* CCA based channel quality report */
  WLC_E_BSSID                          = 125, /* to report change in BSSID while roaming */
  WLC_E_TX_STAT_ERROR                  = 126, /* tx error indication */
  WLC_E_BCMC_CREDIT_SUPPORT            = 127, /* credit check for BCMC supported */
  WLC_E_PSTA_PRIMARY_INTF_IND          = 128, /* psta primary interface indication */
  WLC_E_LAST                           = 129, /* highest val + 1 for range checking */

  WLC_E_FORCE_32_BIT                   = 0x7ffffffe  /* Force enum to be stored in 32 bit variable */
} wl_event_num_t;

#define BCMF_EVENT_COUNT WLC_E_LAST

#define WLC_SUP_STATUS_OFFSET      (256)
#define WLC_DOT11_SC_STATUS_OFFSET (512)

/* Enumerated list of event status codes
 * @note : WLC_SUP values overlap other values, so it is necessary
 *         to check the event type
 */

typedef enum
{
  WLC_E_STATUS_SUCCESS                  =  0,  /* operation was successful */
  WLC_E_STATUS_FAIL                     =  1,  /* operation failed */
  WLC_E_STATUS_TIMEOUT                  =  2,  /* operation timed out */
  WLC_E_STATUS_NO_NETWORKS              =  3,  /* failed due to no matching network found */
  WLC_E_STATUS_ABORT                    =  4,  /* operation was aborted */
  WLC_E_STATUS_NO_ACK                   =  5,  /* protocol failure: packet not ack'd */
  WLC_E_STATUS_UNSOLICITED              =  6,  /* AUTH or ASSOC packet was unsolicited */
  WLC_E_STATUS_ATTEMPT                  =  7,  /* attempt to assoc to an auto auth configuration */
  WLC_E_STATUS_PARTIAL                  =  8,  /* scan results are incomplete */
  WLC_E_STATUS_NEWSCAN                  =  9,  /* scan aborted by another scan */
  WLC_E_STATUS_NEWASSOC                 = 10,  /* scan aborted due to assoc in progress */
  WLC_E_STATUS_11HQUIET                 = 11,  /* 802.11h quiet period started */
  WLC_E_STATUS_SUPPRESS                 = 12,  /* user disabled scanning (WLC_SET_SCANSUPPRESS) */
  WLC_E_STATUS_NOCHANS                  = 13,  /* no allowable channels to scan */
  WLC_E_STATUS_CCXFASTRM                = 14,  /* scan aborted due to CCX fast roam */
  WLC_E_STATUS_CS_ABORT                 = 15,  /* abort channel select */

  /* for WLC_SUP messages */

  WLC_SUP_DISCONNECTED                  = 0 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_CONNECTING                    = 1 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_IDREQUIRED                    = 2 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_AUTHENTICATING                = 3 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_AUTHENTICATED                 = 4 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_KEYXCHANGE                    = 5 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_KEYED                         = 6 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_TIMEOUT                       = 7 + WLC_SUP_STATUS_OFFSET,
  WLC_SUP_LAST_BASIC_STATE              = 8 + WLC_SUP_STATUS_OFFSET,

  /* Extended supplicant authentication states */

  WLC_SUP_KEYXCHANGE_WAIT_M1            = (int) WLC_SUP_AUTHENTICATED    + WLC_SUP_STATUS_OFFSET, /* Waiting   to receive handshake msg M1 */
  WLC_SUP_KEYXCHANGE_PREP_M2            = (int) WLC_SUP_KEYXCHANGE       + WLC_SUP_STATUS_OFFSET, /* Preparing to send    handshake msg M2 */
  WLC_SUP_KEYXCHANGE_WAIT_M3            = (int) WLC_SUP_LAST_BASIC_STATE + WLC_SUP_STATUS_OFFSET, /* Waiting   to receive handshake msg M3 */
  WLC_SUP_KEYXCHANGE_PREP_M4            = 9                              + WLC_SUP_STATUS_OFFSET, /* Preparing to send    handshake msg M4 */
  WLC_SUP_KEYXCHANGE_WAIT_G1            = 10                             + WLC_SUP_STATUS_OFFSET, /* Waiting   to receive handshake msg G1 */
  WLC_SUP_KEYXCHANGE_PREP_G2            = 11                             + WLC_SUP_STATUS_OFFSET, /* Preparing to send    handshake msg G2 */

  WLC_DOT11_SC_SUCCESS                  =  0 + WLC_DOT11_SC_STATUS_OFFSET,  /* Successful */
  WLC_DOT11_SC_FAILURE                  =  1 + WLC_DOT11_SC_STATUS_OFFSET,  /* Unspecified failure */
  WLC_DOT11_SC_CAP_MISMATCH             = 10 + WLC_DOT11_SC_STATUS_OFFSET,  /* Cannot support all requested capabilities in the Capability Information field */
  WLC_DOT11_SC_REASSOC_FAIL             = 11 + WLC_DOT11_SC_STATUS_OFFSET,  /* Reassociation denied due to inability to confirm that association exists */
  WLC_DOT11_SC_ASSOC_FAIL               = 12 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to reason outside the scope of this standard */
  WLC_DOT11_SC_AUTH_MISMATCH            = 13 + WLC_DOT11_SC_STATUS_OFFSET,  /* Responding station does not support the specified authentication algorithm */
  WLC_DOT11_SC_AUTH_SEQ                 = 14 + WLC_DOT11_SC_STATUS_OFFSET,  /* Received an Authentication frame with authentication transaction sequence number out of expected sequence */
  WLC_DOT11_SC_AUTH_CHALLENGE_FAIL      = 15 + WLC_DOT11_SC_STATUS_OFFSET,  /* Authentication rejected because of challenge failure */
  WLC_DOT11_SC_AUTH_TIMEOUT             = 16 + WLC_DOT11_SC_STATUS_OFFSET,  /* Authentication rejected due to timeout waiting for next frame in sequence */
  WLC_DOT11_SC_ASSOC_BUSY_FAIL          = 17 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied because AP is unable to handle additional associated stations */
  WLC_DOT11_SC_ASSOC_RATE_MISMATCH      = 18 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to requesting station not supporting all of the data rates in the BSSBasicRateSet parameter */
  WLC_DOT11_SC_ASSOC_SHORT_REQUIRED     = 19 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to requesting station not supporting the Short Preamble option */
  WLC_DOT11_SC_ASSOC_PBCC_REQUIRED      = 20 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to requesting  station not supporting the PBCC Modulation option */
  WLC_DOT11_SC_ASSOC_AGILITY_REQUIRED   = 21 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to requesting station not supporting the Channel Agility option */
  WLC_DOT11_SC_ASSOC_SPECTRUM_REQUIRED  = 22 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied because Spectrum Management capability is required. */
  WLC_DOT11_SC_ASSOC_BAD_POWER_CAP      = 23 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied because the info in the Power Cap element is unacceptable. */
  WLC_DOT11_SC_ASSOC_BAD_SUP_CHANNELS   = 24 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied because the info in the Supported Channel element is unacceptable */
  WLC_DOT11_SC_ASSOC_SHORTSLOT_REQUIRED = 25 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to requesting station not supporting the Short Slot Time option */
  WLC_DOT11_SC_ASSOC_ERPBCC_REQUIRED    = 26 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to requesting station not supporting the ER-PBCC Modulation option */
  WLC_DOT11_SC_ASSOC_DSSOFDM_REQUIRED   = 27 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to requesting station not supporting the DSS-OFDM option */
  WLC_DOT11_SC_DECLINED                 = 37 + WLC_DOT11_SC_STATUS_OFFSET,  /* request declined */
  WLC_DOT11_SC_INVALID_PARAMS           = 38 + WLC_DOT11_SC_STATUS_OFFSET,  /* One or more params have invalid values */
  WLC_DOT11_SC_INVALID_AKMP             = 43 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to invalid AKMP */
  WLC_DOT11_SC_INVALID_MDID             = 54 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to invalid MDID */
  WLC_DOT11_SC_INVALID_FTIE             = 55 + WLC_DOT11_SC_STATUS_OFFSET,  /* Association denied due to invalid FTIE */

  WLC_E_STATUS_FORCE_32_BIT   = 0x7ffffffe                     /* Force enum to be stored in 32 bit variable */
} wl_event_status_t;

#define WLC_E_PRUNE_REASON_OFFSET    (256)
#define WLC_E_SUP_REASON_OFFSET      (512)
#define WLC_E_DOT11_RC_REASON_OFFSET (768)

/* Enumerated list of event reason codes
 * @note : Several values overlap other values, so it is necessary
 *         to check the event type
 */

typedef enum
{
  /* roam reason codes */

  WLC_E_REASON_INITIAL_ASSOC    = 0,   /* initial assoc */
  WLC_E_REASON_LOW_RSSI         = 1,   /* roamed due to low RSSI */
  WLC_E_REASON_DEAUTH           = 2,   /* roamed due to DEAUTH indication */
  WLC_E_REASON_DISASSOC         = 3,   /* roamed due to DISASSOC indication */
  WLC_E_REASON_BCNS_LOST        = 4,   /* roamed due to lost beacons */
  WLC_E_REASON_FAST_ROAM_FAILED = 5,   /* roamed due to fast roam failure */
  WLC_E_REASON_DIRECTED_ROAM    = 6,   /* roamed due to request by AP */
  WLC_E_REASON_TSPEC_REJECTED   = 7,   /* roamed due to TSPEC rejection */
  WLC_E_REASON_BETTER_AP        = 8,   /* roamed due to finding better AP */

  /* prune reason codes */

  WLC_E_PRUNE_ENCR_MISMATCH     = 1  + WLC_E_PRUNE_REASON_OFFSET,  /* encryption mismatch */
  WLC_E_PRUNE_BCAST_BSSID       = 2  + WLC_E_PRUNE_REASON_OFFSET,  /* AP uses a broadcast BSSID */
  WLC_E_PRUNE_MAC_DENY          = 3  + WLC_E_PRUNE_REASON_OFFSET,  /* STA's MAC addr is in AP's MAC deny list */
  WLC_E_PRUNE_MAC_NA            = 4  + WLC_E_PRUNE_REASON_OFFSET,  /* STA's MAC addr is not in AP's MAC allow list */
  WLC_E_PRUNE_REG_PASSV         = 5  + WLC_E_PRUNE_REASON_OFFSET,  /* AP not allowed due to regulatory restriction */
  WLC_E_PRUNE_SPCT_MGMT         = 6  + WLC_E_PRUNE_REASON_OFFSET,  /* AP does not support STA locale spectrum mgmt */
  WLC_E_PRUNE_RADAR             = 7  + WLC_E_PRUNE_REASON_OFFSET,  /* AP is on a radar channel of STA locale */
  WLC_E_RSN_MISMATCH            = 8  + WLC_E_PRUNE_REASON_OFFSET,  /* STA does not support AP's RSN */
  WLC_E_PRUNE_NO_COMMON_RATES   = 9  + WLC_E_PRUNE_REASON_OFFSET,  /* No rates in common with AP */
  WLC_E_PRUNE_BASIC_RATES       = 10 + WLC_E_PRUNE_REASON_OFFSET,  /* STA does not support all basic rates of BSS */
  WLC_E_PRUNE_CCXFAST_PREVAP    = 11 + WLC_E_PRUNE_REASON_OFFSET,  /* CCX FAST ROAM: prune previous AP */
  WLC_E_PRUNE_CIPHER_NA         = 12 + WLC_E_PRUNE_REASON_OFFSET,  /* BSS's cipher not supported */
  WLC_E_PRUNE_KNOWN_STA         = 13 + WLC_E_PRUNE_REASON_OFFSET,  /* AP is already known to us as a STA */
  WLC_E_PRUNE_CCXFAST_DROAM     = 14 + WLC_E_PRUNE_REASON_OFFSET,  /* CCX FAST ROAM: prune unqualified AP */
  WLC_E_PRUNE_WDS_PEER          = 15 + WLC_E_PRUNE_REASON_OFFSET,  /* AP is already known to us as a WDS peer */
  WLC_E_PRUNE_QBSS_LOAD         = 16 + WLC_E_PRUNE_REASON_OFFSET,  /* QBSS LOAD - AAC is too low */
  WLC_E_PRUNE_HOME_AP           = 17 + WLC_E_PRUNE_REASON_OFFSET,  /* prune home AP */
  WLC_E_PRUNE_AP_BLOCKED        = 18 + WLC_E_PRUNE_REASON_OFFSET,  /* prune blocked AP */
  WLC_E_PRUNE_NO_DIAG_SUPPORT   = 19 + WLC_E_PRUNE_REASON_OFFSET,  /* prune due to diagnostic mode not supported */

  /* WPA failure reason codes carried in the WLC_E_PSK_SUP event */

  WLC_E_SUP_OTHER               = 0  + WLC_E_SUP_REASON_OFFSET,  /* Other reason */
  WLC_E_SUP_DECRYPT_KEY_DATA    = 1  + WLC_E_SUP_REASON_OFFSET,  /* Decryption of key data failed */
  WLC_E_SUP_BAD_UCAST_WEP128    = 2  + WLC_E_SUP_REASON_OFFSET,  /* Illegal use of ucast WEP128 */
  WLC_E_SUP_BAD_UCAST_WEP40     = 3  + WLC_E_SUP_REASON_OFFSET,  /* Illegal use of ucast WEP40 */
  WLC_E_SUP_UNSUP_KEY_LEN       = 4  + WLC_E_SUP_REASON_OFFSET,  /* Unsupported key length */
  WLC_E_SUP_PW_KEY_CIPHER       = 5  + WLC_E_SUP_REASON_OFFSET,  /* Unicast cipher mismatch in pairwise key */
  WLC_E_SUP_MSG3_TOO_MANY_IE    = 6  + WLC_E_SUP_REASON_OFFSET,  /* WPA IE contains > 1 RSN IE in key msg 3 */
  WLC_E_SUP_MSG3_IE_MISMATCH    = 7  + WLC_E_SUP_REASON_OFFSET,  /* WPA IE mismatch in key message 3 */
  WLC_E_SUP_NO_INSTALL_FLAG     = 8  + WLC_E_SUP_REASON_OFFSET,  /* INSTALL flag unset in 4-way msg */
  WLC_E_SUP_MSG3_NO_GTK         = 9  + WLC_E_SUP_REASON_OFFSET,  /* encapsulated GTK missing from msg 3 */
  WLC_E_SUP_GRP_KEY_CIPHER      = 10 + WLC_E_SUP_REASON_OFFSET,  /* Multicast cipher mismatch in group key */
  WLC_E_SUP_GRP_MSG1_NO_GTK     = 11 + WLC_E_SUP_REASON_OFFSET,  /* encapsulated GTK missing from group msg 1 */
  WLC_E_SUP_GTK_DECRYPT_FAIL    = 12 + WLC_E_SUP_REASON_OFFSET,  /* GTK decrypt failure */
  WLC_E_SUP_SEND_FAIL           = 13 + WLC_E_SUP_REASON_OFFSET,  /* message send failure */
  WLC_E_SUP_DEAUTH              = 14 + WLC_E_SUP_REASON_OFFSET,  /* received FC_DEAUTH */
  WLC_E_SUP_WPA_PSK_TMO         = 15 + WLC_E_SUP_REASON_OFFSET,  /* WPA PSK 4-way handshake timeout */

  DOT11_RC_RESERVED             =  0 + WLC_E_DOT11_RC_REASON_OFFSET, /* d11 RC reserved */
  DOT11_RC_UNSPECIFIED          =  1 + WLC_E_DOT11_RC_REASON_OFFSET, /* Unspecified reason */
  DOT11_RC_AUTH_INVAL           =  2 + WLC_E_DOT11_RC_REASON_OFFSET, /* Previous authentication no longer valid */
  DOT11_RC_DEAUTH_LEAVING       =  3 + WLC_E_DOT11_RC_REASON_OFFSET, /* Deauthenticated because sending station is leaving (or has left) IBSS or ESS */
  DOT11_RC_INACTIVITY           =  4 + WLC_E_DOT11_RC_REASON_OFFSET, /* Disassociated due to inactivity */
  DOT11_RC_BUSY                 =  5 + WLC_E_DOT11_RC_REASON_OFFSET, /* Disassociated because AP is unable to handle all currently associated stations */
  DOT11_RC_INVAL_CLASS_2        =  6 + WLC_E_DOT11_RC_REASON_OFFSET, /* Class 2 frame received from nonauthenticated station */
  DOT11_RC_INVAL_CLASS_3        =  7 + WLC_E_DOT11_RC_REASON_OFFSET, /* Class 3 frame received from nonassociated station */
  DOT11_RC_DISASSOC_LEAVING     =  8 + WLC_E_DOT11_RC_REASON_OFFSET, /* Disassociated because sending station is leaving (or has left) BSS */
  DOT11_RC_NOT_AUTH             =  9 + WLC_E_DOT11_RC_REASON_OFFSET, /* Station requesting (re)association is not * authenticated with responding station */
  DOT11_RC_BAD_PC               = 10 + WLC_E_DOT11_RC_REASON_OFFSET, /* Unacceptable power capability element */
  DOT11_RC_BAD_CHANNELS         = 11 + WLC_E_DOT11_RC_REASON_OFFSET, /* Unacceptable supported channels element */
                                                                     /* 12 is unused */
                                                                     /* XXX 13-23 are WPA/802.11i reason codes defined in proto/wpa.h */
                                                                     /* 32-39 are QSTA specific reasons added in 11e */
  DOT11_RC_UNSPECIFIED_QOS      = 32 + WLC_E_DOT11_RC_REASON_OFFSET, /* unspecified QoS-related reason */
  DOT11_RC_INSUFFCIENT_BW       = 33 + WLC_E_DOT11_RC_REASON_OFFSET, /* QAP lacks sufficient bandwidth */
  DOT11_RC_EXCESSIVE_FRAMES     = 34 + WLC_E_DOT11_RC_REASON_OFFSET, /* excessive number of frames need ack */
  DOT11_RC_TX_OUTSIDE_TXOP      = 35 + WLC_E_DOT11_RC_REASON_OFFSET, /* transmitting outside the limits of txop */
  DOT11_RC_LEAVING_QBSS         = 36 + WLC_E_DOT11_RC_REASON_OFFSET, /* QSTA is leaving the QBSS (or restting) */
  DOT11_RC_BAD_MECHANISM        = 37 + WLC_E_DOT11_RC_REASON_OFFSET, /* does not want to use the mechanism */
  DOT11_RC_SETUP_NEEDED         = 38 + WLC_E_DOT11_RC_REASON_OFFSET, /* mechanism needs a setup */
  DOT11_RC_TIMEOUT              = 39 + WLC_E_DOT11_RC_REASON_OFFSET, /* timeout */
  DOT11_RC_MAX                  = 23 + WLC_E_DOT11_RC_REASON_OFFSET, /* Reason codes > 23 are reserved */

  WLC_E_REASON_FORCE_32_BIT     = 0x7ffffffe                     /* Force enum to be stored in 32 bit variable */
} wl_event_reason_t;

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_IOCTL_H */
