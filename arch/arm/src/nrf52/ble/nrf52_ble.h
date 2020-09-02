/****************************************************************************
 * /home/v01d/coding/nuttx_nrf_ble/nuttx/arch/arm/src/nrf52/nrf52_ble.h
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

#ifndef __NRF52_BLE_H
#define __NRF52_BLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_ll.h>
#include "arm_arch.h"
#include "chip.h"
#include "nrf52_ble.h"
#include "nrf52_radio.h"
#include "nrf52_ppi.h"
#include "nrf52_rtc.h"
#include "nrf52_tim.h"
#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"
#include "hardware/nrf52_gpiote.h"
#include "hardware/nrf52_radio.h"
#include "hardware/nrf52_rtc.h"
#include "hardware/nrf52_ficr.h"
#include "hardware/nrf52_uicr.h"
#include "hardware/nrf52_rng.h"
#include "hardware/nrf52_ppi.h"
#include "hardware/nrf52_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_DEBUG_NET) || defined(CONFIG_DEBUG_WIRELESS)
#warning "Logging severly affects correct interrupt handling of link-layer and will most likely break"
#endif

/* CRC polynomial for BLE: x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1
 * NOTE: 24-th bit is assumed to be on, since it only accepts bits [0,23].
 */

#define BLE_CRC_POLYNOMIAL  ((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | \
                             (1 << 6) | (1 << 9) | (1 << 10))

#define BLE_CRC_INIT_VALUE  (0x555555)

/* TODO: make user configurable */

#define BLE_RTC_INSTANCE          1
#define BLE_RTC2_INSTANCE         2

#if BLE_RTC_INSTANCE == 0
#  define BLE_RTC_BASE            NRF52_RTC0_BASE
#elif BLE_RTC_INSTANCE == 1
#  define BLE_RTC_BASE            NRF52_RTC1_BASE
#elif BLE_RTC_INSTANCE == 2
#  define BLE_RTC_BASE            NRF52_RTC2_BASE
#endif

#if BLE_RTC2_INSTANCE == 0
#  define BLE_RTC2_BASE            NRF52_RTC0_BASE
#elif BLE_RTC2_INSTANCE == 1
#  define BLE_RTC2_BASE            NRF52_RTC1_BASE
#elif BLE_RTC2_INSTANCE == 2
#  define BLE_RTC2_BASE            NRF52_RTC2_BASE
#endif

#define BLE_TIM_INSTANCE          0

#if BLE_TIM_INSTANCE == 0
#  define BLE_TIM_BASE            NRF52_TIMER0_BASE
#elif BLE_TIM_INSTANCE == 1
#  define BLE_TIM_BASE            NRF52_TIMER1_BASE
#elif BLE_TIM_INSTANCE == 2
#  define BLE_TIM_BASE            NRF52_TIMER2_BASE
#elif BLE_TIM_INSTANCE == 3
#  define BLE_TIM_BASE            NRF52_TIMER3_BASE
#elif BLE_TIM_INSTANCE == 4
#  define BLE_TIM_BASE            NRF52_TIMER4_BASE
#endif

/* Set RTC prescaler to zero for maximum resolution of ~30uS and
 * maximum period of 512s. The maximum timeout required by BLE spec is
 * 32s.
 */

#define BLE_RTC_PRESCALER         0
#define BLE_RTC_FREQUENCY         (32768 / (BLE_RTC_PRESCALER + 1))

#define BLE_RTC_MAX_PERIOD_US     (512 * USEC_PER_SEC)
#define BLE_RTC_MAX_VALUE_MULT    (UINT32_MAX / BLE_RTC_FREQUENCY)

#define BLE_RTC_MS_TO_COUNT(t)    (((t) * BLE_RTC_FREQUENCY) / MSEC_PER_SEC)
#define BLE_RTC_US_TO_COUNT(t)    (((t) * BLE_RTC_FREQUENCY) / USEC_PER_SEC)

/* We support only four connections, which is what the hardware can natively
 * handle without adding much more additional logic (mainly due to how many
 * independent access addresses it supports, excluding the one used for
 * advertising)
 */

#define BLE_MAX_CONNECTIONS       4

/* Sleep clock accuracy in uS / ppm */

#if 0
#define BLE_SCA                   50    /* from ext. 32K crystal */
#else
#define BLE_SCA                   250   /* testing */
#endif

/* Use GPIOs to reflect events */

#define BLE_PINDEBUG              1

#if BLE_PINDEBUG
#define BLE_PINDEBUG_ADDRESS_END     6  /* ADDRESS: H, END: L */
#define BLE_PINDEBUG_READY_DISABLED  8  /* READY: H, DISABLED: L */
#define BLE_PINDEBUG_RX_WINDOW      13  /* Inside window: H, outside: L */
#define BLE_PINDEBUG_AUX            14  /* Auxiliary */
#define BLE_PINDEBUG_AUX2           15  /* Auxiliary */
#define BLE_PINDEBUG_AUX3           12  /* Auxiliary */
#endif

/* RADIO Timings (in uS) */

/* TODO: measure, this are overestimated for testing */

#if 1
#define BLE_RADIO_TIME_ADDRESS_TO_CRC    300
#define BLE_RADIO_TIME_RXEN_TO_RXIDLE    300
#else
#define BLE_RADIO_TIME_ADDRESS_TO_CRC    120
#define BLE_RADIO_TIME_RXEN_TO_RXIDLE    160
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Represents a BLE connection */

struct nrf52_ble_conn_s
{
  bool used;                /* Is this connection slot used? */
  uint16_t event_counter;
  uint8_t last_unmapped_ch;
  uint8_t channel_map[5];   /* Same format as in CONNECT_REQ PDU */
  uint8_t num_used_channels;

  bool update_map;          /* Whether a channel map update is pending */
  struct bt_control_pdu_channel_map_req_s next_channel_map;

  bool update_conn_params; /* Wether connection params are to be updated */
  struct bt_control_pdu_conn_update_req_s next_conn_params;

  uint8_t hop_increment : 5;
  uint8_t master_sca : 3;
  uint32_t slave_latency;       /* us */
  uint32_t conn_interval;       /* us */
  uint32_t winoffset;           /* us */
  uint32_t winsize;             /* us */
  uint32_t supervision_timeout; /* us */
  uint16_t clk_acc;

  bool more_data;           /* Whether either more is to be transmitted */
  bool resend_data;         /* Whether we should resend */
  bool will_send_nonempty;  /* If outgoing message will be non-empty PDU */

  bool req_rem_features;    /* If the host is expecting remote features */
  bool got_rem_features;    /* If remote features were received */
  uint8_t remote_features[8];

  bool encryption;          /* Whether encryption is enabled */
  uint32_t iv;              /* Initialization Vector */
  uint64_t skd;             /* Slave's Session key identifier */
  uint64_t skdm;            /* Master's Session key identifier */

  uint8_t nesn : 1;         /* Next expected sequence number */
  uint8_t txsn : 1;         /* Transmit sequence number */
};

/* Describes the BLE device */

struct nrf52_ble_dev_s
{
  struct nrf52_rtc_dev_s *rtc;
  struct nrf52_rtc_dev_s *rtc2;
  struct nrf52_tim_dev_s *tim;

  uint8_t channel;          /* Current (logical) channel */
  uint8_t state;            /* Link-layer state */
  uint8_t role;             /* Link-layer role when connected */

  struct nrf52_ble_conn_s connections[BLE_MAX_CONNECTIONS];

  struct bt_hci_cp_le_set_scan_params_s scan_parameters;
  struct bt_hci_cp_le_set_adv_parameters_s adv_parameters;
  struct bt_hci_cp_le_set_adv_data_s adv_data;
  struct bt_hci_cp_le_set_scan_rsp_data_s rsp_data;

  struct bt_addr_s random_address;

  /* I/O packet buffer */

  uint8_t buffer[BLUETOOTH_LE_PDU_MAXSIZE + 1];

  /* Second intermediary buffer to encrypt/decrypt packets */

  uint8_t encrypted_buffer[BLUETOOTH_LE_PDU_MAXSIZE + 1];

  /* I/O buffer for data PDUs. We have two buffers we will use
   * in ping-pong mode: the inactive buffer will be used to store
   * data supplied by the host layer, the active one will be used by
   * RADIO to send. After sending, we switch buffers. This way we avoid
   * copies and take advantage of EasyDMA implementation for RADIO peripheral
   * where we just switch the pointer to the I/O buffer.
   */

  uint8_t data_buffer[2][BLUETOOTH_LE_PDU_MAXSIZE + 1];
  uint8_t active_data_buffer;
  sem_t data_buffer_sem;       /* Used to mediate access to the outgoing data buffer */

  /* Separate buffer to send ctrl data */

  uint8_t ctrl_buffer[BLUETOOTH_LE_PDU_MAXSIZE + 1];
};

enum nrf52_ble_sec_mode_e
{
  BLE_SEC_ENCRYPT,
  BLE_SEC_DECRYPT,
  BLE_SEC_DISABLE
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct nrf52_ble_dev_s g_ble_dev;
extern struct nrf52_radio_dev_s *g_radio_dev;
extern const struct bt_data_pdu_hdr_s g_empty_pdu;
extern const struct nrf52_radio_pktcfg_s g_ble_adv_pkt_config;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void nrf52_ble_set_rtc_timeout(uint8_t rtc, uint8_t ch, uint32_t us);

int nrf52_ble_state(void);
int nrf52_ble_setup_scan(struct bt_hci_cp_le_set_scan_params_s *params);

int nrf52_ble_scan(bool enable);

int nrf52_ble_set_advdata(struct bt_hci_cp_le_set_adv_data_s *data);
int nrf52_ble_set_rspdata(struct bt_hci_cp_le_set_scan_rsp_data_s *data);

int nrf52_ble_setup_advertise(struct bt_hci_cp_le_set_adv_parameters_s
                              *params);
int nrf52_ble_set_random_addr(struct bt_addr_s *addr);
int nrf52_ble_advertise(bool enable);

int nrf52_ble_txpower(int8_t dbm);
int nrf52_ble_setchannel(uint8_t channel);
int nrf52_ble_getaddress(struct bt_addr_s *addr);

int nrf52_ble_reset(void);
int nrf52_ble_configure(void);

int nrf52_ble_trigger_scan(bool start);
int nrf52_ble_trigger_advertisement(bool start);

int nrf52_ble_isr(int irq, FAR void *context, FAR void *arg);
int nrf52_ble_rtc_isr(int irq, FAR void *context, FAR void *arg);
int nrf52_ble_rtc2_isr(int irq, FAR void *context, FAR void *arg);

void nrf52_ble_next_data_channel(int conn_slot);

void nrf52_handle_adv_packet(void);
void nrf52_handle_data_packet(bool crc_pass);

int nrf52_ble_start_connection(struct bt_adv_pdu_hdr_s *hdr,
                                      struct bt_connect_req_pdu_payload_s
                                      *payload, uint8_t role);

void nrf52_ble_send_data(const uint8_t *data, size_t len, bool continuation,
                         bool broadcast);

void nrf52_ble_send_ctrl_pdu(struct bt_control_pdu_payload_s *payload,
                             size_t len);

void nrf52_ble_abort_connection(int conn);

void nrf52_ble_close_connection_event(void);

int nrf52_ble_get_remote_features(uint8_t conn);

void nrf52_ble_rand(uint8_t *data, size_t len);

void nrf52_ble_sec_initialize(void);

void nrf52_ble_set_ltk(uint8_t conn_id, uint8_t *ltk);

void nrf52_ble_sec_mode(enum nrf52_ble_sec_mode_e mode);

void nrf52_ble_sec_set_iv(uint32_t ivs, uint32_t ivm);

void nrf52_ble_sec_gen_key(uint64_t skds, uint64_t skdm, uint8_t *ltk);

void nrf52_ble_sec_set_packetptr(uint8_t *in_ptr, uint8_t *out_ptr);

void nrf52_ble_sec_inctx(void);
void nrf52_ble_sec_incrx(void);
void nrf52_ble_sec_reset_pktctr(void);

#endif // __NRF52_BLE_H
