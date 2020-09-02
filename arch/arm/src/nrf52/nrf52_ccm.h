/****************************************************************************
 * /home/v01d/coding/nuttx_nrf_ble/nuttx/arch/arm/src/nrf52/nrf52_ccm.h
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

#ifndef __NRF52_CCM_H
#define __NRF52_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RAM layout of CCM Nonce */

begin_packed_struct struct nrf52_ccm_nonce_s
{
  uint8_t aes_key[16];        /* AES Key */
  uint64_t pkt_ctr;           /* Packet counter (39 bits) */
  uint8_t dir           : 1;  /* Direction bit  */
  uint8_t dummy_bits    : 7;  /* Ignored bits */
  uint32_t ivm;               /* Master's IV (Initialization Vector) */
  uint32_t ivs;               /* Slave's IV (Initialization Vector) */
} end_packed_struct;

/* RAM layout of (unencrypted) packet. Encrypted packet will be the same
 * but with an extra four octets of the MIC at the end.
 * We cannot have a struct for this case since the MIC in-memory
 * placement depends on actual payload length.
 */

struct nrf52_ccm_pkt_s
{
  uint8_t hdr;
  uint8_t len;
  uint8_t rfu;
  uint8_t payload[0];
};

/* Datarate options */

enum nrf52_ccm_datarate_e
{
  CCM_DATARATE_1M,        /* 1 Mbit */
  CCM_DATARATE_2M,        /* 2 Mbit */
};

/* Packet length options */

enum nrf52_ccm_pktlen_e
{
  CCM_PKTLEN_DEFAULT,     /* 5 bit packet length */
  CCM_PKTLEN_EXTENDED,    /* 8 bit packet length */
};

/* Mode */

enum nrf52_ccm_mode_e
{
  CCM_MODE_ENCRYPT,
  CCM_MODE_DECRYPT,
};

/* Events */

enum nrf52_ccm_int_e
{
  CCM_INT_ENDKSGEN,
  CCM_INT_ENDCRYPT,
  CCM_INT_ERROR
};

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

const struct nrf52_ccm_nonce_s *nrf52_ccm_get_nonce(void);
void nrf52_ccm_set_nonce(const struct nrf52_ccm_nonce_s *nonce);

void nrf52_ccm_set_inptr(struct nrf52_ccm_pkt_s *pkt);
void nrf52_ccm_set_outptr(struct nrf52_ccm_pkt_s *pkt);

struct nrf52_ccm_pkt_s *nrf52_ccm_get_inptr(void);
struct nrf52_ccm_pkt_s *nrf52_ccm_get_outptr(void);

void nrf52_ccm_configure(enum nrf52_ccm_datarate_e rate,
                         enum nrf52_ccm_pktlen_e len);

void nrf52_ccm_setmode(enum nrf52_ccm_mode_e);
void nrf52_ccm_shortcut(bool enable);

void nrf52_ccm_enable(bool enable);

bool nrf52_ccm_passed(void);

void nrf52_ccm_ackint(enum nrf52_ccm_int_e interrupt);
bool nrf52_ccm_checkint(enum nrf52_ccm_int_e interrupt);
void nrf52_ccm_enableint(enum nrf52_ccm_int_e interrupt, bool enable);
void nrf52_ccm_setisr(xcpt_t handler, void *arg);

void nrf52_ccm_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif // __NRF52_CCM_H
