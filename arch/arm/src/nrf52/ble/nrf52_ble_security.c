/****************************************************************************
 * arch/arm/src/nrf52/ble/nrf52_ble_security.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf52_ble.h"
#include "nrf52_ccm.h"
#include "nrf52_ppi.h"
#include "nrf52_ecb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nrf52_ble_ccm_isr(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct nrf52_ccm_nonce_s g_nonce_rx =
{
  .pkt_ctr = 0,
  .dummy_bits = 0,
  .dir = 1,
};

/* NOTE: initial packet counter for TX is -1 since we increase this before
 * next packet is sent and there's an edge case right before the first packet
 * is sent. This is similar to the TXSN handling for a connection.
 */

struct nrf52_ccm_nonce_s g_nonce_tx =
{
  .pkt_ctr = -1,
  .dummy_bits = 0,
  .dir = 0,
};

enum nrf52_ble_sec_mode_e g_mode = BLE_SEC_DISABLE;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nrf52_ble_ccm_isr(int irq, FAR void *context, FAR void *arg)
{
  static uint8_t txsn = -1;

  if (nrf52_ccm_checkint(CCM_INT_ENDCRYPT))
    {
      /* Ack event */

      nrf52_ccm_ackint(CCM_INT_ENDCRYPT);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nrf52_ble_sec_initialize(void)
{
  /* Init CCM */

  nrf52_ccm_initialize();

  /* Configure for BLE */

  nrf52_ccm_configure(CCM_DATARATE_1M, CCM_PKTLEN_DEFAULT);

  /* Set ISR */

  nrf52_ccm_setisr(nrf52_ble_ccm_isr, NULL);
}

void nrf52_ble_sec_mode(enum nrf52_ble_sec_mode_e mode)
{
  if (mode == g_mode)
    {
      return;
    }

  if (mode == BLE_SEC_DISABLE)
    {
      /* Disable pre-programmed PPI channels, which decouples RADIO from
       * CCM.
       */

      if (g_mode == BLE_SEC_DECRYPT)
        {
          nrf52_ppi_channel_enable(PPI_CH_RADIO_READY_CCM_KSGEN, false);
          nrf52_ppi_channel_enable(PPI_CH_RADIO_ADDRESS_CCM_CRYPT, false);
        }

      nrf52_ccm_enableint(CCM_INT_ENDCRYPT, false);

      nrf52_ble_sec_reset_pktctr();
    }
  else if (mode == BLE_SEC_ENCRYPT)
    {
      nrf52_ccm_ackint(CCM_INT_ENDCRYPT);
      nrf52_ccm_ackint(CCM_INT_ENDKSGEN);

      /* Set nonce pointer */

      nrf52_ccm_set_nonce(&g_nonce_tx);

      /* Enable ENDKSGEN -> CRYPT shortcut */

      nrf52_ccm_shortcut(true);

      /* Disable both PPI shortcuts, KSGEN will be handled via own PPI */

      nrf52_ppi_channel_enable(PPI_CH_RADIO_READY_CCM_KSGEN, false);
      nrf52_ppi_channel_enable(PPI_CH_RADIO_ADDRESS_CCM_CRYPT, false);

      /* Set CCM mode */

      nrf52_ccm_setmode(CCM_MODE_ENCRYPT);

      /* Enable ISR to count encrypted packets */

      nrf52_ccm_ackint(CCM_INT_ENDCRYPT);
      nrf52_ccm_enableint(CCM_INT_ENDCRYPT, true);
    }
  else
    {
      /* Set nonce pointer */

      nrf52_ccm_set_nonce(&g_nonce_rx);

      /* Disable ENDKSGEN -> CRYPT shortcut */

      nrf52_ccm_shortcut(false);

      /* Enable both READY -> KSGEN and ADDREESS -> CRYPT shortcuts */

      nrf52_ppi_channel_enable(PPI_CH_RADIO_READY_CCM_KSGEN, true);
      nrf52_ppi_channel_enable(PPI_CH_RADIO_ADDRESS_CCM_CRYPT, true);

      /* Set CCM mode */

      nrf52_ccm_setmode(CCM_MODE_DECRYPT);

      nrf52_ccm_ackint(CCM_INT_ENDCRYPT);
      nrf52_ccm_enableint(CCM_INT_ENDCRYPT, false);
    }

  g_mode = mode;
}

void memcpy_rev(uint8_t *dest, uint8_t *src, size_t len)
{
  while (len--)
    {
      *(dest + len) = *src++;
    }
}

void nrf52_ble_sec_set_iv(uint32_t ivs, uint32_t ivm)
{
  memcpy((uint8_t *)&g_nonce_rx.ivm, (uint8_t *)&ivm, 4);
  memcpy((uint8_t *)&g_nonce_tx.ivm, (uint8_t *)&ivm, 4);

  memcpy((uint8_t *)&g_nonce_rx.ivs, (uint8_t *)&ivs, 4);
  memcpy((uint8_t *)&g_nonce_tx.ivs, (uint8_t *)&ivs, 4);
}

void nrf52_ble_sec_set_packetptr(uint8_t *in_ptr, uint8_t *out_ptr)
{
  nrf52_ccm_set_inptr((struct nrf52_ccm_pkt_s *)in_ptr);
  nrf52_ccm_set_outptr((struct nrf52_ccm_pkt_s *)out_ptr);
}

void nrf52_ble_sec_gen_key(uint64_t skds, uint64_t skdm, uint8_t *ltk)
{
  struct nrf52_ecb_data_s ecb_data;

  /* Build SKD from SKDm and SKDs as input to AES encryption algorithm */

  memcpy_rev(&ecb_data.input[0], (uint8_t *)&skds, 8);
  memcpy_rev(&ecb_data.input[8], (uint8_t *)&skdm, 8);

  /* Use the LTK as encryption key */

  memcpy_rev(ecb_data.key, ltk, sizeof(ecb_data.key));

  /* Generate AES key */

  nrf52_ecb_encrypt(&ecb_data);

  /* Save generated AES key into nonce */

  memcpy(g_nonce_rx.aes_key, ecb_data.output, sizeof(g_nonce_rx.aes_key));
  memcpy(g_nonce_tx.aes_key, ecb_data.output, sizeof(g_nonce_tx.aes_key));
}

void nrf52_ble_sec_inctx(void)
{
  g_nonce_tx.pkt_ctr++;
}

void nrf52_ble_sec_incrx(void)
{
  g_nonce_rx.pkt_ctr++;
}

void nrf52_ble_sec_reset_pktctr(void)
{
  g_nonce_tx.pkt_ctr = -1;
  g_nonce_rx.pkt_ctr = 0;
}
