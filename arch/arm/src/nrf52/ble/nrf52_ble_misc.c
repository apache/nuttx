/****************************************************************************
 * arch/arm/src/chip/ble/nrf52_ble_misc.c
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

#include "nrf52_ble.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Radio packet CRC configuration for BLE: field of three bytes, the address
 * field is not included in computation.
 */

const struct nrf52_radio_crc_s g_ble_crc_config =
{
  .len  = 3,                              /* Three byte CRC */
  .skip = NRF52_RADIO_CRC_SKIPADDR_SKIP,  /* Do not include address in CRC */
  .poly = BLE_CRC_POLYNOMIAL,             /* Polinomial terms */
  .init = BLE_CRC_INIT_VALUE              /* Initial value */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ble_set_rtc_timeout
 *
 * Description:
 *   Set the given timeout in microseconds to the given compare channel of
 * the first or second RTC used by this driver.
 *
 ****************************************************************************/

void nrf52_ble_set_rtc_timeout(uint8_t rtc, uint8_t ch, uint32_t us)
{
  DEBUGASSERT(rtc == 1 || rtc == 2);
  DEBUGASSERT(ch < 3);

  /* This ensure we do not wrap around during conversion, this is around
   * 123s, which we would never need (maximum timeout is 32s from supervision
   * timeout)
   */

  DEBUGASSERT((us / USEC_PER_SEC) < BLE_RTC_MAX_PERIOD_US);

  uint32_t cc;

  if (us < USEC_PER_MSEC)
    {
      /* we can safely convert directly */

      cc = BLE_RTC_US_TO_COUNT(us);
    }
  else
    {
      /* we avoid wraparound by converting us and ms separately */

      cc = BLE_RTC_MS_TO_COUNT(us / USEC_PER_MSEC) +
           BLE_RTC_US_TO_COUNT(us % USEC_PER_MSEC);
    }

  /* set the CC value to the RTC */

  NRF52_RTC_SETCC(rtc == 1 ? g_ble_dev.rtc : g_ble_dev.rtc2, ch, cc);
}

/****************************************************************************
 * Name: nrf52_ble_set_random_addr
 *
 * Description:
 *   This function is called by the upper-half to set the random device
 *   address to be used.
 *
 ****************************************************************************/

int nrf52_ble_set_random_addr(struct bt_addr_s *addr)
{
  memcpy(&g_ble_dev.random_address, addr, sizeof(*addr));

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_set_rf_channel
 *
 * Description:
 *   Choose a RF channel. Channels are separated 2MHz and start at 2402MHz
 *
 ****************************************************************************/

void nrf52_ble_set_rf_channel(uint8_t channel)
{
  DEBUGASSERT(channel < 40);

  g_radio_dev->ops->freq_set(g_radio_dev, 2402 + 2 * channel);

  g_ble_dev.channel = channel;
}

/****************************************************************************
 * Name: nrf52_ble_setchannel
 *
 * Description:
 *   Choose a BLE logical channel. These map to RF channels in the following
 *   pattern:
 *
 *    Logical:  0..10 11..36 37 38 39
 *    Physical: 1..11 13..38  0 12 39
 *
 *   Logical channels 37-39 are for advertisements, the rest are for data
 *
 ****************************************************************************/

int nrf52_ble_setchannel(uint8_t channel)
{
  int ret = OK;

  DEBUGASSERT(channel < 40);

  if (channel <= 10)
    {
      nrf52_ble_set_rf_channel(channel + 1);
    }
  else if (channel <= 36)
    {
      nrf52_ble_set_rf_channel(channel + 2);
    }
  else if (channel == 37)
    {
      nrf52_ble_set_rf_channel(0);
    }
  else if (channel == 38)
    {
      nrf52_ble_set_rf_channel(12);
    }
  else if (channel == 39)
    {
      nrf52_ble_set_rf_channel(39);
    }
  else
    {
      wlerr("Wrong BLE channel selected: %i\n", channel);
      ret = -EINVAL;
    }

  /* the data whitening initial value is derived from logical channel ID */

  g_radio_dev->ops->white_set(g_radio_dev, channel);

  /* remember selected channel */

  g_ble_dev.channel = channel;

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_getaddress
 *
 * Description:
 *   Returns device's public address, if it has one.
 *
 ****************************************************************************/

int nrf52_ble_getaddress(struct bt_addr_s *addr)
{
  /* TODO: a user-defined address could be retrieved from UICR */

  /* if the device holds a public (not random) address in FICR */

  if (getreg32(NRF52_FICR_DEVICEADDRTYPE) == 0)
    {
      /* get lower 32 and upper 16 bit of public address */

      *(uint32_t *)addr       = getreg32(NRF52_FICR_DEVICEADDR0);
      *((uint32_t *)addr + 1) = getreg32(NRF52_FICR_DEVICEADDR1) & 0xffff;

      return 0;
    }
  else
    {
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: nrf52_ble_txpower
 *
 * Description:
 *   Set requested transmission power in dBm
 *
 ****************************************************************************/

int nrf52_ble_txpower(int8_t dbm)
{
  return g_radio_dev->ops->txpower_set(g_radio_dev, dbm);
}

/****************************************************************************
 * Name: nrf52_ble_state
 *
 * Description:
 *   Return the current Link Layer state
 *
 ****************************************************************************/

int nrf52_ble_state(void)
{
  return g_ble_dev.state;
}

/****************************************************************************
 * Name: nrf52_ble_reset
 *
 * Description:
 *   Reset
 *
 ****************************************************************************/

int nrf52_ble_reset(void)
{
  /* TODO: should abort anything in progress */

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_rand
 *
 * Description:
 *   Generate random bytes
 *
 ****************************************************************************/

/* TODO: use not so direct interface to RNG */

void nrf52_ble_rand(uint8_t *data, size_t len)
{
  int i = 0;
#if 0

  /* Start generating data */

  putreg32(0, NRF52_RNG_EVENTS_RDY);
  putreg32(0x1, NRF52_RNG_TASKS_START);

  /* Generate len bytes of data */

  for (i = 0; i < len; i++)
    {
      while (!getreg32(NRF52_RNG_EVENTS_RDY))
        {
          /* Wait until byte is generated */
        }

      putreg32(0, NRF52_RNG_EVENTS_RDY);

      data[i] = getreg32(NRF52_RNG_VALUE);
    }

  /* Stop generating data */

  putreg32(0x1, NRF52_RNG_TASKS_STOP);
#else
  for (i = 0; i < len; i++)
    {
      data[i] = i;
    }
#endif
}

void nrf52_ble_set_ltk(uint8_t conn_id, uint8_t *ltk)
{
  struct bt_control_pdu_payload_s ctrl_pdu;
  struct nrf52_ble_conn_s *conn = &g_ble_dev.connections[0];

  /* received LTK from host, generate session key */

  nrf52_ble_sec_gen_key(conn->skd, conn->skdm, ltk);

  /* send a START_ENC PDU to peer */

  ctrl_pdu.opcode = BT_LL_START_ENC_REQ;
  nrf52_ble_send_ctrl_pdu(&ctrl_pdu, 1);
}

/****************************************************************************
 * Name: nrf52_ble_configure
 *
 * Description:
 *   Configure the RADIO peripheral for BLE
 *
 ****************************************************************************/

int nrf52_ble_configure(void)
{
  /* Configure for standard 1MBit BLE mode */

  g_radio_dev->ops->mode_set(g_radio_dev, NRF52_RADIO_MODE_BLE1MBIT);

  /* Set 150uS inter frame spacing */

  g_radio_dev->ops->tifs_set(g_radio_dev, 150);

  /* Configure CRC for BLE */

  g_radio_dev->ops->crc_cfg(g_radio_dev, &g_ble_crc_config);

  /* Setup RADIO isr */

  g_radio_dev->ops->setisr(g_radio_dev, nrf52_ble_isr, NULL);

  /* Set pointer to communication buffer */

  g_radio_dev->ops->set_packetptr(g_radio_dev, g_ble_dev.buffer);

  /* Configure logical access address 0 for ADV PDUs */

  g_radio_dev->ops->addr_set(g_radio_dev, 0,
                             BT_LL_ADV_ACCESS_ADDRESS >> 24,
                             BT_LL_ADV_ACCESS_ADDRESS << 8);

  return OK;
}
