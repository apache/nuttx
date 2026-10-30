/****************************************************************************
 * libs/zblue/common/rpa.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* rpa.h - Bluetooth Resolvable Private Addresses (RPA) generation and
 * resolution
 */

/* Copyright (c) 2017 Nordic Semiconductor ASA Copyright (c) 2015-2016 Intel
 * Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/hci.h>

bool bt_rpa_irk_matches(const uint8_t irk[16], const bt_addr_t *addr);
int bt_rpa_create(const uint8_t irk[16], bt_addr_t *rpa);
