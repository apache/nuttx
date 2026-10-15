/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/keys.h
 *
 * Ported to Apache NuttX by chao an <anchao.archer@bytedance.com>.
 *
 ****************************************************************************/

/** @file
 *  @brief Keys APIs.
 */

/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_KEYS_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_KEYS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <psa/crypto.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** The structure that keeps representation of key. */

struct bt_mesh_key
{
  /** PSA key representation is the PSA key identifier. */

  psa_key_id_t key;
};

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_KEYS_H */
