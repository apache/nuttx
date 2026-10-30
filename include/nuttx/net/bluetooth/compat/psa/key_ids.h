/****************************************************************************
 * include/nuttx/net/bluetooth/compat/psa/key_ids.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2025 Nordic Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_PSA_KEY_IDS_H_
#define ZEPHYR_PSA_KEY_IDS_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

typedef uint32_t psa_key_id_t;

#define ZEPHYR_PSA_OPENTHREAD_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x20000
#define ZEPHYR_PSA_OPENTHREAD_KEY_ID_RANGE_SIZE  0x10000

#define ZEPHYR_PSA_MATTER_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x30000
#define ZEPHYR_PSA_MATTER_KEY_ID_RANGE_SIZE  0x10000

#define ZEPHYR_PSA_BT_MESH_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x20000000
#define ZEPHYR_PSA_BT_MESH_KEY_ID_RANGE_SIZE  0xc000

#define ZEPHYR_PSA_WIFI_CREDENTIALS_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x20010000
#define ZEPHYR_PSA_WIFI_CREDENTIALS_KEY_ID_RANGE_SIZE  0x100

#define ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x30000000
#define ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_SIZE  0x100000

#endif /* ZEPHYR_PSA_KEY_IDS_H_ */
