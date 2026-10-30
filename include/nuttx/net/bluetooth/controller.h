/****************************************************************************
 * include/nuttx/net/bluetooth/controller.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/** @file
 *  @brief Bluetooth subsystem controller APIs.
 */

/* Copyright (c) 2018 Codecoup
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_CONTROLLER_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_CONTROLLER_H

/**
 * @brief Bluetooth Controller
 * @defgroup bt_ctrl Bluetooth Controller
 * @ingroup bluetooth
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Set public address for controller
 *
 *  Should be called before bt_enable().
 *
 *  @param addr Public address
 */

void bt_ctlr_set_public_addr(const uint8_t *addr);

#ifdef __cplusplus
}
#endif
/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_CONTROLLER_H */
