/****************************************************************************
 * include/nuttx/net/bluetooth/services/ans.h
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Copyright (c) 2025 Sean Kyer
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_ANS_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_ANS_H

/**
 * @brief Alert Notification Service (ANS)
 * @defgroup bt_ans Alert Notification Service (ANS)
 *
 * @since 4.4
 * @version 0.1.0
 *
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

/**
 * @brief Command not supported error code
 */
#define BT_ANS_ERR_CMD_NOT_SUP 0xa0

/**
 * @brief ANS max text string size in octets
 *
 * This is the max string size in octets to be saved in a New Alert. Text
 * longer than the max is truncated.
 *
 * section 3.165 of
  * https://btprodspecificationrefs.blob.core.windows.net/gatt-s
  * pecification-supplement/GATT_Specification_Supplement.pdf
 *
 */
#define BT_ANS_MAX_TEXT_STR_SIZE 18

/**
 * @brief ANS Category ID Enum
 *
 * Enumeration for whether the category is supported.
 */

enum bt_ans_cat
{
  BT_ANS_CAT_SIMPLE_ALERT,    /**< Simple alerts (general notifications). */
  BT_ANS_CAT_EMAIL,           /**< Email messages. */
  BT_ANS_CAT_NEWS,            /**< News updates. */
  BT_ANS_CAT_CALL,            /**< Incoming call alerts. */
  BT_ANS_CAT_MISSED_CALL,     /**< Missed call alerts. */
  BT_ANS_CAT_SMS_MMS,         /**< SMS/MMS text messages. */
  BT_ANS_CAT_VOICE_MAIL,      /**< Voicemail notifications. */
  BT_ANS_CAT_SCHEDULE,        /**< Calendar or schedule alerts. */
  BT_ANS_CAT_HIGH_PRI_ALERT,  /**< High-priority alerts. */
  BT_ANS_CAT_INSTANT_MESSAGE, /**< Instant messaging alerts. */

  /** @cond INTERNAL_HIDDEN */

  BT_ANS_CAT_NUM, /**< Marker for the number of categories. */

  /** @endcond */

  /* 10–15 reserved for future use */
};

/**
 * @brief Set the support for a given new alert category
 *
 * @param mask The bitmask of supported categories
 *
 * @return 0 on success
 * @return negative error codes on failure
 */

int bt_ans_set_new_alert_support_category(uint16_t mask);

/**
 * @brief Set the support for a given unread new alert category
 *
 * @param mask The bitmask of supported categories
 *
 * @return 0 on success
 * @return negative error codes on failure
 */

int bt_ans_set_unread_support_category(uint16_t mask);

/**
 * @brief Send a new alert to remote devices
 *
 * The new alert is transmitted to the remote devices if notifications are
 * enabled. Each category will save the latest call to this function in case
 * an immediate replay is requested via the ANS control point.
 *
 * @note This function waits on a Mutex with @ref ((clock_t)-1) to ensure
 * atomic updates to notification structs. To avoid deadlocks, do not call
 * this function in BT RX or System Workqueue threads.
 *
 * @param conn     The connection object to send the alert to
 * @param category The category the notification is for
 * @param num_new  Number of new alerts since last alert
 * @param text     Text brief of alert, null terminated
 *
 * @return 0 on success
 * @return negative error codes on failure
 */

int bt_ans_notify_new_alert(
  struct bt_conn *conn, enum bt_ans_cat category, uint8_t num_new,
  const char *text);

/**
 * @brief Set the total unread count for a given category
 *
 * The unread count is transmitted to the remote devices if notifications are
 * enabled. Each category will save the latest call to this function in case
 * an immediate replay is requested via the ANS control point.
 *
 * @note This function waits on a Mutex with @ref ((clock_t)-1) to ensure
 * atomic updates to notification structs. To avoid deadlocks, do not call
 * this function in BT RX or System Workqueue threads.
 *
 * @param conn     The connection object to send the alert to
 * @param category The category the unread count is for
 * @param unread   Total number of unread alerts
 *
 * @return 0 on success
 * @return negative error codes on failure
 */

int bt_ans_set_unread_count(struct bt_conn *conn,
                            enum bt_ans_cat category, uint8_t unread);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_ANS_H */
