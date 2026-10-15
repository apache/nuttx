/****************************************************************************
 * libs/zblue/host/classic/conn_br_internal.h
 *
 * Ported to Apache NuttX by chao an <anchao.archer@bytedance.com>.
 *
 ****************************************************************************/

/** @file
 *  @brief Internal APIs for Bluetooth classic connection handling.
 */

/* Copyright (c) 2015 Intel Corporation Copyright (c) 2021 Nordic
 * Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int bt_hci_connect_br_cancel(struct bt_conn *conn);

void bt_br_acl_recv(struct bt_conn *conn,
                    struct net_buf *buf, bool complete);
