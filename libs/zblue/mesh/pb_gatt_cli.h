/****************************************************************************
 * libs/zblue/mesh/pb_gatt_cli.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2021 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int bt_mesh_pb_gatt_cli_setup(const uint8_t uuid[16]);

void bt_mesh_pb_gatt_cli_adv_recv(const struct bt_le_scan_recv_info *info,
                                  struct net_buf_simple *buf);
