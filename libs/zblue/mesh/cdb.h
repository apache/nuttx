/****************************************************************************
 * libs/zblue/mesh/cdb.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

void bt_mesh_cdb_node_store(const struct bt_mesh_cdb_node *node);
void bt_mesh_cdb_pending_store(void);
