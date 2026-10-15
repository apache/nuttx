/****************************************************************************
 * net/bt/services/nus/bt_nus_auto_start_bt.c
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

/* Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/classic/classic.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <stdio.h>
#include <string.h>
#if defined(CONFIG_BT_A2DP_SINK)
#include <nuttx/net/bluetooth/classic/a2dp_codec_sbc.h>
#include <nuttx/net/bluetooth/classic/a2dp.h>
#include <nuttx/net/bluetooth/classic/sdp.h>
#endif
#if defined(CONFIG_BT_AVRCP)
#include <nuttx/net/bluetooth/classic/avrcp.h>
#endif
#include <nuttx/net/bluetooth/services/nus.h>

LOG_MODULE_REGISTER(bt_nus_auto_start);

#define DEVICE_NAME_SEPARATOR "-BT-"
#define DEVICE_NAME_SUFFIX_LEN 4
#define DEVICE_NAME_DYNAMIC_LEN                                                                    \
  (sizeof(CONFIG_BT_DEVICE_NAME) - 1 + sizeof(DEVICE_NAME_SEPARATOR) - 1 + DEVICE_NAME_SUFFIX_LEN)

#if defined(CONFIG_BT_DEVICE_NAME_DYNAMIC)
#define DEVICE_NAME_BUF_LEN MAX(CONFIG_BT_DEVICE_NAME_MAX, DEVICE_NAME_DYNAMIC_LEN)
#else
#define DEVICE_NAME_BUF_LEN MAX(sizeof(CONFIG_BT_DEVICE_NAME) - 1, DEVICE_NAME_DYNAMIC_LEN)
#endif

static char device_name[DEVICE_NAME_BUF_LEN + 1];

#if defined(CONFIG_BT_BROADCASTER)
#if defined(CONFIG_BT_CLASSIC)
#define NUS_AD_FLAGS BT_LE_AD_GENERAL
#else
#define NUS_AD_FLAGS (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)
#endif

static struct bt_data ad[] =
{
        BT_DATA_BYTES(BT_DATA_FLAGS, NUS_AD_FLAGS),
        BT_DATA(BT_DATA_NAME_COMPLETE, device_name, 0),
};

static const struct bt_data sd[] =
{
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};
#endif

#define BT_NUS_GATT_TEST_VALUE_MAX 32

static const struct bt_uuid_128 bt_nus_gatt_test_svc_uuid =
        BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8e7d0001,
                         0xade5,
                         0x4f9f,
                         0x8d15,
                         0x001122334455));

static const struct bt_uuid_128 bt_nus_gatt_test_value_uuid =
        BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8e7d0002,
                         0xade5,
                         0x4f9f,
                         0x8d15,
                         0x001122334455));

static const struct bt_uuid_128 bt_nus_gatt_test_count_uuid =
        BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8e7d0003,
                         0xade5,
                         0x4f9f,
                         0x8d15,
                         0x001122334455));

static const struct bt_uuid_128 bt_nus_gatt_echo_svc_uuid =
        BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8e7d0011,
                         0xade5,
                         0x4f9f,
                         0x8d15,
                         0x001122334455));

static const struct bt_uuid_128 bt_nus_gatt_echo_uuid =
        BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8e7d0012,
                         0xade5,
                         0x4f9f,
                         0x8d15,
                         0x001122334455));

static uint8_t bt_nus_gatt_test_value[BT_NUS_GATT_TEST_VALUE_MAX] =
{
        'N', 'u', 't', 't', 'X', ' ', 'G', 'A',
        'T', 'T', ' ', 'T', 'e', 's', 't'
};

static uint16_t bt_nus_gatt_test_value_len = 15;
static uint32_t bt_nus_gatt_test_count;

static ssize_t bt_nus_gatt_test_read_value(
  struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                          bt_nus_gatt_test_value,
                          bt_nus_gatt_test_value_len);
}

static ssize_t bt_nus_gatt_test_write_value(
  struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
  uint16_t len, uint16_t offset, uint8_t flags)
{
  uint16_t end;

  (void)conn;
  (void)attr;

  if (flags & BT_GATT_WRITE_FLAG_PREPARE)
    {
      return 0;
    }

  if (offset > sizeof(bt_nus_gatt_test_value))
    {
      return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

  end = offset + len;
  if (end > sizeof(bt_nus_gatt_test_value))
    {
      return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

  memcpy(bt_nus_gatt_test_value + offset, buf, len);
  bt_nus_gatt_test_value_len = MAX(bt_nus_gatt_test_value_len, end);
  LOG_INF("GATT test value updated: len %u", bt_nus_gatt_test_value_len);

  return len;
}

static ssize_t bt_nus_gatt_test_read_count(
  struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
  uint16_t len, uint16_t offset)
{
  bt_nus_gatt_test_count++;
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                          &bt_nus_gatt_test_count,
                          sizeof(bt_nus_gatt_test_count));
}

static ssize_t bt_nus_gatt_echo_write(
  struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
  uint16_t len, uint16_t offset, uint8_t flags)
{
  uint16_t ntf_len;
  int max_ntf_size;
  int err;

  (void)offset;

  if (flags & BT_GATT_WRITE_FLAG_PREPARE)
    {
      return 0;
    }

  if (!conn)
    {
      return len;
    }

  max_ntf_size = bt_att_get_max_notify_size(conn, BT_ATT_CHAN_OPT_NONE);
  if (max_ntf_size < 0)
    {
      return len;
    }

  if (!bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY))
    {
      return len;
    }

  ntf_len = MIN(len, (uint16_t)max_ntf_size);
  err = bt_gatt_notify(conn, attr, buf, ntf_len);
  if (err)
    {
      LOG_WRN("Failed to notify GATT echo data (err %d)", err);
    }

  return len;
}

BT_GATT_SERVICE_DEFINE(bt_nus_gatt_test_svc,
  BT_GATT_PRIMARY_SERVICE(&bt_nus_gatt_test_svc_uuid.uuid),
  BT_GATT_CHARACTERISTIC(&bt_nus_gatt_test_value_uuid.uuid,
                         BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                         bt_nus_gatt_test_read_value,
                         bt_nus_gatt_test_write_value, NULL),
  BT_GATT_CHARACTERISTIC(&bt_nus_gatt_test_count_uuid.uuid,
                         BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
                         bt_nus_gatt_test_read_count, NULL, NULL),
  BT_GATT_CUD("NuttX demo counter +1", BT_GATT_PERM_READ),
);

BT_GATT_SERVICE_DEFINE(bt_nus_gatt_echo_svc,
  BT_GATT_PRIMARY_SERVICE(&bt_nus_gatt_echo_svc_uuid.uuid),
  BT_GATT_CHARACTERISTIC(&bt_nus_gatt_echo_uuid.uuid,
                         BT_GATT_CHRC_WRITE_WITHOUT_RESP |
                         BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_WRITE, NULL,
                         bt_nus_gatt_echo_write, NULL),
  BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static enum bt_security_err bt_nus_pairing_accept(struct bt_conn *conn,
  const struct bt_conn_pairing_feat *const feat)
{
  (void)feat;

  LOG_INF("Pairing accepted: %s", bt_conn_dst_str(conn));
  return BT_SECURITY_ERR_SUCCESS;
}
#endif

static void bt_nus_auth_cancel(struct bt_conn *conn)
{
  LOG_INF("Pairing cancelled: %s", bt_conn_dst_str(conn));
}

static void bt_nus_pairing_confirm(struct bt_conn *conn)
{
  int err;

  LOG_INF("Pairing confirmed: %s", bt_conn_dst_str(conn));

  err = bt_conn_auth_pairing_confirm(conn);
  if (err)
    {
      LOG_WRN("Failed to confirm pairing for %s (err %d)",
              bt_conn_dst_str(conn), err);
    }
}

#if defined(CONFIG_BT_CLASSIC)
static void bt_nus_pincode_entry(struct bt_conn *conn, bool highsec)
{
  const char *pin = highsec ? "0000000000000000" : "0000";
  int err;

  LOG_INF("BR/EDR PIN requested: %s highsec %u", bt_conn_dst_str(conn),
          (unsigned int)highsec);

  err = bt_conn_auth_pincode_entry(conn, pin);
  if (err)
    {
      LOG_WRN("Failed to enter BR/EDR PIN for %s (err %d)",
              bt_conn_dst_str(conn), err);
    }
}
#endif

static const struct bt_conn_auth_cb bt_nus_auth_cb =
{
#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
        .pairing_accept = bt_nus_pairing_accept,
#endif
#if defined(CONFIG_BT_CLASSIC)
        .pincode_entry = bt_nus_pincode_entry,
#endif
        .cancel = bt_nus_auth_cancel,
        .pairing_confirm = bt_nus_pairing_confirm,
};

static int bt_nus_auth_register(void)
{
  int err;

  err = bt_conn_auth_cb_register(&bt_nus_auth_cb);
  if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Failed to register auth callbacks (err %d)", err);
      return err;
    }

  LOG_INF("Bluetooth auth callbacks registered");
  return 0;
}
#endif

#if defined(CONFIG_BT_CLASSIC)
static struct bt_data eir[] =
{
        BT_DATA(BT_DATA_NAME_COMPLETE, device_name, 0),
};

#endif

static int bt_nus_update_device_name(void)
{
  bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
  size_t count = ARRAY_SIZE(addrs);
#if defined(CONFIG_BT_DEVICE_NAME_DYNAMIC)
  int err;
#endif
  int len;

  bt_id_get(addrs, &count);
  if (count > 0 && !bt_addr_le_eq(&addrs[BT_ID_DEFAULT], BT_ADDR_LE_ANY))
    {
      len = snprintf(device_name, sizeof(device_name), "%s%s%02X%02X",
              CONFIG_BT_DEVICE_NAME,
                     DEVICE_NAME_SEPARATOR, addrs[BT_ID_DEFAULT].a.val[1],
                     addrs[BT_ID_DEFAULT].a.val[0]);
    }
  else
    {
      len = snprintf(device_name, sizeof(device_name), "%s",
              CONFIG_BT_DEVICE_NAME);
    }

  if (len < 0 || (size_t)len >= sizeof(device_name))
    {
      return -ENOMEM;
    }

#if defined(CONFIG_BT_DEVICE_NAME_DYNAMIC)
  err = bt_set_name(device_name);

  if (err < 0)
    {
      LOG_ERR("Failed to set Bluetooth device name (err %d)", err);
      return err;
    }
#endif

#if defined(CONFIG_BT_BROADCASTER)
  ad[1].data_len = strlen(device_name);
#endif
#if defined(CONFIG_BT_CLASSIC)
  eir[0].data_len = strlen(device_name);
#endif

  LOG_INF("Bluetooth device name: %s", device_name);
  return 0;
}

#if defined(CONFIG_BT_CLASSIC)

static void bt_nus_dump_buf(const char *tag, const struct net_buf *buf,
                            size_t max_len)
{
  size_t dump_len;
  size_t offset;

  if (!buf)
    {
      LOG_INF("%s: buf NULL", tag);
      return;
    }

  dump_len = MIN((size_t)buf->len, max_len);
  LOG_INF("%s: len %u dump %u", tag, buf->len, (unsigned int)dump_len);

  for (offset = 0; offset < dump_len; offset += 16)
    {
      const uint8_t *data = buf->data + offset;
      size_t left = dump_len - offset;
      size_t line_len = MIN(left, 16);

      LOG_INF("%s[%u]: %02x %02x %02x %02x %02x %02x %02x %02x "
              "%02x %02x %02x %02x %02x %02x %02x %02x",
              tag, (unsigned int)offset,
              line_len > 0 ? data[0] : 0,
              line_len > 1 ? data[1] : 0,
              line_len > 2 ? data[2] : 0,
              line_len > 3 ? data[3] : 0,
              line_len > 4 ? data[4] : 0,
              line_len > 5 ? data[5] : 0,
              line_len > 6 ? data[6] : 0,
              line_len > 7 ? data[7] : 0,
              line_len > 8 ? data[8] : 0,
              line_len > 9 ? data[9] : 0,
              line_len > 10 ? data[10] : 0,
              line_len > 11 ? data[11] : 0,
              line_len > 12 ? data[12] : 0,
              line_len > 13 ? data[13] : 0,
              line_len > 14 ? data[14] : 0,
              line_len > 15 ? data[15] : 0);
    }
}

#if defined(CONFIG_BT_A2DP_SINK)
#define A2DP_VERSION 0x0104u

#define A2DP_AAC_OBJECT_TYPE_MPEG2_AAC_LC 0x80u
#define A2DP_AAC_OBJECT_TYPE_MPEG4_AAC_LC 0x40u
#define A2DP_AAC_SAMP_FREQ_48000_HI 0x10u
#define A2DP_AAC_SAMP_FREQ_44100_HI 0x08u
#define A2DP_AAC_CHANNELS_2 0x04u
#define A2DP_AAC_VBR 0x80u
#define A2DP_AAC_BIT_RATE_320000 0x04e200u

BT_A2DP_SBC_SINK_EP_DEFAULT(bt_nus_a2dp_sink_sbc_endpoint);

static struct bt_a2dp_codec_ie bt_nus_a2dp_sink_aac_cap =
{
        .len = BT_A2DP_MPEG_2_4_IE_LENGTH,
        .codec_ie =
        {
          A2DP_AAC_OBJECT_TYPE_MPEG2_AAC_LC |
          A2DP_AAC_OBJECT_TYPE_MPEG4_AAC_LC,
          A2DP_AAC_SAMP_FREQ_48000_HI |
          A2DP_AAC_SAMP_FREQ_44100_HI,
          A2DP_AAC_CHANNELS_2,
          A2DP_AAC_VBR |
          ((A2DP_AAC_BIT_RATE_320000 >> 16) & 0x7fu),
          (A2DP_AAC_BIT_RATE_320000 >> 8) & 0xffu,
          A2DP_AAC_BIT_RATE_320000 & 0xffu,
        },
};

static struct bt_a2dp_ep bt_nus_a2dp_sink_aac_endpoint =
        BT_A2DP_SINK_EP_INIT(BT_A2DP_MPEG2,
                             &bt_nus_a2dp_sink_aac_cap, true);

static struct bt_a2dp_stream bt_nus_a2dp_sink_stream;

static struct bt_sdp_attribute bt_nus_a2dp_sink_attrs[] =
{
  BT_SDP_NEW_SERVICE,
  BT_SDP_LIST(BT_SDP_ATTR_SVCLASS_ID_LIST,
    BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 3),
    BT_SDP_DATA_ELEM_LIST(
        {
          BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
          BT_SDP_ARRAY_16(BT_SDP_AUDIO_SINK_SVCLASS)
        },

    )
  ),

  BT_SDP_LIST(BT_SDP_ATTR_PROTO_DESC_LIST,
    BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 16),
    BT_SDP_DATA_ELEM_LIST(
        {
          BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 6),
          BT_SDP_DATA_ELEM_LIST(
            {
              BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
              BT_SDP_ARRAY_16(BT_SDP_PROTO_L2CAP)
            },

            {
              BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
              BT_SDP_ARRAY_16(BT_UUID_AVDTP_VAL)
            },

            )
        },

        {
          BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 6),
          BT_SDP_DATA_ELEM_LIST(
            {
              BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
              BT_SDP_ARRAY_16(BT_UUID_AVDTP_VAL)
            },

            {
              BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
              BT_SDP_ARRAY_16(AVDTP_VERSION)
            },

            )
        },

    )
  ),

  BT_SDP_LIST(BT_SDP_ATTR_PROFILE_DESC_LIST,
      BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 8),
      BT_SDP_DATA_ELEM_LIST(
        {
          BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 6),
          BT_SDP_DATA_ELEM_LIST(
            {
              BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
              BT_SDP_ARRAY_16(BT_SDP_ADVANCED_AUDIO_SVCLASS)
            },

            {
              BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
              BT_SDP_ARRAY_16(A2DP_VERSION)
            },

          )
        },

        )
  ),

  BT_SDP_SERVICE_NAME("A2DPSink"),
  BT_SDP_SUPPORTED_FEATURES(0x0001u),
};

static struct bt_sdp_record bt_nus_a2dp_sink_rec = BT_SDP_RECORD(
  bt_nus_a2dp_sink_attrs);

static void bt_nus_a2dp_log_codec_cfg(const char *tag,
                                      struct bt_a2dp_codec_cfg *codec_cfg)
{
  struct bt_a2dp_codec_ie *codec_ie;

  if (!codec_cfg)
    {
      LOG_INF("%s: codec cfg NULL", tag);
      return;
    }

  codec_ie = codec_cfg->codec_config;
  LOG_INF("%s: delay_report %u codec_ie %p", tag,
          codec_cfg->delay_report, codec_ie);

  if (codec_ie)
    {
      struct net_buf buf =
        {
          .data = codec_ie->codec_ie,
          .len = codec_ie->len,
        };

      bt_nus_dump_buf(tag, (const struct net_buf *)&buf,
                      codec_ie->len);
    }
}

static void bt_nus_a2dp_connected(struct bt_a2dp *a2dp, int err)
{
  LOG_INF("A2DP sink connected callback: a2dp %p err %d", a2dp, err);

  if (err)
    {
      LOG_WRN("A2DP sink connect failed (err %d)", err);
      return;
    }

  LOG_INF("A2DP sink connected");
}

static void bt_nus_a2dp_disconnected(struct bt_a2dp *a2dp)
{
  LOG_INF("A2DP sink disconnected callback: a2dp %p", a2dp);
}

static void bt_nus_a2dp_stream_configured(struct bt_a2dp_stream *stream)
{
  LOG_INF("A2DP sink stream configured callback: stream %p", stream);

  {
    struct net_buf buf =
      {
        .data = stream->codec_config.codec_ie,
        .len = stream->codec_config.len,
      };

    bt_nus_dump_buf("A2DP stream codec config",
                    (const struct net_buf *)&buf,
                    stream->codec_config.len);
  }
}

static void bt_nus_a2dp_stream_established(struct bt_a2dp_stream *stream)
{
  LOG_INF("A2DP sink stream established callback: stream %p", stream);
}

static void bt_nus_a2dp_stream_released(struct bt_a2dp_stream *stream)
{
  LOG_INF("A2DP sink stream released callback: stream %p", stream);
}

static void bt_nus_a2dp_stream_started(struct bt_a2dp_stream *stream)
{
  LOG_INF("A2DP sink stream started callback: stream %p", stream);
}

static void bt_nus_a2dp_stream_suspended(struct bt_a2dp_stream *stream)
{
  LOG_INF("A2DP sink stream suspended callback: stream %p", stream);
}

static void bt_nus_a2dp_stream_recv(
  struct bt_a2dp_stream *stream, struct net_buf *buf, uint16_t seq_num,
  uint32_t ts)
{
  LOG_INF("A2DP sink media recv callback: stream %p seq %u ts %u len %u",
          stream, seq_num, (unsigned int)ts, buf ? buf->len : 0);
  bt_nus_dump_buf("A2DP media", buf, 32);
}

static struct bt_a2dp_stream_ops bt_nus_a2dp_stream_ops =
{
        .configured = bt_nus_a2dp_stream_configured,
        .established = bt_nus_a2dp_stream_established,
        .released = bt_nus_a2dp_stream_released,
        .started = bt_nus_a2dp_stream_started,
        .suspended = bt_nus_a2dp_stream_suspended,
        .recv = bt_nus_a2dp_stream_recv,
};

static int bt_nus_a2dp_config_req(
  struct bt_a2dp *a2dp, struct bt_a2dp_ep *ep,
  struct bt_a2dp_codec_cfg *codec_cfg, struct bt_a2dp_stream **stream,
  uint8_t *rsp_err_code)
{
  bt_a2dp_stream_cb_register(&bt_nus_a2dp_sink_stream,
          &bt_nus_a2dp_stream_ops);
  *stream = &bt_nus_a2dp_sink_stream;
  *rsp_err_code = 0;

  LOG_INF("A2DP sink config_req callback: a2dp %p ep %p stream %p",
          a2dp, ep, *stream);
  bt_nus_a2dp_log_codec_cfg("A2DP config_req codec", codec_cfg);
  return 0;
}

static int bt_nus_a2dp_accept_req(const char *name,
                                  struct bt_a2dp_stream *stream,
                                  uint8_t *rsp_err_code)
{
  *rsp_err_code = 0;
  LOG_INF("A2DP sink %s callback: stream %p", name, stream);
  return 0;
}

static int bt_nus_a2dp_establish_req(struct bt_a2dp_stream *stream,
                                     uint8_t *rsp_err_code)
{
  return bt_nus_a2dp_accept_req("establish_req", stream, rsp_err_code);
}

static int bt_nus_a2dp_release_req(struct bt_a2dp_stream *stream,
                                   uint8_t *rsp_err_code)
{
  return bt_nus_a2dp_accept_req("release_req", stream, rsp_err_code);
}

static int bt_nus_a2dp_start_req(struct bt_a2dp_stream *stream,
                                 uint8_t *rsp_err_code)
{
  return bt_nus_a2dp_accept_req("start_req", stream, rsp_err_code);
}

static int bt_nus_a2dp_suspend_req(struct bt_a2dp_stream *stream,
                                   uint8_t *rsp_err_code)
{
  return bt_nus_a2dp_accept_req("suspend_req", stream, rsp_err_code);
}

static int bt_nus_a2dp_abort_req(struct bt_a2dp_stream *stream,
                                 uint8_t *rsp_err_code)
{
  return bt_nus_a2dp_accept_req("abort_req", stream, rsp_err_code);
}

static int bt_nus_a2dp_get_config_req(struct bt_a2dp_stream *stream,
                                      uint8_t *rsp_err_code)
{
  return bt_nus_a2dp_accept_req("get_config_req", stream, rsp_err_code);
}

static int bt_nus_a2dp_reconfig_req(struct bt_a2dp_stream *stream,
                                    struct bt_a2dp_codec_cfg *codec_cfg,
                                    uint8_t *rsp_err_code)
{
  *rsp_err_code = 0;
  LOG_INF("A2DP sink reconfig_req callback: stream %p", stream);
  bt_nus_a2dp_log_codec_cfg("A2DP reconfig_req codec", codec_cfg);
  return 0;
}

static void bt_nus_a2dp_config_rsp(struct bt_a2dp_stream *stream,
                                   uint8_t rsp_err_code)
{
  LOG_INF("A2DP sink config_rsp callback: stream %p rsp 0x%02x",
          stream, rsp_err_code);
}

static void bt_nus_a2dp_rsp(const char *name, struct bt_a2dp_stream *stream,
                            uint8_t rsp_err_code)
{
  LOG_INF("A2DP sink %s callback: stream %p rsp 0x%02x", name, stream,
          rsp_err_code);
}

static void bt_nus_a2dp_establish_rsp(struct bt_a2dp_stream *stream,
                                      uint8_t rsp_err_code)
{
  bt_nus_a2dp_rsp("establish_rsp", stream, rsp_err_code);
}

static void bt_nus_a2dp_release_rsp(struct bt_a2dp_stream *stream,
                                    uint8_t rsp_err_code)
{
  bt_nus_a2dp_rsp("release_rsp", stream, rsp_err_code);
}

static void bt_nus_a2dp_start_rsp(struct bt_a2dp_stream *stream,
                                  uint8_t rsp_err_code)
{
  bt_nus_a2dp_rsp("start_rsp", stream, rsp_err_code);
}

static void bt_nus_a2dp_suspend_rsp(struct bt_a2dp_stream *stream,
                                    uint8_t rsp_err_code)
{
  bt_nus_a2dp_rsp("suspend_rsp", stream, rsp_err_code);
}

static void bt_nus_a2dp_abort_rsp(struct bt_a2dp_stream *stream,
                                  uint8_t rsp_err_code)
{
  bt_nus_a2dp_rsp("abort_rsp", stream, rsp_err_code);
}

static void bt_nus_a2dp_get_config_rsp(
  struct bt_a2dp_stream *stream, struct bt_a2dp_codec_cfg *codec_cfg,
  uint8_t rsp_err_code)
{
  LOG_INF("A2DP sink get_config_rsp callback: stream %p rsp 0x%02x",
          stream, rsp_err_code);
  bt_nus_a2dp_log_codec_cfg("A2DP get_config_rsp codec", codec_cfg);
}

static void bt_nus_a2dp_delay_report_rsp(
  struct bt_a2dp_stream *stream, uint8_t rsp_err_code)
{
  bt_nus_a2dp_rsp("delay_report_rsp", stream, rsp_err_code);
}

static struct bt_a2dp_cb bt_nus_a2dp_cb =
{
        .connected = bt_nus_a2dp_connected,
        .disconnected = bt_nus_a2dp_disconnected,
        .config_req = bt_nus_a2dp_config_req,
        .reconfig_req = bt_nus_a2dp_reconfig_req,
        .config_rsp = bt_nus_a2dp_config_rsp,
        .establish_req = bt_nus_a2dp_establish_req,
        .establish_rsp = bt_nus_a2dp_establish_rsp,
        .release_req = bt_nus_a2dp_release_req,
        .release_rsp = bt_nus_a2dp_release_rsp,
        .start_req = bt_nus_a2dp_start_req,
        .start_rsp = bt_nus_a2dp_start_rsp,
        .suspend_req = bt_nus_a2dp_suspend_req,
        .suspend_rsp = bt_nus_a2dp_suspend_rsp,
        .abort_req = bt_nus_a2dp_abort_req,
        .abort_rsp = bt_nus_a2dp_abort_rsp,
        .get_config_req = bt_nus_a2dp_get_config_req,
        .get_config_rsp = bt_nus_a2dp_get_config_rsp,
        .delay_report_rsp = bt_nus_a2dp_delay_report_rsp,
};

static int bt_nus_a2dp_sink_register(void)
{
  int err;

  err = bt_sdp_register_service(&bt_nus_a2dp_sink_rec);
  if (err < 0 && err != -EEXIST)
    {
      LOG_ERR("Failed to register A2DP sink SDP record (err %d)", err);
      return err;
    }

  err = bt_a2dp_register_cb(&bt_nus_a2dp_cb);
  if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Failed to register A2DP sink callbacks (err %d)", err);
      return err;
    }

  err = bt_a2dp_register_ep(&bt_nus_a2dp_sink_sbc_endpoint, BT_AVDTP_AUDIO,
          BT_AVDTP_SINK);
  if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Failed to register A2DP SBC sink endpoint (err %d)", err);
      return err;
    }

  LOG_INF("A2DP SBC sink endpoint registered");

  err = bt_a2dp_register_ep(&bt_nus_a2dp_sink_aac_endpoint, BT_AVDTP_AUDIO,
          BT_AVDTP_SINK);
  if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Failed to register A2DP AAC sink endpoint (err %d)", err);
      return err;
    }

  LOG_INF("A2DP AAC sink endpoint registered");
  return 0;
}
#endif

#if defined(CONFIG_BT_AVRCP)
static void bt_nus_avrcp_log_event_data(const char *tag, uint8_t event_id,
                                        struct bt_avrcp_event_data *data)
{
  if (!data)
    {
      LOG_INF("%s: event 0x%02x data NULL", tag, event_id);
      return;
    }

  switch (event_id)
    {
      case BT_AVRCP_EVT_PLAYBACK_STATUS_CHANGED:
        LOG_INF("%s: playback status 0x%02x", tag, data->play_status);
        break;
      case BT_AVRCP_EVT_TRACK_CHANGED:
        LOG_INF("%s: track id %02x%02x%02x%02x%02x%02x%02x%02x", tag,
                data->identifier[0], data->identifier[1],
                data->identifier[2], data->identifier[3],
                data->identifier[4], data->identifier[5],
                data->identifier[6], data->identifier[7]);
        break;
      case BT_AVRCP_EVT_PLAYBACK_POS_CHANGED:
        LOG_INF("%s: playback pos %u", tag,
                (unsigned int)data->playback_pos);
        break;
      case BT_AVRCP_EVT_BATT_STATUS_CHANGED:
        LOG_INF("%s: battery status 0x%02x", tag, data->battery_status);
        break;
      case BT_AVRCP_EVT_SYSTEM_STATUS_CHANGED:
        LOG_INF("%s: system status 0x%02x", tag, data->system_status);
        break;
      case BT_AVRCP_EVT_PLAYER_APP_SETTING_CHANGED:
        LOG_INF("%s: player app setting count %u", tag,
                data->setting_changed.num_of_attr);
        break;
      case BT_AVRCP_EVT_ADDRESSED_PLAYER_CHANGED:
        LOG_INF("%s: addressed player id %u uid_counter %u", tag,
                data->addressed_player_changed.player_id,
                data->addressed_player_changed.uid_counter);
        break;
      case BT_AVRCP_EVT_UIDS_CHANGED:
        LOG_INF("%s: uid_counter %u", tag, data->uid_counter);
        break;
      case BT_AVRCP_EVT_VOLUME_CHANGED:
        LOG_INF("%s: absolute volume %u", tag, data->absolute_volume);
        break;
      default:
        LOG_INF("%s: event 0x%02x", tag, event_id);
        break;
    }
}

#if defined(CONFIG_BT_AVRCP_CONTROLLER)
static void bt_nus_avrcp_ct_connected(struct bt_conn *conn,
                                      struct bt_avrcp_ct *ct)
{
  LOG_INF("AVRCP CT connected callback: conn %s ct %p",
          bt_conn_dst_str(conn), ct);
}

static void bt_nus_avrcp_ct_disconnected(struct bt_avrcp_ct *ct)
{
  LOG_INF("AVRCP CT disconnected callback: ct %p", ct);
}

static void bt_nus_avrcp_ct_browsing_connected(struct bt_conn *conn,
                                               struct bt_avrcp_ct *ct)
{
  LOG_INF("AVRCP CT browsing_connected callback: conn %s ct %p",
          bt_conn_dst_str(conn), ct);
}

static void bt_nus_avrcp_ct_browsing_disconnected(struct bt_avrcp_ct *ct)
{
  LOG_INF("AVRCP CT browsing_disconnected callback: ct %p", ct);
}

static void bt_nus_avrcp_ct_get_caps(struct bt_avrcp_ct *ct, uint8_t tid,
                                     uint8_t status, struct net_buf *buf)
{
  LOG_INF("AVRCP CT get_caps callback: ct %p tid %u status 0x%02x len %u",
          ct, tid, status, buf ? buf->len : 0);
}

static void bt_nus_avrcp_ct_unit_info_rsp(
  struct bt_avrcp_ct *ct, uint8_t tid, struct bt_avrcp_unit_info_rsp *rsp)
{
  LOG_INF("AVRCP CT unit_info_rsp callback: ct %p tid %u rsp %p", ct, tid,
          rsp);
}

static void bt_nus_avrcp_ct_subunit_info_rsp(
  struct bt_avrcp_ct *ct, uint8_t tid, struct bt_avrcp_subunit_info_rsp *rsp)
{
  LOG_INF("AVRCP CT subunit_info_rsp callback: ct %p tid %u rsp %p", ct,
          tid, rsp);
}

static void bt_nus_avrcp_ct_passthrough_rsp(
  struct bt_avrcp_ct *ct, uint8_t tid, bt_avrcp_rsp_t result,
  const struct bt_avrcp_passthrough_rsp *rsp)
{
  LOG_INF("AVRCP CT passthrough_rsp callback: ct %p tid %u result 0x%02x "
          "rsp %p", ct, tid, result, rsp);
}

static void bt_nus_avrcp_ct_buf_rsp(const char *name,
                                    struct bt_avrcp_ct *ct, uint8_t tid,
                                    struct net_buf *buf)
{
  LOG_INF("AVRCP CT %s callback: ct %p tid %u len %u", name, ct, tid,
          buf ? buf->len : 0);
}

static void bt_nus_avrcp_ct_status_buf_rsp(
  const char *name, struct bt_avrcp_ct *ct, uint8_t tid, uint8_t status,
  struct net_buf *buf)
{
  LOG_INF("AVRCP CT %s callback: ct %p tid %u status 0x%02x len %u", name,
          ct, tid, status, buf ? buf->len : 0);
}

static void bt_nus_avrcp_ct_status_rsp(const char *name,
                                       struct bt_avrcp_ct *ct, uint8_t tid,
                                       uint8_t status)
{
  LOG_INF("AVRCP CT %s callback: ct %p tid %u status 0x%02x", name, ct,
          tid, status);
}

#define BT_NUS_AVRCP_CT_BUF_RSP(_name)                                      \
static void bt_nus_avrcp_ct_##_name(struct bt_avrcp_ct *ct, uint8_t tid,     \
                                    struct net_buf *buf)                     \
{                                                                            \
  bt_nus_avrcp_ct_buf_rsp(#_name, ct, tid, buf);                             \
}

#define BT_NUS_AVRCP_CT_STATUS_BUF_RSP(_name)                               \
static void bt_nus_avrcp_ct_##_name(struct bt_avrcp_ct *ct, uint8_t tid,     \
                                    uint8_t status, struct net_buf *buf)      \
{                                                                            \
  bt_nus_avrcp_ct_status_buf_rsp(#_name, ct, tid, status, buf);              \
}

#define BT_NUS_AVRCP_CT_STATUS_RSP(_name)                                   \
static void bt_nus_avrcp_ct_##_name(struct bt_avrcp_ct *ct, uint8_t tid,     \
                                    uint8_t status)                          \
{                                                                            \
  bt_nus_avrcp_ct_status_rsp(#_name, ct, tid, status);                       \
}

BT_NUS_AVRCP_CT_BUF_RSP(set_browsed_player)
BT_NUS_AVRCP_CT_BUF_RSP(get_folder_items)
BT_NUS_AVRCP_CT_BUF_RSP(get_item_attrs)
BT_NUS_AVRCP_CT_BUF_RSP(get_total_number_of_items)
BT_NUS_AVRCP_CT_BUF_RSP(search)
BT_NUS_AVRCP_CT_STATUS_BUF_RSP(list_player_app_setting_attrs)
BT_NUS_AVRCP_CT_STATUS_BUF_RSP(list_player_app_setting_vals)
BT_NUS_AVRCP_CT_STATUS_BUF_RSP(get_curr_player_app_setting_val)
BT_NUS_AVRCP_CT_STATUS_BUF_RSP(get_player_app_setting_attr_text)
BT_NUS_AVRCP_CT_STATUS_BUF_RSP(get_player_app_setting_val_text)
BT_NUS_AVRCP_CT_STATUS_BUF_RSP(get_element_attrs)
BT_NUS_AVRCP_CT_STATUS_BUF_RSP(get_play_status)
BT_NUS_AVRCP_CT_STATUS_RSP(set_player_app_setting_val)
BT_NUS_AVRCP_CT_STATUS_RSP(inform_displayable_char_set)
BT_NUS_AVRCP_CT_STATUS_RSP(inform_batt_status_of_ct)
BT_NUS_AVRCP_CT_STATUS_RSP(set_addressed_player)
BT_NUS_AVRCP_CT_STATUS_RSP(play_item)
BT_NUS_AVRCP_CT_STATUS_RSP(add_to_now_playing)

static void bt_nus_avrcp_ct_change_path(
  struct bt_avrcp_ct *ct, uint8_t tid, uint8_t status, uint32_t num_items)
{
  LOG_INF("AVRCP CT change_path callback: ct %p tid %u status 0x%02x "
          "num_items %u", ct, tid, status, (unsigned int)num_items);
}

static void bt_nus_avrcp_ct_browsing_general_reject(
  struct bt_avrcp_ct *ct, uint8_t tid, uint8_t status)
{
  LOG_INF("AVRCP CT browsing_general_reject callback: ct %p tid %u "
          "status 0x%02x", ct, tid, status);
}

static void bt_nus_avrcp_ct_notification(
  struct bt_avrcp_ct *ct, uint8_t tid, uint8_t status, uint8_t event_id,
  struct bt_avrcp_event_data *data)
{
  LOG_INF("AVRCP CT notification callback: ct %p tid %u status 0x%02x "
          "event 0x%02x", ct, tid, status, event_id);
  bt_nus_avrcp_log_event_data("AVRCP CT notification", event_id, data);
}

static void bt_nus_avrcp_ct_set_absolute_volume(
  struct bt_avrcp_ct *ct, uint8_t tid, uint8_t status, uint8_t volume)
{
  LOG_INF("AVRCP CT set_absolute_volume callback: ct %p tid %u "
          "status 0x%02x volume %u", ct, tid, status, volume);
}

static struct bt_avrcp_ct_cb bt_nus_avrcp_ct_cb =
{
        .connected = bt_nus_avrcp_ct_connected,
        .disconnected = bt_nus_avrcp_ct_disconnected,
        .browsing_connected = bt_nus_avrcp_ct_browsing_connected,
        .browsing_disconnected = bt_nus_avrcp_ct_browsing_disconnected,
        .get_caps = bt_nus_avrcp_ct_get_caps,
        .unit_info_rsp = bt_nus_avrcp_ct_unit_info_rsp,
        .subunit_info_rsp = bt_nus_avrcp_ct_subunit_info_rsp,
        .passthrough_rsp = bt_nus_avrcp_ct_passthrough_rsp,
        .set_browsed_player = bt_nus_avrcp_ct_set_browsed_player,
        .get_folder_items = bt_nus_avrcp_ct_get_folder_items,
        .change_path = bt_nus_avrcp_ct_change_path,
        .get_item_attrs = bt_nus_avrcp_ct_get_item_attrs,
        .get_total_number_of_items =
          bt_nus_avrcp_ct_get_total_number_of_items,
        .search = bt_nus_avrcp_ct_search,
        .browsing_general_reject = bt_nus_avrcp_ct_browsing_general_reject,
        .notification = bt_nus_avrcp_ct_notification,
        .list_player_app_setting_attrs =
          bt_nus_avrcp_ct_list_player_app_setting_attrs,
        .list_player_app_setting_vals =
          bt_nus_avrcp_ct_list_player_app_setting_vals,
        .get_curr_player_app_setting_val =
          bt_nus_avrcp_ct_get_curr_player_app_setting_val,
        .set_player_app_setting_val =
          bt_nus_avrcp_ct_set_player_app_setting_val,
        .get_player_app_setting_attr_text =
          bt_nus_avrcp_ct_get_player_app_setting_attr_text,
        .get_player_app_setting_val_text =
          bt_nus_avrcp_ct_get_player_app_setting_val_text,
        .inform_displayable_char_set =
          bt_nus_avrcp_ct_inform_displayable_char_set,
        .inform_batt_status_of_ct = bt_nus_avrcp_ct_inform_batt_status_of_ct,
        .set_absolute_volume = bt_nus_avrcp_ct_set_absolute_volume,
        .get_element_attrs = bt_nus_avrcp_ct_get_element_attrs,
        .get_play_status = bt_nus_avrcp_ct_get_play_status,
        .set_addressed_player = bt_nus_avrcp_ct_set_addressed_player,
        .play_item = bt_nus_avrcp_ct_play_item,
        .add_to_now_playing = bt_nus_avrcp_ct_add_to_now_playing,
};

static int bt_nus_avrcp_ct_register(void)
{
  int err;

  err = bt_avrcp_ct_register_cb(&bt_nus_avrcp_ct_cb);
  if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Failed to register AVRCP CT callbacks (err %d)", err);
      return err;
    }

  LOG_INF("AVRCP CT callbacks registered");
  return 0;
}
#endif /* CONFIG_BT_AVRCP_CONTROLLER */

#if defined(CONFIG_BT_AVRCP_TARGET)
static void bt_nus_avrcp_tg_connected(struct bt_conn *conn,
                                      struct bt_avrcp_tg *tg)
{
  LOG_INF("AVRCP TG connected callback: conn %s tg %p",
          bt_conn_dst_str(conn), tg);
}

static void bt_nus_avrcp_tg_disconnected(struct bt_avrcp_tg *tg)
{
  LOG_INF("AVRCP TG disconnected callback: tg %p", tg);
}

static void bt_nus_avrcp_tg_browsing_connected(struct bt_conn *conn,
                                               struct bt_avrcp_tg *tg)
{
  LOG_INF("AVRCP TG browsing_connected callback: conn %s tg %p",
          bt_conn_dst_str(conn), tg);
}

static void bt_nus_avrcp_tg_browsing_disconnected(struct bt_avrcp_tg *tg)
{
  LOG_INF("AVRCP TG browsing_disconnected callback: tg %p", tg);
}

static void bt_nus_avrcp_tg_unit_info_req(struct bt_avrcp_tg *tg,
                                          uint8_t tid)
{
  LOG_INF("AVRCP TG unit_info_req callback: tg %p tid %u", tg, tid);
}

static void bt_nus_avrcp_tg_subunit_info_req(struct bt_avrcp_tg *tg,
                                             uint8_t tid)
{
  LOG_INF("AVRCP TG subunit_info_req callback: tg %p tid %u", tg, tid);
}

static void bt_nus_avrcp_tg_get_caps(struct bt_avrcp_tg *tg, uint8_t tid,
                                     uint8_t cap_id)
{
  LOG_INF("AVRCP TG get_caps callback: tg %p tid %u cap_id 0x%02x", tg,
          tid, cap_id);
}

static void bt_nus_avrcp_tg_register_notification(
  struct bt_avrcp_tg *tg, uint8_t tid, uint8_t event_id, uint32_t interval)
{
  LOG_INF("AVRCP TG register_notification callback: tg %p tid %u "
          "event 0x%02x interval %u", tg, tid, event_id,
          (unsigned int)interval);
}

static void bt_nus_avrcp_tg_set_browsed_player(
  struct bt_avrcp_tg *tg, uint8_t tid, uint16_t player_id)
{
  LOG_INF("AVRCP TG set_browsed_player callback: tg %p tid %u player_id %u",
          tg, tid, player_id);
}

static void bt_nus_avrcp_tg_get_total_number_of_items(
  struct bt_avrcp_tg *tg, uint8_t tid, uint8_t scope)
{
  LOG_INF("AVRCP TG get_total_number_of_items callback: tg %p tid %u "
          "scope 0x%02x", tg, tid, scope);
}

static void bt_nus_avrcp_tg_list_player_app_setting_vals(
  struct bt_avrcp_tg *tg, uint8_t tid, uint8_t attr_id)
{
  LOG_INF("AVRCP TG list_player_app_setting_vals callback: tg %p tid %u "
          "attr_id 0x%02x", tg, tid, attr_id);
}

static void bt_nus_avrcp_tg_inform_batt_status_of_ct(
  struct bt_avrcp_tg *tg, uint8_t tid, uint8_t battery_status)
{
  LOG_INF("AVRCP TG inform_batt_status_of_ct callback: tg %p tid %u "
          "battery 0x%02x", tg, tid, battery_status);
}

static void bt_nus_avrcp_tg_set_absolute_volume(
  struct bt_avrcp_tg *tg, uint8_t tid, uint8_t volume)
{
  LOG_INF("AVRCP TG set_absolute_volume callback: tg %p tid %u volume %u",
          tg, tid, volume);
}

static void bt_nus_avrcp_tg_set_addressed_player(
  struct bt_avrcp_tg *tg, uint8_t tid, uint16_t player_id)
{
  LOG_INF("AVRCP TG set_addressed_player callback:"
         " tg %p tid %u player_id %u",
          tg, tid, player_id);
}

static void bt_nus_avrcp_tg_tid_req(const char *name,
                                    struct bt_avrcp_tg *tg, uint8_t tid)
{
  LOG_INF("AVRCP TG %s callback: tg %p tid %u", name, tg, tid);
}

static void bt_nus_avrcp_tg_buf_req(const char *name,
                                    struct bt_avrcp_tg *tg, uint8_t tid,
                                    struct net_buf *buf)
{
  LOG_INF("AVRCP TG %s callback: tg %p tid %u len %u", name, tg, tid,
          buf ? buf->len : 0);
}

#define BT_NUS_AVRCP_TG_TID_REQ(_name)                                      \
static void bt_nus_avrcp_tg_##_name(struct bt_avrcp_tg *tg, uint8_t tid)     \
{                                                                            \
  bt_nus_avrcp_tg_tid_req(#_name, tg, tid);                                  \
}

#define BT_NUS_AVRCP_TG_BUF_REQ(_name)                                      \
static void bt_nus_avrcp_tg_##_name(struct bt_avrcp_tg *tg, uint8_t tid,     \
                                    struct net_buf *buf)                     \
{                                                                            \
  bt_nus_avrcp_tg_buf_req(#_name, tg, tid, buf);                             \
}

BT_NUS_AVRCP_TG_BUF_REQ(get_folder_items)
BT_NUS_AVRCP_TG_BUF_REQ(change_path)
BT_NUS_AVRCP_TG_BUF_REQ(get_item_attrs)
BT_NUS_AVRCP_TG_BUF_REQ(search)
BT_NUS_AVRCP_TG_BUF_REQ(passthrough_req)
BT_NUS_AVRCP_TG_TID_REQ(list_player_app_setting_attrs)
BT_NUS_AVRCP_TG_BUF_REQ(get_curr_player_app_setting_val)
BT_NUS_AVRCP_TG_BUF_REQ(set_player_app_setting_val)
BT_NUS_AVRCP_TG_BUF_REQ(get_player_app_setting_attr_text)
BT_NUS_AVRCP_TG_BUF_REQ(get_player_app_setting_val_text)
BT_NUS_AVRCP_TG_BUF_REQ(inform_displayable_char_set)
BT_NUS_AVRCP_TG_BUF_REQ(get_element_attrs)
BT_NUS_AVRCP_TG_TID_REQ(get_play_status)
BT_NUS_AVRCP_TG_BUF_REQ(play_item)
BT_NUS_AVRCP_TG_BUF_REQ(add_to_now_playing)

static struct bt_avrcp_tg_cb bt_nus_avrcp_tg_cb =
{
        .connected = bt_nus_avrcp_tg_connected,
        .disconnected = bt_nus_avrcp_tg_disconnected,
        .unit_info_req = bt_nus_avrcp_tg_unit_info_req,
        .register_notification = bt_nus_avrcp_tg_register_notification,
        .subunit_info_req = bt_nus_avrcp_tg_subunit_info_req,
        .get_caps = bt_nus_avrcp_tg_get_caps,
        .browsing_connected = bt_nus_avrcp_tg_browsing_connected,
        .browsing_disconnected = bt_nus_avrcp_tg_browsing_disconnected,
        .set_browsed_player = bt_nus_avrcp_tg_set_browsed_player,
        .get_folder_items = bt_nus_avrcp_tg_get_folder_items,
        .change_path = bt_nus_avrcp_tg_change_path,
        .get_item_attrs = bt_nus_avrcp_tg_get_item_attrs,
        .get_total_number_of_items =
          bt_nus_avrcp_tg_get_total_number_of_items,
        .search = bt_nus_avrcp_tg_search,
        .passthrough_req = bt_nus_avrcp_tg_passthrough_req,
        .list_player_app_setting_attrs =
          bt_nus_avrcp_tg_list_player_app_setting_attrs,
        .list_player_app_setting_vals =
          bt_nus_avrcp_tg_list_player_app_setting_vals,
        .get_curr_player_app_setting_val =
          bt_nus_avrcp_tg_get_curr_player_app_setting_val,
        .set_player_app_setting_val =
          bt_nus_avrcp_tg_set_player_app_setting_val,
        .get_player_app_setting_attr_text =
          bt_nus_avrcp_tg_get_player_app_setting_attr_text,
        .get_player_app_setting_val_text =
          bt_nus_avrcp_tg_get_player_app_setting_val_text,
        .inform_displayable_char_set =
          bt_nus_avrcp_tg_inform_displayable_char_set,
        .inform_batt_status_of_ct =
          bt_nus_avrcp_tg_inform_batt_status_of_ct,
        .set_absolute_volume = bt_nus_avrcp_tg_set_absolute_volume,
        .get_element_attrs = bt_nus_avrcp_tg_get_element_attrs,
        .get_play_status = bt_nus_avrcp_tg_get_play_status,
        .set_addressed_player = bt_nus_avrcp_tg_set_addressed_player,
        .play_item = bt_nus_avrcp_tg_play_item,
        .add_to_now_playing = bt_nus_avrcp_tg_add_to_now_playing,
};

static int bt_nus_avrcp_tg_register(void)
{
  int err;

  err = bt_avrcp_tg_register_cb(&bt_nus_avrcp_tg_cb);
  if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Failed to register AVRCP TG callbacks (err %d)", err);
      return err;
    }

  LOG_INF("AVRCP TG callbacks registered");
  return 0;
}
#endif /* CONFIG_BT_AVRCP_TARGET */
#endif /* CONFIG_BT_AVRCP */

static enum bt_br_conn_req_rsp bt_nus_br_conn_req(const bt_addr_t *addr,
                                                  uint32_t cod)
{
  char addr_str[BT_ADDR_STR_LEN];

  bt_addr_to_str(addr, addr_str, sizeof(addr_str));
  LOG_INF("BR/EDR conn req: addr %s cod "
          "0x%06x", addr_str, (unsigned int)cod);

  return BT_BR_CONN_REQ_ACCEPT_PERIPHERAL;
}

static void bt_nus_connected(struct bt_conn *conn, uint8_t err)
{
  if (!bt_conn_is_type(conn, BT_CONN_TYPE_BR))
    {
      return;
    }

  if (err)
    {
      LOG_WRN("BR/EDR connection failed: %s err 0x%02x",
              bt_conn_dst_str(conn),
              err);
      return;
    }

  LOG_INF("BR/EDR connected: %s", bt_conn_dst_str(conn));
}

static void bt_nus_disconnected(struct bt_conn *conn, uint8_t reason)
{
  if (!bt_conn_is_type(conn, BT_CONN_TYPE_BR))
    {
      return;
    }

  LOG_INF("BR/EDR disconnected: %s reason 0x%02x", bt_conn_dst_str(conn),
          reason);
}

static struct bt_conn_cb bt_nus_conn_callbacks =
{
        .connected = bt_nus_connected,
        .disconnected = bt_nus_disconnected,
};
#endif

#if defined(CONFIG_BT_BROADCASTER)
static int bt_nus_start_le_adv(const char *reason);
#endif

static void bt_nus_le_connected(struct bt_conn *conn, uint8_t err)
{
  if (!bt_conn_is_type(conn, BT_CONN_TYPE_LE))
    {
      return;
    }

  if (err)
    {
      LOG_WRN("LE connection failed: %s err 0x%02x", bt_conn_dst_str(conn),
              err);
      return;
    }

  LOG_INF("LE connected: %s", bt_conn_dst_str(conn));
}

static void bt_nus_le_disconnected(struct bt_conn *conn, uint8_t reason)
{
  if (!bt_conn_is_type(conn, BT_CONN_TYPE_LE))
    {
      return;
    }

  LOG_INF("LE disconnected: %s reason 0x%02x", bt_conn_dst_str(conn),
          reason);

#if defined(CONFIG_BT_BROADCASTER)
  bt_nus_start_le_adv("LE disconnected");
#endif
}

static void bt_nus_le_recycled(void)
{
  LOG_INF("LE connection object recycled");

#if defined(CONFIG_BT_BROADCASTER)
  bt_nus_start_le_adv("connection recycled");
#endif
}

static struct bt_conn_cb bt_nus_le_conn_callbacks =
{
        .connected = bt_nus_le_connected,
        .disconnected = bt_nus_le_disconnected,
        .recycled = bt_nus_le_recycled,
};

#if defined(CONFIG_BT_BROADCASTER)
static int bt_nus_start_le_adv(const char *reason)
{
  int err;

  err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
          ARRAY_SIZE(sd));
  if (err < 0 && err != -EALREADY)
    {
      LOG_WRN("Failed to start LE advertising after %s (err %d)", reason,
              err);
      return err;
    }

  LOG_INF("LE advertising active after %s", reason);
  return 0;
}
#endif

static int bt_nus_auto_start(void)
{
  int err;

  err = bt_enable(NULL);
  DEBUGASSERT(!err);

  err = bt_nus_update_device_name();
  DEBUGASSERT(!err);

#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
  err = bt_nus_auth_register();
  DEBUGASSERT(!err);
#endif

  err = bt_conn_cb_register(&bt_nus_le_conn_callbacks);
  DEBUGASSERT(!err);

#if defined(CONFIG_BT_BROADCASTER)
  err = bt_nus_start_le_adv("Bluetooth enable");
  DEBUGASSERT(!err);
#endif

#ifdef CONFIG_BT_CLASSIC
  err = bt_conn_cb_register(&bt_nus_conn_callbacks);
  DEBUGASSERT(!err);

#if defined(CONFIG_BT_A2DP_SINK)
  err = bt_nus_a2dp_sink_register();
  DEBUGASSERT(!err);
#endif

#if defined(CONFIG_BT_AVRCP_CONTROLLER)
  err = bt_nus_avrcp_ct_register();
  DEBUGASSERT(!err);
#endif

#if defined(CONFIG_BT_AVRCP_TARGET)
  err = bt_nus_avrcp_tg_register();
  DEBUGASSERT(!err);
#endif

  err = bt_br_set_connectable(true, bt_nus_br_conn_req);
  DEBUGASSERT(!err);

  err = bt_br_set_class_of_device(CONFIG_BT_COD);
  DEBUGASSERT(!err);

  err = bt_br_write_eir(eir, ARRAY_SIZE(eir), true);
  DEBUGASSERT(!err);

  err = bt_br_set_discoverable(true, false);
  DEBUGASSERT(!err);
#endif

  return 0;
}

SYS_INIT(bt_nus_auto_start, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
