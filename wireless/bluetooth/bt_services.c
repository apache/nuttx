/****************************************************************************
 * wireless/bluetooth/bt_services.c
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <debug.h>
#include <arpa/inet.h>
#include <sys/param.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_uuid.h>
#include <nuttx/wireless/bluetooth/bt_gatt.h>

#include "bt_hcicore.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GASVC                     0x0001
#define NAME_CHR                  (GASVC + 0x0002)
#define NAME_DSC                  (GASVC + 0x0003)
#define APPEARANCE_CHR            (GASVC + 0x0004)
#define APPEARANCE_DSC            (GASVC + 0x0005)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int read_appearance(FAR struct bt_conn_s *conn,
                           FAR const struct bt_gatt_attr_s *attr,
                           FAR void *buf,
                           uint8_t len,
                           uint16_t offset);

static int read_name(FAR struct bt_conn_s *conn,
                     FAR const struct bt_gatt_attr_s *attr,
                     FAR void *buf,
                     uint8_t len,
                     uint16_t offset);

static struct bt_uuid_s g_gap_uuid =
{
  .type  = BT_UUID_16,
  .u     =
  {
    .u16 = BT_UUID_GAP
  },
};

static struct bt_uuid_s g_device_name_uuid =
{
  .type  = BT_UUID_16,
  .u     =
  {
    .u16 = BT_UUID_GAP_DEVICE_NAME,
  }
};

static struct bt_gatt_chrc_s g_name_chrc =
{
  .properties   = BT_GATT_CHRC_READ,
  .value_handle = NAME_DSC,
  .uuid         = &g_device_name_uuid,
};

static struct bt_uuid_s g_appeareance_uuid =
{
  .type  = BT_UUID_16,
  .u     =
  {
    .u16 = BT_UUID_GAP_APPEARANCE,
  }
};

static struct bt_gatt_chrc_s g_appearance_chrc =
{
  .properties   = BT_GATT_CHRC_READ,
  .value_handle = APPEARANCE_DSC,
  .uuid         = &g_appeareance_uuid,
};

static const struct bt_gatt_attr_s g_attrs[] =
{
    BT_GATT_PRIMARY_SERVICE(GASVC, &g_gap_uuid),
    BT_GATT_CHARACTERISTIC(NAME_CHR, &g_name_chrc),
    BT_GATT_DESCRIPTOR(NAME_DSC, &g_device_name_uuid, BT_GATT_PERM_READ,
                       read_name, NULL, (FAR void *)CONFIG_DEVICE_NAME),
    BT_GATT_CHARACTERISTIC(APPEARANCE_CHR, &g_appearance_chrc),
    BT_GATT_DESCRIPTOR(APPEARANCE_DSC, &g_appeareance_uuid,
                       BT_GATT_PERM_READ, read_appearance, NULL, NULL),
};

static const struct bt_eir_s g_ad[] =
{
  {
    .len  = 2,
    .type = BT_EIR_FLAGS,
    .data =
    {
      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR
    },
  },
  {
    .len  = 0,
    .type = 0,
    .data = "",
  }
};

static const struct bt_eir_s g_sd[] =
{
  {
    .len  = 16,
    .type = BT_EIR_NAME_COMPLETE,
    .data = CONFIG_DEVICE_LOCAL_NAME,
  },
  {
    .len  = 0,
    .type = 0,
    .data = "",
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read_name
 *
 * Description:
 *   Read callback for name descriptor.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

static int read_name(FAR struct bt_conn_s *conn,
                     FAR const struct bt_gatt_attr_s *attr,
                     FAR void *buf,
                     uint8_t len,
                     uint16_t offset)
{
  FAR const char *name = attr->user_data;

  return bt_gatt_attr_read(conn, attr, buf, len, offset, name,
                           strlen(name));
}

/****************************************************************************
 * Name: read_appearance
 *
 * Description:
 *   Read callback for appearance descriptor.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

static int read_appearance(FAR struct bt_conn_s *conn,
                           FAR const struct bt_gatt_attr_s *attr,
                           FAR void *buf,
                           uint8_t len,
                           uint16_t offset)
{
  uint16_t appearance = htole16(CONFIG_DEVICE_APPEARANCE);

  return bt_gatt_attr_read(conn, attr, buf, len, offset, &appearance,
                           sizeof(appearance));
}

/****************************************************************************
 * Name: bt_add_services
 *
 * Description:
 *   Register services and start advertising.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

int bt_add_services(void)
{
  int err;

  bt_gatt_register(g_attrs, nitems(g_attrs));

  err = bt_start_advertising(BT_LE_ADV_IND, g_ad, g_sd);
  if (err)
    {
      wlerr("Advertising failed to start (err %d)\n", err);
    }

  return err;
}
