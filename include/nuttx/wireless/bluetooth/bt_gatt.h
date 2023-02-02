/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_gatt.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_GATT_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_GATT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bluetooth/bt_uuid.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GATT attribute permission bitfield values */

/* BT_GATT_PERM_READ
 * Attribute read permission.
 */

#define BT_GATT_PERM_READ           0x01

/* BT_GATT_PERM_WRITE
 * Attribute write permission.
 */

#define BT_GATT_PERM_WRITE          0x02

/* BT_GATT_PERM_READ_ENCRYPT
 * Attribute read permission with encryption.
 * If set, requires encryption for read access.
 */

#define BT_GATT_PERM_READ_ENCRYPT   0x04

/* BT_GATT_PERM_WRITE_ENCRYPT
 * Attribute write permission with encryption.
 * If set, requires encryption for write access.
 */

#define BT_GATT_PERM_WRITE_ENCRYPT  0x08

/* BT_GATT_PERM_READ_AUTHEN
 * Attribute read permission with authentication.
 * If set, requires encryption using authenticated link-key for read access.
 */

#define BT_GATT_PERM_READ_AUTHEN    0x10

/* BT_GATT_PERM_WRITE_AUTHEN
 * Attribute write permission with authentication.
 * If set, requires encryption using authenticated link-key for write
 * access.
 */

#define BT_GATT_PERM_WRITE_AUTHEN   0x20

/* BT_GATT_PERM_AUTHOR
 * Attribute authorization permission.
 */

#define BT_GATT_PERM_AUTHOR         0x40

/* GATT attribute flush flags */

/* BT_GATT_FLUSH_DISCARD
 * Attribute flush discard flag.
 */

#define BT_GATT_FLUSH_DISCARD       0x00

/* BT_GATT_FLUSH_DISCARD
 * Attribute flush synchronize flag.
 */

#define BT_GATT_FLUSH_SYNC          0x01

/* Characteristic Properties Bitfield values */

/* BT_GATT_CHRC_BROADCAST
 * Characteristic broadcast property.
 * If set, permits broadcasts of the Characteristic Value using Server
 * Characteristic Configuration Descriptor.
 */

#define BT_GATT_CHRC_BROADCAST      0x01

/* BT_GATT_CHRC_READ
 * Characteristic read property.
 * If set, permits reads of the Characteristic Value.
 */

#define BT_GATT_CHRC_READ           0x02

/* BT_GATT_CHRC_WRITE_WITHOUT_RESP
 * Characteristic write without response property.
 * If set, permits write of the Characteristic Value without response.
 */

#define BT_GATT_CHRC_WRITE_WITHOUT_RESP 0x04

/* BT_GATT_CHRC_WRITE
 * Characteristic write with response property.
 * If set, permits write of the Characteristic Value with response.
 */

#define BT_GATT_CHRC_WRITE          0x08

/* BT_GATT_CHRC_NOTIFY
 * Characteristic notify property.
 * If set, permits notifications of a Characteristic Value without
 * acknowledgment.
 */

#define BT_GATT_CHRC_NOTIFY         0x10

/* BT_GATT_CHRC_INDICATE
 * Characteristic indicate property.
 * If set, permits indications of a Characteristic Value with
 * acknowledgment.
 */

#define BT_GATT_CHRC_INDICATE       0x20

/* BT_GATT_CHRC_AUTH
 * Characteristic Authenticated Signed Writes property.
 * If set, permits signed writes to the Characteristic Value.
 */

#define BT_GATT_CHRC_AUTH           0x40

/* BT_GATT_CHRC_EXT_PROP
 * Characteristic Extended Properties property.
 * If set, additional characteristic properties are defined in the
 * Characteristic Extended Properties Descriptor.
 */

#define BT_GATT_CHRC_EXT_PROP       0x80

/* Characteristic Extended Properties Bitfield values */

#define BT_GATT_CEP_RELIABLE_WRITE  0x0001
#define BT_GATT_CEP_WRITABLE_AUX    0x0002

/* Client Characteristic Configuration Values */

/* BT_GATT_CCC_NOTIFY
 * Client Characteristic Configuration Notification.
 * If set, changes to Characteristic Value shall be notified.
 */

#define BT_GATT_CCC_NOTIFY          0x0001

/* BT_GATT_CCC_INDICATE
 * Client Characteristic Configuration Indication.
 * If set, changes to Characteristic Value shall be indicated.
 */

#define BT_GATT_CCC_INDICATE        0x0002

/* BT_GATT_SERVICE
 * Helper macro to declare a service attribute.
 *
 * Arguments:
 *   _handle - Service attribute handle.
 *   _uuid   - Service attribute type.
 *   _data   - Service attribute value.
 */

#define BT_GATT_SERVICE(_handle, _uuid, _service) \
{ \
  .handle    = _handle, \
  .uuid      = _uuid, \
  .perm      = BT_GATT_PERM_READ, \
  .read      = bt_gatt_attr_read_service, \
  .user_data = _service, \
}

/* BT_GATT_PRIMARY_SERVICE
 * Helper macro to declare a primary service attribute.
 *
 * Arguments:
 *   _handle  - Service attribute handle.
 *   _service - Service attribute value.
 */

#define BT_GATT_PRIMARY_SERVICE(_handle, _service) \
{ \
  .handle    = _handle, \
  .uuid      = (&(struct bt_uuid_s) \
               { \
                 BT_UUID_16, \
                 { \
                   BT_UUID_GATT_PRIMARY \
                 } \
               }), \
  .perm      = BT_GATT_PERM_READ, \
  .read      = bt_gatt_attr_read_service, \
  .user_data = _service, \
}

/* BT_GATT_SECONDARY_SERVICE
 * Helper macro to declare a secondary service attribute.
 *
 * Arguments:
 *   _handle  - Service attribute handle.
 *   _service - Service attribute value.
 */

#define BT_GATT_SECONDARY_SERVICE(_handle, _service) \
{ \
  .handle    = _handle, \
  .uuid      = (&(struct bt_uuid_s) \
               { \
                 BT_UUID_16, \
                 { \
                   BT_UUID_GATT_SECONDARY \
                 } \
               }), \
  .perm      = BT_GATT_PERM_READ, \
  .read      = bt_gatt_attr_read_service, \
  .user_data = _service, \
}

/* BT_GATT_INCLUDE_SERVICE
 * Helper macro to declare a include service attribute.
 *
 * Arguments:
 *   _handle Service attribute handle.
 *   _service Service attribute value.
 */

#define BT_GATT_INCLUDE_SERVICE(_handle, _service) \
{ \
  .handle    = _handle, \
  .uuid      = (&(struct bt_uuid_s) \
               { \
                 BT_UUID_16, \
                 { \
                   BT_UUID_GATT_INCLUDE \
                 } \
               }), \
  .perm      = BT_GATT_PERM_READ, \
  .read      = bt_gatt_attr_read_included, \
  .user_data = _service, \
}

/* BT_GATT_CHARACTERISTIC
 * Helper macro to declare a characteristic attribute.
 *
 * Arguments:
 *   _handle - Characteristic attribute handle.
 *   _value  - Characteristic attribute value.
 */

#define BT_GATT_CHARACTERISTIC(_handle, _value) \
{ \
  .handle    = _handle, \
  .uuid      = (&(struct bt_uuid_s) \
               { \
                 BT_UUID_16, \
                 { \
                   BT_UUID_GATT_CHRC \
                 } \
               }), \
  .perm      = BT_GATT_PERM_READ, \
  .read      = bt_gatt_attr_read_chrc, \
  .user_data = _value, \
}

/* BT_GATT_CCC
 * Helper macro to declare a CCC attribute.
 *
 * Arguments:
 *   _handle       - Descriptor attribute handle.
 *   _value_handle - Characteristic attribute value handle.
 *   _cfg          - Initial configuration.
 *   _cfg_changed  - Configuration changed callback.
 */

#define BT_GATT_CCC(_handle, _value_handle, _cfg, _cfg_changed) \
{ \
  .handle    = _handle, \
  .uuid      = (&(struct bt_uuid_s) \
               { \
                 BT_UUID_16, \
                 { \
                   BT_UUID_GATT_CCC \
                 } \
               }), \
  .perm      = BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, \
  .read      = bt_gatt_attr_read_ccc, \
  .write     = bt_gatt_attr_write_ccc, \
  .user_data = (&(struct _bt_gatt_ccc_s) \
               { \
                 .cfg          = _cfg, \
                 .cfg_len      = nitems(_cfg), \
                 .value_handle = _value_handle, \
                 .cfg_changed  = _cfg_changed, \
               }),\
}

/* BT_GATT_CEP
 * Helper macro to declare a CEP attribute.
 *
 * Arguments:
 *   _handle Descriptor attribute handle.
 *   _value Descriptor attribute value.
 */

#define BT_GATT_CEP(_handle, _value) \
{ \
  .handle    = _handle, \
  .uuid      = (&(struct bt_uuid_s) \
               { \
                 BT_UUID_16, \
                 { \
                   BT_UUID_GATT_CEP \
                 } \
               }), \
  .perm      = BT_GATT_PERM_READ, \
  .read      = bt_gatt_attr_read_cep, \
  .user_data = _value, \
}

/* BT_GATT_DESCRIPTOR
 *  Helper macro to declare a descriptor attribute.
 *
 * Arguments:
 *   _handle - Descriptor attribute handle.
 *   _value  - Descriptor attribute value.
 *   _perm   - Descriptor attribute access permissions.
 *   _read   - Descriptor attribute read callback.
 *   _write  - Descriptor attribute write callback.
 *   _value  - Descriptor attribute value.
 */

#define BT_GATT_DESCRIPTOR(_handle, _uuid, _perm, _read, _write, _value) \
{ \
  .handle    = _handle, \
  .uuid      = _uuid, \
  .perm      = _perm, \
  .read      = _read, \
  .write     = _write, \
  .user_data = _value, \
}

/* BT_GATT_LONG_DESCRIPTOR
 *  Helper macro to declare a descriptor attribute.
 *
 * Arguments:
 *   _handle - Descriptor attribute handle.
 *   _value  - Descriptor attribute value.
 *   _perm   - Descriptor attribute access permissions.
 *   _read   - Descriptor attribute read callback.
 *   _write  - Descriptor attribute write callback.
 *   _flush  - Descriptor attribute flush callback.
 *   _value  - Descriptor attribute value.
 */

#define BT_GATT_LONG_DESCRIPTOR(_handle, _uuid, _perm, _read, _write, _flush, \
        _value) \
{ \
  .handle    = _handle, \
  .uuid      = _uuid, \
  .perm      = _perm, \
  .read      = _read, \
  .write     = _write, \
  .flush     = _flush, \
  .user_data = _value, \
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum bt_gatt_iter_e
{
  BT_GATT_ITER_STOP = 0,
  BT_GATT_ITER_CONTINUE,
};

/* Attribute iterator callback.
 *
 * Input Parameters:
 *   attr      - Attribute found.
 *   user_data - Data given.
 *
 * Returned Value:
 *   BT_GATT_ITER_CONTINUE if should continue to the next attribute
 *   or BT_GATT_ITER_STOP to stop.
 */

struct bt_gatt_attr_s; /* Forward reference */

typedef CODE uint8_t
  (*bt_gatt_attr_func_t)(FAR const struct bt_gatt_attr_s *attr,
                         FAR void *user_data);

/* Response callback function
 *
 * Input Parameters:
 *   conn - Connection object.
 *   err  - Error code.
 */

struct bt_conn_s; /* Forward reference */

typedef CODE void (*bt_gatt_rsp_func_t)(FAR struct bt_conn_s *conn,
                   uint8_t err);

/* Read callback function
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   err    - Error code.
 *   data   - Attribute value data.
 *   length - Attribute value length.
 */

typedef CODE void (*bt_gatt_read_func_t)(FAR struct bt_conn_s *conn,
                                         int err, FAR const void *data,
                                         uint16_t length);

/* GATT Attribute structure. */

struct bt_gatt_attr_s
{
  /* Attribute UUID */

  FAR const struct bt_uuid_s  *uuid;

  /* Attribute read callback */

  CODE int (*read)(FAR struct bt_conn_s *conn,
                   FAR const struct bt_gatt_attr_s *attr, FAR void *buf,
                   uint8_t len, uint16_t offset);

  /* Attribute write callback */

  CODE int (*write)(FAR struct bt_conn_s *conn,
                    FAR const struct bt_gatt_attr_s *attr,
                    FAR const void *buf, uint8_t len, uint16_t offset);

  /* Attribute flush callback */

  CODE int (*flush)(FAR struct bt_conn_s *conn,
                    FAR const struct bt_gatt_attr_s *attr,
                    uint8_t flags);

  /* Attribute user data */

  FAR void *user_data;

  /* Attribute handle */

  uint16_t handle;

  /* Attribute permissions */

  uint8_t perm;
};

/* Service Attribute Value. */

struct bt_gatt_service_s
{
  /* Service UUID. */

  FAR const struct bt_uuid_s *uuid;
};

/* Include Attribute Value. */

struct bt_gatt_include_s
{
  /* Service UUID. */

  FAR const struct bt_uuid_s *uuid;

  /* Service start handle. */

  uint16_t start_handle;

  /* Service end handle. */

  uint16_t end_handle;
};

/* Characteristic Attribute Value. */

struct bt_gatt_chrc_s
{
  /* Characteristic UUID. */

  FAR const struct bt_uuid_s *uuid;

  /* Characteristic value handle. */

  uint16_t value_handle;

  /* Characteristic properties. */

  uint8_t properties;
};

/* Characteristic Extended Properties Attribute Value. */

struct bt_gatt_cep_s
{
  /* Characteristic Extended properties */

  uint16_t properties;
};

/* Characteristic User Description Attribute Value. */

struct bt_gatt_cud_s
{
  /* Characteristic User Description string. */

  FAR char *string;
};

/* Client Characteristic Configuration Attribute Value */

struct bt_gatt_ccc_s
{
  /* Client Characteristic Configuration flags */

  uint16_t flags;
};

/* GATT CCC configuration entry. */

struct bt_gatt_ccc_cfg_s
{
  /* Config peer address. */

  bt_addr_le_t peer;

  /* Config peer value. */

  uint16_t value;

  /* Config valid flag. */

  uint8_t valid;
};

/* Internal representation of CCC value */

struct _bt_gatt_ccc_s
{
  FAR struct bt_gatt_ccc_cfg_s *cfg;
  size_t cfg_len;
  uint16_t value;
  uint16_t value_handle;
  CODE void (*cfg_changed)(uint16_t value);
};

/* GATT Discover Primary parameters */

struct bt_gatt_discover_params_s
{
  /* Discover UUID type */

  FAR struct bt_uuid_s *uuid;

  /* Discover attribute callback */

  bt_gatt_attr_func_t func;

  /* Discover destroy callback */

  CODE void (*destroy)(FAR void *user_data);

  /* Discover start handle */

  uint16_t start_handle;

  /* Discover end handle */

  uint16_t end_handle;

  /* Private data */

  FAR void *p_data;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Server API */

/****************************************************************************
 * Name: bt_gatt_register
 *
 * Description:
 *   Register GATT attribute database table. Applications can make use of
 *   macros such as BT_GATT_PRIMARY_SERVICE, BT_GATT_CHARACTERISTIC,
 *   BT_GATT_DESCRIPTOR, etc.
 *
 * Input Parameters:
 *   attrs - Database table containing the available attributes.
 *   count - Size of the database table.
 *
 ****************************************************************************/

void bt_gatt_register(FAR const struct bt_gatt_attr_s *attrs, size_t count);

/****************************************************************************
 * Name: bt_gatt_foreach_attr
 *
 * Description:
 *   Iterate attributes in the given range.
 *
 * Input Parameters:
 *   start_handle - Start handle.
 *   end_handle   - End handle.
 *   func         - Callback function.
 *   user_data    - Data to pass to the callback.
 *
 ****************************************************************************/

void bt_gatt_foreach_attr(uint16_t start_handle, uint16_t end_handle,
                          bt_gatt_attr_func_t func, FAR void *user_data);

/****************************************************************************
 * Name: bt_gatt_attr_read
 *
 * Description:
 *   Read attribute value storing the result into buffer.
 *
 * Input Parameters:
 *   conn      - Connection object.
 *   attr      - Attribute to read.
 *   buf       - Buffer to store the value.
 *   buf_len   - Buffer length.
 *   offset    - Start offset.
 *   value     - Attribute value.
 *   value_len - Length of the attribute value.
 *
 * Returned Value:
 *   int number of bytes read in case of success or negative values in
 *  case of error.
 *
 ****************************************************************************/

int bt_gatt_attr_read(FAR struct bt_conn_s *conn,
                      FAR const struct bt_gatt_attr_s *attr,
                      FAR void *buf, uint8_t buf_len, uint16_t offset,
                      FAR const void *value, uint8_t value_len);

/****************************************************************************
 * Name: bt_gatt_attr_read_service
 *
 * Description:
 *   Read service attribute value storing the result into buffer after
 *   encoding it.
 *   NOTE: Only use this with attributes which user_data is a bt_uuid_s.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   attr   - Attribute to read.
 *   buf    - Buffer to store the value read.
 *   len    - Buffer length.
 *   offset - Start offset.
 *
 * Returned Value:
 *   int number of bytes read in case of success or negative values in
 *  case of error.
 *
 ****************************************************************************/

int bt_gatt_attr_read_service(FAR struct bt_conn_s *conn,
                              FAR const struct bt_gatt_attr_s *attr,
                              FAR void *buf, uint8_t len, uint16_t offset);

/****************************************************************************
 * Name: bt_gatt_attr_read_included
 *
 * Description:
 *   Read include service attribute value storing the result into buffer
 *   after encoding it.
 *   NOTE: Only use this with attributes which user_data is a
 *   bt_gatt_include.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   attr   - Attribute to read.
 *   buf    - Buffer to store the value read.
 *   len    - Buffer length.
 *   offset - Start offset.
 *
 * Returned Value:
 *   int number of bytes read in case of success or negative values in
 *  case of error.
 *
 ****************************************************************************/

int bt_gatt_attr_read_included(FAR struct bt_conn_s *conn,
                               FAR const struct bt_gatt_attr_s *attr,
                               FAR void *buf, uint8_t len, uint16_t offset);

/****************************************************************************
 * Name: bt_gatt_attr_read_chrc
 *
 * Description:
 *   Read characteristic attribute value storing the result into buffer after
 *    encoding it.
 *  NOTE: Only use this with attributes which user_data is a bt_gatt_chrc.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   attr   - Attribute to read.
 *   buf    - Buffer to store the value read.
 *   len    - Buffer length.
 *   offset - Start offset.
 *
 * Returned Value:
 *   number of bytes read in case of success or negative values in
 *  case of error.
 *
 ****************************************************************************/

int bt_gatt_attr_read_chrc(FAR struct bt_conn_s *conn,
                           FAR const struct bt_gatt_attr_s *attr,
                           FAR void *buf, uint8_t len, uint16_t offset);

/****************************************************************************
 * Name: bt_gatt_attr_read_ccc
 *
 * Description:
 *   Read CCC attribute value storing the result into buffer after
 *   encoding it.
 *   NOTE: Only use this with attributes which user_data is a _bt_gatt_ccc_s.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   attr   - Attribute to read.
 *   buf    - Buffer to store the value read.
 *   len    - Buffer length.
 *   offset - Start offset.
 *
 * Returned Value:
 *   number of bytes read in case of success or negative values in
 *  case of error.
 *
 ****************************************************************************/

int bt_gatt_attr_read_ccc(FAR struct bt_conn_s *conn,
                          FAR const struct bt_gatt_attr_s *attr,
                          FAR void *buf, uint8_t len, uint16_t offset);

/****************************************************************************
 * Name: bt_gatt_attr_write_ccc
 *
 * Description:
 *   Write value in the buffer into CCC attribute.
 *   NOTE: Only use this with attributes which user_data is a _bt_gatt_ccc_s.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   attr   - Attribute to read.
 *   buf    - Buffer to store the value read.
 *   len    - Buffer length.
 *   offset - Start offset.
 *
 * Returned Value:
 *   number of bytes written in case of success or negative values in
 *  case of error.
 *
 ****************************************************************************/

int bt_gatt_attr_write_ccc(FAR struct bt_conn_s *conn,
                           FAR const struct bt_gatt_attr_s *attr,
                           FAR const void *buf, uint8_t len,
                           uint16_t offset);

/****************************************************************************
 * Name: bt_gatt_attr_read_cep
 *
 * Description:
 *   Read CEP attribute value storing the result into buffer after
 *   encoding it.
 *   NOTE: Only use this with attributes which user_data is a bt_gatt_cep.
 *
 * Input Parameters:
 *   conn   - Connection object
 *   attr   - Attribute to read
 *   buf    - Buffer to store the value read
 *   len    - Buffer length
 *   offset - Start offset
 *
 * Returned Value:
 *   number of bytes read in case of success or negative values in
 *  case of error.
 *
 ****************************************************************************/

int bt_gatt_attr_read_cep(FAR struct bt_conn_s *conn,
                          FAR const struct bt_gatt_attr_s *attr,
                          FAR void *buf, uint8_t len, uint16_t offset);

/****************************************************************************
 * Name: bt_gatt_notify
 *
 * Description:
 *   Send notification of attribute value change.
 *   Note: This function should only be called if CCC is declared with
 *   BT_GATT_CCC otherwise it cannot find a valid peer configuration.
 *
 * Input Parameters:
 *   handle - Attribute handle.
 *   value  - Attribute value.
 *   len    - Attribute value length.
 *
 ****************************************************************************/

void bt_gatt_notify(uint16_t handle, FAR const void *data, size_t len);

/****************************************************************************
 * Name: bt_gatt_connected
 *
 * Description:
 *   Connected callback.
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 ****************************************************************************/

void bt_gatt_connected(FAR struct bt_conn_s *conn);

/****************************************************************************
 * Name: bt_gatt_disconnected
 *
 * Description:
 *   Disconnected callback.
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 ****************************************************************************/

void bt_gatt_disconnected(FAR struct bt_conn_s *conn);

/* Client API */

/****************************************************************************
 * Name: bt_gatt_exchange_mtu
 *
 * Description:
 *   This client procedure can be used to set the MTU to the maximum possible
 *   size the buffers can hold.
 *   NOTE: Can only be used once per connection.
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 ****************************************************************************/

int bt_gatt_exchange_mtu(FAR struct bt_conn_s *conn,
                         bt_gatt_rsp_func_t func);

/****************************************************************************
 * Name: bt_gatt_discover
 *
 * Description:
 *   This procedure is used by a client to discover a specific primary
 *   service on a server when only the Service UUID is known.
 *
 *   For each attribute found the callback is called which can then decide
 *   whether to continue discovering or stop.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   params - Discover parameters.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

int bt_gatt_discover(FAR struct bt_conn_s *conn,
                     FAR struct bt_gatt_discover_params_s *params);

/****************************************************************************
 * Name: bt_gatt_discover_characteristic
 *
 * Description:
 *   This procedure is used by a client to discover all characteristics on a
 *   server.
 *   Note: In case the UUID is set in the parameter it will be matched
 *   against the attributes found before calling the function callback.
 *
 *   For each attribute found the callback is called which can then decide
 *   whether to continue discovering or stop.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   params - Discover parameters.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

int bt_gatt_discover_characteristic(FAR struct bt_conn_s *conn,
                               FAR struct bt_gatt_discover_params_s *params);

/****************************************************************************
 * Name: bt_gatt_discover_descriptor
 *
 * Description:
 *   This procedure is used by a client to discover descriptors on a server.
 *   Note: In case the UUID is set in the parameter it will be matched
 *   against the attributes found before calling the function callback.
 *
 *   For each attribute found the callback is called which can then decide
 *   whether to continue discovering or stop.
 *
 * Input Parameters:
 *   conn   - Connection object.
 *   params - Discover parameters.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

int bt_gatt_discover_descriptor(FAR struct bt_conn_s *conn,
                               FAR struct bt_gatt_discover_params_s *params);

/****************************************************************************
 * Name: bt_gatt_read
 *
 * Description:
 *   This procedure read the attribute value and return it to the callback.
 *
 * Input Parameters:
 *  conn   - Connection object.
 *  handle - Attribute handle.
 *  offset - Attribute data offset.
 *  func   - Callback function.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

int bt_gatt_read(FAR struct bt_conn_s *conn, uint16_t handle,
                 uint16_t offset, bt_gatt_read_func_t func);

/****************************************************************************
 * Name: bt_gatt_write
 *
 * Description:
 *   This procedure write the attribute value and return the result in the
 *   callback in case it is set.
 *
 * Input Parameters:
 *  conn   - Connection object.
 *  handle - Attribute handle.
 *  data   - Data to be written.
 *  length - Data length.
 *  func   - Callback function.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

int bt_gatt_write(FAR struct bt_conn_s *conn, uint16_t handle,
                  FAR const void *data, uint16_t length,
                  bt_gatt_rsp_func_t func);

/****************************************************************************
 * Name: bt_gatt_cancel
 *
 * Description:
 *   Cancel GATT pending request
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 ****************************************************************************/

void bt_gatt_cancel(FAR struct bt_conn_s *conn);

/****************************************************************************
 * Name: bt_gatt_read_multiple
 *
 * Description:
 *   Routine to be used to retrieve set of attributes values determined by
 *   set of handles in one call.
 *
 * Input Parameters:
 *   conn    - Connection object.
 *   handles - Set of valid handles to attributes.
 *   count   - Number of handles to be read.
 *   func    - User callback routine to get retrieved values.
 *
 * Returned Value:
 *   Zero in case of success or negative value in case of error.
 *
 ****************************************************************************/

int bt_gatt_read_multiple(FAR struct bt_conn_s *conn,
                          FAR const uint16_t *handles, size_t count,
                          bt_gatt_read_func_t func);

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_GATT_H */
