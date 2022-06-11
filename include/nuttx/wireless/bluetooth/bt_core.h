/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_core.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_CORE_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <string.h>

#include <nuttx/wireless/bluetooth/bt_buf.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BT_ADDR_STR_LEN
 * Recommended length of user string buffer for Bluetooth address
 *
 * The recommended length guarantee the output of address conversion will not
 * lose valuable information about address being processed.
 */

#define BT_ADDR_STR_LEN 18

/* BT_ADDR_LE_STR_LEN
 * Recommended length of user string buffer for Bluetooth LE address
 *
 * The recommended length guarantee the output of address conversion will not
 * lose valuable information about address being processed.
 */

#define BT_ADDR_LE_STR_LEN 27

/* BT_LE162HOST
 * Convert 16-bit integer from little-endian to host endianness.
 */

#ifdef CONFIG_ENDIAN_BIG
#  define BT_LE162HOST(le) \
     ((((uint16_t)(le) >> 8) & 0xff) | (((uint16_t)(le) & 0xff) << 8))
#else
#  define BT_LE162HOST(le) (le)
#endif

/* BT_HOST2LE16
 * Convert 16-bit integer from host endianness to little-endian.
 */

#ifdef CONFIG_ENDIAN_BIG
#  define BT_HOST2LE16(h) \
     ((((uint16_t)(h) >> 8) & 0xff) | (((uint16_t)(h) & 0xff) << 8))
#else
#  define BT_HOST2LE16(h) (h)
#endif

/* Unaligned access */

#ifdef CONFIG_ENDIAN_BIG
#  define BT_GETUINT16(p) \
     (((uint16_t)(((FAR uint8_t *)(p))[1])) | \
      (((uint16_t)(((FAR uint8_t *)(p))[0])) << 8))
#  define BT_PUTUINT16(p,v) \
     do \
       { \
         ((FAR uint8_t *)(p))[0] = ((uint16_t)(v)) >> 8; \
         ((FAR uint8_t *)(p))[1] = ((uint16_t)(v)) & 0xff; \
       } \
     while (0)
#else
#  define BT_GETUINT16(p) \
     (((uint16_t)(((FAR uint8_t *)(p))[0])) | \
      (((uint16_t)(((FAR uint8_t *)(p))[1])) << 8))
#  define BT_PUTUINT16(p,v) \
     do \
       { \
         ((FAR uint8_t *)(p))[0] = ((uint16_t)(v)) & 0xff; \
         ((FAR uint8_t *)(p))[1] = ((uint16_t)(v)) >> 8; \
       } \
     while (0)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Advertising API */

begin_packed_struct struct bt_eir_s
{
  uint8_t len;
  uint8_t type;
  uint8_t data[29];
} end_packed_struct;

/* Security level */

enum bt_security_e
{
  BT_SECURITY_LOW,    /* No encryption and no authentication. */
  BT_SECURITY_MEDIUM, /* encryption and no authentication (no MITM). */
  BT_SECURITY_HIGH,   /* encryption and authentication (MITM). */
  BT_SECURITY_FIPS,   /* Authenticated LE Secure Connections and
                       * encryption. */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_addr_to_str
 *
 * Description:
 *   Converts binary Bluetooth address to string.
 *
 * Input Parameters:
 *   addr - Address of buffer containing binary Bluetooth address.
 *   str  - Address of user buffer with enough room to store formatted
 *          string containing binary address.
 *   len  - Length of data to be copied to user string buffer. Refer to
 *          BT_ADDR_STR_LEN about recommended value.
 *
 * Returned Value:
 *   Number of successfully formatted bytes from binary address.
 *
 ****************************************************************************/

static inline int bt_addr_to_str(FAR const bt_addr_t *addr, FAR char *str,
                                 size_t len)
{
  return snprintf(str, len, "%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X",
                  addr->val[5], addr->val[4], addr->val[3],
                  addr->val[2], addr->val[1], addr->val[0]);
}

/****************************************************************************
 * Name: bt_addr_le_to_str
 *
 * Description:
 *   Converts binary LE Bluetooth address to string.
 *
 * Input Parameters:
 *   addr     - Address of buffer containing binary LE Bluetooth address.
 *   user_buf - Address of user buffer with enough room to store
 *              formatted string containing binary LE address.
 *   len      - Length of data to be copied to user string buffer. Refer to
 *              BT_ADDR_LE_STR_LEN about recommended value.
 *
 * Returned Value:
 *   Number of successfully formatted bytes from binary address.
 *
 ****************************************************************************/

static inline int bt_addr_le_to_str(FAR const bt_addr_le_t *addr, char *str,
                                    size_t len)
{
  char type[7];

  switch (addr->type)
  {
    case BT_ADDR_LE_PUBLIC:
      strcpy(type, "public");
      break;

    case BT_ADDR_LE_RANDOM:
      strcpy(type, "random");
      break;

    default:
      sprintf(type, "0x%02x", addr->type);
      break;
  }

  return snprintf(str, len, "%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X (%s)",
                  addr->val[5], addr->val[4], addr->val[3],
                  addr->val[2], addr->val[1], addr->val[0], type);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_CORE_H */
