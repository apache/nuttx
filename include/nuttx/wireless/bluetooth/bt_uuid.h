/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_uuid.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UUID_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UUID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BBT_UUID_GAP
 *  Generic Access
 */

#define BT_UUID_GAP                           0x1800

/* BBT_UUID_GATT
 *  Generic Attribute
 */

#define BT_UUID_GATT                          0x1801

/* BBT_UUID_CTS
 *  Current Time Service
 */

#define BT_UUID_CTS                           0x1805

/* BBT_UUID_DIS
 *  Device Information Service
 */

#define BT_UUID_DIS                           0x180a

/* BBT_UUID_HRS
 *  Heart Rate Service
 */

#define BT_UUID_HRS                           0x180d

/* BBT_UUID_BAS
 *  Battery Service
 */

#define BT_UUID_BAS                           0x180f

/* BT_UUID_GATT_PRIMARY
 *  GATT Primary Service
 */

#define BT_UUID_GATT_PRIMARY                  0x2800

/* BT_UUID_GATT_SECONDARY
 *  GATT Secondary Service
 */

#define BT_UUID_GATT_SECONDARY                0x2801

/* BT_UUID_GATT_INCLUDE
 *  GATT Include Service
 */

#define BT_UUID_GATT_INCLUDE                  0x2802

/* BT_UUID_GATT_CHRC
 *  GATT Characteristic
 */

#define BT_UUID_GATT_CHRC                     0x2803

/* BT_UUID_GATT_CEP
 *  GATT Characteristic Extended Properties
 */

#define BT_UUID_GATT_CEP                      0x2900

/* BT_UUID_GATT_CUD
 *  GATT Characteristic User Description
 */

#define BT_UUID_GATT_CUD                      0x2901

/* BT_UUID_GATT_CCC
 *  GATT Client Characteristic Configuration
 */

#define BT_UUID_GATT_CCC                      0x2902

/* BT_UUID_GAP_DEVICE_NAME
 *  GAP Characteristic Device Name
 */

#define BT_UUID_GAP_DEVICE_NAME               0x2a00

/* BT_UUID_GAP_APPEARANCE
 *  GAP Characteristic Appearance
 */

#define BT_UUID_GAP_APPEARANCE                0x2a01

/* BT_UUID_BAS_BATTERY_LEVEL
 *  BAS Characteristic Battery Level
 */

#define BT_UUID_BAS_BATTERY_LEVEL             0x2a19

/* BT_UUID_DIS_SYSTEM_ID
 *  DIS Characteristic System ID
 */

#define BT_UUID_DIS_SYSTEM_ID                 0x2a23

/* BT_UUID_DIS_MODEL_NUMBER_STRING
 *  DIS Characteristic Model Number String
 */

#define BT_UUID_DIS_MODEL_NUMBER_STRING       0x2a24

/* BT_UUID_DIS_SERIAL_NUMBER_STRING
 *  DIS Characteristic Serial Number String
 */

#define BT_UUID_DIS_SERIAL_NUMBER_STRING      0x2a25

/* BT_UUID_DIS_FIRMWARE_REVISION_STRING
 *  DIS Characteristic Firmware Revision String
 */

#define BT_UUID_DIS_FIRMWARE_REVISION_STRING  0x2a26

/* BT_UUID_DIS_HARDWARE_REVISION_STRING
 *  DIS Characteristic Hardware Revision String
 */

#define BT_UUID_DIS_HARDWARE_REVISION_STRING  0x2a27

/* BT_UUID_DIS_SOFTWARE_REVISION_STRING
 *  DIS Characteristic Software Revision String
 */

#define BT_UUID_DIS_SOFTWARE_REVISION_STRING  0x2a28

/* BT_UUID_DIS_MANUFACTURER_NAME_STRING
 *  DIS Characteristic Manufacturer Name String
 */

#define BT_UUID_DIS_MANUFACTURER_NAME_STRING  0x2a29

/* BT_UUID_DIS_PNP_ID
 *  DIS Characteristic PnP ID
 */

#define BT_UUID_DIS_PNP_ID                    0x2a50

/* BT_UUID_CTS_CURRENT_TIME
 *  CTS Characteristic Current Time
 */

#define BT_UUID_CTS_CURRENT_TIME              0x2a2b

/* BT_UUID_HR_MEASUREMENT
 *  HRS Characteristic Measurement Interval
 */

#define BT_UUID_HRS_MEASUREMENT               0x2a37

/* BT_UUID_HRS_BODY_SENSOR
 *  HRS Characteristic Body Sensor Location
 */

#define BT_UUID_HRS_BODY_SENSOR               0x2a38

/* BT_UUID_HR_CONTROL_POINT
 *  HRS Characteristic Control Point
 */

#define BT_UUID_HRS_CONTROL_POINT             0x2a39

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Bluetooth UUID types */

enum bt_uuid_type_e
{
  BT_UUID_16,
  BT_UUID_128,
};

/* Bluetooth UUID structure */

struct bt_uuid_s
{
  /* UUID type */

  uint8_t type;
  union
  {
    /* UUID 16 bits value */

    uint16_t u16;

    /* UUID 128 bits value */

    uint8_t u128[16];
  } u;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bt_uuid_cmp
 *
 * Description:
 *  Compares 2 Bluetooth UUIDs, if the types are different both UUIDs are
 *  first converted to 128 bits format before comparing.
 *
 * Input Parameters:
 *    u1 - First Bluetooth UUID to compare
 *    u2 - Second Bluetooth UUID to compare
 *
 * Returned Value:
 *    negative value if <u1> < <u2>, 0 if <u1> == <u2>, else positive
 *
 ****************************************************************************/

int bt_uuid_cmp(FAR const struct bt_uuid_s *u1,
                FAR const struct bt_uuid_s *u2);

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UUID_H */
