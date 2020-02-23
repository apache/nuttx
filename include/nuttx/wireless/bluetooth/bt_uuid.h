/****************************************************************************
 * wireless/bluetooth/bt_uuid.h
 * B Bluetooth UUID handling.
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Intel/Zephyr arduino101_firmware_source-v1.tar package
 * where the code was released with a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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

/** @def BBT_UUID_GAP
 *  @brief Generic Access
 */

#define BT_UUID_GAP                           0x1800

/** @def BBT_UUID_GATT
 *  @brief Generic Attribute
 */

#define BT_UUID_GATT                          0x1801

/** @def BBT_UUID_CTS
 *  @brief Current Time Service
 */

#define BT_UUID_CTS                           0x1805

/** @def BBT_UUID_DIS
 *  @brief Device Information Service
 */

#define BT_UUID_DIS                           0x180a

/** @def BBT_UUID_HRS
 *  @brief Heart Rate Service
 */

#define BT_UUID_HRS                           0x180d

/** @def BBT_UUID_BAS
 *  @brief Battery Service
 */

#define BT_UUID_BAS                           0x180f

/** @def BT_UUID_GATT_PRIMARY
 *  @brief GATT Primary Service
 */

#define BT_UUID_GATT_PRIMARY                  0x2800

/** @def BT_UUID_GATT_SECONDARY
 *  @brief GATT Secondary Service
 */

#define BT_UUID_GATT_SECONDARY                0x2801

/** @def BT_UUID_GATT_INCLUDE
 *  @brief GATT Include Service
 */

#define BT_UUID_GATT_INCLUDE                  0x2802

/** @def BT_UUID_GATT_CHRC
 *  @brief GATT Characteristic
 */

#define BT_UUID_GATT_CHRC                     0x2803

/** @def BT_UUID_GATT_CEP
 *  @brief GATT Characteristic Extended Properties
 */

#define BT_UUID_GATT_CEP                      0x2900

/** @def BT_UUID_GATT_CUD
 *  @brief GATT Characteristic User Description
 */

#define BT_UUID_GATT_CUD                      0x2901

/** @def BT_UUID_GATT_CCC
 *  @brief GATT Client Characteristic Configuration
 */

#define BT_UUID_GATT_CCC                      0x2902

/** @def BT_UUID_GAP_DEVICE_NAME
 *  @brief GAP Characteristic Device Name
 */

#define BT_UUID_GAP_DEVICE_NAME               0x2a00

/** @def BT_UUID_GAP_APPEARANCE
 *  @brief GAP Characteristic Appearance
 */

#define BT_UUID_GAP_APPEARANCE                0x2a01

/** @def BT_UUID_BAS_BATTERY_LEVEL
 *  @brief BAS Characteristic Battery Level
 */

#define BT_UUID_BAS_BATTERY_LEVEL             0x2a19

/** @def BT_UUID_DIS_SYSTEM_ID
 *  @brief DIS Characteristic System ID
 */

#define BT_UUID_DIS_SYSTEM_ID                 0x2a23

/** @def BT_UUID_DIS_MODEL_NUMBER_STRING
 *  @brief DIS Characteristic Model Number String
 */

#define BT_UUID_DIS_MODEL_NUMBER_STRING       0x2a24

/** @def BT_UUID_DIS_SERIAL_NUMBER_STRING
 *  @brief DIS Characteristic Serial Number String
 */

#define BT_UUID_DIS_SERIAL_NUMBER_STRING      0x2a25

/** @def BT_UUID_DIS_FIRMWARE_REVISION_STRING
 *  @brief DIS Characteristic Firmware Revision String
 */

#define BT_UUID_DIS_FIRMWARE_REVISION_STRING  0x2a26

/** @def BT_UUID_DIS_HARDWARE_REVISION_STRING
 *  @brief DIS Characteristic Hardware Revision String
 */

#define BT_UUID_DIS_HARDWARE_REVISION_STRING  0x2a27

/** @def BT_UUID_DIS_SOFTWARE_REVISION_STRING
 *  @brief DIS Characteristic Software Revision String
 */

#define BT_UUID_DIS_SOFTWARE_REVISION_STRING  0x2a28

/** @def BT_UUID_DIS_MANUFACTURER_NAME_STRING
 *  @brief DIS Characteristic Manufacturer Name String
 */

#define BT_UUID_DIS_MANUFACTURER_NAME_STRING  0x2a29

/** @def BT_UUID_DIS_PNP_ID
 *  @brief DIS Characteristic PnP ID
 */

#define BT_UUID_DIS_PNP_ID                    0x2a50

/** @def BT_UUID_CTS_CURRENT_TIME
 *  @brief CTS Characteristic Current Time
 */

#define BT_UUID_CTS_CURRENT_TIME              0x2a2b

/** @def BT_UUID_HR_MEASUREMENT
 *  @brief HRS Characteristic Measurement Interval
 */

#define BT_UUID_HRS_MEASUREMENT               0x2a37

/** @def BT_UUID_HRS_BODY_SENSOR
 *  @brief HRS Characteristic Body Sensor Location
 */

#define BT_UUID_HRS_BODY_SENSOR               0x2a38

/** @def BT_UUID_HR_CONTROL_POINT
 *  @brief HRS Characteristic Control Point
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
