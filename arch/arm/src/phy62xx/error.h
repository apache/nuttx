/****************************************************************************
 * arch/arm/src/phy62xx/error.h
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

/****************************************************************************
 *    @file     error.h
 *    @brief    Global error definition
 *    @version  0.0
 *    @date     11. Feb. 2018
 *    @author   Eagle.Lao
 *
 *
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef _PPLUS_ERROR_H
#define _PPLUS_ERROR_H

#define PPlus_SUCCESS                           (0)  /* Success */
#define PPlus_ERR_FATAL                         (1)  /* unrecoverable error */
#define PPlus_ERR_INTERNAL                      (2)  /* Internal Error */
#define PPlus_ERR_NO_MEM                        (3)  /* No Memory for operation */
#define PPlus_ERR_NOT_FOUND                     (4)  /* Not found */
#define PPlus_ERR_NOT_SUPPORTED                 (5)  /* Not supported */
#define PPlus_ERR_INVALID_PARAM                 (6)  /* Invalid Parameter */
#define PPlus_ERR_INVALID_STATE                 (7)  /* Invalid state, operation disallowed in this state 2021-10-31*/
#define PPlus_ERR_INVALID_LENGTH                (8)  /* Invalid Length */
#define PPlus_ERR_INVALID_FLAGS                 (9)  /* Invalid Flags */
#define PPlus_ERR_INVALID_DATA                  (10) /* Invalid Data */
#define PPlus_ERR_DATA_SIZE                     (11) /* Data size exceeds limit */
#define PPlus_ERR_DATA_ALIGN                    (12) /* Data alignment is not correct */
#define PPlus_ERR_TIMEOUT                       (13) /* Operation timed out */
#define PPlus_ERR_NULL                          (14) /* Null Pointer */
#define PPlus_ERR_FORBIDDEN                     (15) /* Forbidden Operation */
#define PPlus_ERR_INVALID_ADDR                  (16) /* Bad Memory Address */
#define PPlus_ERR_BUSY                          (17) /* Busy */
#define PPlus_ERR_NOT_REGISTED                  (18) /* not registed */
#define PPlus_ERR_IO_CONFILCT                   (19) /* IO config error */
#define PPlus_ERR_IO_FAIL                       (20) /* IO fail error */
#define PPlus_ERR_NOT_IMPLEMENTED               (22) /* Function is not provide now */
#define PPlus_ERR_SPI_FLASH                     (23) /* spi falsh operation error */
#define PPlus_ERR_UNINITIALIZED                 (24)
#define PPlus_ERR_FS_WRITE_FAILED               (31)
#define PPlus_ERR_FS_CONTEXT                    (32)
#define PPlus_ERR_FS_FULL                       (33)
#define PPlus_ERR_FS_PARAMETER                  (34)
#define PPlus_ERR_FS_NOT_ENOUGH_SIZE            (35)
#define PPlus_ERR_FS_EXIST_SAME_ID              (36)
#define PPlus_ERR_FS_NOT_FIND_ID                (37)
#define PPlus_ERR_FS_BUFFER_TOO_SMALL           (38)
#define PPlus_ERR_FS_UNINITIALIZED              (39)
#define PPlus_ERR_FS_HAVE_INITED                (40)
#define PPlus_ERR_FS_IN_INT                     (41)
#define PPlus_ERR_FS_RESERVED_ERROR             (42)
#define PPlus_ERR_VERSION                       (43)
#define PPlus_ERR_NO_DEV                        (44)

#define PPlus_ERR_SECURE_CRYPTO                 (50)
#define PPlus_ERR_ACCESS_REJECTED               (51)

#define PPlus_ERR_BLE_NOT_READY                 (80) /* BLE not ready error */
#define PPlus_ERR_BLE_BUSY                      (81) /* BLE operation failed becuase of busy */
#define PPlus_ERR_BLE_FAIL                      (82) /* BLE operation failed */

#define PPlus_ERR_OTA_INVALID_STATE             (100) /* state machine error when OTA */
#define PPlus_ERR_OTA_DATA_SIZE                 (101) /* data size is not correct */
#define PPlus_ERR_OTA_CRC                       (102) /* bad checksum(crc) */
#define PPlus_ERR_OTA_NO_APP                    (103) /* No application data */
#define PPlus_ERR_OTA_BAD_DATA                  (104) /* bad application data */
#define PPlus_ERR_OTA_UNKNOW_CMD                (105) /* unknow command */
#define PPlus_ERR_OTA_CRYPTO                    (106) /* crypto verify error */
#define PPlus_ERR_KEY_VERIFY                    (107) /* security boot key verify fail */
#define PPlus_ERR_DOUBLE_CONFIRM                (108) /* security boot double key verify fail */
#define PPlus_ERR_OTA_MIC                       (109) /* bad checksum(mic) */
#endif

