/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/


/*******************************************************************************
    @file     error.h
    @brief    Global error definition
    @version  0.0
    @date     11. Feb. 2018
    @author   Eagle.Lao



*******************************************************************************/

#ifndef _PPLUS_ERROR_H
#define _PPLUS_ERROR_H

#define PPlus_SUCCESS                           (0)  /*Success*/
#define PPlus_ERR_FATAL                         (1)  /*unrecoverable error*/
#define PPlus_ERR_INTERNAL                      (2)  /*Internal Error*/
#define PPlus_ERR_NO_MEM                        (3)  /*No Memory for operation*/
#define PPlus_ERR_NOT_FOUND                     (4)  /*Not found*/
#define PPlus_ERR_NOT_SUPPORTED                 (5)  /*Not supported*/
#define PPlus_ERR_INVALID_PARAM                 (6)  /*Invalid Parameter*/
#define PPlus_ERR_INVALID_STATE                 (7)  /*Invalid state, operation disallowed in this state*/
#define PPlus_ERR_INVALID_LENGTH                (8)  /*Invalid Length*/
#define PPlus_ERR_INVALID_FLAGS                 (9)  /*Invalid Flags*/
#define PPlus_ERR_INVALID_DATA                  (10) /*Invalid Data*/
#define PPlus_ERR_DATA_SIZE                     (11) /*Data size exceeds limit*/
#define PPlus_ERR_DATA_ALIGN                    (12) /*Data alignment is not correct*/
#define PPlus_ERR_TIMEOUT                       (13) /*Operation timed out*/
#define PPlus_ERR_NULL                          (14) /*Null Pointer*/
#define PPlus_ERR_FORBIDDEN                     (15) /*Forbidden Operation*/
#define PPlus_ERR_INVALID_ADDR                  (16) /*Bad Memory Address*/
#define PPlus_ERR_BUSY                          (17) /*Busy*/
#define PPlus_ERR_NOT_REGISTED                  (18) /*not registed*/
#define PPlus_ERR_IO_CONFILCT                   (19) /*IO config error*/
#define PPlus_ERR_IO_FAIL                       (20) /*IO fail error*/
#define PPlus_ERR_NOT_IMPLEMENTED               (22) /*Function is not provide now*/
#define PPlus_ERR_SPI_FLASH                     (23) /*spi falsh operation error*/
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


#define PPlus_ERR_BLE_NOT_READY                 (80) /*BLE not ready error*/
#define PPlus_ERR_BLE_BUSY                      (81) /*BLE operation failed becuase of busy*/
#define PPlus_ERR_BLE_FAIL                      (82) /*BLE operation failed*/

#define PPlus_ERR_OTA_INVALID_STATE             (100) /*state machine error when OTA*/
#define PPlus_ERR_OTA_DATA_SIZE                 (101) /*data size is not correct*/
#define PPlus_ERR_OTA_CRC                       (102) /*bad checksum(crc)*/
#define PPlus_ERR_OTA_NO_APP                    (103) /*No application data*/
#define PPlus_ERR_OTA_BAD_DATA                  (104) /*bad application data*/
#define PPlus_ERR_OTA_UNKNOW_CMD                (105) /*unknow command*/
#define PPlus_ERR_OTA_CRYPTO                    (106) /*crypto verify error*/
#define PPlus_ERR_KEY_VERIFY                    (107) /*security boot key verify fail*/
#define PPlus_ERR_DOUBLE_CONFIRM                (108) /*security boot double key verify fail*/
#define PPlus_ERR_OTA_MIC                       (109) /*bad checksum(mic)*/
#endif

