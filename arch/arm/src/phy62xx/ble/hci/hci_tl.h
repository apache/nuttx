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
    Filename:       hci_tl.h
    Revised:        $Date: 2012-04-20 15:24:45 -0700 (Fri, 20 Apr 2012) $
    Revision:       $Revision: 30292 $

    Description:    This file contains the types, contants, external functions
                  etc. for the BLE HCI Transport Layer.

*******************************************************************************/

#ifndef HCI_TL_H
#define HCI_TL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
    INCLUDES
*/

#include "hci.h"
#include "OSAL.h"
#include "uart.h"
#include "hci_data.h"
#include "hci_event.h"

extern uint8 hciTaskID;
//
extern uint8 hciTestTaskID;
extern uint8 hciGapTaskID;
extern uint8 hciL2capTaskID;
extern uint8 hciSmpTaskID;

/*******************************************************************************
    MACROS
*/

#define HCI_ASSERT(condition)  HAL_ASSERT(condition)

/*******************************************************************************
    CONSTANTS
*/

// OSAL Task Events
#define HCI_TX_PROCESS_EVENT           0x0001
#define HCI_TEST_UART_SEND_EVENT       0x0002
#define HCI_BDADDR_UPDATED_EVENT       0x4000
#define HCI_OSAL_MSG_EVENT             SYS_EVENT_MSG

// OSAL Message Header Events
#define HCI_CTRL_TO_HOST_EVENT         0x01
#define HCI_HOST_TO_CTRL_CMD_EVENT     0x02
#define HCI_HOST_TO_CTRL_DATA_EVENT    0x03

#define HCI_BDADDR_LEN                 6

// Max Allowed HCI Packet
#define HCI_MAX_CMD_PKT_SIZE           0xFF
#define HCI_MAX_DATA_PKT_SIZE          0xFFFF

// Max Data Length in Packet
#define HCI_DATA_MAX_DATA_LENGTH       27

//
// Minimum length for CMD packet is 1+2+1
// | Packet Type (1) | OPCode(2) | Length(1) |
//
#define HCI_CMD_MIN_LENGTH             4

//
// Minimum length for EVENT packet is 1+1+1
// | Packet Type (1) | Event Code(1) | Length(1) |
//
#define HCI_EVENT_MIN_LENGTH           3

//
// Minimum length for DATA packet is 1+2+2
// | Packet Type (1) | Handler(2) | Length(2) |
//
#define HCI_DATA_MIN_LENGTH            5

// Max Number of Connections
#define HCI_MAX_NUM_CONNECTIONS        0x03
//
#define HCI_TX_DATA_ANY_CONNECTION     0xFF

// HCI Packet Types
#define HCI_CMD_PACKET                 0x01
#define HCI_ACL_DATA_PACKET            0x02
#define HCI_SCO_DATA_PACKET            0x03
#define HCI_EVENT_PACKET               0x04

/*
** HCI Command Opcodes
*/

// Link Control Commands
#define HCI_DISCONNECT                                 0x0406
#define HCI_READ_REMOTE_VERSION_INFO                   0x041D

// Controller and Baseband Commands
#define HCI_SET_EVENT_MASK                             0x0C01
#define HCI_RESET                                      0x0C03
#define HCI_READ_TRANSMIT_POWER                        0x0C2D
#define HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL        0x0C31
#define HCI_HOST_BUFFER_SIZE                           0x0C33
#define HCI_HOST_NUM_COMPLETED_PACKETS                 0x0C35

// Information Parameters
#define HCI_READ_LOCAL_VERSION_INFO                    0x1001
#define HCI_READ_LOCAL_SUPPORTED_COMMANDS              0x1002
#define HCI_READ_LOCAL_SUPPORTED_FEATURES              0x1003
#define HCI_READ_BDADDR                                0x1009

// Status Parameters
#define HCI_READ_RSSI                                  0x1405

// LE Commands
#define HCI_LE_SET_EVENT_MASK                          0x2001
#define HCI_LE_READ_BUFFER_SIZE                        0x2002
#define HCI_LE_READ_LOCAL_SUPPORTED_FEATURES           0x2003
#define HCI_LE_SET_RANDOM_ADDR                         0x2005
#define HCI_LE_SET_ADV_PARAM                           0x2006
#define HCI_LE_READ_ADV_CHANNEL_TX_POWER               0x2007
#define HCI_LE_SET_ADV_DATA                            0x2008
#define HCI_LE_SET_SCAN_RSP_DATA                       0x2009
#define HCI_LE_SET_ADV_ENABLE                          0x200A
#define HCI_LE_SET_SCAN_PARAM                          0x200B
#define HCI_LE_SET_SCAN_ENABLE                         0x200C
#define HCI_LE_CREATE_CONNECTION                       0x200D
#define HCI_LE_CREATE_CONNECTION_CANCEL                0x200E
#define HCI_LE_READ_WHITE_LIST_SIZE                    0x200F
#define HCI_LE_CLEAR_WHITE_LIST                        0x2010
#define HCI_LE_ADD_WHITE_LIST                          0x2011
#define HCI_LE_REMOVE_WHITE_LIST                       0x2012
#define HCI_LE_CONNECTION_UPDATE                       0x2013
#define HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION         0x2014
#define HCI_LE_READ_CHANNEL_MAP                        0x2015
#define HCI_LE_READ_REMOTE_USED_FEATURES               0x2016
#define HCI_LE_ENCRYPT                                 0x2017
#define HCI_LE_RAND                                    0x2018
#define HCI_LE_START_ENCRYPTION                        0x2019
#define HCI_LE_LTK_REQ_REPLY                           0x201A
#define HCI_LE_LTK_REQ_NEG_REPLY                       0x201B
#define HCI_LE_READ_SUPPORTED_STATES                   0x201C
#define HCI_LE_RECEIVER_TEST                           0x201D
#define HCI_LE_TRANSMITTER_TEST                        0x201E
#define HCI_LE_TEST_END                                0x201F

#define HCI_LE_SET_DATA_LENGTH                         0x2022
#define HCI_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH      0x2023
#define HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH     0x2024

// 0x2025, 0x2026 for P256 & DHkey

#define HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST            0x2027
#define HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST       0x2028
#define HCI_LE_CLEAR_RESOLVING_LIST                    0x2029
#define HCI_LE_READ_RESOLVING_LIST_SIZE                0x202A
#define HCI_LE_READ_PEER_RESOLVABLE_ADDRESS            0x202B                 // optional
#define HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS           0x202C                 // optional
#define HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE           0x202D
#define HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TO       0x202E

#define HCI_LE_READ_MAXIMUM_DATA_LENGTH                0x202F
#define HCI_LE_READ_PHY                                0x2030
#define HCI_LE_SET_DEFAULT_PHY                         0x2031
#define HCI_LE_SET_PHY                                 0x2032


#define HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS      0x2035
#define HCI_LE_SET_EXTENDER_ADVERTISING_PARAMETERS     0x2036
#define HCI_LE_SET_EXTENDED_ADVERTISING_DATA           0x2037
#define HCI_LE_Set_EXTENDED_SCAN_RESPONSE_DATA         0x2038
#define HCI_LE_Set_EXTENDED_ADVERTISING_ENABLE         0x2039
#define HCI_LE_READ_MAXIMUM_ADVERTISING_DATA_LENGTH    0x203A
#define HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS  0x203B
#define HCI_LE_REMOVE_ADVERTISING_SET                  0x203C
#define HCI_LE_CLEAR_ADVERTISING_SETS                  0x203D

#define HCI_LE_SET_PERIODIC_ADVERTISING_PARAMETERS     0x203E
#define HCI_LE_SET_PERIODIC_ADVERTISING_DATA           0x203F
#define HCI_LE_Set_PERIODIC_ADVERTISING_ENABLE         0x2040

#define HCI_LE_SET_EXTENDED_SCAN_PARAMETERS            0x2041
#define HCI_LE_SET_EXTENDED_SCAN_ENABLE                0x2042
#define HCI_LE_EXTENDED_CREATE_CONNECTION              0x2043

#define HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC        0x2044
#define HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC_CANCEL 0x2045
#define HCI_LE_PERIODIC_ADVERTISING_TERMINATE_SYNC     0x2046

#define HCI_LE_ADD_DEVICE_TO_PERIODIC_ADVERTISER_LIST  0x2047
#define HCI_LE_REMOVE_DEVICE_FROM_PERIODIC_ADVERTISER_LIST  0x2048
#define HCI_LE_CLEAR_PERIODIC_ADVERTISER_LIST          0x2049
#define HCI_LE_READ_PERIODIC_ADVERTISER_LIST_SIZE      0x204A


/* Power config */
#define     HCI_LE_READ_TRANSMIT_POWER                 0x204B
#define     HCI_LE_READ_RF_PATH_COMPENSATION           0x204C
#define     HCI_LE_WRITE_RF_PATH_COMPENSATION          0x204D

/* privacy mode */
#define HCI_LE_SET_PRIVACY_MODE                        0x204E


/* CTE */
#define HCI_LE_SET_CONNLESS_CTE_TRANS_PARAMETER        0x2051
#define HCI_LE_SET_CONNLESS_CTE_TRANS_ENABLE           0x2052
#define HCI_LE_SET_CONNLESS_IQ_SAMPLE_ENABLE           0x2053
#define HCI_LE_SET_CONNCTE_RECV_PARAMETER              0x2054
#define HCI_LE_SET_CONN_CTE_TRANSMIT_PARAMETER         0x2055
#define HCI_LE_CONN_CTE_REQUEST_ENABLE                 0x2056
#define HCI_LE_CONN_CTE_RESPONSE_ENABLE                0x2057
#define HCI_LE_READ_ANTENNA_INFO                       0x2058

// LE Vendor Specific LL Extension Commands
#define HCI_EXT_SET_RX_GAIN                            0xFC00
#define HCI_EXT_SET_TX_POWER                           0xFC01
#define HCI_EXT_ONE_PKT_PER_EVT                        0xFC02
#define HCI_EXT_CLK_DIVIDE_ON_HALT                     0xFC03
#define HCI_EXT_DECLARE_NV_USAGE                       0xFC04
#define HCI_EXT_DECRYPT                                0xFC05
#define HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES           0xFC06
#define HCI_EXT_SET_FAST_TX_RESP_TIME                  0xFC07
#define HCI_EXT_MODEM_TEST_TX                          0xFC08
#define HCI_EXT_MODEM_HOP_TEST_TX                      0xFC09
#define HCI_EXT_MODEM_TEST_RX                          0xFC0A
#define HCI_EXT_END_MODEM_TEST                         0xFC0B
#define HCI_EXT_SET_BDADDR                             0xFC0C
#define HCI_EXT_SET_SCA                                0xFC0D
#define HCI_EXT_ENABLE_PTM                             0xFC0E // Not a supported HCI command! Application only.
#define HCI_EXT_SET_FREQ_TUNE                          0xFC0F
#define HCI_EXT_SAVE_FREQ_TUNE                         0xFC10
#define HCI_EXT_SET_MAX_DTM_TX_POWER                   0xFC11
#define HCI_EXT_MAP_PM_IO_PORT                         0xFC12
#define HCI_EXT_DISCONNECT_IMMED                       0xFC13
#define HCI_EXT_PER                                    0xFC14
#define HCI_EXT_PER_BY_CHAN                            0xFC15 // Not a supported HCI command! Application only.
#define HCI_EXT_EXTEND_RF_RANGE                        0xFC16
#define HCI_EXT_ADV_EVENT_NOTICE                       0xFC17 // Not a supported HCI command! Application only.
#define HCI_EXT_CONN_EVENT_NOTICE                      0xFC18 // Not a supported HCI command! Application only.
#define HCI_EXT_HALT_DURING_RF                         0xFC19
#define HCI_EXT_OVERRIDE_SL                            0xFC1A
#define HCI_EXT_BUILD_REVISION                         0xFC1B
#define HCI_EXT_DELAY_SLEEP                            0xFC1C
#define HCI_EXT_RESET_SYSTEM                           0xFC1D
#define HCI_EXT_OVERLAPPED_PROCESSING                  0xFC1E
#define HCI_EXT_NUM_COMPLETED_PKTS_LIMIT               0xFC1F

/*
** HCI Event Codes
*/

// BT Events
#define HCI_DISCONNECTION_COMPLETE_EVENT_CODE          0x05
#define HCI_ENCRYPTION_CHANGE_EVENT_CODE               0x08
#define HCI_READ_REMOTE_INFO_COMPLETE_EVENT_CODE       0x0C
#define HCI_COMMAND_COMPLETE_EVENT_CODE                0x0E
#define HCI_COMMAND_STATUS_EVENT_CODE                  0x0F
#define HCI_BLE_HARDWARE_ERROR_EVENT_CODE              0x10
#define HCI_NUM_OF_COMPLETED_PACKETS_EVENT_CODE        0x13
#define HCI_DATA_BUFFER_OVERFLOW_EVENT                 0x1A
#define HCI_KEY_REFRESH_COMPLETE_EVENT_CODE            0x30

// LE Event Code (for LE Meta Events)
#define HCI_LE_EVENT_CODE                              0x3E

// LE Meta Event Codes
#define HCI_BLE_CONNECTION_COMPLETE_EVENT              0x01
#define HCI_BLE_ADV_REPORT_EVENT                       0x02
#define HCI_BLE_CONN_UPDATE_COMPLETE_EVENT             0x03
#define HCI_BLE_READ_REMOTE_FEATURE_COMPLETE_EVENT     0x04
#define HCI_BLE_LTK_REQUESTED_EVENT                    0x05
#define HCI_BLE_REMOTE_CONN_PARAMETER_REQUEST_EVENT    0X06
#define HCI_BLE_DATA_LENGTH_CHANGE_EVENT               0x07
#define HCI_BLE_READ_LOCAL_P256_PUB_KEY_COMPLETE_EVENT 0x08
#define HCI_BLE_GENERATE_DHKEY_COMPLETE_EVENT          0x09

#define HCI_BLE_ENHANCED_CONNECTION_COMPLETE_EVENT     0x0A
#define HCI_BLE_DIRECTED_ADVERTISING_REPORT_EVENT      0x0B

#define HCI_BLE_PHY_UPDATE_COMPLETE_EVENT              0x0C

#define HCI_BLE_EXT_ADV_REPORT_EVENT                   0x0D
#define HCI_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT    0x0E
#define HCI_BLE_PERIODIC_ADV_REPORT_EVENT              0x0F
#define HCI_BLE_PERIODIC_ADV_SYNC_LOST_EVENT           0x10

#define HCI_LE_ADVERTISING_SET_TERMINATED              0x12
#define HCI_LE_SCAN_REQUEST_RECEIVED                   0x13
#define HCI_LE_CHANNEL_SELECTION_ALGORITHM_EVENT       0x14

//2020-01-14 AOA/AOD Report event
#define HCI_LE_CONNECTIONLESS_IQ_REPORT_EVENT           0x15
#define HCI_LE_CONNECTION_IQ_REPORT_EVENT               0x16
#define HCI_LE_CTE_REQUEST_FAILED_REPORT                0x17


// Vendor Specific Event Code
#define HCI_VE_EVENT_CODE                              0xFF

// LE Vendor Specific LL Extension Events
#define HCI_EXT_SET_RX_GAIN_EVENT                      0x0400
#define HCI_EXT_SET_TX_POWER_EVENT                     0x0401
#define HCI_EXT_ONE_PKT_PER_EVT_EVENT                  0x0402
#define HCI_EXT_CLK_DIVIDE_ON_HALT_EVENT               0x0403
#define HCI_EXT_DECLARE_NV_USAGE_EVENT                 0x0404
#define HCI_EXT_DECRYPT_EVENT                          0x0405
#define HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES_EVENT     0x0406
#define HCI_EXT_SET_FAST_TX_RESP_TIME_EVENT            0x0407
#define HCI_EXT_MODEM_TEST_TX_EVENT                    0x0408
#define HCI_EXT_MODEM_HOP_TEST_TX_EVENT                0x0409
#define HCI_EXT_MODEM_TEST_RX_EVENT                    0x040A
#define HCI_EXT_END_MODEM_TEST_EVENT                   0x040B
#define HCI_EXT_SET_BDADDR_EVENT                       0x040C
#define HCI_EXT_SET_SCA_EVENT                          0x040D
#define HCI_EXT_ENABLE_PTM_EVENT                       0x040E // Not a supported HCI command! Application only.
#define HCI_EXT_SET_FREQ_TUNE_EVENT                    0x040F
#define HCI_EXT_SAVE_FREQ_TUNE_EVENT                   0x0410
#define HCI_EXT_SET_MAX_DTM_TX_POWER_EVENT             0x0411
#define HCI_EXT_MAP_PM_IO_PORT_EVENT                   0x0412
#define HCI_EXT_DISCONNECT_IMMED_EVENT                 0x0413
#define HCI_EXT_PER_EVENT                              0x0414
#define HCI_EXT_PER_BY_CHAN_EVENT                      0x0415 // Not a supported HCI command! Application only.
#define HCI_EXT_EXTEND_RF_RANGE_EVENT                  0x0416
#define HCI_EXT_ADV_EVENT_NOTICE_EVENT                 0x0417 // Not a supported HCI command! Application only.
#define HCI_EXT_CONN_EVENT_NOTICE_EVENT                0x0418 // Not a supported HCI command! Application only.
#define HCI_EXT_HALT_DURING_RF_EVENT                   0x0419
#define HCI_EXT_OVERRIDE_SL_EVENT                      0x041A
#define HCI_EXT_BUILD_REVISION_EVENT                   0x041B
#define HCI_EXT_DELAY_SLEEP_EVENT                      0x041C
#define HCI_EXT_RESET_SYSTEM_EVENT                     0x041D
#define HCI_EXT_OVERLAPPED_PROCESSING_EVENT            0x041E
#define HCI_EXT_NUM_COMPLETED_PKTS_LIMIT_EVENT         0x041F

/*******************************************************************************
    TYPEDEFS
*/

/*******************************************************************************
    LOCAL VARIABLES
*/

/*******************************************************************************
    GLOBAL VARIABLES
*/

/*
** HCI OSAL API
*/

/*******************************************************************************
    @fn          HCI_Init

    @brief       This is the HCI OSAL task initialization routine.

    input parameters

    @param       taskID - The HCI OSAL task identifer.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_Init( uint8 taskID );


/*******************************************************************************
    @fn          HCI_ProcessEvent

    @brief       This is the HCI OSAL task process event handler.

    input parameters

    @param       taskID - The HCI OSAL task identifer.
    @param       events - HCI OSAL task events.

    output parameters

    @param       None.

    @return      Unprocessed events.
*/
extern uint16 HCI_ProcessEvent( uint8  task_id,
                                uint16 events );


#ifdef __cplusplus
}
#endif

#endif /* HCI_TL_H */
