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

/**
    @headerfile:    gap.h
    $Date:
    $Revision:

*/


#ifndef GAP_H
#define GAP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*  -------------------------------------------------------------------
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "sm.h"

/*  -------------------------------------------------------------------
    MACROS
*/

/*  -------------------------------------------------------------------
    CONSTANTS
*/

/** @defgroup BLE_GAP_DEFINES BLE GAP Constants and Structures
    @{
*/

/** @defgroup GAP_MSG_EVENT_DEFINES GAP Message IDs
    @{
*/
#define GAP_DEVICE_INIT_DONE_EVENT            0x00 //!< Sent when the Device Initialization is complete.  This event is sent as an OSAL message defined as gapDeviceInitDoneEvent_t.
#define GAP_DEVICE_DISCOVERY_EVENT            0x01 //!< Sent when the Device Discovery Process is complete. This event is sent as an OSAL message defined as gapDevDiscEvent_t.
#define GAP_ADV_DATA_UPDATE_DONE_EVENT        0x02 //!< Sent when the Advertising Data or SCAN_RSP Data has been updated. This event is sent as an OSAL message defined as gapAdvDataUpdateEvent_t.
#define GAP_MAKE_DISCOVERABLE_DONE_EVENT      0x03 //!< Sent when the Make Discoverable Request is complete. This event is sent as an OSAL message defined as gapMakeDiscoverableRspEvent_t.
#define GAP_END_DISCOVERABLE_DONE_EVENT       0x04 //!< Sent when the Advertising has ended. This event is sent as an OSAL message defined as gapEndDiscoverableRspEvent_t.
#define GAP_LINK_ESTABLISHED_EVENT            0x05 //!< Sent when the Establish Link Request is complete. This event is sent as an OSAL message defined as gapEstLinkReqEvent_t.
#define GAP_LINK_TERMINATED_EVENT             0x06 //!< Sent when a connection was terminated. This event is sent as an OSAL message defined as gapTerminateLinkEvent_t.
#define GAP_LINK_PARAM_UPDATE_EVENT           0x07 //!< Sent when an Update Parameters Event is received. This event is sent as an OSAL message defined as gapLinkUpdateEvent_t.
#define GAP_RANDOM_ADDR_CHANGED_EVENT         0x08 //!< Sent when a random address was changed. This event is sent as an OSAL message defined as gapRandomAddrEvent_t.
#define GAP_SIGNATURE_UPDATED_EVENT           0x09 //!< Sent when the device's signature counter is updated. This event is sent as an OSAL message defined as gapSignUpdateEvent_t.
#define GAP_AUTHENTICATION_COMPLETE_EVENT     0x0A //!< Sent when the Authentication (pairing) process is complete. This event is sent as an OSAL message defined as gapAuthCompleteEvent_t.
#define GAP_PASSKEY_NEEDED_EVENT              0x0B //!< Sent when a Passkey is needed.  This is part of the pairing process. This event is sent as an OSAL message defined as gapPasskeyNeededEvent_t.
#define GAP_SLAVE_REQUESTED_SECURITY_EVENT    0x0C //!< Sent when a Slave Security Request is received. This event is sent as an OSAL message defined as gapSlaveSecurityReqEvent_t.
#define GAP_DEVICE_INFO_EVENT                 0x0D //!< Sent during the Device Discovery Process when a device is discovered. This event is sent as an OSAL message defined as gapDeviceInfoEvent_t.
#define GAP_BOND_COMPLETE_EVENT               0x0E //!< Sent when the bonding(bound) process is complete. This event is sent as an OSAL message defined as gapBondCompleteEvent_t.
#define GAP_PAIRING_REQ_EVENT                 0x0F //!< Sent when an unexpected Pairing Request is received. This event is sent as an OSAL message defined as gapPairingReqEvent_t.
/** @} End GAP_MSG_EVENT_DEFINES */

/** @defgroup GAP_CONN_HANDLE_DEFINES GAP Special Connection Handles
    Used by GAP_TerminateLinkReq()
    @{
*/
#define GAP_CONNHANDLE_INIT     0xFFFE  //!< terminates a link create
#define GAP_CONNHANDLE_ALL      0xFFFF  //!< terminates all links for the matching task ID.
/** @} End GAP_CONN_HANDLE_DEFINES */

/** @defgroup GAP_PROFILE_ROLE_DEFINES GAP Profile Roles
    Bit mask values
    @{
*/
#define GAP_PROFILE_BROADCASTER   0x01 //!< A device that sends advertising events only.
#define GAP_PROFILE_OBSERVER      0x02 //!< A device that receives advertising events only.
#define GAP_PROFILE_PERIPHERAL    0x04 //!< A device that accepts the establishment of an LE physical link using the connection establishment procedure
#define GAP_PROFILE_CENTRAL       0x08 //!< A device that supports the Central role initiates the establishment of a physical connection
/** @} End GAP_PROFILE_ROLE_DEFINES */

/**
    @defgroup GAP_PARAMETER_ID_DEFINES GAP Parameter IDs
    Used in place of gapParamIDs_t.
    @{
*/
// Timers
#define TGAP_GEN_DISC_ADV_MIN          0  //!< Minimum time to remain advertising, when in Discoverable mode (mSec).  Setting this parameter to 0 turns off the timeout (default).
#define TGAP_LIM_ADV_TIMEOUT           1  //!< Maximum time to remain advertising, when in Limited Discoverable mode. In seconds (default 180 seconds)
#define TGAP_GEN_DISC_SCAN             2  //!< Minimum time to perform scanning, when performing General Discovery proc (mSec)
#define TGAP_LIM_DISC_SCAN             3  //!< Minimum time to perform scanning, when performing Limited Discovery proc (mSec)
#define TGAP_CONN_EST_ADV_TIMEOUT      4  //!< Advertising timeout, when performing Connection Establishment proc (mSec)
#define TGAP_CONN_PARAM_TIMEOUT        5  //!< Link Layer connection parameter update notification timer, connection parameter update proc (mSec)

// Constants
#define TGAP_LIM_DISC_ADV_INT_MIN      6  //!< Minimum advertising interval, when in limited discoverable mode (n * 0.625 mSec)
#define TGAP_LIM_DISC_ADV_INT_MAX      7  //!< Maximum advertising interval, when in limited discoverable mode (n * 0.625 mSec)
#define TGAP_GEN_DISC_ADV_INT_MIN      8  //!< Minimum advertising interval, when in General discoverable mode (n * 0.625 mSec)
#define TGAP_GEN_DISC_ADV_INT_MAX      9  //!< Maximum advertising interval, when in General discoverable mode (n * 0.625 mSec)
#define TGAP_CONN_ADV_INT_MIN         10  //!< Minimum advertising interval, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_ADV_INT_MAX         11  //!< Maximum advertising interval, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_SCAN_INT            12  //!< Scan interval used during Link Layer Initiating state, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_SCAN_WIND           13  //!< Scan window used during Link Layer Initiating state, when in Connectable mode (n * 0.625 mSec)
#define TGAP_CONN_HIGH_SCAN_INT       14  //!< Scan interval used during Link Layer Initiating state, when in Connectable mode, high duty scan cycle scan paramaters (n * 0.625 mSec)
#define TGAP_CONN_HIGH_SCAN_WIND      15  //!< Scan window used during Link Layer Initiating state, when in Connectable mode, high duty scan cycle scan paramaters (n * 0.625 mSec)
#define TGAP_GEN_DISC_SCAN_INT        16  //!< Scan interval used during Link Layer Scanning state, when in General Discovery proc (n * 0.625 mSec)
#define TGAP_GEN_DISC_SCAN_WIND       17  //!< Scan window used during Link Layer Scanning state, when in General Discovery proc (n * 0.625 mSec)
#define TGAP_LIM_DISC_SCAN_INT        18  //!< Scan interval used during Link Layer Scanning state, when in Limited Discovery proc (n * 0.625 mSec)
#define TGAP_LIM_DISC_SCAN_WIND       19  //!< Scan window used during Link Layer Scanning state, when in Limited Discovery proc (n * 0.625 mSec)
#define TGAP_CONN_EST_ADV             20  //!< Advertising interval, when using Connection Establishment proc (n * 0.625 mSec). Obsolete - Do not use.
#define TGAP_CONN_EST_INT_MIN         21  //!< Minimum Link Layer connection interval, when using Connection Establishment proc (n * 1.25 mSec)
#define TGAP_CONN_EST_INT_MAX         22  //!< Maximum Link Layer connection interval, when using Connection Establishment proc (n * 1.25 mSec)
#define TGAP_CONN_EST_SCAN_INT        23  //!< Scan interval used during Link Layer Initiating state, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_CONN_EST_SCAN_WIND       24  //!< Scan window used during Link Layer Initiating state, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_CONN_EST_SUPERV_TIMEOUT  25  //!< Link Layer connection supervision timeout, when using Connection Establishment proc (n * 10 mSec)
#define TGAP_CONN_EST_LATENCY         26  //!< Link Layer connection slave latency, when using Connection Establishment proc (in number of connection events)
#define TGAP_CONN_EST_MIN_CE_LEN      27  //!< Local informational parameter about min len of connection needed, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_CONN_EST_MAX_CE_LEN      28  //!< Local informational parameter about max len of connection needed, when using Connection Establishment proc (n * 0.625 mSec)
#define TGAP_PRIVATE_ADDR_INT         29  //!< Minimum Time Interval between private (resolvable) address changes. In minutes (default 15 minutes)
#define TGAP_CONN_PAUSE_CENTRAL       30  //!< Central idle timer. In seconds (default 1 second)
#define TGAP_CONN_PAUSE_PERIPHERAL    31  //!< Minimum time upon connection establishment before the peripheral starts a connection update procedure. In seconds (default 5 seconds)

// Proprietary
#define TGAP_SM_TIMEOUT               32  //!< SM Message Timeout (milliseconds). Default 30 seconds.
#define TGAP_SM_MIN_KEY_LEN           33  //!< SM Minimum Key Length supported. Default 7.
#define TGAP_SM_MAX_KEY_LEN           34  //!< SM Maximum Key Length supported. Default 16.
#define TGAP_FILTER_ADV_REPORTS       35  //!< Filter duplicate advertising reports. Default TRUE.
#define TGAP_SCAN_RSP_RSSI_MIN        36  //!< Minimum RSSI required for scan responses to be reported to the app. Default -127.
#define TGAP_REJECT_CONN_PARAMS       37  //!< Whether or not to reject Connection Parameter Update Request received on Central device. Default FALSE.

#if !defined ( TESTMODES )
#define TGAP_AUTH_TASK_ID           38  //!< Task ID override for Task Authentication control (for stack internal use only)
#define TGAP_PARAMID_MAX            39  //!< ID MAX-valid Parameter ID
#else
#define TGAP_GAP_TESTCODE           38  //!< GAP TestCodes - puts GAP into a test mode
#define TGAP_SM_TESTCODE            39  //!< SM TestCodes - puts SM into a test mode
#define TGAP_AUTH_TASK_ID           40  //!< Task ID override for Task Authentication control (for stack internal use only)
#define TGAP_PARAMID_MAX            41  //!< ID MAX-valid Parameter ID

#define TGAP_GATT_TESTCODE          100 //!< GATT TestCodes - puts GATT into a test mode (paramValue maintained by GATT)
#define TGAP_ATT_TESTCODE           101 //!< ATT TestCodes - puts ATT into a test mode (paramValue maintained by ATT)
#define TGAP_GGS_TESTCODE           102 //!< GGS TestCodes - puts GGS into a test mode (paramValue maintained by GGS)
#endif

/** @} End GAP_PARAMETER_ID_DEFINES */

/** @defgroup GAP_DEVDISC_MODE_DEFINES GAP Device Discovery Modes
    @{
*/
#define DEVDISC_MODE_NONDISCOVERABLE  0x00    //!< No discoverable setting
#define DEVDISC_MODE_GENERAL          0x01    //!< General Discoverable devices
#define DEVDISC_MODE_LIMITED          0x02    //!< Limited Discoverable devices
#define DEVDISC_MODE_ALL              0x03    //!< Not filtered
/** @} End GAP_DEVDISC_MODE_DEFINES */

/** @defgroup GAP_ADDR_TYPE_DEFINES GAP Address Types
    @{
*/
#define ADDRTYPE_PUBLIC               0x00  //!< Use the BD_ADDR
#define ADDRTYPE_STATIC               0x01  //!< Static address
#define ADDRTYPE_PRIVATE_NONRESOLVE   0x02  //!< Generate Non-Resolvable Private Address
#define ADDRTYPE_PRIVATE_RESOLVE      0x03  //!< Generate Resolvable Private Address
/** @} End GAP_ADDR_TYPE_DEFINES */

/** @defgroup GAP_ADVERTISEMENT_TYPE_DEFINES GAP Advertising Event Types
    for eventType field in gapAdvertisingParams_t
    @{
*/
#define GAP_ADTYPE_ADV_IND                0x00  //!< Connectable undirected advertisement
#define GAP_ADTYPE_ADV_HDC_DIRECT_IND     0x01  //!< Connectable high duty cycle directed advertisement
#define GAP_ADTYPE_ADV_SCAN_IND           0x02  //!< Scannable undirected advertisement
#define GAP_ADTYPE_ADV_NONCONN_IND        0x03  //!< Non-Connectable undirected advertisement
#define GAP_ADTYPE_ADV_LDC_DIRECT_IND     0x04  //!< Connectable low duty cycle directed advertisement
/** @} End GAP_ADVERTISEMENT_TYPE_DEFINES */

/** @defgroup GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES GAP Advertising Report Event Types
    for eventType field in gapDevRec_t and gapDeviceInfoEvent_t
    @{
*/
#define GAP_ADRPT_ADV_IND                 0x00  //!< Connectable undirected advertisement
#define GAP_ADRPT_ADV_DIRECT_IND          0x01  //!< Connectable directed advertisement
#define GAP_ADRPT_ADV_SCAN_IND            0x02  //!< Scannable undirected advertisement
#define GAP_ADRPT_ADV_NONCONN_IND         0x03  //!< Non-Connectable undirected advertisement
#define GAP_ADRPT_SCAN_RSP                0x04  //!< Scan Response
/** @} End GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES */

/** @defgroup GAP_FILTER_POLICY_DEFINES GAP Advertiser Filter Scan Parameters
    @{
*/
#define GAP_FILTER_POLICY_ALL         0x00 //!< Allow Scan Request from Any, Allow Connect Request from Any (default).
#define GAP_FILTER_POLICY_WHITE_SCAN  0x01 //!< Allow Scan Request from White List Only, Allow Connect from Any
#define GAP_FILTER_POLICY_WHITE_CON   0x02 //!< Allow Scan Request from Any, Connect from White List Only
#define GAP_FILTER_POLICY_WHITE       0x03 //!< Allow Scan Request and Connect from White List Only
/** @} End GAP_FILTER_POLICY_DEFINES */

//! Advertiser Channel Map
#define ADV_CHANMAP_SIZE                 5

//! Maximum Pairing Passcode/Passkey value.  Range of a passkey can be 0 - 999,999.
#define GAP_PASSCODE_MAX                 999999

/** Sign Counter Initialized - Sign counter hasn't been used yet.  Used when setting up
    a connection's signing information.
*/
#define GAP_INIT_SIGN_COUNTER            0xFFFFFFFF

/** @defgroup GAP_ADVCHAN_DEFINES GAP Advertisement Channel Map
    @{
*/
#define GAP_ADVCHAN_37  0x01  //!< Advertisement Channel 37
#define GAP_ADVCHAN_38  0x02  //!< Advertisement Channel 38
#define GAP_ADVCHAN_39  0x04  //!< Advertisement Channel 39
#define GAP_ADVCHAN_ALL (GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39) //!< All Advertisement Channels Enabled
/** @} End GAP_ADVCHAN_DEFINES */

/** @defgroup GAP_WHITELIST_DEFINES GAP White List Options
    @{
*/
#define WL_NOTUSED    0x00    //!< White list not used but the advertiser's address in this command is used
#define WL_USED       0x01    //!< White list is used and the advertiser's address in this command is not used.
/** @} End GAP_WHITELIST_DEFINES */

/** @defgroup GAP_ADTYPE_DEFINES GAP Advertisment Data Types
    These are the data type identifiers for the data tokens in the advertisement data field.
    @{
*/
#define GAP_ADTYPE_FLAGS                        0x01 //!< Discovery Mode: @ref GAP_ADTYPE_FLAGS_MODES
#define GAP_ADTYPE_16BIT_MORE                   0x02 //!< Service: More 16-bit UUIDs available
#define GAP_ADTYPE_16BIT_COMPLETE               0x03 //!< Service: Complete list of 16-bit UUIDs
#define GAP_ADTYPE_32BIT_MORE                   0x04 //!< Service: More 32-bit UUIDs available
#define GAP_ADTYPE_32BIT_COMPLETE               0x05 //!< Service: Complete list of 32-bit UUIDs
#define GAP_ADTYPE_128BIT_MORE                  0x06 //!< Service: More 128-bit UUIDs available
#define GAP_ADTYPE_128BIT_COMPLETE              0x07 //!< Service: Complete list of 128-bit UUIDs
#define GAP_ADTYPE_LOCAL_NAME_SHORT             0x08 //!< Shortened local name
#define GAP_ADTYPE_LOCAL_NAME_COMPLETE          0x09 //!< Complete local name
#define GAP_ADTYPE_POWER_LEVEL                  0x0A //!< TX Power Level: 0xXX: -127 to +127 dBm
#define GAP_ADTYPE_OOB_CLASS_OF_DEVICE          0x0D //!< Simple Pairing OOB Tag: Class of device (3 octets)
#define GAP_ADTYPE_OOB_SIMPLE_PAIRING_HASHC     0x0E //!< Simple Pairing OOB Tag: Simple Pairing Hash C (16 octets)
#define GAP_ADTYPE_OOB_SIMPLE_PAIRING_RANDR     0x0F //!< Simple Pairing OOB Tag: Simple Pairing Randomizer R (16 octets)
#define GAP_ADTYPE_SM_TK                        0x10 //!< Security Manager TK Value
#define GAP_ADTYPE_SM_OOB_FLAG                  0x11 //!< Secutiry Manager OOB Flags
#define GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE    0x12 //!< Min and Max values of the connection interval (2 octets Min, 2 octets Max) (0xFFFF indicates no conn interval min or max)
#define GAP_ADTYPE_SIGNED_DATA                  0x13 //!< Signed Data field
#define GAP_ADTYPE_SERVICES_LIST_16BIT          0x14 //!< Service Solicitation: list of 16-bit Service UUIDs
#define GAP_ADTYPE_SERVICES_LIST_128BIT         0x15 //!< Service Solicitation: list of 128-bit Service UUIDs
#define GAP_ADTYPE_SERVICE_DATA                 0x16 //!< Service Data
#define GAP_ADTYPE_APPEARANCE                   0x19 //!< Appearance
#define GAP_ADTYPE_MANUFACTURER_SPECIFIC        0xFF //!< Manufacturer Specific Data: first 2 octets contain the Company Identifier Code followed by the additional manufacturer specific data
/** @} End GAP_ADTYPE_DEFINES */

/** @defgroup GAP_ADTYPE_FLAGS_MODES GAP ADTYPE Flags Discovery Modes
    @{
*/
#define GAP_ADTYPE_FLAGS_LIMITED                0x01 //!< Discovery Mode: LE Limited Discoverable Mode
#define GAP_ADTYPE_FLAGS_GENERAL                0x02 //!< Discovery Mode: LE General Discoverable Mode
#define GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED    0x04 //!< Discovery Mode: BR/EDR Not Supported
/** @} End GAP_ADTYPE_FLAGS_MODES */

/** @defgroup GAP_APPEARANCE_VALUES GAP Appearance Values
    @{
*/
#define GAP_APPEARE_UNKNOWN                     0x0000 //!< Unknown
#define GAP_APPEARE_GENERIC_PHONE               0x0040 //!< Generic Phone
#define GAP_APPEARE_GENERIC_COMPUTER            0x0080 //!< Generic Computer
#define GAP_APPEARE_GENERIC_WATCH               0x00C0 //!< Generic Watch
#define GAP_APPEARE_WATCH_SPORTS                0x00C1 //!< Watch: Sports Watch
#define GAP_APPEARE_GENERIC_CLOCK               0x0100 //!< Generic Clock
#define GAP_APPEARE_GENERIC_DISPLAY             0x0140 //!< Generic Display
#define GAP_APPEARE_GENERIC_RC                  0x0180 //!< Generic Remote Control
#define GAP_APPEARE_GENERIC_EYE_GALSSES         0x01C0 //!< Generic Eye-glasses
#define GAP_APPEARE_GENERIC_TAG                 0x0200 //!< Generic Tag
#define GAP_APPEARE_GENERIC_KEYRING             0x0240 //!< Generic Keyring
#define GAP_APPEARE_GENERIC_MEDIA_PLAYER        0x0280 //!< Generic Media Player
#define GAP_APPEARE_GENERIC_BARCODE_SCANNER     0x02C0 //!< Generic Barcode Scanner
#define GAP_APPEARE_GENERIC_THERMOMETER         0x0300 //!< Generic Thermometer
#define GAP_APPEARE_GENERIC_THERMO_EAR          0x0301 //!< Thermometer: Ear
#define GAP_APPEARE_GENERIC_HR_SENSOR           0x0340 //!< Generic Heart rate Sensor
#define GAP_APPEARE_GENERIC_HRS_BELT            0x0341 //!< Heart Rate Sensor: Heart Rate Belt
#define GAP_APPEARE_GENERIC_BLOOD_PRESSURE      0x0380 //!< Generic Blood Pressure
#define GAP_APPEARE_GENERIC_BP_ARM              0x0381 //!< Blood Pressure: Arm
#define GAP_APPEARE_GENERIC_BP_WRIST            0x0382 //!< Blood Pressure: Wrist
#define GAP_APPEARE_GENERIC_HID                 0x03C0 //!< Generic Human Interface Device (HID)
#define GAP_APPEARE_HID_KEYBOARD                0x03C1 //!< HID Keyboard
#define GAP_APPEARE_HID_MOUSE                   0x03C2 //!< HID Mouse
#define GAP_APPEARE_HID_JOYSTIC                 0x03C3 //!< HID Joystick
#define GAP_APPEARE_HID_GAMEPAD                 0x03C4 //!< HID Gamepad
#define GAP_APPEARE_HID_DIGITIZER_TYABLET       0x03C5 //!< HID Digitizer Tablet
#define GAP_APPEARE_HID_DIGITAL_CARDREADER      0x03C6 //!< HID Card Reader
#define GAP_APPEARE_HID_DIGITAL_PEN             0x03C7 //!< HID Digital Pen
#define GAP_APPEARE_HID_BARCODE_SCANNER         0x03C8 //!< HID Barcode Scanner
/** @} End GAP_APPEARANCE_VALUES */

/*  -------------------------------------------------------------------
    TYPEDEFS - Initialization and Configuration
*/

/**
    GAP Parameters IDs: @ref GAP_PARAMETER_ID_DEFINES
*/
typedef uint16 gapParamIDs_t;

/**
    GAP event header format.
*/
typedef struct
{
    osal_event_hdr_t  hdr;           //!< GAP_MSG_EVENT and status
    uint8 opcode;                    //!< GAP type of command. Ref: @ref GAP_MSG_EVENT_DEFINES
} gapEventHdr_t;

/**
    GAP_RANDOM_ADDR_CHANGED_EVENT message format.  This message is sent to the
    app when the random address changes.
*/
typedef struct
{
    osal_event_hdr_t  hdr;              //!< GAP_MSG_EVENT and status
    uint8 opcode;                       //!< GAP_RANDOM_ADDR_CHANGED_EVENT
    uint8 addrType;                     //!< Address type: @ref GAP_ADDR_TYPE_DEFINES
    uint8 newRandomAddr[B_ADDR_LEN];    //!< the new calculated private addr
} gapRandomAddrEvent_t;

/**
    Connection parameters for the peripheral device.  These numbers are used
    to compare against connection events and request connection parameter
    updates with the master.
*/
typedef struct
{
    /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
    uint16 intervalMin;
    /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
    uint16 intervalMax;
    /** Number of LL latency connection events (0x0000 - 0x03e8) */
    uint16 latency;
    /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
    uint16 timeout;
} gapPeriConnectParams_t;

/**
    GAP_DEVICE_INIT_DONE_EVENT message format.  This message is sent to the
    app when the Device Initialization is done [initiated by calling
    GAP_DeviceInit()].
*/
typedef struct
{
    osal_event_hdr_t  hdr;              //!< GAP_MSG_EVENT and status
    uint8 opcode;                       //!< GAP_DEVICE_INIT_DONE_EVENT
    uint8 devAddr[B_ADDR_LEN];          //!< Device's BD_ADDR
    uint16 dataPktLen;                  //!< HC_LE_Data_Packet_Length
    uint8 numDataPkts;                  //!< HC_Total_Num_LE_Data_Packets
} gapDeviceInitDoneEvent_t;

/**
    GAP_SIGNATURE_UPDATED_EVENT message format.  This message is sent to the
    app when the signature counter has changed.  This message is to inform the
    application in case it wants to save it to be restored on reboot or reconnect.
    This message is sent to update a connection's signature counter and to update
    this device's signature counter.  If devAddr == BD_ADDR, then this message pertains
    to this device.
*/
typedef struct
{
    osal_event_hdr_t  hdr;              //!< GAP_MSG_EVENT and status
    uint8 opcode;                       //!< GAP_SIGNATURE_UPDATED_EVENT
    uint8 addrType;                     //!< Device's address type for devAddr
    uint8 devAddr[B_ADDR_LEN];          //!< Device's BD_ADDR, could be own address
    uint32 signCounter;                 //!< new Signed Counter
} gapSignUpdateEvent_t;

/**
    GAP_DEVICE_INFO_EVENT message format.  This message is sent to the
    app during a Device Discovery Request, when a new advertisement or scan
    response is received.
*/
typedef struct
{
    osal_event_hdr_t  hdr;    //!< GAP_MSG_EVENT and status
    uint8 opcode;             //!< GAP_DEVICE_INFO_EVENT
    uint8 eventType;          //!< Advertisement Type: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES
    uint8 addrType;           //!< address type: @ref GAP_ADDR_TYPE_DEFINES
    uint8 addr[B_ADDR_LEN];   //!< Address of the advertisement or SCAN_RSP
    int8 rssi;                //!< Advertisement or SCAN_RSP RSSI
    uint8 dataLen;            //!< Length (in bytes) of the data field (evtData)
    uint8* pEvtData;          //!< Data field of advertisement or SCAN_RSP
} gapDeviceInfoEvent_t;

/*  -------------------------------------------------------------------
    TYPEDEFS - Device Discovery
*/

/**
    Type of device discovery (Scan) to perform.
*/
typedef struct
{
    uint8 taskID;       //!< Requesting App's Task ID, used to return results
    uint8 mode;         //!< Discovery Mode: @ref GAP_DEVDISC_MODE_DEFINES
    uint8 activeScan;   //!< TRUE for active scanning
    uint8 whiteList;    //!< TRUE to only allow advertisements from devices in the white list.
} gapDevDiscReq_t;

/**
    Type of device discovery (Scan) to perform.
*/
typedef struct
{
    uint8 eventType;        //!< Indicates advertising event type used by the advertiser: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES
    uint8 addrType;         //!< Address Type: @ref GAP_ADDR_TYPE_DEFINES
    uint8 addr[B_ADDR_LEN]; //!< Device's Address
} gapDevRec_t;

/**
    GAP_DEVICE_DISCOVERY_EVENT message format. This message is sent to the
    Application after a scan is performed.
*/
typedef struct
{
    osal_event_hdr_t  hdr; //!< GAP_MSG_EVENT and status
    uint8 opcode;          //!< GAP_DEVICE_DISCOVERY_EVENT
    uint8 numDevs;         //!< Number of devices found during scan
    gapDevRec_t* pDevList; //!< array of device records
} gapDevDiscEvent_t;

/**
    Advertising Parameters
*/
typedef struct
{
    uint8 eventType;          //!< Advertise Event Type: @ref GAP_ADVERTISEMENT_TYPE_DEFINES
    uint8 initiatorAddrType;  //!< Initiator's address type: @ref GAP_ADDR_TYPE_DEFINES
    uint8 initiatorAddr[B_ADDR_LEN];  //!< Initiator's addr - used only with connectable directed eventType (ADV_EVTTYPE_CONNECTABLE_DIRECTED).
    uint8 channelMap;         //!< Channel Map: Bit mask @ref GAP_ADVCHAN_DEFINES
    uint8 filterPolicy;       //!< Filer Policy: @ref GAP_FILTER_POLICY_DEFINES. Ignored when directed advertising is used.
} gapAdvertisingParams_t;

/**
    GAP_MAKE_DISCOVERABLE_DONE_EVENT message format.  This message is sent to the
    app when the Advertise config is complete.
*/
typedef struct
{
    osal_event_hdr_t  hdr; //!< GAP_MSG_EVENT and status
    uint8 opcode;          //!< GAP_MAKE_DISCOVERABLE_DONE_EVENT
    uint16 interval;       //!< actual advertising interval selected by controller
} gapMakeDiscoverableRspEvent_t;

/**
    GAP_END_DISCOVERABLE_DONE_EVENT message format.  This message is sent to the
    app when the Advertising has stopped.
*/
typedef struct
{
    osal_event_hdr_t  hdr; //!< GAP_MSG_EVENT and status
    uint8 opcode;          //!< GAP_END_DISCOVERABLE_DONE_EVENT
} gapEndDiscoverableRspEvent_t;

/**
    GAP_ADV_DATA_UPDATE_DONE_EVENT message format.  This message is sent to the
    app when Advertising Data Update is complete.
*/
typedef struct
{
    osal_event_hdr_t  hdr; //!< GAP_MSG_EVENT and status
    uint8 opcode;          //!< GAP_ADV_DATA_UPDATE_DONE_EVENT
    uint8 adType;          //!< TRUE if advertising data, FALSE if SCAN_RSP
} gapAdvDataUpdateEvent_t;

/*  -------------------------------------------------------------------
    TYPEDEFS - Link Establishment
*/

/**
    Establish Link Request parameters
*/
typedef struct
{
    uint8 taskID;               //!< Requesting App/Profile's Task ID
    uint8 highDutyCycle;        //!< TRUE to high duty cycle scan, FALSE if not.
    uint8 whiteList;            //!< Determines use of the white list: @ref GAP_WHITELIST_DEFINES
    uint8 addrTypePeer;         //!< Address type of the advertiser: @ref GAP_ADDR_TYPE_DEFINES
    uint8 peerAddr[B_ADDR_LEN]; //!< Advertiser's address
} gapEstLinkReq_t;

/**
    Update Link Parameters Request parameters
*/
typedef struct
{
    uint16 connectionHandle; //!< Connection handle of the update
    uint16 intervalMin;      //!< Minimum Connection Interval
    uint16 intervalMax;      //!< Maximum Connection Interval
    uint16 connLatency;      //!< Connection Latency
    uint16 connTimeout;      //!< Connection Timeout
} gapUpdateLinkParamReq_t;

/**
    GAP_LINK_ESTABLISHED_EVENT message format.  This message is sent to the app
    when the link request is complete.<BR>
    <BR>
    For an Observer, this message is sent to complete the Establish Link Request.<BR>
    For a Peripheral, this message is sent to indicate that a link has been created.
*/
typedef struct
{
    osal_event_hdr_t  hdr;     //!< GAP_MSG_EVENT and status
    uint8 opcode;              //!< GAP_LINK_ESTABLISHED_EVENT
    uint8 devAddrType;         //!< Device address type: @ref GAP_ADDR_TYPE_DEFINES
    uint8 devAddr[B_ADDR_LEN]; //!< Device address of link
    uint16 connectionHandle;   //!< Connection Handle from controller used to ref the device
    uint16 connInterval;       //!< Connection Interval
    uint16 connLatency;        //!< Conenction Latency
    uint16 connTimeout;        //!< Connection Timeout
    uint8 clockAccuracy;       //!< Clock Accuracy
} gapEstLinkReqEvent_t;

/**
    GAP_LINK_PARAM_UPDATE_EVENT message format.  This message is sent to the app
    when the connection parameters update request is complete.
*/
typedef struct
{
    osal_event_hdr_t hdr;     //!< GAP_MSG_EVENT and status
    uint8 opcode;             //!< GAP_LINK_PARAM_UPDATE_EVENT
    uint8 status;             //!< bStatus_t
    uint16 connectionHandle;  //!< Connection handle of the update
    uint16 connInterval;      //!< Requested connection interval
    uint16 connLatency;       //!< Requested connection latency
    uint16 connTimeout;       //!< Requested connection timeout
} gapLinkUpdateEvent_t;

/**
    GAP_LINK_TERMINATED_EVENT message format.  This message is sent to the
    app when a link to a device is terminated.
*/
typedef struct
{
    osal_event_hdr_t  hdr;   //!< GAP_MSG_EVENT and status
    uint8 opcode;            //!< GAP_LINK_TERMINATED_EVENT
    uint16 connectionHandle; //!< connection Handle
    uint8 reason;            //!< termination reason from LL
} gapTerminateLinkEvent_t;

/*  -------------------------------------------------------------------
    TYPEDEFS - Authentication, Bounding and Pairing
*/

/**
    GAP_PASSKEY_NEEDED_EVENT message format.  This message is sent to the
    app when a Passkey is needed from the app's user interface.
*/
typedef struct
{
    osal_event_hdr_t  hdr;        //!< GAP_MSG_EVENT and status
    uint8 opcode;                 //!< GAP_PASSKEY_NEEDED_EVENT
    uint8 deviceAddr[B_ADDR_LEN]; //!< address of device to pair with, and could be either public or random.
    uint16 connectionHandle;      //!< Connection handle
    uint8 uiInputs;               //!< Pairing User Interface Inputs - Ask user to input passcode
    uint8 uiOutputs;              //!< Pairing User Interface Outputs - Display passcode
} gapPasskeyNeededEvent_t;

/**
    GAP_AUTHENTICATION_COMPLETE_EVENT message format.  This message is sent to the app
    when the authentication request is complete.
*/
typedef struct
{
    osal_event_hdr_t  hdr;           //!< GAP_MSG_EVENT and status
    uint8 opcode;                    //!< GAP_AUTHENTICATION_COMPLETE_EVENT
    uint16 connectionHandle;         //!< Connection Handle from controller used to ref the device
    uint8 authState;                 //!< TRUE if the pairing was authenticated (MITM)
    smSecurityInfo_t* pSecurityInfo; //!< BOUND - security information from this device
    smSigningInfo_t* pSigningInfo;   //!< Signing information
    smSecurityInfo_t* pDevSecInfo;   //!< BOUND - security information from connected device
    smIdentityInfo_t* pIdentityInfo; //!< BOUND - identity information
} gapAuthCompleteEvent_t;

/**
    securityInfo and identityInfo are only used if secReqs.bondable == BOUND, which means that
    the device is already bound and we should use the security information and keys.
*/
typedef struct
{
    uint16 connectionHandle;      //!< Connection Handle from controller,
    smLinkSecurityReq_t  secReqs; //!< Pairing Control info
} gapAuthParams_t;

/**
    GAP_SLAVE_REQUESTED_SECURITY_EVENT message format.  This message is sent to the app
    when a Slave Security Request is received.
*/
typedef struct
{
    osal_event_hdr_t  hdr;        //!< GAP_MSG_EVENT and status
    uint8 opcode;                 //!< GAP_SLAVE_REQUESTED_SECURITY_EVENT
    uint16 connectionHandle;      //!< Connection Handle
    uint8 deviceAddr[B_ADDR_LEN]; //!< address of device requesting security
    uint8 authReq;                //!< Authentication Requirements: Bit 2: MITM, Bits 0-1: bonding (0 - no bonding, 1 - bonding)

} gapSlaveSecurityReqEvent_t;

/**
    GAP_BOND_COMPLETE_EVENT message format.  This message is sent to the
    app when a bonding is complete.  This means that a key is loaded and the link is encrypted.
*/
typedef struct
{
    osal_event_hdr_t  hdr;   //!< GAP_MSG_EVENT and status
    uint8 opcode;            //!< GAP_BOND_COMPLETE_EVENT
    uint16 connectionHandle; //!< connection Handle
} gapBondCompleteEvent_t;

/**
    Pairing Request fields - the parsed fields of the SMP Pairing Request command.
*/
typedef struct
{
    uint8 ioCap;         //!< Pairing Request ioCap field
    uint8 oobDataFlag;   //!< Pairing Request OOB Data Flag field
    uint8 authReq;       //!< Pairing Request Auth Req field
    uint8 maxEncKeySize; //!< Pairing Request Maximum Encryption Key Size field
    keyDist_t keyDist;   //!< Pairing Request Key Distribution field
} gapPairingReq_t;

/**
    GAP_PAIRING_REQ_EVENT message format.<BR>
    <BR>
    This message is sent to the
    app when an unexpected Pairing Request is received.  The application is
    expected to setup for a Security Manager pairing/bonding.<BR>
    <BR>
    To setup an SM Pairing, the application should call GAP_Authenticate() with these "pairReq" fields.<BR>
    <BR>
    NOTE: This message should only be sent to peripheral devices.
*/
typedef struct
{
    osal_event_hdr_t hdr;    //!< GAP_MSG_EVENT and status
    uint8 opcode;            //!< GAP_PAIRING_REQ_EVENT
    uint16 connectionHandle; //!< connection Handle
    gapPairingReq_t pairReq; //!< The Pairing Request fields received.
} gapPairingReqEvent_t;

/**
    GAP Advertisement/Scan Response Data Token - These data items are stored as low byte first (OTA
    format).  The data space for these items are passed in and maintained by
    the calling application
*/
typedef struct
{
    uint8 adType;     //!< ADTYPE value: @ref GAP_ADTYPE_DEFINES
    uint8 attrLen;    //!< Number of bytes in the attribute data
    uint8* pAttrData; //!< pointer to Attribute data
} gapAdvDataToken_t;

/** @} End BLE_GAP_DEFINES */

/*  -------------------------------------------------------------------
    GLOBAL VARIABLES
*/

/**
    @defgroup GAP_API GAP API Functions

    @{
*/

/*  -------------------------------------------------------------------
    FUNCTIONS - Initialization and Configuation
*/

/**
    @brief       Called to setup the device.  Call just once on initialization.

     NOTE: When initialization is complete, the calling app will be
           sent the GAP_DEVICE_INIT_DONE_EVENT

    @param       taskID - Default task ID to send events.
    @param       profileRole - GAP Profile Roles: @ref GAP_PROFILE_ROLE_DEFINES
    @param       maxScanResponses - maximum number to scan responses
                  we can receive during a device discovery.
    @param       pIRK - pointer to Identity Root Key, NULLKEY (all zeroes) if the app
                  wants the GAP to generate the key.
    @param       pSRK - pointer to Sign Resolving Key, NULLKEY if the app
                  wants the GAP to generate the key.
    @param       pSignCounter - 32 bit value used in the SM Signing
                  algorithm that shall be initialized to zero and incremented
                  with every new signing. This variable must also be maintained
                  by the application.

    @return      SUCCESS - Processing, expect GAP_DEVICE_INIT_DONE_EVENT, <BR>
                INVALIDPARAMETER - for invalid profile role or role combination, <BR>
                bleIncorrectMode - trouble communicating with HCI
*/
extern bStatus_t GAP_DeviceInit(  uint8 taskID,
                                  uint8 profileRole,
                                  uint8 maxScanResponses,
                                  uint8* pIRK,
                                  uint8* pSRK,
                                  uint32* pSignCounter );

/**
    @brief       Called to setup a GAP Advertisement/Scan Response data token.

    NOTE:        The data in these items are stored as low byte first (OTA format).
                The passed in structure "token" should be allocated by the calling app/profile
                and not released until after calling GAP_RemoveAdvToken().

    @param       pToken - Advertisement/Scan response token to write.

    @return      SUCCESS - advertisement token added to the GAP list <BR>
                INVALIDPARAMETER - Invalid Advertisement Type or pAttrData is NULL <BR>
                INVALID_MEM_SIZE - The tokens take up too much space and don't fit into Advertisment data and Scan Response Data<BR>
                bleInvalidRange - token ID already exists.<BR>
                bleIncorrectMode - not a peripheral device<BR>
                bleMemAllocError - memory allocation failure,
*/
extern bStatus_t GAP_SetAdvToken( gapAdvDataToken_t* pToken );

/**
    @brief       Called to read a GAP Advertisement/Scan Response data token.

    @param       adType - Advertisement type to get

    @return      pointer to the advertisement data token structure, NULL if not found.
*/
extern gapAdvDataToken_t* GAP_GetAdvToken( uint8 adType );

/**
    @brief       Called to remove a GAP Advertisement/Scan Response data token.

    @param       adType - Advertisement type to remove

    @return      pointer to the token structure removed from the GAP ADType list
                NULL if the requested adType wasn't found.
*/
extern gapAdvDataToken_t* GAP_RemoveAdvToken( uint8 adType );

/**
    @brief       Called to rebuild and load Advertisement and Scan Response data from existing
                GAP Advertisement Tokens.

    @return      SUCCESS or bleIncorrectMode
*/
extern bStatus_t GAP_UpdateAdvTokens( void );

/**
    @brief       Set a GAP Parameter value.  Use this function to change
                 the default GAP parameter values.

    @param       paramID - parameter ID: @ref GAP_PARAMETER_ID_DEFINES
    @param       paramValue - new param value

    @return      SUCCESS or INVALIDPARAMETER (invalid paramID)
*/
extern bStatus_t GAP_SetParamValue( gapParamIDs_t paramID, uint16 paramValue );

/**
    @brief       Get a GAP Parameter value.

    @param       paramID - parameter ID: @ref GAP_PARAMETER_ID_DEFINES

    @return      GAP Parameter value or 0xFFFF if invalid
*/
extern uint16 GAP_GetParamValue( gapParamIDs_t paramID );

/**
    @brief       Setup the device's address type.  If ADDRTYPE_PRIVATE_RESOLVE
                is selected, the address will change periodically.

    @param       addrType - @ref GAP_ADDR_TYPE_DEFINES
    @param       pStaticAddr - Only used with ADDRTYPE_STATIC
                         or ADDRTYPE_PRIVATE_NONRESOLVE type.<BR>
                     NULL to auto generate otherwise the application
                     can specify the address value

    @return      SUCCESS: address type updated,<BR>
                bleNotReady: Can't be called until GAP_DeviceInit() is called
                     and the init process is completed,<BR>
                bleIncorrectMode: can't change with an active connection,<BR>
                 or INVALIDPARAMETER.<BR>

                If return value isn't SUCCESS, the address type remains
                the same as before this call.
*/
extern bStatus_t GAP_ConfigDeviceAddr( uint8 addrType, uint8* pStaticAddr );

/**
    @brief       Register your task ID to receive extra (unwanted)
                HCI status and complete events.

    @param       taskID - Default task ID to send events.

    @return      none
*/
extern void GAP_RegisterForHCIMsgs( uint8 taskID );

/*  -------------------------------------------------------------------
    FUNCTIONS - Device Discovery
*/

/**
    @brief       Start a device discovery scan.

    @param       pParams - Device Discovery parameters

    @return      SUCCESS: scan started,<BR>
                bleIncorrectMode: invalid profile role,<BR>
                bleAlreadyInRequestedMode: not available<BR>
*/
extern bStatus_t GAP_DeviceDiscoveryRequest( gapDevDiscReq_t* pParams );

/**
    @brief       Cancel an existing device discovery request.

    @param       taskID - used to return GAP_DEVICE_DISCOVERY_EVENT

    @return      SUCCESS: cancel started,<BR>
                bleInvalidTaskID: Not the task that started discovery,<BR>
                bleIncorrectMode: not in discovery mode<BR>
*/
extern bStatus_t GAP_DeviceDiscoveryCancel( uint8 taskID );

/**
    @brief       Setup or change advertising.  Also starts advertising.

    @param       taskID - used to return GAP_DISCOVERABLE_RESPONSE_EVENT
    @param       pParams - advertising parameters

    @return      SUCCESS: advertising started,<BR>
                bleIncorrectMode: invalid profile role,<BR>
                bleAlreadyInRequestedMode: not available at this time,<BR>
                bleNotReady: advertising data isn't set up yet.<BR>
*/
extern bStatus_t GAP_MakeDiscoverable( uint8 taskID, gapAdvertisingParams_t* pParams );

/**
    @brief       Setup or change advertising and scan response data.

      NOTE:  if the return status from this function is SUCCESS,
             the task isn't complete until the GAP_ADV_DATA_UPDATE_DONE_EVENT
             is sent to the calling application task.

    @param       taskID - task ID of the app requesting the change
    @param       adType - TRUE - advertisement data, FALSE  - scan response data
    @param       dataLen - Octet length of advertData
    @param       pAdvertData - advertising or scan response data

    @return      SUCCESS: data accepted,<BR>
                bleIncorrectMode: invalid profile role,<BR>
*/
extern bStatus_t GAP_UpdateAdvertisingData( uint8 taskID, uint8 adType,
                                            uint8 dataLen, uint8* pAdvertData );

/**
    @brief       Stops advertising.

    @param       taskID - of task that called GAP_MakeDiscoverable

    @return      SUCCESS: stopping discoverable mode,<BR>
                bleIncorrectMode: not in discoverable mode,<BR>
                bleInvalidTaskID: not correct task<BR>
*/
extern bStatus_t GAP_EndDiscoverable( uint8 taskID );

/**
    @brief       Resolves a private address against an IRK.

    @param       pIRK - pointer to the IRK
    @param       pAddr - pointer to the Resovable Private address

    @return      SUCCESS: match,<BR>
                FAILURE: don't match,<BR>
                INVALIDPARAMETER: parameters invalid<BR>
*/
extern bStatus_t GAP_ResolvePrivateAddr( uint8* pIRK, uint8* pAddr );

/*  -------------------------------------------------------------------
    FUNCTIONS - Link Establishment
*/

/**
    @brief       Establish a link to a slave device.

    @param       pParams - link establishment parameters

    @return      SUCCESS: started establish link process,<BR>
                bleIncorrectMode: invalid profile role,<BR>
                bleNotReady: a scan is in progress,<BR>
                bleAlreadyInRequestedMode: can’t process now,<BR>
                bleNoResources: Too many links<BR>
*/
extern bStatus_t GAP_EstablishLinkReq( gapEstLinkReq_t* pParams );

/**
    @brief       Terminate a link connection.

    @param       taskID - requesting app's task id.
    @param       connectionHandle - connection handle of link to terminate
                    or @ref GAP_CONN_HANDLE_DEFINES
    @param       reason - terminate reason.

    @return      SUCCESS: Terminate started,<BR>
                bleIncorrectMode: No Link to terminate,<BR>
                bleInvalidTaskID: not app that established link<BR>
*/
extern bStatus_t GAP_TerminateLinkReq( uint8 taskID, uint16 connectionHandle, uint8 reason );

/**
    @brief       Update the link parameters to a slave device.

    @param       pParams - link update parameters

    @return      SUCCESS: started update link process,<BR
                INVALIDPARAMETER: one of the parameters were invalid,<BR>
                bleIncorrectMode: invalid profile role,<BR>
                bleNotConnected: not in a connection<BR>
*/
extern bStatus_t GAP_UpdateLinkParamReq( gapUpdateLinkParamReq_t* pParams );

/**
    @brief       Returns the number of active connections.

    @return      Number of active connections.
*/
extern uint8 GAP_NumActiveConnections( void );

/*  -------------------------------------------------------------------
    FUNCTIONS - Pairing
*/

/**
    @brief       Start the Authentication process with the requested device.
                This function is used to Initiate/Allow pairing.
                Called by both master and slave device (Central and Peripheral).

    NOTE:        This function is called after the link is established.

    @param       pParams - Authentication parameters
    @param       pPairReq - Enter these parameters if the Pairing Request was already received.
                NULL, if waiting for Pairing Request or if initiating.

    @return      SUCCESS,<BR>
                bleIncorrectMode: Not correct profile role,<BR>
                INVALIDPARAMETER, <BR>
                bleNotConnected,<BR>
                bleAlreadyInRequestedMode,<BR>
                FAILURE - not workable.<BR>
*/
extern bStatus_t GAP_Authenticate( gapAuthParams_t* pParams, gapPairingReq_t* pPairReq );

/**
    @brief       Send a Pairing Failed message and end any existing pairing.

    @param       connectionHandle - connection handle.
    @param       reason - Pairing Failed reason code.

    @return      SUCCESS - function was successful,<BR>
                bleMemAllocError - memory allocation error,<BR>
                INVALIDPARAMETER - one of the parameters were invalid,<BR>
                bleNotConnected - link not found,<BR>
                bleInvalidRange - one of the parameters were not within range.
*/
extern bStatus_t GAP_TerminateAuth( uint16 connectionHandle, uint8 reason );

/**
    @brief       Update the passkey in string format.  This function is called by the
                application/profile in response to receiving the
                GAP_PASSKEY_NEEDED_EVENT message.

    NOTE:        This function is the same as GAP_PasscodeUpdate(), except that
                the passkey is passed in as a string format.

    @param       pPasskey - new passkey - pointer to numeric string (ie. "019655" ).
                This string's range is "000000" to "999999".
    @param       connectionHandle - connection handle.

    @return      SUCCESS: will start pairing with this entry,<BR>
                bleIncorrectMode: Link not found,<BR>
                INVALIDPARAMETER: passkey == NULL or passkey isn't formatted properly.<BR>
*/
extern bStatus_t GAP_PasskeyUpdate( uint8* pPasskey, uint16 connectionHandle );

/**
    @brief       Update the passkey in a numeric value (not string).
                This function is called by the application/profile in response
                to receiving the GAP_PASSKEY_NEEDED_EVENT message.

    NOTE:        This function is the same as GAP_PasskeyUpdate(), except that
                the passkey is passed in as a non-string format.

    @param       passcode - not string - range: 0 - 999,999.
    @param       connectionHandle - connection handle.

    @return      SUCCESS: will start pairing with this entry,<BR>
                bleIncorrectMode: Link not found,<BR>
                INVALIDPARAMETER: passkey == NULL or passkey isn't formatted properly.<BR>
*/
extern bStatus_t GAP_PasscodeUpdate( uint32 passcode, uint16 connectionHandle );

/**
    @brief       Generate a Slave Requested Security message to the master.

    @param       connectionHandle - connection handle.
    @param       authReq - Authentication Requirements: Bit 2: MITM, Bits 0-1: bonding (0 - no bonding, 1 - bonding)

    @return      SUCCESS: will send,<BR>
                bleNotConnected: Link not found,<BR>
                bleIncorrectMode: wrong GAP role, must be a Peripheral Role<BR>
*/
extern bStatus_t GAP_SendSlaveSecurityRequest( uint16 connectionHandle, uint8 authReq );

/**
    @brief       Set up the connection to accept signed data.

    NOTE:        This function is called after the link is established.

    @param       connectionHandle - connection handle of the signing information
    @param       authenticated - TRUE if the signing information is authenticated, FALSE otherwise
    @param       pParams - signing parameters

    @return      SUCCESS, <BR>
                bleIncorrectMode: Not correct profile role,<BR>
                INVALIDPARAMETER, <BR>
                bleNotConnected,<BR>
                FAILURE: not workable.<BR>
*/
extern bStatus_t GAP_Signable( uint16 connectionHandle, uint8 authenticated, smSigningInfo_t* pParams );

/**
    @brief       Set up the connection's bound paramaters.

    NOTE:        This function is called after the link is established.

    @param       connectionHandle - connection handle of the signing information
    @param       authenticated - this connection was previously authenticated
    @param       pParams - the connected device's security parameters
    @param       startEncryption - whether or not to start encryption

    @return      SUCCESS, <BR>
                bleIncorrectMode: Not correct profile role,<BR>
                INVALIDPARAMETER, <BR>
                bleNotConnected,<BR>
                FAILURE: not workable.<BR>
*/
extern bStatus_t GAP_Bond( uint16 connectionHandle, uint8 authenticated,
                           smSecurityInfo_t* pParams, uint8 startEncryption );

/**
    @} End GAP_API
*/

/*  -------------------------------------------------------------------
    Internal API - These functions are only called from gap.c module.
*/

/**
    @internal

    @brief       Setup the device configuration parameters.

    @param       taskID - Default task ID to send events.
    @param       profileRole - GAP Profile Roles

    @return      SUCCESS or bleIncorrectMode
*/
extern bStatus_t GAP_ParamsInit( uint8 taskID, uint8 profileRole );

/**
    @internal

    @brief       Setup the device security configuration parameters.

    @param       pIRK - pointer to Identity Root Key, NULLKEY (all zeroes) if the app
                  wants the GAP to generate the key.
    @param       pSRK - pointer to Sign Resolving Key, NULLKEY if the app
                  wants the GAP to generate the key.
    @param       pSignCounter - 32 bit value used in the SM Signing
                  algorithm that shall be initialized to zero and incremented
                  with every new signing. This variable must also be maintained
                  by the application.

    @return      none
*/
extern void GAP_SecParamsInit( uint8* pIRK, uint8* pSRK, uint32* pSignCounter );

/**
    @internal

    @brief       Initialize the GAP Peripheral Dev Manager.

    @param       none

    @return      SUCCESS or bleMemAllocError
*/
extern bStatus_t GAP_PeriDevMgrInit( void );

/**
    @internal

    @brief       Initialize the GAP Central Dev Manager.

    @param       maxScanResponses - maximum number to scan responses
                  we can receive during a device discovery.

    @return      SUCCESS or bleMemAllocError
*/
extern bStatus_t GAP_CentDevMgrInit( uint8 maxScanResponses );

/**
    @internal

    @brief       Register the GAP Central Connection processing functions.

    @param       none

    @return      none
*/
extern void GAP_CentConnRegister( void );


/*  -------------------------------------------------------------------
    TASK API - These functions must only be called OSAL.
*/

/**
    @internal

    @brief       GAP Task initialization function.

    @param       taskID - GAP task ID.

    @return      void
*/
extern void GAP_Init( uint8 task_id );

/**
    @internal

    @brief       GAP Task event processing function.

    @param       taskID - GAP task ID
    @param       events - GAP events.

    @return      events not processed
*/
extern uint16 GAP_ProcessEvent( uint8 task_id, uint16 events );


/*  -------------------------------------------------------------------
    -------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* GAP_H */
