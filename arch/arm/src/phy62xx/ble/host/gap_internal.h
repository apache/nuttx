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

/**************************************************************************************************
**************************************************************************************************/

#ifndef GAP_INTERNAL_H
#define GAP_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "hci.h"
#include "l2cap.h"
#include "gap.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

// GAP OSAL Events
#define GAP_OSAL_TIMER_SCAN_DURATION_EVT        0x0001
#define GAP_END_ADVERTISING_EVT                 0x0002
#define GAP_CHANGE_RESOLVABLE_PRIVATE_ADDR_EVT  0x0004

#define ADDRTYPE_RANDOM                         1  // Not public

#define GAP_PRIVATE_ADDR_CHANGE_RESOLUTION      0xEA60 // Timer resolution is 1 minute

#define ADV_TOKEN_HDR   2

// Address header bits
#define RANDOM_ADDR_HDR                       0xC0  // Used for LL RANDOM Address
#define STATIC_ADDR_HDR                       0xC0  // Host Static Address, same as RANDOM address
#define PRIVATE_RESOLVE_ADDR_HDR              0x40

#if defined ( TESTMODES )
// GAP TestModes
#define GAP_TESTMODE_OFF                        0 // No Test mode
#define GAP_TESTMODE_NO_RESPONSE                1 // Don't respond to any GAP message
#endif  // TESTMODES

// L2CAP Connection Parameters Update Request event
#define L2CAP_PARAM_UPDATE                      0xFFFF

/*********************************************************************
    TYPEDEFS
*/

typedef struct gapAdvToken
{
    struct gapAdvToken* pNext;     // Pointer to next item in link list
    gapAdvDataToken_t*  pToken;    // Pointer to data token
} gapAdvToken_t;

/** Advertising and Scan Response Data **/
typedef struct
{
    uint8   dataLen;                  // Number of bytes used in "dataField"
    uint8   dataField[B_MAX_ADV_LEN]; // Data field of the advertisement or SCAN_RSP
} gapAdvertisingData_t;

typedef struct
{
    uint8   dataLen;       // length (in bytes) of "dataField"
    uint8   dataField[1];  // This is just a place holder size
    // The dataField will be allocated bigger
} gapAdvertRecData_t;

// Temporary advertising record
typedef struct
{
    uint8  eventType;               // Avertisement or SCAN_RSP
    uint8  addrType;                // Advertiser's address type
    uint8  addr[B_ADDR_LEN];        // Advertiser's address
    gapAdvertRecData_t* pAdData;    // Advertising data field. This space is allocated.
    gapAdvertRecData_t* pScanData;  // SCAN_RSP data field. This space is allocated.
} gapAdvertRec_t;

typedef enum
{
    GAP_ADSTATE_SET_PARAMS,     // Setting the advertisement parameters
    GAP_ADSTATE_SET_MODE,       // Turning on advertising
    GAP_ADSTATE_ADVERTISING,    // Currently Advertising
    GAP_ADSTATE_ENDING          // Turning off advertising
} gapAdvertStatesIDs_t;

// Advertising State Information
typedef struct
{
    uint8 taskID;                   // App that started an advertising period
    gapAdvertStatesIDs_t state;     // Make Discoverable state
    gapAdvertisingParams_t params;  // Advertisement parameters
} gapAdvertState_t;

typedef struct
{
    uint8                state;            // Authentication states
    uint16               connectionHandle; // Connection Handle from controller,
    smLinkSecurityReq_t  secReqs;          // Pairing Control info

    // The following are only used if secReqs.bondable == BOUND, which means that
    // the device is already bound and we should use the security information and
    // keys
    smSecurityInfo_t*     pSecurityInfo;    // BOUND - security information
    smIdentityInfo_t*     pIdentityInfo;    // BOUND - identity information
    smSigningInfo_t*      pSigningInfo;     // Signing information
} gapAuthStateParams_t;

// Callback when an HCI Command Event has been received on the Central.
typedef uint8 (*gapProcessHCICmdEvt_t)( uint16 cmdOpcode, hciEvt_CmdComplete_t* pMsg );

// Callback when an Scanning Report has been received on the Central.
typedef void (*gapProcessScanningEvt_t)( hciEvt_BLEAdvPktReport_t* pMsg );

// Callback to cancel a connection initiation on the Central.
typedef bStatus_t (*gapCancelLinkReq_t)( uint8 taskID, uint16 connectionHandle );

// Callback when a connection-related event has been received on the Central.
typedef uint8(*gapProcessConnEvt_t)( uint16 cmdOpcode, hciEvt_CommandStatus_t* pMsg );

// Callback when an HCI Command Command Event on the Peripheral.
typedef uint8 (*gapProcessHCICmdCompleteEvt_t)( hciEvt_CmdComplete_t* pMsg );

// Callback when an Advertising Event has been received on the Peripheral.
typedef void (*gapProcessAdvertisingEvt_t)( uint8 timeout );

// Callback when a Set Advertising Params has been received on the Peripheral.
typedef bStatus_t (*gapSetAdvParams_t)( void );

// Central callback structure - must be setup by the Central.
typedef struct
{
    gapProcessHCICmdEvt_t   pfnProcessHCICmdEvt;   // When HCI Command Event received
    gapProcessScanningEvt_t pfnProcessScanningEvt; // When Scanning Report received
} gapCentralCBs_t;

// Central connection-related callback structure - must be setup by the Central.
typedef struct
{
    gapCancelLinkReq_t  pfnCancelLinkReq;  // When cancel connection initiation requested
    gapProcessConnEvt_t pfnProcessConnEvt; // When connection-related event received
} gapCentralConnCBs_t;

// Peripheral callback structure - must be setup by the Peripheral.
typedef struct
{
    gapProcessHCICmdCompleteEvt_t pfnProcessHCICmdCompleteEvt; // When HCI Command Complete Event received
    gapProcessAdvertisingEvt_t    pfnProcessAdvertisingEvt;    // When Advertising Event received
    gapSetAdvParams_t             pfnSetAdvParams;             // When Set Advertising Params received
} gapPeripheralCBs_t;

/*********************************************************************
    GLOBAL VARIABLES
*/

extern uint8 gapTaskID;
extern uint8 gapUnwantedTaskID;

extern uint8 gapAppTaskID;         // default task ID to send events
extern uint8 gapProfileRole;       // device GAP Profile Role(s)

extern uint8 gapDeviceAddrMode;   //  ADDRTYPE_PUBLIC, ADDRTYPE_STATIC,
//  ADDRTYPE_PRIVATE_NONRESOLVE
//  or ADDRTYPE_PRIVATE_RESOLVE

// Central Peripheral variables
extern gapDevDiscReq_t* pGapDiscReq;
extern gapEstLinkReq_t* pEstLink;
extern gapCentralConnCBs_t* pfnCentralConnCBs;

// Peripheral variables
extern gapAdvertState_t* pGapAdvertState;
extern gapPeripheralCBs_t* pfnPeripheralCBs;

// Common variables
extern gapAuthStateParams_t* pAuthLink[];
extern uint16 gapPrivateAddrChangeTimeout;
extern uint8 gapAutoAdvPrivateAddrChange;

/*********************************************************************
    FUNCTIONS - API
*/

/*********************************************************************
    Application Level Functions
*/

/*
    gapSetScanParamStatus - Process HCI Command Complete Event status for
                the call to HCI_BLESetScanParamCmd().
*/
extern uint8 gapSetScanParamStatus( uint8 status );

/*
    gapSetAdvParamsStatus - Process HCI Command Complete Event status for
                the call to HCI_BLESetAdvParamCmd().
*/
extern uint8 gapSetAdvParamsStatus( uint8 status );

/*
    gapWriteAdvEnableStatus - Process HCI Command Complete Event status for
                the call to HCI_BLEWriteAdvEnableCmd().
*/
extern uint8 gapWriteAdvEnableStatus( uint8 status, uint16 interval );

/*
    gapWriteAdvDataStatus - Process HCI Command Complete Event status for
                the call to HCI_BLEWriteAdvDataCmd() or
                HCI_BLEWriteScanRspDataCmd().
*/
extern void gapWriteAdvDataStatus( uint8 adType, uint8 status );

/*
    gapReadBD_ADDRStatus - Process the HCI Command Complete Event for the
                call to HCI_ReadBDADDRCmd().
*/
extern uint8 gapReadBD_ADDRStatus( uint8 status, uint8* pBdAddr );

/*
    gapReadBufSizeCmdStatus - Process the HCI Command Complete Event for the
                call to HCI_BLEReadBufSizeCmd().
*/
extern uint8 gapReadBufSizeCmdStatus( hciRetParam_LeReadBufSize_t* pCmdStat );

/*
    gapProcessConnectionCompleteEvt - Process the HCI Connection Complete
                event for the call to HCI_BLECreateLLConnCmd().
*/
extern void gapProcessConnectionCompleteEvt( hciEvt_BLEConnComplete_t* pPkt );

/*
    gapProcessConnUpdateCompleteEvt - Process the HCI Connection Parameters
                Update Complete event for the call to HCI_BLEUpdateLLConnCmd().
*/
extern void gapProcessConnUpdateCompleteEvt( hciEvt_BLEConnUpdateComplete_t* pPkt );

/*
    gapProcessDisconnectCompleteEvt - Process the LL Disconnection Complete Event
                for the call to HCI_DisconnectCmd().
*/
extern void gapProcessDisconnectCompleteEvt( hciEvt_DisconnComplete_t* pPkt );

/*
    gapProcessCreateLLConnCmdStatus - Process the status for the HCI_BLECreateLLConnCmd().
*/
extern void gapProcessCreateLLConnCmdStatus( uint8 status );

/*
    gapProcessConnUpdateCmdStatus - Process the status for the HCI_LE_ConnUpdateCmd().
*/
extern void gapProcessConnUpdateCmdStatus( uint8 status );

/*
    gapProcessNewAddr - Process message SM
*/
extern bStatus_t gapProcessNewAddr( uint8* pNewAddr );

/*
    gapAddAddrAdj - Add the top two bits based on the address type.
*/
extern uint8 gapAddAddrAdj( uint8 addrType, uint8* pAddr );

/*
    gapDetermineAddrType - Convert from LL address type to host address type.
*/
extern uint8 gapDetermineAddrType( uint8 addrType, uint8* pAddr );

/*
    gapProcessRandomAddrComplete - Process message HCI
*/
extern void gapProcessRandomAddrComplete( uint8 status );

/*
    gapGetSRK - Get pointer to the SRK
*/
extern uint8* gapGetSRK( void );

/*
    gapGetSignCounter - Get the signature counter
*/
extern uint32 gapGetSignCounter( void );

/*
    gapIncSignCounter - Increment the signature counter
*/
extern  void gapIncSignCounter( void );

/*
    gapUpdateConnSignCounter - Update a connection's signature's counter
*/
extern  void gapUpdateConnSignCounter( uint16 connHandle, uint32 newSignCounter );

/*
    gapLinkCheck - linkDB callback function
*/
extern void gapLinkCheck( uint16 connectionHandle, uint8 changeType );

/*
    gapGetDevAddressMode - Get the device address mode.
*/
extern uint8 gapGetDevAddressMode( void );

/*
    gapGetDevAddress - Get the device address.
        real - TRUE if you always want BD_ADDR, FALSE will allow random addresses.
*/
extern uint8* gapGetDevAddress( uint8 real );

/*
    gapGetIRK - Get the device's IRK.
*/
extern uint8* gapGetIRK( void );

/*
    gapPasskeyNeededCB - Callback function to ask for passkey
*/
extern void gapPasskeyNeededCB( uint16 connectionHandle, uint8 type );

/*
    gapPairingCompleteCB - Callback function to inform pairing process complete.
*/
extern void gapPairingCompleteCB( uint8 status, uint8 initiatorRole,
                                  uint16 connectionHandle,
                                  uint8 authState,
                                  smSecurityInfo_t* pEncParams,
                                  smSecurityInfo_t* pDevEncParams,
                                  smIdentityInfo_t* pIdInfo,
                                  smSigningInfo_t*  pSigningInfo );

/*
    gapTerminateConnComplete - Process command complete for HCI_BLECreateLLConnCancelCmd.
*/
extern void gapTerminateConnComplete( void );

/*
    gapSendSlaveSecurityReqEvent - Generate a Slave Security Request event to the app.
*/
extern void gapSendSlaveSecurityReqEvent( uint8 taskID, uint16 connHandle, uint8* pDevAddr, uint8 authReq );

/*
    gapSetAdvParams - Send the advertisement parameters to the LL.
*/
extern bStatus_t gapSetAdvParams( void );

/*
    gapAddAdvToken - Add token to the end of the list.
*/
extern bStatus_t gapAddAdvToken( gapAdvDataToken_t* pToken );

/*
    gapDeleteAdvToken - Remove a token from the list.
*/
extern gapAdvDataToken_t* gapDeleteAdvToken( uint8 ADType );

/*
    gapFindAdvToken - Find a Advertisement data token from the advertisement type.
*/
extern gapAdvToken_t* gapFindAdvToken( uint8 ADType );

/*
    gapCalcAdvTokenDataLen - Find a Advertisement data token from the advertisement type.
*/
extern void gapCalcAdvTokenDataLen( uint8* pAdLen, uint8* pSrLen );

/*
    gapValidADType - Is a Advertisement Data Type valid.
*/
extern uint8 gapValidADType( uint8 adType );

/*
    gapBuildADTokens - Is a Advertisement Data Type valid.
*/
extern bStatus_t gapBuildADTokens( void );

/*
    gapSendBondCompleteEvent - Indicate that a bond has occurred.
*/
extern void gapSendBondCompleteEvent( uint8 status, uint16 connectionHandle );

/*
    gapSendPairingReqEvent - Indicate that an unexpected Pairing Request was received.
*/
extern void gapSendPairingReqEvent( uint8 status, uint16 connectionHandle,
                                    uint8 ioCap,
                                    uint8 oobDataFlag,
                                    uint8 authReq,
                                    uint8 maxEncKeySize,
                                    keyDist_t keyDist );

/*
    gapFindADType - Find Advertisement Data Type field in advertising data
                   field.
*/
extern uint8* gapFindADType( uint8 adType, uint8* pAdLen,
                             uint8 dataLen, uint8* pDataField );

/*
    gapRegisterCentral - Register Central's processing function with GAP task
*/
extern void gapRegisterCentral( gapCentralCBs_t* pfnCBs );

/*
    gapRegisterCentralConn - Register Central's connection-related processing function with GAP task
*/
extern void gapRegisterCentralConn( gapCentralConnCBs_t* pfnCBs);

/*
    gapRegisterPeripheral - Register Peripheral's processing function with GAP task
*/
extern void gapRegisterPeripheral( gapPeripheralCBs_t* pfnCBs );

/*
    gapIsAdvertising - Check if we are currently advertising.
*/
extern uint8 gapIsAdvertising( void );

/*
    gapIsScanning - Check if we are currently scanning.
*/
extern uint8 gapIsScanning( void );

/*
    gapCancelLinkReq - Cancel a connection create request.
*/
extern bStatus_t gapCancelLinkReq( uint8 taskID, uint16 connectionHandle );

/*
    gapFreeEstLink - Free the establish link memory.
*/
extern void gapFreeEstLink( void );

/*
    sendEstLinkEvent - Build and send the GAP_LINK_ESTABLISHED_EVENT to the app.
*/
extern void sendEstLinkEvent( uint8 status, uint8 taskID, uint8 devAddrType,
                              uint8* pDevAddr, uint16 connectionHandle,
                              uint16 connInterval, uint16 connLatency,
                              uint16 connTimeout, uint16 clockAccuracy );

/*
    gapSendLinkUpdateEvent - Build and send the GAP_LINK_PARAM_UPDATE_EVENT to the app.

*/
extern void gapSendLinkUpdateEvent( uint8 status, uint16 connectionHandle,
                                    uint16 connInterval, uint16 connLatency,
                                    uint16 connTimeout );

/*
    gapProcessL2CAPSignalEvt - Process L2CAP Signaling messages.
*/
extern void gapProcessL2CAPSignalEvt( l2capSignalEvent_t* pCmd );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GAP_INTERNAL_H */
