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

#ifndef SM_INTERNAL_H
#define SM_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "l2cap.h"
#include "smp.h"
#include "linkdb.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

// Security Manager Task Events
#define SM_TIMEOUT_EVT            0x0001    // Message timeout event
#define SM_PAIRING_STATE_EVT      0x0002    // Event used to progress to the next pairing state

// Pairing states
#define SM_PAIRING_STATE_INITIALIZE                       0  // Pairing has started
#define SM_PAIRING_STATE_PAIRING_REQ_SENT                 1  // Initiator: Pairing Request has been sent, Responder: waiting for Pairing Request.
#define SM_PAIRING_STATE_WAIT_CONFIRM                     2  // Waiting for Confirm message
#define SM_PAIRING_STATE_WAIT_PASSKEY                     3  // Waiting for Passkey from app/profile
#define SM_PAIRING_STATE_WAIT_CONFIRM_PASSKEY             4  // Received Initiator Confirm message and waiting for Passkey from app/profile (responder only)
#define SM_PAIRING_STATE_WAIT_RANDOM                      5  // Waiting for Random message
#define SM_PAIRING_STATE_WAIT_STK                         6  // Waiting for STK process to finish
#define SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO       7  // Waiting for Slave Encryption Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO           8  // Waiting for Slave Master Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO         9  // Waiting for Slave Identity Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO    10 // Waiting for Slave Identity Addr Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO          11 // Waiting for Slave Signing Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO      12 // Waiting for Master Encryption Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO          13 // Waiting for Master Master Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO        14 // Waiting for Master Identity Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO   15 // Waiting for Master Identity Addr Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO         16 // Waiting for Master Signing Info to be sent
#define SM_PAIRING_STATE_WAIT_ENCRYPT                     17 // Waiting for LTK process to finish
#define SM_PAIRING_STATE_DONE                             18 // Closing out the pairing process

#if defined ( TESTMODES )
// SM TestModes
#define SM_TESTMODE_OFF                           0   // No Test mode
#define SM_TESTMODE_NO_RESPONSE                   1   // Don't respond to any SM message
#define SM_TESTMODE_SEND_BAD_CONFIRM              2   // Force a bad confirm value in the Confirm Message
#define SM_TESTMODE_BAD_CONFIRM_VERIFY            3   // Force a bad confirm check of the received Confirm Message
#define SM_TESTMODE_SEND_CONFIRM                  4   // Force a SMP Confirm message
#endif  // TESTMODES

// Pairing Types
#define SM_PAIRING_TYPE_INIT                        0 // Pairing has been started but the type hasn't been determined yet
#define SM_PAIRING_TYPE_JUST_WORKS                  1 // Pairing is Just Works
#define SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS    2 // Pairing is MITM Passkey with initiator inputs passkey
#define SM_PAIRING_TYPE_PASSKEY_RESPONDER_INPUTS    3 // Pairing is MITM Passkey with responder inputs passkey
#define SM_PAIRING_TYPE_PASSKEY_BOTH_INPUTS         4 // Pairing is MITM Passkey with both initiator and responder input passkey
#define SM_PAIRING_TYPE_OOB                         5 // Pairing is MITM OOB

#define SM_PAIRING_STATE_WAIT                       500 // The default wait time between key distribution messages.

/*********************************************************************
    TYPEDEFS
*/

typedef struct
{
    uint8  confirm[KEYLEN];       // calculated confirm value
    uint8  rand[SMP_RANDOM_LEN];  // First MRand or Srand, then RAND
} devPairing_t;

typedef struct
{
    // From the start
    uint8                initiator;        // TRUE if initiator
    uint8                state;            // Pairing state
    uint8                taskID;           // Task ID of the app/profile that requested the pairing

    uint8                timerID;           // 2021-03-29 add , timerid for simultaneously SMP for multi-role(the same as single connection )
    uint8                stateID;           // 2021-03-29 add , stateid for simultaneously SMP pairing state change idx
    uint16               connectionHandle; // Connection Handle from controller,
    smLinkSecurityReq_t*  pSecReqs;        // Pairing Control info
    uint8                tk[KEYLEN];       // Holds tk from app
    uint8                authState;        // uses SM_AUTH_STATE_AUTHENTICATED & SM_AUTH_STATE_BONDING

    // During pairing
    smpPairingReq_t*      pPairDev;        // Info of paired device.
    uint8                type;             // ie. SM_PAIRING_TYPE_JUST_WORKS

    // device information
    devPairing_t         myComp;          // This device's pairing components
    devPairing_t         devComp;         // The other device's components

    // Encrypt Params
    smSecurityInfo_t*     pEncParams;     // Your (device's) encryption parameters
    smSecurityInfo_t*     pDevEncParams;  // Connected device's encryption parameters
    smIdentityInfo_t*     pIdInfo;        // Connected device's identity parameters
    smSigningInfo_t*      pSigningInfo;    // Connected device's signing parameters

} smPairingParams_t;

// Callback when an SMP message has been received on the Initiator or Responder.
typedef uint8 (*smProcessMsg_t)( linkDBItem_t* pLinkItem, uint8 cmdID, smpMsgs_t* pParsedMsg );

// Callback to send next key message, and sets state for next event on the Initiator or Responder.
typedef void (*smSendNextKeyInfo_t)( uint16 connectionHandle );

// Callback to send Start Encrypt through HCI on the Initiator.
typedef bStatus_t (*smStartEncryption_t)( uint16 connHandle, uint8* pLTK, uint16 div,
                                          uint8* pRandNum, uint8 keyLen );

// Callback when an HCI BLE LTK Request has been received on the Responder.
typedef uint8 (*smProcessLTKReq_t)( uint16 connectionHandle, uint8* pRandom, uint16 encDiv );

// Initiator callback structure - must be setup by the Initiator.
typedef struct
{
    smProcessMsg_t      pfnProcessMsg;      // When SMP message received
    smSendNextKeyInfo_t pfnSendNextKeyInfo; // When need to send next key message
    smStartEncryption_t pfnStartEncryption; // When Start Encrypt requested
} smInitiatorCBs_t;

// Responder callback structure - must be setup by the Initiator.
typedef struct
{
    smProcessMsg_t      pfnProcessMsg;      // When SMP message received
    smSendNextKeyInfo_t pfnSendNextKeyInfo; // When need to send next key message
    smProcessLTKReq_t   pfnProcessLTKReq;   // When HCI BLE LTK Request received
} smResponderCBs_t;

/*********************************************************************
    GLOBAL VARIABLES
*/

// Security Manager's OSAL task ID
extern uint8 smTaskID;

extern smPairingParams_t* pPairingParams[];

extern smResponderCBs_t* pfnResponderCBs;

/*********************************************************************
    FUNCTIONS - API
*/

/*********************************************************************
    Application Level Functions
*/

/*
    smLinkCheck - link database callback function.
*/
extern void smLinkCheck( uint16 connectionHandle, uint8 changeType );

/*
    smProcessRandComplete - Process the HCI Random Complete Event.
*/
extern uint8 smProcessRandComplete( uint8 status, uint8* rand );

/*
    smTimedOut - Process the SM timeout.
*/
extern void smTimedOut( uint16 connectionHandle );

/*
    smStartRspTimer - Start the SM Response Timer.
*/
extern void smStartRspTimer( uint16 connectionHandle );

/*
    smStopRspTimer - Stop the SM Response Timer.
*/
extern void smStopRspTimer( uint16 connectionHandle );

/*
    smProcessDataMsg - Process incoming L2CAP messages.
*/
extern void smProcessDataMsg( l2capDataEvent_t* pMsg );

/*
    smProcessEncryptChange - Process the HCI BLE Encrypt Change Event.
*/
extern uint8 smProcessEncryptChange( uint16 connectionHandle, uint8 reason );

/*
    smInProcess - Is SM already processing something?
*/
extern uint8 smInProcess( void );

/*
    sm_d1 - SM diversifying function d1
*/
extern bStatus_t sm_d1( uint8* pK, uint16 d, uint8* pD1 );

/*
    sm_ah - Random address hash function
*/
extern bStatus_t sm_ah( uint8* pK, uint8* pR, uint8* pAh );

/*
    sm_dm - SM DIV Maxk generation function dm
*/
extern bStatus_t sm_dm( uint8* pK, uint8* pR, uint16* pDm );

/*
    sm_c1 - SM Confirm value generation function c1
*/
extern bStatus_t sm_c1( uint16 connectionHandle,uint8* pK, uint8* pR, uint8* pC1 );

/*
    sm_c1new - SM Confirm value generation function c1
*/
extern bStatus_t sm_c1new( uint8* pK, uint8* pR, uint8* pRes, uint8* pReq,
                           uint8 iat, uint8* pIA, uint8 rat, uint8* pRA, uint8* pC1 );
/*
    sm_s1 - SM key generation function s1
*/
extern bStatus_t sm_s1( uint8* pK, uint8* pR1, uint8* pR2, uint8* pS1 );

/*
    smGenerateRandBuf - generate a buffer of random numbers
*/
extern void smGenerateRandBuf( uint8* pRandNum, uint8 len );

/*
    smEncLTK - start LTK Encryption
*/
extern void smEncLTK( uint16 connectionHandle );

/*
    smNextPairingState - trigger next state machine
*/
extern void smNextPairingState( uint16 connectionHandle );

/*
    smAuthReqToUint8 - conversion function
*/
extern uint8 smAuthReqToUint8( authReq_t* pAuthReq );

/*
    smUint8ToAuthReq - conversion function
*/
extern void smUint8ToAuthReq( authReq_t* pAuthReq, uint8 authReqUint8 );

/*
    smpResponderProcessPairingReq - Process an incoming Pairing Request message
*/
extern uint8 smpResponderProcessPairingReq( uint16 connectionHandle,smpPairingReq_t* pParsedMsg );

/*
    smSendFailAndEnd - Send the pairing failed message and end existing pairing
*/
extern bStatus_t smSendFailAndEnd( uint16 connHandle, smpPairingFailed_t* pFailedMsg );

/*
    generateRandMsg - Generate a Pairing Random
*/
extern bStatus_t smGenerateRandMsg( uint16 connectionHandle);

/*
    smSavePairInfo - Save the Pairing Req or Rsp information
*/
extern bStatus_t smSavePairInfo( uint16 connectionHandle,smpPairingReq_t* pPair );

/*
    generateConfirm - Generate a Pairing Confirm
*/
extern bStatus_t smGenerateConfirm( uint16 connectionHandle );

/*
    smEndPairing - Pairing mode has ended.  Yeah. Notify the GAP and free
                  up the memory used.
*/
extern void smEndPairing( uint16 connectionHandle,uint8 status );

/*
    determineKeySize - Determine the maximum encryption key size
*/
extern uint8 smDetermineKeySize( uint16 connectionHandle );

/*
    smGeneratePairingReqRsp - Generate a pairing req or response
*/
extern bStatus_t smGeneratePairingReqRsp( uint16 connectionHandle );

/*
    smPairingSendEncInfo - Send SM Encryption Information message
*/
extern void smPairingSendEncInfo( uint16 connHandle, uint8* pLTK );

/*
    smPairingSendMasterID - Send SM Master Identification message
*/
extern void smPairingSendMasterID( uint16 connHandle, uint16 ediv, uint8* pRand );

/*
    smPairingSendIdentityInfo - Send SM Identity Information message
*/
extern void smPairingSendIdentityInfo( uint16 connHandle, uint8* pIRK );

/*
    smPairingSendIdentityAddrInfo - Send SM Identity Addr Information message
*/
extern void smPairingSendIdentityAddrInfo( uint16 connHandle, uint8 addrType, uint8* pMACAddr );

/*
    smPairingSendSingingInfo - Send SM Signing Information message
*/
extern void smPairingSendSingingInfo( uint16 connHandle, uint8* pSRK );

/*
    smPairingSendEncInfo - Send SM Encryption Information message
*/
extern void smPairingSendEncInfo( uint16 connHandle, uint8* pLTK );

/*
    smProcessPairingReq - Process Pairing Request
*/
extern void smProcessPairingReq( linkDBItem_t* pLinkItem, gapPairingReq_t* pPairReq );

/*
    smStartEncryption - Perform Encrypt through HCI
*/
extern bStatus_t smStartEncryption( uint16 connHandle, uint8* pLTK, uint16 div,
                                    uint8* pRandNum, uint8 keyLen );

/*
    smRegisterInitiator - egister Initiator's processing function with SM task
*/
extern void smRegisterInitiator( smInitiatorCBs_t* pfnCBs );

/*
    smRegisterResponder - Register Responder's processing function with SM task
*/
extern void smRegisterResponder( smResponderCBs_t* pfnCBs );

/*
    smp timerout callback for SMP Timeout and pairing state
*/
extern void smTo_timerCB( uint8* pData );
extern void smState_timerCB( uint8* pData );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SM_INTERNAL_H */
