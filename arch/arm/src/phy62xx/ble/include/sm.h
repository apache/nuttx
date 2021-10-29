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
    @headerfile:    sm.h
    $Date:
    $Revision:

    @mainpage BLE SM API

    This file contains the interface to the SM.



*/

#ifndef SM_H
#define SM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*  -------------------------------------------------------------------
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"

/*  -------------------------------------------------------------------
    MACROS
*/

/*  -------------------------------------------------------------------
    CONSTANTS
*/
/** @defgroup SM_IO_CAP_DEFINES SM I/O Capabilities
    @{
*/
#define DISPLAY_ONLY              0x00  //!< Display Only Device
#define DISPLAY_YES_NO            0x01  //!< Display and Yes and No Capable
#define KEYBOARD_ONLY             0x02  //!< Keyboard Only
#define NO_INPUT_NO_OUTPUT        0x03  //!< No Display or Input Device
#define KEYBOARD_DISPLAY          0x04  //!< Both Keyboard and Display Capable
/** @} End SM_IO_CAP_DEFINES */

#define SM_AUTH_MITM_MASK(a)    (((a) & 0x04) >> 2)

/** @defgroup SM_PASSKEY_TYPE_DEFINES SM Passkey Types (Bit Masks)
    @{
*/
#define SM_PASSKEY_TYPE_INPUT   0x01    //!< Input the passkey
#define SM_PASSKEY_TYPE_DISPLAY 0x02    //!< Display the passkey
/** @} End SM_PASSKEY_TYPE_DEFINES */

/** @defgroup SM_BONDING_FLAGS_DEFINES SM AuthReq Bonding Flags
    Bonding flags 0x02 and 0x03 are reserved.
    @{
*/
#define SM_AUTH_REQ_NO_BONDING    0x00  //!< No bonding
#define SM_AUTH_REQ_BONDING       0x01  //!< Bonding
/** @} End SM_BONDING_FLAGS_DEFINES */

#define PASSKEY_LEN     6   //! Passkey Character Length (ASCII Characters)

#define SM_AUTH_STATE_CT2                 0x20
#define SM_AUTH_STATE_KEYPRESS            0x10
#define SM_AUTH_STATE_SC                  0x08
#define SM_AUTH_STATE_AUTHENTICATED       0x04  //! Authenticate requested
#define SM_AUTH_STATE_BONDING             0x01  //! Bonding requested

/*  -------------------------------------------------------------------
    General TYPEDEFS
*/

/**
    SM_NEW_RAND_KEY_EVENT message format.  This message is sent to the
    requesting task.
*/
typedef struct
{
    osal_event_hdr_t  hdr;      //!< SM_NEW_RAND_KEY_EVENT and status
    uint8 newKey[KEYLEN];       //!< New key value - if status is SUCCESS
} smNewRandKeyEvent_t;

/**
    Key Distribution field  - True or False fields
*/
typedef struct
{
    unsigned int sEncKey:1;    //!< Set to distribute slave encryption key
    unsigned int sIdKey:1;     //!< Set to distribute slave identity key
    unsigned int sSign:1;      //!< Set to distribute slave signing key
    unsigned int sReserved:5;  // bug fixed 2018-11-26, reserved bits should be saved for upward compatibility

    unsigned int mEncKey:1;    //!< Set to distribute master encryption key
    unsigned int mIdKey:1;     //!< Set to distribute master identity key
    unsigned int mSign:1;      //!< Set to distribute master signing key
    unsigned int mReserved:5;  // bug fixed 2018-11-26, reserved bits should be saved for upward compatibility
} keyDist_t;

/**
    Link Security Requirements
*/
typedef struct
{
    uint8 ioCaps;               //!< I/O Capabilities (ie.
    uint8 oobAvailable;         //!< True if Out-of-band key available
    uint8 oob[KEYLEN];          //!< Out-Of-Bounds key
    uint8 authReq;              //!< Authentication Requirements
    keyDist_t keyDist;          //!< Key Distribution mask
    uint8 maxEncKeySize;        //!< Maximum Encryption Key size (7-16 bytes)
} smLinkSecurityReq_t;

/**
    Link Security Information
*/
typedef struct
{
    uint8 ltk[KEYLEN];              //!< Long Term Key (LTK)
    uint16 div;                     //!< LTK Diversifier
    uint8 rand[B_RANDOM_NUM_SIZE];  //!< LTK random number
    uint8 keySize;                  //!< LTK Key Size (7-16 bytes)
} smSecurityInfo_t;

/**
    Link Identity Information
*/
typedef struct
{
    uint8 irk[KEYLEN];          //!< Identity Resolving Key (IRK)
    uint8 bd_addr[B_ADDR_LEN];  //!< The advertiser may set this to zeroes to not disclose its BD_ADDR (public address).
} smIdentityInfo_t;

/**
    Signing Information
*/
typedef struct
{
    uint8  srk[KEYLEN]; //!< Signature Resolving Key (CSRK)
    uint32 signCounter; //!< Sign Counter
} smSigningInfo_t;

/**
    Pairing Request & Response - authReq field
*/
typedef struct
{
    unsigned int bonding:2;    //!< Bonding flags
    unsigned int mitm:1;       //!< Man-In-The-Middle (MITM)
    unsigned int reserved:5;   //!< Reserved - don't use
} authReq_t;

/*  -------------------------------------------------------------------
    GLOBAL VARIABLES
*/

/**
    @defgroup SM_API Security Manager API Functions

    @{
*/

/*  -------------------------------------------------------------------
    FUNCTIONS - MASTER API - Only use these in a master device
*/

/**
    @brief       Initialize SM Initiator on a master device.

    @return      SUCCESS
*/
extern bStatus_t SM_InitiatorInit( void );

/**
    @brief       Start the pairing process.  This function is also
                called if the device is already bound.

    NOTE:        Only one pairing process at a time per device.

    @param       initiator - TRUE to start pairing as Initiator.
    @param       taskID - task ID to send results.
    @param       connectionHandle - Link's connection handle
    @param       pSecReqs - Security parameters for pairing

    @return      SUCCESS,<BR>
                INVALIDPARAMETER,<BR>
                bleAlreadyInRequestedMode
*/
extern bStatus_t SM_StartPairing(  uint8 initiator,
                                   uint8 taskID,
                                   uint16 connectionHandle,
                                   smLinkSecurityReq_t* pSecReqs );

/**
    @brief       Send Start Encrypt through HCI

    @param       connHandle - Connection Handle
    @param       pLTK - pointer to 16 byte lkt
    @param       div - div or ediv
    @param       pRandNum - pointer to 8 byte random number
    @param       keyLen - length of LTK (bytes)

    @return      SUCCESS,<BR>
                INVALIDPARAMETER,<BR>
                other from HCI/LL
*/
extern bStatus_t SM_StartEncryption( uint16 connHandle, uint8* pLTK,
                                     uint16 div, uint8* pRandNum, uint8 keyLen );


/*  -------------------------------------------------------------------
    FUNCTIONS - SLAVE API - Only use these in a slave device
*/

/**
    @brief       Initialize SM Responder on a slave device.

    @return      SUCCESS
*/
extern bStatus_t SM_ResponderInit( void );

/*  -------------------------------------------------------------------
    FUNCTIONS - GENERAL API - both master and slave
*/

/**
    @brief       Generate a key with a random value.

    @param       taskID - task ID to send results.

    @return      SUCCESS,<BR>
                bleNotReady,<BR>
                bleMemAllocError,<BR>
                FAILURE
*/
extern bStatus_t SM_NewRandKey( uint8 taskID );

/**
    @brief       Calculate a new Private Resolvable address.

    @param       pIRK - Identity Root Key.
    @param       pNewAddr - pointer to place to put new calc'd address

    @return      SUCCESS - if started,<BR>
                INVALIDPARAMETER
*/
extern bStatus_t SM_CalcRandomAddr( uint8* pIRK, uint8* pNewAddr );

/**
    @brief       Resolve a Private Resolveable Address.

    @param       pIRK - pointer to the IRK
    @param       pAddr - pointer to the random address

    @return      SUCCESS - match,<BR>
                FAILURE - don't match,<BR>
                INVALIDPARAMETER - parameters invalid
*/
extern bStatus_t SM_ResolveRandomAddrs( uint8* pIRK, uint8* pAddr );

/**
    @brief       Encrypt the plain text data with the key..

    @param       pKey - key data
    @param       pPlainText - Plain text data
    @param       pResult - place to put the encrypted result

    @return      SUCCESS - if started,<BR>
                INVALIDPARAMETER - one of the parameters are NULL,<BR>
                bleAlreadyInRequestedMode,<BR>
                bleMemAllocError
*/
extern bStatus_t SM_Encrypt( uint8* pKey, uint8* pPlainText, uint8* pResult );

/**
    @brief       Generate an outgoing Authentication Signature.

    @param       pData - message data
    @param       len - length of pData
    @param       pAuthenSig - place to put new signature

    @return      SUCCESS - signature authentication generated,<BR>
                INVALIDPARAMETER - pData or pAuthenSig is NULL,<BR>
                bleMemAllocError
*/
extern bStatus_t SM_GenerateAuthenSig( uint8* pData, uint8 len, uint8* pAuthenSig );

/**
    @brief       Verify an Authentication Signature.

    @param       connHandle - connection to verify against.
    @param       authentication - TRUE if requires an authenticated CSRK, FALSE if not
    @param       pData - message data
    @param       len - length of pData
    @param       pAuthenSig - message signature to verify

    @return      SUCCESS - signature authentication verified,<BR>
                FAILURE - if not verified,<BR>
                bleNotConnected - Connection not found,<BR>
                INVALIDPARAMETER - pData or pAuthenSig is NULL, or signCounter is invalid,<BR>
                bleMemAllocError
*/
extern bStatus_t SM_VerifyAuthenSig( uint16 connHandle,
                                     uint8 authentication,
                                     uint8* pData,
                                     uint16 len,
                                     uint8* pAuthenSig );

/**
    @brief       Update the passkey for the pairing process.

    @param       pPasskey - pointer to the 6 digit passkey
    @param       connectionHandle - connection handle to link.

    @return      SUCCESS,<BR>
                bleIncorrectMode - Not pairing,<BR>
                INVALIDPARAMETER - link is incorrect
*/
extern bStatus_t SM_PasskeyUpdate( uint8* pPasskey, uint16 connectionHandle );

/**
    @} End SM_API
*/

/*  -------------------------------------------------------------------
    TASK API - These functions must only be called OSAL.
*/

/**
    @internal

    @brief       SM Task Initialization Function.

    @param       taskID - SM task ID.

    @return      void
*/
extern void SM_Init( uint8 task_id );

/**
    @internal

    @brief       SM Task event processing function.

    @param       taskID - SM task ID
    @param       events - SM events.

    @return      events not processed
*/
extern uint16 SM_ProcessEvent( uint8 task_id, uint16 events );

/*  -------------------------------------------------------------------
    -------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* SM_H */
