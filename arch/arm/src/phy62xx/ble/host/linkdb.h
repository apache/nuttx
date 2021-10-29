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
    Filename:       linkdb.h
    Revised:
    Revision:

    Description:    This file contains the linkDB interface.


**************************************************************************************************/

#ifndef LINKDB_H
#define LINKDB_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

// Special case connection handles
#define INVALID_CONNHANDLE              0xFFFF  // Invalid connection handle, used for no connection handle
#define LOOPBACK_CONNHANDLE             0xFFFE  // Loopback connection handle, used to loopback a message

// Link state flags
#define LINK_NOT_CONNECTED              0x00    // Link isn't connected
#define LINK_CONNECTED                  0x01    // Link is connected
#define LINK_AUTHENTICATED              0x02    // Link is authenticated
#define LINK_BOUND                      0x04    // Link is bonded
#define LINK_ENCRYPTED                  0x10    // Link is encrypted

// Link Database Status callback changeTypes
#define LINKDB_STATUS_UPDATE_NEW        0       // New connection created
#define LINKDB_STATUS_UPDATE_REMOVED    1       // Connection was removed
#define LINKDB_STATUS_UPDATE_STATEFLAGS 2       // Connection state flag changed

// Link Authentication Errors
#define LINKDB_ERR_INSUFFICIENT_AUTHEN      0x05  // Link isn't even encrypted
#define LINBDB_ERR_INSUFFICIENT_KEYSIZE     0x0c  // Link is encrypted but the key size is too small
#define LINKDB_ERR_INSUFFICIENT_ENCRYPTION  0x0f  // Link is encrypted but it's not authenticated

/*********************************************************************
    TYPEDEFS
*/

typedef struct
{
    uint8 srk[KEYLEN];  // Signature Resolving Key
    uint32 signCounter; // Sign Counter
} linkSec_t;

typedef struct
{
    uint8 ltk[KEYLEN];             // Long Term Key
    uint16 div;                    // Diversifier
    uint8 rand[B_RANDOM_NUM_SIZE]; // random number
    uint8 keySize;                 // LTK Key Size
} encParams_t;

typedef struct
{
    uint8 taskID;            // Application that controls the link
    uint16 connectionHandle; // Controller connection handle
    uint8 stateFlags;        // LINK_CONNECTED, LINK_AUTHENTICATED...
    uint8 role;               // 2020-04-22 add (case for multi-role SMP )
    uint8 addrType;          // Address type of connected device
    uint8 addr[B_ADDR_LEN];  // Other Device's address
    uint16 connInterval;     // The connection's interval (n * 1.23 ms)
    linkSec_t sec;           // Connection Security related items
    encParams_t* pEncParams; // pointer to LTK, ediv, rand. if needed.
} linkDBItem_t;

// function pointer used to register for a status callback
typedef void (*pfnLinkDBCB_t)( uint16 connectionHandle, uint8 changeType );

// function pointer used to perform specialized link database searches
typedef void (*pfnPerformFuncCB_t)( linkDBItem_t* pLinkItem );

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    PUBLIC FUNCTIONS
*/
/*
    linkDB_Init - Initialize the Link Database.
*/
extern void linkDB_Init( void );

/*
    linkDB_Register - Register with this function to receive a callback when
                status changes on a connection.
*/
extern uint8 linkDB_Register( pfnLinkDBCB_t pFunc );

/*
    linkDB_Add - Adds a record to the link database.
*/
extern uint8 linkDB_Add( uint8 taskID, uint16 connectionHandle, uint8  stateFlags, uint8 role,
                         uint8 addrType, uint8* pAddr, uint16 connInterval );

/*
    linkDB_Remove - Removes a record from the link database.
*/
extern uint8 linkDB_Remove( uint16 connectionHandle );

/*
    linkDB_Update - This function is used to update the stateFlags of
                a link record.
*/
extern uint8 linkDB_Update( uint16 connectionHandle, uint8 newState );

/*
    linkDB_NumActive - returns the number of active connections.
*/
extern uint8 linkDB_NumActive( void );

/*
    linkDB_Find - Find link database item (link information)

      returns a pointer to the link item, NULL if not found
*/
extern linkDBItem_t* linkDB_Find( uint16 connectionHandle );

/*
    linkDB_FindFirst - Find the first link that matches the taskID.

      returns a pointer to the link item, NULL if not found
*/
extern linkDBItem_t* linkDB_FindFirst( uint8 taskID );

/*
    linkDB_State - Check to see if a physical link is in a specific state.

      returns TRUE is the link is in state. FALSE, otherwise.
*/
extern uint8 linkDB_State( uint16 connectionHandle, uint8 state );

/*
    linkDB_Authen - Check to see if the physical link is encrypted and authenticated.
      returns SUCCESS if the link is authenticated or
              bleNotConnected - connection handle is invalid,
              LINKDB_ERR_INSUFFICIENT_AUTHEN - link is not encrypted,
              LINBDB_ERR_INSUFFICIENT_KEYSIZE - key size encrypted is not large enough,
              LINKDB_ERR_INSUFFICIENT_ENCRYPTION - link is encrypted, but not authenticated
*/
extern uint8 linkDB_Authen( uint16 connectionHandle, uint8 keySize, uint8 mitmRequired );

/*
    linkDB_PerformFunc - Perform a function of each connection in the link database.
*/
extern void linkDB_PerformFunc( pfnPerformFuncCB_t cb );

/*
    linkDB_Up - Check to see if a physical link is up (connected).
      Use like:  uint8 linkDB_Up( uint16 connectionHandle );
              connectionHandle - controller link connection handle.
              returns TRUE if the link is up. FALSE, otherwise.
*/
#define linkDB_Up( connectionHandle )  linkDB_State( (connectionHandle), LINK_CONNECTED )

/*
    linkDB_Encrypted - Check to see if the physical link is encrypted.
      Use like:  linkDB_Encrypted( uint16 connectionHandle );
              connectionHandle - controller link connection handle.
              returns TRUE if the link is encrypted. FALSE, otherwise.
*/
#define linkDB_Encrypted( connectionHandle )  linkDB_State( (connectionHandle), LINK_ENCRYPTED )

/*
    linkDB_Authenticated - Check to see if the physical link is authenticated.
      Use like:  linkDB_Authenticated( uint16 connectionHandle );
              connectionHandle - controller link connection handle.
              returns TRUE if the link is authenticated. FALSE, otherwise.
*/
#define linkDB_Authenticated( connectionHandle )  linkDB_State( (connectionHandle), LINK_AUTHENTICATED )

/*
    linkDB_Bonded - Check to see if the physical link is bonded.
      Use like:  linkDB_Bonded( uint16 connectionHandle );
              connectionHandle - controller link connection handle.
              returns TRUE if the link is bonded. FALSE, otherwise.
*/
#define linkDB_Bonded( connectionHandle )  linkDB_State( (connectionHandle), LINK_BOUND )

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* LINKDB_H */
