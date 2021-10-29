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

#ifndef L2CAP_INTERNAL_H
#define L2CAP_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "hci.h"
#include "l2cap.h"

/*********************************************************************
    MACROS
*/

// Macro to see if a given channel is a fixed channel
#define FIX_CHANNEL( CID )        ( (CID) == L2CAP_CID_GENERIC ||\
                                    (CID) == L2CAP_CID_SIG ||\
                                    (CID) == L2CAP_CID_ATT ||\
                                    (CID) == L2CAP_CID_SMP )

// Marco to convert a channel ID to an index into L2CAP Channel table
#define CID_TO_INDEX( CID )       ( (CID) - BASE_DYNAMIC_CID )

// Marco to convert a fixed channel ID to an index into L2CAP Fixed Channel table
#define FCID_TO_INDEX( CID )      ( (CID) - L2CAP_CID_ATT )

// Macro to return the record maintained a given fix channel
#define FIX_CHANNEL_REC( CID )    ( l2capFixedChannels[FCID_TO_INDEX( CID )] )

/*********************************************************************
    CONSTANTS
*/
// Signaling command header: Code (1 byte) + Identifier (1 byte) + Length (2 bytes)
#define SIGNAL_HDR_SIZE                            4

// Maximum size of data field of Signaling commands
#define SIGNAL_DATA_SIZE                           ( L2CAP_SIG_MTU_SIZE - SIGNAL_HDR_SIZE )

/*********************************************************************
    L2CAP Channel States: states used for l2capChannel 'state' field
*/
// Closed - no channel associated with this CID
#define L2CAP_CLOSED                               0x00

// Waiting for Echo Response
#define L2CAP_W4_ECHO_RSP                          0x01

// Waiting for Info Response
#define L2CAP_W4_INFO_RSP                          0x02

// Waiting for Connection Parameter Update Response
#define L2CAP_W4_PARAM_UPDATE_RSP                  0x03

/*********************************************************************
    TYPEDEFS
*/

// L2CAP Channel structure. Allocated one per application connection
// between two devices. CID assignment is relative to a particular device
// and a device can assign CIDs independently from other devices (except
// for the reserved CIDs). The CIDs are dynamically allocated in the range
// from 0x0040 to 0xFFFF.
typedef struct
{
    // Channel info
    uint8  state; // Channel connection state
    uint16 CID;   // Local channel id
    uint8  id;    // Local identifier - matches responses with requests

    // Link info
    uint16 connHandle; // link connection handle

    // Application info
    uint8 taskId; // task that channel belongs to

    // Timer id
    uint8 timerId;
} l2capChannel_t;

// L2CAP Fixed Channel structure. Allocated one for each fixed channel.
typedef struct
{
    uint16 CID;   // channel id
    uint8 taskId; // task registered with channel
} l2capFixedChannel_t;

// Signaling packet header format
typedef struct
{
    uint8 opcode; // type of command
    uint8 id;     // identifier - matches responses with requests
    uint16 len;   // length of data field (doesn't cover Code, Identifier and Length fields)
} l2capSignalHdr_t;

/**
    @brief   Callback function prototype for building a Signaling command.

    @param   pBuf - pointer to buffer to hold command data
    @param   pData - pointer to command data

    @return  length of the command data
*/
typedef uint16 (*pfnL2CAPBuildCmd_t)( uint8* pBuf, uint8* pData );

/*********************************************************************
    GLOBAL VARIABLES
*/
extern uint8 l2capTaskID;
extern l2capChannel_t l2capChannels[];
extern l2capFixedChannel_t l2capFixedChannels[];

/*********************************************************************
    FUNCTIONS - API
*/

/*
    Send L2CAP Command.
*/
extern bStatus_t l2capSendCmd( uint16 connHandle, uint8 opcode, uint8 id,
                               uint8* pCmd, pfnL2CAPBuildCmd_t pfnBuildCmd );
/*
    Send L2CAP Request.
*/
extern bStatus_t l2capSendReq( uint16 connHandle, uint8 opcode, uint8* pReq,
                               pfnL2CAPBuildCmd_t pfnBuildCmd, uint8 state, uint8 taskId );
/*
    Build Echo Request.
*/
extern uint16 l2capBuildEchoReq( uint8* pBuf, uint8* pCmd );

/*
    Build Info Request.
*/
extern uint16 l2capBuildInfoReq( uint8* pBuf, uint8* pCmd );

/*
    Build Parameter Update Request.
*/
extern uint16 l2capBuildParamUpdateReq( uint8* pBuf, uint8* pData );

/*
    Encapsulate and send L2CAP packet.
*/
extern bStatus_t l2capEncapSendData( uint16 connHandle, l2capPacket_t* pPkt );

/*
    Parse L2CAP packet.
*/
extern uint8 l2capParsePacket( l2capPacket_t* pPkt, hciDataEvent_t* pHciMsg );

/*
    Parse L2CAP Signaling header.
*/
extern void l2capParseSignalHdr( l2capSignalHdr_t* pHdr, uint8* pData );

/*
    Build Echo Response.
*/
extern uint16 l2capBuildEchoRsp( uint8* pBuf, uint8* pCmd );

/*
    Parse Command Reject.
*/
extern bStatus_t l2capParseCmdReject( l2capSignalCmd_t* pCmd, uint8* pData, uint16 len );

/*
    Parse Echo Response.
*/
extern bStatus_t l2capParseEchoRsp( l2capSignalCmd_t* pCmd, uint8* pData, uint16 len );

/*
    Parse Information Response.
*/
extern bStatus_t l2capParseInfoRsp( l2capSignalCmd_t* pCmd, uint8* pData, uint16 len );

/*
    Parse Connection Parameter Update Response.
*/
extern bStatus_t l2capParseParamUpdateRsp( l2capSignalCmd_t* pCmd, uint8* pData, uint16 len );

/*
    Find a channel using the local identifier.
*/
extern l2capChannel_t* l2capFindLocalId( uint8 id );

/*
    Free a channel.
*/
extern void l2capFreeChannel( l2capChannel_t* pChannel );

/*
    Stop an active timer for a given channel.
*/
extern void l2capStopTimer( l2capChannel_t* pChannel );

/*
    Handle an incoming packet error.
*/
extern void l2capHandleRxError( uint16 connHandle );

/*
    Forward a data message to upper layer application.
*/
extern bStatus_t l2capNotifyData( uint8 taskId, uint16 connHandle, l2capPacket_t* pPkt );

/*
    Send a Signaling command to upper layer application.
*/
extern void l2capNotifySignal( uint8 taskId, uint16 connHandle, uint8 status,
                               uint8 opcode, uint8 id, l2capSignalCmd_t* pCmd );

extern void* L2CAP_Fragment_bm_alloc( uint16 size );

extern uint8 L2CAP_Fragment_SendDataPkt( uint16 connHandle, uint8 fragFlg,uint16 pktLen, uint8* pBuf );


/*********************************************************************
    @fn      l2capInfoRsp

    @brief   Send Info Response.

            Use like: l2capInfoRsp( uint16 connHandle, uint8 id, l2capInfoRsp_t *pInfoRsp );

    @param   connHandle - connection to use
    @param   id - identifier received in request
    @param   pInfoRsp - pointer to Info Response to be sent

    @return  SUCCESS: Request was sent successfully.
            INVALIDPARAMETER: Data can not fit into one packet.
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
            bleNotConnected: Connection is down.
            bleMemAllocError: Memory allocation error occurred.
*/
#define l2capInfoRsp( connHandle, id, pInfoRsp )  l2capSendCmd( (connHandle), L2CAP_INFO_RSP, (id),\
                                                                (uint8 *)(pInfoRsp), L2CAP_BuildInfoRsp )

/*********************************************************************
    @fn      l2capEchoRsp

    @brief   Send Ehco Response.

            Use like: l2capEchoRsp( uint16 connHandle, uint8 id, l2capEchoRsp_t *pEchoRsp );

    @param   connHandle - connection to use
    @param   id - identifier received in request
    @param   pEchoRsp - pinter to Echo Response to be sent

    @return  SUCCESS: Request was sent successfully.
            INVALIDPARAMETER: Data can not fit into one packet.
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
            bleNotConnected: Connection is down.
            bleMemAllocError: Memory allocation error occurred.
*/
#define l2capEchoRsp( connHandle, id, pEchoRsp )  l2capSendCmd( (connHandle), L2CAP_ECHO_RSP, (id),\
                                                                (uint8 *)(pEchoRsp), l2capBuildEchoRsp )


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* L2CAP_INTERNAL_H */
