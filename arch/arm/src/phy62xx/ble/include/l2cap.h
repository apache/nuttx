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
    Filename:       l2cap.h
    Revised:
    Revision:

    Description:    This file contains the L2CAP definitions.


 **************************************************************************************************/

#ifndef L2CAP_H
#define L2CAP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"

#include "log.h"
/*********************************************************************
    CONSTANTS
*/
#ifndef MTU_SIZE
#define MTU_SIZE 23
#endif
#if( MTU_SIZE < 23 )
#error "MTU_SIZE define error"
#endif

// Minimum supported information payload for the Basic information frame (B-frame)
#define L2CAP_MTU_SIZE                   MTU_SIZE

// Minimum supported information payload for the Control frame (C-frame)
#define L2CAP_SIG_MTU_SIZE               23

// Basic L2CAP header: Length (2 bytes) + Channel ID (2 bytes)
#define L2CAP_HDR_SIZE                   4

// Minimum size of PDU received from lower layer protocol (incoming
// packet), or delivered to lower layer protocol (outgoing packet).
#define L2CAP_PDU_SIZE                   ( L2CAP_HDR_SIZE + L2CAP_MTU_SIZE )

// L2CAP Channel Identifiers. Identifiers from 0x0001 to 0x003F are
// reserved for specific L2CAP functions. Identifiers 0x0001-0x0003
// are reserved by BR/EDR.
#define L2CAP_CID_NULL                   0x0000 // Illegal Identifier
#define L2CAP_CID_ATT                    0x0004 // Attribute Protocol
#define L2CAP_CID_SIG                    0x0005 // L2CAP Signaling
#define L2CAP_CID_SMP                    0x0006 // Security Management Protocol
#define L2CAP_CID_GENERIC                0x0007 // Generic Fixed Channel

// L2CAP Dynamic Channel Identifiers
#define L2CAP_BASE_DYNAMIC_CID           0x0040
#define L2CAP_LAST_DYNAMIC_CID           ( BASE_DYNAMIC_CID + L2CAP_NUM_CHANNELS - 1 )

// Number of Fixed channels: one for each of ATT, Signaling, SMP channels and one Generic Channel
#define L2CAP_NUM_FIXED_CHANNELS         4

// Number of Protocols supported -- for future use
#define L2CAP_NUM_PROTOCOLS              0

// Number of Auxiliary channels: one for each of Echo Request, Information
// Request and Connection Parameter Update Request
#define L2CAP_NUM_AUX_CHANNELS           3

// Number of Dynamic channels: one per each protocol supported on each physical connection
#define L2CAP_NUM_DYNAMIC_CHANNELS       ( L2CAP_NUM_PROTOCOLS * MAX_NUM_LL_CONN )

// Total number of L2CAP channels: Dynamic channels plus Auxiliary channels
#define L2CAP_NUM_CHANNELS               ( L2CAP_NUM_DYNAMIC_CHANNELS + L2CAP_NUM_AUX_CHANNELS )

// L2CAP Response Timeout expired (RTX) value for Signaling commands (in seconds).
// The RTX timer is used for response timeout or to terminate a dynamic channel
// when the remote device is unresponsive to signaling requests. Its value may
// range from 1 to 60 seconds.
#define L2CAP_RTX_TIMEOUT                30

// L2CAP Signaling Codes (type of commands)
#define L2CAP_CMD_REJECT                 0x01
#define L2CAP_ECHO_REQ                   0x08 // No longer supported
#define L2CAP_ECHO_RSP                   0x09 // No longer supported
#define L2CAP_INFO_REQ                   0x0a // No longer supported
#define L2CAP_INFO_RSP                   0x0b // No longer supported
#define L2CAP_PARAM_UPDATE_REQ           0x12
#define L2CAP_PARAM_UPDATE_RSP           0x13

/*********************************************************************
    Command Reject: Reason Codes
*/
// Command not understood
#define L2CAP_REJECT_CMD_NOT_UNDERSTOOD  0x0000

// Signaling MTU exceeded
#define L2CAP_REJECT_SIGNAL_MTU_EXCEED   0x0001

// Invalid CID in request
#define L2CAP_REJECT_INVALID_CID         0x0002

/*********************************************************************
    Information Request/Response: Info Type
*/
// Connectionless MTU
#define L2CAP_INFO_CONNLESS_MTU          0x0001

// Extended features supported
#define L2CAP_INFO_EXTENDED_FEATURES     0x0002

// Fixed channels supported
#define L2CAP_INFO_FIXED_CHANNELS        0x0003

/*********************************************************************
    Information Response: Extended Features Mask Values
*/
// Fixed channels are supported
#define L2CAP_FIXED_CHANNELS             0x00000080

// Length of Extended Features bit mask
#define L2CAP_EXTENDED_FEATURES_SIZE     4

/*********************************************************************
    Information Response: Fixed Channels Mask Values
*/
// Fixed Channel ATT is supported
#define L2CAP_FIXED_CHANNELS_ATT         0x10

// Fixed Channel L2CAP Signaling is supported
#define L2CAP_FIXED_CHANNELS_SIG         0x20

// Fixed Channel SMP is supported
#define L2CAP_FIXED_CHANNELS_SMP         0x40

// Length of Fixed Channels bit mask
#define L2CAP_FIXED_CHANNELS_SIZE        8

/*********************************************************************
    Information Response: Result Values
*/
// Success
#define L2CAP_INFO_SUCCESS               0x0000

// Not supported
#define L2CAP_INFO_NOT_SUPPORTED         0x0001

/*********************************************************************
    Connection Parameter Update Response: Result values
*/
// Connection Parameters accepted
#define L2CAP_CONN_PARAMS_ACCEPTED       0x0000

// Connection Parameters rejected
#define L2CAP_CONN_PARAMS_REJECTED       0x0001


/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    MACROS
*/

/*********************************************************************
    TYPEDEFS
*/

// Invalid CID in Request format
typedef struct
{
    uint16 localCID;  // Destination CID from the rejected command
    uint16 remoteCID; // Source CID from the rejected command
} l2capInvalidCID_t;

// Command Reject Reason Data format
typedef union
{
    uint16 signalMTU;             // Maximum Signaling MTU
    l2capInvalidCID_t invalidCID; // Invalid CID in Request
} l2capReasonData_t;

// Command Reject format
typedef struct
{
    uint16 reason;                // Reason
    l2capReasonData_t reasonData; // Reason Data

    // Shorthand access for union members
#define maxSignalMTU     reasonData.signalMTU
#define invalidLocalCID  reasonData.invalidCID.localCID
#define invalidRemoteCID reasonData.invalidCID.remoteCID
} l2capCmdReject_t;

// Echo Request format
typedef struct
{
    uint8* pData; // Optional data field
    uint16 len;   // Length of data
} l2capEchoReq_t;

// Echo Response format
typedef struct
{
    uint8* pData; // Optional data field -- must be freed by the application
    uint16 len;   // Length of data
} l2capEchoRsp_t;

// Information Request format
typedef struct
{
    uint16 infoType; // Information type
} l2capInfoReq_t;

// Information Response Data field
typedef union
{
    uint16 connectionlessMTU;                       // Connectionless MTU
    uint32 extendedFeatures;                        // Extended features supported
    uint8 fixedChannels[L2CAP_FIXED_CHANNELS_SIZE]; // Fixed channels supported
} l2capInfo_t;

// Information Response format
typedef struct
{
    uint16 result;    // Result
    uint16 infoType;  // Information type
    l2capInfo_t info; // Content of Info field depends on infoType
} l2capInfoRsp_t;

// Connection Parameter Update Request format
typedef struct
{
    uint16 intervalMin;       // Minimum Interval
    uint16 intervalMax;       // Maximum Interval
    uint16 slaveLatency;      // Slave Latency
    uint16 timeoutMultiplier; // Timeout Multiplier
} l2capParamUpdateReq_t;

// Connection Parameter Update Response format
typedef struct
{
    uint16 result; // Result
} l2capParamUpdateRsp_t;

// Union of all L2CAP Signaling commands
typedef union
{
    // Requests
    l2capEchoReq_t echoReq;
    l2capInfoReq_t infoReq;
    l2capParamUpdateReq_t updateReq;

    // Responses
    l2capCmdReject_t cmdReject;
    l2capEchoRsp_t echoRsp;
    l2capInfoRsp_t infoRsp;
    l2capParamUpdateRsp_t updateRsp;
} l2capSignalCmd_t;

// OSAL L2CAP_SIGNAL_EVENT message format. This message is used to deliver an
// incoming Signaling command up to an upper layer application.
typedef struct
{
    osal_event_hdr_t hdr; // L2CAP_SIGNAL_EVENT and status
    uint16 connHandle;    // connection message was received on
    uint8 id;             // identifier to match responses with requests
    uint8 opcode;         // type of command
    l2capSignalCmd_t cmd; // command data
} l2capSignalEvent_t;

// L2CAP packet structure
typedef struct
{
    uint16 CID;      // local channel id
    uint8* pPayload; // pointer to information payload. This contains the payload
    // received from the upper layer protocol (outgoing packet),
    // or delivered to the upper layer protocol (incoming packet).
    uint16 len;      // length of information payload
} l2capPacket_t;

// OSAL L2CAP_DATA_EVENT message format. This message is used to forward an
// incoming data packet up to an upper layer application.
typedef struct
{
    osal_event_hdr_t hdr; // L2CAP_DATA_EVENT and status
    uint16 connHandle;    // connection packet was received on
    l2capPacket_t pkt;    // received packet
} l2capDataEvent_t;


typedef struct
{
    uint16 cIdx;          // reassemble packet current idx
    l2capPacket_t pkt;    // received packet
} l2capReassemblePkt_t;

typedef struct
{
    uint8  len;           // pkt len
    uint8* ptr ;          // pkt point
} segmentBuff_t;

typedef struct
{
    segmentBuff_t pkt[10];//251/27->9.2
    uint8 depth;
    uint8 idx;
    uint8* pBufScr;       //source buffer ptr
    uint8 fragment;
} l2capSegmentBuff_t;


typedef struct
{
    uint32 reassembleInCnt;
    uint32 reassembleOutCnt;
    uint32 reassembleErrIdx;
    uint32 reassembleErrCID;
    uint32 reassembleErrInComp;
    uint32 reassembleErrMiss;
    uint32 resssambleMemAlocErr;

    uint32 segmentInCnt;
    uint32 segmentOutCnt;
    uint32 segmentErrCnt;
    uint32 fragmentSendCounter;
    uint32 segmentMemAlocErr;
    uint32 segmentSentToLinkLayerErr;

} l2capSARDbugCnt_t;
//typedef enum
//{
//  DATA_IN_YBUF_FIRST   = 0,            // YBUF fisrt bufin fisrt shift out
//  DATA_IN_XBUF_FIRST   = 1
//} SegmentBuffOrder_t;

//typedef struct
//{
//    l2capSegmentBuff_t  xBuf;
//    l2capSegmentBuff_t  yBuf;
//    SegmentBuffOrder_t  order;     //which buffer
//
//}l2capSegmentPkt_t;



/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/*
    Initialize L2CAP layer.
*/
extern void L2CAP_Init( uint8 taskId );

/*
    L2CAP Task event processing function.
*/
extern uint16 L2CAP_ProcessEvent( uint8 taskId, uint16 events );

/*
    Register a protocol/application with an L2CAP channel.
*/
extern bStatus_t L2CAP_RegisterApp( uint8 taskId, uint16 CID );

/*
    Send L2CAP Data Packet.
*/
extern bStatus_t L2CAP_SendData( uint16 connHandle, l2capPacket_t* pPkt );

/*
    Send Command Reject.
*/
extern bStatus_t L2CAP_CmdReject( uint16 connHandle, uint8 id, l2capCmdReject_t* pCmdReject );

/*
    Build Command Reject.
*/
extern uint16 L2CAP_BuildCmdReject( uint8* pBuf, uint8* pCmd );

/*
    Send L2CAP Echo Request.
*/
extern bStatus_t L2CAP_EchoReq( uint16 connHandle, l2capEchoReq_t* pEchoReq, uint8 taskId );

/*
    Send L2CAP Information Request.
*/
extern bStatus_t L2CAP_InfoReq( uint16 connHandle, l2capInfoReq_t* pInfoReq, uint8 taskId );

/*
    Build Information Response.
*/
extern uint16 L2CAP_BuildInfoRsp( uint8* pBuf, uint8* pCmd );

/*
    Parse Information Request.
*/
extern bStatus_t L2CAP_ParseInfoReq( l2capSignalCmd_t* pCmd, uint8* pData, uint16 len );

/*
    Send L2CAP Connection Parameter Update Request.
*/
extern bStatus_t L2CAP_ConnParamUpdateReq( uint16 connHandle, l2capParamUpdateReq_t* pUpdateReq, uint8 taskId );

/*
    Parse Connection Parameter Update Request.
*/
extern bStatus_t L2CAP_ParseParamUpdateReq( l2capSignalCmd_t* pCmd, uint8* pData, uint16 len );

/*
    Send L2CAP Connection Parameter Update Response.
*/
extern bStatus_t L2CAP_ConnParamUpdateRsp( uint16 connHandle, uint8 id, l2capParamUpdateRsp_t* pUpdateRsp );

/*
    Build Connection Parameter Update Response.
*/
extern uint16 L2CAP_BuildParamUpdateRsp( uint8* pBuf, uint8* pData );

/*
    Allocate a block of memory at the L2CAP layer.
*/
extern void* L2CAP_bm_alloc( uint16 size );

/*
    This API is used by the upper layer to turn flow control on
    or off for data packets sent from the Controller to the Host.
*/
extern void L2CAP_SetControllerToHostFlowCtrl( uint16 hostBuffSize, uint8 flowCtrlMode );
/*
    This API is used by the upper layer to turn flow control on
    or off for data packets sent from the Controller to the Host.
    support DLE update
*/
extern void L2CAP_SetControllerToHostFlowCtrl_DLE( uint16 hostBuffSize, uint8 flowCtrlMode );
/*
    This API is used by the upper layer to notify L2CAP of the
    number of data packets that have been completed for connection
    handle since this API was previously called.
*/
extern void L2CAP_HostNumCompletedPkts( uint16 connHandle, uint16 numCompletedPkts );


extern uint8 l2capPktToSegmentBuff(uint16 connHandle,l2capSegmentBuff_t* pSegBuf, uint8 blen,uint8* pBuf);
extern uint8 l2capSegmentBuffToLinkLayer(uint16 connHandle, l2capSegmentBuff_t* pSegBuf);
extern void l2capPocessFragmentTxData(uint16 connHandle);
extern void l2capSarBufReset(void);
extern void L2CAP_ReassemblePkt_Reset(uint16 connHandle);
extern void L2CAP_SegmentPkt_Reset(uint16 connHandle);

extern void L2CAP_ExtendFramgents_Config(uint8 flag);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* L2CAP_H */
