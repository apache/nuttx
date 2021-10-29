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
    @headerfile:    att.h

    <!--
    Revised:
    Revision:

    Description:    This file contains Attribute Protocol (ATT) definitions
                  and prototypes.



    -->
**************************************************************************************************/

#ifndef ATT_H
#define ATT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"

#include "l2cap.h"

/*********************************************************************
    CONSTANTS
*/

// The Exchanging MTU Size is defined as the maximum size of any packet
// transmitted between a client and a server. A higher layer specification
// defines the default ATT MTU value. The ATT MTU value should be within
// the range 23 to 517 inclusive.
#define ATT_MTU_SIZE                     L2CAP_MTU_SIZE //!< Minimum ATT MTU size
#define ATT_MAX_MTU_SIZE                 517            //!< Maximum ATT MTU size
#define ATT_MTU_SIZE_MIN                 23
/** @defgroup ATT_METHOD_DEFINES ATT Methods
    @{
*/

#define ATT_ERROR_RSP                    0x01 //!< ATT Error Response
#define ATT_EXCHANGE_MTU_REQ             0x02 //!< ATT Exchange MTU Request
#define ATT_EXCHANGE_MTU_RSP             0x03 //!< ATT Exchange MTU Response
#define ATT_FIND_INFO_REQ                0x04 //!< ATT Find Information Request
#define ATT_FIND_INFO_RSP                0x05 //!< ATT Find Information Response
#define ATT_FIND_BY_TYPE_VALUE_REQ       0x06 //!< ATT Find By Type Vaue Request
#define ATT_FIND_BY_TYPE_VALUE_RSP       0x07 //!< ATT Find By Type Vaue Response
#define ATT_READ_BY_TYPE_REQ             0x08 //!< ATT Read By Type Request
#define ATT_READ_BY_TYPE_RSP             0x09 //!< ATT Read By Type Response
#define ATT_READ_REQ                     0x0a //!< ATT Read Request
#define ATT_READ_RSP                     0x0b //!< ATT Read Response
#define ATT_READ_BLOB_REQ                0x0c //!< ATT Read Blob Request
#define ATT_READ_BLOB_RSP                0x0d //!< ATT Read Blob Response
#define ATT_READ_MULTI_REQ               0x0e //!< ATT Read Multiple Request
#define ATT_READ_MULTI_RSP               0x0f //!< ATT Read Multiple Response
#define ATT_READ_BY_GRP_TYPE_REQ         0x10 //!< ATT Read By Group Type Request
#define ATT_READ_BY_GRP_TYPE_RSP         0x11 //!< ATT Read By Group Type Response
#define ATT_WRITE_REQ                    0x12 //!< ATT Write Request
#define ATT_WRITE_RSP                    0x13 //!< ATT Write Response
#define ATT_PREPARE_WRITE_REQ            0x16 //!< ATT Prepare Write Request
#define ATT_PREPARE_WRITE_RSP            0x17 //!< ATT Prepare Write Response
#define ATT_EXECUTE_WRITE_REQ            0x18 //!< ATT Execute Write Request
#define ATT_EXECUTE_WRITE_RSP            0x19 //!< ATT Execute Write Response
#define ATT_HANDLE_VALUE_NOTI            0x1b //!< ATT Handle Value Notification
#define ATT_HANDLE_VALUE_IND             0x1d //!< ATT Handle Value Indication
#define ATT_HANDLE_VALUE_CFM             0x1e //!< ATT Handle Value Confirmation

#define ATT_WRITE_CMD                    0x52 //!< ATT Write Command
#define ATT_SIGNED_WRITE_CMD             0xD2 //!< ATT Signed Write Command

/** @} End ATT_METHOD_DEFINES */

/*** Opcode fields: bitmasks ***/
// Method (bits 5-0)
#define ATT_METHOD_BITS                  0x3f

// Command Flag (bit 6)
#define ATT_CMD_FLAG_BIT                 0x40

// Authentication Signature Flag (bit 7)
#define ATT_AUTHEN_SIG_FLAG_BIT          0x80

// Size of 16-bit Bluetooth UUID
#define ATT_BT_UUID_SIZE                 2

// Size of 128-bit UUID
#define ATT_UUID_SIZE                    16

// ATT Response or Confirmation timeout
#define ATT_MSG_TIMEOUT                  30

// Authentication Signature status for received PDU; it's TRUE or FALSE for PDU to be sent
#define ATT_SIG_NOT_INCLUDED             0x00 // Signature not included
#define ATT_SIG_VALID                    0x01 // Included signature valid
#define ATT_SIG_INVALID                  0x02 // Included signature not valid

/*********************************************************************
    Error Response: Error Code
*/

/** @defgroup ATT_ERR_CODE_DEFINES ATT Error Codes
    @{
*/

#define ATT_ERR_INVALID_HANDLE           0x01 //!< Attribute handle value given was not valid on this attribute server
#define ATT_ERR_READ_NOT_PERMITTED       0x02 //!< Attribute cannot be read
#define ATT_ERR_WRITE_NOT_PERMITTED      0x03 //!< Attribute cannot be written
#define ATT_ERR_INVALID_PDU              0x04 //!< The attribute PDU was invalid
#define ATT_ERR_INSUFFICIENT_AUTHEN      0x05 //!< The attribute requires authentication before it can be read or written
#define ATT_ERR_UNSUPPORTED_REQ          0x06 //!< Attribute server doesn't support the request received from the attribute client
#define ATT_ERR_INVALID_OFFSET           0x07 //!< Offset specified was past the end of the attribute
#define ATT_ERR_INSUFFICIENT_AUTHOR      0x08 //!< The attribute requires an authorization before it can be read or written
#define ATT_ERR_PREPARE_QUEUE_FULL       0x09 //!< Too many prepare writes have been queued
#define ATT_ERR_ATTR_NOT_FOUND           0x0a //!< No attribute found within the given attribute handle range
#define ATT_ERR_ATTR_NOT_LONG            0x0b //!< Attribute cannot be read or written using the Read Blob Request or Prepare Write Request
#define ATT_ERR_INSUFFICIENT_KEY_SIZE    0x0c //!< The Encryption Key Size used for encrypting this link is insufficient
#define ATT_ERR_INVALID_VALUE_SIZE       0x0d //!< The attribute value length is invalid for the operation
#define ATT_ERR_UNLIKELY                 0x0e //!< The attribute request that was requested has encountered an error that was very unlikely, and therefore could not be completed as requested
#define ATT_ERR_INSUFFICIENT_ENCRYPT     0x0f //!< The attribute requires encryption before it can be read or written
#define ATT_ERR_UNSUPPORTED_GRP_TYPE     0x10 //!< The attribute type is not a supported grouping attribute as defined by a higher layer specification
#define ATT_ERR_INSUFFICIENT_RESOURCES   0x11 //!< Insufficient Resources to complete the request

/*** Reserved for future use: 0x12 - 0x7F ***/

/*** Application error code defined by a higher layer specification: 0x80-0x9F ***/

#define ATT_ERR_INVALID_VALUE            0x80 //!< The attribute value is invalid for the operation

/** @} End ATT_ERR_CODE_DEFINES */

/*********************************************************************
    Find Information Response: UUID Format
*/
// Handle and 16-bit Bluetooth UUID
#define ATT_HANDLE_BT_UUID_TYPE          0x01

// Handle and 128-bit UUID
#define ATT_HANDLE_UUID_TYPE             0x02

// Maximum number of handle and 16-bit UUID pairs in a single Find Info Response
#define ATT_MAX_NUM_HANDLE_BT_UUID       ( ( ATT_MTU_SIZE_MIN - 2 ) / ( 2 + ATT_BT_UUID_SIZE ) )

// Maximum number of handle and 128-bit UUID pairs in a single Find Info Response
#define ATT_MAX_NUM_HANDLE_UUID          ( ( ATT_MTU_SIZE_MIN - 2 ) / ( 2 + ATT_UUID_SIZE ) )

/*********************************************************************
    Find By Type Value Response: Handles Infomation (Found Attribute Handle and Group End Handle)
*/
// Maximum number of handles info in a single Find By Type Value Response
#define ATT_MAX_NUM_HANDLES_INFO         ( ( ATT_MTU_SIZE - 1 ) / 4 )

/*********************************************************************
    Read Multiple Request: Handles
*/
// Maximum number of handles in a single Read Multiple Request
#define ATT_MAX_NUM_HANDLES              ( ( ATT_MTU_SIZE - 1 ) / 2 )

// Minimum number of handles in a single Read Multiple Request
#define ATT_MIN_NUM_HANDLES              2

/*********************************************************************
    Execute Write Request: Flags
*/
// Cancel all prepared writes
#define ATT_CANCEL_PREPARED_WRITES       0x00

// Immediately write all pending prepared values
#define ATT_WRITE_PREPARED_VALUES        0x01

#if defined ( TESTMODES )
// ATT Test Modes
#define ATT_TESTMODE_OFF               0 // Test mode off
#define ATT_TESTMODE_UNAUTHEN_SIG      1 // Do not authenticate incoming signature
#endif

/*********************************************************************
    Size of mandatory fields of ATT requests
*/
// Length of Read By Type Request's fixed fields: First handle number (2) + Last handle number (2)
#define READ_BY_TYPE_REQ_FIXED_SIZE        4

// Length of Prepare Write Request's fixed size: Attribute Handle (2) + Value Offset (2)
#define PREPARE_WRITE_REQ_FIXED_SIZE       4

/*********************************************************************
    VARIABLES
*/
extern CONST uint8 btBaseUUID[ATT_UUID_SIZE];

/*********************************************************************
    MACROS
*/

/*********************************************************************
    TYPEDEFS
*/

/**
    Attribute Protocol PDU format.
*/
typedef struct
{
    uint8 sig;      //!< Authentication Signature status (not included (0), valid (1), invalid (2))
    uint8 cmd;      //!< Command Flag
    uint8 method;   //!< Method
    uint16 len;     //!< Length of Attribute Parameters
    uint8* pParams; //!< Attribute Parameters
} attPacket_t;

/**
    Attribute Type format (2 or 16 octet UUID).
*/
typedef struct
{
    uint8 len;                 //!< Length of UUID
    uint8 uuid[ATT_UUID_SIZE]; //!< 16 or 128 bit UUID
} attAttrType_t;

/**
    Attribute Type format (2-octet Bluetooth UUID).
*/
typedef struct
{
    uint8 len;                    //!< Length of UUID
    uint8 uuid[ATT_BT_UUID_SIZE]; //!< 16 bit UUID
} attAttrBtType_t;

/**
    Error Response format.
*/
typedef struct
{
    uint8 reqOpcode; //!< Request that generated this error response
    uint16 handle;   //!< Attribute handle that generated error response
    uint8 errCode;   //!< Reason why the request has generated error response
} attErrorRsp_t;

/**
    Exchange MTU Request format.
*/
typedef struct
{
    uint16 clientRxMTU; //!< Client receive MTU size
} attExchangeMTUReq_t;

/**
    Exchange MTU Response format.
*/
typedef struct
{
    uint16 serverRxMTU; //!< Server receive MTU size
} attExchangeMTURsp_t;

typedef struct
{
    uint16 clientMTU;
    uint16 serverMTU;
} attMTU_t;

/**
    Find Information Request format.
*/
typedef struct
{
    uint16 startHandle;       //!< First requested handle number (must be first field)
    uint16 endHandle;         //!< Last requested handle number
} attFindInfoReq_t;

/**
    Handle and its 16-bit Bluetooth UUIDs.
*/
typedef struct
{
    uint16 handle;                //!< Handle
    uint8 uuid[ATT_BT_UUID_SIZE]; //!< 2-octet Bluetooth UUID
} attHandleBtUUID_t;

/**
    Handle and its 128-bit UUID.
*/
typedef struct
{
    uint16 handle;             //!< Handle
    uint8 uuid[ATT_UUID_SIZE]; //!< 16-octect UUID
} attHandleUUID_t;

/**
    Info data format for Find Information Response (handle-UUID pair).
*/
typedef union
{
    attHandleBtUUID_t btPair[ATT_MAX_NUM_HANDLE_BT_UUID]; //!< A list of 1 or more handles with their 16-bit Bluetooth UUIDs
    attHandleUUID_t   pair[ATT_MAX_NUM_HANDLE_UUID];      //!< A list of 1 or more handles with their 128-bit UUIDs
} attFindInfo_t;

/**
    Find Information Response format.
*/
typedef struct
{
    uint8 numInfo;      //!< Number of attribute handle-UUID pairs found
    uint8 format;       //!< Format of information data
    attFindInfo_t info; //!< Information data whose format is determined by format field
} attFindInfoRsp_t;

/**
    Find By Type Value Request format.
*/
typedef struct
{
    uint16 startHandle;          //!< First requested handle number (must be first field)
    uint16 endHandle;            //!< Last requested handle number
    attAttrBtType_t type;        //!< 2-octet UUID to find
//  uint8 len;                   //!< Length of value
    uint16 len;                  //!< Length of value
    uint8 value[ATT_MTU_SIZE-7]; //!< Attribute value to find
} attFindByTypeValueReq_t;

/**
    Handles Infomation format.
*/
typedef struct
{
    uint16 handle;       //!< Found attribute handle
    uint16 grpEndHandle; //!< Group end handle
} attHandlesInfo_t;

/**
    Find By Type Value Response format.
*/
typedef struct
{
    uint8 numInfo;                                          //!< Number of handles information found
    attHandlesInfo_t handlesInfo[ATT_MAX_NUM_HANDLES_INFO]; //!< List of 1 or more handles information
} attFindByTypeValueRsp_t;

/**
    Read By Type Request format.
*/
typedef struct
{
    uint16 startHandle; //!< First requested handle number (must be first field)
    uint16 endHandle;   //!< Last requested handle number
    attAttrType_t type; //!< Requested type (2 or 16 octet UUID)
} attReadByTypeReq_t;

/**
    Read By Type Response format.
*/
typedef struct
{
    uint8 numPairs;                 //!< Number of attribute handle-UUID pairs found
//  uint8 len;                      //!< Size of each attribute handle-value pair
    uint16 len;
    uint8 dataList[ATT_MTU_SIZE-2]; //!< List of 1 or more attribute handle-value pairs
} attReadByTypeRsp_t;

/**
    Read Request format.
*/
typedef struct
{
    uint16 handle; //!< Handle of the attribute to be read (must be first field)
} attReadReq_t;

/**
    Read Response format.
*/
typedef struct
{
//  uint8 len;                   //!< Length of value
    uint16 len;
    uint8 value[ATT_MTU_SIZE-1]; //!< Value of the attribute with the handle given
} attReadRsp_t;

/**
    Read Blob Req format.
*/
typedef struct
{
    uint16 handle; //!< Handle of the attribute to be read (must be first field)
    uint16 offset; //!< Offset of the first octet to be read
} attReadBlobReq_t;

/**
    Read Blob Response format.
*/
typedef struct
{
//  uint8 len;                   //!< Length of value
    uint16 len;
    uint8 value[ATT_MTU_SIZE-1]; //!< Part of the value of the attribute with the handle given
} attReadBlobRsp_t;

/**
    Read Multiple Request format.
*/
typedef struct
{
    uint16 handle[ATT_MAX_NUM_HANDLES]; //!< Set of two or more attribute handles (must be first field)
    uint8 numHandles;                   //!< Number of attribute handles
} attReadMultiReq_t;

/**
    Read Multiple Response format.
*/
typedef struct
{
//  uint8 len;                    //!< Length of values
    uint16 len;
    uint8 values[ATT_MTU_SIZE-1]; //!< Set of two or more values
} attReadMultiRsp_t;

/**
    Read By Group Type Request format.
*/
typedef struct
{
    uint16 startHandle; //!< First requested handle number (must be first field)
    uint16 endHandle;   //!< Last requested handle number
    attAttrType_t type; //!< Requested group type (2 or 16 octet UUID)
} attReadByGrpTypeReq_t;

/**
    Read By Group Type Response format.
*/
typedef struct
{
    uint8 numGrps;                  //!< Number of attribute handle, end group handle and value sets found
//  uint8 len;                      //!< Length of each attribute handle, end group handle and value set
    uint16 len;
    uint8 dataList[ATT_MTU_SIZE-2]; //!< List of 1 or more attribute handle, end group handle and value
} attReadByGrpTypeRsp_t;

/**
    Write Request format.
*/
typedef struct
{
    uint16 handle;               //!< Handle of the attribute to be written (must be first field)
//  uint8 len;                   //!< Length of value
    uint16 len;
    uint8 value[ATT_MTU_SIZE-3]; //!< Value of the attribute to be written
    uint8 sig;                   //!< Authentication Signature status (not included (0), valid (1), invalid (2))
    uint8 cmd;                   //!< Command Flag
} attWriteReq_t;

/**
    Prepare Write Request format.
*/
typedef struct
{
    uint16 handle;               //!< Handle of the attribute to be written (must be first field)
    uint16 offset;               //!< Offset of the first octet to be written
//  uint8 len;                   //!< Length of value
    uint16 len;
    uint8 value[ATT_MTU_SIZE-5]; //!< Part of the value of the attribute to be written
} attPrepareWriteReq_t;

/**
    Prepare Write Response format.
*/
typedef struct
{
    uint16 handle;               //!< Handle of the attribute that has been read
    uint16 offset;               //!< Offset of the first octet to be written
//  uint8 len;                   //!< Length of value
    uint16 len;
    uint8 value[ATT_MTU_SIZE-5]; //!< Part of the value of the attribute to be written
} attPrepareWriteRsp_t;

/**
    Execute Write Request format.
*/
typedef struct
{
    uint8 flags; //!< 0x00 - cancel all prepared writes.
    //!< 0x01 - immediately write all pending prepared values.
} attExecuteWriteReq_t;

/**
    Handle Value Notification format.
*/
typedef struct
{
    uint16 handle;               //!< Handle of the attribute that has been changed (must be first field)
//  uint8 len;                   //!< Length of value
    uint16 len;                  //!< Length of value
    uint8 value[ATT_MTU_SIZE-3]; //!< New value of the attribute
} attHandleValueNoti_t;

/**
    Handle Value Indication format.
*/
typedef struct
{
    uint16 handle;               //!< Handle of the attribute that has been changed (must be first field)
//  uint8 len;                   //!< Length of value
    uint16 len;
    uint8 value[ATT_MTU_SIZE-3]; //!< New value of the attribute
} attHandleValueInd_t;

/**
    ATT Message format. It's a union of all attribute protocol messages used
    between the attribute protocol and upper layer profile/application.
*/
typedef union
{
    // Request messages
    attExchangeMTUReq_t exchangeMTUReq;         //!< ATT Exchange MTU Request
    attFindInfoReq_t findInfoReq;               //!< ATT Find Information Request
    attFindByTypeValueReq_t findByTypeValueReq; //!< ATT Find By Type Vaue Request
    attReadByTypeReq_t readByTypeReq;           //!< ATT Read By Type Request
    attReadReq_t readReq;                       //!< ATT Read Request
    attReadBlobReq_t readBlobReq;               //!< ATT Read Blob Request
    attReadMultiReq_t readMultiReq;             //!< ATT Read Multiple Request
    attReadByGrpTypeReq_t readByGrpTypeReq;     //!< ATT Read By Group Type Request
    attWriteReq_t writeReq;                     //!< ATT Write Request
    attPrepareWriteReq_t prepareWriteReq;       //!< ATT Prepare Write Request
    attExecuteWriteReq_t executeWriteReq;       //!< ATT Execute Write Request

    // Response messages
    attErrorRsp_t errorRsp;                     //!< ATT Error Response
    attExchangeMTURsp_t exchangeMTURsp;         //!< ATT Exchange MTU Response
    attFindInfoRsp_t findInfoRsp;               //!< ATT Find Information Response
    attFindByTypeValueRsp_t findByTypeValueRsp; //!< ATT Find By Type Vaue Response
    attReadByTypeRsp_t readByTypeRsp;           //!< ATT Read By Type Response
    attReadRsp_t readRsp;                       //!< ATT Read Response
    attReadBlobRsp_t readBlobRsp;               //!< ATT Read Blob Response
    attReadMultiRsp_t readMultiRsp;             //!< ATT Read Multiple Response
    attReadByGrpTypeRsp_t readByGrpTypeRsp;     //!< ATT Read By Group Type Response
    attPrepareWriteRsp_t prepareWriteRsp;       //!< ATT Prepare Write Response

    // Indication and Notification messages
    attHandleValueNoti_t handleValueNoti;       //!< ATT Handle Value Notification
    attHandleValueInd_t handleValueInd;         //!< ATT Handle Value Indication
} attMsg_t;

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    API FUNCTIONS
*/

/*  -------------------------------------------------------------------
    General Utility APIs
*/

/*
    Parse an attribute protocol message.
*/
extern uint8 ATT_ParsePacket( l2capDataEvent_t* pL2capMsg, attPacket_t* pPkt );

/*
    Compare two UUIDs. The UUIDs are converted if necessary.
*/
extern uint8 ATT_CompareUUID( const uint8* pUUID1, uint16 len1,
                              const uint8* pUUID2, uint16 len2 );
/*
    Convert a 16-bit UUID to 128-bit UUID.
*/
extern uint8 ATT_ConvertUUIDto128( const uint8* pUUID16, uint8* pUUID128 );

/*
    Convert a 128-bit UUID to 16-bit UUID.
*/
extern uint8 ATT_ConvertUUIDto16( const uint8* pUUID128, uint8* pUUID16 );


/*  -------------------------------------------------------------------
    Attribute Client Utility APIs
*/

/*
    Build Error Response.
*/
extern uint16 ATT_BuildErrorRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Error Response.
*/
extern bStatus_t ATT_ParseErrorRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Exchange MTU Request.
*/
extern uint16 ATT_BuildExchangeMTUReq( uint8* pBuf, uint8* pMsg );

/*
    Build Exchange MTU Respnose.
*/
extern uint16 ATT_BuildExchangeMTURsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Exchange MTU Response.
*/
extern bStatus_t ATT_ParseExchangeMTURsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Find Information Request.
*/
extern uint16 ATT_BuildFindInfoReq( uint8* pBuf, uint8* pMsg );

/*
    Parse Find Information Response.
*/
extern bStatus_t ATT_ParseFindInfoRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Find Information Response.
*/
extern uint16 ATT_BuildFindInfoRsp( uint8* pBuf, uint8* pMsg );

/*
    Build Find By Type Value Request.
*/
extern uint16 ATT_BuildFindByTypeValueReq( uint8* pBuf, uint8* pMsg );

/*
    Build Find By Type Value Response.
*/
extern uint16 ATT_BuildFindByTypeValueRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Find By Type Value Response.
*/
extern bStatus_t ATT_ParseFindByTypeValueRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Read By Type Request.
*/
extern uint16 ATT_BuildReadByTypeReq( uint8* pBuf, uint8* pMsg );

/*
    Build Read By Type Response.
*/
extern uint16 ATT_BuildReadByTypeRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Read By Type Response.
*/
extern bStatus_t ATT_ParseReadByTypeRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Read Request.
*/
extern uint16 ATT_BuildReadReq( uint8* pBuf, uint8* pMsg );

/*
    Build Read Response.
*/
extern uint16 ATT_BuildReadRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Read Response.
*/
extern bStatus_t ATT_ParseReadRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Read Blob Request.
*/
extern uint16 ATT_BuildReadBlobReq( uint8* pBuf, uint8* pMsg );

/*
    Build Read Blob Response.
*/
extern uint16 ATT_BuildReadBlobRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Read Blob Response.
*/
extern bStatus_t ATT_ParseReadBlobRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Read Multiple Request.
*/
extern uint16 ATT_BuildReadMultiReq( uint8* pBuf, uint8* pMsg );

/*
    Build Read Multiple Response.
*/
extern uint16 ATT_BuildReadMultiRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Read Multiple Response.
*/
extern bStatus_t ATT_ParseReadMultiRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Read By Group Type Response.
*/
extern uint16 ATT_BuildReadByGrpTypeRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Read By Group Type Response.
*/
extern bStatus_t ATT_ParseReadByGrpTypeRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Write Request.
*/
extern uint16 ATT_BuildWriteReq( uint8* pBuf, uint8* pMsg );

/*
    Parse Write Response.
*/
extern bStatus_t ATT_ParseWriteRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Prepare Write Request.
*/
extern uint16 ATT_BuildPrepareWriteReq( uint8* pBuf, uint8* pMsg );

/*
    Build Prepare Write Response.
*/
extern uint16 ATT_BuildPrepareWriteRsp( uint8* pBuf, uint8* pMsg );

/*
    Parse Prepare Write Response.
*/
extern bStatus_t ATT_ParsePrepareWriteRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Execute Write Request.
*/
extern uint16 ATT_BuildExecuteWriteReq( uint8* pBuf, uint8* pMsg );

/*
    Parse Execute Write Response.
*/
extern bStatus_t ATT_ParseExecuteWriteRsp( uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Build Handle Value Indication.
*/
extern uint16 ATT_BuildHandleValueInd( uint8* pBuf, uint8* pMsg );

/*
    Parse Handle Value Indication.
*/
extern bStatus_t ATT_ParseHandleValueInd( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );


/*  -------------------------------------------------------------------
    Attribute Server Utility APIs
*/

/*
    Parse Exchange MTU Request.
*/
extern bStatus_t ATT_ParseExchangeMTUReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Find Information Request.
*/
extern bStatus_t ATT_ParseFindInfoReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Find By Type Value Request.
*/
extern bStatus_t ATT_ParseFindByTypeValueReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Read By Type Request.
*/
extern bStatus_t ATT_ParseReadByTypeReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Read Request.
*/
extern bStatus_t ATT_ParseReadReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Write Blob Request.
*/
extern bStatus_t ATT_ParseReadBlobReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Read Multiple Request.
*/
extern bStatus_t ATT_ParseReadMultiReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Write Request.
*/
extern bStatus_t ATT_ParseWriteReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Execute Write Request.
*/
extern bStatus_t ATT_ParseExecuteWriteReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Prepare Write Request.
*/
extern bStatus_t ATT_ParsePrepareWriteReq( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

/*
    Parse Handle Value Confirmation.
*/
extern bStatus_t ATT_ParseHandleValueCfm( uint8* pParams, uint16 len, attMsg_t* pMsg );


/*  -------------------------------------------------------------------
    Attribute Client Public APIs
*/

/**
    @defgroup ATT_CLIENT_API ATT Client API Functions

    @{
*/

/**
    @brief   Send Exchange MTU Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ExchangeMTUReq( uint16 connHandle, attExchangeMTUReq_t* pReq );

/**
    @brief   Send Find Information Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_FindInfoReq( uint16 connHandle, attFindInfoReq_t* pReq );

/**
    @brief   Send Find By Type Value Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_FindByTypeValueReq( uint16 connHandle, attFindByTypeValueReq_t* pReq );

/**
    @brief   Send Read By Type Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadByTypeReq( uint16 connHandle, attReadByTypeReq_t* pReq );

/**
    @brief   Send Read Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadReq( uint16 connHandle, attReadReq_t* pReq );

/**
    @brief   Send Read Blob Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadBlobReq( uint16 connHandle, attReadBlobReq_t* pReq );

/**
    @brief   Send Read Multiple Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadMultiReq( uint16 connHandle, attReadMultiReq_t* pReq );

/**
    @brief   Send Read By Group Type Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadByGrpTypeReq( uint16 connHandle, attReadByGrpTypeReq_t* pReq );

/**
    @brief   Send Write Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleLinkEncrypted: Connection is already encrypted.<BR>
*/
extern bStatus_t ATT_WriteReq( uint16 connHandle, attWriteReq_t* pReq );

/**
    @brief   Send Prepare Write Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_PrepareWriteReq( uint16 connHandle, attPrepareWriteReq_t* pReq );

/**
    @brief   Send Execute Write Request.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ExecuteWriteReq( uint16 connHandle, attExecuteWriteReq_t* pReq );

/**
    @brief   Send Handle Value Confirmation.

    @param   connHandle - connection to use

    @return  SUCCESS: Confirmation was sent successfully.<BR>
            INVALIDPARAMETER: Invalid confirmation field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_HandleValueCfm( uint16 connHandle );

/**
    @}
*/

/*  -------------------------------------------------------------------
    Attribute Server Public APIs
*/

/**
    @defgroup ATT_SERVER_API ATT Server API Functions

    @{
*/

/**
    @brief   Send Error Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to error response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ErrorRsp( uint16 connHandle, attErrorRsp_t* pRsp );

/**
    @brief   Send Exchange MTU Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to request to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ExchangeMTURsp( uint16 connHandle, attExchangeMTURsp_t* pRsp );

/**
    @brief   Send Find Information Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_FindInfoRsp( uint16 connHandle, attFindInfoRsp_t* pRsp );

/**
    @brief   Send Find By Tyep Value Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_FindByTypeValueRsp( uint16 connHandle, attFindByTypeValueRsp_t* pRsp );

/**
    @brief   Send Read By Type Respond.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadByTypeRsp( uint16 connHandle, attReadByTypeRsp_t* pRsp );

/**
    @brief   Send Read Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadRsp( uint16 connHandle, attReadRsp_t* pRsp );

/**
    @brief   Send Read Blob Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadBlobRsp( uint16 connHandle, attReadBlobRsp_t* pRsp );

/**
    @brief   Send Read Multiple Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadMultiRsp( uint16 connHandle, attReadMultiRsp_t* pRsp ) ;

/**
    @brief   Send Read By Group Type Respond.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ReadByGrpTypeRsp( uint16 connHandle, attReadByGrpTypeRsp_t* pRsp );

/**
    @brief   Send Write Response.

    @param   connHandle - connection to use

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_WriteRsp( uint16 connHandle );

/**
    @brief   Send Prepare Write Response.

    @param   connHandle - connection to use
    @param   pRsp - pointer to response to be sent

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_PrepareWriteRsp( uint16 connHandle, attPrepareWriteRsp_t* pRsp );

/**
    @brief   Send Execute Write Response.

    @param   connHandle - connection to use

    @return  SUCCESS: Response was sent successfully.<BR>
            INVALIDPARAMETER: Invalid response field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_ExecuteWriteRsp( uint16 connHandle );

/**
    @brief   Send Handle Value Notification.

    @param   connHandle - connection to use
    @param   pNoti - pointer to notification to be sent

    @return  SUCCESS: Notification was sent successfully.<BR>
            INVALIDPARAMETER: Invalid notification field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_HandleValueNoti( uint16 connHandle, attHandleValueNoti_t* pNoti );

/**
    @brief   Send Handle Value Indication.

    @param   connHandle - connection to use
    @param   pInd - pointer to indication to be sent

    @return  SUCCESS: Indication was sent successfully.<BR>
            INVALIDPARAMETER: Invalid indication field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t ATT_HandleValueInd( uint16 connHandle, attHandleValueInd_t* pInd );

/**
    @}
*/

/**
    @brief   Set a ATT Parameter value.  Use this function to change
            the default ATT parameter values.

    @param   value - new param value

    @return  void
*/
extern void ATT_SetParamValue( uint16 value );

/**
    @brief   Get a ATT Parameter value.

    @param   none

    @return  ATT Parameter value
*/
extern uint16 ATT_GetParamValue( void );

extern uint16 ATT_GetCurrentMTUSize( uint16 connHandle );
extern void ATT_UpdateMtuSize(uint16 connHandle, uint16 mtuSize);
extern void ATT_SetMTUSizeMax(uint16 mtuSize);
extern void ATT_MTU_SIZE_UPDATE(uint8 mtuSize);

extern void ATT_InitMtuSize(void);

//extern uint16 g_ATT_MTU_SIZE;
extern uint16 g_ATT_MTU_SIZE_MAX;
extern uint16 g_ATT_MAX_NUM_HANDLES;
extern uint16 g_ATT_MAX_NUM_HANDLES_INFO;
//extern uint16 g_ATT_MAX_NUM_HANDLE_BT_UUID;
extern attMTU_t g_attMtuClientServer;


// for multi-role
extern uint16  gAttMtuSize[];

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ATT_H */
