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
    @headerfile:       gatt.h
    $Date:
    $Revision:

    @mainpage BLE GATT API

    Description:    This file contains Generic Attribute Profile (GATT)
                  definitions and prototypes.<BR><BR>


*/

#ifndef GATT_H
#define GATT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"

#include "att.h"

/*********************************************************************
    CONSTANTS
*/

/** @defgroup GATT_PERMIT_BITMAPS_DEFINES GATT Attribute Access Permissions Bit Fields
    @{
*/

#define GATT_PERMIT_READ                 0x01 //!< Attribute is Readable
#define GATT_PERMIT_WRITE                0x02 //!< Attribute is Writable
#define GATT_PERMIT_AUTHEN_READ          0x04 //!< Read requires Authentication
#define GATT_PERMIT_AUTHEN_WRITE         0x08 //!< Write requires Authentication
#define GATT_PERMIT_AUTHOR_READ          0x10 //!< Read requires Authorization
#define GATT_PERMIT_AUTHOR_WRITE         0x20 //!< Write requires Authorization
#define GATT_PERMIT_ENCRYPT_READ         0x40 //!< Read requires Encryption
#define GATT_PERMIT_ENCRYPT_WRITE        0x80 //!< Write requires Encryption

/** @} End GATT_PERMIT_BITMAPS_DEFINES */


/** @defgroup GATT_NUM_PREPARE_WRITES_DEFINES GATT Maximum Number of Prepare Writes
    @{
*/
#define PREPARE_QUEUE_STATIC
#if !defined( GATT_MAX_NUM_PREPARE_WRITES )
#define GATT_MAX_NUM_PREPARE_WRITES      1//20 //!< GATT Maximum number of attributes that Attribute Server can prepare for writing per Attribute Client
#endif

/** @} End GATT_NUM_PREPARE_WRITES_DEFINES */


/** @defgroup GATT_ENCRYPT_KEY_SIZE_DEFINES GATT Encryption Key Size
    @{
*/

#define GATT_ENCRYPT_KEY_SIZE            16 //!< GATT Encryption Key Size used for encrypting a link

/** @} End GATT_ENCRYPT_KEY_SIZE_DEFINES */


/** @defgroup GATT_MAX_ATTR_SIZE_DEFINES GATT Maximum Attribute Value Length
    @{
*/

#define GATT_MAX_ATTR_SIZE               512 //!< GATT Maximum length of an attribute value

/** @} End GATT_MAX_ATTR_SIZE_DEFINES */

// GATT Maximum number of connections (including loopback)
#define GATT_MAX_NUM_CONN                ( MAX_NUM_LL_CONN + 1 )

// GATT Base Method
#define GATT_BASE_METHOD                 0x40

// Attribute handle defintions
#define GATT_INVALID_HANDLE              0x0000 // Invalid attribute handle
#define GATT_MIN_HANDLE                  0x0001 // Minimum attribute handle
#define GATT_MAX_HANDLE                  0xFFFF // Maximum attribute handle

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    MACROS
*/

// Attribute Access Permissions
#define gattPermitRead( a )              ( (a) & GATT_PERMIT_READ )
#define gattPermitWrite( a )             ( (a) & GATT_PERMIT_WRITE )
#define gattPermitAuthenRead( a )        ( (a) & GATT_PERMIT_AUTHEN_READ )
#define gattPermitAuthenWrite( a )       ( (a) & GATT_PERMIT_AUTHEN_WRITE )
#define gattPermitAuthorRead( a )        ( (a) & GATT_PERMIT_AUTHOR_READ )
#define gattPermitAuthorWrite( a )       ( (a) & GATT_PERMIT_AUTHOR_WRITE )
#define gattPermitEncryptRead( a )       ( (a) & GATT_PERMIT_ENCRYPT_READ )
#define gattPermitEncryptWrite( a )      ( (a) & GATT_PERMIT_ENCRYPT_WRITE )

// Check for different UUID types
#define gattPrimaryServiceType( t )      ( ATT_CompareUUID( primaryServiceUUID, ATT_BT_UUID_SIZE, \
                                                            (t).uuid, (t).len ) )
#define gattSecondaryServiceType( t )    ( ATT_CompareUUID( secondaryServiceUUID, ATT_BT_UUID_SIZE, \
                                                            (t).uuid, (t).len ) )
#define gattCharacterType( t )           ( ATT_CompareUUID( characterUUID, ATT_BT_UUID_SIZE, \
                                                            (t).uuid, (t).len ) )
#define gattIncludeType( t )             ( ATT_CompareUUID( includeUUID, ATT_BT_UUID_SIZE, \
                                                            (t).uuid, (t).len ) )
#define gattServiceType( t )             ( gattPrimaryServiceType( (t) ) || \
                                           gattSecondaryServiceType( (t) ) )

/*********************************************************************
    TYPEDEFS
*/

/**
    GATT Read By Type Request format.
*/
typedef struct
{
    uint8 discCharsByUUID;  //!< Whether this is a GATT Discover Characteristics by UUID sub-procedure
    attReadByTypeReq_t req; //!< Read By Type Request
} gattReadByTypeReq_t;

/**
    GATT Prepare Write Request format.
*/
typedef struct
{
    uint16 handle; //!< Handle of the attribute to be written (must be first field)
    uint16 offset; //!< Offset of the first octet to be written
    uint8 len;     //!< Length of value
    uint8* pValue; //!< Part of the value of the attribute to be written (must be allocated)
} gattPrepareWriteReq_t;

/**
    GATT Write Long Request format. Do not change the order of the members.
*/
typedef struct
{
    uint8 reliable;            //!< Whether reliable writes requested (always FALSE for Write Long)
    gattPrepareWriteReq_t req; //!< GATT Prepare Write Request
    uint16 lastOffset;         //!< Offset of last Prepare Write Request sent
} gattWriteLongReq_t;

/**
    GATT Reliable Writes Request format. Do not change the order of the members.
*/
typedef struct
{
    uint8 reliable;              //!< Whether reliable writes requested (always TRUE for Reliable Writes)
    attPrepareWriteReq_t* pReqs; //!< Arrary of Prepare Write Requests (must be allocated)
    uint8 numReqs;               //!< Number of Prepare Write Requests
    uint8 index;                 //!< Index of last Prepare Write Request sent
    uint8 flags;                 //!< 0x00 - cancel all prepared writes.
    //!< 0x01 - immediately write all pending prepared values.
} gattReliableWritesReq_t;

/**
    GATT Message format. It's a union of all attribute protocol/profile messages
    used between the attribute protocol/profile and upper layer application.
*/
typedef union
{
    // Request messages
    attExchangeMTUReq_t exchangeMTUReq;              //!< ATT Exchange MTU Request
    attFindInfoReq_t findInfoReq;                    //!< ATT Find Information Request
    attFindByTypeValueReq_t findByTypeValueReq;      //!< ATT Find By Type Vaue Request
    attReadByTypeReq_t readByTypeReq;                //!< ATT Read By Type Request
    attReadReq_t readReq;                            //!< ATT Read Request
    attReadBlobReq_t readBlobReq;                    //!< ATT Read Blob Request
    attReadMultiReq_t readMultiReq;                  //!< ATT Read Multiple Request
    attReadByGrpTypeReq_t readByGrpTypeReq;          //!< ATT Read By Group Type Request
    attWriteReq_t writeReq;                          //!< ATT Write Request
    attPrepareWriteReq_t prepareWriteReq;            //!< ATT Prepare Write Request
    attExecuteWriteReq_t executeWriteReq;            //!< ATT Execute Write Request
    gattReadByTypeReq_t gattReadByTypeReq;           //!< GATT Read By Type Request
    gattWriteLongReq_t gattWriteLongReq;             //!< GATT Long Write Request
    gattReliableWritesReq_t gattReliableWritesReq;   //!< GATT Reliable Writes Request

    // Response messages
    attErrorRsp_t errorRsp;                          //!< ATT Error Response
    attExchangeMTURsp_t exchangeMTURsp;              //!< ATT Exchange MTU Response
    attFindInfoRsp_t findInfoRsp;                    //!< ATT Find Information Response
    attFindByTypeValueRsp_t findByTypeValueRsp;      //!< ATT Find By Type Vaue Response
    attReadByTypeRsp_t readByTypeRsp;                //!< ATT Read By Type Response
    attReadRsp_t readRsp;                            //!< ATT Read Response
    attReadBlobRsp_t readBlobRsp;                    //!< ATT Read Blob Response
    attReadMultiRsp_t readMultiRsp;                  //!< ATT Read Multiple Response
    attReadByGrpTypeRsp_t readByGrpTypeRsp;          //!< ATT Read By Group Type Response
    attPrepareWriteRsp_t prepareWriteRsp;            //!< ATT Prepare Write Response

    // Indication and Notification messages
    attHandleValueNoti_t handleValueNoti;            //!< ATT Handle Value Notification
    attHandleValueInd_t handleValueInd;              //!< ATT Handle Value Indication
} gattMsg_t;

/**
    GATT OSAL GATT_MSG_EVENT message format. This message is used to forward an
    incoming attribute protocol/profile message up to upper layer application.
*/
typedef struct
{
    osal_event_hdr_t hdr; //!< GATT_MSG_EVENT and status
    uint16 connHandle;    //!< Connection message was received on
    uint8 method;         //!< Type of message
    gattMsg_t msg;        //!< Attribute protocol/profile message
} gattMsgEvent_t;

/**
    GATT Attribute Type format.
*/
typedef struct
{
    uint8 len;         //!< Length of UUID
    const uint8* uuid; //!< Pointer to UUID
} gattAttrType_t;

/**
    GATT Attribute format.
*/
typedef struct attAttribute_t
{
    gattAttrType_t type; //!< Attribute type (2 or 16 octet UUIDs)
    uint8 permissions;   //!< Attribute permissions
    uint16 handle;       //!< Attribute handle - assigned internally by attribute server
    uint8* const pValue; //!< Attribute value - encoding of the octet array is defined in
    //!< the applicable profile. The maximum length of an attribute
    //!< value shall be 512 octets.
} gattAttribute_t;

/**
    GATT Service format.
*/
typedef struct
{
    uint16 numAttrs; //!< Number of attributes in attrs

    /** Array of attribute records.
        NOTE: The list must start with a Service attribute followed by
              all attributes associated with this Service attribute.
    */
    gattAttribute_t* attrs;
} gattService_t;

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    API FUNCTIONS
*/

/*  -------------------------------------------------------------------
    GATT Client Public APIs
*/

/**
    @defgroup GATT_CLIENT_API GATT Client API Functions

    @{
*/

/**
    @brief   Initialize the Generic Attribute Profile Client.

    @return  SUCCESS
*/
extern bStatus_t GATT_InitClient( void );

/**
    @brief   Register to receive incoming ATT Indications or Notifications
            of attribute values.

    @param   taskId ? task to forward indications or notifications to

    @return  void
*/
extern void GATT_RegisterForInd( uint8 taskId );

/**
    @brief   The Prepare Write Request is used to request the server to
            prepare to write the value of an attribute.

            Note: This function is needed only for GATT testing.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_PrepareWriteReq( uint16 connHandle, attPrepareWriteReq_t* pReq, uint8 taskId );

/**
    @brief   The Execute Write Request is used to request the server to
            write or cancel the write of all the prepared values currently
            held in the prepare queue from this client.

            Note: This function is needed only for GATT testing.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ExecuteWriteReq( uint16 connHandle, attExecuteWriteReq_t* pReq, uint8 taskId );

/**
    @}
*/

/*  -------------------------------------------------------------------
    GATT Server Public APIs
*/

/**
    @defgroup GATT_SERVER_API GATT Server API Functions

    @{
*/

/**
    @brief   Initialize the Generic Attribute Profile Server.

    @return  SUCCESS
*/
extern bStatus_t GATT_InitServer( void );

/**
    @brief   Register a service attribute list with the GATT Server. A service
            is composed of characteristics or references to other services.
            Each characteristic contains a value and may contain optional
            information about the value. There are two types of services:
            primary service and secondary service.

            A service definition begins with a service declaration and ends
            before the next service declaration or the maximum Attribute Handle.

            A characteristic definition begins with a characteristic declaration
            and ends before the next characteristic or service declaration or
            maximum Attribute Handle.

            The attribute server will only keep a pointer to the attribute
            list, so the calling application will have to maintain the code
            and RAM associated with this list.

    @param   pService - pointer to service attribute list to be registered

    @return  SUCCESS: Service registered successfully.<BR>
            INVALIDPARAMETER: Invalid service field.<BR>
            FAILURE: Not enough attribute handles available.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
*/
extern bStatus_t GATT_RegisterService( gattService_t* pService );

/**
    @brief   Deregister a service attribute list with the GATT Server.

            NOTE: It's the caller's responsibility to free the service attribute
            list returned from this API.

    @param   handle - handle of service to be deregistered
    @param   pService - pointer to deregistered service (to be returned)

    @return  SUCCESS: Service deregistered successfully.<BR>
            FAILURE: Service not found.<BR>
*/
extern bStatus_t GATT_DeregisterService( uint16 handle, gattService_t* pService );

/**
    @brief   Register to receive incoming ATT Requests.

    @param   taskId ? task to forward requests to

    @return  void
*/
extern void GATT_RegisterForReq( uint8 taskId );

/**
    @brief   Verify the permissions of an attribute for reading.

    @param   connHandle - connection to use
    @param   permissions - attribute permissions

    @return  SUCCESS: Attribute can be read.<BR>
            ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be read.<BR>
            ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication.<BR>
            ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient.<BR>
            ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption.<BR>
*/
extern bStatus_t GATT_VerifyReadPermissions( uint16 connHandle, uint8 permissions );

/**
    @brief   Verify the permissions of an attribute for writing.

    @param   connHandle - connection to use
    @param   permissions - attribute permissions
    @param   pReq - pointer to write request

    @return  SUCCESS: Attribute can be written.<BR>
            ATT_ERR_READ_NOT_PERMITTED: Attribute cannot be written.<BR>
            ATT_ERR_INSUFFICIENT_AUTHEN: Attribute requires authentication.<BR>
            ATT_ERR_INSUFFICIENT_KEY_SIZE: Key Size used for encrypting is insufficient.<BR>
            ATT_ERR_INSUFFICIENT_ENCRYPT: Attribute requires encryption.<BR>
*/
extern bStatus_t GATT_VerifyWritePermissions( uint16 connHandle, uint8 permissions, attWriteReq_t* pReq );

/**
    @brief   Send out a Service Changed Indication.

    @param   connHandle - connection to use
    @param   taskId - task to be notified of confirmation

    @return  SUCCESS: Indication was sent successfully.<BR>
            FAILURE: Service Changed attribute not found.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A confirmation is pending with this client.<BR>
*/
extern uint8 GATT_ServiceChangedInd( uint16 connHandle, uint8 taskId );

/**
    @brief   Find the attribute record for a given handle and UUID.

    @param   startHandle - first handle to look for
    @param   endHandle - last handle to look for
    @param   pUUID - pointer to UUID to look for
    @param   len - length of UUID
    @param   pHandle - handle of owner of attribute (to be returned)

    @return  Pointer to attribute record. NULL, otherwise.
*/
extern gattAttribute_t* GATT_FindHandleUUID( uint16 startHandle, uint16 endHandle, const uint8* pUUID,
                                             uint16 len, uint16* pHandle );
/**
    @brief   Find the attribute record for a given handle

    @param   handle - handle to look for
    @param   pHandle - handle of owner of attribute (to be returned)

    @return  Pointer to attribute record. NULL, otherwise.
*/
extern gattAttribute_t* GATT_FindHandle( uint16 handle, uint16* pHandle );

/**
    @brief   Find the next attribute of the same type for a given attribute.

    @param   pAttr - pointer to attribute to find a next for
    @param   endHandle - last handle to look for
    @param   service - handle of owner service
    @param   pLastHandle - handle of last attribute (to be returned)

    @return  Pointer to next attribute record. NULL, otherwise.
*/
extern gattAttribute_t* GATT_FindNextAttr( gattAttribute_t* pAttr, uint16 endHandle,
                                           uint16 service, uint16* pLastHandle );
/**
    @brief   Get the number of attributes for a given service

    @param   handle - service handle to look for

    @return  Number of attributes. 0, otherwise.
*/
extern uint16 GATT_ServiceNumAttrs( uint16 handle );

/**
    @}
*/

/*  -------------------------------------------------------------------
    GATT Server Sub-Procedure APIs
*/

/**
    @defgroup GATT_SERVER_SUB_PROCEDURE_API GATT Server Sub-Procedure API Functions

    @{
*/

/**
    @brief   This sub-procedure is used when a server is configured to
            indicate a characteristic value to a client and expects an
            attribute protocol layer acknowledgement that the indication
            was successfully received.

            The ATT Handle Value Indication is used in this sub-procedure.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT message.
            The type of the message will be ATT_HANDLE_VALUE_CFM.

            Note: This sub-procedure is complete when ATT_HANDLE_VALUE_CFM
                  (with SUCCESS or bleTimeout status) is received by the
                  calling application task.

    @param   connHandle - connection to use
    @param   pInd - pointer to indication to be sent
    @param   authenticated - whether an authenticated link is required
    @param   taskId - task to be notified of response

    @return  SUCCESS: Indication was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A confirmation is pending with this client.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_Indication( uint16 connHandle, attHandleValueInd_t* pInd,
                                  uint8 authenticated, uint8 taskId );
/**
    @brief   This sub-procedure is used when a server is configured to
            notify a characteristic value to a client without expecting
            any attribute protocol layer acknowledgement that the
            notification was successfully received.

            The ATT Handle Value Notification is used in this sub-procedure.

            Note: A notification may be sent at any time and does not
            invoke a confirmation.

            No confirmation will be sent to the calling application task for
            this sub-procedure.

    @param   connHandle - connection to use
    @param   pNoti - pointer to notification to be sent
    @param   authenticated - whether an authenticated link is required

    @return  SUCCESS: Notification was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_Notification( uint16 connHandle, attHandleValueNoti_t* pNoti,
                                    uint8 authenticated );
/**
    @}
*/

/*  -------------------------------------------------------------------
    GATT Client Sub-Procedure APIs
*/

/**
    @defgroup GATT_CLIENT_SUB_PROCEDURE_API GATT Client Sub-Procedure API Functions

    @{
*/

/**
    @brief   This sub-procedure is used by the client to set the ATT_MTU
            to the maximum possible value that can be supported by both
            devices when the client supports a value greater than the
            default ATT_MTU for the Attribute Protocol. This sub-procedure
            shall only be initiated once during a connection.

            The ATT Exchange MTU Request is used by this sub-procedure.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT message.
            The type of the message will be either ATT_EXCHANGE_MTU_RSP or
            ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_EXCHANGE_MTU_RSP
                  (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
                  SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ExchangeMTU( uint16 connHandle, attExchangeMTUReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used by a client to discover all
            the primary services on a server.

            The ATT Read By Group Type Request is used with the Attribute
            Type parameter set to the UUID for "Primary Service". The
            Starting Handle is set to 0x0001 and the Ending Handle is
            set to 0xFFFF.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_READ_BY_GRP_TYPE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_BY_GRP_TYPE_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application
                  task.

    @param   connHandle - connection to use
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_DiscAllPrimaryServices( uint16 connHandle, uint8 taskId );

/**
    @brief   This sub-procedure is used by a client to discover a specific
            primary service on a server when only the Service UUID is
            known. The primary specific service may exist multiple times
            on a server. The primary service being discovered is identified
            by the service UUID.

            The ATT Find By Type Value Request is used with the Attribute
            Type parameter set to the UUID for "Primary Service" and the
            Attribute Value set to the 16-bit Bluetooth UUID or 128-bit
            UUID for the specific primary service. The Starting Handle shall
            be set to 0x0001 and the Ending Handle shall be set to 0xFFFF.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_FIND_BY_TYPE_VALUE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_FIND_BY_TYPE_VALUE_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pValue - pointer to value to look for
    @param   len - length of value
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_DiscPrimaryServiceByUUID( uint16 connHandle, uint8* pValue,
                                                uint8 len, uint8 taskId );
/**
    @brief   This sub-procedure is used by a client to find include
            service declarations within a service definition on a
            server. The service specified is identified by the service
            handle range.

            The ATT Read By Type Request is used with the Attribute
            Type parameter set to the UUID for "Included Service". The
            Starting Handle is set to starting handle of the specified
            service and the Ending Handle is set to the ending handle
            of the specified service.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_READ_BY_TYPE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   startHandle - starting handle
    @param   endHandle - end handle
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_FindIncludedServices( uint16 connHandle, uint16 startHandle,
                                            uint16 endHandle, uint8 taskId );
/**
    @brief   This sub-procedure is used by a client to find all the
            characteristic declarations within a service definition on
            a server when only the service handle range is known. The
            service specified is identified by the service handle range.

            The ATT Read By Type Request is used with the Attribute Type
            parameter set to the UUID for "Characteristic". The Starting
            Handle is set to starting handle of the specified service and
            the Ending Handle is set to the ending handle of the specified
            service.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_READ_BY_TYPE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   startHandle - starting handle
    @param   endHandle - end handle
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_DiscAllChars( uint16 connHandle, uint16 startHandle,
                                    uint16 endHandle, uint8 taskId );
/**
    @brief   This sub-procedure is used by a client to discover service
            characteristics on a server when only the service handle
            ranges are known and the characteristic UUID is known.
            The specific service may exist multiple times on a server.
            The characteristic being discovered is identified by the
            characteristic UUID.

            The ATT Read By Type Request is used with the Attribute Type
            is set to the UUID for "Characteristic" and the Starting
            Handle and Ending Handle parameters is set to the service
            handle range.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_READ_BY_TYPE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_DiscCharsByUUID( uint16 connHandle, attReadByTypeReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used by a client to find all the
            characteristic descriptor’s Attribute Handles and Attribute
            Types within a characteristic definition when only the
            characteristic handle range is known. The characteristic
            specified is identified by the characteristic handle range.

            The ATT Find Information Request is used with the Starting
            Handle set to starting handle of the specified characteristic
            and the Ending Handle set to the ending handle of the specified
            characteristic. The UUID Filter parameter is NULL (zero length).

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_FIND_INFO_RSP or
            ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_FIND_INFO_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   startHandle - starting handle
    @param   endHandle - end handle
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_DiscAllCharDescs( uint16 connHandle, uint16 startHandle,
                                        uint16 endHandle, uint8 taskId );
/**
    @brief   This sub-procedure is used to read a Characteristic Value
            from a server when the client knows the Characteristic Value
            Handle. The ATT Read Request is used with the Attribute Handle
            parameter set to the Characteristic Value Handle. The Read
            Response returns the Characteristic Value in the Attribute
            Value parameter.

            The Read Response only contains a Characteristic Value that
            is less than or equal to (ATT_MTU ? 1) octets in length. If
            the Characteristic Value is greater than (ATT_MTU ? 1) octets
            in length, the Read Long Characteristic Value procedure may
            be used if the rest of the Characteristic Value is required.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT message.
            The type of the message will be either ATT_READ_RSP or
            ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_RSP
                  (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
                  SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ReadCharValue( uint16 connHandle, attReadReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to read a Characteristic Value
            from a server when the client only knows the characteristic
            UUID and does not know the handle of the characteristic.

            The ATT Read By Type Request is used to perform the sub-procedure.
            The Attribute Type is set to the known characteristic UUID and
            the Starting Handle and Ending Handle parameters shall be set
            to the range over which this read is to be performed. This is
            typically the handle range for the service in which the
            characteristic belongs.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT messages.
            The type of the message will be either ATT_READ_BY_TYPE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_BY_TYPE_RSP
                  (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
                  SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ReadUsingCharUUID( uint16 connHandle, attReadByTypeReq_t* pReq, uint8 taskId );
/**
    @brief   This sub-procedure is used to read a Characteristic Value from
            a server when the client knows the Characteristic Value Handle
            and the length of the Characteristic Value is longer than can
            be sent in a single Read Response Attribute Protocol message.

            The ATT Read Blob Request is used in this sub-procedure.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_READ_BLOB_RSP or
            ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_BLOB_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ReadLongCharValue( uint16 connHandle, attReadBlobReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to read multiple Characteristic Values
            from a server when the client knows the Characteristic Value
            Handles. The Attribute Protocol Read Multiple Requests is used
            with the Set Of Handles parameter set to the Characteristic Value
            Handles. The Read Multiple Response returns the Characteristic
            Values in the Set Of Values parameter.

            The ATT Read Multiple Request is used in this sub-procedure.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT message.
            The type of the message will be either ATT_READ_MULTI_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_MULTI_RSP
                  (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
                  SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ReadMultiCharValues( uint16 connHandle, attReadMultiReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to write a Characteristic Value
            to a server when the client knows the Characteristic Value
            Handle and the client does not need an acknowledgement that
            the write was successfully performed. This sub-procedure
            only writes the first (ATT_MTU ? 3) octets of a Characteristic
            Value. This sub-procedure can not be used to write a long
            characteristic; instead the Write Long Characteristic Values
            sub-procedure should be used.

            The ATT Write Command is used for this sub-procedure. The
            Attribute Handle parameter shall be set to the Characteristic
            Value Handle. The Attribute Value parameter shall be set to
            the new Characteristic Value.

            No response will be sent to the calling application task for this
            sub-procedure. If the Characteristic Value write request is the
            wrong size, or has an invalid value as defined by the profile,
            then the write will not succeed and no error will be generated
            by the server.

    @param   connHandle - connection to use
    @param   pReq - pointer to command to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_WriteNoRsp( uint16 connHandle, attWriteReq_t* pReq );

/**
    @brief   This sub-procedure is used to write a Characteristic Value
            to a server when the client knows the Characteristic Value
            Handle and the ATT Bearer is not encrypted. This sub-procedure
            shall only be used if the Characteristic Properties authenticated
            bit is enabled and the client and server device share a bond as
            defined in the GAP.

            This sub-procedure only writes the first (ATT_MTU ? 15) octets
            of an Attribute Value. This sub-procedure cannot be used to
            write a long Attribute.

            The ATT Write Command is used for this sub-procedure. The
            Attribute Handle parameter shall be set to the Characteristic
            Value Handle. The Attribute Value parameter shall be set to
            the new Characteristic Value authenticated by signing the
            value, as defined in the Security Manager.

            No response will be sent to the calling application task for this
            sub-procedure. If the authenticated Characteristic Value that is
            written is the wrong size, or has an invalid value as defined by
            the profile, or the signed value does not authenticate the client,
            then the write will not succeed and no error will be generated by
            the server.

    @param   connHandle - connection to use
    @param   pReq - pointer to command to be sent

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleLinkEncrypted: Connection is already encrypted.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_SignedWriteNoRsp( uint16 connHandle, attWriteReq_t* pReq );

/**
    @brief   This sub-procedure is used to write a characteristic value
            to a server when the client knows the characteristic value
            handle. This sub-procedure only writes the first (ATT_MTU-3)
            octets of a characteristic value. This sub-procedure can not
            be used to write a long attribute; instead the Write Long
            Characteristic Values sub-procedure should be used.

            The ATT Write Request is used in this sub-procedure. The
            Attribute Handle parameter shall be set to the Characteristic
            Value Handle. The Attribute Value parameter shall be set to
            the new characteristic.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT message.
            The type of the message will be either ATT_WRITE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_WRITE_RSP
                  (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
                  SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_WriteCharValue( uint16 connHandle, attWriteReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to write a Characteristic Value to
            a server when the client knows the Characteristic Value Handle
            but the length of the Characteristic Value is longer than can
            be sent in a single Write Request Attribute Protocol message.

            The ATT Prepare Write Request and Execute Write Request are
            used to perform this sub-procedure.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_PREPARE_WRITE_RSP,
            ATT_EXECUTE_WRITE_RSP or ATT_ERROR_RSP (if an error occurred on
            the server).

            Note: This sub-procedure is complete when either ATT_PREPARE_WRITE_RSP
                  (with bleTimeout status), ATT_EXECUTE_WRITE_RSP (with SUCCESS
                  or bleTimeout status), or ATT_ERROR_RSP (with SUCCESS status)
                  is received by the calling application task.

            Note: The 'pReq->pValue' pointer will be freed when the sub-procedure
                  is complete.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_WriteLongCharValue( uint16 connHandle, gattPrepareWriteReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to write a Characteristic Value to
            a server when the client knows the Characteristic Value Handle,
            and assurance is required that the correct Characteristic Value
            is going to be written by transferring the Characteristic Value
            to be written in both directions before the write is performed.
            This sub-procedure can also be used when multiple values must
            be written, in order, in a single operation.

            The sub-procedure has two phases, the first phase prepares the
            characteristic values to be written.  Once this is complete,
            the second phase performs the execution of all of the prepared
            characteristic value writes on the server from this client.

            In the first phase, the ATT Prepare Write Request is used.
            In the second phase, the attribute protocol Execute Write
            Request is used.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_PREPARE_WRITE_RSP,
            ATT_EXECUTE_WRITE_RSP or ATT_ERROR_RSP (if an error occurred on
            the server).

            Note: This sub-procedure is complete when either ATT_PREPARE_WRITE_RSP
                  (with bleTimeout status), ATT_EXECUTE_WRITE_RSP (with SUCCESS
                  or bleTimeout status), or ATT_ERROR_RSP (with SUCCESS status)
                  is received by the calling application task.

            Note: The 'pReqs' pointer will be freed when the sub-procedure is
                  complete.

    @param   connHandle - connection to use
    @param   pReqs - pointer to requests to be sent (must be allocated)
    @param   numReqs - number of requests in pReq
    @param   flags - execute write request flags
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ReliableWrites( uint16 connHandle, attPrepareWriteReq_t* pReqs,
                                      uint8 numReqs, uint8 flags, uint8 taskId );
/**
    @brief   This sub-procedure is used to read a characteristic descriptor
            from a server when the client knows the characteristic descriptor
            declaration’s Attribute handle.

            The ATT Read Request is used for this sub-procedure. The Read
            Request is used with the Attribute Handle parameter set to the
            characteristic descriptor handle. The Read Response returns the
            characteristic descriptor value in the Attribute Value parameter.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT message.
            The type of the message will be either ATT_READ_RSP or
            ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_RSP
                  (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
                  SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ReadCharDesc( uint16 connHandle, attReadReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to read a characteristic descriptor
            from a server when the client knows the characteristic descriptor
            declaration’s Attribute handle and the length of the characteristic
            descriptor declaration is longer than can be sent in a single Read
            Response attribute protocol message.

            The ATT Read Blob Request is used to perform this sub-procedure.
            The Attribute Handle parameter shall be set to the characteristic
            descriptor handle. The Value Offset parameter shall be the offset
            within the characteristic descriptor to be read.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_READ_BLOB_RSP or
            ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_READ_BLOB_RSP
                  (with bleProcedureComplete or bleTimeout status) or ATT_ERROR_RSP
                  (with SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_ReadLongCharDesc( uint16 connHandle, attReadBlobReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to write a characteristic
            descriptor value to a server when the client knows the
            characteristic descriptor handle.

            The ATT Write Request is used for this sub-procedure. The
            Attribute Handle parameter shall be set to the characteristic
            descriptor handle. The Attribute Value parameter shall be
            set to the new characteristic descriptor value.

            If the return status from this function is SUCCESS, the calling
            application task will receive an OSAL GATT_MSG_EVENT message.
            The type of the message will be either ATT_WRITE_RSP
            or ATT_ERROR_RSP (if an error occurred on the server).

            Note: This sub-procedure is complete when either ATT_WRITE_RSP
                  (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
                  SUCCESS status) is received by the calling application task.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.<BR>
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_WriteCharDesc( uint16 connHandle, attWriteReq_t* pReq, uint8 taskId );

/**
    @brief   This sub-procedure is used to write a Characteristic Value to
            a server when the client knows the Characteristic Value Handle
            but the length of the Characteristic Value is longer than can
            be sent in a single Write Request Attribute Protocol message.

            The ATT Prepare Write Request and Execute Write Request are
            used to perform this sub-procedure.

            If the return status from this function is SUCCESS, the calling
            application task will receive multiple OSAL GATT_MSG_EVENT messages.
            The type of the messages will be either ATT_PREPARE_WRITE_RSP,
            ATT_EXECUTE_WRITE_RSP or ATT_ERROR_RSP (if an error occurred on
            the server).

            Note: This sub-procedure is complete when either ATT_PREPARE_WRITE_RSP
                  (with bleTimeout status), ATT_EXECUTE_WRITE_RSP (with SUCCESS
                  or bleTimeout status), or ATT_ERROR_RSP (with SUCCESS status)
                  is received by the calling application task.

            Note: The 'pReq->pValue' pointer will be freed when the sub-procedure
                  is complete.

    @param   connHandle - connection to use
    @param   pReq - pointer to request to be sent
    @param   taskId - task to be notified of response

    @return  SUCCESS: Request was sent successfully.<BR>
            INVALIDPARAMETER: Invalid connection handle or request field.v
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
            bleNotConnected: Connection is down.<BR>
            blePending: A response is pending with this server.<BR>
            bleMemAllocError: Memory allocation error occurred.<BR>
            bleTimeout: Previous transaction timed out.<BR>
*/
extern bStatus_t GATT_WriteLongCharDesc( uint16 connHandle, gattPrepareWriteReq_t* pReq, uint8 taskId );

/**
    @}
*/

/*  -------------------------------------------------------------------
    GATT Flow Control APIs
*/

/**
    @defgroup GATT_FLOW_CTRL_API GATT Flow Control API Functions

    @{
*/

/**
    @brief   This API is used by the Application to turn flow control on
            or off for GATT messages sent from the Host to the Application.

            Note: If the flow control is enabled then the Application must
                  call the GATT_AppCompletedMsg() API when it completes
                  processing an incoming GATT message.

    @param   flowCtrlMode ? flow control mode: TRUE or FALSE

    @return  void
*/
extern void GATT_SetHostToAppFlowCtrl( uint16 hostBufSize,uint8 flowCtrlMode );

/**
    @brief   This API is used by the Application to notify GATT that
            the processing of a message has been completed.

    @param   pMsg ? pointer to the processed GATT message

    @return  void
*/
extern void GATT_AppCompletedMsg( gattMsgEvent_t* pMsg );

/**
    @}
*/

/*  -------------------------------------------------------------------
    Internal API - This function is only called from GATT Qualification modules.
*/

/**
    @internal

    @brief       Set the next available attribute handle.

    @param       handle - next attribute handle.

    @return      none
*/
extern void GATT_SetNextHandle( uint16 handle );

/*  -------------------------------------------------------------------
    TASK API - These functions must only be called by OSAL.
*/

/**
    @internal

    @brief   GATT Task initialization function.

    @param   taskId - GATT task ID.

    @return  void
*/
extern void GATT_Init( uint8 taskId );

/**
    @internal

    @brief   GATT Task event processing function.

    @param   taskId - GATT task ID
    @param   events - GATT events.

    @return  events not processed
*/
extern uint16 GATT_ProcessEvent( uint8 taskId, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATT_H */
