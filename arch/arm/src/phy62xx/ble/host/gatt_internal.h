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

#ifndef GATT_INTERNAL_H
#define GATT_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "osal_cbTimer.h"

#include "att.h"
#include "gatt.h"

/*********************************************************************
    MACROS
*/
#define TIMER_VALID( id )                ( ( (id) != INVALID_TIMER_ID ) && \
                                           ( (id) != TIMEOUT_TIMER_ID ) )

#define TIMER_STATUS( id )               ( (id) == TIMEOUT_TIMER_ID ? bleTimeout : \
                                           (id) == INVALID_TIMER_ID ? SUCCESS : blePending )

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/
// Srtucture for Attribute Version Information attribute
typedef struct
{
    uint8 attVersion;        // Attribute Protocol Version
    uint8 gattVersion;       // Generic Attribute Profile Version
    uint16 manufacturerName; // Manufacturer Name
} gattVersionInfo_t;

// Function prototype to parse an attribute protocol request message
typedef bStatus_t (*gattParseReq_t)( uint8 sig, uint8 cmd, uint8* pParams, uint16 len, attMsg_t* pMsg );

// Function prototype to parse an attribute protocol response message
typedef bStatus_t (*gattParseRsp_t)( uint8* pParams, uint16 len, attMsg_t* pMsg );

// Function prototype to process an attribute protocol message
typedef bStatus_t (*gattProcessMsg_t)( uint16 connHandle,  attPacket_t* pPkt );

// Function prototype to process an attribute protocol request message
typedef bStatus_t (*gattProcessReq_t)( uint16 connHandle,  attMsg_t* pMsg );

/*********************************************************************
    VARIABLES
*/
extern uint8 gattTaskID;

/*********************************************************************
    FUNCTIONS
*/
extern void gattRegisterServer( gattProcessMsg_t pfnProcessMsg );

extern void gattRegisterClient( gattProcessMsg_t pfnProcessMsg );

extern bStatus_t gattNotifyEvent( uint8 taskId, uint16 connHandle, uint8 status,
                                  uint8 method, gattMsg_t* pMsg );

extern void gattStartTimer( pfnCbTimer_t pfnCbTimer, uint8* pData,
                            uint16 timeout, uint8* pTimerId );

extern void gattStopTimer( uint8* pTimerId );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATT_INTERNAL_H */
