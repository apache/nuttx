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

/*******************************************************************************
    Filename:       hci_c_event.h
    Revised:        $Date: 2012-05-01 12:13:50 -0700 (Tue, 01 May 2012) $
    Revision:       $Revision: 30418 $

    Description:    This file contains the HCI Event types, contants,
                  external functions etc. for the BLE Controller.


*******************************************************************************/

#ifndef HCI_C_EVENT_H
#define HCI_C_EVENT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
    INCLUDES
*/
#include "hci_tl.h"

extern uint32 bleEvtMask;
extern uint8 pHciEvtMask[];

/*******************************************************************************
    MACROS
*/

/*******************************************************************************
    CONSTANTS
*/

// Event Mask Default Values
#define BT_EVT_MASK_BYTE0                  0xFF
#define BT_EVT_MASK_BYTE1                  0xFF
#define BT_EVT_MASK_BYTE2                  0xFF
#define BT_EVT_MASK_BYTE3                  0xFF
#define BT_EVT_MASK_BYTE4                  0xFF
#define BT_EVT_MASK_BYTE5                  0x9F
#define BT_EVT_MASK_BYTE6                  0x00
#define BT_EVT_MASK_BYTE7                  0x20
//
#define LE_EVT_MASK_DEFAULT                0x00005F

/*******************************************************************************
    TYPEDEFS
*/

/*******************************************************************************
    LOCAL VARIABLES
*/

/*******************************************************************************
    GLOBAL VARIABLES
*/

/*
** Internal Functions
*/

extern void hciInitEventMasks( void );

/*
** HCI Controller Events
*/

/*******************************************************************************
    @fn          HCI_DataBufferOverflowEvent

    @brief       This function sends the Data Buffer Overflow Event to the Host.

    input parameters

    @param       linkType - HCI_LINK_TYPE_SCO_BUFFER_OVERFLOW,
                           HCI_LINK_TYPE_ACL_BUFFER_OVERFLOW

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_DataBufferOverflowEvent( uint8 linkType );


/*******************************************************************************
    @fn          HCI_NumOfCompletedPacketsEvent

    @brief       This function sends the Number of Completed Packets Event to
                the Host.

                Note: Currently, the number of handles is always one.

    input parameters

    @param       numHandles       - Number of handles.
    @param       handlers         - Array of connection handles.
    @param       numCompletedPkts - Array of number of completed packets for
                                   each handle.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_NumOfCompletedPacketsEvent( uint8 numHandles,
                                            uint16* handlers,
                                            uint16* numCompletedPackets );


/*******************************************************************************
    @fn          HCI_CommandCompleteEvent

    @brief       This function sends a Command Complete Event to the Host.

    input parameters

    @param       opcode   - The opcode of the command that generated this event.
    @param       numParam - The number of parameters in the event.
    @param       param    - The event parameters associated with the command.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_CommandCompleteEvent( uint16 opcode,
                                      uint8  numParam,
                                      uint8*  param );


/*******************************************************************************
    @fn          HCI_VendorSpecifcCommandCompleteEvent

    @brief       This function sends a Vendor Specific Command Complete Event to
                the Host.

    input parameters

    @param       opcode   - The opcode of the command that generated this event.
    @param       numParam - The number of parameters in the event.
    @param       param    - The event parameters associated with the command.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_VendorSpecifcCommandCompleteEvent( uint16 opcode,
                                                   uint8 len,
                                                   uint8* param );


/*******************************************************************************
    @fn          HCI_CommandStatusEvent

    @brief       This function sends a Command Status Event to the Host.

    input parameters

    @param       status - The resulting status of the comamnd.
    @param       opcode - The opcode of the command that generated this event.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_CommandStatusEvent( uint8 status,
                                    uint16 opcode );


/*******************************************************************************
    @fn          HCI_HardwareErrorEvent

    @brief       This function sends a Hardware Error Event to the Host.

    input parameters

    @param       hwErrorCode - The hardware error code.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_HardwareErrorEvent( uint8 hwErrorCode );


/*******************************************************************************
    @fn          HCI_SendCommandStatusEvent

    @brief       This generic function sends a Command Status event to the Host.
                It is provided as a direct call so the Host can use it directly.

    input parameters

    @param       eventCode - The event code.
    @param       status    - The resulting status of the comamnd.
    @param       opcode    - The opcode of the command that generated this event.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_SendCommandStatusEvent ( uint8  eventCode,
                                         uint16 status,
                                         uint16 opcode );


/*******************************************************************************
    @fn          HCI_SendCommandCompleteEvent

    @brief       This generic function sends a Command Complete or a Vendor
                Specific Command Complete Event to the Host.

    input parameters

    @param       eventCode - The event code.
    @param       opcode    - The opcode of the command that generated this event.
    @param       numParam  - The number of parameters in the event.
    @param       param     - The event parameters associated with the command.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_SendCommandCompleteEvent ( uint8  eventCode,
                                           uint16 opcode,
                                           uint8  numParam,
                                           uint8*  param );


/*******************************************************************************
    @fn          HCI_SendControllerToHostEvent

    @brief       This generic function sends a Controller to Host Event.

    input parameters

    @param       eventCode - Bluetooth event code.
    @param       dataLen   - Length of dataField.
    @param       pData     - Pointer to data.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_SendControllerToHostEvent( uint8 eventCode,
                                           uint8 dataLen,
                                           uint8* pData );

#ifdef __cplusplus
}
#endif

#endif /* HCI_C_EVENT_H */
