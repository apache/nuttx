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

/*************************************************************************************************
**************************************************************************************************/
#ifndef HCI_TASK_H
#define HCI_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "OSAL.h"
#include "hci.h"
#include "uart.h"
#include "hci_host.h"

#include "hal.h"     // added by ZJP


/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/


/* UART port */
#define HCI_UART_PORT                  HAL_UART_PORT_0
#define HCI_UART_BR                    HAL_UART_BR_38400
#define HCI_UART_FC                    TRUE
#define HCI_UART_FC_THRESHOLD          48
#define HCI_UART_RX_BUF_SIZE           128
#define HCI_UART_TX_BUF_SIZE           128
#define HCI_UART_IDLE_TIMEOUT          6
#define HCI_UART_INT_ENABLE            TRUE

/* HCI Event List */
#define HCI_EVENT_SEND_DATA            0x01
#define HCI_EVENT_SEND_CMD             0x02
#define HCI_HOST_PARSE_EVT             0x04
#define HCI_HOST_INCOMING_EVT          0x08
#define HCI_HOST_INCOMING_DATA         0x10


/* Define the osal queue size for data and cmd */
#define HCI_HOST_MAX_DATAQUEUE_SIZE    20
#define HCI_HOST_MAX_CMDQUEUE_SIZE     20

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
osal_msg_q_t HCI_HostDataQueue;

uint8 hciHostNumQueuedData;           /* Number of data packets queued */
const uint8 hciHostMaxNumDataQueue;   /* Max number of data packets queued */

/*********************************************************************
    FUNCTIONS - API
*/
extern Status_t HCI_AddDataQueue( void* buf );
extern Status_t HCI_AddCmdQueue( void* buf );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HCI_TASK_H */






