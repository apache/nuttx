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
    Filename:       hci_c_data.h
    Revised:        $Date: 2011-08-22 08:41:40 -0700 (Mon, 22 Aug 2011) $
    Revision:       $Revision: 27235 $

    Description:    This file handles HCI data for the BLE Controller.


*******************************************************************************/

#ifndef HCI_C_DATA_H
#define HCI_C_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
    INCLUDES
*/

/*******************************************************************************
    MACROS
*/

/*******************************************************************************
    CONSTANTS
*/

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
** HCI Data API
*/


/*******************************************************************************
    @fn          HCI_ReverseBytes

    @brief       This function is used to reverse the order of the bytes in
                an array in place.

    input parameters

    @param       *buf - Pointer to buffer containing bytes to be reversed.
    @param       len  - Number of bytes in buffer.

                Note: The length must be even.

                Note: The maximum length is 128 bytes.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_ReverseBytes( uint8* buf,
                              uint8 len );


#ifdef __cplusplus
}
#endif

#endif /* HCI_C_DATA_H */
