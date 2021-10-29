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

#ifndef GATTTEST_H
#define GATTTEST_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"

/*********************************************************************
    CONSTANTS
*/
// Length of attribute
#define GATT_TEST_ATTR_LEN               20

// Length of long attribute
#define GATT_TEST_LONG_ATTR_LEN          50

// GATT Test Services bit fields
#define GATT_TEST_SERVICE                0x00000001 // GATT Test
#define GATT_BATT_STATE_SERVICE          0x00000002 // Battery State
#define GATT_THERMO_HUMID_SERVICE        0x00000004 // Thermometer Humidity
#define GATT_WEIGHT_SERVICE              0x00000008 // Weight
#define GATT_POSITION_SERVICE            0x00000010 // Position
#define GATT_ALERT_SERVICE               0x00000020 // Alert
#define GATT_MANUFACT_SENSOR_SERVICE     0x00000040 // Sensor Manufacturer
#define GATT_MANUFACT_SCALES_SERVICE     0x00000080 // Scales Manufacturer
#define GATT_ADDRESS_SERVICE             0x00000100 // Address
#define GATT_128BIT_UUID1_SERVICE        0x00000200 // 128-bit UUID 1
#define GATT_128BIT_UUID2_SERVICE        0x00000400 // 128-bit UUID 2
#define GATT_128BIT_UUID3_SERVICE        0x00000800 // 128-bit UUID 3

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    MACROS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/**
    @brief   Add function for the GATT Test Services.

    @param   services - services to add. This is a bit map and can
                       contain more than one service.

    @return  SUCCESS: Service added successfully.
            INVALIDPARAMETER: Invalid service field.
            FAILURE: Not enough attribute handles available.
            bleMemAllocError: Memory allocation error occurred.
*/
extern bStatus_t GATTTest_AddService( uint32 services );

/**
    @brief   Delete function for the GATT Test Services.

    @param   services - services to delete. This is a bit map and can
                       contain more than one service.

    @return  SUCCESS: Service deleted successfully.
            FAILURE: Service not found.
*/
extern bStatus_t GATTTest_DelService( uint32 services );

/*  -------------------------------------------------------------------
    TASK API - These functions must only be called by OSAL.
*/

/**
    @internal

    @brief   Initialize the GATT Test Application.

    @param   taskId - Task identifier for the desired task

    @return  void

*/
extern void GATTTest_Init( uint8 taskId );

/**
    @internal

    @brief   GATT Test Application Task event processor. This function
            is called to process all events for the task. Events include
            timers, messages and any other user defined events.

    @param   task_id - The OSAL assigned task ID.
    @param   events - events to process. This is a bit map and can
                     contain more than one event.

    @return  none
*/
extern uint16 GATTTest_ProcessEvent( uint8 task_id, uint16 events );

/**
    @brief   Add function for the GATT Qualification Services.

    @param   services - services to add. This is a bit map and can
                       contain more than one service.

    @return  SUCCESS: Service added successfully.
            INVALIDPARAMETER: Invalid service field.
            FAILURE: Not enough attribute handles available.
            bleMemAllocError: Memory allocation error occurred.
*/
extern bStatus_t GATTQual_AddService( uint32 services );

/**
    @brief   Delete function for the GATT Qualification Services.

    @param   services - services to delete. This is a bit map and can
                       contain more than one service.

    @return  SUCCESS: Service deleted successfully.
            FAILURE: Service not found.
*/
extern bStatus_t GATTQual_DelService( uint32 services );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATTTEST_H */
