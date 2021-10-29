/**************************************************************************************************
    Filename:       osal_cbtimer.h
    Revised:
    Revision:

    Description:    This file contains the Callback Timer definitions.


 **************************************************************************************************/

#ifndef OSAL_CBTIMER_H
#define OSAL_CBTIMER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/
// Invalid timer id
#define INVALID_TIMER_ID                           0xFF

// Timed out timer
#define TIMEOUT_TIMER_ID                           0xFE

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    MACROS
*/
#define  OSAL_CBTIMER_NUM_TASKS 1                 // set by HZF, align to TI project setting
#if ( OSAL_CBTIMER_NUM_TASKS == 0 )
#error "Callback Timer module shouldn't be included (no callback timer is needed)!"
#elif ( OSAL_CBTIMER_NUM_TASKS == 1 )
#define OSAL_CBTIMER_PROCESS_EVENT( a )          ( a )
#elif ( OSAL_CBTIMER_NUM_TASKS == 2 )
#define OSAL_CBTIMER_PROCESS_EVENT( a )          ( a ), ( a )
#else
#error "Maximum of 2 callback timer tasks are supported! Modify it here."
#endif

/*********************************************************************
    TYPEDEFS
*/

// Callback Timer function prototype. Callback function will be called
// when the associated timer expires.
//
// pData - pointer to data registered with timer
//
typedef void (*pfnCbTimer_t)( uint8* pData );

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/*
    Callback Timer task initialization function.
*/
extern void osal_CbTimerInit( uint8 taskId );

/*
    Callback Timer task event processing function.
*/
extern uint16 osal_CbTimerProcessEvent( uint8 taskId, uint16 events );

/*
    Function to start a timer to expire in n mSecs.
*/
extern Status_t osal_CbTimerStart( pfnCbTimer_t pfnCbTimer, uint8* pData,
                                   uint32 timeout, uint8* pTimerId );

/*
    Function to update a timer that has already been started.
*/
extern Status_t osal_CbTimerUpdate( uint8 timerId, uint32 timeout );

/*
    Function to stop a timer that has already been started.
*/
extern Status_t osal_CbTimerStop( uint8 timerId );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_CBTIMER_H */
