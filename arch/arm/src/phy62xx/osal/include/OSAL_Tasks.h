/**************************************************************************************************
    Filename:       OSAL_Tasks.h
    Revised:
    Revision:

    Description:    This file contains the OSAL Task definition and manipulation functions.


**************************************************************************************************/

#ifndef OSAL_TASKS_H
#define OSAL_TASKS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/
#define TASK_NO_TASK      0xFF

/*********************************************************************
    TYPEDEFS
*/

/*
    Event handler function prototype
*/
typedef unsigned short (*pTaskEventHandlerFn)( unsigned char task_id, unsigned short event );

/*********************************************************************
    GLOBAL VARIABLES
*/

extern  const pTaskEventHandlerFn tasksArr[];
extern  const uint8 tasksCnt;
extern uint16* tasksEvents;

/*********************************************************************
    FUNCTIONS
*/

/*
    Call each of the tasks initailization functions.
*/
extern void osalInitTasks( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_TASKS_H */
