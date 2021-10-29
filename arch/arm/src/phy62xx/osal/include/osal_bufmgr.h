/**************************************************************************************************
    Filename:       osal_bufmgr.h
    Revised:
    Revision:

    Description:    This file contains the buffer management definitions.



**************************************************************************************************/

#ifndef OSAL_BUFMGR_H
#define OSAL_BUFMGR_H

#include "comdef.h"
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

/*
    Allocate a block of memory.
*/
extern void* osal_bm_alloc( uint16 size );

/*
    Add or remove header space for the payload pointer.
*/
extern void* osal_bm_adjust_header( void* payload_ptr, int16 size );

/*
    Add or remove tail space for the payload pointer.
*/
extern void* osal_bm_adjust_tail( void* payload_ptr, int16 size );

/*
    Free a block of memory.
*/
extern void osal_bm_free( void* payload_ptr );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_BUFMGR_H */
