/**************************************************************************************************
    Filename:       OSAL_Memory.h
    Revised:
    Revision:

    Description:    This module defines the OSAL memory control functions.



**************************************************************************************************/

#ifndef OSAL_MEMORY_H
#define OSAL_MEMORY_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "comdef.h"

/*********************************************************************
    CONSTANTS
*/

#if !defined ( OSALMEM_METRICS )
#define OSALMEM_METRICS  FALSE
#endif

/*********************************************************************
    MACROS
*/

//#define osal_stack_used()  OnBoard_stack_used()

/*********************************************************************
    TYPEDEFS
*/

typedef struct
{
    // The 15 LSB's of 'val' indicate the total item size, including the header, in 8-bit bytes.
    unsigned short len : 15;   // unsigned short len : 15;
    // The 1 MSB of 'val' is used as a boolean to indicate in-use or freed.
    unsigned short inUse : 1;  // unsigned short inUse : 1;
} osalMemHdrHdr_t;

typedef union
{
    /*  Dummy variable so compiler forces structure to alignment of largest element while not wasting
        space on targets when the halDataAlign_t is smaller than a UINT16.
    */
    halDataAlign_t alignDummy;
    uint32 val;            // uint16    // TODO: maybe due to 4 byte alignment requirement in M0, this union should be 4 byte, change from uint16 to uint32, investigate more later -  04-25
    osalMemHdrHdr_t hdr;
} osalMemHdr_t;

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/*
    Initialize memory manager.
*/
void osal_mem_init( void );

/*
    Setup efficient search for the first free block of heap.
*/
void osal_mem_kick( void );

/*
    Allocate a block of memory.
*/
void* osal_mem_alloc( uint16 size );

/*
    Free a block of memory.
*/
void osal_mem_free( void* ptr );


//  ====== A2 metal change add
/*
    Set osal memory buffer
*/
void osal_mem_set_heap(osalMemHdr_t* hdr, uint32 size);

#if ( OSALMEM_METRICS )
/*
    Return the maximum number of blocks ever allocated at once.
*/
uint16 osal_heap_block_max( void );

/*
    Return the current number of blocks now allocated.
*/
uint16 osal_heap_block_cnt( void );

/*
    Return the current number of free blocks.
*/
uint16 osal_heap_block_free( void );

/*
    Return the current number of bytes allocated.
*/
uint16 osal_heap_mem_used( void );
#endif


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* #ifndef OSAL_MEMORY_H */
