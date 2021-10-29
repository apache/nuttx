/******************************************************************************
    Filename:     OSAL.h
    Revised:
    Revision:
******************************************************************************/

#ifndef OSAL_H
#define OSAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

#include "comdef.h"
#include "OSAL_Memory.h"
#include "OSAL_Timers.h"

/*********************************************************************
    MACROS
*/
#if ( UINT_MAX == 65535 ) /* 8-bit and 16-bit devices */
#define osal_offsetof(type, member) ((uint16) &(((type *) 0)->member))
#else /* 32-bit devices */
#define osal_offsetof(type, member) ((uint32) &(((type *) 0)->member))
#endif

#define OSAL_MSG_NEXT(msg_ptr)      ((osal_msg_hdr_t *) (msg_ptr) - 1)->next

#define OSAL_MSG_Q_INIT(q_ptr)      *(q_ptr) = NULL

#define OSAL_MSG_Q_EMPTY(q_ptr)     (*(q_ptr) == NULL)

#define OSAL_MSG_Q_HEAD(q_ptr)      (*(q_ptr))

#define OSAL_MSG_LEN(msg_ptr)      ((osal_msg_hdr_t *) (msg_ptr) - 1)->len

#define OSAL_MSG_ID(msg_ptr)      ((osal_msg_hdr_t *) (msg_ptr) - 1)->dest_id

/*********************************************************************
    CONSTANTS
*/

/*** Interrupts ***/
#define INTS_ALL    0xFF

/*********************************************************************
    TYPEDEFS
*/
typedef struct
{
    void*   next;
    uint16 len;
    uint8  dest_id;
} osal_msg_hdr_t;

typedef struct
{
    uint8  event;
    uint8  status;
} osal_event_hdr_t;

typedef void* osal_msg_q_t;

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/*** Message Management ***/

/*
    Task Message Allocation
*/
extern uint8* osal_msg_allocate(uint16 len );

/*
    Task Message Deallocation
*/
extern uint8 osal_msg_deallocate( uint8* msg_ptr );

/*
    Send a Task Message
*/
extern uint8 osal_msg_send( uint8 destination_task, uint8* msg_ptr );

/*
    Push a Task Message to head of queue
*/
extern uint8 osal_msg_push_front( uint8 destination_task, uint8* msg_ptr );

/*
    Receive a Task Message
*/
extern uint8* osal_msg_receive( uint8 task_id );

/*
    Find in place a matching Task Message / Event.
*/
extern osal_event_hdr_t* osal_msg_find(uint8 task_id, uint8 event);

/*
    Enqueue a Task Message
*/
extern void osal_msg_enqueue( osal_msg_q_t* q_ptr, void* msg_ptr );

/*
    Enqueue a Task Message Up to Max
*/
extern uint8 osal_msg_enqueue_max( osal_msg_q_t* q_ptr, void* msg_ptr, uint8 max );

/*
    Dequeue a Task Message
*/
extern void* osal_msg_dequeue( osal_msg_q_t* q_ptr );

/*
    Push a Task Message to head of queue
*/
extern void osal_msg_push( osal_msg_q_t* q_ptr, void* msg_ptr );

/*
    Extract and remove a Task Message from queue
*/
extern void osal_msg_extract( osal_msg_q_t* q_ptr, void* msg_ptr, void* prev_ptr );


/*** Task Synchronization  ***/

/*
    Set a Task Event
*/
extern uint8 osal_set_event( uint8 task_id, uint16 event_flag );


/*
    Clear a Task Event
*/
extern uint8 osal_clear_event( uint8 task_id, uint16 event_flag );


/*** Interrupt Management  ***/

/*
    Register Interrupt Service Routine (ISR)
*/
extern uint8 osal_isr_register( uint8 interrupt_id, void (*isr_ptr)( uint8* ) );

/*
    Enable Interrupt
*/
extern uint8 osal_int_enable( uint8 interrupt_id );

/*
    Disable Interrupt
*/
extern uint8 osal_int_disable( uint8 interrupt_id );


/*** Task Management  ***/

/*
    Initialize the Task System
*/
extern uint8 osal_init_system( void );

/*
    System Processing Loop
*/
#if defined (ZBIT)
extern __declspec(dllexport)  void osal_start_system( void );
#else
extern void osal_start_system( void );
#endif

/*
    One Pass Throu the OSAL Processing Loop
*/
extern void osal_run_system( void );

/*
    Get the active task ID
*/
extern uint8 osal_self( void );


/*** Helper Functions ***/

/*
    String Length
*/
extern int osal_strlen( char* pString );

/*
    Memory copy
*/
extern void* osal_memcpy( void*, const void GENERIC*, unsigned int );

/*
    Memory Duplicate - allocates and copies
*/
extern void* osal_memdup( const void GENERIC* src, unsigned int len );

/*
    Reverse Memory copy
*/
extern void* osal_revmemcpy( void*, const void GENERIC*, unsigned int );

/*
    Memory compare
*/
extern uint8 osal_memcmp( const void GENERIC* src1, const void GENERIC* src2, unsigned int len );

/*
    Memory set
*/
extern void* osal_memset( void* dest, uint8 value, int len );

/*
    Build a uint16 out of 2 bytes (0 then 1).
*/
extern uint16 osal_build_uint16( uint8* swapped );

/*
    Build a uint32 out of sequential bytes.
*/
extern uint32 osal_build_uint32( uint8* swapped, uint8 len );

/*
    Convert long to ascii string
*/
#if !defined ( ZBIT ) && !defined ( ZBIT2 ) && !defined (UBIT)
extern uint8* _ltoa( uint32 l, uint8* buf, uint8 radix );
#endif

/*
    Random number generator
*/
extern uint16 osal_rand( void );

/*
    Buffer an uint32 value - LSB first.
*/
extern uint8* osal_buffer_uint32( uint8* buf, uint32 val );

/*
    Buffer an uint24 value - LSB first
*/
extern uint8* osal_buffer_uint24( uint8* buf, uint24 val );

/*
    Is all of the array elements set to a value?
*/
extern uint8 osal_isbufset( uint8* buf, uint8 val, uint8 len );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_H */
