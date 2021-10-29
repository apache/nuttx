/**************************************************************************************************
    Filename:       OSAL_snv.h
    Revised:
    Revision:

    Description:    This module defines the OSAL snv control functions.



**************************************************************************************************/
#ifndef OSAL_SNV_H
#define OSAL_SNV_H

#include "comdef.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

//#include "hal_types.h"

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    MACROS
*/

/*********************************************************************
    TYPEDEFS
*/
#ifdef OSAL_SNV_UINT16_ID
typedef uint16 osalSnvId_t;
typedef uint16 osalSnvLen_t;
#else
typedef uint8 osalSnvId_t;
typedef uint8 osalSnvLen_t;
#endif

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/*********************************************************************
    @fn      osal_snv_init

    @brief   Initialize NV service.

    @return  SUCCESS if initialization succeeds. FAILURE, otherwise.
*/
extern uint8 osal_snv_init( void );

/*********************************************************************
    @fn      osal_snv_read

    @brief   Read data from NV.

    @param   id  - Valid NV item Id.
    @param   len - Length of data to read.
    @param   *pBuf - Data is read into this buffer.

    @return  SUCCESS if successful.
            Otherwise, NV_OPER_FAILED for failure.
*/
extern uint8 osal_snv_read( osalSnvId_t id, osalSnvLen_t len, void* pBuf);

/*********************************************************************
    @fn      osal_snv_write

    @brief   Write a data item to NV.

    @param   id  - Valid NV item Id.
    @param   len - Length of data to write.
    @param   *pBuf - Data to write.

    @return  SUCCESS if successful, NV_OPER_FAILED if failed.
*/
extern uint8 osal_snv_write( osalSnvId_t id, osalSnvLen_t len, void* pBuf);

/*********************************************************************
    @fn      osal_snv_compact

    @brief   Compacts NV if its usage has reached a specific threshold.

    @param   threshold - compaction threshold

    @return  SUCCESS if successful,
            NV_OPER_FAILED if failed, or
            INVALIDPARAMETER if threshold invalid.
*/
extern uint8 osal_snv_compact( uint8 threshold );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_SNV.H */
