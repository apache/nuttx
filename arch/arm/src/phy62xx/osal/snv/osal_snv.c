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


#include <stdint.h>
#include "osal.h"
#include "flash.h"
#include "error.h"
#include "osal_snv.h"
#include "log.h"

#ifndef USE_FS
    #define USE_FS 1
#endif
#ifdef USE_FS
    #include "fs.h"
#endif


#if (USE_FS == 0)
#define NVM_BASE_ADDR           0x1103C000  //16K bytes


uint8 osal_snv_init( void )
{
    return PPlus_ERR_FATAL;
}

uint8 osal_snv_read( osalSnvId_t id, osalSnvLen_t len, void* pBuf)
{
    (void)(id);
    (void)(len);
    (void)(pBuf);
    return PPlus_ERR_FATAL;
}

uint8 osal_snv_write( osalSnvId_t id, osalSnvLen_t len, void* pBuf)
{
    (void)(id);
    (void)(len);
    (void)(pBuf);
    return PPlus_ERR_FATAL;
}

uint8 osal_snv_compact( uint8 threshold )
{
    return SUCCESS;
}

#else

uint8 osal_snv_init( void )
{
    if(!hal_fs_initialized())
        return NV_OPER_FAILED;

    return SUCCESS;
}

uint8 osal_snv_read( osalSnvId_t id, osalSnvLen_t len, void* pBuf)
{
    int ret;
    LOG("osal_snv_read:%x\n",id);
    ret = hal_fs_item_read((uint16_t)id,(uint8_t*) pBuf, (uint16_t)len,NULL);

    if(ret != PPlus_SUCCESS)
    {
        LOG("rd_ret:%d\n",ret);
        return NV_OPER_FAILED;
    }

    LOG_DUMP_BYTE(pBuf, len);
    return SUCCESS;
}

uint8 osal_snv_write( osalSnvId_t id, osalSnvLen_t len, void* pBuf)
{
    int ret = PPlus_SUCCESS;
    LOG("osal_snv_write:%x,%d\n",id,len);
    LOG_DUMP_BYTE(pBuf, len);

    if(hal_fs_get_free_size() < len+32)
    {
        if(hal_fs_get_garbage_size(NULL) > len+32)
        {
            hal_fs_garbage_collect();
        }
        else
        {
            return NV_OPER_FAILED;
        }
    }

    ret = hal_fs_item_write((uint16_t) id, (uint8_t*) pBuf, (uint16_t) len);

    if(ret !=0)
    {
        LOG("wr_ret:%d\n",ret);
        return NV_OPER_FAILED;
    }

    //LOG("Success\n");
    return SUCCESS;
}

uint8 osal_snv_compact( uint8 threshold )
{
    return 0;
}

#endif

