/****************************************************************************
 * arch/arm/src/phy62xx/types.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>
#include <stdbool.h>

#include <sys/param.h>

typedef signed   char   int8;     /* !< Signed 8 bit integer */
typedef unsigned char   uint8;    /* !< Unsigned 8 bit integer */

typedef signed   short  int16;    /* !< Signed 16 bit integer */
typedef unsigned short  uint16;   /* !< Unsigned 16 bit integer */

typedef signed   long   int32;    /* !< Signed 32 bit integer */
typedef unsigned long   uint32;   /* !< Unsigned 32 bit integer */

typedef uint8           halDataAlign_t; /* !< Used for byte alignment */

#define ALIGN4_U8       __align(4) uint8
#define ALIGN4_U16      __align(4) uint16
#define ALIGN4_INT8     __align(4) int8
#define ALIGN4_INT16    __align(4) int16

#define    BIT(n)      (1ul << (n))

#define write_reg(addr,data)      (*(volatile unsigned int  *)(addr) = (unsigned int)(data))
#define read_reg(addr)            (*(volatile unsigned int  *)(addr))

/* bit operations */
#define BM_SET(addr,bit)                ( *(addr) |= (bit) )     /* bit set */
#define BM_CLR(addr,bit)                ( *(addr) &= ~(bit) )    /* bit clear */
#define BM_IS_SET(addr,bit)             ( *(addr) & (bit) )      /* judge bit is set */

#ifndef BV
    #define BV(n)      (1 << (n))
#endif

#ifndef BF
    #define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef ABS
    #define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif

/* takes a byte out of a uint32 : var - uint32,
 * ByteNum - byte to take out (0 - 3)
 */

#define BREAK_UINT32( var, ByteNum ) \
    (uint8)((uint32)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
    ((uint32)((uint32)((Byte0) & 0x00FF) \
              + ((uint32)((Byte1) & 0x00FF) << 8) \
              + ((uint32)((Byte2) & 0x00FF) << 16) \
              + ((uint32)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
    ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT8(hiByte, loByte) \
    ((uint8)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

/* Write the 32bit value of 'val' in little endian format
 * to the buffer pointed to by pBuf, and increment pBuf by 4
 */

#define UINT32_TO_BUF_LITTLE_ENDIAN(pBuf,val) \
    do { \
        *(pBuf)++ = (((val) >>  0) & 0xFF); \
        *(pBuf)++ = (((val) >>  8) & 0xFF); \
        *(pBuf)++ = (((val) >> 16) & 0xFF); \
        *(pBuf)++ = (((val) >> 24) & 0xFF); \
    } while (0)

/* Return the 32bit little-endian formatted value pointed to by pBuf,
 * and increment pBuf by 4
 */

#define BUF_TO_UINT32_LITTLE_ENDIAN(pBuf) (((pBuf) += 4), BUILD_UINT32((pBuf)[-4], (pBuf)[-3], (pBuf)[-2], (pBuf)[-1]))

#ifndef GET_BIT
    #define GET_BIT(DISCS, IDX)  (((DISCS)[((IDX) / 8)] & BV((IDX) % 8)) ? TRUE : FALSE)
#endif
#ifndef SET_BIT
    #define SET_BIT(DISCS, IDX)  (((DISCS)[((IDX) / 8)] |= BV((IDX) % 8)))
#endif
#ifndef CLR_BIT
    #define CLR_BIT(DISCS, IDX)  (((DISCS)[((IDX) / 8)] &= (BV((IDX) % 8) ^ 0xFF)))
#endif

/*  -------------------------------------------------------------------------
 *                               Standard Defines
 *   ------------------------------------------------------------------------
 */

#define HAL_WAIT_CONDITION(condition)  {while(!(condition)){}}

#define HAL_WAIT_CONDITION_TIMEOUT(condition, timeout)  {\
        volatile int val = 0;\
        while(!(condition)){\
            if(val ++ > timeout)\
                return PPlus_ERR_TIMEOUT;\
        }\
    }

#define HAL_WAIT_CONDITION_TIMEOUT_WO_RETURN(condition, timeout)  {\
        volatile int val = 0;\
        while (!(condition)){\
            if (val++ > timeout)\
                break;\
        }\
    }

typedef struct _comm_evt_t
{
    unsigned int    type;
    unsigned char  *data;
    unsigned int    len;
} comm_evt_t;

typedef void (*comm_cb_t)(comm_evt_t *pev);

#define __RAMRUN               /* __attribute__((section(".ramscttext"))) */

#define __ATTR_SECTION_SRAM__  /* __attribute__((section("_section_sram_code_"))) */
#define __ATTR_SECTION_XIP__   /* __attribute__((section("_section_xip_code_"))) */

#endif

