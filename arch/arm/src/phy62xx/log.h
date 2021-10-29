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
    @file     log.h
    @brief    Contains all functions support for uart driver
    @version  0.0
    @date     31. Jan. 2018
    @author   eagle.han



*******************************************************************************/
#ifndef ENABLE_LOG_ROM
#ifndef __LOG_H__
#define __LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"
void dbg_printf_(const char* format, ...);
void dbg_printf_init(void);
void my_dump_byte(uint8_t* pData, int dlen);
#ifndef DEBUG_INFO
#error "DEBUG_INFO undefined!"
#endif
typedef void(*std_putc)(char* data, uint16_t size);

#if 0
   #if(DEBUG_INFO == 1)
     #define AT_LOG(...)
     #define LOG_DEBUG(...)
     #define LOG(...)  dbg_printf_(__VA_ARGS__)
     #define LOG_INIT() dbg_printf_init()
     #define LOG_DUMP_BYTE(a,b) my_dump_byte(a,b)
   #elif(DEBUG_INFO == 2)
     
     #define AT_LOG(...)  dbg_printf_(__VA_ARGS__)
     #define LOG_DEBUG(...)  dbg_printf_(__VA_ARGS__)
     #define LOG(...)
     #define LOG_INIT() dbg_printf_init()
     #define LOG_DUMP_BYTE(a,b) my_dump_byte(a,b)
   #elif(DEBUG_INFO == 3)
     #define LOG(...)  dbg_printf_(__VA_ARGS__)
     #define AT_LOG(...)  dbg_printf_(__VA_ARGS__)
     #define LOG_DEBUG(...)  dbg_printf_(__VA_ARGS__)
     #define LOG_INIT() dbg_printf_init()
     #define LOG_DUMP_BYTE(a,b) my_dump_byte(a,b)
   #else

   #endif
#else
#define AT_LOG(...)
#define LOG_DEBUG(...)
#define LOG(...)
#define LOG_INIT()  //{clk_gate_enable(MOD_UART);clk_reset(MOD_UART);clk_gate_disable(MOD_UART);}
#define LOG_DUMP_BYTE(a,b)
#define LOG_BLE(...) printf(__VA_ARGS__)
#endif

#ifdef __cplusplus
}
#endif

#endif //__LOG_H__

#else

#ifndef __PHY_LOG_H
#define __PHY_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

#define LOG_LEVEL_NONE  0 //no log output*/
#define LOG_LEVEL_ERROR 1 //only log error*/
#define LOG_LEVEL_DEBUG 2 //output debug info and error info*/
#define LOG_LEVEL_LOG   3 //output all infomation*/

#define LOG_INIT()                  {hal_uart_init(115200,P9,P10,NULL);}

#if 0//DEBUG_FPGA
#define LOG(...) do{;}while(0);
#else

//conditional output
#define LOG(...)                    {if(s_rom_debug_level == LOG_LEVEL_LOG) log_printf(__VA_ARGS__);}
#define LOG_DEBUG(...)              {if(s_rom_debug_level >= LOG_LEVEL_DEBUG) log_printf(__VA_ARGS__);}
#define LOG_ERROR(...)              {if(s_rom_debug_level >= LOG_LEVEL_ERROR) log_printf(__VA_ARGS__);}

//tx data anyway
#define PRINT(...)                  {SWU_TX(); log_printf(__VA_ARGS__);}
#endif

extern volatile uint32_t s_rom_debug_level;

typedef void(*std_putc)(char* data, int size);

void log_vsprintf(std_putc putc, const char* fmt, va_list args);
void log_printf(const char* format, ...);
void log_set_putc(std_putc putc);
void log_clr_putc(std_putc putc);
int log_debug_level(uint8_t level);
uint32_t log_get_debug_level(void);

#ifdef __cplusplus
}
#endif

#endif

#endif
