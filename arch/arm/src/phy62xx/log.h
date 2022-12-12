/****************************************************************************
 * arch/arm/src/phy62xx/log.h
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
 *    @file     log.h
 *    @brief    Contains all functions support for uart driver
 *    @version  0.0
 *    @date     31. Jan. 2018
 *    @author   eagle.han
 ****************************************************************************/

#ifndef ENABLE_LOG_ROM
#ifndef __ARCH_ARM_SRC_PHY62XX_LOG_H
#define __ARCH_ARM_SRC_PHY62XX_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "uart.h"
void dbg_printf_(const char *format, ...) printf_like(1, 2);
void dbg_printf_init(void);
void my_dump_byte(uint8_t *pdata, int dlen);
#ifndef DEBUG_INFO
#error "DEBUG_INFO undefined!"
#endif
typedef void(*std_putc)(char *data, uint16_t size);

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
#define LOG_DEBUG_PHY(...)
#define LOG(...)
#define LOG_INIT()  //{clk_gate_enable(MOD_UART);clk_reset(MOD_UART);clk_gate_disable(MOD_UART);}
#define LOG_DUMP_BYTE(a,b)
#define LOG_BLE(...) printf(__VA_ARGS__)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_PHY62XX_LOG_H */

#else

#ifndef __ARCH_ARM_SRC_PHY62XX_LOG_H
#define __ARCH_ARM_SRC_PHY62XX_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "types.h"
#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

#define LOG_LEVEL_NONE  0 /* no log output */
#define LOG_LEVEL_ERROR 1 /* only log error */
#define LOG_LEVEL_DEBUG 2 /* output debug info and error info */
#define LOG_LEVEL_LOG   3 /* output all information */

#define LOG_INIT()                  {hal_uart_init(115200, P9, P10, NULL);}

#if 0//DEBUG_FPGA
#define LOG(...) do{;}while(0);
#else

/* conditional output */
#define LOG(...)                    {if(s_rom_debug_level == LOG_LEVEL_LOG) log_printf(__VA_ARGS__);}
#define LOG_DEBUG(...)              {if(s_rom_debug_level >= LOG_LEVEL_DEBUG) log_printf(__VA_ARGS__);}
#define LOG_ERROR(...)              {if(s_rom_debug_level >= LOG_LEVEL_ERROR) log_printf(__VA_ARGS__);}

/* tx data anyway */
#define PRINT(...)                  {SWU_TX(); log_printf(__VA_ARGS__);}
#endif

extern volatile uint32_t s_rom_debug_level;

typedef void(*std_putc)(char *data, int size);

void log_vsprintf(std_putc putc, const char *fmt, va_list args)
     printf_like(2, 0);
void log_printf(const char *format, ...) printf_like(1, 2);
void log_set_putc(std_putc putc);
void log_clr_putc(std_putc putc);
int log_debug_level(uint8_t level);
uint32_t log_get_debug_level(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_PHY62XX_LOG_H */

#endif
