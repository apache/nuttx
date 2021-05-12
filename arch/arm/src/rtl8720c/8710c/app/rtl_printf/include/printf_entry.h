/**************************************************************************//**
 * @file     printf_entry.h
 * @brief    This file defines the printf API entry functions table.
 * 
 * @version  V1.00
 * @date     2016-09-28
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
 
#ifndef _PRINTF_ENTRY_H
#define _PRINTF_ENTRY_H

#include "basic_types.h"
#include "stdio_port.h"
#include "log_buf.h"

#ifdef  __cplusplus
extern "C" {
#endif
/**
  \brief  The data structure of the stubs function for the printf API in ROM
*/
// Secure region printf not support full function, and not support buffer log
typedef struct stdio_printf_func_stubs_s {
    void (*stdio_port_init)(void *adapter, stdio_putc_t putc, stdio_getc_t getc);
    void (*stdio_port_deinit)(void);
    int (*stdio_port_putc)(char c);
    int (*stdio_port_sputc)(void *arg, char c);
    int (*stdio_port_bufputc)(void *buf, char c);
    int (*stdio_port_getc) (char *data);

    unsigned (*printf_corel)(printf_putc_t putc, void *arg, const char *fmt, va_list args);

    int (*rt_printfl)(const char *fmt,...);
    int (*rt_sprintfl)(char *buf, const char *fmt,...);
    int (*rt_snprintfl)(char *buf, size_t size, const char *fmt,...);
#if !defined(CONFIG_BUILD_SECURE)
    unsigned (*printf_core)(printf_putc_t putc, void *arg, const char * fmt, va_list _args);
    int (*rt_printf)(const char * fmt,...);
    int (*rt_sprintf)(char *buf, const char * fmt,...);
    int (*rt_snprintf)(char *buf, size_t size, const char *fmt,...);

    void (*log_buf_init)(log_buf_type_t *plog);
    int (*log_buf_putc) (log_buf_type_t *plog, char ch);
    void (*log_buf_set_msg_buf)(log_buf_type_t *plog, char *pbuf, uint32_t buf_size);
    void (*log_buf_show)(log_buf_type_t *plog);
    int (*log_buf_printf)(log_buf_type_t *pglog, const char * fmt, ...);

    int (*rt_sscanf)(const char *buf, const char *fmt, ...);
#endif
    uint32_t reserved[8];  // reserved space for next ROM code version function table extending.
} stdio_printf_func_stubs_t;

#ifdef  __cplusplus
}
#endif

#endif  // end of '#ifndef _PRINTF_ENTRY_H'

