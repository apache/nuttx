/**************************************************************************//**
 * @file     log_buf.h
 * @brief    The log buffer function related definition and declaration.
 * @version  V1.00
 * @date     2016-05-31
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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

#ifndef _LOG_BUF_H_
#define _LOG_BUF_H_

#include <stdarg.h>
#include "basic_types.h"
#ifdef  __cplusplus
extern "C" {
#endif

#define LOG_BUF_SIZE            256

//typedef void (*log_buf_port_putc_t) (void *adapter, char);

typedef struct log_buf_type_s {
    uint32_t    buf_w;      /*!< the log buffer write index */
    uint32_t    buf_r;      /*!< the log buffer read (dump) index */
    uint32_t    buf_sz;     /*!< the size, in byte, of the log buffer write index */
//    log_buf_port_putc_t port_putc;  /*!< the function to put character to the real debug port, like an UART port */
//    void        *port_adapter;  /*!< the argument for the real debug port put char function calling  */    
    char        *log_buf;   /*!< the log message buffer */
    BOOL        initialed;  /*!< is the log buffer structure initialed */
} log_buf_type_t, *plog_buf_type_t;


void _log_buf_show (log_buf_type_t *plog);
void _log_buf_init (log_buf_type_t *plog);
int _log_buf_putc (log_buf_type_t *plog, char ch);

int _log_buf_printf(log_buf_type_t *pglog, const char * fmt, ...);
void _log_buf_set_msg_buf (log_buf_type_t *plog, char *pbuf, uint32_t buf_size);
// 
#ifdef  __cplusplus
}
#endif

#endif
