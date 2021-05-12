/**************************************************************************//**
 * @file     stdio_port.h
 * @brief    The header file of the UART STDIO functions.
 *           
 * @version  V1.00
 * @date     2016-09-22
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

#ifndef _STDIO_PORT_H_
#define _STDIO_PORT_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/**
  \brief  Type of UART STDIO functions.
 */
typedef void (*stdio_putc_t)(void *apapter, const char);
typedef int (*stdio_getc_t) (void *adapter, char *);
typedef int (*printf_putc_t)(void *arg, const char);


/**
  \brief  Type of the UART STDIO adapter.
 */
typedef struct _stdio_port {
    void *adapter;
    stdio_putc_t putc;
    stdio_getc_t getc;
} stdio_port_t, *pstdio_port_t;

/**
  \brief  Type of the UART STDIO adapter.
 */
typedef struct _stdio_buf_s {
    char *pbuf;
    char *pbuf_lim;
} stdio_buf_t, *pstdio_buf_t;

#if defined(ROM_REGION)

void _stdio_port_init (void *adapter, stdio_putc_t putc, stdio_getc_t getc);
void _stdio_port_deinit (void);
int _stdio_port_putc(char c);
int _stdio_port_sputc(void *arg, char c);
int _stdio_port_bufputc(void *buf, char c);
int _stdio_port_getc (char *data);

#endif  // end of "#if defined(ROM_REGION)"

#ifdef  __cplusplus
}
#endif

#endif  // #ifndef _STDIO_PORT_H_

