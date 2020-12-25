/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_bcm20706.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BSP_BOARD_COMMON_INCLUDE_CXD56_BCM20706_H
#define __BSP_BOARD_COMMON_INCLUDE_CXD56_BCM20706_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_BCM20706

/****************************************************************************
 * Name: board_bluetooth_pin_cfg
 *
 * Description:
 *   Initialize bcm20707 control pins, it must be called before any operation
 *   to do power control, wake up and reset.
 *
 ****************************************************************************/

int board_bluetooth_pin_cfg(void);

/****************************************************************************
 * Name: board_bluetooth_uart_pin_cfg
 *
 * Description:
 *   Setup UART pin configuration for bcm20706.
 *
 ****************************************************************************/

int board_bluetooth_uart_pin_cfg(void);

/****************************************************************************
 * Name: board_bluetooth_reset
 *
 * Description:
 *   Reset bcm20707 chip
 *
 ****************************************************************************/

void board_bluetooth_reset(void);

/****************************************************************************
 * Name: board_bluetooth_power_control
 *
 * Description:
 *   Power on/off bcm20707 chip
 *
 ****************************************************************************/

int board_bluetooth_power_control(bool en);

/****************************************************************************
 * Name: board_bluetooth_enable_sleep
 *
 * Description:
 *   Enable/disable bcm20707 enters sleep mode
 *
 ****************************************************************************/

void board_bluetooth_enable_sleep(bool en);
#endif /* CONFIG_BCM20706 */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BSP_BOARD_COMMON_INCLUDE_CXD56_BCM20706_H */
