/************************************************************************************
 * configs/stm32l476-mdk/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Motorola Mobility, LLC.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ************************************************************************************/

#ifndef __CONFIGS_STM32L476_MDK_INCLUDE_BOARD_H
#define __CONFIGS_STM32L476_MDK_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32l4.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

#include <arch/board/stm32l476-mdk-clocking.h>

/* DMA Channel/Stream Selections ****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 */

/* Alternate function pin selections ************************************************/

/* USART1:
 *   RXD: PB7
 *   TXD: PA6
 *
 * these pins are shared with the I2C1, which is used for some onboard
 * devices, so I am disabling USART1, however I'm leaving the declarations
 * here for documentation for anyone who might want to use it instead of
 * those onboard devices.
 */

/* USART */

#define GPIO_USART1_CTS  GPIO_USART1_CTS_3   /* PG11 */
#define GPIO_USART1_RTS  GPIO_USART1_RTS_DE_2  /* PB3  */
#define GPIO_USART1_RX   GPIO_USART1_RX_2    /* PB7  */
#define GPIO_USART1_TX   GPIO_USART1_TX_2    /* PB6  */

#define GPIO_USART2_CTS  GPIO_USART2_CTS_1   /* PA0  */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_DE_1 /* PA1  */
#define GPIO_USART2_RX   GPIO_USART2_RX_1    /* PA3  */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2  */

#define GPIO_USART3_RX   GPIO_USART3_RX_3    /* PC11 */
#define GPIO_USART3_TX   GPIO_USART3_TX_3    /* PC10 */

/* I2C */

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1     /* PB10 */
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1     /* PB11 */

#define GPIO_I2C3_SCL    GPIO_I2C3_SCL_1     /* PC0  */
#define GPIO_I2C3_SDA    GPIO_I2C3_SDA_1     /* PC1  */

/* SPI */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1    /* PA6  */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1    /* PA7  */
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1     /* PA5  */
#define GPIO_SPI1_NSS    GPIO_SPI1_NSS_1     /* PA4  */
#define DMACHAN_SPI1_RX  DMACHAN_SPI1_RX_1
#define DMACHAN_SPI1_TX  DMACHAN_SPI1_TX_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1    /* PB14 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1    /* PB15 */
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2     /* PB13 */
#define GPIO_SPI2_NSS    GPIO_SPI2_NSS_2     /* PB12 */

/* Buttons **************************************************************************/
/* The board only has one button */

#define BUTTON_POWER       0
#define NUM_BUTTONS        1
#define BUTTON_POWER_BIT   (1 << BUTTON_POWER)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32L4 architectures must provide the following entry point.  This entry
 *   point is called early in the initialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32l4_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_STM32L476_MDK_INCLUDE_BOARD_H */
