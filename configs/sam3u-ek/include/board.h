/************************************************************************************
 * configs/sam3u-ek/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "sam3u_internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the sam3u device is running on a 4MHz internal RC.  These
 * definitions will configure clocking with MCK = 48MHz, PLLA = 96, and CPU=48MHz.
 */

/* Main oscillator register settings */

#define BOARD_CKGR_MOR_MOSCXTST    (63 << CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration */

#define BOARD_CKGR_PLLAR_MULA      (7 << CKGR_PLLAR_MULA_SHIFT)
#define BOARD_CKGR_PLLAR_STMODE    CKGR_PLLAR_STMODE_FAST
#define BOARD_CKGR_PLLAR_PLLACOUNT (63 << CKGR_PLLAR_PLLACOUNT_SHIFT)
#define BOARD_CKGR_PLLAR_DIVA      CKGR_PLLAR_DIVA_BYPASS

/* PMC master clock register settings */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV2

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT (3 << CKGR_UCKR_UPLLCOUNT_SHIFT)

/* Resulting frequencies */

#define SAM3U_MAINOSC_FREQUENCY    (12000000)
#define SAM3U_MCK_FREQUENCY        (48000000)
#define SAM3U_PLLA_FREQUENCY       (96000000)
#define SAM3U_CPU_FREQUENCY        (48000000)

/* LED definitions ******************************************************************/

#define LED_STARTED                0
#define LED_HEAPALLOCATE           1
#define LED_IRQSENABLED            2
#define LED_STACKCREATED           3
#define LED_INIRQ                  4
#define LED_SIGNAL                 5
#define LED_ASSERTION              6
#define LED_PANIC                  7 

/* GPIO pin definitions *************************************************************/

#define GPIO_ADC0_AD0             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN21)
#define GPIO_ADC0_AD1             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN30)
#define GPIO_ADC0_AD2             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN3)
#define GPIO_ADC0_AD3             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN4)
#define GPIO_ADC0_AD4             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN15)
#define GPIO_ADC0_AD5             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN16)
#define GPIO_ADC0_AD6             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN17)
#define GPIO_ADC0_AD7             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN18)


#define GPIO_CAN_XCVR_RS          (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_OUTPUT_SET|GPIO_PIN23)
#define GPIO_CAN1_XCVR_TXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN27)
#define GPIO_CAN1_XCVR_RXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN26)
#define GPIO_CAN2_XCVR_TXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN29)
#define GPIO_CAN2_XCVR_RXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN28)

#define GPIO_UART_TXD             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN12)
#define GPIO_UART_RXD             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN11)

#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN9)  /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN10) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN11) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN12) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN13) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN14) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN15) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN16) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN25) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN26) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN27) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN28) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN29) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN30) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN31) /* Check! */
#define GPIO_SMC_D0               (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN6)   /* Check! */
#define GPIO_SMC_NCS0             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN20)
#define GPIO_SMC_NRD              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN19)
#define GPIO_SMC_NWE              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN23)
#define GPIO_SMC_PSRAM_A0         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN0)  /* Check! */
#define GPIO_SMC_PSRAM_A1         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN1)  /* Check! */
#define GPIO_SMC_PSRAM_A2         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN2)  /* Check! */
#define GPIO_SMC_PSRAM_A3         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN3)  /* Check! */
#define GPIO_SMC_PSRAM_A4         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN4)  /* Check! */
#define GPIO_SMC_PSRAM_A5         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN5)  /* Check! */
#define GPIO_SMC_PSRAM_A6         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN6)  /* Check! */
#define GPIO_SMC_PSRAM_A7         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN7)  /* Check! */
#define GPIO_SMC_PSRAM_A8         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN8)  /* Check! */
#define GPIO_SMC_PSRAM_A9         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN9)  /* Check! */
#define GPIO_SMC_PSRAM_A10        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN10) /* Check! */
#define GPIO_SMC_PSRAM_A11        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN11) /* Check! */
#define GPIO_SMC_PSRAM_A12        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN24) /* Check! */
#define GPIO_SMC_PSRAM_A13        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN25) /* Check! */
#define GPIO_SMC_PSRAM_A14        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN26) /* Check! */
#define GPIO_SMC_PSRAM_A15        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN27) /* Check! */
#define GPIO_SMC_PSRAM_A16        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN27) /* Check! */
#define GPIO_SMC_PSRAM_A17        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN28) /* Check! */
#define GPIO_SMC_PSRAM_A18        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN29) /* Check! */
#define GPIO_SMC_PSRAM_NBS0       (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN7)   /* Check! */
#define GPIO_SMC_PSRAM_NBS1       (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|GPIO_PIN15)
#define GPIO_SMC_A1               (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN8)
#define GPIO_SMC_NCS2             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|GPIO_PIN16)
#define GPIO_SMC_LCD_RS           (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN8)

#define GPIO_LED0                 (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_CLEAR|GPIO_PIN0)
#define GPIO_LED1                 (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_SET|GPIO_PIN1)
#define GPIO_LED2                 (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_OUTPUT_SET|GPIO_PIN2)

#define GPIO_MCI_DAT0             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN5)
#define GPIO_MCI_DAT1             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN6)
#define GPIO_MCI_DAT2             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN7)
#define GPIO_MCI_DAT3             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN8)
#define GPIO_MCI_DAT4             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN28)
#define GPIO_MCI_DAT5             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN29)
#define GPIO_MCI_DAT6             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN30)
#define GPIO_MCI_DAT7             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN31)
#define GPIO_MCI_CK               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN3)
#define GPIO_MCI_DA               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN4)
#define GPIO_MCI_CD               (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|{AT91C_PIO_PA25)

#define GPIO_BUTTON1              (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_CFG_DEGLITCH|GPIO_PORT_PIOA|GPIO_PIN18)
#define GPIO_BUTTON2              (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_CFG_DEGLITCH|GPIO_PORT_PIOA|GPIO_PIN19)

#define GPIO_PWMC_PWMH0           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN0)
#define GPIO_PWMC_PWML0           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN7)
#define GPIO_PWMC_PWMH1           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN1)
#define GPIO_PWMC_PWML1           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN8)
#define GPIO_PWMC_PWMH2           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN2)
#define GPIO_PWMC_PWML2           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN9)

#define GPIO_SPI0_MISO            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN13)
#define GPIO_SPI0_MOSI            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN14)
#define GPIO_SPI0_SPCK            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN15)
//#define GPIO_SPI0_NPCS2_PC14    (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN14)
#define GPIO_SPI0_NPCS2_PC14      (GPIO_OUTPUT|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|GPIO_OUTPUT_CLEAR|GPIO_PIN14)

#define GPIO_SSC_TD               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN26)
#define GPIO_SSC_TK               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN28)
#define GPIO_SSC_TF               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN30)

#define GPIO_PCK0                 (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN21)

#define GPIO_TWI_TWD0             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN9)
#define GPIO_TWI_TWCK0            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN10)
#define GPIO_TWI_TWD1             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN24)
#define GPIO_TWI_TWCK1            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN25)

#define GPIO_USART0_RXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN19)
#define GPIO_USART0_TXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN18)
#define GPIO_USART0_CTS           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN8)
#define GPIO_USART0_RTS           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN7)
#define GPIO_USART0_SCK           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN17)

#define GPIO_USART1_RXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN21)
#define GPIO_USART1_TXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN20)
#define GPIO_USART1_CTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN23)
#define GPIO_USART1_RTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN22)
#define GPIO_USART1_SCK           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN24)

#define GPIO_USB_VBUS             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN0)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: sam3u_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void sam3u_boardinitialize(void);

/************************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After that,
 *   up_buttons() may be called to collect the state of all buttons.  up_buttons()
 *   returns an 8-bit bit set with each bit associated with a button.  See the
 *   BUTTON_* and JOYSTICK_* definitions above for the meaning of each bit.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
