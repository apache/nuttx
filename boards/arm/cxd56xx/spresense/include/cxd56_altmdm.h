/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_altmdm.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ALTMDM_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ALTMDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/boardctl.h>
#include <stdbool.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Altair modem gpio definitions *******************************************/

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_GPIO_IRQ)

/* definitions of gpio pin number */

#define ALTMDM_GPIO_MODEM_WAKEUP            (0)
#define ALTMDM_GPIO_MASTER_REQ              (1)
#define ALTMDM_GPIO_SLAVE_REQ               (2)

/* definitions of gpio interrupt polarity */

#define ALTMDM_GPIOINT_LEVEL_HIGH           (0)
#define ALTMDM_GPIOINT_LEVEL_LOW            (1)
#define ALTMDM_GPIOINT_EDGE_RISE            (2)
#define ALTMDM_GPIOINT_EDGE_FALL            (3)
#define ALTMDM_GPIOINT_EDGE_BOTH            (4)

/* definitions of gpio interrupt noise filter */

#define ALTMDM_GPIOINT_NOISE_FILTER_ENABLE  (0)
#define ALTMDM_GPIOINT_NOISE_FILTER_DISABLE (1)

#endif

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

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_GPIO_IRQ)

/****************************************************************************
 * Name: board_altmdm_initialize
 *
 * Description:
 *   Initialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_initialize(FAR const char *devpath);

/****************************************************************************
 * Name: board_altmdm_uninitialize
 *
 * Description:
 *   Uninitialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_uninitialize(void);

/****************************************************************************
 * Name: board_altmdm_power_control
 *
 * Description:
 *   Power on/off the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_power_control(bool en);

/****************************************************************************
 * Name: board_altmdm_poweron
 *
 * Description:
 *   Power on the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_poweron(void);

/****************************************************************************
 * Name: board_altmdm_poweroff
 *
 * Description:
 *   Power off the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_poweroff(void);

/****************************************************************************
 * Name: board_altmdm_gpio_write
 *
 * Description:
 *   Write GPIO pin.
 *
 ****************************************************************************/

void board_altmdm_gpio_write(uint32_t pin, bool value);

/****************************************************************************
 * Name: board_altmdm_gpio_read
 *
 * Description:
 *   Read GPIO pin.
 *
 ****************************************************************************/

bool board_altmdm_gpio_read(uint32_t pin);

/****************************************************************************
 * Name: board_altmdm_gpio_irq
 *
 * Description:
 *   Register GPIO irq.
 *
 ****************************************************************************/

void board_altmdm_gpio_irq(uint32_t pin, uint32_t polarity,
                           uint32_t noise_filter, xcpt_t irqhandler);

/****************************************************************************
 * Name: board_altmdm_gpio_int_control
 *
 * Description:
 *   Enable or disable GPIO interrupt.
 *
 ****************************************************************************/

void board_altmdm_gpio_int_control(uint32_t pin, bool en);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_ALTMDM_H */
