/****************************************************************************
 * arch/arm/src/lc823450/lc823450_gpio.h
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_GPIO_H
#define __ARCH_ARM_SRC_LC823450_LC823450_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Max number of GPIO ports and the maximum number of pins per port */

#ifdef CONFIG_IOEX
#  define NUM_GPIO_PORTS                7
#  define NUM_GPIOEX_PINS               CONFIG_IOEX_NPINS
#else
#  define NUM_GPIO_PORTS                6
#endif

#define NUM_GPIO_PINS                  16

/* Input/Output mode
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * MM.. .... .... ....
 */

#define GPIO_MODE_SHIFT            (14)       /* Bits 14-15: Mode of the GPIO pin */
#define GPIO_MODE_MASK             (3 << GPIO_MODE_SHIFT)
#  define GPIO_MODE_INPUT          (0 << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_MODE_OUTPUT         (1 << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_MODE_PININTR        (2 << GPIO_MODE_SHIFT) /* GPIO pin interrupt */

#define GPIO_IS_OUTPUT(p)          (((p) & GPIO_MODE_MASK) == GPIO_MODE_OUTPUT)
#define GPIO_IS_INPUT(p)           (((p) & GPIO_MODE_MASK) == GPIO_MODE_INPUT)
#define GPIO_IS_PININT(p)          (((p) & GPIO_MODE_MASK) == GPIO_MODE_PININTR)

/* Initial value (for GPIO outputs only)
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ..V. .... .... ....
 */

#define GPIO_VALUE_ONE             (1 << 13) /* Bit 13: 1=High */
#define GPIO_VALUE_ZERO            (0)       /* Bit 13: 0=Low */

#define GPIO_IS_ONE(p)             (((p) & GPIO_VALUE_ONE) != 0)
#define GPIO_IS_ZERO(p)            (((p) & GPIO_VALUE_ONE) == 0)

/* GPIO pinmux
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... XX.. .... ....
 */

#define GPIO_MUX_SHIFT             (10)  /* Bits 10-11: pinmux */
#define GPIO_MUX_MASK              (3 << GPIO_MUX_SHIFT)
#  define GPIO_MUX0                (0 << GPIO_MUX_SHIFT)     /* mux mode 0 */
#  define GPIO_MUX1                (1 << GPIO_MUX_SHIFT)     /* mux mode 1 */
#  define GPIO_MUX2                (2 << GPIO_MUX_SHIFT)     /* mux mode 2 */
#  define GPIO_MUX3                (3 << GPIO_MUX_SHIFT)     /* mux mode 3 */


/* GPIO pull-ups/downs
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... ..UU .... ....
 */

#define GPIO_PUPD_SHIFT            (8)  /* Bits 8-9: Pull-up/pull down */
#define GPIO_PUPD_MASK             (3 << GPIO_PUPD_SHIFT)
#  define GPIO_DEFAULT             (0 << GPIO_PUPD_SHIFT)     /* H/W default */
#  define GPIO_PULLUP              (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLDOWN            (2 << GPIO_PUPD_SHIFT)     /* Pull-down */
#  define GPIO_FLOAT               (3 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */

/* GPIO Port Number:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... PPP. ....
 */

#define GPIO_PORT_SHIFT            (5)        /* Bits 5-7: Port number */
#define GPIO_PORT_MASK             (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0               (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1               (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2               (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3               (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4               (4 << GPIO_PORT_SHIFT)
#  define GPIO_PORT5               (5 << GPIO_PORT_SHIFT)
#ifdef CONFIG_IOEX
#  define GPIO_PORTEX              (6 << GPIO_PORT_SHIFT)
#endif
#ifdef CONFIG_LC823450_VGPIO
#  define GPIO_PORTV               (7 << GPIO_PORT_SHIFT)
#endif /* CONFIG_LC823450_VGPIO */

/* GPIO Pin Number:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             (0)        /* Bits 0-4: Pin number */
#define GPIO_PIN_MASK              (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                (9 << GPIO_PIN_SHIFT)
#  define GPIO_PINA                (10 << GPIO_PIN_SHIFT)
#  define GPIO_PINB                (11 << GPIO_PIN_SHIFT)
#  define GPIO_PINC                (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIND                (13 << GPIO_PIN_SHIFT)
#  define GPIO_PINE                (14 << GPIO_PIN_SHIFT)
#  define GPIO_PINF                (15 << GPIO_PIN_SHIFT)
#ifdef CONFIG_IOEX
#  define GPIO_PIN10               (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11               (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12               (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13               (19 << GPIO_PIN_SHIFT)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_LC823450_VGPIO
struct vgpio_ops_s
{
  void  (*write)(uint32_t pin, bool value);
  bool  (*read)(uint32_t pin);
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_gpio_mux
 *
 * Description:
 *   Configure pin mux for a GPIO
 *
 ****************************************************************************/

int lc823450_gpio_mux(uint16_t gpiocfg);

/****************************************************************************
 * Name: lc823450_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 ****************************************************************************/

int lc823450_gpio_config(uint16_t gpiocfg);

/****************************************************************************
 * Name: lc823450_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lc823450_gpio_write(uint16_t gpiocfg, bool value);

/****************************************************************************
 * Name: lc823450_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool lc823450_gpio_read(uint16_t gpiocfg);

/****************************************************************************
 * Function:  lc823450_gpio_dump
 *
 * Description:
 *   Dump all pin configuration registers
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
int lc823450_gpio_dump(uint16_t gpiocfg, FAR const char *msg);
#else
#  define lc823450_gpio_dump(p,m)
#endif

/****************************************************************************
 * Name: lc823450_vgpio_register
 *
 * Description:
 *   Register Virtual GPIO driver
 *
 ****************************************************************************/

#ifdef CONFIG_LC823450_VGPIO
int lc823450_vgpio_register(unsigned int pin, FAR struct vgpio_ops_s *ops);
#endif /* CONFIG_LC823450_VGPIO */


#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_GPIO_H */
