/****************************************************************************
 * configs/stm32ldiscovery/src/stm32_lcd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "up_arch.h"
#include "chip/stm32_lcd.h"
#include "stm32ldiscovery.h"

#ifdef CONFIG_STM32_LCD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug must
 * also be enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

/* LCD **********************************************************************/
/* LCD.  The STM32L152RBT6 supports either a 4x32 or 8x28.  The STM32L-
 * Discovery has an LCD 24 segments, 4 commons.  See stm32ldiscovery.h for
 * the pin mapping.
 */

/* Macro to convert an LCD register offset and bit number into a bit-band
 * address:
 */

#define LCD_OFFSET       (STM32_LCD_BASE - STM32_PERIPH_BASE)
#define LCD_BBADDR(o,b)  (STM32_PERIPHBB_BASE + ((LCD_OFFSET + (o)) << 5) + ((b) << 2))

/* Some useful bit-band addresses */

#define LCD_CR_LCDEN_BB  LCD_BBADDR(STM32_LCD_CR_OFFSET,0)
#define LCD_SR_UDR_BB    LCD_BBADDR(STM32_LCD_SR_OFFSET,2)

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg         dbg
#  define lcdvdbg        vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Indices into the g_lcdgpio[] array */

enum stm32_gpio_e
{
  LCD_COM0 = 0,  LCD_COM1, LCD_COM2, LCD_COM3,

  LCD_SEG0,  LCD_SEG1,  LCD_SEG2,  LCD_SEG3,  LCD_SEG4,  LCD_SEG5,  LCD_SEG6,
  LCD_SEG7,  LCD_SEG8,  LCD_SEG9,  LCD_SEG10, LCD_SEG11, LCD_SEG12, LCD_SEG13,
  LCD_SEG14, LCD_SEG15, LCD_SEG16, LCD_SEG17, LCD_SEG18, LCD_SEG19, LCD_SEG20,
  LCD_SEG21, LCD_SEG22, LCD_SEG23,

  LCD_NGPIOS
};

struct stm32_lcd_s
{
  bool initialized; /* True: Completed initialization sequence */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/
/* Internal utilities */

static void lcd_clear(void)

/* Character driver methods */

static ssize_t lcd_read(FAR struct file *, FAR char *, size_t);
static ssize_t lcd_write(FAR struct file *, FAR const char *, size_t);
#ifndef CONFIG_DISABLE_POLL
static int     lcd_poll(FAR struct file *filp, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the driver state structure (there is no retained state information) */

static const struct file_operations g_lcdops =
{
  0,             /* open */
  0,             /* close */
  lcd_read,      /* read */
  lcd_write,     /* write */
  0,             /* seek */
  0              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , lcd_poll     /* poll */
#endif
};

/* All GPIOs that need to be configured for the STM32L-Discovery LCD */

static uint32_t g_lcdgpio[BOARD_LCD_NGPIOS] =
{
  BOARD_LCD_COM0,  BOARD_LCD_COM1, BOARD_LCD_COM2, BOARD_LCD_COM3,

  BOARD_LCD_SEG0,  BOARD_LCD_SEG1,  BOARD_LCD_SEG2,  BOARD_LCD_SEG3,
  BOARD_LCD_SEG4,  BOARD_LCD_SEG5,  BOARD_LCD_SEG6,  BOARD_LCD_SEG7,
  BOARD_LCD_SEG8,  BOARD_LCD_SEG9,  BOARD_LCD_SEG10, BOARD_LCD_SEG11,
  BOARD_LCD_SEG12, BOARD_LCD_SEG13, BOARD_LCD_SEG14, BOARD_LCD_SEG15,
  BOARD_LCD_SEG16, BOARD_LCD_SEG17, BOARD_LCD_SEG18, BOARD_LCD_SEG19,
  BOARD_LCD_SEG20, BOARD_LCD_SEG21, BOARD_LCD_SEG22, BOARD_LCD_SEG23
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcd_clear
 ****************************************************************************/

/****************************************************************************
 * Name: lcd_clear
 ****************************************************************************/

static void lcd_clear(void)
{
  uint32_t regaddr;
  int i;

  /* Make sure that any previous transfer is complete.  The firmware sets
   * the UDR each it modifies the LCD_RAM. The UDR bit stays set until the
   * end of the update.  During this time the LCD_RAM is write protected.
   */

  while ((getreg32(STM32_LCD_SR) & LCD_SR_UDR) != 0);

  /* Write all zerios in to the LCD RAM */

  for (regaddr = STM32_LCD_RAML(0); i <= STM32_LCD_RAMH(7); regaddr++)
    {
      putreg32(0, regaddr);
    }

  /* Set the UDR bit to transfer the updated data to the second level
   * buffer.
   */

  putreg32(1, LCD_SR_UDR_BB);
}

/****************************************************************************
 * Name: lcd_read
 ****************************************************************************/

static ssize_t lcd_read(FAR struct file *filp, FAR char *buffer, size_t len)
{
}

/****************************************************************************
 * Name: lcd_write
 ****************************************************************************/

static ssize_t lcd_write(FAR struct file *filp, FAR const char *buffer, size_t len)
{
}

/****************************************************************************
 * Name: lcd_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int lcd_poll(FAR struct file *filp, FAR struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      /* Data is always avaialble to be read */

      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  stm32_lcd_initialize
 *
 * Description:
 *   Initialize the LCD1602 hardware and register the character driver.
 *
 ****************************************************************************/

int stm32_lcd_initialize(void)
{
  uint32_t regval;
  int ret = OK;
  int i;

  /* Only initialize the driver once. */

  if (!g_lcdops.initialized)
    {
      lcdvdbg("Initializing\n");

      /* Configure LCD GPIO pins */

      for (i = 0; i < BOARD_LCD_NGPIOS; i++);
        {
          stm32_configgpio(g_lcdgpio[i]);
        }

      /* Set the LCD prescaler and divider values */

      regval = getreg32(STM32_LCD_FCR);
      regval &= ~(LCD_FCR_DIV_MASK | LCD_FCR_PS_MASK);
      regval |= ( LCD_FCR_PS_DIV1 |  LCD_FCR_DIV(31));
      putreg32(regval, STM32_LCD_FCR);

      /* Wait for the FCRSF flag to be set */

      while ((getreg32(STM32_LCD_SR) & LCD_SR_FCRSF) == 0);

      /* Set the duty (1/4), bias (1/3), and the internal voltage source (VSEL=0) */

      regval = getreg32(STM32_LCD_CR);
      regval &= ~(LCD_CR_BIAS_MASK | LCD_CR_DUTY_MASK | LCD_CR_VSEL);
      regval |= (LCD_CR_DUTY_1TO4 | LCD_CR_BIAS_1TO3);
      putreg32(regval, STM32_LCD_CR);

      /* SEG[31:28] are multiplexed with SEG[43:40] */

      regval |= LCD_CR_MUX_SEG;
      putreg32(regval, STM32_LCD_CR);

      /* Set the contrast to the mean value */

      regval  = getreg32(STM32_LCD_FCR);
      regval &= ~LCD_FCR_CC_MASK;
      regval |=  LCD_FCR_CC_VLCD(4);
      putreg32(regval, STM32_LCD_FCR);

      /* No dead time */

      regval &= ~LCD_FCR_DEAD_MASK;
      putreg32(regval, STM32_LCD_FCR);

      /* Set the pulse-on duration to 4/ck_ps */

      regval &= ~LCD_FCR_PON_MASK;
      regval |= LCD_FCR_PON(4);
      putreg32(regval, STM32_LCD_FCR);

      /* Wait Until the LCD FCR register is synchronized */

      while ((getreg32(STM32_LCD_SR) & LCD_SR_FCRSF) == 0);

      /* Enable LCD peripheral */

      putreg32(1, LCD_CR_LCDEN_BB);

      /* Wait Until the LCD is enabled and the LCD booster is ready */

      while ((getreg32(STM32_LCD_SR) & (LCD_SR_ENS | LCD_SR_RDY)) != (LCD_SR_ENS | LCD_SR_RDY));

      /* Disable blinking */

      regval  = getreg32(STM32_LCD_FCR);
      regval &= ~(LCD_FCR_BLINKF_MASK | LCD_FCR_BLINK_MASK);
      regval |=  (LCD_FCR_BLINK_DISABLE | LCD_FCR_BLINKF_DIV32);
      putreg32(regval, STM32_LCD_FCR);

      /* Register the LCD device driver */

      ret = register_driver("/dev/slcd", &g_lcdops, 0644, &g_lcdops);
      g_lcdops.initialized = true;

      /* Then clear the display */

      lcd_clear();
    }

  return ret;
}

#endif /* CONFIG_STM32_LCD */
