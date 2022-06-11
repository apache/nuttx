/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_spi_console.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>
#include <execinfo.h>

#include <nuttx/compiler.h>
#include <nuttx/board.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "tlsr82_gpio.h"
#include "hardware/tlsr82_spi.h"
#include "hardware/tlsr82_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_TLSR82_SPI_CS_ON 1

#define SPI_WAIT_BUSY(t, _timeout_) \
  do \
    { \
      t = 0; \
      while ((SPI_CTRL_REG & SPI_CTRL_BUSY) && (t < (_timeout_))) \
        { \
          t++; \
        } \
    } \
  while (0)

#ifdef CONFIG_TLSR82_SPI_CS_ON
#  define TLSR82_SPI_CS_HIGH     SPI_CS_HIGH
#  define TLSR82_SPI_CS_LOW      SPI_CS_LOW
#else
#  define TLSR82_SPI_CS_HIGH     0
#  define TLSR82_SPI_CS_LOW      0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_console_dev_s
{
  uint32_t clkdiv;
  uint32_t pin_mosi;
  uint32_t pin_miso;
  uint32_t pin_sck;
  uint32_t pin_cs;
  uint32_t cpol;
  uint32_t cpha;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_TLSR82_SPI_SYSLOG
static inline void spi_putc(char ch);
#endif

static ssize_t spi_console_read(struct file *filep, char *buffer,
                                size_t buflen);
static ssize_t spi_console_write(struct file *filep,
                                 const char *buffer, size_t buflen);
static int     spi_console_ioctl(struct file *filep, int cmd,
                                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_console_dev_s g_console_dev =
{
  .clkdiv   = 2,
  .pin_mosi = GPIO_PIN_PA2 | GPIO_AF_MUX0,
  .pin_miso = GPIO_PIN_PA3 | GPIO_AF_MUX0,
  .pin_sck  = GPIO_PIN_PD7 | GPIO_AF_MUX0,
  .pin_cs   = GPIO_PIN_PD2 | GPIO_AF_MUX0,
  .cpol     = 0,
  .cpha     = 0,
};

static const struct file_operations g_consoleops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  spi_console_read,     /* read */
  spi_console_write,    /* write */
  NULL,                 /* seek */
  spi_console_ioctl,    /* ioctl */
  NULL                  /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                /* unlink */
#endif
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_console_ioctl
 ****************************************************************************/

static int spi_console_ioctl(struct file *filep, int cmd,
                             unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: gpio_console_read
 ****************************************************************************/

static ssize_t spi_console_read(struct file *filep, char *buffer,
                                size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: gpio_console_write
 ****************************************************************************/

static ssize_t spi_console_write(struct file *filep,
                                 const char *buffer, size_t buflen)
{
  int i;

  /* Pull low the CS pin */

  TLSR82_SPI_CS_LOW;

  /* Enable SPI output and set status to write */

  BM_CLR(SPI_CTRL_REG, SPI_CTRL_OUT_OFF);
  BM_CLR(SPI_CTRL_REG, SPI_CTRL_RW_STA);

  for (i = 0; i < buflen; i++)
    {
      SPI_DATA_REG = buffer[i];

      while (SPI_CTRL_REG & SPI_CTRL_BUSY);
    }

  /* Pull high the CS pin */

  TLSR82_SPI_CS_HIGH;

  return buflen;
}

/****************************************************************************
 * Name: spi_putc
 *
 * Description:
 *   Directly output a char by spi per
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   handler       - Callback to be invoked on timer interrupt.
 *   arg           - Argument to be passed to the handler callback.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_SPI_SYSLOG
static inline void spi_putc(char ch)
{
  int t;

  TLSR82_SPI_CS_LOW;

  BM_CLR(SPI_CTRL_REG, SPI_CTRL_OUT_OFF);
  BM_CLR(SPI_CTRL_REG, SPI_CTRL_RW_STA);

  SPI_DATA_REG = ch;

  SPI_WAIT_BUSY(t, 100);

  TLSR82_SPI_CS_HIGH;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_console_init
 ****************************************************************************/

/****************************************************************************
 * Name: spi_console_early_init
 *
 * Description:
 *   SPI early init, after this, the spi can be used for syslog before the
 *   console register, used for debug.
 *
 * Parameters:
 *   void
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

void spi_console_early_init(void)
{
  /* SPI I2C multiplex config:
   * 1. Enable the SPI and I2C for GroupA and GroupD;
   * 2. Enable the PD7 SPI function;
   * 3. Disable the PD7 I2C function;
   */

  BM_SET(SPI_I2C_GROUP_REG, SPI_I2C_GROUPA_EN);
  BM_SET(SPI_I2C_GROUP_REG, SPI_I2C_GROUPD_EN);
  BM_SET(SPI_I2C_PIN_REG, SPI_PD7_SPI_EN);
  BM_CLR(SPI_I2C_PIN_REG, I2C_PD7_I2C_EN);

  /* GPIO config, the miso pin config can be removed */

  tlsr82_gpioconfig(g_console_dev.pin_miso);
  tlsr82_gpioconfig(g_console_dev.pin_mosi);
  tlsr82_gpioconfig(g_console_dev.pin_sck);
  tlsr82_gpioconfig(g_console_dev.pin_cs);

  /* Pull-up the cs pin */

  SPI_CS_HIGH;

  /* Enable SPI clock source; */

  BM_SET(CLK_EN1_REG, CLK_EN1_SPI);

  /* Config the SPI clkdiv = 2,
   * SpiClosk = SystemClock / ((clkdiv + 1) * 2)
   *          = 4MHz
   */

  SPI_CLK_REG = 0;
  BM_SET(SPI_CLK_REG, 2);

  /* Enable SPI clock and config SPI as master; */

  BM_SET(SPI_CLK_REG, SPI_CLK_EN);
  BM_SET(SPI_CTRL_REG, SPI_CTRL_MASTER_EN);

  /* CPOL = 0, CPHA = 0 */

  SPI_MODE_REG = SPI_MODE_REG & (~0x3);
}

/****************************************************************************
 * Name: spi_console_init
 *
 * Description:
 *   SPI as the console, only support write, the printf() will output by
 *   spi, used for debug.
 *
 * Parameters:
 *   void
 *
 * Returned Values:
 *   void
 *
 ****************************************************************************/

void spi_console_init(void)
{
  int ret;

  /* Driver register */

  ret = register_driver("/dev/console", &g_consoleops, 0666, NULL);
  if (ret < 0)
    {
      syslog(LOG_DEBUG, "register spi_console failed\n");
    }
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes. When the SPI is used for syslog ouput, the up_putc() should be
 *   implemented by spi.
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_SPI_SYSLOG
int up_putc(int ch)
{
  if (ch == '\n')
    {
      /* Add CR */

      spi_putc('\r');
    }

  spi_putc((uint8_t)ch);

  return 0;
}
#endif
