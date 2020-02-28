/****************************************************************************
 * boards/arm/stm32/stm32f429i-disco/src/stm32_ili93414ws.c
 *
 * Driver for the ILI9341 Single Chip LCD driver connected
 * via 4 wire serial (spi) mcu interface
 *
 *   Copyright (C) 2014 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
 *
 * References: ILI9341_DS_V1.10.pdf (Rev: 1.10), "a-Si TFT LCD Single Chip
 *             Driver 240RGBx320 Resolution and 262K color", ILI TECHNOLOGY
 *             CORP., http://www.ilitek.com.
 *             ILI TECHNOLOGY CORP., http://www.ilitek.com.
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "stm32_rcc.h"
#include "stm32_spi.h"
#include "stm32f429i-disco.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Display is connected at spi5 device */

#define ILI93414WS_SPI_DEVICE       5

/* spi frequency based on arch/arm/src/stm32/stm32_spi.c */

#ifndef CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY
#  define CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY      20000000
#endif

#if CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY >= \
                                        (STM32_PCLK1_FREQUENCY >> 1)
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd2    /* 000: fPCLK/2 */
#  define ILI93414WS_BAUD_DIVISOR   2
#elif CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY >= \
                                        (STM32_PCLK1_FREQUENCY >> 2)
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd4    /* 001: fPCLK/4 */
#  define ILI93414WS_BAUD_DIVISOR   4
#elif CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY >= \
                                        (STM32_PCLK1_FREQUENCY >> 3)
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd8    /* 010: fPCLK/8 */
#  define ILI93414WS_BAUD_DIVISOR   8
#elif CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY >= \
                                        (STM32_PCLK1_FREQUENCY >> 4)
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd16   /* 011: fPCLK/16 */
#  define ILI93414WS_BAUD_DIVISOR   16
#elif CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY >= \
                                        (STM32_PCLK1_FREQUENCY >> 5)
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd32   /* 100: fPCLK/32 */
#  define ILI93414WS_BAUD_DIVISOR   32
#elif CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY >= \
                                        (STM32_PCLK1_FREQUENCY >> 6)
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd64   /* 101: fPCLK/64 */
#  define ILI93414WS_BAUD_DIVISOR   64
#elif CONFIG_STM32F429I_DISCO_ILI9341_SPIFREQUENCY >= \
                                        (STM32_PCLK1_FREQUENCY >> 7)
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd128  /* 110: fPCLK/128 */
#  define ILI93414WS_BAUD_DIVISOR   128
#else
#  define ILI93414WS_SPI_BR         SPI_CR1_FPCLCKd256  /* 111: fPCLK/256 */
#  define ILI93414WS_BAUD_DIVISOR   256
#endif

/*
 * Permitted clock delay for a pixel transmission from the LCD gram.
 * Calculated by cpu clock / (spi clock / baud divisor)
 */

#define ILI93414WS_RECV_CLK         (STM32_SYSCLK_FREQUENCY / \
                                    (STM32_PCLK1_FREQUENCY / \
                                    ILI93414WS_BAUD_DIVISOR))

/* Definition of the spi mcu register */

#define ILI93414WS_SPI_BASE         STM32_SPI5_BASE
#define ILI93414WS_SPI_CR1          (ILI93414WS_SPI_BASE + STM32_SPI_CR1_OFFSET)
#define ILI93414WS_SPI_CR2          (ILI93414WS_SPI_BASE + STM32_SPI_CR2_OFFSET)
#define ILI93414WS_SPI_SR           (ILI93414WS_SPI_BASE + STM32_SPI_SR_OFFSET)
#define ILI93414WS_SPI_DR           (ILI93414WS_SPI_BASE + STM32_SPI_DR_OFFSET)

/* Activates the usage of the spi interface structure if several active devices
 * connected on the SPI5 bus, e.g. LCD Display, MEMS. This will perform locking
 * of the spi bus by SPI_LOCK at each selection of the SPI5 device.
 */

#ifdef CONFIG_STM32_SPI5
#  define ILI93414WS_SPI
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

struct ili93414ws_lcd_s
{
  /* Publicly visible device structure */

  struct ili9341_lcd_s dev;

#ifdef ILI93414WS_SPI
  /* Reference to spi device structure */

  FAR struct spi_dev_s *spi;

  /* Backup cr1 register at selection */

  uint16_t cr1;

  /* Backup cr2 register at selection */

  uint16_t cr2;
#endif

#ifndef CONFIG_STM32F429I_DISCO_ILI9341_SPIBITS16
  /* Marks current display operation mode (gram or command/parameter)
   * If 16-bit spi mode is enabled for pixel data operation, the flag is not
   * necessary. The pixel data operation mode can then be recognized by the
   * DFF flag in the cr1 register.
   */

  uint8_t gmode;
#endif
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Low-level spi transfer */

static void stm32_ili93414ws_modifyreg(uint32_t reg, uint16_t setbits,
              uint16_t clrbits);
static inline void stm32_ili93414ws_modifycr1(uint16_t setbits,
              uint16_t clrbits);
static inline void stm32_ili93414ws_modifycr2(uint16_t setbits,
              uint16_t clrbits);
static void stm32_ili93414ws_spisendmode(void);
static void stm32_ili93414ws_spirecvmode(void);
static void stm32_ili93414ws_spienable(void);
static void stm32_ili93414ws_spidisable(void);

static inline void stm32_ili93414ws_set8bitmode(
              FAR struct ili93414ws_lcd_s *dev);
static inline void stm32_ili93414ws_set16bitmode(
              FAR struct ili93414ws_lcd_s *dev);

/* Command and data transmission control */

static void stm32_ili93414ws_sndword(uint16_t wd);
static int stm32_ili93414ws_sendblock(FAR struct ili93414ws_lcd_s *lcd,
              const uint16_t *wd, uint16_t nwords);
static uint16_t stm32_ili93414ws_recvword(void);
static int stm32_ili93414ws_recvblock(FAR struct ili93414ws_lcd_s *lcd,
              uint16_t *wd, uint16_t nwords);
static inline void stm32_ili93414ws_cmddata(
              FAR struct ili9341_lcd_s *lcd, bool cmd);

/* Initializing / Configuration */

static void stm32_ili93414ws_spiconfig(FAR struct ili9341_lcd_s *lcd);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct ili93414ws_lcd_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ili93414ws_modifyreg
 *
 * Description:
 *   Clear and set bits in the CR register (based on
 *   arch/arm/src/stm32/stm32_spi.c).
 *
 * Input Parameters:
 *   reg      - register to set
 *   clrbits  - The bits to clear
 *   setbits  - The bits to set
 *
 * Returned Value:
 *
 ****************************************************************************/

static void stm32_ili93414ws_modifyreg(uint32_t reg, uint16_t setbits,
                                       uint16_t clrbits)
{
  uint16_t regval;

  regval  = getreg16(reg);
  regval &= ~clrbits;
  regval |= setbits;
  putreg16(regval, reg);
}

/****************************************************************************
 * Name: stm32_ili93414ws_modifycr1
 *
 * Description:
 *   Clear and set bits in the CR1 register.
 *
 * Input Parameters:
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_ili93414ws_modifycr1(uint16_t setbits,
                                              uint16_t clrbits)
{
  stm32_ili93414ws_modifyreg(ILI93414WS_SPI_CR1, setbits, clrbits);
}

/****************************************************************************
 * Name: stm32_ili93414ws_modifycr2
 *
 * Description:
 *   Clear and set bits in the CR2 register.
 *
 * Input Parameters:
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_ili93414ws_modifycr2(uint16_t setbits,
                                              uint16_t clrbits)
{
  stm32_ili93414ws_modifyreg(ILI93414WS_SPI_CR2, setbits, clrbits);
}

/****************************************************************************
 * Name: stm32_ili93414ws_spirecvmode
 *
 * Description:
 *   Sets the spi device to the bidirectional receive mode
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void stm32_ili93414ws_spirecvmode(void)
{
  /* Set to bidirectional rxonly mode */

  stm32_ili93414ws_modifycr1(0, SPI_CR1_BIDIOE);

  /* Disable spi */

  stm32_ili93414ws_spidisable();

  /* Clear the rx buffer if received data exist e.g. from previous
   * broken transfer.
   */

  getreg16(ILI93414WS_SPI_DR);
}

/****************************************************************************
 * Name: stm32_ili93414ws_spisendmode
 *
 * Description:
 *   Sets the spi device to the bidirectional transmit mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_ili93414ws_spisendmode(void)
{
  /* Set to bidirectional transmit mode */

  stm32_ili93414ws_modifycr1(SPI_CR1_BIDIOE, 0);

  /* enable spi */

  stm32_ili93414ws_spienable();
}

/****************************************************************************
 * Name: stm32_ili93414ws_spienable
 *
 * Description:
 *   Enable the spi device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_ili93414ws_spienable(void)
{
  uint16_t  regval;

  regval = getreg16(ILI93414WS_SPI_CR1);
  regval |= SPI_CR1_SPE;
  putreg16(regval, ILI93414WS_SPI_CR1);
}

/****************************************************************************
 * Name: stm32_ili93414ws_spidisable
 *
 * Description:
 *   Disable the spi device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_ili93414ws_spidisable(void)
{
  uint16_t  regval;

  regval = getreg16(ILI93414WS_SPI_CR1);
  regval &= ~SPI_CR1_SPE;
  putreg16(regval, ILI93414WS_SPI_CR1);
}

/****************************************************************************
 * Name: stm32_ili93414ws_sndword
 *
 * Description:
 *   Send a word to the lcd driver.
 *
 * Input Parameters:
 *   wd  - word to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_ili93414ws_sndword(uint16_t wd)
{
  /* Send the word */

  putreg16(wd, ILI93414WS_SPI_DR);

  /* Wait until the transmit buffer is empty */

  while ((getreg16(ILI93414WS_SPI_SR) & SPI_SR_TXE) == 0);
}

/****************************************************************************
 * Name: stm32_ili93414ws_sendblock
 *
 * Description:
 *   Send a number of words to the lcd driver.
 *
 * Input Parameters:
 *   spi    - Reference to the private device structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int stm32_ili93414ws_sendblock(FAR struct ili93414ws_lcd_s *lcd,
                                      const uint16_t *wd, uint16_t nwords)
{
  /* Set to bidirectional transmit mode and enable spi */

  stm32_ili93414ws_spisendmode();

  /* Check if 16-bit spi mode is configured for transmit */

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_SPIBITS16
  if ((getreg16(ILI93414WS_SPI_CR1) & SPI_CR1_DFF) != 0)
    {
      /* 16-bit spi mode */

      const uint16_t *src  = wd;
            uint16_t word;

      while (nwords-- > 0)
        {
          word = *src++;
          stm32_ili93414ws_sndword(word);
        }
    }
#else
  /* 8-bit spi mode is enabled for pixel data operations.
   * Each pixel must be transmitted by two write operations.
   */

  if (lcd->gmode == 16)
    {
      /* 8-bit spi mode */

      const uint16_t *src  = wd;
            uint16_t word;

      while (nwords-- > 0)
        {
          word = *src++;
          stm32_ili93414ws_sndword((word >> 8));
          stm32_ili93414ws_sndword((word & 0xff));
        }
    }
#endif
  else
    {
      /* 8-bit spi mode */

      const uint8_t *src  = (const uint8_t*)wd;
            uint8_t word;

      while (nwords-- > 0)
        {
          word = *src++;
          stm32_ili93414ws_sndword((uint16_t)word);
        }

    }

  /* Wait until transmit is not busy after the last word is transmitted, marked
   * by the BSY flag in the cr1 register. This is necessary if entering in halt
   * mode or disable the spi periphery.
   */

  while ((getreg16(ILI93414WS_SPI_SR) & SPI_SR_BSY) != 0);

  /* Disable spi */

  stm32_ili93414ws_spidisable();

  return OK;
}

/****************************************************************************
 * Name: stm32_ili93414ws_recvword
 *
 * Description:
 *   Receive a word from the lcd driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success - The received word from the LCD Single Chip Driver.
 *   On error   - 0 (If timeout during receiving)
 *
 ****************************************************************************/

static uint16_t stm32_ili93414ws_recvword(void)
{
  volatile int       n;
  volatile uint16_t  regval;
         irqstate_t  flags;

  /* Disable interrupts during time critical spi sequence.
   * In bidrectional receive mode the data transfer can only be stopped by
   * disabling the spi device. This is here done by disabling the spi device
   * immediately after enabling it. If the pixel data stream is interrupted
   * during receiving, a synchronized transfer can not ensure. Especially on
   * higher frequency it can happen that the interrupted driver isn't fast
   * enough to stop transmitting by disabling the spi device. So pixels lost but
   * not recognized by the driver. This results in a big lock because the driver
   * wants to receive missing pixel data.
   * The critical section here ensures that the spi device is disabled fast
   * enough during a pixel is transmitted.
   */

  flags = enter_critical_section();

  /* Backup the content of the current cr1 register only 1 times */

  regval = getreg16(ILI93414WS_SPI_CR1);

  /* Enable spi device followed by disable the spi device.
   *
   * Ensure that the spi is disabled within 8 or 16 spi clock cycles depending
   * on the configured spi bit mode. This is necessary to prevent that the next
   * data word is transmitted by the slave before the RX buffer is cleared.
   * Otherwise the RX buffer will be overwritten.
   *
   * Physically the spi clock is disabled after the current 8/16 clock cycles
   * are completed.
   */

  regval |= SPI_CR1_SPE;
  putreg16(regval, ILI93414WS_SPI_CR1);

  /* Disable spi device */

  regval &= ~SPI_CR1_SPE;
  putreg16(regval, ILI93414WS_SPI_CR1);

  /* The spi device is in disabled state so it is safe to enable interrupts */

  leave_critical_section(flags);

  /* Waits until the RX buffer is filled with the received data word signalized
   * by the spi hardware through the RXNE flag.
   * A busy loop is preferred against interrupt driven receiving method here
   * because this happened fairly often. Also we have to ensure to avoid a big
   * lock if the lcd driver doesn't send data anymore.
   * A latency of CPU clock / SPI clock * 16 SPI clocks should be enough here.
   */

  for (n = 0; n < ILI93414WS_RECV_CLK * 16; n++)
    {
      if ((getreg16(ILI93414WS_SPI_SR) & SPI_SR_RXNE) != 0)
        {
          /* Receive the data word from the RX buffer */

          return getreg16(ILI93414WS_SPI_DR);
        }
    }

  lcdinfo("Timeout during receiving pixel word\n");

  return 0;
}

/****************************************************************************
 * Name: stm32_ili93414ws_recvblock
 *
 * Description:
 *   Receive a number of words from to the lcd driver.
 *   Note: The first received word is the dummy word and discarded!
 *
 * Input Parameters:
 *   spi    - Reference to the private device structure
 *   wd    -  Reference to where the words receive
 *   nwords - number of words to receive
 *
 * Returned Value:
 *  OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93414ws_recvblock(FAR struct ili93414ws_lcd_s *lcd,
                                uint16_t *wd, uint16_t nwords)
{
  /* ili9341 uses a 18-bit pixel format packed in a 24-bit stream per pixel.
   * The following format is transmitted: RRRRRR00 GGGGGG00 BBBBBB00
   * Convert it to:                       RRRRRGGG GGGBBBBB
   */

  /* Set to bidirectional transmit mode and disable spi */

  stm32_ili93414ws_spirecvmode();

  /* Check if 16-bit spi mode is configured for receive */

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_SPIBITS16
  /* Two contiguous pixel must be received by three read operations. */

  if ((getreg16(ILI93414WS_SPI_CR1) & SPI_CR1_DFF) != 0)
    {
      /* 16-bit mode */

      uint16_t *dest  = wd;
      uint16_t w1;
      uint16_t w2;

      /* Receive first pixel */

      if (nwords)
        {
          /* Discard the first 8 bit dummy */

          /* 00000000 RRRRRR00 */
          w1 = stm32_ili93414ws_recvword();

          /* GGGGGG00 BBBBBB00 */
          w2 = stm32_ili93414ws_recvword();

          *dest++ = (((w1 << 8) & 0xf800) |
                     ((w2 >> 2) & 0x7e0) |
                     ((w2 >> 3) & 0x1f));

          --nwords;
        }

      /* Receive
       * if nwords even and greater than 2: pixel 2 to n-1
       * if nwords odd and greater than 2:  pixel 2 to n
       */

      while (nwords--)
        {
          /* RRRRRR00 GGGGGG00 */

          w1 = stm32_ili93414ws_recvword();

          /* BBBBBB00 RRRRRR00 */

          w2 = stm32_ili93414ws_recvword();

          *dest++ = ((w1 & 0xf800) | ((w1 << 3) & 0x7e0) | (w2 >> 11));

          if (!nwords)
            {
              break;
            }

          /* GGGGGG00 BBBBBB00 */

          w1 = stm32_ili93414ws_recvword();

          *dest++ = (((w1 >> 5) & 0x7e0) |
                     ((w1 >> 3) & 0x1f) |
                     ((w2 << 8) & 0xf800));

          --nwords;
        }
    }
#else
  /* 8-bit spi mode is enabled for pixel data operations.
   * Each pixel must be received by three read operations.
   */

  if (lcd->gmode == 16)
    {
      /* 8-bit spi mode but 16-bit mode is done by two 8-bit transmits */

      uint16_t *dest = wd;

      /* Dummy read to discard the first 8 bit. */

      stm32_ili93414ws_recvword();

      while (nwords--)
        {
          uint8_t r, g, b;
          r = (uint8_t)(stm32_ili93414ws_recvword() >> 3);
          g = (uint8_t)(stm32_ili93414ws_recvword() >> 2);
          b = (uint8_t)(stm32_ili93414ws_recvword() >> 3);
          *dest++ = ((r << 11) | (g << 5) | b);
        }
    }
#endif
  else
    {
      /* 8-bit mode */

      uint8_t *dest  = (uint8_t*)wd;

      while (nwords--)
        {
          *dest++ = (uint8_t)stm32_ili93414ws_recvword();
        }
    }

  /* Disable spi device is done by recvword  */

  return OK;
}

/****************************************************************************
 * Name: stm32_ili93414ws_set8bitmode
 *
 * Description:
 *   Set spi device to 8-bit data format
 *
 * Input Parameters:
 *   dev  - Reference to the private driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_ili93414ws_set8bitmode(
                      FAR struct ili93414ws_lcd_s *dev)
{
  stm32_ili93414ws_modifycr1(0, SPI_CR1_DFF);
#ifndef CONFIG_STM32F429I_DISCO_ILI9341_SPIBITS16
  dev->gmode = 8;
#endif
}

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_SPIBITS16
/****************************************************************************
 * Name: stm32_ili93414ws_set16bitmode
 *
 * Description:
 *   Set spi device to 16-bit data format.
 *
 * Input Parameters:
 *   dev  - Reference to the private driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_ili93414ws_set16bitmode(
                      FAR struct ili93414ws_lcd_s *dev)
{
  stm32_ili93414ws_modifycr1(SPI_CR1_DFF, 0);
}
#else

static inline void stm32_ili93414ws_set16bitmode(
                      FAR struct ili93414ws_lcd_s *dev)
{
  dev->gmode = 16;
}
#endif

/****************************************************************************
 * Name: stm32_ili93414ws_spiconfig
 *
 * Description:
 *   Disable spi device and configure to bidirectional mode.
 *
 * Input Parameters:
 *   lcd  - Reference to the private driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_ili93414ws_spiconfig(FAR struct ili9341_lcd_s *lcd)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;
  irqstate_t   flags;

  uint16_t   clrbitscr1 = SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_BR_MASK|
                          SPI_CR1_CRCEN|SPI_CR1_LSBFIRST|SPI_CR1_RXONLY|
                          SPI_CR1_DFF;

  uint16_t   setbitscr1 = SPI_CR1_BIDIOE|SPI_CR1_BIDIMODE|SPI_CR1_MSTR|
                          SPI_CR1_SSI|SPI_CR1_SSM|ILI93414WS_SPI_BR;

  uint16_t   clrbitscr2 = SPI_CR2_TXEIE|SPI_CR2_RXNEIE|SPI_CR2_ERRIE|
                          SPI_CR2_FRF|SPI_CR2_SSOE;

  uint16_t   setbitscr2 = 0;


  flags = enter_critical_section();

  /* Disable spi */

  stm32_ili93414ws_spidisable();

  /* Set to default 8-bit transfer mode */

  stm32_ili93414ws_set8bitmode(priv);

#ifdef ILI93414WS_SPI
  /* Backup cr1 and cr2 register to be sure they will be usable
   * by default spi interface. Disable spi device here is necessary at the time
   * restoring the register during deselection.
   */

  priv->cr2 = getreg16(ILI93414WS_SPI_CR2);
  priv->cr1 = getreg16(ILI93414WS_SPI_CR1);
#endif

  /* Set spi device to bidirectional half duplex
   * Configure to master with 8-bit data format and SPIDEV_MODE0
   */

  stm32_ili93414ws_modifycr1(setbitscr1, clrbitscr1);

  /* Disable dma, set to motorola spi. */

  stm32_ili93414ws_modifycr2(setbitscr2, clrbitscr2);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32_ili93414ws_cmddata
 *
 * Description:
 *   Select command or data transfer mode
 *
 * Input Parameters:
 *   lcd  - Reference to the private driver structure
 *   cmd  - Refers to command or parameter
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef ILI93414WS_SPI
static inline void stm32_ili93414ws_cmddata(
                      FAR struct ili9341_lcd_s *lcd, bool cmd)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), cmd);
}
#else
static inline void stm32_ili93414ws_cmddata(
                      FAR struct ili9341_lcd_s *lcd, bool cmd)
{
  stm32_gpiowrite(GPIO_LCD_DC, !cmd);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ili93414ws_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *
 * Input Parameters:
 *   spi   - Reference to the public driver structure
 *   level - backligth level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93414ws_backlight(FAR struct ili9341_lcd_s *lcd, int level)
{
  return OK;
}

/****************************************************************************
 * Name: stm32_ili93414ws_select
 *
 * Description:
 *   Select the SPI, locking and re-configuring if necessary
 *
 * Input Parameters:
 *   spi  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef ILI93414WS_SPI
static void stm32_ili93414ws_select(FAR struct ili9341_lcd_s *lcd)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  /* Select ili9341 (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(priv->spi, true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);

  /* Configure spi and disable */

  stm32_ili93414ws_spiconfig(lcd);
}
#else
static void stm32_ili93414ws_select(FAR struct ili9341_lcd_s *lcd)
{
  /* We own the spi bus, so just select the chip */

  stm32_gpiowrite(GPIO_CS_LCD, false);

  /* Disable spi */

  stm32_ili93414ws_spidisable();
}
#endif

/****************************************************************************
 * Name: stm32_ili93414ws_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef ILI93414WS_SPI
static void stm32_ili93414ws_deselect(FAR struct ili9341_lcd_s *lcd)
{
  irqstate_t   flags;
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  flags = enter_critical_section();

  /* Restore cr1 and cr2 register to be sure they will be usable
   * by default spi interface structure. (This is an important workaround as
   * long as half duplex mode is not supported by the spi interface in
   * arch/arm/src/stm32/stm32_spi.c).
   */

  putreg16(priv->cr2, ILI93414WS_SPI_CR2);
  putreg16(priv->cr1, ILI93414WS_SPI_CR1);

  /* Enable spi device is default for initialized spi ports (see
   * arch/arm/src/stm32/stm32_spi.c).
   */

  stm32_ili93414ws_spienable();

  leave_critical_section(flags);

  /* de-select ili9341 and relinquish the spi bus. */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi, false);
}
#else
static void stm32_ili93414ws_deselect(FAR struct ili9341_lcd_s *lcd)
{
  stm32_gpiowrite(GPIO_CS_LCD, true);
}
#endif

/****************************************************************************
 * Name: stm32_ili93414ws_sndcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the ili9341_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int stm32_ili93414ws_sendcmd(
              FAR struct ili9341_lcd_s *lcd, const uint8_t cmd)
{
  int ret;
  const uint16_t bw = (const uint16_t)cmd;
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  /* Set to 8-bit mode in disabled state, spi device is in disabled state */

  stm32_ili93414ws_set8bitmode(priv);

  lcdinfo("cmd=%04x\n", bw);
  stm32_ili93414ws_cmddata(lcd, true);
  ret = stm32_ili93414ws_sendblock(priv, &bw, 1);
  stm32_ili93414ws_cmddata(lcd, false);

  return ret;
}

/****************************************************************************
 * Name: stm32_ili93414ws_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93414ws_sendparam(FAR struct ili9341_lcd_s *lcd,
                                    const uint8_t param)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;
  const uint16_t bw = (const uint16_t)param;

  /* Set to 8-bit mode in disabled state, spi device is in disabled state */

  stm32_ili93414ws_set8bitmode(priv);

  lcdinfo("param=%04x\n", bw);
  return stm32_ili93414ws_sendblock(priv, &bw, 1);
}

/****************************************************************************
 * Name: stm32_ili93414ws_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93414ws_sendgram(FAR struct ili9341_lcd_s *lcd,
                          const uint16_t *wd, uint32_t nwords)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  lcdinfo("wd=%p, nwords=%d\n", wd, nwords);

  /* Set to 16-bit mode transfer mode, spi device is in disabled state */

  stm32_ili93414ws_set16bitmode(priv);

  return stm32_ili93414ws_sendblock(priv, wd, nwords);
};

/****************************************************************************
 * Name: stm32_ili93414ws_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Reference to where parameter receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93414ws_recvparam(FAR struct ili9341_lcd_s *lcd,
                                        uint8_t *param)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_SPIBITS16
  /* Set to 8-bit mode in disabled state, spi device is in disabled state. */

  stm32_ili93414ws_set8bitmode(priv);
#endif

  lcdinfo("param=%04x\n", param);
  return stm32_ili93414ws_recvblock(priv, (uint16_t*)param, 1);
}

/****************************************************************************
 * Name: stm32_ili93414ws_recvgram
 *
 * Description:
 *   Receive pixel words from the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the public driver structure
 *   wd     - Reference to where the pixel words receive
 *   nwords - number of pixel words to receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93414ws_recvgram(FAR struct ili9341_lcd_s *lcd,
                                    uint16_t *wd, uint32_t nwords)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  lcdinfo("wd=%p, nwords=%d\n", wd, nwords);

  /* Set to 16-bit mode in disabled state */

  stm32_ili93414ws_set16bitmode(priv);

  return stm32_ili93414ws_recvblock(priv, wd, nwords);
};

/****************************************************************************
 * Name:  stm32_ili93414ws_initialize
 *
 * Description:
 *   Initialize the device structure to control the LCD Single chip driver.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD control object
 *   for the specified ILI9341 LCD Single chip driver connected as 4 wire serial
 *   (spi). NULL is returned on any failure.
 *
 ****************************************************************************/

#ifdef ILI93414WS_SPI
FAR struct ili9341_lcd_s *stm32_ili93414ws_initialize(void)
{
  FAR struct spi_dev_s *spi;
  FAR struct ili93414ws_lcd_s *priv = &g_lcddev;

  lcdinfo("initialize ili9341 4-wire serial subdriver\n");

  lcdinfo("initialize spi device: %d\n", ILI93414WS_SPI_DEVICE);
  spi = stm32_spi5initialize();

  if (spi)
    {
      /* Initialize structure */

      priv->dev.select      = stm32_ili93414ws_select;
      priv->dev.deselect    = stm32_ili93414ws_deselect;
      priv->dev.sendcmd     = stm32_ili93414ws_sendcmd;
      priv->dev.sendparam   = stm32_ili93414ws_sendparam;
      priv->dev.recvparam   = stm32_ili93414ws_recvparam;
      priv->dev.sendgram    = stm32_ili93414ws_sendgram;
      priv->dev.recvgram    = stm32_ili93414ws_recvgram;
      priv->dev.backlight   = stm32_ili93414ws_backlight;
      priv->spi             = spi;

      return &priv->dev;
    }

  return NULL;
}

#else

FAR struct ili9341_lcd_s *stm32_ili93414ws_initialize(void)
{
  uint32_t    regval;
  FAR struct ili93414ws_lcd_s *priv = &g_lcddev;

  lcdinfo("initialize ili9341 4-wire serial subdriver\n");

  /* Enable spi bus */

  regval= getreg32(STM32_RCC_APB2ENR);
  regval |= RCC_APB2ENR_SPI5EN;
  putreg32(regval, STM32_RCC_APB2ENR);

  /* Configure gpios */

  stm32_configgpio(GPIO_CS_LCD);     /* LCD chip select */
  stm32_configgpio(GPIO_LCD_DC);     /* LCD Data/Command select */
  stm32_configgpio(GPIO_LCD_ENABLE); /* LCD enable select */
  stm32_configgpio(GPIO_SPI5_SCK);   /* SPI clock */
  stm32_configgpio(GPIO_SPI5_MOSI);  /* SPI MOSI */

  /* Initialize structure */

  priv->dev.select      = stm32_ili93414ws_select;
  priv->dev.deselect    = stm32_ili93414ws_deselect;
  priv->dev.sendcmd     = stm32_ili93414ws_sendcmd;
  priv->dev.sendparam   = stm32_ili93414ws_sendparam;
  priv->dev.recvparam   = stm32_ili93414ws_recvparam;
  priv->dev.sendgram    = stm32_ili93414ws_sendgram;
  priv->dev.recvgram    = stm32_ili93414ws_recvgram;
  priv->dev.backlight   = stm32_ili93414ws_backlight;


  /* Configure to bidirectional transfer mode */

  lcdinfo("Configure spi device %d to bidirectional transfer mode\n",
            ILI93414WS_SPI_DEVICE);

  stm32_ili93414ws_spiconfig(&priv->dev);

  return &priv->dev;
}
#endif
