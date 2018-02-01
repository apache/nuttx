/****************************************************************************
 * arch/arm/src/samv7/sam_spi_slave.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/spi/slave.h>

#include "up_arch.h"

#include "sam_config.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "sam_spi.h"

#include "chip/sam_spi.h"
#include "chip/sam_pinmap.h"
#include <arch/board/board.h>

#ifdef CONFIG_SAMV7_SPI_SLAVE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SAMV7_SPI_SLAVE_QSIZE
#  define CONFIG_SAMV7_SPI_SLAVE_QSIZE 8
#endif

#ifndef CONFIG_DEBUG_SPI_INFO
#  undef CONFIG_SAMV7_SPI_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The overall state of one SPI controller */

struct sam_spidev_s
{
  struct spi_sctrlr_s sctrlr;  /* Externally visible part of the SPI slave
                                * controller interface */
  struct spi_sdev_s *sdev;     /* Bound SPI slave device interface */
  xcpt_t handler;              /* SPI interrupt handler */
  uint32_t base;               /* SPI controller register base address */
  sem_t spisem;                /* Assures mutually exclusive access to SPI */
  uint16_t outval;             /* Default shift-out value */
  uint16_t irq;                /* SPI IRQ number */
  uint8_t mode;                /* Mode 0,1,2,3 */
  uint8_t nbits;               /* Width of word in bits (8 to 16) */
  uint8_t spino;               /* SPI controller number (0 or 1) */
  bool initialized;            /* True: Controller has been initialized */
  bool nss;                    /* True: Chip selected */

  /* Output queue */

  uint8_t head;                /* Location of next value */
  uint8_t tail;                /* Index of first value */

  uint16_t outq[CONFIG_SAMV7_SPI_SLAVE_QSIZE];

  /* Debug stuff */

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
   bool     wrlast;            /* Last was a write */
   uint32_t addresslast;       /* Last address */
   uint32_t valuelast;         /* Last value */
   int      ntimes;            /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
static bool     spi_checkreg(struct sam_spidev_s *priv, bool wr,
                  uint32_t value, uint32_t address);
#else
# define        spi_checkreg(priv,wr,value,address) (false)
#endif

static uint32_t spi_getreg(struct sam_spidev_s *priv,
                  unsigned int offset);
static void     spi_putreg(struct sam_spidev_s *priv, uint32_t value,
                  unsigned int offset);

#ifdef CONFIG_DEBUG_SPI_INFO
static void     spi_dumpregs(struct sam_spidev_s *priv, const char *msg);
#else
# define        spi_dumpregs(priv,msg)
#endif

static void     spi_semtake(struct sam_spidev_s *priv);
#define         spi_semgive(priv) (nxsem_post(&(priv)->spisem))

/* Interrupt Handling */

static int      spi_interrupt(int irq, void *context, FAR void *arg);

/* SPI Helpers */

static uint16_t spi_dequeue(struct sam_spidev_s *priv);
static void     spi_setmode(struct sam_spidev_s *priv,
                 enum spi_smode_e mode);
static void     spi_setbits(struct sam_spidev_s *priv,
                  int nbits);

/* SPI slave controller methods */

static void     spi_bind(struct spi_sctrlr_s *sctrlr,
                  struct spi_sdev_s *sdev, enum spi_smode_e mode,
                  int nbits);
static void     spi_unbind(struct spi_sctrlr_s *sctrlr);
static int      spi_enqueue(struct spi_sctrlr_s *sctrlr, uint16_t data);
static bool     spi_qfull(struct spi_sctrlr_s *sctrlr);
static void     spi_qflush(struct spi_sctrlr_s *sctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps chip select numbers (0-3) to CSR register offsets */

static const uint8_t g_csroffset[4] =
{
  SAM_SPI_CSR0_OFFSET, SAM_SPI_CSR1_OFFSET,
  SAM_SPI_CSR2_OFFSET, SAM_SPI_CSR3_OFFSET
};

/* SPI slave controller driver operations */

static const struct spi_sctrlrops_s g_sctrlr_ops =
{
  .bind              = spi_bind,
  .unbind            = spi_unbind,
  .enqueue           = spi_enqueue,
  .qfull             = spi_qfull,
  .qflush            = spi_qflush,
};

#ifdef CONFIG_SAMV7_SPI0_SLAVE
/* This is the overall state of the SPI0 controller */

static struct sam_spidev_s g_spi0_sctrlr;
#endif

#ifdef CONFIG_SAMV7_SPI1_SLAVE
/* This is the overall state of the SPI0 controller */

static struct sam_spidev_s g_spi1_sctrlr;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
static bool spi_checkreg(struct sam_spidev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      value   == priv->valuelast &&  /* Same value? */
      address == priv->addresslast)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          spiinfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast      = wr;
      priv->valuelast   = value;
      priv->addresslast = address;
      priv->ntimes      = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

static uint32_t spi_getreg(struct sam_spidev_s *priv, unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
  if (spi_checkreg(priv, false, value, address))
    {
      spiinfo("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static void spi_putreg(struct sam_spidev_s *priv, uint32_t value,
                       unsigned int offset)
{
  uint32_t address = priv->base + offset;

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
  if (spi_checkreg(priv, true, value, address))
    {
      spiinfo("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * Name: spi_dumpregs
 *
 * Description:
 *   Dump the contents of all SPI registers
 *
 * Input Parameters:
 *   priv - The SPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
static void spi_dumpregs(struct sam_spidev_s *priv, const char *msg)
{
  spiinfo("%s:\n", msg);
  spiinfo("    MR:%08x   SR:%08x  IMR:%08x\n",
          getreg32(priv->base + SAM_SPI_MR_OFFSET),
          getreg32(priv->base + SAM_SPI_SR_OFFSET),
          getreg32(priv->base + SAM_SPI_IMR_OFFSET));
  spiinfo("  CSR0:%08x CSR1:%08x CSR2:%08x CSR3:%08x\n",
          getreg32(priv->base + SAM_SPI_CSR0_OFFSET),
          getreg32(priv->base + SAM_SPI_CSR1_OFFSET),
          getreg32(priv->base + SAM_SPI_CSR2_OFFSET),
          getreg32(priv->base + SAM_SPI_CSR3_OFFSET));
  spiinfo("  WPCR:%08x WPSR:%08x\n",
          getreg32(priv->base + SAM_SPI_WPCR_OFFSET),
          getreg32(priv->base + SAM_SPI_WPSR_OFFSET));
}
#endif

/****************************************************************************
 * Name: spi_semtake
 *
 * Description:
 *   Take the semaphore that enforces mutually exclusive access to SPI
 *   resources, handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the MCAN peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void spi_semtake(struct sam_spidev_s *priv)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      ret = nxsem_wait(&priv->spisem);
      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Name: spi_interrupt
 *
 * Description:
 *   Common SPI interrupt handler
 *
 * Input Parameters:
 *   priv - SPI controller CS state
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int spi_interrupt(int irq, void *context, FAR void *arg)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)arg;
  uint32_t sr;
  uint32_t imr;
  uint32_t pending;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* We loop because the TDRE interrupt will probably immediately follow the
   * RDRF interrupt and we might be able to catch it in this handler
   * execution.
   */

  for (; ; )
    {
      /* Get the current set of pending/enabled interrupts */

      sr      = spi_getreg(priv, SAM_SPI_SR_OFFSET);
      imr     = spi_getreg(priv, SAM_SPI_IMR_OFFSET);
      pending = sr & imr;

      /* Return from the interrupt handler when all pending interrupts have
       * been processed.
       */

      if (pending == 0)
        {
          return OK;
        }

      /* TThe SPI waits until NSS goes active before receiving the serial
       * clock from an external master. When NSS falls, the clock is
       * validated and the data is loaded in the SPI_RDR depending on the
       * BITS field configured in the SPI_CSR0.  These bits are processed
       * following a phase and a polarity defined respectively by the NCPHA
       * and CPOL bits in the SPI_CSR0.
       *
       * When all bits are processed, the received data is transferred in
       * the SPI_RDR and the RDRF bit rises. If the SPI_RDR has not been
       * read before new data is received, the Overrun Error Status (OVRES)
       * bit in the SPI_SR is set. As long as this flag is set, data is
       * loaded in the SPI_RDR. The user must read SPI_SR to clear the OVRES
       * bit.
       */

#ifdef CONFIG_DEBUG_SPI_ERROR
      /* Check the RX data overflow condition */

      if ((pending & SPI_INT_OVRES) != 0)
        {
          /* If debug is enabled, report any overrun errors */

          spierr("ERROR: Overrun (OVRES): %08x\n", pending);

          /* OVRES was cleared by the status read. */
        }
#endif

      /* Check for the availability of RX data */

      if ((pending & SPI_INT_RDRF) != 0)
        {
          uint16_t data;

          /* We get no indication of the falling edge of NSS.  But if we are
           * here then it must have fallen.
           */

          if (priv->nss)
            {
              priv->nss = false;
              SPI_SDEV_SELECT(priv->sdev, true);
            }

          /* Read the RDR to get the data and to clear the pending RDRF
           * interrupt.
           */

          regval = spi_getreg(priv, SAM_SPI_RDR_OFFSET);
          data   = (uint16_t)((regval & SPI_RDR_RD_MASK) >> SPI_RDR_RD_SHIFT);

          /* Enable TXDR/OVRE interrupts */

          regval = (SPI_INT_TDRE | SPI_INT_UNDES);
          spi_putreg(priv, regval, SAM_SPI_IER_OFFSET);

          /* Report the receipt of data to the SPI device driver */

          SPI_SDEV_RECEIVE(priv->sdev, data);
        }

      /* When a transfer starts, the data shifted out is the data present
       * in the Shift register. If no data has been written in the SPI_TDR,
       * the last data received is transferred. If no data has been received
       * since the last reset, all bits are transmitted low, as the Shift
       * register resets to 0.
       *
       * When a first data is written in the SPI_TDR, it is transferred
       * immediately in the Shift register and the TDRE flag rises. If new
       * data is written, it remains in the SPI_TDR until a transfer occurs,
       * i.e., NSS falls and there is a valid clock on the SPCK pin. When
       * the transfer occurs, the last data written in the SPI_TDR is
       * transferred in the Shift register and the TDRE flag rises. This
       * enables frequent updates of critical variables with single transfers.
       *
       * Then, new data is loaded in the Shift register from the SPI_TDR. If
       * no character is ready to be transmitted, i.e., no character has been
       * written in the SPI_TDR since the last load from the SPI_TDR to the
       * Shift register, the SPI_TDR is retransmitted. In this case the
       * Underrun Error Status Flag (UNDES) is set in the SPI_SR.
       */

#ifdef CONFIG_DEBUG_SPI_ERROR
      /* Check the TX data underflow condition */

      if ((pending & SPI_INT_UNDES) != 0)
        {
          /* If debug is enabled, report any overrun errors */

          spierr("ERROR: Underrun (UNDEX): %08x\n", pending);

          /* UNDES was cleared by the status read. */
        }
#endif

      /* Output the next TX data */

      if ((pending & SPI_INT_TDRE) != 0)
        {
          /* Get the next output value and write it to the TDR
           * The TDRE interrupt is cleared by writing to the from RDR.
           */

          regval = spi_dequeue(priv);
          spi_putreg(priv, regval, SAM_SPI_TDR_OFFSET);
        }

      /* The SPI slave hardware provides only an event when NSS rises
       * which may or many not happen at the end of a transfer.  NSSR was
       * cleared by the status read.
       */

      if ((pending & SPI_INT_NSSR) != 0)
        {
          /* Disable further TXDR/OVRE interrupts */

          regval = (SPI_INT_TDRE | SPI_INT_UNDES);
          spi_putreg(priv, regval, SAM_SPI_IDR_OFFSET);

          /* Report the state change to the SPI device driver */

          priv->nss = true;
          SPI_SDEV_SELECT(priv->sdev, false);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: spi_dequeue
 *
 * Description:
 *   Return the next queued output value.  If nothing is in the output queue,
 *   then return the last value obtained from getdata();
 *
 * Input Parameters:
 *   priv - SPI controller CS state
 *
 * Assumptions:
 *   Called only from the SPI interrupt handler so all interrupts are
 *   disabled.
 *
 ****************************************************************************/

static uint16_t spi_dequeue(struct sam_spidev_s *priv)
{
  uint32_t regval;
  uint16_t ret;
  int next;

  /* Is the queue empty? */

  if (priv->head != priv->tail)
    {
      /* No, take the oldest value from the tail of the cicular buffer */

      ret = priv->outq[priv->tail];

      /* Update the tail index, handling wraparound */

      next = priv->tail + 1;
      if (next >= CONFIG_SAMV7_SPI_SLAVE_QSIZE)
        {
          next = 0;
        }

      priv->tail = next;

      /* If the queue is empty Disable further TXDR/OVRE interrupts until
       * spi_enqueue() is called or until we received another command.  We
       * do this only for the case where NSS is non-functional (tied to
       * ground) and we need to end transfers in some fashion.
       */

      if (priv->head == next)
        {
           regval = (SPI_INT_TDRE | SPI_INT_UNDES);
           spi_putreg(priv, regval, SAM_SPI_IDR_OFFSET);
        }
    }
  else
    {
      /* Yes, return the last value we got from the getdata() method */

      ret = priv->outval;

      /* Disable further TXDR/OVRE interrupts until spi_enqueue() is called. */

      regval = (SPI_INT_TDRE | SPI_INT_UNDES);
      spi_putreg(priv, regval, SAM_SPI_IDR_OFFSET);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. See enum spi_smode_e for mode definitions
 *
 * Input Parameters:
 *   priv - SPI device data structure
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(struct sam_spidev_s *priv, enum spi_smode_e mode)
{
  uint32_t regval;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set the mode appropriately:
       *
       * SPI  CPOL NCPHA
       * MODE
       *  0    0    1
       *  1    0    0
       *  2    1    1
       *  3    1    0
       */

      regval  = spi_getreg(priv, SAM_SPI_CSR0_OFFSET);
      regval &= ~(SPI_CSR_CPOL | SPI_CSR_NCPHA);

      switch (mode)
        {
        case SPISLAVE_MODE0: /* CPOL=0; NCPHA=1 */
          regval |= SPI_CSR_NCPHA;
          break;

        case SPISLAVE_MODE1: /* CPOL=0; NCPHA=0 */
          break;

        case SPISLAVE_MODE2: /* CPOL=1; NCPHA=1 */
          regval |= (SPI_CSR_CPOL | SPI_CSR_NCPHA);
          break;

        case SPISLAVE_MODE3: /* CPOL=1; NCPHA=0 */
          regval |= SPI_CSR_CPOL;
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(priv, regval, SAM_SPI_CSR0_OFFSET);
      spiinfo("csr0=%08x\n", regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   priv   - SPI device data structure
 *   nbits  - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(struct sam_spidev_s *priv, int nbits)
{
  uint32_t regval;

  spiinfo("nbits=%d\n", nbits);
  DEBUGASSERT(priv && nbits > 7 && nbits < 17);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set number of bits appropriately */

      regval  = spi_getreg(priv, SAM_SPI_CSR0_OFFSET);
      regval &= ~SPI_CSR_BITS_MASK;
      regval |= SPI_CSR_BITS(nbits);
      spi_putreg(priv, regval, SAM_SPI_CSR0_OFFSET);

      spiinfo("csr0=%08x\n", regval);

      /* Save the selection so the subsequence re-configurations will be faster */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_bind
 *
 * Description:
 *   Bind the SPI slave device interface to the SPI slave controller
 *   interface and configure the SPI interface.  Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   sdev   - SPI slave device interface instance
 *   mode   - The SPI mode requested
 *   nbits  - The number of bits requests.
 *            If value is greater > 0 then it implies MSB first
 *            If value is below < 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_bind(struct spi_sctrlr_s *sctrlr,
                     struct spi_sdev_s *sdev, enum spi_smode_e mode,
                     int nbits)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)sctrlr;
  uint32_t regval;

  spiinfo("sdev=%p mode=%d nbits=%d\n", sdv, mode, nbits);

  DEBUGASSERT(priv != NULL && priv->sdev == NULL && sdev != NULL);

  /* Get exclusive access to the SPI device */

  spi_semtake(priv);

  /* Bind the SPI slave device interface instance to the SPI slave
   * controller interface.
   */

  priv->sdev = sdev;

  /* Call the slaved device's select() and cmddata() methods to indicate
   * the initial state of the chip select and  command/data discretes.
   *
   * NOTE:  Unless we reconfigure the NSS GPIO pin, it may not be possible
   * to read the NSS pin value (I haven't actually tried just reading it).
   * And, since the is no interrupt on the falling edge of NSS, we get no
   * notification when we are selected... not until the arrival of data.
   *
   * REVISIT:  A board-level interface would be required in order to support
   * the Command/Data indication (not yet impklemented).
   */

  SPI_SDEV_SELECT(sdev, false);
#warning Missing logic
  SPI_SDEV_CMDDATA(sdev, false);

  /* Discard any queued data */

  priv->head  = 0;
  priv->tail  = 0;

  /* Call the slave device's getdata() method to get the value that will
   * be shifted out the SPI clock is detected.
   */

  priv->outval = SPI_SDEV_GETDATA(sdev);
  spi_putreg(priv, priv->outval, SAM_SPI_TDR_OFFSET);

  /* Setup to begin normal SPI operation */

  spi_setmode(priv, mode);
  spi_setbits(priv, nbits);

  /* Clear pending interrupts by reading the SPI Status Register */

  regval = spi_getreg(priv, SAM_SPI_SR_OFFSET);
  UNUSED(regval);

  /* Enable SPI interrupts (already enabled at the NVIC):
   *
   * Data Transfer:
   *   SPI_INT_RDRF - Receive Data Register Full Interrupt
   *   SPI_INT_TDRE - Transmit Data Register Empty Interrupt
   *   SPI_INT_NSSR - NSS Rising Interrupt
   *
   * Transfer Errors (for DEBUG purposes only):
   *   SPI_INT_OVRES - Overrun Error Interrupt
   *   SPI_INT_UNDES - Underrun Error Status Interrupt (slave)
   *
   * Not Used:
   *  SPI_INT_MODF    - Mode Fault Error Interrupt
   *  SPI_INT_TXEMPTY  - Transmission Registers Empty Interrupt
   *
   * TX interrupts (SPI_INT_TDRE and SPI_INT_UNDES) are not enabled until
   * the transfer of data actually starts.
   */

  regval  = (SPI_INT_RDRF | SPI_INT_NSSR);
#ifdef CONFIG_DEBUG_SPI_ERROR
  regval |= SPI_INT_OVRES;
#endif

  spi_putreg(priv, regval, SAM_SPI_IER_OFFSET);

  spi_semgive(priv);
}

/****************************************************************************
 * Name: spi_unbind
 *
 * Description:
 *   Un-bind the SPI slave device interface from the SPI slave controller
 *   interface.  Reset the SPI interface and restore the SPI slave
 *   controller driver to its initial state,
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_unbind(struct spi_sctrlr_s *sctrlr)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)sctrlr;

  DEBUGASSERT(priv != NULL);
  spiinfo("Unbinding %p\n", priv->sdev);

  DEBUGASSERT(priv->sdev != NULL);

  /* Get exclusive access to the SPI device */

  spi_semtake(priv);

  /* Disable SPI interrupts (still enabled at the NVIC) */

  spi_putreg(priv, SPI_INT_ALL, SAM_SPI_IDR_OFFSET);

  /* Unbind the SPI slave interface */

  priv->sdev = NULL;

  /* Disable the SPI peripheral */

  spi_putreg(priv, SPI_CR_SPIDIS, SAM_SPI_CR_OFFSET);

  /* Execute a software reset of the SPI (twice) */

  spi_putreg(priv, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);
  spi_putreg(priv, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);

  spi_semgive(priv);
}

/****************************************************************************
 * Name: spi_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface.  This adds
 *   the word the controller driver for a subsequent transfer but has no
 *   effect on anyin-process or currently "committed" transfers
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   data   - Command/data mode data value to be shifted out.  The width of
 *            the data must be the same as the nbits parameter previously
 *            provided to the bind() methods.
 *
 * Returned Value:
 *   Zero if the word was successfully queue; A negated errno valid is
 *   returned on any failure to enqueue the word (such as if the queue is
 *   full).
 *
 ****************************************************************************/

static int spi_enqueue(struct spi_sctrlr_s *sctrlr, uint16_t data)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)sctrlr;
  irqstate_t flags;
  uint32_t regval;
  int next;
  int ret;

  spiinfo("data=%04x\n", data);
  DEBUGASSERT(priv != NULL && priv->sdev != NULL);

  /* Get exclusive access to the SPI device */

  spi_semtake(priv);

  /* Check if this word would overflow the circular buffer
   *
   * Interrupts are disabled briefly.
   */

  flags = enter_critical_section();
  next = priv->head + 1;
  if (next >= CONFIG_SAMV7_SPI_SLAVE_QSIZE)
    {
      next = 0;
    }

  if (next == priv->tail)
    {
      ret = -ENOSPC;
    }
  else
    {
      /* Save this new word as the next word to shifted out.  The current
       * word written to the TX data registers is "committed" and will not
       * be overwritten.
       */

      priv->outq[priv->head] = data;
      priv->head = next;
      ret = OK;

      /* Enable TX interrupts if we have begun the transfer */

      if (!priv->nss)
        {
          /* Enable TXDR/OVRE interrupts */

          regval = (SPI_INT_TDRE | SPI_INT_UNDES);
          spi_putreg(priv, regval, SAM_SPI_IER_OFFSET);
        }
    }

  leave_critical_section(flags);
  spi_semgive(priv);
  return ret;
}

/****************************************************************************
 * Name: spi_qfull
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   true if the output wueue is full
 *
 ****************************************************************************/

static bool spi_qfull(struct spi_sctrlr_s *sctrlr)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)sctrlr;
  irqstate_t flags;
  int next;
  bool ret;

  DEBUGASSERT(priv != NULL && priv->sdev != NULL);

  /* Get exclusive access to the SPI device */

  spi_semtake(priv);

  /* Check if another word would overflow the circular buffer
   *
   * Interrupts are disabled briefly.
   */

  flags = enter_critical_section();
  next = priv->head + 1;
  if (next >= CONFIG_SAMV7_SPI_SLAVE_QSIZE)
    {
      next = 0;
    }

  ret = (next == priv->tail);
  leave_critical_section(flags);
  spi_semgive(priv);
  return ret;
}

/****************************************************************************
 * Name: spi_qflush
 *
 * Description:
 *   Discard all saved values in the output queue.  On return from this
 *   function the output queue will be empty.  Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_qflush(struct spi_sctrlr_s *sctrlr)
{
  struct sam_spidev_s *priv = (struct sam_spidev_s *)sctrlr;
  irqstate_t flags;

  spiinfo("data=%04x\n", data);

  DEBUGASSERT(priv != NULL && priv->sdev != NULL);

  /* Get exclusive access to the SPI device */

  spi_semtake(priv);

  /* Mark the buffer empty, momentarily disabling interrupts */

  flags = enter_critical_section();
  priv->head = 0;
  priv->tail = 0;
  leave_critical_section(flags);
  spi_semgive(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spi_slave_initialize
 *
 * Description:
 *   Initialize the selected SPI port in slave mode.
 *
 * Input Parameters:
 *   port - Chip select number identifying the "logical" SPI port.  Includes
 *          encoded port and chip select information.
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_sctrlr_s *sam_spi_slave_initialize(int port)
{
  struct sam_spidev_s *priv;
  int spino = (port & __SPI_SPI_MASK) >> __SPI_SPI_SHIFT;
  irqstate_t flags;
  uint32_t regval;

  /* The support SAM parts have only a single SPI port */

  spiinfo("port: %d spino: %d\n", port, spino);

#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
  DEBUGASSERT(spino >= 0 && spino <= 1);
#elif defined(CONFIG_SAMV7_SPI0_SLAVE)
  DEBUGASSERT(spino == 0);
#else
  DEBUGASSERT(spino == 1);
#endif

#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
  if (spino == 0)
    {
      priv = &g_spi0_sctrlr;
    }
  else
    {
      priv = &g_spi1_sctrlr;
    }

#elif defined(CONFIG_SAMV7_SPI0_SLAVE)
  priv = &g_spi0_sctrlr;

#elif defined(CONFIG_SAMV7_SPI1_SLAVE)
  priv = &g_spi1_sctrlr;
#endif

  /* Set up the initial state for this chip select structure.  Other fields
   * are zeroed.
   */

  memset(priv, 0, sizeof(struct sam_spidev_s));

  /* Initialize the SPI operations */

  priv->sctrlr.ops = &g_sctrlr_ops;

  /* Save the SPI controller number */

  priv->spino = spino;

  /* Has the SPI hardware been initialized? */

  if (!priv->initialized)
    {
      /* Enable clocking to the SPI block */

      flags = enter_critical_section();
#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
      if (spino == 0)
#endif
#if defined(CONFIG_SAMV7_SPI0_SLAVE)
        {
          /* Set the SPI0 register base address and interrupt information */

          priv->base = SAM_SPI0_BASE,
          priv->irq  = SAM_IRQ_SPI0;

          /* Enable peripheral clocking to SPI0 */

          sam_spi0_enableclk();

          /* Configure multiplexed pins as connected on the board. */

          sam_configgpio(GPIO_SPI0_MISO); /* Output */
          sam_configgpio(GPIO_SPI0_MOSI); /* Input */
          sam_configgpio(GPIO_SPI0_SPCK); /* Drives slave */
          sam_configgpio(GPIO_SPI0_NSS);  /* aka NPCS0 */
        }
#endif
#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
      else
#endif
#if defined(CONFIG_SAMV7_SPI1_SLAVE)
        {
          /* Set the SPI1 register base address and interrupt information */

          priv->base = SAM_SPI1_BASE,
          priv->irq  = SAM_IRQ_SPI1;

          /* Enable peripheral clocking to SPI1 */

          sam_spi1_enableclk();

          /* Configure multiplexed pins as connected on the board. */

          sam_configgpio(GPIO_SPI1_MISO); /* Output */
          sam_configgpio(GPIO_SPI1_MOSI); /* Input */
          sam_configgpio(GPIO_SPI1_SPCK); /* Drives slave */
          sam_configgpio(GPIO_SPI0_NSS);  /* aka NPCS0 */
        }
#endif

      /* Disable the SPI peripheral */

      spi_putreg(priv, SPI_CR_SPIDIS, SAM_SPI_CR_OFFSET);

      /* Execute a software reset of the SPI (twice) */

      spi_putreg(priv, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);
      spi_putreg(priv, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);
      leave_critical_section(flags);

      /* Configure the SPI mode register */

      spi_putreg(priv, SPI_MR_SLAVE | SPI_MR_MODFDIS, SAM_SPI_MR_OFFSET);

      /* And enable the SPI */

      spi_putreg(priv, SPI_CR_SPIEN, SAM_SPI_CR_OFFSET);
      up_mdelay(20);

      /* Flush any pending interrupts/transfers */

      (void)spi_getreg(priv, SAM_SPI_SR_OFFSET);
      (void)spi_getreg(priv, SAM_SPI_RDR_OFFSET);

      /* Initialize the SPI semaphore that enforces mutually exclusive
       * access to the SPI registers.
       */

      nxsem_init(&priv->spisem, 0, 1);
      priv->nss         = true;
      priv->initialized = true;

      /* Disable all SPI interrupts at the SPI peripheral */

      spi_putreg(priv, SPI_INT_ALL, SAM_SPI_IDR_OFFSET);

      /* Attach and enable interrupts at the NVIC */

      DEBUGVERIFY(irq_attach(priv->irq, spi_interrupt, priv));
      up_enable_irq(priv->irq);

      spi_dumpregs(priv, "After initialization");
    }

  /* Set to mode=0 and nbits=8 */

  regval = spi_getreg(priv, SAM_SPI_CSR0_OFFSET);
  regval &= ~(SPI_CSR_CPOL | SPI_CSR_NCPHA | SPI_CSR_BITS_MASK);
  regval |= (SPI_CSR_NCPHA | SPI_CSR_BITS(8));
  spi_putreg(priv, regval, SAM_SPI_CSR0_OFFSET);

  priv->nbits = 8;
  spiinfo("csr[offset=%02x]=%08x\n", offset, regval);

  return &priv->sctrlr;
}
#endif /* CONFIG_SAMV7_SPI_SLAVE */
