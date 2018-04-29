/****************************************************************************
 * arch/arm/src/tiva/tiva_eeprom.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Shirshak Sengupta <sgshirshak@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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
#include <errno.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "up_arch.h"
#include "chip.h"
#include "chip/tiva_syscontrol.h"
#include "tiva_eeprom.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This macro extracts the array index out of the peripheral number. */

#define SYSCTL_PERIPH_INDEX(a)  (((a) >> 28) & 0xf)

/* This macro constructs the peripheral bit mask from the peripheral number. */

#define SYSCTL_PERIPH_MASK(a)   (((a) & 0xffff) << (((a) & 0x001f0000) >> 16))
#define SYSCTL_PERIPH_EEPROM0   0xf0005800  /* REVISIT:  What is this? */

/* Macros for hardware access, both direct and via the bit-band region. */

#define PERIPHADDR(x, b)                            \
  (((unsigned long)(x) & 0xf0000000) | 0x02000000 | \
   (((unsigned long)(x) & 0x000fffff) << 5) | ((b) << 2))

/* Useful macros to extract the number of EEPROM blocks available on the
 * target device and the total EEPROM storage in bytes from the EESIZE
 * register.
 */

#define BLOCKS_FROM_EESIZE(x)   (((x) & EEPROM_EESIZE_BLKCNT_M) >> \
                                 EEPROM_EESIZE_BLKCNT_S)
#define SIZE_FROM_EESIZE(x)     ((((x) & EEPROM_EESIZE_WORDCNT_M) >> \
                                 EEPROM_EESIZE_WORDCNT_S) * 4)

/* Useful macro to extract the offset from a linear address. */

#define EEPROM_ADDR2OFFSET(x)   (((x) >> 2) & 0x0f)

/* The key value required to initiate a mass erase. */

#define EEPROM_MASS_ERASE_KEY   ((uint32_t)0xE37B << EEPROM_EEDBGME_KEY_S)
#define EEPROM_ADDR2BLOCK(addr) ((addr) >> 6)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct tiva_dev_s.
 */

struct tiva_dev_s
{
  struct mtd_dev_s mtd;

  /* Other implementation specific data may follow here */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int tiva_eeprom_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks);
static ssize_t tiva_eeprom_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                                 size_t nblocks, FAR uint8_t *buf);
static ssize_t tiva_eeprom_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks, FAR const uint8_t *buf);
static ssize_t tiva_eeprom_read(FAR struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tiva_eeprom_write(FAR struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, FAR const uint8_t *buf);
#endif
static int tiva_eeprom_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure holds the state of the MTD driver */

static struct tiva_dev_s g_eeprom_dev =
{
  {
    tiva_eeprom_erase,
    tiva_eeprom_bread,
    tiva_eeprom_bwrite,
    tiva_eeprom_read,
#ifdef CONFIG_MTD_BYTE_WRITE
    tiva_eeprom_write,
#endif
    tiva_eeprom_ioctl
  },

  /* Initialization of any other implementation specific data goes here */
};

/* An array that maps the "peripheral set" number (which is stored in the
 * upper nibble of the SYSCTL_PERIPH_* defines) to the SYSCTL_RCGC? register
 * that controls the run-mode enable for that peripheral.
 */

static const unsigned long g_pulrcgc_regs[] =
{
  TIVA_SYSCON_RCGC0,
  TIVA_SYSCON_RCGC1,
  TIVA_SYSCON_RCGC2
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_delay
 *
 * Description:
 *   Wait a small amount of time.  The total delay is about 3 * delay cycles.
 *
 ****************************************************************************/

static inline void tiva_delay(uint32_t delay)
{
  __asm__ __volatile__("1:\n"
                       "\tsubs  %0, #1\n"
                       "\tbne   1b\n"
                       : "=r"(delay) : "r"(delay));
}

/****************************************************************************
 * Name: tiva_eeprom_enable
 *
 * Description:
 *   Enables the EEPROM peripheral.
 *
 *   This function enables peripherals.  At power-up, all peripherals
 *   are disabled; they must be enabled in order to operate or respond to
 *   register reads/writes.
 *
 *   NOTE: It takes five clock cycles after the write to enable a peripheral
 *   before the the peripheral is actually enabled.  During this time, attempts
 *   to access the peripheral result in a bus fault.  Care should be taken
 *   to ensure that the peripheral is not accessed during this brief time
 *   period.
 *
 * Input Parameters:
 *   peripheral - The peripheral to enable.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void tiva_eeprom_enable(unsigned long peripheral)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* See if the peripheral index is 15, indicating a peripheral that is
   * accessed via the SYSCTL_RCGCperiph registers.
   */

  if ((peripheral & 0xf0000000) == 0xf0000000)
    {
      /* Enable this peripheral. */

      regaddr = PERIPHADDR(TIVA_SYSCON_RCGC_BASE + ((peripheral & 0xff00) >> 8),
                           peripheral & 0xff);
      putreg32(1, regaddr);
    }
  else
    {
      /* Enable this peripheral. */

      regaddr   = g_pulrcgc_regs[SYSCTL_PERIPH_INDEX(peripheral)];
      regval    = getreg32(regaddr);
      regval    |= SYSCTL_PERIPH_MASK(peripheral);
      putreg32(regval, regaddr);
    }
}

/****************************************************************************
 * Name: tiva_eeprom_reset
 *
 * Description:
 *   Performs a software reset of the EEPROM peripheral.
 *
 *   This function performs a software reset of the specified peripheral.  An
 *   individual peripheral reset signal is asserted for a brief period and then
 *   de-asserted, returning the internal state of the peripheral to its reset
 *   condition.
 *
 * Input Parameters:
 *   peripheral - The peripheral to reset.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void tiva_eeprom_reset(uint32_t peripheral)
{
  volatile uint_fast8_t delay;
  uintptr_t regaddr;

  /* Put the peripheral into the reset state. */

  regaddr = PERIPHADDR(TIVA_SYSCON_SR_BASE + ((peripheral & 0xff00) >> 8),
                       peripheral & 0xff);
  putreg32(1, regaddr);

  /* Delay for a little bit. */

  for (delay = 0; delay < 16; delay++)
    {
    }

  /* Take the peripheral out of the reset state. */

  putreg32(0, regaddr);
}

/****************************************************************************
 * Name: tiva_eeprom_waitdone
 *
 * Description:
 *   Block until the EEPROM peripheral is not busy.
 *
 ****************************************************************************/

static void tiva_eeprom_waitdone(void)
{
  /* Is the EEPROM still busy? */

  while (getreg32(TIVA_EEPROM_EEDONE) & EEPROM_EEDONE_WORKING)
    {
      /* Spin while EEPROM is busy. */
    }
}

/****************************************************************************
 * Name:tiva_eeprom_sectormask_set
 *
 * Description:
 *   This function implements a workaround for a bug in Blizzard rev A silicon.
 *   It ensures that only the 1KB flash sector containing a given EEPROM
 *   address is erased if an erase/copy operation is required as a result of
 *   a following EEPROM write.
 *
 ****************************************************************************/

#if 0 /* Not used */
static void tiva_eeprom_sectormask_set(uint32_t address)
{
  uint32_t mask;

  /* Determine which page contains the passed EEPROM address.  The 2KB EEPROM
   * is implemented in 16KB of flash with each 1KB sector of flash holding
   * values for 32 consecutive EEPROM words (or 128 bytes).
   */

  mask = ~(1 << (address >> 7));

  tiva_delay(10);

  putreg32(3, 0x400fd0fc);
  tiva_delay(10);

  putreg32(mask, 0x400ae2c0);
  tiva_delay(10);

  putreg32(0, 0x400fd0fc);
  tiva_delay(10);
}
#endif

/****************************************************************************
 * Name: tiva_eeprom_sectormask_clear
 *
 * Description:
 *   Clear the FSM sector erase mask to ensure that any following main array
 *   flash erase operations operate as expected.
 *
 ****************************************************************************/

#if 0 /* Not used */
static void tiva_eeprom_sectormask_clear(void)
{
  tiva_delay(10);

  putreg32(3, 0x400fd0fc);
  tiva_delay(10);

  putreg32(0, 0x400ae2c0);
  tiva_delay(10);

  putreg32(0, 0x400fd0fc);
  tiva_delay(10);
}
#endif

/****************************************************************************
 * Name: tiva_eeprom_write
 *
 * Description:
 *   Writes data to the EEPROM.
 *
 *   This function may be called to write data into the EEPROM at a given
 *   word-aligned address.  The call is synchronous and returns only after
 *   all data has been written or an error occurs.
 *
 * Input Parameters:
 *   data    - Points to the first word of data to write to the EEPROM.
 *   address - Defines the byte address within the EEPROM that the data
 *             is to be written to.  This value must be a multiple of 4.
 *   count   - defines the number of bytes of data that is to be written.
 *             This value must be a multiple of 4.
 *
 * Returned Value:
 *   Returns 0 on success or non-zero values on failure.  Failure codes
 *   are logical OR combinations of EEPROM_RC_WRBUSY, EEPROM_RC_NOPERM,
 *   EEPROM_RC_WKCOPY, EEPROM_RC_WKERASE, and EEPROM_RC_WORKING.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tiva_eeprom_write(FAR struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, FAR const uint8_t *buf)
{
  FAR uint32_t *data = (uint32_t*)buf;
  size_t remaining;
  uint32_t status;
  uint32_t regval;

  do
    {
      /* Read the status. */

      status = getreg32(TIVA_EEPROM_EEDONE);
    }
  while (status & EEPROM_EEDONE_WORKING);

  /* Set the block and offset appropriately to program the first word. */

  putreg32(EEPROM_ADDR2BLOCK(offset), TIVA_EEPROM_EEBLOCK);
  putreg32(EEPROM_ADDR2OFFSET(offset), TIVA_EEPROM_EEOFFSET);

  /* Convert the byte count to a word count. */

  remaining = nbytes >> 2;
  nbytes   &= ~3;;

  /* Write each word in turn. */

  while (remaining)
    {
      /* Write the next word through the autoincrementing register. */

      putreg32(*data, TIVA_EEPROM_EERDWRINC);

      /* Wait a few cycles.  In some cases, the WRBUSY bit is not set
       * immediately and this prevents us from dropping through the polling
       * loop before the bit is set.
       */

      tiva_delay(10);

      /* Wait for the write to complete. */

      do
        {
          /* Read the status. */

          status = getreg32(TIVA_EEPROM_EEDONE);
        }
      while (status & EEPROM_EEDONE_WORKING);

      /* Make sure we completed the write without errors.  Note that we
       * must check this per-word because write permission can be set per
       * block resulting in only a section of the write not being performed.
       */

      if (status & EEPROM_EEDONE_NOPERM)
        {
          return status;
        }

      /* Move on to the next word. */

      data++;
      remaining--;

      /* Do we need to move to the next block?  This is the case if the
       * offset register has just wrapped back to 0.  Note that we only
       * write the block register if we have more data to read.  If this
       * register is written, the hardware expects a read or write operation
       * next.  If a mass erase is requested instead, the mass erase will
       * fail.
       */

      if (remaining > 0 && (getreg32(TIVA_EEPROM_EEOFFSET) == 0))
        {
          regval = getreg32(TIVA_EEPROM_EEBLOCK);
          putreg32(regval + 1, TIVA_EEPROM_EEBLOCK);
        }
    }

#if 0
  /* Clear the sector protection bits to prevent possible problems when
   * programming the main flash array later.
   */

  if (CLASS_IS_TM4C123 && REVISION_IS_A0)
    {
      tiva_eeprom_sectormask_clear();
    }
#endif

  /* Return the current status to the caller. */

  return nbytes;
}
#endif

/****************************************************************************
 * Name: tiva_eeprom_read
 *
 * Description:
 *   Reads data from the EEPROM.
 *
 *   This function may be called to read a number of words of data from a
 *   word-aligned address within the EEPROM.  Data read is copied into the
 *   buffer pointed to by the \e data parameter.
 *
 * Input Parameters:
 *   data    - A pointer to storage for the data read from the EEPROM.
 *             This pointer must point to at least 'count' bytes of available
 *             memory.
 *   address - The byte address within the EEPROM from which data is
 *             to be read.  This value must be a multiple of 4.
 *   count   - The number of bytes of data to read from the EEPROM.
 *             This value must be a multiple of 4.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static ssize_t tiva_eeprom_read(FAR struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, FAR uint8_t *buf)
{
  FAR uint32_t *data = (uint32_t*)buf;
  size_t remaining;
  uint32_t regval;

  /* Set the block and offset appropriately to read the first word. */

  putreg32(EEPROM_ADDR2BLOCK(offset), TIVA_EEPROM_EEBLOCK);
  putreg32(EEPROM_ADDR2OFFSET(offset), TIVA_EEPROM_EEOFFSET);

  /* Convert the byte count to a word count. */

  remaining  = nbytes >> 2;
  nbytes    &= ~3;

  /* Read each word in turn. */

  while (remaining > 0)
    {
      /* Read the next word through the auto-incrementing register. */

      *data = getreg32(TIVA_EEPROM_EERDWRINC);

      /* Move on to the next word. */

      data++;
      remaining--;

      /* Do we need to move to the next block?  This is the case if the
       * offset register has just wrapped back to 0.  Note that we only
       * write the block register if we have more data to read.  If this
       * register is written, the hardware expects a read or write operation
       * next.  If a mass erase is requested instead, the mass erase will
       * fail.
       */

      if (remaining > 0 && (getreg32(TIVA_EEPROM_EEOFFSET) == 0))
        {
          regval = getreg32(TIVA_EEPROM_EEBLOCK);
          putreg32(regval + 1, TIVA_EEPROM_EEBLOCK);
        }
    }

  return nbytes;
}

/****************************************************************************
 * Unimplemented Methods
 ****************************************************************************/

static int tiva_eeprom_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks)
{
  return -ENOSYS;
}

static ssize_t tiva_eeprom_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                                 size_t nblocks, FAR uint8_t *buf)
{
  return -ENOSYS;
}

ssize_t tiva_eeprom_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf)
{
  return -ENOSYS;
}

static int tiva_eeprom_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                             unsigned long arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_eeprom_initialize
 *
 * Description:
 *  Performs any necessary recovery in case of power failures during write.
 *
 *  This function must be called after tiva_eeprom_enable() and before
 *  the EEPROM is accessed.  It is used to check for errors in the EEPROM state
 *  such as from power failure during a previous write operation.  The function
 *  detects these errors and performs as much recovery as possible.
 *
 *  If -ENODEV is returned, the EEPROM was unable to recover its
 *  state.  If power is stable when this occurs, this indicates a fatal
 *  error and is likely an indication that the EEPROM memory has exceeded its
 *  specified lifetime write/erase specification.  If the supply voltage is
 *  unstable when this return code is observed, retrying the operation once the
 *  voltage is stabilized may clear the error.
 *
 *  Failure to call this function after a reset may lead to incorrect operation
 *  or permanent data loss if the EEPROM is later written.
 *
 * Returned Value:
 *   Returns OK if no errors were detected or -ENODEV if the EEPROM
 *   peripheral cannot currently recover from an interrupted write or erase
 *   operation.
 *
 ****************************************************************************/

int tiva_eeprom_initialize(void)
{
  uint32_t status;

  tiva_eeprom_enable(SYSCTL_PERIPH_EEPROM0);

  /* Insert a small delay (6 cycles + call overhead) to guard against the
   * possibility that this function is called immediately after the EEPROM
   * peripheral is enabled.  Without this delay, there is a slight chance
   * that the first EEPROM register read will fault if you are using a
   * compiler with a ridiculously good optimizer!
   */

  tiva_delay(2);

  /* Make sure the EEPROM has finished any ongoing processing. */

  tiva_eeprom_waitdone();

  /* Read the EESUPP register to see if any errors have been reported. */

  status = getreg32(TIVA_EEPROM_EESUPP);

  /* Did an error of some sort occur during initialization? */

  if (status & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY))
    {
      return -ENODEV;
    }

  /* Perform a second EEPROM reset. */

  tiva_eeprom_reset(SYSCTL_PERIPH_EEPROM0);

  /* Wait for the EEPROM to complete its reset processing once again. */

  tiva_delay(2);
  tiva_eeprom_waitdone();

  /* Read EESUPP once again to determine if any error occurred. */

  status = getreg32(TIVA_EEPROM_EESUPP);

  /* Was an error reported following the second reset? */

  if (status & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY))
    {
      return -ENODEV;
    }

  /* The EEPROM does not indicate that any error occurred. */

  return OK;
}

/****************************************************************************
 * Name: tiva_eeprom_instance
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *tiva_eeprom_instance(void)
{
  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)&g_eeprom_dev;
}
