/****************************************************************************
 * arch/arm/src/efm32/efm32_flash.c
 *
 *  Copyright 2014 Silicon Laboratories, Inc. http://www.silabs.com
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of
 * any kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties
 * against infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 *   Copyright (C) 2015 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
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

/* Provides standard flash access functions, to be used by the flash mtd
 * driver. The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/efm32_msc.h"
#include "hardware/efm32_devinfo.h"

#include "efm32_bitband.h"

#include "nuttx/progmem.h"

/* Only for the EFM32 family for now */

#if defined(CONFIG_ARCH_CHIP_EFM32) && defined(CONFIG_EFM32_FLASHPROG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_RAMFUNCS
#   error "Flashing function should executed in ram"
#endif

#ifndef EFM32_USERDATA_SIZE
#   error "EFM32_USERDATA_SIZE should be defined"
#endif

#ifndef EFM32_USERDATA_BASE
#   define "EFM32_USERDATA_BASE should be defined"
#endif

#ifndef EFM32_USERDATA_NPAGES
#   define "EFM32_FLASH_NPAGES should be defined"
#endif

#ifndef EFM32_USERDATA_PAGESIZE
#   define EFM32_USERDATA_PAGESIZE (EFM32_USERDATA_SIZE/EFM32_USERDATA_NPAGES)
#endif

#define EFM32_FLASH_ERASEDVAL (0xffu)

/* brief:
 *    The timeout used while waiting for the flash to become ready after
 *    a write. This number indicates the number of iterations to perform
 *    before issuing a timeout.
 * note:
 *    This timeout is set very large (in the order of 100x longer than
 *    necessary). This is to avoid any corner cases.
 */

#define MSC_PROGRAM_TIMEOUT    10000000ul

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void efm32_flash_unlock(void)
{
  uint32_t regval;

  /* Unlock the EFM32_MSC */

  putreg32(MSC_UNLOCK_CODE, EFM32_MSC_LOCK);

  /* Disable writing to the flash */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL, _MSC_WRITECTRL_WREN_SHIFT, 0);

#if defined(_MSC_TIMEBASE_MASK)

  regval = getreg32(EFM32_MSC_TIMEBASE);
  regval &= ~(_MSC_TIMEBASE_BASE_MASK | _MSC_TIMEBASE_PERIOD_MASK);

  /* Configure EFM32_MSC_TIMEBASE according to selected frequency */

  if (BOARD_AUXCLK_FREQUENCY > 7000000)
    {
      uint32_t freq;
      uint32_t cycles;

      /* Calculate number of clock cycles for 1us as base period */

      freq   = (BOARD_AUXCLK_FREQUENCY * 11) / 10;
      cycles = (freq / 1000000) + 1;

      /* Configure clock cycles for flash timing */

      regval |= MSC_TIMEBASE_PERIOD_1US;
      regval |= (cycles << _MSC_TIMEBASE_BASE_SHIFT);
    }
  else
    {
      uint32_t freq;
      uint32_t cycles;

      /* Calculate number of clock cycles for 5us as base period */

      freq   = (BOARD_AUXCLK_FREQUENCY * 5 * 11) / 10;
      cycles = (freq / 1000000) + 1;

      /* Configure clock cycles for flash timing */

      regval |= MSC_TIMEBASE_PERIOD_5US;
      regval |= (cycles << _MSC_TIMEBASE_BASE_SHIFT);
    }

  putreg32(regval, EFM32_MSC_TIMEBASE);

#endif
}

/****************************************************************************
 * Name: msc_load_verify_address
 *
 * Description:
 *   Perform address phase of FLASH write cycle.
 *
 *   This function performs the address phase of a Flash write operation by
 *   writing the given flash address to the ADDRB register and issuing the
 *   LADDRIM command to load the address.
 * note:
 *   This function MUST be executed from RAM. Failure to execute this portion
 *   of the code in RAM will result in a hardfault. For IAR, Rowley and
 *   Codesourcery this will be achieved automatically. For Keil uVision 4 you
 *   must define a section called "ram_code" and place this manually in your
 *   project's scatter file.
 * param:
 *   address : Address in flash memory. Must be aligned at a 4 byte boundary.
 * return:
 *   Returns the status of the address load operation, #msc_Return_TypeDef
 *   OK         - Operation completed successfully.
 *   -EBUSY     - Busy timeout.
 *   -EINVAL    - Operation tried to access a non-flash area.
 *   -EACCES    - Operation tried to access a locked area of the flash.
 ****************************************************************************/

int __ramfunc__ msc_load_verify_address(uint32_t *address)
{
  uint32_t status;
  uint32_t timeout;

  /* Wait for the MSC to become ready. */

  timeout = MSC_PROGRAM_TIMEOUT;
  while ((getreg32(EFM32_MSC_STATUS) & MSC_STATUS_BUSY) && (timeout != 0))
    {
      timeout--;
    }

  /* Check for timeout */

  if (timeout == 0)
    {
      return -EBUSY;
    }

  /* Load address */

  putreg32((uint32_t) (address), EFM32_MSC_ADDRB);
  putreg32(MSC_WRITECMD_LADDRIM, EFM32_MSC_WRITECMD);

  status = getreg32(EFM32_MSC_STATUS);
  if (status & (MSC_STATUS_INVADDR | MSC_STATUS_LOCKED))
    {
      /* Check for invalid address */

      if (status & MSC_STATUS_INVADDR)
        {
          return -EINVAL;
        }

      /* Check for write protected page */

      if (status & MSC_STATUS_LOCKED)
        {
          return -EACCES;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: msc_load_data
 *
 * Description:
 *   Perform data phase of FLASH write cycle.
 *
 *   This function performs the data phase of a Flash write operation by
 *   loading the given number of 32-bit words to the WDATA register.
 *
 * note:
 *   This function MUST be executed from RAM. Failure to execute this portion
 *   of the code in RAM will result in a hardfault. For IAR, Rowley and
 *   Codesourcery this will be achieved automatically. For Keil uVision 4 you
 *   must define a section called "ram_code" and place this manually in your
 *   project's scatter file.
 *
 * Input Parameters:
 *      data        :  Pointer to the first data word to load.
 *      num_words   :  Number of data words (32-bit) to load.
 *
 * Returned Value:
 *   Returns the status of the data load operation, #msc_Return_TypeDef
 *   OK         - Operation completed successfully.
 *   -ETIMEDOUT - Operation timed out waiting for flash operation
 *                      to complete.
 ****************************************************************************/

int __ramfunc__ msc_load_write_data(uint32_t *data, uint32_t num_words,
                                    bool write_strategy_safe)
{
  int timeout;
  int word_index;
  int words_per_data_phase;
  int ret = 0;

#if defined(_MSC_WRITECTRL_LPWRITE_MASK) && defined(_MSC_WRITECTRL_WDOUBLE_MASK)

  /* If LPWRITE (Low Power Write) is NOT enabled,
   * set WDOUBLE (Write Double word)
   */

  if (!(getreg32(EFM32_MSC_WRITECTRL) & MSC_WRITECTRL_LPWRITE))
    {
      /* If the number of words to be written are odd, we need to align by
       * writing a single word first, before setting the WDOUBLE bit.
       */

      if (num_words & 0x1)
        {
          /* Wait for the msc to be ready for the next word. */

          timeout = MSC_PROGRAM_TIMEOUT;
          while ((!(getreg32(EFM32_MSC_STATUS) & MSC_STATUS_WDATAREADY)) && \
                 (timeout != 0))
            {
              timeout--;
            }

          /* Check for timeout */

          if (timeout == 0)
            {
              return -ETIMEDOUT;
            }

          /* Clear double word option, in order to write one single word. */

          bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                                 _MSC_WRITECTRL_WDOUBLE_SHIFT, 0);

          /* Write first data word. */

          putreg32(*data++, EFM32_MSC_WDATA);
          putreg32(MSC_WRITECMD_WRITEONCE, EFM32_MSC_WRITECMD);

          /* Wait for the operation to finish. It may be required to change
           * the WDOUBLE config after the initial write. It should not be
           * changed while BUSY.
           */

          timeout = MSC_PROGRAM_TIMEOUT;
          while ((getreg32(EFM32_MSC_STATUS) & MSC_STATUS_BUSY) &&
                 (timeout != 0))
            {
              timeout--;
            }

          /* Check for timeout */

          if (timeout == 0)
            {
              return -ETIMEDOUT;
            }

          /* Subtract this initial odd word for the write loop below */

          num_words--;
          ret = 0;
        }

      /* Now we can set the double word option in order to write two words
       * per data phase.
       */

      bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                             _MSC_WRITECTRL_WDOUBLE_SHIFT, 1);
      words_per_data_phase = 2;
    }
  else
#endif
    {
        words_per_data_phase = 1;
    }

  /* Write the rest as double word write if wordsPerDataPhase == 2 */

  if (num_words > 0)
    {
      /* Write strategy: msc_write_int_safe */

      if (write_strategy_safe)
        {
          /* Requires a system core clock at 1MHz or higher */

          DEBUGASSERT(BOARD_SYSTEM_FREQUENCY >= 1000000);

          word_index = 0;
          while (word_index < num_words)
            {
              putreg32(*data++, EFM32_MSC_WDATA);
              word_index++;
              if (words_per_data_phase == 2)
                {
                  while (!(getreg32(EFM32_MSC_STATUS) &
                           MSC_STATUS_WDATAREADY))
                    {
                    }

                  putreg32(*data++, EFM32_MSC_WDATA);
                  word_index++;
                }

              putreg32(MSC_WRITECMD_WRITEONCE, EFM32_MSC_WRITECMD);

              /* Wait for the transaction to finish. */

              timeout = MSC_PROGRAM_TIMEOUT;
              while ((getreg32(EFM32_MSC_STATUS) & MSC_STATUS_BUSY) && \
                     (timeout != 0))
                {
                  timeout--;
                }

              /* Check for timeout */

              if (timeout == 0)
                {
                  ret = -ETIMEDOUT;
                  break;
                }

#if defined(CONFIG_EFM32_EFM32G)
              putreg32(getreg32(EFM32_MSC_ADDRB)+4, EFM32_MSC_ADDRB);
              putreg32(MSC_WRITECMD_LADDRIM, EFM32_MSC_WRITECMD);
#endif
            }
        }

      /* Write strategy: msc_write_fast */

      else
        {
#if defined(CONFIG_EFM32_EFM32G)

          /* Gecko does not have auto-increment of ADDR. */

          DEBUGPANIC();
#else

          /* Requires a system core clock at 14MHz or higher */

          DEBUGASSERT(BOARD_SYSTEM_FREQUENCY >= 14000000);

          word_index = 0;

          while (word_index < num_words)
            {
              /* Wait for the MSC to be ready for the next word. */

              while (!(getreg32(EFM32_MSC_STATUS) & MSC_STATUS_WDATAREADY))
                {
                  uint32_t regval;

                  /* If the write to MSC->WDATA below missed the 30us timeout
                   * and the following MSC_WRITECMD_WRITETRIG command arrived
                   * while MSC_STATUS_BUSY is 1, then the
                   * MSC_WRITECMD_WRITETRIG could be ignored by the MSC.
                   * In this case, MSC_STATUS_WORDTIMEOUT is set to 1 and
                   * MSC_STATUS_BUSY is 0. A new trigger is therefore needed
                   * here to complete write of data in MSC->WDATA.
                   * If WDATAREADY became high since entry into this loop,
                   * exit and continue to the next WDATA write.
                   */

                  regval = getreg32(EFM32_MSC_STATUS);
                  regval &= MSC_STATUS_WORDTIMEOUT;
                  regval &= MSC_STATUS_BUSY;
                  regval &= MSC_STATUS_WDATAREADY;
                  if (regval == MSC_STATUS_WORDTIMEOUT)
                    {
                      putreg32(MSC_WRITECMD_WRITETRIG, EFM32_MSC_WRITECMD);
                    }
                }

              putreg32(*data, EFM32_MSC_WDATA);
              if ((words_per_data_phase == 1) || \
                  ((words_per_data_phase == 2) && (word_index & 0x1)))
                {
                  putreg32(MSC_WRITECMD_WRITETRIG, EFM32_MSC_WRITECMD);
                }

              data++;
              word_index++;
            }

          /* Wait for the transaction to finish. */

          timeout = MSC_PROGRAM_TIMEOUT;
          while ((getreg32(EFM32_MSC_STATUS) & MSC_STATUS_BUSY) && \
                 (timeout != 0))
            {
              timeout--;
            }

          /* Check for timeout */

          if (timeout == 0)
            {
              ret = -ETIMEDOUT;
            }
#endif
        }
    }
#ifdef _MSC_WRITECTRL_WDOUBLE_MASK

  /* Clear double word option, which should not be left on when returning. */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                         _MSC_WRITECTRL_WDOUBLE_SHIFT, 0);

#endif

  return ret;
}

void efm32_flash_lock(void)
{
  /* Disable writing to the flash */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                         _MSC_WRITECTRL_WREN_SHIFT, 0);

  /* Unlock the EFM32_MSC */

  putreg32(0, EFM32_MSC_LOCK);
}

#ifndef EFM32_FLASH_SIZE
#define EFM32_FLASH_SIZE efm32_get_flash_size()
uint32_t efm32_get_flash_size(void)
{
  uint32_t regval;
  regval = getreg32(EFM32_DEVINFO_MEMINFO_SIZE);
  regval = (regval & _DEVINFO_MEMINFO_SIZE_FLASH_MASK) \
           >> _DEVINFO_MEMINFO_SIZE_FLASH_SHIFT;

  return regval * 1024;
}
#endif

#ifndef EFM32_FLASH_PAGESIZE
#define EFM32_FLASH_PAGESIZE efm32_get_flash_page_size()
uint32_t efm32_get_flash_page_size(void)
{
  uint32_t regval;
  regval = getreg32(EFM32_DEVINFO_MEMINFO_PAGE_SIZE);
  regval = (regval & _DEVINFO_MEMINFO_FLASH_PAGE_SIZE_MASK) \
           >> _DEVINFO_MEMINFO_FLASH_PAGE_SIZE_SHIFT;
  if (regval == 0xff)
    {
      return 512;
    }

  return 1 << (regval + 10);
}
#endif

#ifndef EFM32_FLASH_NPAGES
#define EFM32_FLASH_NPAGES efm32_get_flash_page_nbr()
uint32_t efm32_get_flash_page_nbr(void)
{
  return (EFM32_FLASH_SIZE / EFM32_FLASH_PAGESIZE);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  if (page < EFM32_FLASH_NPAGES)
    {
      return EFM32_FLASH_PAGESIZE;
    }

  page -= EFM32_FLASH_NPAGES;

  if (page < EFM32_USERDATA_NPAGES)
    {
      return EFM32_USERDATA_PAGESIZE;
    }

  return 0;
}

size_t up_progmem_erasesize(size_t block)
{
  return up_progmem_pagesize(block);
}

ssize_t up_progmem_getpage(size_t addr)
{
#if (EFM32_FLASH_BASE != 0)
  if ((addr >= (EFM32_FLASH_BASE)) && \
       (addr <  (EFM32_FLASH_BASE + EFM32_FLASH_SIZE)))
    {
      addr -= EFM32_FLASH_BASE;

      return addr / EFM32_FLASH_PAGESIZE;
    }

#else
  if (addr < EFM32_FLASH_SIZE)
    {
      return addr / EFM32_FLASH_PAGESIZE;
    }
#endif

  if ((addr >= (EFM32_USERDATA_BASE)) && \
       (addr <  (EFM32_USERDATA_BASE + EFM32_USERDATA_SIZE)))
    {
      addr -= EFM32_USERDATA_BASE;

      return (addr / EFM32_USERDATA_NPAGES) + EFM32_FLASH_NPAGES;
    }

  return -EFAULT;
}

size_t up_progmem_getaddress(size_t page)
{
  if (page < EFM32_FLASH_NPAGES)
    {
      return page * EFM32_FLASH_PAGESIZE + EFM32_FLASH_BASE;
    }

  page -= EFM32_FLASH_NPAGES;

  if (page < EFM32_USERDATA_NPAGES)
    {
      return EFM32_USERDATA_BASE + (page * EFM32_USERDATA_NPAGES);
    }

  return SIZE_MAX;
}

size_t up_progmem_neraseblocks(void)
{
  return EFM32_FLASH_NPAGES + EFM32_USERDATA_NPAGES;
}

bool up_progmem_isuniform(void)
{
  return false;
}

ssize_t __ramfunc__ up_progmem_eraseblock(size_t block)
{
  int ret = 0;
  int timeout;
  uint32_t regval;
  irqstate_t flags;

  if (block >= (EFM32_FLASH_NPAGES + EFM32_USERDATA_NPAGES))
    {
      return -EFAULT;
    }

  efm32_flash_unlock();

  flags = enter_critical_section();

  /* enable writing to the flash */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                         _MSC_WRITECTRL_WREN_SHIFT, 1);

  /* Load address */

  putreg32((uint32_t)up_progmem_getaddress(block), EFM32_MSC_ADDRB);
  putreg32(MSC_WRITECMD_LADDRIM, EFM32_MSC_WRITECMD);

  regval = getreg32(EFM32_MSC_STATUS);

  /* Check for invalid address */

  if (regval & MSC_STATUS_INVADDR)
    {
      ret = -EINVAL;
    }

  /* Check for write protected block */

  if ((ret == 0) && (regval & MSC_STATUS_LOCKED))
    {
      ret = -EPERM;
    }

  /* Send erase block command */

  if (ret == 0)
    {
      putreg32(MSC_WRITECMD_ERASEPAGE, EFM32_MSC_WRITECMD);

      /* Wait for the erase to complete */

      timeout = MSC_PROGRAM_TIMEOUT;
      while ((getreg32(EFM32_MSC_STATUS) & MSC_STATUS_BUSY) &&
             (timeout != 0))
        {
          timeout--;
        }

      if (timeout == 0)
        {
          ret = -ETIMEDOUT;
        }
    }

  /* Disable writing to the MSC */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                         _MSC_WRITECTRL_WREN_SHIFT, 0);

  if (ret == 0)
    {
      /* Verify */

      if (up_progmem_ispageerased(block) != 0)
        {
          ret = -EIO;
        }
    }

  leave_critical_section(flags);

  if (ret != 0)
    {
      return ret;
    }

  /* Success */

  return up_progmem_erasesize(block);
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= (EFM32_FLASH_NPAGES + EFM32_USERDATA_NPAGES))
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page),
       count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != EFM32_FLASH_ERASEDVAL)
        {
          bwritten++;
        }
    }

  return bwritten;
}

ssize_t __ramfunc__ up_progmem_write(size_t addr,
                                     const void *buf, size_t size)
{
  int       ret = 0;
  int       word_count;
  int       num_words;
  int       page_words;
  uint32_t *p_data;
  uint32_t *address = (uint32_t *)addr;
  uint32_t  num_bytes = size;

  /* EFM32 requires word access */

  if (addr & 3)
    {
      return -EINVAL;
    }

  /* EFM32 requires word access */

  if (num_bytes & 3)
    {
      return -EINVAL;
    }

  efm32_flash_unlock();

  /* enable writing to the flash */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                         _MSC_WRITECTRL_WREN_SHIFT, 1);

  /* Convert bytes to words */

  num_words = num_bytes >> 2;

  /* The following loop splits the data into chunks corresponding to flash
   * pages. The address is loaded only once per page, because the hardware
   * automatically increments the address internally for each data load
   * inside a page.
   */

  for (word_count = 0, p_data = (uint32_t *)buf;
       word_count < num_words;
      )
    {
      int page_bytes;
      ssize_t page_idx;
      irqstate_t flags;

      /* Compute the number of words to write to the current page. */

      page_idx = up_progmem_getpage((size_t)address + (word_count << 2));
      if (page_idx < 0)
        {
          ret = -EINVAL;
          break;
        }

      page_bytes = up_progmem_pagesize(page_idx);
      if (page_bytes < 0)
        {
          ret = -EINVAL;
          break;
        }

      page_words = (page_bytes - (((uint32_t) (address + word_count)) & \
                   (page_bytes - 1))) / sizeof(uint32_t);

      if (page_words > num_words - word_count)
        {
          page_words = num_words - word_count;
        }

      flags = enter_critical_section();

      /* First we load address.
       * The address is auto-incremented within a page.
       * Therefore the address phase is only needed once for each page.
       */

      ret = msc_load_verify_address(address + word_count);

      /* Now write the data in the current page. */

      if (ret == 0)
        {
          ret = msc_load_write_data(p_data, page_words, true);
        }

      leave_critical_section(flags);

      if (ret != 0)
        {
          break;
        }

      word_count += page_words;
      p_data += page_words;
    }

  /* Disable writing to the MSC */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL, _MSC_WRITECTRL_WREN_SHIFT, 0);

#if (defined(CONFIG_EFM32_EFM32GG) || defined(CONFIG_EFM32_EFM32WG))

  /* Turn off double word write cycle support. */

  bitband_set_peripheral(EFM32_MSC_WRITECTRL,
                         _MSC_WRITECTRL_WDOUBLE_SHIFT, 0);

#endif

  if (ret < 0)
    {
      return ret;
    }

  return word_count;
}

uint8_t up_progmem_erasestate(void)
{
  return EFM32_FLASH_ERASEDVAL;
}

#endif /* defined(CONFIG_ARCH_CHIP_EFM32)  */
