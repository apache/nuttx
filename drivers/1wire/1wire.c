/****************************************************************************
 * drivers/1wire/1wire.c
 *
 *   Copyright (C) 2018 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/drivers/1wire.h>

#include "1wire_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MATCH_FAMILY(rom, family) ((family) == 0 || ((rom) & 0xff) == (family))

#ifndef CONFIG_ENDIAN_BIG
#  define onewire_leuint64(x) (x)
#  define onewire_leuint32(x) (x)
#endif

#define NO_HOLDER ((pid_t)-1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: onewire_leuint64
 *
 * Description:
 *   Get a 64-bit value stored in little endian order for a big-endian
 *   machine.
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static inline uint64_t onewire_leuint64(uint64_t x)
{
  return (((x & 0xff00000000000000ull) >> 56) |
          ((x & 0x00ff000000000000ull) >> 40) |
          ((x & 0x0000ff0000000000ull) >> 24) |
          ((x & 0x000000ff00000000ull) >> 8)  |
          ((x & 0x00000000ff000000ull) << 8)  |
          ((x & 0x0000000000ff0000ull) << 24) |
          ((x & 0x000000000000ff00ull) << 40) |
          ((x & 0x00000000000000ffull) << 56));
}
#endif

/****************************************************************************
 * Name: onewire_leuint32
 *
 * Description:
 *   Get a 32-bit value stored in little endian order for a big-endian
 *   machine.
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static inline uint32_t onewire_leuint32(uint32_t x)
{
  return (((x & 0xff000000) >> 24) |
          ((x & 0x00ff0000) >> 8)  |
          ((x & 0x0000ff00) << 8)  |
          ((x & 0x000000ff) << 24));
}
#endif

/****************************************************************************
 * Name: onewire_sem_init
 *
 * Description:
 *
 ****************************************************************************/

static inline void onewire_sem_init(FAR struct onewire_sem_s *sem)
{
  sem->holder = NO_HOLDER;
  sem->count  = 0;
  nxsem_init(&sem->sem, 0, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: onewire_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ****************************************************************************/

void onewire_sem_wait(FAR struct onewire_master_s *master)
{
  pid_t me;

  /* Do we already hold the semaphore? */

  me = getpid();
  if (me == master->devsem.holder)
    {
      /* Yes... just increment the count */

      master->devsem.count++;
      DEBUGASSERT(master->devsem.count > 0);
    }

  /* Take the semaphore (perhaps waiting) */

  else
    {
      int ret;

      do
        {
          ret = nxsem_wait(&master->devsem.sem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);

      /* No we hold the semaphore */

      master->devsem.holder = me;
      master->devsem.count  = 1;
    }
}

/****************************************************************************
 * Name: onewire_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

void onewire_sem_post(FAR struct onewire_master_s *master)
{
  DEBUGASSERT(master->devsem.holder == getpid());

  /* Is this our last count on the semaphore? */

  if (master->devsem.count > 1)
    {
      /* No.. just decrement the count */

      master->devsem.count--;
    }

  /* Yes.. then we can really release the semaphore */

  else
    {
      master->devsem.holder = NO_HOLDER;
      master->devsem.count  = 0;
      nxsem_post(&master->devsem.sem);
    }
}

/****************************************************************************
 * Name: onewire_reset_resume
 *
 * Description:
 *
 ****************************************************************************/

int onewire_reset_resume(FAR struct onewire_master_s *master)
{
  int ret;
  uint8_t buf[] = { ONEWIRE_CMD_RESUME };

  ret = ONEWIRE_RESET(master->dev);
  if (ret < 0)
    {
      return ret;
    }

  return ONEWIRE_WRITE(master->dev, buf, 1);
}

/****************************************************************************
 * Name: onewire_reset_select
 *
 * Description:
 *
 ****************************************************************************/

int onewire_reset_select(FAR struct onewire_slave_s *slave)
{
  FAR struct onewire_master_s *master = slave->master;
  uint64_t tmp;
  uint8_t skip_rom[1] = { ONEWIRE_CMD_SKIP_ROM };
  uint8_t match_rom[9] = { ONEWIRE_CMD_MATCH_ROM, 0 };
  int ret;

  ret = ONEWIRE_RESET(master->dev);
  if (ret < 0)
    {
      return ret;
    }

  /* Issue skip-ROM if single slave, match-ROM otherwise. */

  if (master->nslaves == 1)
    {
      ret = ONEWIRE_WRITE(master->dev, skip_rom, sizeof(skip_rom));
    }
  else
    {
      tmp = onewire_leuint64(slave->romcode);
      memcpy(&match_rom[1], &tmp, 8);
      ret = ONEWIRE_WRITE(master->dev, match_rom, sizeof(match_rom));
    }

  return ret;
}

/****************************************************************************
 * Name: onewire_readrom
 *
 * Description:
 *
 ****************************************************************************/

int onewire_readrom(FAR struct onewire_master_s *master, FAR uint64_t *rom)
{
  uint8_t txbuf[] = { ONEWIRE_CMD_READ_ROM };
  uint8_t rxbuf[8] = { 0 };
  uint64_t tmp = -1;
  int ret;

  DEBUGASSERT(master != NULL && rom != NULL);

  /* Read ROM-code of single connected slave and check its checksum. */

  ret = ONEWIRE_RESET(master->dev);
  if (ret < 0)
    {
      return ret;
    }

  ret = ONEWIRE_WRITE(master->dev, txbuf, sizeof(txbuf));
  if (ret < 0)
    {
      return ret;
    }

  ret = ONEWIRE_READ(master->dev, rxbuf, sizeof(rxbuf));
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_DEBUG_I2C_INFO
  lib_dumpbuffer("onewire_readrom: rxbuf", rxbuf, sizeof(rxbuf));
#endif

  tmp = onewire_leuint64(*(uint64_t *)rxbuf);

#ifdef CONFIG_DEBUG_FEATURES
  if (!onewire_valid_rom(tmp))
    {
      i2cerr("ERROR: crc8 does not match!\n");
      ret = -EIO;
    }
#endif

  *rom = tmp;
  return ret;
}

/****************************************************************************
 * Name: onewire_triplet
 *
 * Description:
 *   Used by 1-wire search algorithm. Reads two bits and writes
 *   one based on comparison of read bits.
 *
 * Input Parameters:
 *   search_bit - Bit to write if both id_bit and cmp_id_bit match
 *
 * Output Parameters:
 *   taken_bit  - Bit indicating the direction where the search is
 *                progressing.
 *
 * Return Value:
 *   Number of valid bits or negative on error.
 *
 ****************************************************************************/

int onewire_triplet(FAR struct onewire_master_s *master,
                    uint8_t search_bit,
                    FAR uint8_t *taken_bit)
{
  int ret;
  int nvalid;
  uint8_t id_bit;
  uint8_t cmp_id_bit;

  ret = ONEWIRE_READBIT(master->dev, &id_bit);
  if (ret < 0)
    {
      return ret;
    }

  ret = ONEWIRE_READBIT(master->dev, &cmp_id_bit);
  if (ret < 0)
    {
      return ret;
    }

  if (id_bit == 1 && cmp_id_bit == 1)
    {
      /* No devices on bus */

      return 0;
    }
  else if (id_bit != cmp_id_bit)
    {
      /* Only one bit is valid, search to that direction. */

      nvalid = 1;
      *taken_bit = id_bit;
    }
  else
    {
      /* Two bits are valid, search to pre-determined direction. */

      nvalid = 2;
      *taken_bit = search_bit;
    }

  ret = ONEWIRE_WRITEBIT(master->dev, taken_bit);
  if (ret < 0)
    {
      return ret;
    }

  return nvalid;
}

/****************************************************************************
 * Name: onewire_search
 *
 * Description:
 *   Search all devices from a 1-wire network. This is the 1-wire search
 *   algorithm from Maxim Application Note 187.
 *
 * Input Parameters:
 *   master    - Pointer to the allocated 1-wire interface
 *   family    - Limit search to devices of matching family
 *   alarmonly - Limit search to devices on alarm state
 *   cb_search - Callback to call on each device found
 *   arg       - Argument passed to cb_search
 *
 * Return Value:
 *   Number of slaves present and matching family.
 *
 ****************************************************************************/

int onewire_search(FAR struct onewire_master_s *master,
                   int family,
                   bool alarmonly,
                   CODE void (*cb_search)(int family, uint64_t romcode, FAR void *arg),
                   FAR void *arg)
{
  FAR struct onewire_dev_s *dev = master->dev;
  uint8_t cmd[1];
  uint64_t rom = 0;
  uint64_t last_rom;
  int last_zero = -1;
  int last_bit = 64;      /* bit of last discrepancy */
  int nslaves = 0;
  int nslaves_match = 0;
  int ret;

  /* Disallow nested search. */

  DEBUGASSERT(master->insearch == false);

  /* Skip costly search if bus supports only a single slave. */

  if (master->maxslaves == 1)
    {
      ret = onewire_readrom(master, &rom);
      if (ret >= 0 && MATCH_FAMILY(rom, family))
        {
          master->insearch = true;
          cb_search(rom & 0xff, rom, arg);
          master->insearch = false;
          nslaves_match++;
        }

      return (ret < 0) ? ret : nslaves_match;
    }

  /* Select search type. */

  cmd[0] = alarmonly ? ONEWIRE_CMD_ALARM_SEARCH : ONEWIRE_CMD_SEARCH;

  while (nslaves++ < master->maxslaves)
    {
      int i;

      /* Reset bus so slaves are ready to answer our probing. */

      ret = ONEWIRE_RESET(dev);
      if (ret < 0)
        {
          return ret;
        }

      /* Send the Search-ROM command. */

      ret = ONEWIRE_WRITE(dev, cmd, sizeof(cmd));
      if (ret < 0)
        {
          return ret;
        }

      last_rom = rom;
      rom = 0;

      /* TODO: setup initial rom from family to reduce search space. */

      for (i = 0; i < 64; i++)
        {
          uint8_t search_bit;
          uint8_t taken_bit;

          /* Setup search direction. */

          if (i == last_bit)
            {
              search_bit = 1;
            }
          else if (i > last_bit)
            {
              search_bit = 0;
            }
          else
            {
              search_bit = !!(last_rom & (1 << i));
            }

          ret = onewire_triplet(master, search_bit, &taken_bit);
          if (ret <= 0)
            {
              /* Error or zero valid directions. */

              return ret;
            }

          if (ret == 2 && taken_bit == 0)
            {
              /* If both directions were valid, and we took the 0 path,
               * remember this.
               */

              last_zero = i;
            }

          /* Update rom code from taken bit. */

          rom |= (uint64_t)taken_bit << i;
        }

#ifdef CONFIG_DEBUG_FEATURES
      if (!onewire_valid_rom(rom))
        {
          i2cerr("ERROR: crc8 does not match!\n");
        }
#endif

      if (last_zero == last_bit || last_zero == -1)
        {
          i2cinfo("search complete, rom=0x%llx\n", rom);

          /* Found last device, quit searching. */

          if (MATCH_FAMILY(rom, family))
            {
              master->insearch = true;
              cb_search(rom & 0xff, rom, arg);
              master->insearch = false;
              nslaves_match++;
            }
          break;
        }

      last_bit = last_zero;

      /* Found device, will keep looking for more. */

      if (MATCH_FAMILY(rom, family))
        {
          master->insearch = true;
          cb_search(rom & 0xff, rom, arg);
          master->insearch = false;
          nslaves_match++;
        }

      /* TODO: does not handle case when there are more than maxslaves
       * slaves present in the bus.
       */
    }

  return nslaves_match;
}

/****************************************************************************
 * Name: onewire_addslave
 *
 * Description:
 *
 ****************************************************************************/

int onewire_addslave(FAR struct onewire_master_s *master,
                     FAR struct onewire_slave_s *slave)
{
  DEBUGASSERT(master != NULL && slave != NULL);
  DEBUGASSERT(slave->master == NULL);

  if (master->nslaves >= master->maxslaves)
    {
      return -EAGAIN;
    }

  /* TODO: linked list of slaves? */

  master->nslaves++;
  slave->master = master;
  return OK;
}

/****************************************************************************
 * Name: onewire_removeslave
 *
 * Description:
 *
 ****************************************************************************/

int onewire_removeslave(FAR struct onewire_master_s *master,
                        FAR struct onewire_slave_s *slave)
{
  DEBUGASSERT(master != NULL && slave != NULL);
  DEBUGASSERT(master->nslaves > 0 && slave->master == master);

  /* TODO: linked list of slaves? */

  master->nslaves--;
  slave->master = NULL;
  return OK;
}

/****************************************************************************
 * Name: onewire_initialize
 *
 * Description:
 *   Return 1-wire bus master from 1-wire lower half device.
 *
 * Input Parameters:
 *   dev       - Pointer to the allocated 1-wire lower half
 *   maxslaves - Maximum number of 1-wire slave devices
 *
 ****************************************************************************/

FAR struct onewire_master_s *
  onewire_initialize(FAR struct onewire_dev_s *dev, int maxslaves)
{
  FAR struct onewire_master_s *master;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(maxslaves > 0);

  master = (FAR struct onewire_master_s *)kmm_malloc(sizeof(struct onewire_master_s));
  if (master == NULL)
    {
      i2cerr("ERROR: Failed to allocate\n");
      return NULL;
    }

  /* Initialize the device structure */

  master->dev = dev;
  onewire_sem_init(&master->devsem);
  master->nslaves = 0;
  master->maxslaves = maxslaves;
  master->insearch = false;

  return master;
}
