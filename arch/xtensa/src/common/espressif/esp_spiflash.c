/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_spiflash.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/init.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/mutex.h>
#include <sys/types.h>
#include <inttypes.h>
#include <sched/sched.h>

#include "esp_flash_internal.h"
#include "esp_flash.h"
#include "esp_flash_encrypt.h"
#include "esp_private/cache_utils.h"

#if defined(CONFIG_ARCH_CHIP_ESP32)
#  include "esp32_irq.h"
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#  include "esp32s2_irq.h"
#else
#  include "esp32s3_irq.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ESP32_PARTITION_TABLE) || defined(CONFIG_ESP32S3_PARTITION_TABLE)
#  error "Partition table requires legacy SPI Flash driver"
#endif

#if defined(CONFIG_ESP32S3_SPIRAM)
#  error "SPIRAM requires legacy SPI Flash driver"
#endif

#if defined(CONFIG_ARCH_CHIP_ESP32)
#  define esp_intr_noniram_enable  esp32_irq_noniram_enable
#  define esp_intr_noniram_disable esp32_irq_noniram_disable
#elif defined(CONFIG_ARCH_CHIP_ESP32S2)
#  define esp_intr_noniram_disable esp32s2_irq_noniram_disable
#  define esp_intr_noniram_enable  esp32s2_irq_noniram_enable
#else
#  define esp_intr_noniram_enable  esp32s3_irq_noniram_enable
#  define esp_intr_noniram_disable esp32s3_irq_noniram_disable
#endif

#define SPIFLASH_OP_TASK_STACKSIZE 768

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_flash_op_block_task
 *
 * Description:
 *   Disable the non-IRAM interrupts on the other core (the one that isn't
 *   handling the SPI flash operation) and notify that the SPI flash
 *   operation can start. Wait on a busy loop until it's finished and then
 *   re-enable the non-IRAM interrupts.
 *
 * Input Parameters:
 *   argc          - Not used.
 *   argv          - Not used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static int spi_flash_op_block_task(int argc, char *argv[])
{
  struct tcb_s *tcb = this_task();
  int cpu = this_cpu();

  for (; ; )
    {
      DEBUGASSERT((1 << cpu) & tcb->affinity);
      /* Wait for a request from the other CPU to suspend interrupts
       * and cache on this CPU.
       */

      nxsem_wait(&g_cpu_prepare_sem[cpu]);

      sched_lock();
      esp_intr_noniram_disable();

      s_flash_op_complete = false;
      s_flash_op_can_start = true;
      while (!s_flash_op_complete)
        {
          /* Wait for a request to restore interrupts and cache on this CPU.
           * This indicates SPI Flash operation is complete.
           */
        }

      esp_intr_noniram_enable();
      sched_unlock();
    }

  return OK;
}

/****************************************************************************
 * Name: spiflash_init_spi_flash_op_block_task
 *
 * Description:
 *   Starts a kernel thread that waits for a semaphore indicating that a SPI
 *   flash operation is going to take place in the other CPU.
 *
 * Input Parameters:
 *   cpu - The CPU core that will run the created task to wait on a busy
 *         loop while the SPI flash operation finishes
 *
 * Returned Value:
 *   0 (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int spiflash_init_spi_flash_op_block_task(int cpu)
{
  FAR struct tcb_s *tcb;
  int ret;
  char *argv[2];
  char arg1[32];
  cpu_set_t cpuset;

  snprintf(arg1, sizeof(arg1), "%p", &cpu);
  argv[0] = arg1;
  argv[1] = NULL;

  /* Allocate a TCB for the new task. */

  tcb = kmm_zalloc(sizeof(struct tcb_s));
  if (!tcb)
    {
      serr("ERROR: Failed to allocate TCB\n");
      return -ENOMEM;
    }

  /* Setup the task type */

  tcb->flags = TCB_FLAG_TTYPE_KERNEL | TCB_FLAG_FREE_TCB;

  /* Initialize the task */

  ret = nxtask_init((FAR struct task_tcb_s *)tcb, "spiflash_op",
                    SCHED_PRIORITY_MAX,
                    NULL, SPIFLASH_OP_TASK_STACKSIZE,
                    spi_flash_op_block_task, argv, environ, NULL);
  if (ret < OK)
    {
      kmm_free(tcb);
      return ret;
    }

  /* Set the affinity */

  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);
  tcb->affinity = cpuset;

  /* Activate the task */

  nxtask_activate(tcb);

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_spiflash_read
 *
 * Description:
 *   Read data from flash.
 *
 * Parameters:
 *   address - Source address of the data in flash.
 *   buffer  - Pointer to the destination buffer.
 *   length  - Length of data in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int esp_spiflash_read(uint32_t address, void *buffer,
                      uint32_t length)
{
  int ret = OK;

  if (!esp_flash_encryption_enabled())
    {
      ret = esp_flash_read(NULL, buffer, address, length);
    }
  else
    {
#ifdef CONFIG_ARCH_CHIP_ESP32S2
      ferr("encryption not supported on ESP32-S2\n");
      ret = ERROR;
#else
      ret = esp_flash_read_encrypted(NULL, address, buffer, length);
#endif
    }

  if (ret != 0)
    {
      ferr("esp_flash_read failed %d", ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_spiflash_write
 *
 * Description:
 *   Write data to Flash.
 *
 * Parameters:
 *   address - Destination address in Flash.
 *   buffer  - Pointer to the source buffer.
 *   length  - Length of data, in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int esp_spiflash_write(uint32_t address, const void *buffer,
                       uint32_t length)
{
  int ret = OK;

  if (!esp_flash_encryption_enabled())
    {
      ret = esp_flash_write(NULL, buffer, address, length);
    }
  else
    {
#ifdef CONFIG_ARCH_CHIP_ESP32S2
      ferr("encryption not supported on ESP32-S2\n");
      ret = -ERROR;
#else
      ret = esp_flash_write_encrypted(NULL, address, buffer, length);
#endif
    }

  if (ret != 0)
    {
      ferr("ERROR: esp_flash_write failed %d", ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_spiflash_erase
 *
 * Description:
 *   Erase data from Flash.
 *
 * Parameters:
 *   address - Start address of the data in Flash.
 *   length  - Length of the data to erase.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int esp_spiflash_erase(uint32_t address, uint32_t length)
{
  int ret = OK;

  ret = esp_flash_erase_region(NULL, address, length);
  if (ret != 0)
    {
      ferr("ERROR: erase failed: ret=%d", ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_spiflash_init
 *
 * Description:
 *   Initialize ESP SPI flash driver.
 *   SPI Flash actual chip initialization initial is done by esp_start on
 *   STARTUP_FN hook.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_spiflash_init(void)
{
  int ret = OK;
  int cpu;

#ifdef CONFIG_SMP
  sched_lock();

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      ret = spiflash_init_spi_flash_op_block_task(cpu);
      if (ret != OK)
        {
          return ret;
        }
    }

  sched_unlock();
#else
  UNUSED(cpu);
#endif

  return ret;
}
