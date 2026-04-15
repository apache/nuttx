/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_ulp.c
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
#include <nuttx/debug.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "ulp_lp_core.h"
#include "ulp/ulp_var_map.h"
#include "esp_ulp.h"
#include "driver/rtc_io.h"

#include "ulp_lp_core_lp_uart_shared.h"
#include "hal/uart_ll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_ulp_ioctl(struct file *filep, int cmd, unsigned long arg);
static int esp_ulp_write(struct file *filep,
                         const char *buffer,
                         size_t buflen);
int esp_ulp_load_bin(const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_esp_ulp_fops =
{
  .write = esp_ulp_write, /* write */
  .ioctl = esp_ulp_ioctl, /* ioctl */
};

/* Configuration for ULP LP Core */

ulp_lp_core_cfg_t cfg =
{
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_HP_CPU
  .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_TIMER
  .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER,
  .lp_timer_sleep_duration_us =
    CONFIG_ESPRESSIF_ULP_WAKEUP_SLEEP_DURATION_US,
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO
  .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_IO,
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_UART
  .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_UART,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_UART
/****************************************************************************
 * Name: esp_ulp_wakeup_lpuart_init
 *
 * Description:
 *   Configure LPUART to use as ULP wakeup source.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

static int esp_ulp_wakeup_lpuart_init(void)
{
  uart_wakeup_cfg_t wake_up_cfg =
    {
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LPUART_ACTIVE_EDGE_THRESHOLD_MODE
      .wakeup_mode = UART_WK_MODE_ACTIVE_THRESH,
      .rx_edge_threshold =
        CONFIG_ESPRESSIF_ULP_WAKEUP_LPUART_ACTIVE_EDGE_THRESHOLD
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LPUART_FIFO_THRESHOLD_MODE
      .wakeup_mode = UART_WK_MODE_FIFO_THRESH,
      .rx_fifo_threshold = CONFIG_ESPRESSIF_ULP_WAKEUP_LPUART_FIFO_THRESHOLD
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LPUART_START_BIT_MODE
      .wakeup_mode = UART_WK_MODE_START_BIT,
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LPUART_CHAR_SEQ_MODE
      .wakeup_mode = UART_WK_MODE_CHAR_SEQ,
      .wake_chars_seq = CONFIG_ESPRESSIF_ULP_WAKEUP_LPUART_CHAR_SEQ
#endif
    };

  return uart_wakeup_setup(LP_UART_NUM_0, &wake_up_cfg);
}
#endif

#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO
/****************************************************************************
 * Name: esp_ulp_get_high_io_mask
 *
 * Description:
 *   Get ULP wakeup high IO mask value from configured wakeup pins.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A 64-bit unsigned integer where each bit corresponds to an RTC GPIO pin
 *   that has been configured high as an ULP wakeup source.
 *
 ****************************************************************************/

static uint64_t esp_ulp_get_high_io_mask(void)
{
  uint64_t io_mask = 0;
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO0_HIGH
  io_mask |= BIT(0);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO1_HIGH
  io_mask |= BIT(1);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO2_HIGH
  io_mask |= BIT(2);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO3_HIGH
  io_mask |= BIT(3);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO4_HIGH
  io_mask |= BIT(4);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO5_HIGH
  io_mask |= BIT(5);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO6_HIGH
  io_mask |= BIT(6);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO7_HIGH
  io_mask |= BIT(7);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO8_HIGH
  io_mask |= BIT(8);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO9_HIGH
  io_mask |= BIT(9);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO10_HIGH
  io_mask |= BIT(10);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO11_HIGH
  io_mask |= BIT(11);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO12_HIGH
  io_mask |= BIT(12);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO13_HIGH
  io_mask |= BIT(13);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO14_HIGH
  io_mask |= BIT(14);
#endif

  return io_mask;
}

/****************************************************************************
 * Name: esp_ulp_get_low_io_mask
 *
 * Description:
 *   Get ULP wakeup low IO mask value from configured wakeup pins.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A 64-bit unsigned integer where each bit corresponds to an RTC GPIO pin
 *   that has been configured low as an ULP wakeup source.
 *
 ****************************************************************************/

static uint64_t esp_ulp_get_low_io_mask(void)
{
  uint64_t io_mask = 0;
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO0_LOW
  io_mask |= BIT(0);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO1_LOW
  io_mask |= BIT(1);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO2_LOW
  io_mask |= BIT(2);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO3_LOW
  io_mask |= BIT(3);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO4_LOW
  io_mask |= BIT(4);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO5_LOW
  io_mask |= BIT(5);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO6_LOW
  io_mask |= BIT(6);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO7_LOW
  io_mask |= BIT(7);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO8_LOW
  io_mask |= BIT(8);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO9_LOW
  io_mask |= BIT(9);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO10_LOW
  io_mask |= BIT(10);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO11_LOW
  io_mask |= BIT(11);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO12_LOW
  io_mask |= BIT(12);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO13_LOW
  io_mask |= BIT(13);
#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO14_LOW
  io_mask |= BIT(14);
#endif

  return io_mask;
}

/****************************************************************************
 * Name: esp_ulp_wakeup_io_init
 *
 * Description:
 *   Configure GPIO to use as ULP wakeup source.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

static int esp_ulp_wakeup_io_init(void)
{
  int ret = OK;
  int pin_mask;
  gpio_int_type_t wake_up_type;
  uint64_t io_mask_low = esp_ulp_get_low_io_mask();
  uint64_t io_mask_high = esp_ulp_get_high_io_mask();

  for (int i = 0; i < CONFIG_SOC_RTCIO_PIN_COUNT; i++)
    {
      pin_mask = BIT(i);
      if ((io_mask_low & pin_mask) != 0 ||
          (io_mask_high & pin_mask) != 0)
        {
          ret = esp_rtcio_config_gpio(i, ESP_RTC_GPIO_MODE_INPUT_OUTPUT);
          if (ret != OK)
            {
              ferr("Failed to configure pin: %d\n", i);
              return ret;
            }

          wake_up_type = GPIO_INTR_HIGH_LEVEL;
          if ((io_mask_low & pin_mask) != 0)
            {
              wake_up_type = GPIO_INTR_LOW_LEVEL;
            }

          rtc_gpio_wakeup_enable(i, wake_up_type);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_ulp_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor
 *   cmd   - The ioctl command
 *   arg   - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_ulp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  int ret = 0;
  int index = -1;
  struct symtab_s *sym = (struct symtab_s *)arg;
  int var_map_size = sizeof(ulp_var_map) / sizeof(ulp_var_map[0]);

  DEBUGASSERT(sym);

  /* Decode and dispatch the driver-specific IOCTL command */

  for (int i = 0; i < var_map_size; i++)
    {
      if (strcmp(ulp_var_map[i].sym.sym_name, sym->sym_name) == 0)
        {
          index = i;
          break;
        }
    }

  if (index == -1)
    {
      ferr("Symbol name does not exist\n");
      return ERROR;
    }

  switch (cmd)
    {
      case FIONREAD:
        memcpy((void *)sym->sym_value, ulp_var_map[index].sym.sym_value,
               ulp_var_map[index].size);
        break;

      case FIONWRITE:
        memcpy((void *)ulp_var_map[index].sym.sym_value, sym->sym_value,
               ulp_var_map[index].size);
        break;

      default:
        ferr("Unrecognized IOCTL command: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_ulp_write
 *
 * Description:
 *   Load binary data into ULP.
 *
 * Input Parameters:
 *   filep  - The pointer of file
 *   buffer - Buffer that includes binary to run on ULP.
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_ulp_write(struct file *filep,
                         const char *buffer,
                         size_t buflen)
{
  UNUSED(filep);
  return esp_ulp_load_bin(buffer, buflen);
}

/****************************************************************************
 * Name: esp_ulp_register
 *
 * Description:
 *   This function registers ULP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_ulp_register(void)
{
  register_driver("/dev/ulp", &g_esp_ulp_fops, 0666, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_ulp_load_bin
 *
 * Description:
 *   Load binary data into ULP.
 *
 * Input Parameters:
 *   buffer - Buffer that includes binary to run on ULP.
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_ulp_load_bin(const char *buffer, size_t buflen)
{
  int ret = ERROR;
  ret = ulp_lp_core_load_binary((const uint8_t *)buffer, buflen);
  ulp_lp_core_run(&cfg);
  return ret;
}

/****************************************************************************
 * Name: esp_ulp_init
 *
 * Description:
 *   Initialize ULP co-processor
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_ulp_init(void)
{
  int ret = OK;
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_IO
  ret = esp_ulp_wakeup_io_init();
  if (ret != OK)
    {
      return ret;
    }

#endif
#ifdef CONFIG_ESPRESSIF_ULP_WAKEUP_LP_UART
  esp_ulp_wakeup_lpuart_init();
  if (ret != OK)
    {
      ferr("Failed to LP UART wakeup\n");
      return ret;
    }

#endif
  esp_ulp_register();
  return ret;
}
