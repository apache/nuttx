/****************************************************************************
 * arch/arm/src/rp2040/rp2040_cyw43439.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/ieee80211/bcmf_gspi.h>

#include "barriers.h"

#include "rp2040_cyw43439.h"
#include "rp2040_pio.h"
#include "rp2040_pio_instructions.h"

#ifdef CONFIG_NDEBUG
#  define PRINT_GSPI(block)
#else
void bcmf_hexdump(uint8_t *data, unsigned int len, unsigned long offset);
bool  g_print_gspi = false;
#  define PRINT_GSPI(block) if (g_print_gspi) { block }
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GSPI_CLOCK_FREQ 31250000  /* Hz  (Max: 50MHz) */

#define PIO_WRAP_TARGET 0
#define PIO_WRAP        5

#define TX_FIFO_SIZE    4

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

typedef struct dma_info_s
{
  sem_t         sem;
  uint8_t       status;
} dma_info_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the PIO program to write and then read from the data pin.
 *
 * The X register is the output bit count register.  It should be set
 * to one less than the total number of BITS to be transmitted.
 *
 * The Y register is the input bit count register.  It too must be set
 * to one less thant the total number of bits to be read.
 *
 * The PIO's state machine is set up to auto-pull data from the input
 * fifo whenever the output shift register is empty.  This happens at
 * the start and every 32 bits thereafter.
 *
 * The state machine also auto-pushes whenever we have a full 32 bits
 * in the input shift register.  To make sure we receive all the data,
 * we need to make sure that the read bit count is a multiple of 32.
 * (Y = 32*N - 1  for some integer N)
 */

static const uint16_t cyw_program_instructions[] =
{
  0x6001, /* 0: out  pins, 1     side 0 # Write one bit                  */
  0x1040, /* 1: jmp  x--, 0      side 1 # Loop until write count is zero */
  0xe080, /* 2: set  pindirs, 0  side 0 # Make data pin an input         */
  0xb042, /* 3: nop              side 1 # Keep clock in sync             */
  0x4001, /* 4: in   pins, 1     side 0 # Read 1 bit                     */
  0x1084, /* 5: jmp  y--, 4      side 1 # Loop until read count is zero  */
};

static const rp2040_pio_program_t pio_program =
{
  .instructions = cyw_program_instructions,
  .length       = 6,    /* Six, count 'em, six.    */
  .origin       = -1,   /* Put it wherever it fits */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dma_complete
 *
 * Description:
 *   Called on completion of the DMA transfer.
 *
 * Input Parameters:
 *   handle - handle to our DMA channel
 *   status - status of the transfer
 *   arg    - Pointer to dma info structure.
 *
 ****************************************************************************/

static void dma_complete(DMA_HANDLE handle, uint8_t status, void *arg)
{
  FAR dma_info_t *dma_info = arg;

  /* Remember the status and post the dma complete event. */

  dma_info->status = status;
  nxsem_post(&(dma_info->sem));
}

/****************************************************************************
 * Name: my_init
 *
 * Description:
 *   Connect to and initialize the cyw43439.
 *
 ****************************************************************************/

static int my_init(FAR gspi_dev_t  *gspi)
{
  FAR rp2040_gspi_t *rp_io = (FAR rp2040_gspi_t *)(gspi->io_dev);

  uint32_t              divisor;
  irqstate_t            flags;
  rp2040_pio_sm_config  config =
    {
      0
    };

  /* Make sure the cyw43439 chip is deselected, and off.
   * so we know it is reset
   */

  rp2040_gpio_put(rp_io->gpio_select, true);  /* deselect  */
  rp2040_gpio_put(rp_io->gpio_on, false);     /* power off */

  /* Pull the data line low so that we initialise to gSPI mode */

  rp2040_gpio_init(rp_io->gpio_data);
  rp2040_gpio_setdir(rp_io->gpio_data, true);
  rp2040_gpio_put(rp_io->gpio_data, false);

  usleep(50000); /* Leave off for at least 50ms. */

  rp2040_gpio_put(rp_io->gpio_on, true);     /* power on */

  usleep(50000); /* Wait a bit to let the power come up. */

  /* Don't let anyone else grab a PIO while we are doing so. */

  flags = enter_critical_section();

  /* Find a PIO instance and load program. */

  for (rp_io->pio = 0; rp_io->pio < RP2040_PIO_NUM; ++rp_io->pio)
    {
      /* Try to claim a state machine. */

      rp_io->pio_sm = rp2040_pio_claim_unused_sm(rp_io->pio, false);

      /* If we did not get one try the next pio block, if any. */

      if (rp_io->pio_sm < 0) continue;

      /* See if we have space in this block to load our program. */

      if (rp2040_pio_can_add_program(rp_io->pio, &pio_program))
        {
          /* Great! load the program and exit the pio choice loop. */

          rp_io->pio_location = rp2040_pio_add_program(rp_io->pio,
                                                      &pio_program);

          break;
        }

      /* Oops -- no room at the inn!  Release sm and try next pio. */

      rp2040_pio_sm_unclaim(rp_io->pio, rp_io->pio_sm);
    }

  if (rp_io->pio >= RP2040_PIO_NUM)
    {
      leave_critical_section(flags);
      return -ENOMEM;
    }

  leave_critical_section(flags);

  /* ==== Configure the PIO State Machine ==== */

  /* Configure pins that are used by PIO. */

  rp2040_pio_gpio_init(rp_io->pio, rp_io->gpio_data);
  rp2040_pio_gpio_init(rp_io->pio, rp_io->gpio_clock);

  rp2040_gpio_set_input_hysteresis_enabled(rp_io->gpio_data, true);

  rp2040_gpio_set_slew_fast(rp_io->gpio_clock, true);
  rp2040_gpio_set_drive_strength(rp_io->gpio_clock,
                                 RP2040_PADS_BANK0_GPIO_DRIVE_12MA);

  /* Set the clock divisor as appropriate for our system clock
   * speed, so the pio clock runs the requested bit clock rate.
   */

  divisor = ((uint64_t)BOARD_SYS_FREQ << 8) / (2 * GSPI_CLOCK_FREQ);

  rp2040_sm_config_set_clkdiv_int_frac(&config,
                                       divisor >> 8,
                                       divisor & 0xff);

  /* Set the wrap points as required by the program. */

  rp2040_sm_config_set_wrap(&config,
                            rp_io->pio_location + PIO_WRAP_TARGET,
                            rp_io->pio_location + PIO_WRAP);

  /* set to shift left, 32 bits, with autopull/push  */

  rp2040_sm_config_set_in_shift(&config, false, true, 32);
  rp2040_sm_config_set_out_shift(&config, false, true, 32);

  /* Configure a single mandatory side-set pin. */

  rp2040_sm_config_set_sideset(&config, 1, false, false);

  /* Configure our GPIO clock pin as side-set output. */

  rp2040_sm_config_set_sideset_pins(&config, rp_io->gpio_clock);

  /* Configure out GPIO data pin as an OUT pin, SET pin, and IN pin */

  rp2040_sm_config_set_out_pins(&config, rp_io->gpio_data, 1);
  rp2040_sm_config_set_set_pins(&config, rp_io->gpio_data, 1);
  rp2040_sm_config_set_in_pins (&config, rp_io->gpio_data);

  /* Load the configuration into the state machine. */

  rp2040_pio_sm_init(rp_io->pio,
                     rp_io->pio_sm,
                     rp_io->pio_location,
                     &config);

  /* Disable the input synchronizers on the data pin */

  rp2040_pio_set_input_sync_bypass(rp_io->pio, rp_io->gpio_data, true);

  wlinfo("finished\n");

  return OK;
}

/****************************************************************************
 * Name: my_set_isr
 *
 * Description:
 *   Setup the data line interrupt service routine.
 *
 ****************************************************************************/

static int my_set_isr(FAR gspi_dev_t  *gspi,
                      xcpt_t           thread_isr,
                      FAR void        *thread_isr_arg)
{
  FAR rp2040_gspi_t *rp_io = (FAR rp2040_gspi_t *)(gspi->io_dev);

  /* Set up, but do not enable, interrupt service for the data pin */

  rp2040_gpio_irq_attach(rp_io->gpio_intr,
                         RP2040_GPIO_INTR_LEVEL_HIGH,
                         thread_isr,
                         thread_isr_arg);

  wlinfo("attached\n");

  return OK;
}

/****************************************************************************
 * Name: my_interrupt_enable
 *
 * Description:
 *   Setup the data line isr.
 *
 ****************************************************************************/

static int my_interrupt_enable(FAR gspi_dev_t  *gspi,
                               bool             enable)
{
  FAR rp2040_gspi_t *rp_io         = (FAR rp2040_gspi_t *)(gspi->io_dev);
  static int         disable_count = 0;

  if (enable)
    {
      if (--disable_count <= 0) rp2040_gpio_enable_irq(rp_io->gpio_intr);
    }
  else
    {
      rp2040_gpio_disable_irq(rp_io->gpio_intr);
      ++disable_count;
    }

  return OK;
}

/****************************************************************************
 * Name: my_deinit
 *
 * Description:
 *   Disconnect from cyw43439 and cleanup.
 *
 ****************************************************************************/

static int my_deinit(FAR gspi_dev_t *gspi)
{
  FAR rp2040_gspi_t *rp_io = (FAR rp2040_gspi_t *)(gspi->io_dev);

  rp2040_gpio_irq_attach(rp_io->gpio_data,
                         RP2040_GPIO_INTR_EDGE_LOW,
                         NULL,
                         NULL);

  rp2040_pio_sm_set_enabled(rp_io->pio, rp_io->pio_sm, false);
  rp2040_pio_remove_program(rp_io->pio, &pio_program, rp_io->pio_location);
  rp2040_pio_sm_unclaim(rp_io->pio, rp_io->pio_sm);

  /* Turn the power off to the cyw43439. */

  rp2040_gpio_put(rp_io->gpio_on, false);

  return OK;
}

/****************************************************************************
 * Name: my_write
 *
 * Description:
 *   write data to the cyw43439
 *
 ****************************************************************************/

static int my_write(FAR struct gspi_dev_s  *gspi,
                    bool                    increment,
                    enum gspi_cmd_func_e    function,
                    uint32_t                address,
                    uint16_t                length,
                    FAR const uint32_t     *data)
{
  FAR rp2040_gspi_t *rp_io      = (FAR rp2040_gspi_t *)(gspi->io_dev);
  dma_info_t         dma_info;
  DMA_HANDLE         xfer_dma   = rp2040_dmachannel();
  uint32_t           command    =    (0x1                  << 31)
                                   | ((increment ? 1 : 0)  << 30)
                                   | ((function & 0x3)     << 28)
                                   | ((address  & 0x1ffff) << 11)
                                   | (length    & 0x7ff);

  dma_config_t       dma_config =
    {
      .size   = RP2040_DMA_SIZE_WORD,
      .noincr = false,
      .dreq   = rp2040_pio_get_dreq(rp_io->pio,
                                    rp_io->pio_sm,
                                    true),
    };

  PRINT_GSPI(
    printf("------ cmd: 0x%08lx [W %d %d 0x%05lX %d]\n", command,
                                                         increment,
                                                         function,
                                                         address,
                                                         length);
    bcmf_hexdump((void *)data, length, (uint32_t)data);
  )

  /* Claim the exclusive lock */

  nxsem_wait(&gspi->exclsem);

  /* Reset the PIO state machine just to be sure. */

  rp2040_pio_sm_set_enabled(rp_io->pio, rp_io->pio_sm, false);

  rp2040_pio_sm_clear_fifos(rp_io->pio, rp_io->pio_sm);

  rp2040_pio_sm_restart(rp_io->pio, rp_io->pio_sm);

  rp2040_pio_sm_clkdiv_restart(rp_io->pio, rp_io->pio_sm);

  rp2040_pio_sm_exec(rp_io->pio,
                     rp_io->pio_sm,
                     pio_encode_jmp(rp_io->pio_location));

  /* Set the PIO X and Y registers.
   *
   * We load X (the TX bit length) with one less than the number of
   * bits to transmit to the chip. This length includes the 32-bit
   * command word and all the 32-bit data words. Since the length
   * parameter is a byte count we round up just to be sure.
   *
   * We load Y (the RX bit length) with zero as we're not reading
   * any data.
   *
   * This is slightly magical.  The way we load the X is to first
   * push the the number of bits to transmit onto the transmit fifo.
   * Then we force the PIO state machine to execute the instruction
   * "out x, 32" which transfers the word from the output shift
   * register (OSR) to the X register.  When this instruction executes
   * the PIO will notice that the OSR is empty, so will automatically
   * pull a value (the one we just added) from the input fifo.
   *
   * Loading the Y works the same way.
   */

  rp2040_pio_sm_put(rp_io->pio, rp_io->pio_sm, 32 * ((length + 3) / 4) + 31);
  rp2040_pio_sm_exec(rp_io->pio, rp_io->pio_sm, pio_encode_out(pio_x, 32));

  rp2040_pio_sm_put(rp_io->pio, rp_io->pio_sm, 0);
  rp2040_pio_sm_exec(rp_io->pio, rp_io->pio_sm, pio_encode_out(pio_y, 32));

  /* Disable interrupts so data won't trigger interrupt. */

  my_interrupt_enable(gspi, false);

  /* Make sure the clock and data pins direction set to output. */

  rp2040_pio_sm_set_pindirs_with_mask(rp_io->pio,
                                      rp_io->pio_sm,
                                        (1 << rp_io->gpio_data)
                                      | (1 << rp_io->gpio_clock),
                                        (1 << rp_io->gpio_data)
                                      | (1 << rp_io->gpio_clock));

  /* Make sure there is nothing in the fifos before starting DMA. */

  rp2040_pio_sm_clear_fifos(rp_io->pio, rp_io->pio_sm);

  /* Load the command into the transmit fifo. */

  if (function == gspi_f0_bus_rev16)
    {
      __asm ("rev16 %0, %0" : "+l" (command) : :);
    }

  __asm ("rev %0, %0" : "+l" (command) : :);

  putreg32(command, RP2040_PIO_TXF(rp_io->pio, rp_io->pio_sm));

  /* Initialize and start the transmit DMA.  It will
   * keep adding data to the TX_FIFO until all data is sent.
   */

  nxsem_init(&dma_info.sem, 0, 0);

  rp2040_dmastop(xfer_dma);

  rp2040_txdmasetup(xfer_dma,
                    (uintptr_t) RP2040_PIO_TXF(rp_io->pio,
                                                rp_io->pio_sm),
                    (uintptr_t) data,
                    (length + 3) & 0xfffffffc,
                    dma_config);

  modifyreg32(rp2040_dma_register(xfer_dma,
                                  RP2040_DMA_CTRL_TRIG_OFFSET),
                                  RP2040_DMA_CTRL_TRIG_BSWAP,
                                  RP2040_DMA_CTRL_TRIG_BSWAP);

  rp2040_dmastart(xfer_dma, dma_complete, &dma_info);

  /* Assert gpio_select by pulling line low */

  rp2040_gpio_put(rp_io->gpio_select, false);
  ARM_DMB();

  /* Enable the state machine.  This starts the pio program running */

  rp2040_pio_sm_set_enabled(rp_io->pio, rp_io->pio_sm, true);

  /* Wait for transfer to complete */

  nxsem_wait(&dma_info.sem);

  /* At this point all the data has been queued but my not have all been
   * sent.  We know that the PIO program will make the data line an input
   * once all the data is sent so we'll check for this.
   */

  while (getreg32(RP2040_IO_BANK0_GPIO_STATUS(rp_io->gpio_data))
                  & RP2040_IO_BANK0_GPIO_STATUS_OEFROMPERI)
    {
      /* Just busy wait -- testing indicates a worst case of
       * 20 loops (100 instructions).
       */
    }

  /* Un-assert select by pulling line high. */

  ARM_DMB();
  rp2040_gpio_put(rp_io->gpio_select, true);

  /* Free the DMA controller */

  rp2040_dmafree(xfer_dma);
  nxsem_destroy(&dma_info.sem);

  /* Disable the PIO */

  rp2040_pio_sm_set_enabled(rp_io->pio, rp_io->pio_sm, false);

  /* At this point the data pin is input so it should have been
   * pulled high by rp2040's gpio pullup.
   */

  my_interrupt_enable(gspi, true);

  /* Release the exclusive lock */

  nxsem_post(&gspi->exclsem);

  return dma_info.status;
}

/****************************************************************************
 * Name: my_read
 *
 * Description:
 *   read data to the cyw43439
 *
 ****************************************************************************/

static int my_read(FAR struct gspi_dev_s  *gspi,
                   bool                    increment,
                   enum gspi_cmd_func_e    function,
                   uint32_t                address,
                   uint16_t                length,
                   FAR uint32_t           *buffer)
{
  FAR rp2040_gspi_t *rp_io      = (FAR rp2040_gspi_t *)(gspi->io_dev);
  dma_info_t         dma_info;
  DMA_HANDLE         xfer_dma   = rp2040_dmachannel();
  DMA_HANDLE         ctrl_dma   = rp2040_dmachannel();
  uint32_t           temp_word;
  uint32_t           bit_length;
  uint32_t           command    =    ((increment ? 1 : 0) << 30)
                                   | ((function & 0x3)    << 28)
                                   | ((address & 0x1ffff) << 11)
                                   | (length & 0x7ff);

  uint32_t           pacing = rp2040_pio_get_dreq(rp_io->pio,
                                                  rp_io->pio_sm,
                                                  false);

  dma_control_block_t ctrl_blks[] =
    {
      /* For F1 transfers we read 1 word that we throw away */

        {
          rp2040_dma_ctrl_blk_ctrl(ctrl_dma,
                                    RP2040_DMA_SIZE_WORD,
                                    pacing,
                                    0),
          (uintptr_t) RP2040_PIO_RXF(rp_io->pio, rp_io->pio_sm),
          (uintptr_t) &temp_word,
          1,
        },

      /* Read requested data into output buffer */

        {
          rp2040_dma_ctrl_blk_ctrl(ctrl_dma,
                                    RP2040_DMA_SIZE_WORD,
                                    pacing,
                                      RP2040_DMA_CTRL_TRIG_INCR_WRITE
                                    | RP2040_DMA_CTRL_TRIG_BSWAP),
          (uintptr_t) RP2040_PIO_RXF(rp_io->pio, rp_io->pio_sm),
          (uintptr_t) buffer,
          (length + 3) / 4,
        },

      RP2040_DMA_CTRL_BLOCK_END
    };

  PRINT_GSPI(
    printf("------ cmd: 0x%08lx [R %d %d 0x%05lX %d]\n", command,
                                                         increment,
                                                         function,
                                                         address,
                                                         length);
  )

  /* Claim the exclusive lock */

  nxsem_wait(&gspi->exclsem);

  /* Reset the PIO state machine just to be sure. */

  rp2040_pio_sm_set_enabled(rp_io->pio, rp_io->pio_sm, false);

  rp2040_pio_sm_clear_fifos(rp_io->pio, rp_io->pio_sm);

  rp2040_pio_sm_restart(rp_io->pio, rp_io->pio_sm);

  rp2040_pio_sm_clkdiv_restart(rp_io->pio, rp_io->pio_sm);

  rp2040_pio_sm_exec(rp_io->pio,
                     rp_io->pio_sm,
                     pio_encode_jmp(rp_io->pio_location));

  /* Set the PIO X and Y registers.
   *
   * We load X (the TX bit length) with one less than the number of
   * bits to transmit to the chip. Since we only send the 32-bit command
   * word we set X to 31.
   *
   * We load Y with the number of bits to read.  This is based on the
   * byte count in "length" which we round up to a 32-bit boundry so the
   * pio program will be sure to autopush the final data to the output fifo.
   *
   * This is slightly magical.  The way we load the X is to first
   * push the the number of bits to transmit onto the transmit fifo.
   * Then we force the PIO state machine to execute the instruction
   * "out x, 32" which transfers the word from the output shift
   * register (OSR) to the X register.  When this instruction executes
   * the PIO will notice that the OSR is empty, so will automatically
   * pull a value (the one we just added) from the input fifo.
   *
   * Loading the Y works the same way.
   */

  rp2040_pio_sm_put(rp_io->pio, rp_io->pio_sm, 31);
  rp2040_pio_sm_exec(rp_io->pio, rp_io->pio_sm, pio_encode_out(pio_x, 32));

  /* RX bit length is 32 bits for each 4 bytes requested. */

  bit_length = 32 * ((length + 3) / 4);

  /* For F1 reads add 32 bits for delay */

  if (function == gspi_f1_backplane)
    {
      bit_length += 32;
    }

  rp2040_pio_sm_put(rp_io->pio, rp_io->pio_sm, bit_length);
  rp2040_pio_sm_exec(rp_io->pio, rp_io->pio_sm, pio_encode_out(pio_y, 32));

  /* Disable interrupts so data won't trigger interrupt. */

  my_interrupt_enable(gspi, false);

  /* Make sure the clock and data pins direction set to output. */

  rp2040_pio_sm_set_pindirs_with_mask(rp_io->pio,
                                      rp_io->pio_sm,
                                        (1 << rp_io->gpio_data)
                                      | (1 << rp_io->gpio_clock),
                                        (1 << rp_io->gpio_data)
                                      | (1 << rp_io->gpio_clock));

  /* Make sure there is nothing in the fifos before starting DMA. */

  rp2040_pio_sm_clear_fifos(rp_io->pio, rp_io->pio_sm);

  /* Load the command into the transmit fifo. */

  if (function == gspi_f0_bus_rev16)
    {
      __asm ("rev16 %0, %0" : "+l" (command) : :);
    }

  __asm ("rev %0, %0" : "+l" (command) : :);

  putreg32(command, RP2040_PIO_TXF(rp_io->pio, rp_io->pio_sm));

  /* Initialize and start the control DMA.  It will
   * use the xfer_dma to transfer data from the chip.
   */

  nxsem_init(&dma_info.sem, 0, 0);

  rp2040_dmastop(ctrl_dma);
  rp2040_dmastop(xfer_dma);

  /* Check the function bits if the command word to see if
   * this is an F1 read.  If it is we throw away the first
   * four bytes read as this is a delay word.
   */

  rp2040_ctrl_dmasetup(ctrl_dma,
                       xfer_dma,
                       &ctrl_blks[function == 1 ? 0 : 1],
                       dma_complete,
                       &dma_info);

  rp2040_dmastart(ctrl_dma, NULL, NULL);

  /* Assert gpio_select by pulling line low */

  rp2040_gpio_put(rp_io->gpio_select, false);
  ARM_DMB();

  /* Enable the state machine.  This starts the pio program running */

  rp2040_pio_sm_set_enabled(rp_io->pio, rp_io->pio_sm, true);

  /* Wait for transfer to complete */

  nxsem_wait(&dma_info.sem);

  PRINT_GSPI(
    bcmf_hexdump((void *)buffer, length, (uint32_t)buffer);
  )

  /* Un-assert select by pulling line high. */

  ARM_DMB();
  rp2040_gpio_put(rp_io->gpio_select, true);

  /* Free the DMA controllers */

  rp2040_dmafree(ctrl_dma);
  rp2040_dmafree(xfer_dma);

  nxsem_destroy(&dma_info.sem);

  /* Disable the PIO */

  rp2040_pio_sm_set_enabled(rp_io->pio, rp_io->pio_sm, false);

  /* At this point the data pin is input so it should have been
   * pulled high by rp2040's gpio pullup.
   */

  my_interrupt_enable(gspi, true);

  /* Release the exclusive lock */

  nxsem_post(&gspi->exclsem);

  return dma_info.status;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_cyw_setup
 *
 * Description:
 *   Initialize the cyw43439 private data and PIO communication.
 ****************************************************************************/

gspi_dev_t *rp2040_cyw_setup(uint8_t gpio_on,
                             uint8_t gpio_select,
                             uint8_t gpio_data,
                             uint8_t gpio_clock,
                             uint8_t gpio_intr)
{
  FAR gspi_dev_t    *gspi;
  FAR rp2040_gspi_t *rp_io;
  int                err;

  wlinfo("entered.\n");

  gspi = kmm_zalloc(sizeof(gspi_dev_t));

  if (gspi == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  rp_io = kmm_zalloc(sizeof(rp2040_gspi_t));

  if (rp_io == NULL)
    {
      kmm_free(gspi);
      set_errno(ENOMEM);
      return NULL;
    }

  gspi->init             = my_init;
  gspi->deinit           = my_deinit;
  gspi->set_isr          = my_set_isr;
  gspi->interrupt_enable = my_interrupt_enable;
  gspi->write            = my_write;
  gspi->read             = my_read;
  gspi->io_dev           = rp_io;

  rp_io->gpio_on     = gpio_on;
  rp_io->gpio_select = gpio_select;
  rp_io->gpio_data   = gpio_data;
  rp_io->gpio_clock  = gpio_clock;
  rp_io->gpio_intr   = gpio_intr;

  nxsem_init(&gspi->exclsem, 0, 1);

  /* Initialize the cyw43439 power-on and chip select lines. */

  rp2040_gpio_init(gpio_on);
  rp2040_gpio_setdir(gpio_on, true);
  rp2040_gpio_put(gpio_on, false);  /* set low to turn wifi chip off */

  rp2040_gpio_init(gpio_select);
  rp2040_gpio_setdir(gpio_select, true);
  rp2040_gpio_put(gpio_select, true); /* set high to deselect chip */

  err = bcmf_gspi_initialize(gspi);

  if (err != OK)
    {
      kmm_free(gspi);
      kmm_free(rp_io);

      set_errno(err);
      return NULL;
    }

  wlinfo("setup complete. gspi = 0x%08lX\n", (uint32_t) gspi);

  return (FAR void *) gspi;
}

/****************************************************************************
 * Name: rp2040_cyw_remove
 *
 * Description:
 *   Deinitialize the cyw43439 PIO communication.
 ****************************************************************************/

void rp2040_cyw_remove(gspi_dev_t *gspi)
{
  /* gspi_deregister((FAR gspi_dev_t *)gspi); */
}

/****************************************************************************
 * Name: bcmf_board_etheraddr
 ****************************************************************************/

bool bcmf_board_etheraddr(struct ether_addr *ethaddr)
{
  return false;
}
