/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_dma.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <stdint.h>
#include <nuttx/arch.h>
#include <nuttx/config.h>

#include "cxd56_audio_config.h"
#include "cxd56_audio_dma.h"
#include "cxd56_audio_mic.h"
#include "cxd56_audio_ac_reg.h"
#include "cxd56_audio_bca_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register type. */

enum audio_irq_reg_type_e
{
  INT_EN1_REG = 0,
  INT_POL_REG,
  INT_IRQ1_REG
};

/* INT_EN1 */

#define CXD56_INTC_BASE      0xe0045000
#define INT_EN1_REG_ADDR     (CXD56_INTC_BASE + 0x10 + 3 * 4)
#define INT_EN1_BIT_AU0      6
#define INT_EN1_BIT_AU1      7
#define INT_EN1_BIT_AU2      8
#define INT_EN1_BIT_AU3      9

/* INT_POL */

#define INT_POL_REG          (CXD56_INTC_BASE + 0x20 + 3 * 4)
#define INT_POL_BIT_AU0      6
#define INT_POL_BIT_AU1      7
#define INT_POL_BIT_AU2      8
#define INT_POL_BIT_AU3      9

/* INT_IRQ1 */

#define INT_IRQ1_REG_ADDR    (CXD56_INTC_BASE + 0x30 + 3 * 4)
#define INT_IRQ1_BIT_AU0     6
#define INT_IRQ1_BIT_AU1     7
#define INT_IRQ1_BIT_AU2     8
#define INT_IRQ1_BIT_AU3     9

#define DMA_HANDLE_MAX_NUM  (CXD56_AUDIO_DMAC_I2S1_DOWN + 1)

#define DMA_TIMEOUT_CNT      10000
#define DMA_START_RETRY_CNT  10
#define DMA_SMP_WAIT_HIRES   10 /* usec per sample. */
#define DMA_SMP_WAIT_NORMALT 40 /* usec per sample. */

/* Private Macros */

#define SET_DMA_ACT(_path_)  g_dma_act_status |= (1 << _path_)
#define CLR_DMA_ACT(_path_)  g_dma_act_status &= ~(1 << _path_)
#define IS_DMA_ACT(_path_)   ((g_dma_act_status & (1 << _path_)) != 0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_dma_act_status = 0;
static cxd56_audio_dma_cb_t g_dma_cb[DMA_HANDLE_MAX_NUM];

static bool s_work_arroud_dmac[DMA_HANDLE_MAX_NUM] =
{
  true,
  true,
  true
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t read_int_reg(uint32_t reg)
{
  volatile uint32_t *addr;
  uint32_t data = 0;

  if (reg == INT_EN1_REG)
    {
      addr = (volatile uint32_t *)INT_EN1_REG_ADDR;
    }
  else if (reg == INT_IRQ1_REG)
    {
      addr = (volatile uint32_t *)INT_IRQ1_REG_ADDR;
    }
  else
    {
      addr = (volatile uint32_t *)INT_POL_REG;
    }

  data = *addr;

  return data;
}

static uint32_t write_int_reg(uint32_t reg, uint32_t data)
{
  volatile uint32_t *addr;

  if (reg == INT_EN1_REG)
    {
      addr = (volatile uint32_t *)INT_EN1_REG_ADDR;
      *addr = data;
    }

  return 0;
}

static CXD56_AUDIO_ECODE get_dma_handle(cxd56_audio_dma_path_t path,
                                        FAR cxd56_audio_dma_t *handle)
{
  switch (path)
    {
      case CXD56_AUDIO_DMA_PATH_MIC_TO_MEM:
        *handle = CXD56_AUDIO_DMAC_MIC;
        break;

      case CXD56_AUDIO_DMA_PATH_MEM_TO_BUSIF1:
        *handle = CXD56_AUDIO_DMAC_I2S0_DOWN;
        break;

      case CXD56_AUDIO_DMA_PATH_MEM_TO_BUSIF2:
        *handle = CXD56_AUDIO_DMAC_I2S1_DOWN;
        break;

      default:
        return CXD56_AUDIO_ECODE_DMA_PATH_INV;
    }

  return CXD56_AUDIO_ECODE_OK;
}

static CXD56_AUDIO_ECODE get_dma_path(cxd56_audio_dma_t handle,
                                      FAR cxd56_audio_dma_path_t *path)
{
  switch (handle)
    {
      case CXD56_AUDIO_DMAC_MIC:
        *path = CXD56_AUDIO_DMA_PATH_MIC_TO_MEM;
        break;

      case CXD56_AUDIO_DMAC_I2S0_DOWN:
        *path = CXD56_AUDIO_DMA_PATH_MEM_TO_BUSIF1;
        break;

      case CXD56_AUDIO_DMAC_I2S1_DOWN:
        *path = CXD56_AUDIO_DMA_PATH_MEM_TO_BUSIF2;
        break;

      default:
        return CXD56_AUDIO_ECODE_DMA_HANDLE_INV;
    }

  return CXD56_AUDIO_ECODE_OK;
}

static CXD56_AUDIO_ECODE start_dma(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_start_dma(handle, false);

  return CXD56_AUDIO_ECODE_OK;
}

static CXD56_AUDIO_ECODE exec_dma_ch_sync_workaround(
  cxd56_audio_dma_t handle)
{
  int timeout_cnt = 0;
  int retry_cnt;

  cxd56_audio_clkmode_t clk_mode = cxd56_audio_config_get_clkmode();

  /* Execute out-of-sync workaround.
   * 1. Clear smp interrupt status
   * 2. Read until smp interrupt state is true
   * 3. Reset channel select setting
   * 4. Start dma transfer
   * It needs to be less than 9 us by the processing so far.
   * If it does not fit below 9 us, err_int is generated, so retry.
   */

  /* Mask dma done interrupt. */

  cxd56_audio_bca_reg_mask_done_int(handle);

  for (retry_cnt = 0; retry_cnt < DMA_START_RETRY_CNT; retry_cnt++)
    {
      /* Clear interrupt status */

      cxd56_audio_bca_reg_clear_err_int(handle);
      cxd56_audio_bca_reg_clear_smp_int(handle);

      /* Lock interrupt */

      up_irq_disable();
      sched_lock();

      /* Wait smp interrupt. */

      for (timeout_cnt = 0; timeout_cnt < DMA_TIMEOUT_CNT; timeout_cnt++)
        {
          if (cxd56_audio_bca_reg_is_smp_int(handle))
            {
              break;
            }
        }

      if (timeout_cnt == DMA_TIMEOUT_CNT)
        {
          return CXD56_AUDIO_ECODE_DMA_SMP_TIMEOUT;
        }

      /* Reset Channel select. */

      cxd56_audio_bca_reg_reset_chsel(handle);

      /* Start dma. */

      cxd56_audio_bca_reg_start_dma(handle, false);

      /* Unlock interrupt */

      sched_unlock();
      up_irq_enable();

      /* Wait for 1sample tramsfer. */

      if (clk_mode == CXD56_AUDIO_CLKMODE_HIRES)
        {
          up_udelay(DMA_SMP_WAIT_HIRES);
        }
      else
        {
          up_udelay(DMA_SMP_WAIT_NORMALT);
        }

      /* Check whether an error interrupt has occurred. */

      if (cxd56_audio_bca_reg_is_err_int(handle))
        {
          cxd56_audio_bca_reg_stop_dma(handle);
          cxd56_audio_bca_reg_clear_err_int(handle);

          for (timeout_cnt = 0; timeout_cnt < DMA_TIMEOUT_CNT; timeout_cnt++)
            {
              if (DMA_MSTATE_BUF_EMPTY ==
                  cxd56_audio_bca_reg_get_mon_state_buf(handle))
                {
                  if (cxd56_audio_bca_reg_is_done_int(handle))
                    {
                      cxd56_audio_bca_reg_clear_done_int(handle);
                      break;
                    }
                }
            }
        }
      else
        {
          break;
        }
    }

  /* Unmask dma done interrupt. */

  cxd56_audio_bca_reg_unmask_done_int(handle);

  return CXD56_AUDIO_ECODE_OK;
}

static CXD56_AUDIO_ECODE start_dma_workaround(cxd56_audio_dma_t handle)
{
  /* There are two workarounds.
   * One is a workaround in which the error interrupt of
   * dma is incorrectly generated.
   * The other is a workaround for the problem that the channel
   * is out of sync.
   * Because both require processing at the beginning of dma,
   * call out workaround with out-of-sync from the workaround
   * for interrupt error.
   */

  /* Execute error interrupt workaround.
   * 1. Mask dma error interrupt
   * 2. Wait 77 cycle after dma transfer starts
   * 3. Clear interrupt status
   * 4. Unmask dma error interrupt
   */

  cxd56_audio_bca_reg_mask_err_int(handle);

  /* Transfer start and wait processing of dma is done
   * in out-of-sync workaround.
   */

  CXD56_AUDIO_ECODE ret = exec_dma_ch_sync_workaround(handle);

  cxd56_audio_bca_reg_clear_err_int(handle);
  cxd56_audio_bca_reg_unmask_err_int(handle);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_dma_get_handle(cxd56_audio_dma_path_t path,
                                             FAR cxd56_audio_dma_t *handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  /* Check error of argument */

  if (handle == NULL)
    {
      return CXD56_AUDIO_ECODE_DMA_HANDLE_NULL;
    }

  /* Check duplicate order */

  if (IS_DMA_ACT(path))
    {
      return CXD56_AUDIO_ECODE_DMA_PATH_DUP;
    }

  ret = get_dma_handle(path, handle);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  SET_DMA_ACT(path);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_free_handle(FAR cxd56_audio_dma_t handle)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;
  cxd56_audio_dma_path_t path;

  ret = get_dma_path(handle, &path);
  if (CXD56_AUDIO_ECODE_OK != ret)
    {
      return ret;
    }

  CLR_DMA_ACT(path);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_init(cxd56_audio_dma_t handle,
                                       cxd56_audio_samp_fmt_t fmt,
                                       FAR uint8_t *ch_num)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;
  uint32_t ch_setting;

  cxd56_audio_ac_reg_enable_dma();

  if (handle == CXD56_AUDIO_DMAC_MIC)
    {
      ret = cxd56_audio_mic_set_seloutch(*ch_num, fmt);
      if (CXD56_AUDIO_ECODE_OK != ret)
        {
          return ret;
        }
    }
  else
    {
      /* Set Stereo channel number. */

      *ch_num = 2;
    }

  if (fmt == CXD56_AUDIO_SAMP_FMT_24)
    {
      cxd56_audio_bca_reg_en_fmt24(handle, *ch_num);
    }
  else
    {
      cxd56_audio_bca_reg_en_fmt16(handle, *ch_num);
    }

  /* Clear interrupt state. */

  cxd56_audio_bca_reg_clear_done_int(handle);
  cxd56_audio_bca_reg_clear_err_int(handle);
  cxd56_audio_bca_reg_clear_cmb_int(handle);

  /* Enable interrupt. */

  cxd56_audio_bca_reg_unmask_done_int(handle);

  /* cxd56_audio_bca_reg_mask_done_int(handle); TODO: polling */

  /* Enable interrupt. */

  cxd56_audio_bca_reg_unmask_err_int(handle);
  cxd56_audio_bca_reg_unmask_cmb_int(handle);
  cxd56_audio_bca_reg_unmask_bus_int(handle);

  /* Check channel setting. */

  ch_setting = cxd56_audio_bca_reg_get_mon_state_err(handle);
  switch (ch_setting)
    {
      case DMA_MSTATE_ERR_NO_ENABLE_CH:
        return CXD56_AUDIO_ECODE_DMA_CH_NO_ENABLE;

      case DMA_MSTATE_ERR_CH1_4_INVALID:
        return CXD56_AUDIO_ECODE_DMA_CH1_4_INV;

      case DMA_MSTATE_ERR_CH5_8_INVALID:
        return CXD56_AUDIO_ECODE_DMA_CH5_8_INV;

      default:
        break;
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_set_cb(cxd56_audio_dma_t handle,
                                         FAR cxd56_audio_dma_cb_t cb)
{
  g_dma_cb[handle] = cb;

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_get_mstate(cxd56_audio_dma_t handle,
                                    FAR cxd56_audio_dma_mstate_t *state)
{
  cxd56_audio_bca_reg_get_dma_mstate(handle, state);

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_en_dmaint(void)
{
  volatile uint32_t int_en;

  int_en = read_int_reg(INT_EN1_REG);

  int_en |= (1 << INT_EN1_BIT_AU0);
  int_en |= (1 << INT_EN1_BIT_AU1);
  int_en |= (1 << INT_EN1_BIT_AU2);
  int_en |= (1 << INT_EN1_BIT_AU3);

  write_int_reg(INT_EN1_REG, int_en);

  /* Enable bus error interrupt. */

  cxd56_audio_bca_reg_en_bus_err_int();

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_dis_dmaint(void)
{
  volatile uint32_t int_en;

  int_en = read_int_reg(INT_EN1_REG);

  int_en &= ~(1 << INT_EN1_BIT_AU0);
  int_en &= ~(1 << INT_EN1_BIT_AU1);
  int_en &= ~(1 << INT_EN1_BIT_AU2);
  int_en &= ~(1 << INT_EN1_BIT_AU3);

  write_int_reg(INT_EN1_REG, int_en);

  /* Disable bus error interrupt. */

  cxd56_audio_bca_reg_dis_bus_err_int();

  return CXD56_AUDIO_ECODE_OK;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_start(cxd56_audio_dma_t handle,
                                        uint32_t addr,
                                        uint32_t sample)
{
  CXD56_AUDIO_ECODE ret = CXD56_AUDIO_ECODE_OK;

  if (DMA_CMD_FIFO_NOT_FULL != cxd56_audio_bca_reg_get_dma_state(handle))
    {
      return CXD56_AUDIO_ECODE_DMA_BUSY;
    }

  cxd56_audio_bca_reg_set_start_addr(handle, addr);
  cxd56_audio_bca_reg_set_sample_no(handle, sample);

  if (s_work_arroud_dmac[handle])
    {
      s_work_arroud_dmac[handle] = false;
      ret = start_dma_workaround(handle);
    }
  else
    {
      ret = start_dma(handle);
    }

  return ret;
}

CXD56_AUDIO_ECODE cxd56_audio_dma_stop(cxd56_audio_dma_t handle)
{
  cxd56_audio_bca_reg_stop_dma(handle);
  s_work_arroud_dmac[handle] = true;

  return CXD56_AUDIO_ECODE_OK;
}

void cxd56_audio_dma_int_handler(void)
{
  uint32_t int_irq  = read_int_reg(INT_IRQ1_REG);
  uint32_t int_ac   = cxd56_audio_bca_reg_get_dma_done_state_mic();
  uint32_t int_i2s  = cxd56_audio_bca_reg_get_dma_done_state_i2s1();
  uint32_t int_i2s2 = cxd56_audio_bca_reg_get_dma_done_state_i2s2();

  /* AUDIO_INT_AC : check interruption from mic */

  if ((int_irq & (1 << INT_IRQ1_BIT_AU0)) && (int_ac != 0))
    {
      /* Clear interrupt. */

      cxd56_audio_bca_reg_clear_dma_done_state_mic(int_ac);

      /* Check done complete state. */

      if (int_ac & DMA_STATE_BIT_AC_DONE)
        {
          (*g_dma_cb[CXD56_AUDIO_DMAC_MIC])(CXD56_AUDIO_DMAC_MIC,
                                              CXD56_AUDIO_ECODE_DMA_CMPLT);
        }

      /* Check transfer err state. */

      if (int_ac & DMA_STATE_BIT_AC_ERR)
        {
          cxd56_audio_bca_reg_mask_err_int(CXD56_AUDIO_DMAC_MIC);

          cxd56_audio_bca_reg_clear_err_int(CXD56_AUDIO_DMAC_MIC);

          (*g_dma_cb[CXD56_AUDIO_DMAC_MIC])(CXD56_AUDIO_DMAC_MIC,
                                              CXD56_AUDIO_ECODE_DMA_TRANS);
        }

      /* Check bus err state. */

      if (int_ac & DMA_STATE_BIT_AC_CMB)
        {
          cxd56_audio_bca_reg_mask_cmb_int(CXD56_AUDIO_DMAC_MIC);

          cxd56_audio_bca_reg_clear_cmb_int(CXD56_AUDIO_DMAC_MIC);

          (*g_dma_cb[CXD56_AUDIO_DMAC_MIC])(CXD56_AUDIO_DMAC_MIC,
                                              CXD56_AUDIO_ECODE_DMA_CMB);
        }
    }

  /* AUDIO_INT_I2S1 : check interruption from I2S-1 */

  if ((int_irq & (1 << INT_IRQ1_BIT_AU1)) && (int_i2s != 0))
    {
      /* Clear interrupt. */

      cxd56_audio_bca_reg_clear_dma_done_state_i2s1(int_i2s);

      /* Check done complete state. */

      if (int_i2s & DMA_STATE_BIT_I2S_OUT_DONE)
        {
          (*g_dma_cb[CXD56_AUDIO_DMAC_I2S0_DOWN])(CXD56_AUDIO_DMAC_I2S0_DOWN,
                                                CXD56_AUDIO_ECODE_DMA_CMPLT);
        }

      /* Check transfer err state. */

      if (int_i2s & DMA_STATE_BIT_I2S_OUT_ERR)
        {
          cxd56_audio_bca_reg_mask_err_int(CXD56_AUDIO_DMAC_I2S0_DOWN);

          cxd56_audio_bca_reg_clear_err_int(CXD56_AUDIO_DMAC_I2S0_DOWN);

          (*g_dma_cb[CXD56_AUDIO_DMAC_I2S0_DOWN])(CXD56_AUDIO_DMAC_I2S0_DOWN,
                                                CXD56_AUDIO_ECODE_DMA_TRANS);
        }

      /* Check bus err state. */

      if (int_i2s & DMA_STATE_BIT_I2S_CMB)
        {
          cxd56_audio_bca_reg_mask_cmb_int(CXD56_AUDIO_DMAC_I2S0_DOWN);

          cxd56_audio_bca_reg_clear_cmb_int(CXD56_AUDIO_DMAC_I2S0_DOWN);

          (*g_dma_cb[CXD56_AUDIO_DMAC_I2S0_DOWN])(CXD56_AUDIO_DMAC_I2S0_DOWN,
                                                CXD56_AUDIO_ECODE_DMA_CMB);
        }
    }

  /* AUDIO_INT_I2S2 : check interruption from I2S-2 */

  if ((int_irq & (1 << INT_IRQ1_BIT_AU2)) && (int_i2s2 != 0))
    {
      /* Clear interrupt. */

      cxd56_audio_bca_reg_clear_dma_done_state_i2s2(int_i2s2);

      /* Check done complete state. */

      if (int_i2s2 & DMA_STATE_BIT_I2S_OUT_DONE)
        {
          (*g_dma_cb[CXD56_AUDIO_DMAC_I2S1_DOWN])
          (CXD56_AUDIO_DMAC_I2S1_DOWN,
           CXD56_AUDIO_ECODE_DMA_CMPLT);
        }

      /* Check transfer err state. */

      if (int_i2s2 & DMA_STATE_BIT_I2S_OUT_ERR)
        {
          cxd56_audio_bca_reg_mask_err_int(CXD56_AUDIO_DMAC_I2S1_DOWN);

          cxd56_audio_bca_reg_clear_err_int(CXD56_AUDIO_DMAC_I2S1_DOWN);

          (*g_dma_cb[CXD56_AUDIO_DMAC_I2S1_DOWN])
          (CXD56_AUDIO_DMAC_I2S1_DOWN,
           CXD56_AUDIO_ECODE_DMA_TRANS);
        }

      /* Check bus err state. */

      if (int_i2s2 & DMA_STATE_BIT_I2S_CMB)
        {
          cxd56_audio_bca_reg_mask_cmb_int(CXD56_AUDIO_DMAC_I2S1_DOWN);

          cxd56_audio_bca_reg_clear_cmb_int(CXD56_AUDIO_DMAC_I2S1_DOWN);

          (*g_dma_cb[CXD56_AUDIO_DMAC_I2S1_DOWN])(CXD56_AUDIO_DMAC_I2S1_DOWN,
                                                 CXD56_AUDIO_ECODE_DMA_CMB);
        }
    }

  if (int_irq & (1 << INT_IRQ1_BIT_AU3))
    {
      uint32_t int_au = cxd56_audio_bca_reg_get_int_status();

      if (int_au != 0)
        {
          cxd56_audio_bca_reg_clear_int_status(int_au);
        }
    }
}
