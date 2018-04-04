/****************************************************************************
 * arch/arm/src/lc823450/lc823450_syscontrol.h
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SYSCONTROL_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SYSCONTROL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_SYSCONTROL_REGBASE 0x40080000

#define CORECNT (LC823450_SYSCONTROL_REGBASE + 0x0000)
#define CORECNT_C1CLKEN  (1 << 0)
#define CORECNT_C1RSTN   (1 << 1)

#define REMAP (LC823450_SYSCONTROL_REGBASE + 0x0008)

#define MEMEN4 (LC823450_SYSCONTROL_REGBASE + 0x0024)
#define   MEMEN4_HWAIT   (1 << 0)
#define   MEMEN4_DWAIT   (1 << 1)

#define LSISTBY (LC823450_SYSCONTROL_REGBASE + 0x0028)
#define   LSISTBY_STBYA  (1 << 0)  /* Audio */
#define   LSISTBY_STBYB  (1 << 1)  /* SRAM */
#define   LSISTBY_STBYC  (1 << 2)  /* SRAM */
#define   LSISTBY_STBYD  (1 << 3)  /* SRAM + ROM (DSP) */
#define   LSISTBY_STBYE  (1 << 4)  /* USB */
#define   LSISTBY_STBYG  (1 << 6)  /* S-Flash cache */
#define   LSISTBY_STBYH  (1 << 7)  /* SD/MS */
#define   LSISTBY_STBYI  (1 << 8)  /* Internal ROM */

#define ISOCNT (LC823450_SYSCONTROL_REGBASE + 0x002c)

#define LOCKUPR (LC823450_SYSCONTROL_REGBASE + 0x0044)
#define   LOCKUPR_LOCKUPR0  (1 << 0)
#define   LOCKUPR_LOCKUPR1  (1 << 1)

#define MODEM (LC823450_SYSCONTROL_REGBASE + 0x00FC)
#define   MODEM_MAV_MASK (15 << 24)
#define   MODEM_MAV_ES1  (1 << 24)
#define   MODEM_MAV_ES2  (2 << 24)

#define MCLKCNTBASIC (LC823450_SYSCONTROL_REGBASE + 0x0100)
#define   MCLKCNTBASIC_EXTMEMC_CLKEN  (1 << 0)
#define   MCLKCNTBASIC_SFIF_CLKEN     (1 << 1)
#define   MCLKCNTBASIC_USBHOST_CLKEN  (1 << 2)
#define   MCLKCNTBASIC_DMAC_CLKEN     (1 << 4)
#define   MCLKCNTBASIC_CACHE_CLKEN    (1 << 10)
#define   MCLKCNTBASIC_USBDEV_CLKEN   (1 << 11)

#define MCLKCNTEXT1 (LC823450_SYSCONTROL_REGBASE + 0x0104)
#define   MCLKCNTEXT1_MTM0_CLKEN   (1 << 0)
#define   MCLKCNTEXT1_MTM1_CLKEN   (1 << 1)
#define   MCLKCNTEXT1_MTM2_CLKEN   (1 << 2)
#define   MCLKCNTEXT1_MTM3_CLKEN   (1 << 3)
#define   MCLKCNTEXT1_PTM0_CLKEN   (1 << 4)
#define   MCLKCNTEXT1_PTM1_CLKEN   (1 << 5)
#define   MCLKCNTEXT1_PTM2_CLKEN   (1 << 6)
#define   MCLKCNTEXT1_SDIF0_CLKEN  (1 << 8)
#define   MCLKCNTEXT1_SDIF1_CLKEN  (1 << 9)
#define   MCLKCNTEXT1_SDIF2_CLKEN  (1 << 10)
#define   MCLKCNTEXT1_MSHOST_CLKEN (1 << 11)
#define   MCLKCNTEXT1_MSPB_CLKEN   (1 << 12)
#define   MCLKCNTEXT1_MTM0C_CLKEN  (1 << 24)
#define   MCLKCNTEXT1_MTM1C_CLKEN  (1 << 25)
#define   MCLKCNTEXT1_MTM2C_CLKEN  (1 << 26)
#define   MCLKCNTEXT1_MTM3C_CLKEN  (1 << 27)
#define   MCLKCNTEXT1_PTM0C_CLKEN  (1 << 28)
#define   MCLKCNTEXT1_PTM1C_CLKEN  (1 << 29)
#define   MCLKCNTEXT1_PTM2C_CLKEN  (1 << 30)

#define MCLKCNTEXT3 (LC823450_SYSCONTROL_REGBASE + 0x0108)
#define   MCLKCNTEXT3_AUDIOBUF_CLKEN   (1 << 0)

#define MCLKCNTEXT4 (LC823450_SYSCONTROL_REGBASE + 0x010c)
#define   MCLKCNTEXT4_SDRAMC_CLKEN0  (1 << 0)
#define   MCLKCNTEXT4_SDRAMC_CLKEN1  (1 << 1)

#define MCLKCNTAPB (LC823450_SYSCONTROL_REGBASE + 0x0110)
#define   MCLKCNTAPB_PORT0_CLKEN   (1 << 0)
#define   MCLKCNTAPB_PORT1_CLKEN   (1 << 1)
#define   MCLKCNTAPB_PORT2_CLKEN   (1 << 2)
#define   MCLKCNTAPB_PORT3_CLKEN   (1 << 3)
#define   MCLKCNTAPB_PORT4_CLKEN   (1 << 4)
#define   MCLKCNTAPB_PORT5_CLKEN   (1 << 5)
#define   MCLKCNTAPB_ADC_CLKEN     (1 << 6)
#define   MCLKCNTAPB_SPI_CLKEN     (1 << 7)
#define   MCLKCNTAPB_I2C0_CLKEN    (1 << 8)
#define   MCLKCNTAPB_I2C1_CLKEN    (1 << 9)
#define   MCLKCNTAPB_UART0_CLKEN   (1 << 10)
#define   MCLKCNTAPB_UART1_CLKEN   (1 << 11)
#define   MCLKCNTAPB_UART2_CLKEN   (1 << 12)
#define   MCLKCNTAPB_RTC_CLKEN     (1 << 13)
#define   MCLKCNTAPB_GPO_CLKEN     (1 << 14)
#define   MCLKCNTAPB_UART0IF_CLKEN (1 << 24)
#define   MCLKCNTAPB_UART1IF_CLKEN (1 << 25)
#define   MCLKCNTAPB_UART2IF_CLKEN (1 << 26)

#define MRSTCNTBASIC (LC823450_SYSCONTROL_REGBASE + 0x0114)
#define   MRSTCNTBASIC_EXTMEMC_RSTB (1 << 0)
#define   MRSTCNTBASIC_SFIF_RSTB    (1 << 1)
#define   MRSTCNTBASIC_USB_RSTB     (1 << 2)
#define   MRSTCNTBASIC_IRQCNT_RSTB  (1 << 3)
#define   MRSTCNTBASIC_DMAC_RSTB    (1 << 4)
#define   MRSTCNTBASIC_MUTEX_RSTB   (1 << 8)
#define   MRSTCNTBASIC_DSPCMD_RSTB  (1 << 9)
#define   MRSTCNTBASIC_CACHE_RSTB   (1 << 10)
#define   MRSTCNTBASIC_USBDEV_RSTB  (1 << 11)

#define MRSTCNTEXT1 (LC823450_SYSCONTROL_REGBASE + 0x0118)
#define   MRSTCNTEXT1_MTM0_RSTB   (1 << 0)
#define   MRSTCNTEXT1_MTM1_RSTB   (1 << 1)
#define   MRSTCNTEXT1_MTM2_RSTB   (1 << 2)
#define   MRSTCNTEXT1_MTM3_RSTB   (1 << 3)
#define   MRSTCNTEXT1_PTM0_RSTB   (1 << 4)
#define   MRSTCNTEXT1_PTM1_RSTB   (1 << 5)
#define   MRSTCNTEXT1_PTM2_RSTB   (1 << 6)
#define   MRSTCNTEXT1_SDIF0_RSTB  (1 << 8)
#define   MRSTCNTEXT1_SDIF1_RSTB  (1 << 9)
#define   MRSTCNTEXT1_SDIF2_RSTB  (1 << 10)
#define   MRSTCNTEXT1_MSIF_RSTB   (1 << 11)

#define MRSTCNTEXT3 (LC823450_SYSCONTROL_REGBASE + 0x011c)
#define   MRSTCNTEXT3_AUDIOBUF_RSTB   (1 << 0)

#define MRSTCNTEXT4 (LC823450_SYSCONTROL_REGBASE + 0x0120)
#define   MRSTCNTEXT4_SDRAMC_RSTB (1 << 0)

#define MRSTCNTAPB (LC823450_SYSCONTROL_REGBASE + 0x0124)
#define   MRSTCNTAPB_PORT0_RSTB  (1 << 0)
#define   MRSTCNTAPB_PORT1_RSTB  (1 << 1)
#define   MRSTCNTAPB_PORT2_RSTB  (1 << 2)
#define   MRSTCNTAPB_PORT3_RSTB  (1 << 3)
#define   MRSTCNTAPB_PORT4_RSTB  (1 << 4)
#define   MRSTCNTAPB_PORT5_RSTB  (1 << 5)
#define   MRSTCNTAPB_ADC_RSTB    (1 << 6)
#define   MRSTCNTAPB_SPI_RSTB    (1 << 7)
#define   MRSTCNTAPB_I2C0_RSTB   (1 << 8)
#define   MRSTCNTAPB_I2C1_RSTB   (1 << 9)
#define   MRSTCNTAPB_UART0_RSTB  (1 << 10)
#define   MRSTCNTAPB_UART1_RSTB  (1 << 11)
#define   MRSTCNTAPB_UART2_RSTB  (1 << 12)
#define   MRSTCNTAPB_RTC_RSTB    (1 << 13)

#define PMDCNT0  (LC823450_SYSCONTROL_REGBASE + 0x0400)
#define PMDCNT1  (LC823450_SYSCONTROL_REGBASE + 0x0404)
#define PMDCNT2  (LC823450_SYSCONTROL_REGBASE + 0x0408)
#define PMDCNT3  (LC823450_SYSCONTROL_REGBASE + 0x040c)
#define PMDCNT4  (LC823450_SYSCONTROL_REGBASE + 0x0410)
#define PMDCNT5  (LC823450_SYSCONTROL_REGBASE + 0x0414)

#define PUDCNT0  (LC823450_SYSCONTROL_REGBASE + 0x044c)
#define PUDCNT1  (LC823450_SYSCONTROL_REGBASE + 0x0450)
#define PUDCNT2  (LC823450_SYSCONTROL_REGBASE + 0x0454)
#define PUDCNT3  (LC823450_SYSCONTROL_REGBASE + 0x0458)
#define PUDCNT4  (LC823450_SYSCONTROL_REGBASE + 0x045c)
#define PUDCNT5  (LC823450_SYSCONTROL_REGBASE + 0x0460)
#define PUDCNT6  (LC823450_SYSCONTROL_REGBASE + 0x0464)

#define PTDRVCNT0  (LC823450_SYSCONTROL_REGBASE + 0x0430)
#define PTDRVCNT1  (LC823450_SYSCONTROL_REGBASE + 0x0434)
#define PTDRVCNT2  (LC823450_SYSCONTROL_REGBASE + 0x0438)
#define PTDRVCNT3  (LC823450_SYSCONTROL_REGBASE + 0x043c)
#define PTDRVCNT4  (LC823450_SYSCONTROL_REGBASE + 0x0440)
#define PTDRVCNT5  (LC823450_SYSCONTROL_REGBASE + 0x0444)
#define PTDRVCNT6  (LC823450_SYSCONTROL_REGBASE + 0x0448)

#define SDCTL  (LC823450_SYSCONTROL_REGBASE + 0x0800)
#define   SDCTL_COREVLT         (1 << 31)
#define   SDCTL_MMCVLT0_18V     (1 << 4)
#define   SDCTL_ACSMODE0_MASK   (7 << 1)
#define   SDCTL_ACSMODE0_HS     (2 << 1)
#define   SDCTL_ACSMODE0_MMCDDR (4 << 1)
#define   SDCTL_SDMMC0_MMC      (1 << 0)

#define DREQ0_3  (LC823450_SYSCONTROL_REGBASE + 0x808)
#define DREQ4_7  (LC823450_SYSCONTROL_REGBASE + 0x80c)
#define DREQ8_C (LC823450_SYSCONTROL_REGBASE + 0x810)
#define DREQD_F (LC823450_SYSCONTROL_REGBASE + 0x814)
#define CACHE_CTL  (LC823450_SYSCONTROL_REGBASE + 0x0818)
#define   CACHE_CTL_USE    (1 << 0)
#define BMODE_CNT  (LC823450_SYSCONTROL_REGBASE + 0x81c)
#define   BMODE_CNT_BMODE0EN  (1 << 0)
#define BMODE_DT  (LC823450_SYSCONTROL_REGBASE + 0x820)
#define   BMODE_DT_BMODE0DT      (1 << 0)
#define   BMODE_DT_XTALINFO_MASK (3 << 4)
#define   BMODE_DT_XTALINFO_20   (2 << 4)
#define   BMODE_DT_XTALINFO_24   (0 << 4)
#define USBCNT  (LC823450_SYSCONTROL_REGBASE + 0x834)
#define   USBCNT_ANPD          (1 << 1)
#define   USBCNT_CLK24MHZ      (1 << 2)
#define   USBCNT_CLK20MHZ      (6 << 2)
#define   USBCNT_CLK_MASK      (7 << 2)
#define   USBCNT_CRYCNTSW24MHZ (1 << 5)
#define   USBCNT_CRYCNTSW20MHZ (1 << 5)
#define   USBCNT_CRYCNTSW_MASK (7 << 5)
#define   USBCNT_VBUS_VALID    (1 << 8)
#define   USBCNT_RSM_CONT      (1 << 11)
#define USBSTAT  (LC823450_SYSCONTROL_REGBASE + 0x838)
#define    USBSTAT_LINESTE_MASK (3 << 0)
#define    USBSTAT_LINESTE_0    (1 << 0)
#define    USBSTAT_LINESTE_1    (1 << 1)

#define SFIFSEL (LC823450_SYSCONTROL_REGBASE + 0x83c)
#define SDRAMIFSEL (LC823450_SYSCONTROL_REGBASE + 0x840)

#define I2CMODE  (LC823450_SYSCONTROL_REGBASE + 0x844)
#define   I2CMODE0 (1 << 0)
#define   I2CMODE1 (1 << 1)

/* GPIO */

#define PORT0_BASE  0x40081000
#define rP0DT  (PORT0_BASE + 0x0000 + 0x04)
#define rP1DT  (PORT0_BASE + 0x1000 + 0x04)
#define rP2DT  (PORT0_BASE + 0x2000 + 0x04)
#define rP3DT  (PORT0_BASE + 0x3000 + 0x04)
#define rP4DT  (PORT0_BASE + 0x4000 + 0x04)
#define rP5DT  (PORT0_BASE + 0x5000 + 0x04)

#define rP0DRC (PORT0_BASE + 0x0000 + 0x00)
#define rP1DRC (PORT0_BASE + 0x1000 + 0x00)
#define rP2DRC (PORT0_BASE + 0x2000 + 0x00)
#define rP3DRC (PORT0_BASE + 0x3000 + 0x00)
#define rP4DRC (PORT0_BASE + 0x4000 + 0x00)
#define rP5DRC (PORT0_BASE + 0x5000 + 0x00)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint32_t get_cpu_ver(void);
void init_default_mux(void);
void lc823450_clock_dump(void);

#ifdef CONFIG_LC823450_LSISTBY
void mod_stby_regs(uint32_t clearbits, uint32_t setbits);
void lc823450_mod_stby_regs(uint32_t clearbits, uint32_t setbits);
#else
# define mod_stby_regs(...)
# define lc823450_mod_stby_regs(...)
#endif

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SYSCONTROL_H */
