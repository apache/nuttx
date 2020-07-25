/********************************************************************************************
 * arch/arm/src/lpc55xx/chip/lpc55_syscon.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55_SYSCON_H
#define __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55_SYSCON_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc55_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *************************************************************************/

/* Main system configuration */

#define LPC55_SYSCON_MEMREMAPCTRL_OFFSET       0x0000  /* AHB multilayer matrix priority control */
#define LPC55_SYSCON_AHBMATPRIO_OFFSET         0x0010  /* AHB multilayer matrix priority control */
#define LPC55_SYSCON_SYSTCKCALSEC_OFFSET       0x0040  /* System tick counter calibration (secure) */
#define LPC55_SYSCON_SYSTCKCALNSEC_OFFSET      0x0040  /* System tick counter calibration (non-secure) */
#define LPC55_SYSCON_NMISRC_OFFSET             0x0048  /* NMI source select */
#define LPC55_SYSCON_PRESETCTRL0_OFFSET        0x0100  /* Peripheral reset control 0 */
#define LPC55_SYSCON_PRESETCTRL1_OFFSET        0x0104  /* Peripheral reset control 1 */
#define LPC55_SYSCON_PRESETCTRL2_OFFSET        0x0108  /* Peripheral reset control 2 */
#define LPC55_SYSCON_PRESETCTRLSET0_OFFSET     0x0120  /* Set bits in PRESETCTRL0 */
#define LPC55_SYSCON_PRESETCTRLSET1_OFFSET     0x0124  /* Set bits in PRESETCTRL1 */
#define LPC55_SYSCON_PRESETCTRLSET2_OFFSET     0x0128  /* Set bits in PRESETCTRL2 */
#define LPC55_SYSCON_PRESETCTRLCLR0_OFFSET     0x0140  /* Clear bits in PRESETCTRL0 */
#define LPC55_SYSCON_PRESETCTRLCLR1_OFFSET     0x0144  /* Clear bits in PRESETCTRL1 */
#define LPC55_SYSCON_PRESETCTRLCLR2_OFFSET     0x0148  /* Clear bits in PRESETCTRL2 */
#define LPC55_SYSCON_SWRRESET_OFFSET           0x0160  /* System reset */
#define LPC55_SYSCON_AHBCLKCTRL0_OFFSET        0x0200  /* AHB Clock control 0 */
#define LPC55_SYSCON_AHBCLKCTRL1_OFFSET        0x0204  /* AHB Clock control 1 */
#define LPC55_SYSCON_AHBCLKCTRL2_OFFSET        0x0208  /* AHB Clock control 2 */
#define LPC55_SYSCON_AHBCLKCTRLSET0_OFFSET     0x0220  /* Set bits in AHBCLKCTRL0 */
#define LPC55_SYSCON_AHBCLKCTRLSET1_OFFSET     0x0224  /* Set bits in AHBCLKCTRL1 */
#define LPC55_SYSCON_AHBCLKCTRLSET2_OFFSET     0x0228  /* Set bits in AHBCLKCTRL2 */
#define LPC55_SYSCON_AHBCLKCTRLCLR0_OFFSET     0x0240  /* Clear bits in AHBCLKCTRL0 */
#define LPC55_SYSCON_AHBCLKCTRLCLR1_OFFSET     0x0244  /* Clear bits in AHBCLKCTRL1 */
#define LPC55_SYSCON_AHBCLKCTRLCLR2_OFFSET     0x0248  /* Clear bits in AHBCLKCTRL2 */
#define LPC55_SYSCON_SYSTICKCLKSEL0_OFFSET     0x0260  /* System Tick Timer for CPU0 source select */
#define LPC55_SYSCON_SYSTICKCLKSEL1_OFFSET     0x0264  /* System Tick Timer for CPU1 source select */
#define LPC55_SYSCON_TRACECLKSEL_OFFSET        0x0268  /* Trace clock source select */
#define LPC55_SYSCON_CTIMER0SRC_OFFSET         0x026c  /* CTimer 0 clock source select */
#define LPC55_SYSCON_CTIMER1SRC_OFFSET         0x0270  /* CTimer 1 clock source select */
#define LPC55_SYSCON_CTIMER2SRC_OFFSET         0x0274  /* CTimer 2 clock source select */
#define LPC55_SYSCON_CTIMER3SRC_OFFSET         0x0278  /* CTimer 3 clock source select */
#define LPC55_SYSCON_CTIMER4SRC_OFFSET         0x027c  /* CTimer 4 clock source select */
#define LPC55_SYSCON_MAINCLKSELA_OFFSET        0x0280  /* Main clock source select A */
#define LPC55_SYSCON_MAINCLKSELB_OFFSET        0x0284  /* Main clock source select B */
#define LPC55_SYSCON_CLKOUTSELA_OFFSET         0x0288  /* CLKOUT clock source select */
#define LPC55_SYSCON_PLL0CLKSEL_OFFSET         0x0290  /* PLL0 clock source select */
#define LPC55_SYSCON_PLL1CLKSEL_OFFSET         0x0294  /* PLL1 clock source select */
#define LPC55_SYSCON_ADCCLKSEL_OFFSET          0x02a4  /* ADC clock source select */
#define LPC55_SYSCON_USB0CLKSEL_OFFSET         0x02a8  /* FS USB clock source select */
#define LPC55_SYSCON_FCLKSEL0_OFFSET           0x02b0  /* Flexcomm Interface 0 clock source select */
#define LPC55_SYSCON_FCLKSEL1_OFFSET           0x02b4  /* Flexcomm Interface 1 clock source select */
#define LPC55_SYSCON_FCLKSEL2_OFFSET           0x02b8  /* Flexcomm Interface 2 clock source select */
#define LPC55_SYSCON_FCLKSEL3_OFFSET           0x02bc  /* Flexcomm Interface 3 clock source select */
#define LPC55_SYSCON_FCLKSEL4_OFFSET           0x02c0  /* Flexcomm Interface 4 clock source select */
#define LPC55_SYSCON_FCLKSEL5_OFFSET           0x02c4  /* Flexcomm Interface 5 clock source select */
#define LPC55_SYSCON_FCLKSEL6_OFFSET           0x02c8  /* Flexcomm Interface 6 clock source select */
#define LPC55_SYSCON_FCLKSEL7_OFFSET           0x02cc  /* Flexcomm Interface 7 clock source select */
#define LPC55_SYSCON_HSLSPICLKSEL_OFFSET       0x02d0  /* HS SPI clock source select */
#define LPC55_SYSCON_MCLKCLKSEL_OFFSET         0x02e0  /* MCLK clock source select */
#define LPC55_SYSCON_SCTCLKSEL_OFFSET          0x02f0  /* SCTimer/PWM clock source select */
#define LPC55_SYSCON_SDIOCLKSEL_OFFSET         0x02f8  /* SDIO clock source select */
#define LPC55_SYSCON_SYSTICKCLKDIV0_OFFSET     0x0300  /* SYSTICK clock divider CPU 0 */
#define LPC55_SYSCON_SYSTICKCLKDIV1_OFFSET     0x0304  /* SYSTICK clock divider CPU 1 */
#define LPC55_SYSCON_TRACECLKDIV_OFFSET        0x0308  /* ARM Trace clock divider */
#define LPC55_SYSCON_FLEXFRG0CTRL_OFFSET       0x0320  /* Fractional Rate Divider for Flexcomm0 */
#define LPC55_SYSCON_FLEXFRG1CTRL_OFFSET       0x0324  /* Fractional Rate Divider for Flexcomm1 */
#define LPC55_SYSCON_FLEXFRG2CTRL_OFFSET       0x0328  /* Fractional Rate Divider for Flexcomm2 */
#define LPC55_SYSCON_FLEXFRG3CTRL_OFFSET       0x032c  /* Fractional Rate Divider for Flexcomm3 */
#define LPC55_SYSCON_FLEXFRG4CTRL_OFFSET       0x0330  /* Fractional Rate Divider for Flexcomm4 */
#define LPC55_SYSCON_FLEXFRG5CTRL_OFFSET       0x0334  /* Fractional Rate Divider for Flexcomm5 */
#define LPC55_SYSCON_FLEXFRG6CTRL_OFFSET       0x0338  /* Fractional Rate Divider for Flexcomm6 */
#define LPC55_SYSCON_FLEXFRG7CTRL_OFFSET       0x033c  /* Fractional Rate Divider for Flexcomm7 */
#define LPC55_SYSCON_AHBCLKDIV_OFFSET          0x0380  /* System clock divider */
#define LPC55_SYSCON_CLKOUTDIV_OFFSET          0x0384  /* CLKOUT clock divider */
#define LPC55_SYSCON_FROHFDIV_OFFSET           0x0388  /* FROHF clock divider */
#define LPC55_SYSCON_ADCCLKDIV_OFFSET          0x0394  /* ADC clock divider */
#define LPC55_SYSCON_USB0CLKDIV_OFFSET         0x0398  /* USB0 clock divider */
#define LPC55_SYSCON_FRGCTRL_OFFSET            0x03a0  /* Fractional rate divider */
#define LPC55_SYSCON_MCLKDIV_OFFSET            0x03ac  /* I2S MCLK clock divider */
#define LPC55_SYSCON_SCTCLKDIV_OFFSET          0x03b4  /* SCT/PWM clock divider */
#define LPC55_SYSCON_SDIOCLKDIV_OFFSET         0x03bc  /* SDIO clock divider */
#define LPC55_SYSCON_PLL0CLKDIV_OFFSET         0x03c4  /* PLL0 clock divider */
#define LPC55_SYSCON_CLOCKGENUPDATELOCK_OFFSET 0x03fc  /* Control clock configuration registers access */
#define LPC55_SYSCON_FMCCR_OFFSET              0x0400  /* FMC configuration */
#define LPC55_SYSCON_USB0CLKCTRL_OFFSET        0x040c  /* USB0 clock control */
#define LPC55_SYSCON_USB0CLKSTAT_OFFSET        0x0410  /* USB0 clock status */
#define LPC55_SYSCON_FMCFLUSH_OFFSET           0x041c  /* FMC flush control */
#define LPC55_SYSCON_MCLKIO_OFFSET             0x0420  /* MCLK input/output control */
#define LPC55_SYSCON_USB1CLKCTRL_OFFSET        0x0424  /* USB1 clock control */
#define LPC55_SYSCON_USB1CLKSTAT_OFFSET        0x0428  /* USB1 clock status */
#define LPC55_SYSCON_SDIOCLKCTRL_OFFSET        0x0460  /* SDIO CCLKIN phase and delay control */
#define LPC55_SYSCON_PLL1CTRL_OFFSET           0x0560  /* PLL1 control */
#define LPC55_SYSCON_PLL1STAT_OFFSET           0x0564  /* PLL1 status */
#define LPC55_SYSCON_PLL1NDEC_OFFSET           0x0568  /* PLL1 N divider */
#define LPC55_SYSCON_PLL1MDEC_OFFSET           0x056c  /* PLL1 M divider */
#define LPC55_SYSCON_PLL1PDEC_OFFSET           0x0580  /* PLL1 P divider */
#define LPC55_SYSCON_PLL0CTRL_OFFSET           0x0580  /* PLL0 control */
#define LPC55_SYSCON_PLL0STAT_OFFSET           0x0584  /* PLL0 status */
#define LPC55_SYSCON_PLL0NDEC_OFFSET           0x0588  /* PLL0 N divider */
#define LPC55_SYSCON_PLL0PDEC_OFFSET           0x058c  /* PLL0 P divider */
#define LPC55_SYSCON_PLL0SSCG0_OFFSET          0x0590  /* System PLL Spread Spectrum Wrapper control 0 */
#define LPC55_SYSCON_PLL0SSCG1_OFFSET          0x0594  /* System PLL Spread Spectrum Wrapper control 1 */
#define LPC55_SYSCON_CPUCTRL_OFFSET            0x0800  /* CPU control for multiple processors */
#define LPC55_SYSCON_CPBOOT_OFFSET             0x0804  /* Coprocessor boot address */
#define LPC55_SYSCON_CPUSTAT_OFFSET            0x080c  /* CPU status */
#define LPC55_SYSCON_CLOCKCTRL_OFFSET          0x0a18  /* Various system clock controls */
#define LPC55_SYSCON_COMPINTCTRL_OFFSET        0x0b10  /* Comparator interrupt control */
#define LPC55_SYSCON_COMPINTSTAUS_OFFSET       0x0b14  /* Comparator interrupt status */
#define LPC55_SYSCON_AUTOCLKGATEOVER_OFFSET    0x0e04  /* Control automatic clock gating */
#define LPC55_SYSCON_GPIOPSYNC_OFFSET          0x0e08  /* Bypass of first stage of sync in GPIO_INT */
#define LPC55_SYSCON_DEBUGLOCKEN_OFFSET        0x0fa0  /* Control write access to sec regs */
#define LPC55_SYSCON_DEBUGLFEATURES_OFFSET     0x0fa4  /* CPU0/1 debug feature control */
#define LPC55_SYSCON_SWDACCESSCPU0_OFFSET      0x0fb4  /* Enable SWD for CPU0 */
#define LPC55_SYSCON_SWDACCESSCPU1_OFFSET      0x0fb8  /* Enable SWD for CPU1 */
#define LPC55_SYSCON_KEYBLOCK_OFFSET           0x0fbc  /* Block access to PUF indexes */
#define LPC55_SYSCON_DEBUGAUTHBEACON_OFFSET    0x0fc0  /* Debug authentication BEACON */
#define LPC55_SYSCON_CPUCFG_OFFSET             0x0fd4  /* CPU configuration */
#define LPC55_SYSCON_DEVICE_ID0_OFFSET         0x0ff8  /* Part ID */
#define LPC55_SYSCON_DIEID_OFFSET              0x0ffc  /* Boot ROM and die revision */

/* Register addresses ***********************************************************************/

/* Main system configuration */

#define LPC55_SYSCON_MEMREMAPCTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_MEMREMAPCTRL_OFFSET)       
#define LPC55_SYSCON_AHBMATPRIO                (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBMATPRIO_OFFSET)         
#define LPC55_SYSCON_SYSTCKCALSEC              (LPC55_SYSCON_BASE + LPC55_SYSCON_SYSTCKCALSEC_OFFSET)       
#define LPC55_SYSCON_SYSTCKCALNSEC             (LPC55_SYSCON_BASE + LPC55_SYSCON_SYSTCKCALNSEC_OFFSET)      
#define LPC55_SYSCON_NMISRC                    (LPC55_SYSCON_BASE + LPC55_SYSCON_NMISRC_OFFSET)             
#define LPC55_SYSCON_PRESETCTRL0               (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRL0_OFFSET)        
#define LPC55_SYSCON_PRESETCTRL1               (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRL1_OFFSET)        
#define LPC55_SYSCON_PRESETCTRL2               (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRL2_OFFSET)        
#define LPC55_SYSCON_PRESETCTRLSET0            (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRLSET0_OFFSET)     
#define LPC55_SYSCON_PRESETCTRLSET1            (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRLSET1_OFFSET)     
#define LPC55_SYSCON_PRESETCTRLSET2            (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRLSET2_OFFSET)     
#define LPC55_SYSCON_PRESETCTRLCLR0            (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRLCLR0_OFFSET)     
#define LPC55_SYSCON_PRESETCTRLCLR1            (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRLCLR1_OFFSET)     
#define LPC55_SYSCON_PRESETCTRLCLR2            (LPC55_SYSCON_BASE + LPC55_SYSCON_PRESETCTRLCLR2_OFFSET)     
#define LPC55_SYSCON_SWRRESET                  (LPC55_SYSCON_BASE + LPC55_SYSCON_SWRRESET_OFFSET)           
#define LPC55_SYSCON_AHBCLKCTRL0               (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRL0_OFFSET)        
#define LPC55_SYSCON_AHBCLKCTRL1               (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRL1_OFFSET)        
#define LPC55_SYSCON_AHBCLKCTRL2               (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRL2_OFFSET)        
#define LPC55_SYSCON_AHBCLKCTRLSET0            (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRLSET0_OFFSET)     
#define LPC55_SYSCON_AHBCLKCTRLSET1            (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRLSET1_OFFSET)     
#define LPC55_SYSCON_AHBCLKCTRLSET2            (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRLSET2_OFFSET)     
#define LPC55_SYSCON_AHBCLKCTRLCLR0            (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRLCLR0_OFFSET)     
#define LPC55_SYSCON_AHBCLKCTRLCLR1            (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRLCLR1_OFFSET)     
#define LPC55_SYSCON_AHBCLKCTRLCLR2            (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKCTRLCLR2_OFFSET)     
#define LPC55_SYSCON_SYSTICKCLKSEL0            (LPC55_SYSCON_BASE + LPC55_SYSCON_SYSTICKCLKSEL0_OFFSET)     
#define LPC55_SYSCON_SYSTICKCLKSEL1            (LPC55_SYSCON_BASE + LPC55_SYSCON_SYSTICKCLKSEL1_OFFSET)     
#define LPC55_SYSCON_TRACECLKSEL               (LPC55_SYSCON_BASE + LPC55_SYSCON_TRACECLKSEL_OFFSET)        
#define LPC55_SYSCON_CTIMER0SRC                (LPC55_SYSCON_BASE + LPC55_SYSCON_CTIMER0SRC_OFFSET)         
#define LPC55_SYSCON_CTIMER1SRC                (LPC55_SYSCON_BASE + LPC55_SYSCON_CTIMER1SRC_OFFSET)         
#define LPC55_SYSCON_CTIMER2SRC                (LPC55_SYSCON_BASE + LPC55_SYSCON_CTIMER2SRC_OFFSET)         
#define LPC55_SYSCON_CTIMER3SRC                (LPC55_SYSCON_BASE + LPC55_SYSCON_CTIMER3SRC_OFFSET)         
#define LPC55_SYSCON_CTIMER4SRC                (LPC55_SYSCON_BASE + LPC55_SYSCON_CTIMER4SRC_OFFSET)         
#define LPC55_SYSCON_MAINCLKSELA               (LPC55_SYSCON_BASE + LPC55_SYSCON_MAINCLKSELA_OFFSET)        
#define LPC55_SYSCON_MAINCLKSELB               (LPC55_SYSCON_BASE + LPC55_SYSCON_MAINCLKSELB_OFFSET)        
#define LPC55_SYSCON_CLKOUTSELA                (LPC55_SYSCON_BASE + LPC55_SYSCON_CLKOUTSELA_OFFSET)         
#define LPC55_SYSCON_PLL0CLKSEL                (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0CLKSEL_OFFSET)         
#define LPC55_SYSCON_PLL1CLKSEL                (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL1CLKSEL_OFFSET)         
#define LPC55_SYSCON_ADCCLKSEL                 (LPC55_SYSCON_BASE + LPC55_SYSCON_ADCCLKSEL_OFFSET)          
#define LPC55_SYSCON_USB0CLKSEL                (LPC55_SYSCON_BASE + LPC55_SYSCON_USB0CLKSEL_OFFSET)         
#define LPC55_SYSCON_FCLKSEL0                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL0_OFFSET)           
#define LPC55_SYSCON_FCLKSEL1                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL1_OFFSET)           
#define LPC55_SYSCON_FCLKSEL2                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL2_OFFSET)           
#define LPC55_SYSCON_FCLKSEL3                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL3_OFFSET)           
#define LPC55_SYSCON_FCLKSEL4                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL4_OFFSET)           
#define LPC55_SYSCON_FCLKSEL5                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL5_OFFSET)           
#define LPC55_SYSCON_FCLKSEL6                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL6_OFFSET)           
#define LPC55_SYSCON_FCLKSEL7                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FCLKSEL7_OFFSET)           
#define LPC55_SYSCON_HSLSPICLKSEL              (LPC55_SYSCON_BASE + LPC55_SYSCON_HSLSPICLKSEL_OFFSET)       
#define LPC55_SYSCON_MCLKCLKSEL                (LPC55_SYSCON_BASE + LPC55_SYSCON_MCLKCLKSEL_OFFSET)         
#define LPC55_SYSCON_SCTCLKSEL                 (LPC55_SYSCON_BASE + LPC55_SYSCON_SCTCLKSEL_OFFSET)          
#define LPC55_SYSCON_SDIOCLKSEL                (LPC55_SYSCON_BASE + LPC55_SYSCON_SDIOCLKSEL_OFFSET)         
#define LPC55_SYSCON_SYSTICKCLKDIV0            (LPC55_SYSCON_BASE + LPC55_SYSCON_SYSTICKCLKDIV0_OFFSET)     
#define LPC55_SYSCON_SYSTICKCLKDIV1            (LPC55_SYSCON_BASE + LPC55_SYSCON_SYSTICKCLKDIV1_OFFSET)     
#define LPC55_SYSCON_TRACECLKDIV               (LPC55_SYSCON_BASE + LPC55_SYSCON_TRACECLKDIV_OFFSET)        
#define LPC55_SYSCON_FLEXFRG0CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG0CTRL_OFFSET)       
#define LPC55_SYSCON_FLEXFRG1CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG1CTRL_OFFSET)       
#define LPC55_SYSCON_FLEXFRG2CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG2CTRL_OFFSET)       
#define LPC55_SYSCON_FLEXFRG3CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG3CTRL_OFFSET)       
#define LPC55_SYSCON_FLEXFRG4CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG4CTRL_OFFSET)       
#define LPC55_SYSCON_FLEXFRG5CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG5CTRL_OFFSET)       
#define LPC55_SYSCON_FLEXFRG6CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG6CTRL_OFFSET)       
#define LPC55_SYSCON_FLEXFRG7CTRL              (LPC55_SYSCON_BASE + LPC55_SYSCON_FLEXFRG7CTRL_OFFSET)       
#define LPC55_SYSCON_AHBCLKDIV                 (LPC55_SYSCON_BASE + LPC55_SYSCON_AHBCLKDIV_OFFSET)          
#define LPC55_SYSCON_CLKOUTDIV                 (LPC55_SYSCON_BASE + LPC55_SYSCON_CLKOUTDIV_OFFSET)          
#define LPC55_SYSCON_FROHFDIV                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FROHFDIV_OFFSET)           
#define LPC55_SYSCON_ADCCLKDIV                 (LPC55_SYSCON_BASE + LPC55_SYSCON_ADCCLKDIV_OFFSET)          
#define LPC55_SYSCON_USB0CLKDIV                (LPC55_SYSCON_BASE + LPC55_SYSCON_USB0CLKDIV_OFFSET)         
#define LPC55_SYSCON_FRGCTRL                   (LPC55_SYSCON_BASE + LPC55_SYSCON_FRGCTRL_OFFSET)            
#define LPC55_SYSCON_MCLKDIV                   (LPC55_SYSCON_BASE + LPC55_SYSCON_MCLKDIV_OFFSET)            
#define LPC55_SYSCON_SCTCLKDIV                 (LPC55_SYSCON_BASE + LPC55_SYSCON_SCTCLKDIV_OFFSET)          
#define LPC55_SYSCON_SDIOCLKDIV                (LPC55_SYSCON_BASE + LPC55_SYSCON_SDIOCLKDIV_OFFSET)         
#define LPC55_SYSCON_PLL0CLKDIV                (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0CLKDIV_OFFSET)         
#define LPC55_SYSCON_CLOCKGENUPDATELOCK        (LPC55_SYSCON_BASE + LPC55_SYSCON_CLOCKGENUPDATELOCK_OFFSET) 
#define LPC55_SYSCON_FMCCR                     (LPC55_SYSCON_BASE + LPC55_SYSCON_FMCCR_OFFSET)              
#define LPC55_SYSCON_USB0CLKCTRL               (LPC55_SYSCON_BASE + LPC55_SYSCON_USB0CLKCTRL_OFFSET)        
#define LPC55_SYSCON_USB0CLKSTAT               (LPC55_SYSCON_BASE + LPC55_SYSCON_USB0CLKSTAT_OFFSET)        
#define LPC55_SYSCON_FMCFLUSH                  (LPC55_SYSCON_BASE + LPC55_SYSCON_FMCFLUSH_OFFSET)           
#define LPC55_SYSCON_MCLKIO                    (LPC55_SYSCON_BASE + LPC55_SYSCON_MCLKIO_OFFSET)             
#define LPC55_SYSCON_USB1CLKCTRL               (LPC55_SYSCON_BASE + LPC55_SYSCON_USB1CLKCTRL_OFFSET)        
#define LPC55_SYSCON_USB1CLKSTAT               (LPC55_SYSCON_BASE + LPC55_SYSCON_USB1CLKSTAT_OFFSET)        
#define LPC55_SYSCON_SDIOCLKCTRL               (LPC55_SYSCON_BASE + LPC55_SYSCON_SDIOCLKCTRL_OFFSET)        
#define LPC55_SYSCON_PLL1CTRL                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL1CTRL_OFFSET)           
#define LPC55_SYSCON_PLL1STAT                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL1STAT_OFFSET)           
#define LPC55_SYSCON_PLL1NDEC                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL1NDEC_OFFSET)           
#define LPC55_SYSCON_PLL1MDEC                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL1MDEC_OFFSET)           
#define LPC55_SYSCON_PLL1PDEC                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL1PDEC_OFFSET)           
#define LPC55_SYSCON_PLL0CTRL                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0CTRL_OFFSET)           
#define LPC55_SYSCON_PLL0STAT                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0STAT_OFFSET)           
#define LPC55_SYSCON_PLL0NDEC                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0NDEC_OFFSET)           
#define LPC55_SYSCON_PLL0PDEC                  (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0PDEC_OFFSET)           
#define LPC55_SYSCON_PLL0SSCG0                 (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0SSCG0_OFFSET)          
#define LPC55_SYSCON_PLL0SSCG1                 (LPC55_SYSCON_BASE + LPC55_SYSCON_PLL0SSCG1_OFFSET)          
#define LPC55_SYSCON_CPUCTRL                   (LPC55_SYSCON_BASE + LPC55_SYSCON_CPUCTRL_OFFSET)            
#define LPC55_SYSCON_CPBOOT                    (LPC55_SYSCON_BASE + LPC55_SYSCON_CPBOOT_OFFSET)             
#define LPC55_SYSCON_CPUSTAT                   (LPC55_SYSCON_BASE + LPC55_SYSCON_CPUSTAT_OFFSET)            
#define LPC55_SYSCON_CLOCKCTRL                 (LPC55_SYSCON_BASE + LPC55_SYSCON_CLOCKCTRL_OFFSET)          
#define LPC55_SYSCON_COMPINTCTRL               (LPC55_SYSCON_BASE + LPC55_SYSCON_COMPINTCTRL_OFFSET)        
#define LPC55_SYSCON_COMPINTSTAUS              (LPC55_SYSCON_BASE + LPC55_SYSCON_COMPINTSTAUS_OFFSET)       
#define LPC55_SYSCON_AUTOCLKGATEOVER           (LPC55_SYSCON_BASE + LPC55_SYSCON_AUTOCLKGATEOVER_OFFSET)    
#define LPC55_SYSCON_GPIOPSYNC                 (LPC55_SYSCON_BASE + LPC55_SYSCON_GPIOPSYNC_OFFSET)          
#define LPC55_SYSCON_DEBUGLOCKEN               (LPC55_SYSCON_BASE + LPC55_SYSCON_DEBUGLOCKEN_OFFSET)        
#define LPC55_SYSCON_DEBUGLFEATURES            (LPC55_SYSCON_BASE + LPC55_SYSCON_DEBUGLFEATURES_OFFSET)     
#define LPC55_SYSCON_SWDACCESSCPU0             (LPC55_SYSCON_BASE + LPC55_SYSCON_SWDACCESSCPU0_OFFSET)      
#define LPC55_SYSCON_SWDACCESSCPU1             (LPC55_SYSCON_BASE + LPC55_SYSCON_SWDACCESSCPU1_OFFSET)      
#define LPC55_SYSCON_KEYBLOCK                  (LPC55_SYSCON_BASE + LPC55_SYSCON_KEYBLOCK_OFFSET)           
#define LPC55_SYSCON_DEBUGAUTHBEACON           (LPC55_SYSCON_BASE + LPC55_SYSCON_DEBUGAUTHBEACON_OFFSET)    
#define LPC55_SYSCON_CPUCFG                    (LPC55_SYSCON_BASE + LPC55_SYSCON_CPUCFG_OFFSET)             
#define LPC55_SYSCON_DEVICE_ID0                (LPC55_SYSCON_BASE + LPC55_SYSCON_DEVICE_ID0_OFFSET)         
#define LPC55_SYSCON_DIEID                     (LPC55_SYSCON_BASE + LPC55_SYSCON_DIEID_OFFSET)              

/* Register bit definitions *****************************************************************/

/* Peripheral reset control 0: PRESETCTRL0, PRESETCTRLSET0, and PRESETCTRLCLR0 */
#define SYSCON_PRESETCTRL0_ROM                 (1 << 1)  /* Bit 3:  Reset the ROM controller */
#define SYSCON_PRESETCTRL0_SRAM1               (1 << 3)  /* Bit 3:  Reset the SRAM 1 controller */
#define SYSCON_PRESETCTRL0_SRAM2               (1 << 4)  /* Bit 4:  Reset the SRAM 2 controller */
#define SYSCON_PRESETCTRL0_SRAM3               (1 << 5)  /* Bit 5:  Reset the SRAM 3 controller */
#define SYSCON_PRESETCTRL0_SRAM4               (1 << 6)  /* Bit 6:  Reset the SRAM 4 controller */
#define SYSCON_PRESETCTRL0_FLASH               (1 << 7)  /* Bit 7:  Reset the flash controller */
#define SYSCON_PRESETCTRL0_FMC                 (1 << 8)  /* Bit 8:  Reset the Flash accelerator */
#define SYSCON_PRESETCTRL0_INPUTMUX            (1 << 11) /* Bit 11: Reset the input muxes */
#define SYSCON_PRESETCTRL0_IOCON               (1 << 13) /* Bit 13: Reset the IOCON block */
#define SYSCON_PRESETCTRL0_GPIO0               (1 << 14) /* Bit 14: Reset the GPIO0 port registers */
#define SYSCON_PRESETCTRL0_GPIO1               (1 << 15) /* Bit 15: Reset the GPIO1 port registers */
#define SYSCON_PRESETCTRL0_PINT                (1 << 18) /* Bit 18: Reset the pin interrupt block */
#define SYSCON_PRESETCTRL0_GINT                (1 << 19) /* Bit 19: Reset the grouped pin interrupt block */
#define SYSCON_PRESETCTRL0_DMA0                (1 << 20) /* Bit 20: Reset the DMA0 controller */
#define SYSCON_PRESETCTRL0_CRC                 (1 << 21) /* Bit 21: Reset the CRC engine */
#define SYSCON_PRESETCTRL0_WWDT                (1 << 22) /* Bit 22: Reset the Watchdog Timer */
#define SYSCON_PRESETCTRL0_RTC                 (1 << 23) /* Bit 23: Enables the bus clock for the RTC */
#define SYSCON_PRESETCTRL0_MAILBOX             (1 << 26) /* Bit 26: Reset the inter CPU comm mailbox */
#define SYSCON_PRESETCTRL0_ADC0                (1 << 27) /* Bit 27: Reset the ADC0 register interface */

/* Peripheral reset control 2: PRESETCTRL1, PRESETCTRLSET1, and PRESETCTRLCLR1 */

#define SYSCON_PRESETCTRL1_MRT                 (1 << 0)  /* Bit 0:  Reset the Multi-Rate Timer */
#define SYSCON_PRESETCTRL1_OSTIMER             (1 << 1)  /* Bit 1:  Reset the OS Event Timer */
#define SYSCON_PRESETCTRL1_SCT0                (1 << 2)  /* Bit 2:  Reset SCT0 */
#define SYSCON_PRESETCTRL1_UTICK               (1 << 10) /* Bit 10: Reset the Micro-tick Timer */
#define SYSCON_PRESETCTRL1_FLEXCOMM0           (1 << 11) /* Bit 11: Reset Flexcomm Interface 0 */
#define SYSCON_PRESETCTRL1_FLEXCOMM1           (1 << 12) /* Bit 12: Reset Flexcomm Interface 1 */
#define SYSCON_PRESETCTRL1_FLEXCOMM2           (1 << 13) /* Bit 13: Reset Flexcomm Interface 2 */
#define SYSCON_PRESETCTRL1_FLEXCOMM3           (1 << 14) /* Bit 14: Reset Flexcomm Interface 3 */
#define SYSCON_PRESETCTRL1_FLEXCOMM4           (1 << 15) /* Bit 15: Reset Flexcomm Interface 4 */
#define SYSCON_PRESETCTRL1_FLEXCOMM5           (1 << 16) /* Bit 16: Reset Flexcomm Interface 5 */
#define SYSCON_PRESETCTRL1_FLEXCOMM6           (1 << 17) /* Bit 17: Reset Flexcomm Interface 6 */
#define SYSCON_PRESETCTRL1_FLEXCOMM7           (1 << 18) /* Bit 18: Reset Flexcomm Interface 7 */
#define SYSCON_PRESETCTRL1_CTIMER2             (1 << 22) /* Bit 22: Reset CTIMER 2 */
#define SYSCON_PRESETCTRL1_USB0D               (1 << 25) /* Bit 25: Reset the USB0 device interface */
#define SYSCON_PRESETCTRL1_CTIMER0             (1 << 26) /* Bit 26: Reset timer CTIMER0 */
#define SYSCON_PRESETCTRL1_CTIMER1             (1 << 27) /* Bit 27: Reset timer CTIMER1 */

/* Peripheral reset control 2: PRESETCTRL2, PRESETCTRL2, and PRESETCTRLCLR2 */

#define SYSCON_PRESETCTRL2_DMA1                (1 << 1)  /* Bit 1:  Reset the DMA1 interface */
#define SYSCON_PRESETCTRL2_COMP                (1 << 2)  /* Bit 2:  Reset the COMP interface */
#define SYSCON_PRESETCTRL2_SDIO                (1 << 3)  /* Bit 3:  Reset the SDIO interface */
#define SYSCON_PRESETCTRL2_USB1H               (1 << 4)  /* Bit 4:  Reset the USB1 host interface */
#define SYSCON_PRESETCTRL2_USB1D               (1 << 5)  /* Bit 5:  Reset the USB1 device interface */
#define SYSCON_PRESETCTRL2_USB1RAM             (1 << 6)  /* Bit 6:  Reset the USB1 RAM interface */
#define SYSCON_PRESETCTRL2_USB1PHY             (1 << 7)  /* Bit 7:  Reset the USB1 PHY interface */
#define SYSCON_PRESETCTRL2_FREQME              (1 << 8)  /* Bit 8:  Reset the frequency meter interface */
#define SYSCON_PRESETCTRL2_RNG                 (1 << 13) /* Bit 13: Reset the RNG interface */
#define SYSCON_PRESETCTRL2_SYSCTL              (1 << 15) /* Bit 15: Reset the SYSCTL block */
#define SYSCON_PRESETCTRL2_USB0HMR             (1 << 16) /* Bit 16: Reset the USB host master interface */
#define SYSCON_PRESETCTRL2_USB0HSL             (1 << 17) /* Bit 17: Reset the USB host slave interface */
#define SYSCON_PRESETCTRL2_HASHAES             (1 << 18) /* Bit 18: Reset the HASH AES interface */
#define SYSCON_PRESETCTRL2_PQ                  (1 << 19) /* Bit 19: Reset the power quad interface */
#define SYSCON_PRESETCTRL2_PLULUT              (1 << 20) /* Bit 20: Reset the PLU LUT interface */
#define SYSCON_PRESETCTRL2_CTIMER3             (1 << 21) /* Bit 21: Reset timer CTIMER3 */
#define SYSCON_PRESETCTRL2_CTIMER4             (1 << 22) /* Bit 22: Reset timer CTIMER4 */
#define SYSCON_PRESETCTRL2_PUF                 (1 << 23) /* Bit 23: Reset PUF interface */
#define SYSCON_PRESETCTRL2_CASPER              (1 << 24) /* Bit 24: Reset CASPER interface */
#define SYSCON_PRESETCTRL2_ANALOG              (1 << 27) /* Bit 27: Reset Analog interface */
#define SYSCON_PRESETCTRL2_HSLSPI              (1 << 28) /* Bit 28: Reset HP SPI interface */
#define SYSCON_PRESETCTRL2_GPIOSEC             (1 << 29) /* Bit 29: Reset GPIO secure interface */
#define SYSCON_PRESETCTRL2_GPIOSECINT          (1 << 30) /* Bit 30: Reset GPIO Secure int interface */

/* AHB Clock control 0: AHBCLKCTRL0, AHBCLKCTRLCLR0, and AHBCLKCTRLSET0 */

#define SYSCON_AHBCLKCTRL0_ROM                 (1 << 1)  /* Bit 1:  Enables the clock for the Boot ROM */
#define SYSCON_AHBCLKCTRL0_SRAM1               (1 << 3)  /* Bit 3:  Enables the clock for SRAM1 */
#define SYSCON_AHBCLKCTRL0_SRAM2               (1 << 4)  /* Bit 4:  Enables the clock for SRAM2 */
#define SYSCON_AHBCLKCTRL0_SRAM3               (1 << 5)  /* Bit 5:  Enables the clock for SRAM3 */
#define SYSCON_AHBCLKCTRL0_SRAM4               (1 << 5)  /* Bit 6:  Enables the clock for SRAM4 */
#define SYSCON_AHBCLKCTRL0_FLASH               (1 << 7)  /* Bit 7:  Enables the clock for the flash controller */
#define SYSCON_AHBCLKCTRL0_FMC                 (1 << 8)  /* Bit 8:  Enables the clock for the Flash accelerator */
#define SYSCON_AHBCLKCTRL0_INPUTMUX            (1 << 11) /* Bit 11: Enables the clock for the input muxes */
#define SYSCON_AHBCLKCTRL0_IOCON               (1 << 13) /* Bit 13: Enables the clock for the IOCON block */
#define SYSCON_AHBCLKCTRL0_GPIO0               (1 << 14) /* Bit 14: Enables the clock for the GPIO0 port registers */
#define SYSCON_AHBCLKCTRL0_GPIO1               (1 << 15) /* Bit 15: Enables the clock for the GPIO1 port registers */
#define SYSCON_AHBCLKCTRL0_GPIO2               (1 << 16) /* Bit 16: Enables the clock for the GPIO2 port registers */
#define SYSCON_AHBCLKCTRL0_GPIO3               (1 << 17) /* Bit 17: Enables the clock for the GPIO3 port registers */
#define SYSCON_AHBCLKCTRL0_PINT                (1 << 18) /* Bit 18: Enables the clock for the pin interrupt block */
#define SYSCON_AHBCLKCTRL0_GINT                (1 << 19) /* Bit 19: Enables the clock for the grouped pin interrupt block */
#define SYSCON_AHBCLKCTRL0_DMA0                (1 << 20) /* Bit 20: Enables the clock for the DMA0 controller */
#define SYSCON_AHBCLKCTRL0_CRC                 (1 << 21) /* Bit 21: Enables the clock for the CRC engine */
#define SYSCON_AHBCLKCTRL0_WWDT                (1 << 22) /* Bit 22: Enables the clock for the Watchdog Timer */
#define SYSCON_AHBCLKCTRL0_RTC                 (1 << 23) /* Bit 23: Enables the bus clock for the RTC */
#define SYSCON_AHBCLKCTRL0_MAILBOX             (1 << 26) /* Bit 26: Enables the clock for the inter-cpu mailbox interface */
#define SYSCON_AHBCLKCTRL0_ADC0                (1 << 27) /* Bit 27: Enables the clock for the ADC0 register interface */

/* AHB Clock control 1: AHBCLKCTRL1, AHBCLKCTRLCLR1, and AHBCLKCTRLSET1 */

#define SYSCON_AHBCLKCTRL1_MRT                 (1 << 0)  /* Bit 0:  Enables the clock for the Multi-Rate Timer */
#define SYSCON_AHBCLKCTRL1_OSTIMER             (1 << 1)  /* Bit 1:  Enables the clock for the OS Event Timer */
#define SYSCON_AHBCLKCTRL1_SCT0                (1 << 2)  /* Bit 2:  Enables the clock for SCT0 */
#define SYSCON_AHBCLKCTRL1_UTICK               (1 << 10) /* Bit 10: Enables the clock for the Micro-tick Timer */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM0           (1 << 11) /* Bit 11: Enables the clock for Flexcomm Interface 0 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM1           (1 << 12) /* Bit 12: Enables the clock for Flexcomm Interface 1 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM2           (1 << 13) /* Bit 13: Enables the clock for Flexcomm Interface 2 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM3           (1 << 14) /* Bit 14: Enables the clock for Flexcomm Interface 3 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM4           (1 << 15) /* Bit 15: Enables the clock for Flexcomm Interface 4 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM5           (1 << 16) /* Bit 16: Enables the clock for Flexcomm Interface 5 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM6           (1 << 17) /* Bit 17: Enables the clock for Flexcomm Interface 6 */
#define SYSCON_AHBCLKCTRL1_FLEXCOMM7           (1 << 18) /* Bit 18: Enables the clock for Flexcomm Interface 7 */
#define SYSCON_AHBCLKCTRL1_CTIMER2             (1 << 22) /* Bit 22: Enables the clock for CTIMER 2 */
#define SYSCON_AHBCLKCTRL1_USB0D               (1 << 25) /* Bit 25: Enables the clock for the USB0 device interface */
#define SYSCON_AHBCLKCTRL1_CTIMER0             (1 << 26) /* Bit 26: Enables the clock for timer CTIMER0 */
#define SYSCON_AHBCLKCTRL1_CTIMER1             (1 << 27) /* Bit 27: Enables the clock for timer CTIMER1 */

/* AHB Clock control 2: AHBCLKCTRL2, AHBCLKCTRLCLR2, and AHBCLKCTRLSET2 */

#define SYSCON_AHBCLKCTRL2_DMA1                (1 << 1)  /* Bit 1:  Enables the clock for the DMA1 interface */
#define SYSCON_AHBCLKCTRL2_COMP                (1 << 2)  /* Bit 2:  Enables the clock for the LCD interface */
#define SYSCON_AHBCLKCTRL2_SDIO                (1 << 3)  /* Bit 3:  Enables the clock for the SDIO interface */
#define SYSCON_AHBCLKCTRL2_USB1H               (1 << 4)  /* Bit 4:  Enables the clock for the USB1 host interface */
#define SYSCON_AHBCLKCTRL2_USB1D               (1 << 5)  /* Bit 5:  Enables the clock for the USB1 device interface */
#define SYSCON_AHBCLKCTRL2_USB1RAM             (1 << 6)  /* Bit 6:  Enables the clock for the USB1 RAM interface */
#define SYSCON_AHBCLKCTRL2_USB1PHY             (1 << 7)  /* Bit 7:  Enables the clock for the USB1 PHY interface */
#define SYSCON_AHBCLKCTRL2_FREQME              (1 << 8)  /* Bit 8:  Enables the clock for the FREQME interface */
#define SYSCON_AHBCLKCTRL2_RNG                 (1 << 13) /* Bit 13: Enables the clock for the RNG interface */
#define SYSCON_AHBCLKCTRL2_SYSCTL              (1 << 15) /* Bit 15: Enables the clock for the SYSCTL Interface 9 */
#define SYSCON_AHBCLKCTRL2_USB0HMR             (1 << 16) /* Bit 16: Enables the clock for the USB host master interface */
#define SYSCON_AHBCLKCTRL2_USB0HSL             (1 << 17) /* Bit 17: Enables the clock for the USB host slave interface */
#define SYSCON_AHBCLKCTRL2_HASHAES             (1 << 18) /* Bit 18: Enables the clock for the HASH_AES interface */
#define SYSCON_AHBCLKCTRL2_PQ                  (1 << 19) /* Bit 19: Enables the clock for the power quad interface */
#define SYSCON_AHBCLKCTRL2_PLULUT              (1 << 20) /* Bit 20: Enables the clock for the PLU LUT interface */
#define SYSCON_AHBCLKCTRL2_CTIMER3             (1 << 21) /* Bit 21: Enables the clock for the CTIMER3 interface */
#define SYSCON_AHBCLKCTRL2_CTIMER4             (1 << 22) /* Bit 22: Enables the clock for the CTIMER4 interface */
#define SYSCON_AHBCLKCTRL2_PUF                 (1 << 23) /* Bit 23: Enables the clock for the PUF interface */
#define SYSCON_AHBCLKCTRL2_CASPER              (1 << 24) /* Bit 24: Enables the clock for the CASPER interface */
#define SYSCON_AHBCLKCTRL2_ANALOG              (1 << 27) /* Bit 27: Enables the clock for the ANALOG interface */
#define SYSCON_AHBCLKCTRL2_HSLSPI              (1 << 28) /* Bit 28: Enables the clock for the HS SPI interface */
#define SYSCON_AHBCLKCTRL2_GPIOSEC             (1 << 29) /* Bit 29: Enables the clock for the GPIO Secure interface */
#define SYSCON_AHBCLKCTRL2_GPIOSECINT          (1 << 30) /* Bit 30: Enables the clock for the GPIO Secure int interface */

/* System tick timer for CPU0/1 source select */

#define SYSCON_SYSTICKCLKSEL_SHIFT             (0)       /* Bits 0-2:  Clock source for system tick clock */
#define SYSCON_SYSTICKCLKSEL_MASK              (7 << SYSCON_SYSTICKCLKSEL_SHIFT)
#  define SYSCON_SYSTICKCLKSEL_DIV0            (0 << SYSCON_SYSTICKCLKSEL_SHIFT) /* System tick timer divided clk */
#  define SYSCON_SYSTICKCLKSEL_FRO1            (1 << SYSCON_SYSTICKCLKSEL_SHIFT) /* FRO 1 MHz (fro_1m) */
#  define SYSCON_SYSTICKCLKSEL_RTC             (2 << SYSCON_SYSTICKCLKSEL_SHIFT) /* 32kHz oscillator */
#  define SYSCON_SYSTICKCLKSEL_NONE            (7 << SYSCON_SYSTICKCLKSEL_SHIFT) /* None */

/* Trace clock source select */

#define SYSCON_TRACECLKSEL_SHIFT               (0)       /* Bits 0-2:  Clock source for trace clock */
#define SYSCON_TRACECLKSEL_MASK                (7 << SYSCON_TRACECLKSEL_SHIFT)
#  define SYSCON_TRACECLKSEL_DIV0              (0 << SYSCON_TRACECLKSEL_SHIFT) /* trace divided clk */
#  define SYSCON_TRACECLKSEL_FRO1              (1 << SYSCON_TRACECLKSEL_SHIFT) /* FRO 1 MHz (fro_1m) */
#  define SYSCON_TRACECLKSEL_RTC               (2 << SYSCON_TRACECLKSEL_SHIFT) /* 32kHz oscillator */
#  define SYSCON_TRACECLKSEL_NONE              (7 << SYSCON_TRACECLKSEL_SHIFT) /* None */

/* Main clock source select A */

#define SYSCON_MAINCLKSELA_SHIFT               (0)       /* Bits 0-1:  Clock source for main clock */
#define SYSCON_MAINCLKSELA_MASK                (7 << SYSCON_MAINCLKSELA_SHIFT)
#  define SYSCON_MAINCLKSELA_FRO12             (0 << SYSCON_MAINCLKSELA_SHIFT) /* FRO 12 MHz (fro_12m) */
#  define SYSCON_MAINCLKSELA_CLKIN             (1 << SYSCON_MAINCLKSELA_SHIFT) /* CLKIN (clk_in) */
#  define SYSCON_MAINCLKSELA_FRO1              (2 << SYSCON_MAINCLKSELA_SHIFT) /* FRO 1 MHz (fro_1m) */
#  define SYSCON_MAINCLKSELA_FROHF             (3 << SYSCON_MAINCLKSELA_SHIFT) /* FRO 96 (fro_hf) */

/* Main clock source select B */

#define SYSCON_MAINCLKSELB_SHIFT               (0)       /* Bits 0-1: Clock source for main clock */
#define SYSCON_MAINCLKSELB_MASK                (7 << SYSCON_MAINCLKSELB_SHIFT)
#  define SYSCON_MAINCLKSELB_MAINCLKSELA       (0 << SYSCON_MAINCLKSELB_SHIFT) /* Use MAINCLKSELA selection */
#  define SYSCON_MAINCLKSELB_PLL0CLK           (1 << SYSCON_MAINCLKSELB_SHIFT) /* System PLL0 output (pll0_clk) */
#  define SYSCON_MAINCLKSELB_PLL1CLK           (2 << SYSCON_MAINCLKSELB_SHIFT) /* System PLL1 output (pll1_clk) */
#  define SYSCON_MAINCLKSELB_RTC               (3 << SYSCON_MAINCLKSELB_SHIFT) /* RTC oscillator 32 kHz output (32k_clk) */

/* PLL0/1 clock source select */

#define SYSCON_PLLCLKSEL_SHIFT                 (0)       /* Bits 0-2: System PLL clock source selection */
#define SYSCON_PLLCLKSEL_MASK                  (7 << SYSCON_PLLCLKSEL_SHIFT)
#  define SYSCON_PLLCLKSEL_FFRO                (0 << SYSCON_PLLCLKSEL_SHIFT) /* FRO 12 MHz (fro_12m) */
#  define SYSCON_PLLCLKSEL_CLKIN               (1 << SYSCON_PLLCLKSEL_SHIFT) /* CLKIN (clk_in) */
#  define SYSCON_PLLCLKSEL_FRO1                (2 << SYSCON_PLLCLKSEL_SHIFT) /* FRO 1 MHz (fro_1m) */
#  define SYSCON_PLLCLKSEL_RTC                 (3 << SYSCON_PLLCLKSEL_SHIFT) /* RTC oscillator 32 KHz output */
#  define SYSCON_PLLCLKSEL_NONE                (7 << SYSCON_PLLCLKSEL_SHIFT) /* None */

/* Flexcomm Interface 0..7 clock source select */

#define SYSCON_FCLKSEL_SHIFT                   (0)     /* Bits 0-2: Flexcomm Interface clock source selection */
#define SYSCON_FCLKSEL_MASK                    (7 << SYSCON_FCLKSEL_SHIFT)
#  define SYSCON_FCLKSEL_MAIN                  (0 << SYSCON_FCLKSEL_SHIFT) /* Main clock */
#  define SYSCON_FCLKSEL_SYSPLL                (1 << SYSCON_FCLKSEL_SHIFT) /* System PLL */
#  define SYSCON_FCLKSEL_FRO12                 (2 << SYSCON_FCLKSEL_SHIFT) /* FRO 12 MHz (fro_12m) */
#  define SYSCON_FCLKSEL_FRO96                 (3 << SYSCON_FCLKSEL_SHIFT) /* FRO 96 MHz (fro_96m) */
#  define SYSCON_FCLKSEL_FRO1                  (4 << SYSCON_FCLKSEL_SHIFT) /* FRO 1 MHz  (fro_1m) */
#  define SYSCON_FCLKSEL_MCLK                  (5 << SYSCON_FCLKSEL_SHIFT) /* MCLK */
#  define SYSCON_FCLKSEL_RTC                   (6 << SYSCON_FCLKSEL_SHIFT) /* RTC oscillator 32 KHz output */
#  define SYSCON_FCLKSEL_NONE                  (7 << SYSCON_FCLKSEL_SHIFT) /* None */

/* MCLK clock source select */

#define SYSCON_MCLKCLKSEL_SHIFT                (0)     /* Bits 0-2: MCLK source select */
#define SYSCON_MCLKCLKSEL_MASK                 (7 << SYSCON_MCLKCLKSEL_SHIFT)
#  define SYSCON_MCLKCLKSEL_FRO96              (0 << SYSCON_MCLKCLKSEL_SHIFT) /* FRO 96MHz (fro_96m) */
#  define SYSCON_MCLKCLKSEL_PLL0               (1 << SYSCON_MCLKCLKSEL_SHIFT) /* PLL0 */
#  define SYSCON_MCLKCLKSEL_NONE               (7 << SYSCON_MCLKCLKSEL_SHIFT) /* None */

/* SYSTICK clock divider */

#define SYSCON_SYSTICKCLKDIV_DIV_SHIFT         (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_SYSTICKCLKDIV_DIV_MASK          (0xff << SYSCON_SYSTICKCLKDIV_DIV_SHIFT)
#  define SYSCON_SYSTICKCLKDIV_DIV(n)          ((uint32_t)((n)-1) << SYSCON_SYSTICKCLKDIV_DIV_SHIFT)
#define SYSCON_SYSTICKCLKDIV_RESET             (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_SYSTICKCLKDIV_HALT              (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_SYSTICKCLKDIV_REQFLAG           (1 << 31) /* Bit 31: Divider status flag */

/* System clock divider */

#define SYSCON_AHBCLKDIV_DIV_SHIFT             (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_AHBCLKDIV_DIV_MASK              (0xff << SYSCON_AHBCLKDIV_DIV_SHIFT)
#  define SYSCON_AHBCLKDIV_DIV(n)              ((uint32_t)((n)-1) << SYSCON_AHBCLKDIV_DIV_SHIFT)
#define SYSCON_AHBCLKDIV_RESET                 (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_AHBCLKDIV_HALT                  (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_AHBCLKDIV_REQFLAG               (1 << 31) /* Bit 32: Divider status flag */

/* Fractional rate divider */

#define SYSCON_FRGCTRL_DIV_SHIFT               (0)       /*  Bits 0-7: Denominator of the fractional divider */
#define SYSCON_FRGCTRL_DIV_MASK                (0xff << SYSCON_FRGCTRL_DIV_SHIFT)
#  define SYSCON_FRGCTRL_DIV(n)                ((uint32_t)((n)-1) << SYSCON_FRGCTRL_DIV_SHIFT)
#define SYSCON_FRGCTRL_MULT_SHIFT              (8)       /* Bit 8-15: Numerator of the fractional divider */
#define SYSCON_FRGCTRL_MULT_MASK               (0xff << SYSCON_FRGCTRL_MULT_SHIFT)
#  define SYSCON_FRGCTRL_MULT(n)               ((uint32_t)(n) << SYSCON_FRGCTRL_MULT_SHIFT)

/* PLL0 clock divider */

#define SYSCON_PLL0CLKDIV_DIV_SHIFT            (0)       /* Bits 0-7: Clock divider value */
#define SYSCON_PLL0CLKDIV_DIV_MASK             (0xff << SYSCON_PLL0CLKDIV_DIV_SHIFT)
#  define SYSCON_PLL0CLKDIV_DIV(n)             ((uint32_t)((n)-1) << SYSCON_PLL0CLKDIV_DIV_SHIFT)
#define SYSCON_PLL0CLKDIV_RESET                (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_PLL0CLKDIV_HALT                 (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_PLL0CLKDIV_REQFLAG              (1 << 31) /* Bit 31: Divider status flag */

/* Clock configuration regiser access */

#define SYSCON_CLOCKGENUPDATE_LOCK             (0)
#define SYSCON_CLOCKGENUPDATE_UNLOCK           (1)

/* System PLL control */

#define SYSCON_PLLCTRL_SELR_SHIFT              (0)       /* Bits 0-3: Bandwidth select R value */
#define SYSCON_PLLCTRL_SELR_MASK               (15 << SYSCON_PLLCTRL_SELR_SHIFT)
#  define SYSCON_PLLCTRL_SELR(n)               ((uint32_t)(n) << SYSCON_PLLCTRL_SELR_SHIFT)
#define SYSCON_PLLCTRL_SELI_SHIFT              (4)       /* Bits 4-9: Bandwidth select I value */
#define SYSCON_PLLCTRL_SELI_MASK               (0x3f << SYSCON_PLLCTRL_SELR_SHIFT)
#  define SYSCON_PLLCTRL_SELI(n)               ((uint32_t)(n) << SYSCON_PLLCTRL_SELR_SHIFT)
#define SYSCON_PLLCTRL_SELP_SHIFT              (10)      /* Bits 10-14: Bandwidth select P value */
#define SYSCON_PLLCTRL_SELP_MASK               (0x1f << SYSCON_PLLCTRL_SELR_SHIFT)
#  define SYSCON_PLLCTRL_SELP(n)               ((uint32_t)(n) << SYSCON_PLLCTRL_SELR_SHIFT)
#define SYSCON_PLLCTRL_BYPASS                  (1 << 15) /* Bit 15: PLL bypass control */
#define SYSCON_PLLCTRL_BYPASSPOST              (1 << 16) /* Bit 16: PLL bypass post div 2 control */
#define SYSCON_PLLCTRL_UPLIMOFF                (1 << 17) /* Bit 17: Disable upper frequency limiter */
#define SYSCON_PLLCTRL_DIRECTI                 (1 << 19) /* Bit 19: PLL direct input enable */
#define SYSCON_PLLCTRL_DIRECTO                 (1 << 20) /* Bit 20: PLL direct output enable */
#define SYSCON_PLLCTRL_CLKEN                   (1 << 21) /* Bit 21: Enable output clock */
#define SYSCON_PLLCTRL_FRMEN                   (1 << 22) /* Bit 22: Enable free running */
#define SYSCON_PLLCTRL_FRMCLKSTABLE            (1 << 23) /* Bit 23: Free running clock stable */
#define SYSCON_PLLCTRL_SKEWEN                  (1 << 24) /* Bit 24: Skew mode enable */

/* PLL status */

#define SYSCON_PLLSTAT_LOCK                    (1 << 0)  /* Bit 0: PLL lock indicator */
#define SYSCON_PLLSTAT_PREDIVACK               (1 << 1)  /* Bit 1: Pre-div ratio change ack */
#define SYSCON_PLLSTAT_FEEDDIVACK              (1 << 2)  /* Bit 2: Feedback ratio change ack */
#define SYSCON_PLLSTAT_POSTDIVACK              (1 << 3)  /* Bit 3: Post-div radio change ack */
#define SYSCON_PLLSTAT_FRMDET                  (1 << 4)  /* Bit 4: Free running detector output */


/* PLL N divider */

#define SYSCON_PLLNDEC_NDEC_SHIFT              (0)       /* Bits 0-7: Decoded N-divider coefficient */
#define SYSCON_PLLNDEC_NDEC_MASK               (0xff << SYSCON_PLLNDEC_NDEC_SHIFT)
#  define SYSCON_PLLNDEC_NDEC(n)               ((uint32_t)(n) << SYSCON_PLLNDEC_NDEC_SHIFT)
#define SYSCON_PLLNDEC_NREQ                    (1 << 8)  /* Bit 8:  NREQ reload request */

/* PLL P divider */

#define SYSCON_PLLPDEC_PDEC_SHIFT              (0)       /* Bits 0-4: Decoded P-divider coefficient */
#define SYSCON_PLLPDEC_PDEC_MASK               (0x1f << SYSCON_PLLPDEC_PDEC_SHIFT)
#  define SYSCON_PLLPDEC_PDEC(n)               ((uint32_t)(n) << SYSCON_PLLPDEC_PDEC_SHIFT)
#define SYSCON_PLLPDEC_PREQ                    (1 << 5)  /* Bit 5:  PREQ reload request */

/* PLL M divider */

#define SYSCON_PLLMDEC_MDEC_SHIFT              (0)       /* Bits 0-16: Decoded M-divider coefficient */
#define SYSCON_PLLMDEC_MDEC_MASK               (0xffff << SYSCON_PLLMDEC_MDEC_SHIFT)
#  define SYSCON_PLLMDEC_MDEC(n)               ((uint32_t)(n) << SYSCON_PLLMDEC_MDEC_SHIFT)
#define SYSCON_PLLMDEC_MREQ                    (1 << 16) /* Bit 16:  MREQ reload request */

#endif /* __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55_SYSCON_H */
