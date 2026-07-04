===========
STM32 ports
===========

This page records the STM32 families supported by the NuttX ARM port and the
peripheral IP-core selections used by the common STM32 implementation.  The
goal of this inventory is to keep shared STM32 code selected by peripheral IP
version, not by family name.

Family status
=============

============  ================  ===========  ===========================
Family        CPU core          Status       NuttX source directory
============  ================  ===========  ===========================
STM32C0       Cortex-M0+        supported    ``arch/arm/src/stm32c0``
STM32C5       Cortex-M33        unsupported  none
STM32F0       Cortex-M0         supported    ``arch/arm/src/stm32f0``
STM32F1       Cortex-M3         supported    ``arch/arm/src/stm32f1``
STM32F2       Cortex-M3         supported    ``arch/arm/src/stm32f2``
STM32F3       Cortex-M4         supported    ``arch/arm/src/stm32f3``
STM32F4       Cortex-M4         supported    ``arch/arm/src/stm32f4``
STM32F7       Cortex-M7         supported    ``arch/arm/src/stm32f7``
STM32G0       Cortex-M0+        supported    ``arch/arm/src/stm32g0``
STM32G4       Cortex-M4         supported    ``arch/arm/src/stm32g4``
STM32H5       Cortex-M33        supported    ``arch/arm/src/stm32h5``
STM32H7       Cortex-M7         supported    ``arch/arm/src/stm32h7``
STM32L0       Cortex-M0+        supported    ``arch/arm/src/stm32l0``
STM32L1       Cortex-M3         supported    ``arch/arm/src/stm32l1``
STM32L4       Cortex-M4         supported    ``arch/arm/src/stm32l4``
STM32L5       Cortex-M33        supported    ``arch/arm/src/stm32l5``
STM32MP1      Cortex-A7         unsupported  none
STM32MP1      Cortex-M4         unsupported  none
STM32MP2      Cortex-A35        unsupported  none
STM32MP2      Cortex-M33        unsupported  none
STM32N6       Cortex-M55        supported    ``arch/arm/src/stm32n6``
STM32U0       Cortex-M0+        unsupported  none
STM32U3       Cortex-M33        unsupported  none
STM32U5       Cortex-M33        supported    ``arch/arm/src/stm32u5``
STM32WB       Cortex-M4         supported    ``arch/arm/src/stm32wb``
STM32WB       Cortex-M0+        unsupported  none
STM32WB0      Cortex-M0+        unsupported  none
STM32WBA      Cortex-M33        unsupported  none
STM32WL3      Cortex-M0+        unsupported  none
STM32WL5      Cortex-M4         supported    ``arch/arm/src/stm32wl5``
STM32WL5      Cortex-M0+        unsupported  none
STM32WLE      Cortex-M4         unsupported  none
============  ================  ===========  ===========================


Migration to NuttX 13.0
=======================

Release 13.0 normalizes every STM32 port onto a single, consistent Kconfig
namespace, public API and source tree.  All of the changes below are breaking.
Each entry states what changed and the exact migration step; almost every
migration is a mechanical search-and-replace that can be scripted (``sed``,
IDE refactor, etc.).

Apply the steps in this order so that each replacement stays unambiguous:

#. Chip family selectors (``CONFIG_ARCH_CHIP_*``).
#. Kconfig options (``CONFIG_STM32xx_*`` to ``CONFIG_STM32_*``).
#. Hardware definition macros (``STM32xx_*`` to ``STM32_*``).
#. Public API names (``stm32xx_*`` to ``stm32_*``) and family includes.
#. Out-of-tree source / include / board paths.

Chip family selectors split
---------------------------

PR `#19067 <https://github.com/apache/nuttx/pull/19067>`__.

The combined ``CONFIG_ARCH_CHIP_STM32`` selector was replaced by one concrete
selector per legacy sub-family.  Update out-of-tree defconfigs::

    CONFIG_ARCH_CHIP_STM32   ->  one of:
                                 CONFIG_ARCH_CHIP_STM32F1
                                 CONFIG_ARCH_CHIP_STM32F2
                                 CONFIG_ARCH_CHIP_STM32F3
                                 CONFIG_ARCH_CHIP_STM32F4
                                 CONFIG_ARCH_CHIP_STM32G4
                                 CONFIG_ARCH_CHIP_STM32L1

Kconfig options unified to ``CONFIG_STM32_*``
---------------------------------------------

PR `#19136 <https://github.com/apache/nuttx/pull/19136>`__.

All family-prefixed Kconfig options were renamed to the common
``CONFIG_STM32_*`` namespace (common options now live in
``arch/arm/src/common/stm32``).  Replace the family prefix with
``CONFIG_STM32_`` for every affected family::

    CONFIG_STM32F0L0G0_  ->  CONFIG_STM32_
    CONFIG_STM32F7_      ->  CONFIG_STM32_
    CONFIG_STM32L4_      ->  CONFIG_STM32_
    CONFIG_STM32L5_      ->  CONFIG_STM32_
    CONFIG_STM32U5_      ->  CONFIG_STM32_
    CONFIG_STM32H5_      ->  CONFIG_STM32_
    CONFIG_STM32H7_      ->  CONFIG_STM32_
    CONFIG_STM32WB_      ->  CONFIG_STM32_
    CONFIG_STM32WL5_     ->  CONFIG_STM32_
    CONFIG_STM32N6_      ->  CONFIG_STM32_

Board clock-config options are normalized too::

    CONFIG_ARCH_BOARD_STM32F0L0G0_CUSTOM_CLOCKCONFIG  ->  CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
    CONFIG_ARCH_BOARD_STM32H5_CUSTOM_CLOCKCONFIG      ->  CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
    CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG      ->  CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG

QUADSPI was also renamed to QSPI on F7/H7 (the IP block is the same;
PR `#19069 <https://github.com/apache/nuttx/pull/19069>`__).  The QUADSPI
register/pin/DMA hardware macros are unchanged::

    CONFIG_STM32F7_QUADSPI  ->  CONFIG_STM32_QSPI
    CONFIG_STM32H7_QUADSPI  ->  CONFIG_STM32_QSPI

Hardware definition macros unified to ``STM32_*``
-------------------------------------------------

PR `#19114 <https://github.com/apache/nuttx/pull/19114>`__.

Non-standard hardware definition macros (IRQ numbers, peripheral counts, SRAM
and related) were renamed to the common ``STM32_*`` prefix.  Replace the family
prefix in out-of-tree code (do this *after* the Kconfig migration so a plain
prefix replace is safe)::

    STM32L4_   ->  STM32_
    STM32L5_   ->  STM32_
    STM32F7_   ->  STM32_
    STM32H5_   ->  STM32_
    STM32H7_   ->  STM32_
    STM32U5_   ->  STM32_
    STM32WB_   ->  STM32_
    STM32WL5_  ->  STM32_
    STM32N6_   ->  STM32_

Board timer input clock unified to ``STM32_TIMn_CLKIN``
-------------------------------------------------------

The board-provided timer input clock frequency macros were unified to a
single convention: every board defines ``STM32_TIMn_CLKIN`` for the TIM
peripherals, ``STM32_LPTIMn_CLKIN`` for the LPTIM peripherals and
``STM32_HRTIM1_CLKIN`` for HRTIM.  The APB bus is no longer encoded in the
macro name.  Update out-of-tree board headers and drivers (drop the bus
from the name, keep the value)::

    STM32_APBx_TIMn_CLKIN     ->  STM32_TIMn_CLKIN
    BOARD_TIMn_FREQUENCY      ->  STM32_TIMn_CLKIN
    STM32_APBx_LPTIMn_CLKIN   ->  STM32_LPTIMn_CLKIN
    BOARD_LPTIMn_FREQUENCY    ->  STM32_LPTIMn_CLKIN
    STM32_LPTIMn_FREQUENCY    ->  STM32_LPTIMn_CLKIN
    STM32_APB1_THRTIM1_CLKIN  ->  STM32_HRTIM1_CLKIN
    BOARD_HRTIM1_FREQUENCY    ->  STM32_HRTIM1_CLKIN

Public API names unified to ``stm32_*``
---------------------------------------

PR `#19125 <https://github.com/apache/nuttx/pull/19125>`__.

Public STM32 interfaces (functions and API-facing types, including the
timer / lptimer / dma / freerun APIs) were renamed from ``stm32<family>_*`` to
canonical ``stm32_*`` for F7, H5, H7, L4, L5, WB and WL5.  The per-family root
header ``stm32<family>.h`` was renamed to ``stm32.h``.  Migrate out-of-tree
code (the same rule applies to ``stm32f7_``, ``stm32h5_``, ``stm32h7_``,
``stm32l5_``, ``stm32wb_`` and ``stm32wl5_``)::

    stm32l4_              ->  stm32_
    #include "stm32l4.h"  ->  #include "stm32.h"

Common sources moved to ``common/stm32``
----------------------------------------

PR `#19143 <https://github.com/apache/nuttx/pull/19143>`__.

Shared STM32 code was relocated so that all families build from one tree.
Out-of-tree code that references family-local common source/include paths must
point at the new common locations::

    arch source and private headers  ->  arch/arm/src/common/stm32
    board common sources             ->  boards/arm/common/stm32

Per-family directory split
--------------------------

PR `#19143 <https://github.com/apache/nuttx/pull/19143>`__.

The two combined super-directories were split into one directory per chip
family.  Out-of-tree boards, include paths, source paths and defconfigs must
move to the matching family directory::

    arch/arm/src/stm32f0l0g0  ->  stm32f0, stm32l0, stm32g0, stm32c0
    boards/arm/stm32f0l0g0    ->  boards/arm/stm32{f0,l0,g0,c0}
    arch/arm/src/stm32        ->  stm32f1, stm32f2, stm32f3, stm32f4, stm32g4, stm32l1
    boards/arm/stm32          ->  boards/arm/stm32{f1,f2,f3,f4,g4,l1}

Family peripheral inventory
===========================

Each family table records every peripheral group currently tracked by this
inventory.  ``unsupported`` means the peripheral is not available in that
family or is intentionally not implemented by the NuttX family port.
``to be done`` means the peripheral exists or needs confirmation, but the
exact IP core version has not been documented yet.

STM32C0
-------

============  ============  =================================================
Peripheral    Core version  Driver                                           
============  ============  =================================================
ADC           v1            arch/arm/src/common/stm32/stm32_adc_m0_v1.c      
AES/CRYP      v1            not supported                                    
CAN/FDCAN     FDCAN         arch/arm/src/common/stm32/stm32_fdcan_m0_v1.c    
CRC           to be done    not supported                                    
DAC           v1            not supported                                    
DBGMCU        v1            not supported                                    
DMA           v1            arch/arm/src/common/stm32/stm32_dma_m0_v1_7ch.c  
DMAMUX        v1 7ch        arch/arm/src/common/stm32/stm32_dma_m0_v1_7ch.c  
EXTI          v2            arch/arm/src/common/stm32/stm32_exti_gpio_m0_v1.c
FLASH         G0/C0         arch/arm/src/common/stm32/stm32_flash_m0_g0c0.c  
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m0_v1.c     
I2C           v1            arch/arm/src/common/stm32/stm32_i2c_m0_v1.c      
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m0_v1.c     
PWR           G0            arch/arm/src/common/stm32/stm32_pwr_m0_g0.c      
RCC           to be done    arch/arm/src/stm32c0/stm32_rcc.c                 
RTC           RTCC M0       not supported                                    
SPI/I2S       v2            arch/arm/src/common/stm32/stm32_spi_m0_v1.c      
SYSCFG        to be done    not supported                                    
TIM           v1            arch/arm/src/common/stm32/stm32_tim_m0_v1.c      
USART/LPUART  v4            arch/arm/src/common/stm32/stm32_serial_m0_v4.c   
USB           device        not supported                                    
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m0_v1.c     
============  ============  =================================================

STM32F0
-------

============  ============  =================================================
Peripheral    Core version  Driver                                           
============  ============  =================================================
ADC           v1            arch/arm/src/common/stm32/stm32_adc_m0_v1.c      
CAN/FDCAN     bxCAN         not supported                                    
CRC           to be done    not supported                                    
DAC           v1            not supported                                    
DMA           v1            arch/arm/src/common/stm32/stm32_dma_m0_v1_7ch.c  
EXTI          v1            arch/arm/src/common/stm32/stm32_exti_gpio_m0_v1.c
FLASH         M0 v1         not supported                                    
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m0_v1.c     
HDMI-CEC      to be done    not supported                                    
I2C           v1            arch/arm/src/common/stm32/stm32_i2c_m0_v1.c      
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m0_v1.c     
PWR           v1            arch/arm/src/common/stm32/stm32_pwr_m0_v1.c      
RCC           to be done    arch/arm/src/stm32f0/stm32_rcc.c                 
RTC           RTCC M0       not supported                                    
SPI/I2S       v2            arch/arm/src/common/stm32/stm32_spi_m0_v1.c      
SYSCFG        to be done    not supported                                    
TIM           v1            arch/arm/src/common/stm32/stm32_tim_m0_v1.c      
USART/LPUART  v3            arch/arm/src/common/stm32/stm32_serial_m0_v3.c   
USB           device        arch/arm/src/common/stm32/stm32_usbdev_m0_v1.c   
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m0_v1.c     
============  ============  =================================================

STM32F1
-------

============  =============  =======================================================
Peripheral    Core version   Driver                                                 
============  =============  =======================================================
ADC           v1 basic       arch/arm/src/common/stm32/stm32_adc_m3m4_v1v2.c        
CAN/FDCAN     bxCAN          arch/arm/src/common/stm32/stm32_can_m3m4_v1.c          
CRC           to be done     not supported                                          
DAC           v1             arch/arm/src/common/stm32/stm32_dac_m3m4_v1.c          
DBGMCU        v1             not supported                                          
DMA           v1             arch/arm/src/common/stm32/stm32_dma_m3m4_v1_8ch.c      
ETH           to be done     arch/arm/src/common/stm32/stm32_eth_m3m4_v1.c          
EXTI          v1             arch/arm/src/common/stm32/stm32_exti_gpio_m3m4_v1v2.c  
FLASH         F1/F3          arch/arm/src/common/stm32/stm32_flash_m3m4_f1f3.c      
FSMC          v1             arch/arm/src/common/stm32/stm32_fsmc_m3m4_v1.c         
GPIO          v1             arch/arm/src/common/stm32/stm32_gpio_m3m4_v1v2.c       
I2C           v1             arch/arm/src/common/stm32/stm32_i2c_m3m4_v1.c          
IWDG          v1             arch/arm/src/common/stm32/stm32_iwdg_m3m4_v1.c         
PWR           v1             arch/arm/src/common/stm32/stm32_pwr_m3m4_v1.c          
RCC           to be done     arch/arm/src/stm32f1/stm32_rcc.c                       
RTC           counter        arch/arm/src/common/stm32/stm32_rtc_m3m4_v1_lowerhalf.c
SDIO          v1             arch/arm/src/common/stm32/stm32_sdio_m3m4_v1.c         
SPI/I2S       v1             arch/arm/src/common/stm32/stm32_i2s_m3m4_v1.c          
TIM           v1             arch/arm/src/common/stm32/stm32_tim_m3m4_v1v2v3.c      
USART/LPUART  v1             arch/arm/src/common/stm32/stm32_serial_m3m4_v1v2v3v4.c 
USB           device/OTG FS  arch/arm/src/common/stm32/stm32_otgfsdev_m3m4_v1.c     
WWDG          v1             arch/arm/src/common/stm32/stm32_wwdg_m3m4_v1.c         
============  =============  =======================================================

STM32F2
-------

============  ============  =======================================================
Peripheral    Core version  Driver                                                 
============  ============  =======================================================
ADC           v1            arch/arm/src/common/stm32/stm32_adc_m3m4_v1v2.c        
CAN/FDCAN     bxCAN         arch/arm/src/common/stm32/stm32_can_m3m4_v1.c          
CRYP          v1            arch/arm/src/common/stm32/stm32_aes_m3m4_v1.c          
DAC           v1            arch/arm/src/common/stm32/stm32_dac_m3m4_v1.c          
DMA           v2            arch/arm/src/common/stm32/stm32_dma_m3m4_v2_stream.c   
ETH           to be done    arch/arm/src/common/stm32/stm32_eth_m3m4_v1.c          
EXTI          v1            arch/arm/src/common/stm32/stm32_exti_gpio_m3m4_v1v2.c  
FLASH         F2/F4         arch/arm/src/common/stm32/stm32_flash_m3m4_f2f4.c      
FSMC          v1            arch/arm/src/common/stm32/stm32_fsmc_m3m4_v1.c         
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m3m4_v1v2.c       
HASH          to be done    not supported                                          
I2C           v1            arch/arm/src/common/stm32/stm32_i2c_m3m4_v1.c          
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m3m4_v1.c         
RCC           to be done    arch/arm/src/stm32f2/stm32_rcc.c                       
RNG           v1            arch/arm/src/common/stm32/stm32_rng_m3m4_v1.c          
RTC           RTCC v1       arch/arm/src/common/stm32/stm32_rtc_m3m4_v1_lowerhalf.c
SDIO          v1            arch/arm/src/common/stm32/stm32_sdio_m3m4_v1.c         
SPI/I2S       v2            arch/arm/src/common/stm32/stm32_spi_m3m4_v2v3v4.c      
TIM           v1            arch/arm/src/common/stm32/stm32_tim_m3m4_v1v2v3.c      
USART/LPUART  v2            arch/arm/src/common/stm32/stm32_serial_m3m4_v1v2v3v4.c 
USB           OTG FS/HS     arch/arm/src/common/stm32/stm32_otgfsdev_m3m4_v1.c     
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m3m4_v1.c         
============  ============  =======================================================

STM32F3
-------

============  ==============  =======================================================
Peripheral    Core version    Driver                                                 
============  ==============  =======================================================
ADC           v1 basic or v2  arch/arm/src/common/stm32/stm32_adc_m3m4_v1v2.c        
CAN/FDCAN     bxCAN           arch/arm/src/common/stm32/stm32_can_m3m4_v1.c          
COMP          v1              arch/arm/src/common/stm32/stm32_comp_m3m4_v1.c         
DAC           v1              arch/arm/src/common/stm32/stm32_dac_m3m4_v1.c          
DBGMCU        v2              not supported                                          
DMA           v1              arch/arm/src/common/stm32/stm32_dma_m3m4_v1_8ch.c      
EXTI          v1              arch/arm/src/common/stm32/stm32_exti_gpio_m3m4_v1v2.c  
FLASH         F1/F3           arch/arm/src/common/stm32/stm32_flash_m3m4_f1f3.c      
GPIO          v1              arch/arm/src/common/stm32/stm32_gpio_m3m4_v1v2.c       
HRTIM         v1              arch/arm/src/common/stm32/stm32_hrtim_m3m4_v1.c        
I2C           v2              arch/arm/src/common/stm32/stm32_i2c_m3m4_v2.c          
IWDG          v1              arch/arm/src/common/stm32/stm32_iwdg_m3m4_v1.c         
OPAMP         v1              arch/arm/src/common/stm32/stm32_opamp_m3m4_v1.c        
RCC           to be done      arch/arm/src/stm32f3/stm32_rcc.c                       
RTC           RTCC v1         arch/arm/src/common/stm32/stm32_rtc_m3m4_v1_lowerhalf.c
SDADC         v1              arch/arm/src/common/stm32/stm32_sdadc_m3m4_v1.c        
SPI/I2S       v3              arch/arm/src/common/stm32/stm32_spi_m3m4_v2v3v4.c      
SYSCFG        v1              not supported                                          
TIM           v1 or v2        arch/arm/src/common/stm32/stm32_tim_m3m4_v1v2v3.c      
USART/LPUART  v3              arch/arm/src/common/stm32/stm32_serial_m3m4_v1v2v3v4.c 
USB           device          arch/arm/src/common/stm32/stm32_otgfsdev_m3m4_v1.c     
WWDG          v1              arch/arm/src/common/stm32/stm32_wwdg_m3m4_v1.c         
============  ==============  =======================================================

STM32F4
-------

============  ============  ======================================================
Peripheral    Core version  Driver                                                
============  ============  ======================================================
ADC           v1            arch/arm/src/common/stm32/stm32_adc_m3m4_v1v2.c       
CAN/FDCAN     bxCAN         arch/arm/src/common/stm32/stm32_can_m3m4_v1.c         
CRYP          v1            arch/arm/src/common/stm32/stm32_aes_m3m4_v1.c         
DAC           v1            arch/arm/src/common/stm32/stm32_dac_m3m4_v1.c         
DMA           v2            arch/arm/src/common/stm32/stm32_dma_m3m4_v2_stream.c  
DMA2D         v1            arch/arm/src/common/stm32/stm32_dma2d_m3m4_v1.c       
ETH           to be done    arch/arm/src/common/stm32/stm32_eth_m3m4_v1.c         
EXTI          v1            arch/arm/src/common/stm32/stm32_exti_gpio_m3m4_v1v2.c 
FLASH         F2/F4         arch/arm/src/common/stm32/stm32_flash_m3m4_f2f4.c     
FMC/FSMC      v1            arch/arm/src/common/stm32/stm32_fmc_m3m4_v1.c         
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m3m4_v1v2.c      
HASH          to be done    not supported                                         
I2C           v1            arch/arm/src/common/stm32/stm32_i2c_m3m4_v1.c         
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m3m4_v1.c        
LTDC          v1            arch/arm/src/common/stm32/stm32_ltdc_m3m4_v1.c        
QUADSPI       to be done    not supported                                         
RCC           to be done    arch/arm/src/stm32f4/stm32_rcc.c                      
RNG           v1            arch/arm/src/common/stm32/stm32_rng_m3m4_v1.c         
RTC           RTCC F4       arch/arm/src/common/stm32/stm32_rtcc_m3m4_f4.c        
SDIO          v1            arch/arm/src/common/stm32/stm32_sdio_m3m4_v1.c        
SPI/I2S       v2            arch/arm/src/common/stm32/stm32_spi_m3m4_v2v3v4.c     
TIM           v1            arch/arm/src/common/stm32/stm32_tim_m3m4_v1v2v3.c     
USART/LPUART  v2            arch/arm/src/common/stm32/stm32_serial_m3m4_v1v2v3v4.c
USB           OTG FS/HS     arch/arm/src/common/stm32/stm32_otgfsdev_m3m4_v1.c    
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m3m4_v1.c        
============  ============  ======================================================

STM32G4
-------

============  ============  =======================================================
Peripheral    Core version  Driver                                                 
============  ============  =======================================================
ADC           v2            arch/arm/src/common/stm32/stm32_adc_m3m4_v1v2.c        
AES           v1            arch/arm/src/common/stm32/stm32_aes_m3m4_v1.c          
CAN/FDCAN     FDCAN         arch/arm/src/common/stm32/stm32_can_m3m4_v1.c          
COMP          v2            arch/arm/src/common/stm32/stm32_comp_m3m4_v2.c         
CORDIC        v1            arch/arm/src/common/stm32/stm32_cordic_m3m4_v1.c       
DAC           v2            arch/arm/src/common/stm32/stm32_dac_m3m4_v1.c          
DMA           v1            arch/arm/src/common/stm32/stm32_dma_m3m4_v1_8ch.c      
DMAMUX        v1 8ch        arch/arm/src/common/stm32/stm32_dma_m3m4_v1_8ch.c      
EXTI          v2            arch/arm/src/common/stm32/stm32_exti_gpio_m3m4_v1v2.c  
FLASH         G4            arch/arm/src/common/stm32/stm32_flash_m3m4_g4.c        
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m3m4_v1v2.c       
HRTIM         v1            arch/arm/src/common/stm32/stm32_hrtim_m3m4_v1.c        
I2C           v2            arch/arm/src/common/stm32/stm32_i2c_m3m4_v2.c          
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m3m4_v1.c         
OPAMP         v1            arch/arm/src/common/stm32/stm32_opamp_m3m4_v1.c        
RCC           to be done    arch/arm/src/stm32g4/stm32_rcc.c                       
RNG           v1            arch/arm/src/common/stm32/stm32_rng_m3m4_v1.c          
RTC           RTCC v1       arch/arm/src/common/stm32/stm32_rtc_m3m4_v1_lowerhalf.c
SPI/I2S       v3            arch/arm/src/common/stm32/stm32_spi_m3m4_v2v3v4.c      
TIM           v3            arch/arm/src/common/stm32/stm32_tim_m3m4_v1v2v3.c      
USART/LPUART  v4            arch/arm/src/common/stm32/stm32_serial_m3m4_v1v2v3v4.c 
USB           device        arch/arm/src/common/stm32/stm32_otgfsdev_m3m4_v1.c     
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m3m4_v1.c         
============  ============  =======================================================

STM32L4
-------

============  ============  =========================================
Peripheral    Core version  Driver                                   
============  ============  =========================================
ADC           to be done    arch/arm/src/stm32l4/stm32l4_adc.c       
CAN/FDCAN     bxCAN         arch/arm/src/stm32l4/stm32l4_can.c       
COMP          to be done    arch/arm/src/stm32l4/stm32l4_comp.c      
DAC           to be done    arch/arm/src/stm32l4/stm32l4_dac.c       
DFSDM         to be done    arch/arm/src/stm32l4/stm32l4_dfsdm.c     
DMA           v1+DMAMUX     arch/arm/src/stm32l4/stm32l4_dma.c       
EXTI          to be done    arch/arm/src/stm32l4/stm32l4_exti_gpio.c 
FLASH         to be done    arch/arm/src/stm32l4/stm32l4_flash.c     
GPIO          to be done    arch/arm/src/stm32l4/stm32l4_gpio.c      
I2C           to be done    arch/arm/src/stm32l4/stm32l4_i2c.c       
IWDG          v1            arch/arm/src/stm32l4/stm32l4_iwdg.c      
LPTIM         to be done    arch/arm/src/stm32l4/stm32l4_lptim.c     
PWR           to be done    arch/arm/src/stm32l4/stm32l4_pwr.c       
QSPI          to be done    arch/arm/src/stm32l4/stm32l4_qspi.c      
RCC           to be done    arch/arm/src/stm32l4/stm32l4_rcc.c       
RNG           to be done    arch/arm/src/stm32l4/stm32l4_rng.c       
RTC           to be done    arch/arm/src/stm32l4/stm32l4_rtc.c       
SAI           to be done    arch/arm/src/stm32l4/stm32l4_sai.c       
SDMMC         to be done    arch/arm/src/stm32l4/stm32l4_sdmmc.c     
SPI/I2S       to be done    arch/arm/src/stm32l4/stm32l4_spi.c       
TIM           to be done    arch/arm/src/stm32l4/stm32l4_tim.c       
USART/LPUART  v3            arch/arm/src/stm32l4/stm32l4_serial.c    
USB           OTG FS        arch/arm/src/stm32l4/stm32l4_otgfsdev.c  
PULSECOUNT    n/a           arch/arm/src/stm32l4/stm32l4_pulsecount.c
PWM           n/a           arch/arm/src/stm32l4/stm32l4_pwm.c       
QENCODER      n/a           arch/arm/src/stm32l4/stm32l4_qencoder.c  
USB host      n/a           arch/arm/src/stm32l4/stm32l4_otgfshost.c 
============  ============  =========================================

STM32F7
-------

============  ============  =======================================
Peripheral    Core version  Driver                                 
============  ============  =======================================
ADC           to be done    arch/arm/src/stm32f7/stm32_adc.c       
CAN/FDCAN     bxCAN         arch/arm/src/stm32f7/stm32_can.c       
DAC           to be done    not supported                          
DMA           v2 style      arch/arm/src/stm32f7/stm32_dma.c       
DMA2D         v1            arch/arm/src/stm32f7/stm32_dma2d.c     
ETH           to be done    arch/arm/src/stm32f7/stm32_ethernet.c  
EXTI          to be done    arch/arm/src/stm32f7/stm32_exti_gpio.c 
FLASH         to be done    arch/arm/src/stm32f7/stm32_flash.c     
FMC/FSMC      to be done    arch/arm/src/stm32f7/stm32_fmc.c       
GPIO          to be done    arch/arm/src/stm32f7/stm32_gpio.c      
I2C           to be done    arch/arm/src/stm32f7/stm32_i2c.c       
IWDG          v1            not supported                          
LTDC          to be done    arch/arm/src/stm32f7/stm32_ltdc.c      
PWR           to be done    arch/arm/src/stm32f7/stm32_pwr.c       
QUADSPI       to be done    arch/arm/src/stm32f7/stm32_qspi.c      
RCC           to be done    arch/arm/src/stm32f7/stm32_rcc.c       
RNG           to be done    arch/arm/src/stm32f7/stm32_rng.c       
RTC           to be done    arch/arm/src/stm32f7/stm32_rtc.c       
SDMMC         to be done    arch/arm/src/stm32f7/stm32_sdmmc.c     
SPI/I2S       to be done    arch/arm/src/stm32f7/stm32_i2s.c       
TIM           to be done    arch/arm/src/stm32f7/stm32_tim.c       
USART/LPUART  v3            arch/arm/src/stm32f7/stm32_serial.c    
USB           OTG FS/HS     arch/arm/src/stm32f7/stm32_otgdev.c    
WWDG          v1            not supported                          
BBSRAM        n/a           arch/arm/src/stm32f7/stm32_bbsram.c    
CAPTURE       n/a           arch/arm/src/stm32f7/stm32_capture.c   
FOC           n/a           arch/arm/src/stm32f7/stm32_foc.c       
PULSECOUNT    n/a           arch/arm/src/stm32f7/stm32_pulsecount.c
PWM           n/a           arch/arm/src/stm32f7/stm32_pwm.c       
QENCODER      n/a           arch/arm/src/stm32f7/stm32_qencoder.c  
SAI           n/a           arch/arm/src/stm32f7/stm32_sai.c       
USB host      n/a           arch/arm/src/stm32f7/stm32_otghost.c   
============  ============  =======================================

STM32G0
-------

============  ============  =================================================
Peripheral    Core version  Driver                                           
============  ============  =================================================
ADC           v1            arch/arm/src/common/stm32/stm32_adc_m0_v1.c      
AES           v1            arch/arm/src/common/stm32/stm32_aes_m0_v1.c      
CAN/FDCAN     FDCAN         not supported                                    
DAC           v1            not supported                                    
DMA           v1            arch/arm/src/common/stm32/stm32_dma_m0_v1_7ch.c  
DMAMUX        v1 7ch        arch/arm/src/common/stm32/stm32_dma_m0_v1_7ch.c  
EXTI          v2            arch/arm/src/common/stm32/stm32_exti_gpio_m0_v1.c
FLASH         G0/C0         not supported                                    
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m0_v1.c     
I2C           v1            arch/arm/src/common/stm32/stm32_i2c_m0_v1.c      
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m0_v1.c     
PWR           G0            not supported                                    
RCC           to be done    arch/arm/src/stm32g0/stm32_rcc.c                 
RNG           v1            arch/arm/src/common/stm32/stm32_rng_m0_v1.c      
RTC           RTCC M0       not supported                                    
SPI/I2S       v2            arch/arm/src/common/stm32/stm32_spi_m0_v1.c      
SYSCFG        to be done    not supported                                    
TIM           v1            arch/arm/src/common/stm32/stm32_tim_m0_v1.c      
USART/LPUART  v4            arch/arm/src/common/stm32/stm32_serial_m0_v4.c   
USB           device        arch/arm/src/common/stm32/stm32_usbdev_m0_v1.c   
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m0_v1.c     
============  ============  =================================================

STM32H5
-------

============  ============  =======================================
Peripheral    Core version  Driver                                 
============  ============  =======================================
ADC           to be done    arch/arm/src/stm32h5/stm32_adc.c       
AES/CRYP      to be done    not supported                          
CAN/FDCAN     FDCAN         arch/arm/src/stm32h5/stm32_fdcan.c     
DAC           to be done    not supported                          
DTS           to be done    arch/arm/src/stm32h5/stm32_dts.c       
ETH           to be done    arch/arm/src/stm32h5/stm32_ethernet.c  
EXTI          to be done    not supported                          
FLASH         to be done    arch/arm/src/stm32h5/stm32_flash.c     
GPIO          to be done    arch/arm/src/stm32h5/stm32_gpio.c      
GPDMA         to be done    arch/arm/src/stm32h5/stm32_dma.c       
I2C           to be done    arch/arm/src/stm32h5/stm32_i2c.c       
ICACHE        to be done    arch/arm/src/stm32h5/stm32_icache.c    
OCTOSPI       to be done    arch/arm/src/stm32h5/stm32_qspi.c      
PWR           to be done    arch/arm/src/stm32h5/stm32_pwr.c       
RCC           to be done    arch/arm/src/stm32h5/stm32_rcc.c       
SPI/I2S       to be done    arch/arm/src/stm32h5/stm32_spi.c       
TIM           to be done    arch/arm/src/stm32h5/stm32_tim.c       
USART/LPUART  v3            arch/arm/src/stm32h5/stm32_serial.c    
USB           FS            arch/arm/src/stm32h5/stm32_usbfs.c     
PULSECOUNT    n/a           arch/arm/src/stm32h5/stm32_pulsecount.c
PWM           n/a           arch/arm/src/stm32h5/stm32_pwm.c       
RNG           n/a           arch/arm/src/stm32h5/stm32_rng.c       
USB host      n/a           arch/arm/src/stm32h5/stm32_usbdrdhost.c
============  ============  =======================================

STM32H7
-------

============  ============  =======================================
Peripheral    Core version  Driver                                 
============  ============  =======================================
ADC           to be done    arch/arm/src/stm32h7/stm32_adc.c       
BDMA          to be done    arch/arm/src/stm32h7/stm32_dma.c       
CAN/FDCAN     FDCAN         arch/arm/src/stm32h7/stm32_fdcan_sock.c
DAC           to be done    not supported                          
DMA           v2 style      arch/arm/src/stm32h7/stm32_dma.c       
DMA2D         to be done    not supported                          
ETH           to be done    arch/arm/src/stm32h7/stm32_ethernet.c  
EXTI          to be done    arch/arm/src/stm32h7/stm32_exti_gpio.c 
FLASH         to be done    arch/arm/src/stm32h7/stm32_flash.c     
FMC/FSMC      to be done    arch/arm/src/stm32h7/stm32_fmc.c       
GPIO          to be done    arch/arm/src/stm32h7/stm32_gpio.c      
HSEM          to be done    not supported                          
I2C           to be done    arch/arm/src/stm32h7/stm32_i2c.c       
IWDG          v1            arch/arm/src/stm32h7/stm32_iwdg.c      
LTDC          to be done    arch/arm/src/stm32h7/stm32_ltdc.c      
MDMA          to be done    arch/arm/src/stm32h7/stm32_dma.c       
PWR           to be done    arch/arm/src/stm32h7/stm32_pwr.c       
QUADSPI       to be done    arch/arm/src/stm32h7/stm32_qspi.c      
RCC           to be done    arch/arm/src/stm32h7/stm32_rcc.c       
RNG           to be done    arch/arm/src/stm32h7/stm32_rng.c       
RTC           to be done    arch/arm/src/stm32h7/stm32_rtc.c       
SDMMC         to be done    arch/arm/src/stm32h7/stm32_sdmmc.c     
SPI/I2S       to be done    arch/arm/src/stm32h7/stm32_spi.c       
TIM           to be done    arch/arm/src/stm32h7/stm32_tim.c       
USART/LPUART  v4            arch/arm/src/stm32h7/stm32_serial.c    
USB           OTG FS/HS     arch/arm/src/stm32h7/stm32_otgdev.c    
WWDG          v1            arch/arm/src/stm32h7/stm32_wwdg.c      
AES/CRYP      n/a           arch/arm/src/stm32h7/stm32_aes.c       
BBSRAM        n/a           arch/arm/src/stm32h7/stm32_bbsram.c    
CAPTURE       n/a           arch/arm/src/stm32h7/stm32_capture.c   
LPTIM         n/a           arch/arm/src/stm32h7/stm32_lptim.c     
PULSECOUNT    n/a           arch/arm/src/stm32h7/stm32_pulsecount.c
PWM           n/a           arch/arm/src/stm32h7/stm32_pwm.c       
QENCODER      n/a           arch/arm/src/stm32h7/stm32_qencoder.c  
USB host      n/a           arch/arm/src/stm32h7/stm32_otghost.c   
============  ============  =======================================

STM32L0
-------

============  ============  =================================================
Peripheral    Core version  Driver                                           
============  ============  =================================================
ADC           v1            arch/arm/src/common/stm32/stm32_adc_m0_v1.c      
AES           v1            arch/arm/src/common/stm32/stm32_aes_m0_v1.c      
DAC           v1            not supported                                    
DMA           v1            arch/arm/src/common/stm32/stm32_dma_m0_v1_7ch.c  
EXTI          v1            arch/arm/src/common/stm32/stm32_exti_gpio_m0_v1.c
FLASH         M0 v1         not supported                                    
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m0_v1.c     
I2C           v1            arch/arm/src/common/stm32/stm32_i2c_m0_v1.c      
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m0_v1.c     
PWR           v1            arch/arm/src/common/stm32/stm32_pwr_m0_v1.c      
RCC           to be done    arch/arm/src/stm32l0/stm32_rcc.c                 
RNG           v1            arch/arm/src/common/stm32/stm32_rng_m0_v1.c      
RTC           RTCC M0       not supported                                    
SPI/I2S       v1            arch/arm/src/common/stm32/stm32_spi_m0_v1.c      
TIM           v1            arch/arm/src/common/stm32/stm32_tim_m0_v1.c      
USART/LPUART  v3            arch/arm/src/common/stm32/stm32_serial_m0_v3.c   
USB           device        arch/arm/src/common/stm32/stm32_usbdev_m0_v1.c   
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m0_v1.c     
============  ============  =================================================

STM32L1
-------

============  ============  ======================================================
Peripheral    Core version  Driver                                                
============  ============  ======================================================
ADC           v1            arch/arm/src/common/stm32/stm32_adc_m3m4_v1v2.c       
AES           v1            arch/arm/src/common/stm32/stm32_aes_m3m4_v1.c         
DAC           v1            arch/arm/src/common/stm32/stm32_dac_m3m4_v1.c         
DBGMCU        v2            not supported                                         
DMA           v1            arch/arm/src/common/stm32/stm32_dma_m3m4_v1_8ch.c     
EXTI          v1            arch/arm/src/common/stm32/stm32_exti_gpio_m3m4_v1v2.c 
FLASH         L1            arch/arm/src/common/stm32/stm32_flash_m3m4_l1.c       
FSMC          v1            arch/arm/src/common/stm32/stm32_fsmc_m3m4_v1.c        
GPIO          v1            arch/arm/src/common/stm32/stm32_gpio_m3m4_v1v2.c      
I2C           v1            arch/arm/src/common/stm32/stm32_i2c_m3m4_v1.c         
IWDG          v1            arch/arm/src/common/stm32/stm32_iwdg_m3m4_v1.c        
LCD           to be done    not supported                                         
PWR           v1            arch/arm/src/common/stm32/stm32_pwr_m3m4_v1.c         
RCC           to be done    arch/arm/src/stm32l1/stm32_rcc.c                      
RTC           RTCC L1       arch/arm/src/common/stm32/stm32_rtcc_m3m4_l1.c        
SDIO          v1            arch/arm/src/common/stm32/stm32_sdio_m3m4_v1.c        
SPI/I2S       v1            arch/arm/src/common/stm32/stm32_i2s_m3m4_v1.c         
TIM           v1            arch/arm/src/common/stm32/stm32_tim_m3m4_v1v2v3.c     
USART/LPUART  v2            arch/arm/src/common/stm32/stm32_serial_m3m4_v1v2v3v4.c
USB           device        arch/arm/src/common/stm32/stm32_otgfsdev_m3m4_v1.c    
WWDG          v1            arch/arm/src/common/stm32/stm32_wwdg_m3m4_v1.c        
============  ============  ======================================================

STM32L5
-------

============  ============  ========================================
Peripheral    Core version  Driver                                  
============  ============  ========================================
ADC           to be done    not supported                           
DAC           to be done    not supported                           
DMA/DMAMUX    to be done    not supported                           
EXTI          to be done    arch/arm/src/stm32l5/stm32l5_exti_gpio.c
FLASH         to be done    arch/arm/src/stm32l5/stm32l5_flash.c    
GPIO          to be done    arch/arm/src/stm32l5/stm32l5_gpio.c     
GTZC          to be done    not supported                           
HASH          to be done    not supported                           
I2C           to be done    not supported                           
OCTOSPI       to be done    not supported                           
PWR           to be done    arch/arm/src/stm32l5/stm32l5_pwr.c      
RCC           to be done    arch/arm/src/stm32l5/stm32l5_rcc.c      
RNG           to be done    not supported                           
RTC           to be done    not supported                           
SDMMC         to be done    not supported                           
SPI/I2S       to be done    arch/arm/src/stm32l5/stm32l5_spi.c      
TIM/LPTIM     to be done    arch/arm/src/stm32l5/stm32l5_tim.c      
USART/LPUART  v3            arch/arm/src/stm32l5/stm32l5_serial.c   
USB           device        not supported                           
============  ============  ========================================

STM32N6
-------

============  ============  ===================================
Peripheral    Core version  Driver                             
============  ============  ===================================
ADC           to be done    not supported                      
CAN/FDCAN     to be done    not supported                      
DAC           to be done    not supported                      
ETH           to be done    not supported                      
EXTI          to be done    not supported                      
GPIO          to be done    arch/arm/src/stm32n6/stm32_gpio.c  
GPDMA         to be done    not supported                      
I2C           to be done    not supported                      
PWR           to be done    arch/arm/src/stm32n6/stm32_pwr.c   
RCC           to be done    arch/arm/src/stm32n6/stm32_rcc.c   
SPI/I2S       to be done    not supported                      
TIM           to be done    not supported                      
USART/LPUART  v4            arch/arm/src/stm32n6/stm32_serial.c
USB           device        not supported                      
XSPI          to be done    not supported                      
============  ============  ===================================

STM32U5
-------

============  ============  ======================================
Peripheral    Core version  Driver                                
============  ============  ======================================
ADC           to be done    not supported                         
DAC           to be done    not supported                         
DCACHE        to be done    not supported                         
EXTI          to be done    arch/arm/src/stm32u5/stm32_exti_gpio.c
FLASH         to be done    arch/arm/src/stm32u5/stm32_flash.c    
GPIO          to be done    arch/arm/src/stm32u5/stm32_gpio.c     
GPDMA         to be done    not supported                         
HASH          to be done    not supported                         
I2C           to be done    arch/arm/src/stm32u5/stm32_i2c.c      
ICACHE        to be done    not supported                         
OCTOSPI       to be done    not supported                         
PWR           to be done    arch/arm/src/stm32u5/stm32_pwr.c      
RCC           to be done    arch/arm/src/stm32u5/stm32_rcc.c      
RNG           to be done    not supported                         
RTC           to be done    not supported                         
SAES/AES      to be done    not supported                         
SDMMC         to be done    not supported                         
SPI/I2S       to be done    arch/arm/src/stm32u5/stm32_spi.c      
TAMP          to be done    not supported                         
TIM/LPTIM     to be done    arch/arm/src/stm32u5/stm32_tim.c      
USART/LPUART  v3            arch/arm/src/stm32u5/stm32_serial.c   
USB           device        not supported                         
============  ============  ======================================

STM32WB
-------

============  ============  ========================================
Peripheral    Core version  Driver                                  
============  ============  ========================================
ADC           to be done    not supported                           
AES           to be done    not supported                           
DAC           to be done    not supported                           
DMA           to be done    arch/arm/src/stm32wb/stm32wb_dma.c      
EXTI          to be done    arch/arm/src/stm32wb/stm32wb_exti_gpio.c
FLASH         to be done    arch/arm/src/stm32wb/stm32wb_flash.c    
GPIO          to be done    arch/arm/src/stm32wb/stm32wb_gpio.c     
I2C           to be done    arch/arm/src/stm32wb/stm32wb_i2c.c      
IPCC          to be done    arch/arm/src/stm32wb/stm32wb_ipcc.c     
PWR           to be done    arch/arm/src/stm32wb/stm32wb_pwr.c      
Radio         to be done    not supported                           
RCC           to be done    arch/arm/src/stm32wb/stm32wb_rcc.c      
RNG           to be done    not supported                           
RTC           to be done    arch/arm/src/stm32wb/stm32wb_rtc.c      
SPI/I2S       to be done    arch/arm/src/stm32wb/stm32wb_spi.c      
TIM/LPTIM     to be done    arch/arm/src/stm32wb/stm32wb_tim.c      
USART/LPUART  v4            arch/arm/src/stm32wb/stm32wb_serial.c   
USB           device        not supported                           
============  ============  ========================================

STM32WL5
--------

============  ============  ==========================================
Peripheral    Core version  Driver                                    
============  ============  ==========================================
ADC           to be done    not supported                             
AES           to be done    not supported                             
DAC           to be done    not supported                             
DMA           to be done    not supported                             
EXTI          to be done    arch/arm/src/stm32wl5/stm32wl5_exti_gpio.c
FLASH         to be done    arch/arm/src/stm32wl5/stm32wl5_flash.c    
GPIO          to be done    arch/arm/src/stm32wl5/stm32wl5_gpio.c     
I2C           to be done    not supported                             
IPCC          to be done    arch/arm/src/stm32wl5/stm32wl5_ipcc.c     
PWR           to be done    arch/arm/src/stm32wl5/stm32wl5_pwr.c      
Radio         to be done    not supported                             
RCC           to be done    arch/arm/src/stm32wl5/stm32wl5_rcc.c      
RNG           to be done    not supported                             
RTC           to be done    not supported                             
SPI/I2S       to be done    arch/arm/src/stm32wl5/stm32wl5_spi.c      
TIM/LPTIM     to be done    arch/arm/src/stm32wl5/stm32wl5_tim.c      
USART/LPUART  v3            arch/arm/src/stm32wl5/stm32wl5_serial.c   
USB           to be done    not supported                             
============  ============  ==========================================

Kconfig option organization
===========================

STM32 common Kconfig symbols are split into three layers, each with a fixed
home so contributors always know where a given option belongs:

=================================  ============================  ====================
Layer                              Symbol form                   File
=================================  ============================  ====================
Capability flags (hidden)          ``STM32_HAVE_*``              ``Kconfig.have``
Instance feature flags (hidden)    ``STM32_<INSTANCE>_HAVE_*``   ``Kconfig.<periph>``
Peripheral selection               ``STM32_<PERIPH>``            ``Kconfig.periph``
Peripheral options                 ``STM32_<PERIPH>_*``          ``Kconfig.<periph>``
=================================  ============================  ====================

All ``STM32_HAVE_*`` and ``STM32_HAVE_IP_*`` symbols are defined in the single
file ``arch/arm/src/common/stm32/Kconfig.have`` — the authoritative description
of what hardware exists.  The ``STM32_HAVE_IP_*`` selectors listed above are a
subset of it.  The rules for these capability flags are:

* They are prompt-less ``bool`` symbols, defined exactly once.  Never give a
  capability flag a prompt.
* Family Kconfig files (``arch/arm/src/stm32<family>/Kconfig``) ``select`` these
  symbols to describe the silicon of each chip.
* Presence and feature flags may use ``default y if <STM32_HAVE_IP_* flag>`` so
  the right instances and features light up automatically once a family selects
  its IP-core version.
* User-visible options in ``Kconfig.periph`` and ``Kconfig.<periph>`` should
  ``depends on`` the capability flags rather than on family names.

The ``STM32_HAVE_`` prefix is reserved for **chip capabilities** — things the
silicon *has*, independent of any single peripheral instance.  Those live in
``Kconfig.have``.  A flag that instead describes a feature of one **specific
peripheral instance** ("ADC3 *has* a DMA path", "USART1 *supports* RX DMA") is
not a chip capability: it must be named ``STM32_<INSTANCE>_HAVE_<FEATURE>``
(``HAVE`` *after* the instance) and defined in that peripheral's
``Kconfig.<periph>`` file, never in ``Kconfig.have``.  It is still a prompt-less
``bool`` ``select``-ed by the family Kconfig files, just homed with the rest of
its peripheral's options.  For example:

============================  ============================  ====================
Wrong (chip-capability form)  Correct (instance-feature)    Home
============================  ============================  ====================
``STM32_HAVE_ADC3_DMA``       ``STM32_ADC3_HAVE_DMA``       ``Kconfig.adc``
``STM32_HAVE_SDADC1_DMA``     ``STM32_SDADC1_HAVE_DMA``     ``Kconfig.sdadc``
``STM32_HAVE_USART1_RXDMA``   ``STM32_USART1_HAVE_RXDMA``   ``Kconfig.uart``
``STM32_HAVE_PHY_POLLED``     ``STM32_PHY_HAVE_POLLED``     ``Kconfig.eth``
============================  ============================  ====================

Capability-flag naming:

* IP-core version selectors: ``STM32_HAVE_IP_<PERIPH>_<CORE>_V<N>`` (see
  `IP-core naming convention`_; the version number resets per core).
* Per-instance presence: ``STM32_HAVE_<INSTANCE>`` (for example
  ``STM32_HAVE_ADC2`` or ``STM32_HAVE_USART3``).
* Feature capability (chip-wide, **no** instance number): ``STM32_HAVE_<FEATURE>``
  (for example ``STM32_HAVE_ADC_OVERSAMPLE``).  A feature tied to a specific
  instance is **not** written this way — see ``STM32_<INSTANCE>_HAVE_<FEATURE>``
  above.
* Per-instance feature: ``STM32_<INSTANCE>_HAVE_<FEATURE>`` (for example
  ``STM32_ADC3_HAVE_DMA``), defined in ``Kconfig.<periph>``.

Source naming rules
===================

IP-core naming convention
-------------------------

STM32 common files use a ``<core>_<version>`` naming convention that
encodes the CPU core family and peripheral IP version in filenames and
Kconfig symbols.  Version numbers are reset per core: the first version
of a peripheral on a given core is ``V1``.

**Header file naming** (under ``arch/arm/src/common/stm32/``):

- Facade headers: ``stm32_<periph>.h`` — dispatch on Kconfig IP symbols
- M0-core variants: ``stm32_<periph>_m0_v1.h``
- M3/M4-core variants: ``stm32_<periph>_m3m4_v1.h``, ``stm32_<periph>_m3m4_v2.h``
- Combined variants: ``stm32_<periph>_m3m4_v1v2.h``, ``stm32_<periph>_m3m4_v1v2v3.h``

**Source file naming** (``.c`` files):

- Follow the same convention as headers: ``stm32_gpio_m3m4_v1v2.c``,
  ``stm32_adc_m0_v1.c``, ``stm32_tim_m0_v1.c``, ``stm32_pwm_m3m4_v1v2v3.c``

**Kconfig symbol naming** (``CONFIG_STM32_HAVE_IP_<PERIPH>_<CORE>_<VERSION>``):

- M0: ``CONFIG_STM32_HAVE_IP_GPIO_M0_V1``, ``CONFIG_STM32_HAVE_IP_ADC_M0_V1``
- M3/M4: ``CONFIG_STM32_HAVE_IP_GPIO_M3M4_V1``,
  ``CONFIG_STM32_HAVE_IP_ADC_M3M4_V1``, ``CONFIG_STM32_HAVE_IP_ADC_M3M4_V2``,
  ``CONFIG_STM32_HAVE_IP_TIMERS_M3M4_V1``, ``CONFIG_STM32_HAVE_IP_TIMERS_M3M4_V2``,
  ``CONFIG_STM32_HAVE_IP_TIMERS_M3M4_V3``

Note that in the future the core prefix may be dropped and a single
version number space used across all cores.  For now the core prefix is
kept to make migration of families into common code simpler.

Common STM32 files should follow these rules:

* Files shared by more than one family are placed under
  ``arch/arm/src/common/stm32``.
* Board-facing public headers use stable peripheral names such as
  ``stm32_adc.h``, ``stm32_gpio.h``, and ``stm32_uart.h``.  Boards and
  drivers should not include IP-versioned public headers directly.
* Public STM32 symbols must use canonical ``stm32_*`` names, not
  family-prefixed names (for example, use ``stm32_tim_init``,
  ``stm32_dmachannel``, and ``stm32_caninitialize`` rather than
  ``stm32l4_*``/``stm32h7_*`` forms).
* Public API-facing type names must also use canonical ``stm32_*`` forms
  (for example, ``struct stm32_tim_dev_s``, ``struct stm32_dmaregs_s``,
  and ``struct stm32_freerun_s``).
* When an equivalent STM32 API already exists, new family code must reuse
  the exact public function name and compatible prototype/type signature.
* Family root aggregation headers should use ``stm32.h`` and family code
  should include ``<stm32.h>`` for that root header include path.
* Hardware register selectors keep the ``hardware/stm32_<peripheral>.h``
  public name and may include hardware-version headers when those headers
  describe real register layouts.
* Private common implementation selectors may be split only when the API,
  register map, or reusable hardware shape really differs.  Prefer capability
  names over family names for these splits.
* Common STM32 source and hardware header names must not encode composite
  family groups.  If a reusable distinction is still named after a family, the
  next cleanup step is to map that distinction to an IP-core version or a
  hardware feature.
* Public compatibility selectors such as ``stm32_adc.h`` and
  ``hardware/stm32_adc.h`` keep their public include names while selecting the
  proper common implementation internally.

Hardware definition naming convention
--------------------------------------

All STM32 hardware definitions (IRQ vectors, peripheral counts, SRAM sizes)
must use the standard ``STM32_`` prefix rather than family-specific prefixes.
Family-specific prefixes prevent common code from referencing hardware
definitions across families and create unnecessary differences between ports.

**IRQ vectors** must use ``STM32_IRQ_`` (not ``STM32L4_IRQ_``,
``STM32WB_IRQ_``, etc.).

**Peripheral count macros** (``NATIM``, ``NADC``, ``NSPI``, etc.) and SRAM
size macros must use ``STM32_`` prefix (not ``STM32F7_``, ``STM32H5_``, etc.).

**Per-family hardware register definitions** (in ``arch/arm/src/<family>/hardware/``)
must use ``STM32_XXX_`` (not ``STM32L4_XXX_``, ``STM32WB4_XXX_``, etc.).

Next steps
==========

The goal is for all STM32 families to follow the rules above, so that shared
code is selected by peripheral IP version instead of family name and code
duplication is limited.  The remaining work is:

Architecture
------------

* Migrate the remaining families to use the common ``arch/arm/src/common/stm32``
  code. 
* Review every family's selectors so they use the correct IP cores and select
  only the peripherals each family actually supports.
* Remove all family-specific options that were left for later, and simplify the
  complex Kconfig conditions.
* Unify the remaining peripheral-driver public headers so the API is the same
  for all families.
* Better organize the CMake and Make build files for common STM32. Now it's a mess.
* Unify all supported features across families.

Boards
------

* Move all families to shared ``boards/common/stm32`` board code, including
  boards that do not yet use the recently added common code.
* Unify all supported features across boards of the same class (such as the
  Nucleo boards) so they follow the same patterns.

Kconfig simplification
----------------------

Many Kconfig conditions are still long, mix normalized ``STM32_HAVE_IP_*``
flags with family names, and rely on composite family-group symbols
(``STM32_COMMON_LEGACY``, ``STM32_COMMON_F7_H7``, ``STM32_COMMON_L4_L5_U5``,
``STM32_COMMON_L4_H5_L5_U5``, …) that the naming rules forbid.  These are the
most likely to need rework and should be reduced to depend on capability flags
only:

* ``Kconfig.tim`` and ``Kconfig.uart`` hold the bulk of the family-name
  conditions (hundreds of lines each) and are the biggest cleanup.
* ``Kconfig.adc``, ``Kconfig.periph`` and ``Kconfig.have`` carry many mixed
  family/IP ``||`` chains (for example ``(STM32_HAVE_IP_ADC_M3M4_V1 ||
  STM32_HAVE_IP_ADC_M3M4_V2) || ARCH_CHIP_STM32F7``), where F7/H5/L4 are patched
  in by family name only because they have no IP flag yet.
* Replace the composite ``STM32_COMMON_*`` family-group symbols with per-IP-core
  or per-feature flags.
