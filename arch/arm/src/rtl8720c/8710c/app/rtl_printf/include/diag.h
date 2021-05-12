/**************************************************************************//**
 * @file     diag.h
 * @brief    This file defines macros for debug message printing.
 * @version  V1.00
 * @date     2016-7-20
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef _DIAG_H_
#define _DIAG_H_
#include "platform_conf.h"
#include "basic_types.h"
#include <stdarg.h>
#include <stddef.h> /* for size_t */
#include "log_buf.h"
#include "rt_printf.h"
#include "utility.h"

#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
extern osMutexId PrintLock_id;
extern log_buf_type_t debug_log;
#endif

#if defined(ROM_REGION)
extern u32 ConfigDebugErr;
extern u32 ConfigDebugInfo;
extern u32 ConfigDebugWarn;
#else
#define ConfigDebugErr          (*(utility_stubs.config_debug_err))
#define ConfigDebugWarn         (*(utility_stubs.config_debug_warn))
#define ConfigDebugInfo         (*(utility_stubs.config_debug_info))
#if !defined(CONFIG_BUILD_SECURE) && !defined(CONFIG_BUILD_NONSECURE)
#define ConfigDebugErr_NS       (*(__rom_stubs_utility_ns.config_debug_err))
#define ConfigDebugWarn_NS      (*(__rom_stubs_utility_ns.config_debug_warn))
#define ConfigDebugInfo_NS      (*(__rom_stubs_utility_ns.config_debug_info))
#endif
#endif

#if !defined(CONFIG_BUILD_SECURE) && !defined(CONFIG_BUILD_NONSECURE)
#define DBG_ERR_MSG_ON(x)       do {(ConfigDebugErr |= (x)); (ConfigDebugErr_NS |= (x));} while(0)
#define DBG_WARN_MSG_ON(x)      do {(ConfigDebugWarn |= (x)); (ConfigDebugWarn_NS |= (x));} while(0)
#define DBG_INFO_MSG_ON(x)      do {(ConfigDebugInfo |= (x)); (ConfigDebugInfo_NS |= (x));} while(0)

#define DBG_ERR_MSG_OFF(x)      do {(ConfigDebugErr &= ~(x)); (ConfigDebugErr_NS &= ~(x));} while(0)
#define DBG_WARN_MSG_OFF(x)     do {(ConfigDebugWarn &= ~(x)); (ConfigDebugWarn_NS &= ~(x));} while(0)
#define DBG_INFO_MSG_OFF(x)     do {(ConfigDebugInfo &= ~(x)); (ConfigDebugInfo_NS &= ~(x));} while(0)
#else
#define DBG_ERR_MSG_ON(x)       (ConfigDebugErr |= (x))
#define DBG_WARN_MSG_ON(x)      (ConfigDebugWarn |= (x))
#define DBG_INFO_MSG_ON(x)      (ConfigDebugInfo |= (x))

#define DBG_ERR_MSG_OFF(x)      (ConfigDebugErr &= ~(x))
#define DBG_WARN_MSG_OFF(x)     (ConfigDebugWarn &= ~(x))
#define DBG_INFO_MSG_OFF(x)     (ConfigDebugInfo &= ~(x))
#endif

// Define debug group
#define	_DBG_BOOT_          	0x00000001
#define	_DBG_GDMA_          	0x00000002
#define	_DBG_GPIO_			    0x00000004
#define	_DBG_TIMER_         	0x00000008
#define	_DBG_I2C_           	0x00000010
#define	_DBG_PWM_           	0x00000040
#define	_DBG_SSI_           	0x00000080
#define	_DBG_SPI_FLASH_         0x00000100
#define	_DBG_SDR_           	0x00000200
#define	_DBG_SDIO_DEV_         	0x00000400
#define	_DBG_UART_          	0x00000800
#define _DBG_CRYPTO_            0x00001000
#define _DBG_SCE_               0x00002000
#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define _DBG_MAIN_              0x00200000
#define _DBG_FIBO_              0x00400000
#endif
#define _DBG_MISC_              0x40000000
#define	_DBG_FAULT_        	    0x80000000

#if CONFIG_LIGHT_PRINTF
// Light weight printf
#if defined(ROM_REGION)
#define dbg_printf(...)                     do { if(ConfigDebugErr & _DBG_BOOT_){_rtl_printf(__VA_ARGS__);} }while(0)
#define dbg_sprintf(buf, ...)               do { _rtl_sprintf(buf, __VA_ARGS__); }while(0)
#define dbg_snprintf(buf, size, ...)        do { _rtl_snprintf(buf, size, __VA_ARGS__); }while(0)
// Verifi mode, _DbgDump() for ROM Code
#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define _DbgDump(...) do{\
    if(PrintLock_id != NULL) \
        _mutex_rtl_printf(PrintLock_id,__VA_ARGS__);\
    else \
        _rtl_printf(__VA_ARGS__); \
}while(0)
#else
#define _DbgDump                            _rtl_printf
#endif

#else
#define dbg_printf                          rt_printfl
#define dbg_sprintf                         rt_sprintfl
#define dbg_snprintf                        rt_snprintfl
// Verifi mode, _DbgDump() for RAM Code
#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define _DbgDump(...) do{\
    if(PrintLock_id != NULL) \
        _mutex_rtl_printf(PrintLock_id,__VA_ARGS__);\
    else \
        rt_printfl(__VA_ARGS__); \
}while(0)
#else
#define _DbgDump                            rt_printfl
#endif
#endif

#else
// all-format printf
#if defined(ROM_REGION)
#define dbg_printf(...)                     do { if(ConfigDebugErr & _DBG_BOOT_){_xprintf(__VA_ARGS__);} }while(0)
#define dbg_sprintf(buf, ...)               do { _xsprintf(buf, __VA_ARGS__); }while(0)
#define dbg_snprintf(buf, size, ...)        do { _xsnprintf(buf, size, __VA_ARGS__); }while(0)
#define _DbgDump                            _xprintf
#else
#define dbg_printf                          rt_printf
#define dbg_sprintf                         rt_sprintf
#define dbg_snprintf                        rt_snprintf
#define _DbgDump                            rt_printf
#endif
#endif

#if defined(ROM_REGION)
#define log_printf(plog, ...)       do { _log_buf_printf(plog, __VA_ARGS__); }while(0)
#else
// Verifi mode, log_printf() for RAM Code
#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define log_printf(...) do {\
    rt_log_printf(&debug_log,__VA_ARGS__); \
}while(0)
#else
#define log_printf                  rt_log_printf
#endif
#endif

#define DRIVER_PREFIX	        "[Driver]: "
#define HAL_PREFIX              "[HAL]: "

#define BOOT_ERR_PREFIX         "[BOOT Err]"
#define BOOT_WARN_PREFIX        "[BOOT Wrn]"
#define BOOT_INFO_PREFIX        "[BOOT Inf]"

#define GDMA_ERR_PREFIX         "[GDMA Err]"
#define GDMA_WARN_PREFIX        "[GDMA Wrn]"
#define GDMA_INFO_PREFIX        "[GDMA Inf]"

#define GPIO_ERR_PREFIX         "[GPIO Err]"
#define GPIO_WARN_PREFIX        "[GPIO Wrn]"
#define GPIO_INFO_PREFIX        "[GPIO Inf]"

#define TIMER_ERR_PREFIX        "[TIMR Err]"
#define TIMER_WARN_PREFIX       "[TIMR Wrn]"
#define TIMER_INFO_PREFIX       "[TIMR Inf]"

#define I2C_ERR_PREFIX          "[I2C  Err]"
#define I2C_WARN_PREFIX         "[I2C  Wrn]"
#define I2C_INFO_PREFIX         "[I2C  Inf]"

#define PWM_ERR_PREFIX          "[PWM  Err]"
#define PWM_WARN_PREFIX         "[PWM  Wrn]"
#define PWM_INFO_PREFIX         "[PWM  Inf]"

#define SSI_ERR_PREFIX          "[SSI  Err]"
#define SSI_WARN_PREFIX         "[SSI  Wrn]"
#define SSI_INFO_PREFIX         "[SSI  Inf]"

#define SPIF_ERR_PREFIX         "[SPIF Err]"
#define SPIF_WARN_PREFIX        "[SPIF Wrn]"
#define SPIF_INFO_PREFIX        "[SPIF Inf]"

#define SDR_ERR_PREFIX          "[SDR  Err]"
#define SDR_WARN_PREFIX         "[SDR  Wrn]"
#define SDR_INFO_PREFIX         "[SDR  Inf]"

#define SDIO_DEV_ERR_PREFIX     "[SDIOD  Err]"
#define SDIO_DEV_WARN_PREFIX    "[SDIOD  Wrn]"
#define SDIO_DEV_INFO_PREFIX    "[SDIOD  Inf]"

#define UART_ERR_PREFIX         "[UART Err]"
#define UART_WARN_PREFIX        "[UART Wrn]"
#define UART_INFO_PREFIX        "[UART Inf]"

#define IPSEC_ERR_PREFIX        "[CRYP Err]"
#define IPSEC_WARN_PREFIX       "[CRYP Wrn]"
#define IPSEC_INFO_PREFIX       "[CRYP Inf]"

#define MISC_ERR_PREFIX         "[MISC Err]"
#define MISC_WARN_PREFIX        "[MISC Wrn]"
#define MISC_INFO_PREFIX        "[MISC Inf]"

#define SCE_ERR_PREFIX          "[SCE Err]"
#define SCE_WARN_PREFIX         "[SCE Wrn]"
#define SCE_INFO_PREFIX         "[SCE Inf]"

#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define MAIN_INFO_PREFIX        "[MAIN Inf]"
#define FIBO_INFO_PREFIX        "[FIBO Inf]"
#endif


#ifndef likely
#define likely(x)               (x)
#define unlikely(x)             (x)
#endif

#if CONFIG_DEBUG_LOG

#if CONFIG_DEBUG_ERROR      // if Build-In Debug Error Message

#define DBG_BOOT_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_BOOT_)) \
        _DbgDump("\r" BOOT_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_GDMA_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_GDMA_)) \
        _DbgDump("\r" GDMA_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_GPIO_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_GPIO_)) \
        _DbgDump("\r" GPIO_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_TIMER_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_TIMER_)) \
        _DbgDump("\r" TIMER_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_I2C_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_I2C_)) \
        _DbgDump("\r" I2C_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_PWM_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_PWM_)) \
        _DbgDump("\r" PWM_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SSI_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_SSI_)) \
        _DbgDump("\r" SSI_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SPIF_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_SPI_FLASH_)) \
        _DbgDump("\r" SPIF_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SDR_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_SDR_)) \
        _DbgDump("\r" SDR_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SDIO_DEV_ERR(...)     do {\
        if (likely(ConfigDebugErr & _DBG_SDIO_DEV_)) \
            _DbgDump("\r" SDIO_DEV_ERR_PREFIX __VA_ARGS__);\
    }while(0)

#define DBG_UART_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_UART_)) \
        _DbgDump("\r" UART_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_CRYPTO_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_CRYPTO_)) \
        _DbgDump("\r" IPSEC_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_MISC_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_MISC_)) \
        _DbgDump("\r" MISC_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SDIOH_ERR(...)     do {\
    if (likely(ConfigDebugErr & _DBG_SDIO_HOST_)) \
        _DbgDump("\r" SDIOH_ERR_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SCE_ERR(...)     do {\
                if (likely(ConfigDebugErr & _DBG_SCE_)) \
                    _DbgDump("\r" SCE_ERR_PREFIX __VA_ARGS__);\
            }while(0)

#else   // else of "#if CONFIG_DEBUG_ERROR"

#define DBG_BOOT_ERR(...)
#define DBG_GDMA_ERR(...)
#define DBG_GPIO_ERR(...)
#define DBG_TIMER_ERR(...)
#define DBG_I2C_ERR(...)
#define DBG_PWM_ERR(...)
#define DBG_SSI_ERR(...)
#define DBG_SPIF_ERR(...)
#define DBG_SDR_ERR(...)
#define DBG_UART_ERR(...)
#define DBG_CRYPTO_ERR(...)
#define DBG_MISC_ERR(...)

#define MSG_MBOX_ERR(...)
#define MSG_TIMER_ERR(...)

#endif  // end of else of "#if CONFIG_DEBUG_ERROR"

// =============================================================

#if CONFIG_DEBUG_WARN     // if Build-In Debug Warring Message

#define DBG_BOOT_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn& _DBG_BOOT_)) \
        _DbgDump("\r" BOOT_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_GDMA_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_GDMA_)) \
        _DbgDump("\r" GDMA_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_GPIO_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_GPIO_)) \
        _DbgDump("\r" GPIO_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_TIMER_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_TIMER_)) \
        _DbgDump("\r" TIMER_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_I2C_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_I2C_)) \
        _DbgDump("\r" I2C_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_PWM_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_PWM_)) \
        _DbgDump("\r" PWM_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SSI_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_SSI_)) \
        _DbgDump("\r" SSI_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SPIF_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_SPI_FLASH_)) \
        _DbgDump("\r" SPIF_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SDR_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_SDR_)) \
        _DbgDump("\r" SDR_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SDIO_DEV_WARN(...)     do {\
        if (unlikely(ConfigDebugWarn & _DBG_SDIO_DEV_)) \
            _DbgDump("\r" SDIO_DEV_WARN_PREFIX __VA_ARGS__);\
    }while(0)

#define DBG_UART_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_UART_)) \
        _DbgDump("\r" UART_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_CRYPTO_WARN(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_CRYPTO_)) \
        _DbgDump("\r" IPSEC_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_MISC_WARN(...)     do {\
    if (likely(ConfigDebugWarn & _DBG_MISC_)) \
        _DbgDump("\r" MISC_WARN_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SCE_WARN(...)     do {\
        if (likely(ConfigDebugWarn & _DBG_SCE_)) \
            _DbgDump("\r" SCE_ERR_PREFIX __VA_ARGS__);\
    }while(0)

#else   // else of "#if CONFIG_DEBUG_WARN"

#define DBG_BOOT_WARN(...)
#define DBG_GDMA_WARN(...)
#define DBG_GPIO_WARN(...)
#define DBG_TIMER_WARN(...)
#define DBG_I2C_WARN(...)
#define DBG_PWM_WARN(...)
#define DBG_SSI_WARN(...)
#define DBG_SPIF_WARN(...)
#define DBG_SDR_WARN(...)
#define DBG_UART_WARN(...)
#define DBG_CRYPTO_WARN(...)

#define DBG_MISC_WARN(...)

#define MSG_MBOX_WARN(...)
#define MSG_TIMER_WARN(...)

#endif  // end of else of "#if CONFIG_DEBUG_WARN"

// =============================================================
// Verifi mode for IP_CALL_BACK_INFO()
#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define DBG_I2C_CLBKINFO(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_I2C_)) \
        log_printf("\r" I2C_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_UART_CLBKINFO(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_UART_)) \
        log_printf("\r" UART_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_TIMER_CLBKINFO(...)     do {\
    if (unlikely(ConfigDebugWarn & _DBG_TIMER_)) \
        log_printf("\r" TIMER_INFO_PREFIX __VA_ARGS__);\
}while(0)

#endif

// =============================================================

#if CONFIG_DEBUG_INFO     // if Build-In Debug Information Message

#define DBG_BOOT_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_BOOT_)) \
        _DbgDump("\r" BOOT_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_GDMA_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_GDMA_)) \
        _DbgDump("\r" GDMA_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_GPIO_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_GPIO_)) \
        _DbgDump("\r" GPIO_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_TIMER_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_TIMER_)) \
        _DbgDump("\r" TIMER_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_I2C_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_I2C_)) \
        _DbgDump("\r" I2C_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_PWM_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_PWM_)) \
        _DbgDump("\r" PWM_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SSI_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_SSI_)) \
        _DbgDump("\r" SSI_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SPIF_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_SPI_FLASH_)) \
        _DbgDump("\r" SPIF_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SDR_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_SDR_)) \
        _DbgDump("\r" SDR_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SDIO_DEV_INFO(...)     do {\
        if (unlikely(ConfigDebugInfo & _DBG_SDIO_DEV_)) \
            _DbgDump("\r" SDIO_DEV_INFO_PREFIX __VA_ARGS__);\
    }while(0)

#define DBG_UART_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_UART_)) \
        _DbgDump("\r" UART_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_CRYPTO_INFO(...)     do {\
    if (unlikely(ConfigDebugInfo & _DBG_CRYPTO_)) \
        _DbgDump("\r" IPSEC_INFO_PREFIX __VA_ARGS__);\
}while(0)

#define DBG_SCE_INFO(...)     do {\
                if (unlikely(ConfigDebugInfo & _DBG_SCE_)) \
                    _DbgDump("\r" SCE_INFO_PREFIX __VA_ARGS__);\
            }while(0)

#define DBG_MISC_INFO(...)     do {\
    if (likely(ConfigDebugInfo & _DBG_MISC_)) \
        _DbgDump("\r" MISC_INFO_PREFIX __VA_ARGS__);\
}while(0)

#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define DBG_MAIN_INFO(...)     do {\
        if (unlikely(ConfigDebugInfo & _DBG_MAIN_)) \
            _DbgDump("\r" MAIN_INFO_PREFIX __VA_ARGS__);\
    }while(0)

#define DBG_FIBO_INFO(...)     do {\
        if (unlikely(ConfigDebugInfo & _DBG_FIBO_)) \
            _DbgDump("\r" FIBO_INFO_PREFIX __VA_ARGS__);\
    }while(0)
#endif

#else   // else of "#if CONFIG_DEBUG_INFO"

#define DBG_BOOT_INFO(...)
#define DBG_GDMA_INFO(...)
#define DBG_GPIO_INFO(...)
#define DBG_TIMER_INFO(...)
#define DBG_I2C_INFO(...)
#define DBG_PWM_INFO(...)
#define DBG_SSI_INFO(...)
#define DBG_SPIF_INFO(...)
#define DBG_SDR_INFO(...)
#define DBG_UART_INFO(...)
#define DBG_CRYPTO_INFO(...)
#define DBG_SCE_INFO(...)
#define DBG_SDIO_DEV_INFO(...)

#if defined(CONFIG_VRF_MODE) && (CONFIG_VRF_MODE==1)
#define DBG_MAIN_INFO(...)
#define DBG_FIBO_INFO(...)
#endif

#define DBG_MISC_INFO(...)

#define MSG_MBOX_INFO(...)
#define MSG_TIMER_INFO(...)

#endif  // end of else of "#if CONFIG_DEBUG_INFO"

#define DBG_8710C(...)     do {\
    if (unlikely(ConfigDebugErr & _DBG_MISC_)) \
        _DbgDump("\r" __VA_ARGS__);\
}while(0)

#define MONITOR_LOG(...)     do {\
    if (unlikely(ConfigDebugErr & _DBG_MISC_)) \
        _DbgDump( __VA_ARGS__);\
}while(0)

#define DBG_ERROR_LOG(...)     do {\
    if (unlikely(ConfigDebugErr & _DBG_FAULT_)) \
        _DbgDump( __VA_ARGS__);\
}while(0)

#ifdef __GNUC__
#define DBG_ASSERT(x)     do {\
        if (unlikely(!(x))) \
            _DbgDump("Assertion: %s:%s, %d\n", __FILE__, __func__, __LINE__);\
    }while(0)
#endif

#ifdef __ICCARM__
#define DBG_ASSERT(x)     do {\
        if (unlikely(!(x))) \
            _DbgDump("Assertion: %s:%s, %d\n", __FILE__, __func__, __LINE__);\
    }while(0)
#endif

#else   // else of "#if CONFIG_DEBUG_LOG"
#define DBG_8710C(...)

#define MONITOR_LOG(...)

#define DBG_ERROR_LOG(...)

#define DBG_BOOT_ERR(...)
#define DBG_GDMA_ERR(...)
#define DBG_GPIO_ERR(...)
#define DBG_TIMER_ERR(...)
#define DBG_I2C_ERR(...)
#define DBG_PWM_ERR(...)
#define DBG_SPIF_ERR(...)
#define DBG_SDR_ERR(...)
#define DBG_UART_ERR(...)
#define DBG_CRYPTO_ERR(...)
#define DBG_MISC_ERR(...)

#define MSG_MBOX_ERR(...)
#define MSG_TIMER_ERR(...)

#define DBG_BOOT_WARN(...)
#define DBG_GDMA_WARN(...)
#define DBG_GPIO_WARN(...)
#define DBG_TIMER_WARN(...)
#define DBG_I2C_WARN(...)
#define DBG_PWM_WARN(...)
#define DBG_SSI_WARN(...)
#define DBG_SPIF_WARN(...)
#define DBG_SDR_WARN(...)
#define DBG_UART_WARN(...)
#define DBG_CRYPTO_WARN(...)
#define DBG_MISC_WARN(...)

#define MSG_MBOX_WARN(...)
#define MSG_TIMER_WARN(...)

#define DBG_BOOT_INFO(...)
#define DBG_GDMA_INFO(...)
#define DBG_GPIO_INFO(...)
#define DBG_TIMER_INFO(...)
#define DBG_I2C_INFO(...)
#define DBG_PWM_INFO(...)
#define DBG_SSI_INFO(...)
#define DBG_SPIF_INFO(...)
#define DBG_SDR_INFO(...)
#define DBG_UART_INFO(...)
#define DBG_CRYPTO_INFO(...)
#define DBG_MISC_INFO(...)

#define MSG_MBOX_INFO(...)
#define MSG_TIMER_INFO(...)

#define DBG_ASSERT(x)

#endif

#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define IDENT_ONE_SPACE    " "
#define IDENT_TWO_SPACE    "  "
#define IDENT_FOUR_SPACE   "    "
#define IDENT_SIX_SPACE    "      "
#define IDENT_EIGHT_SPACE  "        "

#if CONFIG_DEBUG_LOG
typedef enum _DBG_CFG_TYPE_ {
	DBG_CFG_ERR=0,
	DBG_CFG_WARN=1,
	DBG_CFG_INFO=2
} DBG_CFG_TYPE;

typedef struct _DBG_CFG_CMD_ {
	u8 cmd_name[16];
	u32	cmd_type;
} DBG_CFG_CMD, *PDBG_CFG_CMD;

#endif

typedef enum _CONSOLE_OP_STAGE_ {
    ROM_STAGE = 0,
    RAM_STAGE = 1
}CONSOLE_OP_STAGE;

#endif //_DIAG_H_
