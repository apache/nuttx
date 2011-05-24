/****************************************************************************
 * include/nuttx/resource.h
 *
 *   Copyright(C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/** \file
 *  \author Uros Platise
 *  \brief System Resources
 * 
 * System resource provides interface to device drivers to list possible
 * configurations that can be changed by system to minimize (optimize)
 * energy and power usage, to lock a specific resource, fixing it at
 * desired value and to support multiple power down capabilities by the
 * up_idle().
 * 
 * System resources are provided on the chip or board level, such as:
 *  - clock scaling (chip level)
 *  - voltage scaling (board level)
 * 
 * This interface is to be used by low-level peripherals only and its
 * purpose is to be as fast as possible to do quick power and energy
 * optimizations (including deep power savings).
 * 
 * A separate common device driver interface would be required to allow
 * access from user programs (i.e. to select voltage and similar).
 **/ 

#ifdef CONFIG_RESOURCE

#ifndef __INCLUDE_NUTTX_RESOURCE_H
#define __INCLUDE_NUTTX_RESOURCE_H

#include <nuttx/config.h>
#include <nuttx/arch/chip/resource.h>
#include <nuttx/arch/board/resource.h>

#include <stdint.h>

/****************************************************************************
 * TO BE MOVED UNDER arch/chip/resource.h
 ****************************************************************************/

/** Clocking */
#define RESOURCE_CLOCK_HCLK
#define RESOURCE_CLOCK_PCLK1
#define RESOURCE_CLOCK_PCLK2


/****************************************************************************
 * TO BE MOVED UNDER arch/board/resource.h
 ****************************************************************************/

/** Voltage, Vcc Minimum Requirements */
#define RESOURCE_VOLTAGE_VCC_MIN2_3     0x00
#define RESOURCE_VOLTAGE_VCC_MIN3_0     0x01    /* A/D converter starts working at this level */
#define RESOURCE_VOLTAGE_VCC_MIN3_3     0x02


/****************************************************************************
 * Available Options to be enabled in the .config
 ****************************************************************************/

#if 0

/** Enable Resources 
 */
#define CONFIG_RESOURCE

/** Enable resource: clock scaling, allocates an array of options bits
 *  for each peripheral. On STM32 this option adds about 64-128 bytes.
 *  This option is provided by chip: nuttx/arch/board/resource.h
 * 
 * Clock resources are used by low-level driver before they starts a 
 * transaction. System may pick up any of the clock options that suit 
 * all of the drivers. The selected clock option is returned, and is up 
 * to the driver to recalculate and modify i.e. period or additional 
 * prescaler to achieve the desired frequency of operation. After lock() 
 * call, clock will not change, until unlock() is called.
 * 
 * As long there is a single clock lock, system will not be able to 
 * enter low power-down mode in which all of the clocks are disabled.
 * 
 * The isactive() function is called typically by the up_idle() to 
 * decide which power-down mode to select. If none of the clocks are
 * required to run, then ultra low power-down mode can be entered,
 * reducing power as low as downto 35 uA or 25 uA (with LDO in low-power
 * mode).
 * 
 * Note: AD may continue to consume power in stop mode, so it's up 
 *   to AD driver to disable its power when its unused.
 */
#define CONFIG_RESOURCE_CLOCK_SCALING

/** Enable resource: voltage scaling.
 * 
 *  This option is provided by the board: nuttx/arch/board/resource.h
 */
#define CONFIG_RESOURCE_VOLTAGE_SCALING

/** Resource Check: check for multiple locks/unlocks conditions and 
 *  report errors.
 * 
 *  For proper operation after lock(resourceN) only one unlock(resourceN)
 *  may follow. Enabling this option adds time consuming checks to detect
 *  errors in low-level drivers.
 */
#define CONFIG_RESOURCE_CHECK_MULTIPLES

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Interface
 ****************************************************************************/
 
/** Resource Interface */

struct resource_ops_s {

    /** Set possible options for specific domain and peripheral ID */
    uint32_t (*setopts)(struct resource_dev_t *dev, uint8_t domain_id);
    
    /** Get available options */
    uint32_t (*getopts)(struct resource_dev_t *dev, uint8_t domain);

    /** Lock resource, means, that it cannot be changed meanwhile */
    uint32_t (*lock)(struct resource_dev_t *dev, uint8_t domain_id);
    
    /** Unlock resource, means, allow further changes */
    uint32_t (*unlock)(struct resource_dev_t *dev, uint8_t domain_id);
    
    /** Reports present activity 
     * \retval >0 activity
     * \retval =0 Inactivity allowing system entering deep power down modes.
     * \retval <0 internal error, system cannot resolve requests.
     */
    int  (*isactive)(struct resource_dev_t *dev);
};

#define RESOURCE_SETOPTS(dev, domain_id)    dev->setopts(dev, domain_id)
#define RESOURCE_GETOPTS(dev, domain)       dev->getopts(dev, domain)
#define RESOURCE_LOCK(dev, domain_id)       dev->lock(dev, domain_id)
#define RESOURCE_UNLOCK(dev, domain_id)     dev->unlock(dev, domain_id)
#define RESOURCE_ISACTIVE(dev)              dev->isactive(dev)


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_EVENT_H */
#endif /* CONFIG_RESOURCE */
