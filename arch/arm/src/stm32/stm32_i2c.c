/************************************************************************************
 * arch/arm/src/stm32/stm32_i2c.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/** \file
 *  \author Uros Platise
 *  \brief STM32 I2C Hardware Layer - Device Driver
 * 
 * Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 * 
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained by
 *      the i2c_init(); it extends the Device structure from the nuttx/i2c.h; 
 *      Instance points to OPS, to common I2C Hardware private data and contains
 *      its own private data, as frequency, address, mode of operation (in the future)
 *  - Private: Private data of an I2C Hardware
 * 
 * \todo
 *  - SMBus support (hardware layer timings are already supported) and add SMBA gpio pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or 
 *      - 1 x 10 bit adresses + 1 x 7 bit address (?)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
 *  - DMA (to get rid of too many CPU wake-ups and interventions)
 *  - Be ready for IPMI
 **/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/i2c.h>
#include <arch/board/board.h>

#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include "up_arch.h"

#include "stm32_rcc.h"
#include "stm32_i2c.h"


/************************************************************************************
 * Private Types
 ************************************************************************************/


/** I2C Device Private Data
 */
struct stm32_i2c_priv_s {
    uint32_t    base;
    int         refs;
    sem_t       sem;
};


/** I2C Device, Instance
 */
struct stm32_i2c_inst_s {
    struct i2c_ops_s        * ops;
    struct stm32_i2c_priv_s * priv;
    
    uint32_t    frequency;
    int         address;
};



/************************************************************************************
 * Private Functions
 ************************************************************************************/

/** Get register value by offset */
static inline uint16_t stm32_i2c_getreg(FAR struct i2c_dev_s *dev, uint8_t offset)
{
    return getreg16( ((struct stm32_i2c_inst_s *)dev)->priv->base + offset);
}


/** Put register value by offset */
static inline void stm32_i2c_putreg(FAR struct i2c_dev_s *dev, uint8_t offset, uint16_t value)
{
    //printf("putreg(%8x)=%4x\n", ((struct stm32_i2c_priv_s *)dev)->base + offset, value );
    putreg16(value, ((struct stm32_i2c_inst_s *)dev)->priv->base + offset);
}


/** Modify register value by offset */
static inline void stm32_i2c_modifyreg(FAR struct i2c_dev_s *dev, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
    modifyreg16( ((struct stm32_i2c_inst_s *)dev)->priv->base + offset, clearbits, setbits);
}


void inline stm32_i2c_sem_wait(FAR struct i2c_dev_s *dev)
{
    while( sem_wait( &((struct stm32_i2c_inst_s *)dev)->priv->sem ) != 0 ) {
        ASSERT(errno == EINTR);
    }
}


void inline stm32_i2c_sem_post(FAR struct i2c_dev_s *dev)
{
    sem_post( &((struct stm32_i2c_inst_s *)dev)->priv->sem );
}


void inline stm32_i2c_sem_init(FAR struct i2c_dev_s *dev)
{
    sem_init( &((struct stm32_i2c_inst_s *)dev)->priv->sem, 0, 1);
}


void inline stm32_i2c_sem_destroy(FAR struct i2c_dev_s *dev)
{
    sem_destroy( &((struct stm32_i2c_inst_s *)dev)->priv->sem );
}


static void stm32_i2c_setclock(FAR struct i2c_dev_s *inst, bool fast)
{
    /* Disable Peripheral if rising time is to be changed, 
     * and restore state on return. */

    uint16_t cr1 = stm32_i2c_getreg(inst, STM32_I2C_CR1_OFFSET);
    
    if (cr1 & I2C_CR1_PE)
        stm32_i2c_putreg(inst, STM32_I2C_CR1_OFFSET, cr1 ^ I2C_CR1_PE);
        
    /* Update timing and control registers */
    
    if (!fast) {
        
        /* Speed: 100 kHz 
         * Risetime: 1000 ns
         * Duty: t_low / t_high = 1
         */
        stm32_i2c_putreg(inst, STM32_I2C_CCR_OFFSET, STM32_BOARD_HCLK / 200000);
        stm32_i2c_putreg(inst, STM32_I2C_TRISE_OFFSET, 1 + STM32_BOARD_HCLK/1000000);
    }
    else {
    
        /* Speed: 400 kHz 
         * Risetime: 1000 ns ??? \todo check rise time for 400 kHz devices
         * Duty: t_low / t_high = 2
         */
        stm32_i2c_putreg(inst, STM32_I2C_CCR_OFFSET, STM32_BOARD_HCLK / 1200000);
        stm32_i2c_putreg(inst, STM32_I2C_TRISE_OFFSET, 1 + STM32_BOARD_HCLK/1000000); 
    }
        
    /* Restore state */
    
    if (cr1 & I2C_CR1_PE)
        stm32_i2c_putreg(inst, STM32_I2C_CR1_OFFSET, cr1);
}


/** Setup the I2C hardware, ready for operation with defaults */
static int stm32_i2c_init(FAR struct i2c_dev_s *inst)
{
    /* Power-up and configure GPIOs */

    switch( ((struct stm32_i2c_inst_s *)inst)->priv->base ) {
        case STM32_I2C1_BASE:
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_I2C1EN);
            stm32_configgpio(GPIO_I2C1_SCL);
            stm32_configgpio(GPIO_I2C1_SDA);
            break;
            
        case STM32_I2C2_BASE:
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_I2C2EN);
            stm32_configgpio(GPIO_I2C2_SCL);
            stm32_configgpio(GPIO_I2C2_SDA);
            break;
        
        default: return ERROR;
    }

    /* Set peripheral frequency, where it must be at least 2 MHz 
     * for 100 kHz or 4 MHz for 400 kHz. Enable interrupt generation.
     */
     
#if STM32_BOARD_HCLK < 4000000
#   error STM32_I2C_INIT: Peripheral clock must be at least 4 MHz to support 100/400 kHz operation.
#endif
    
    stm32_i2c_putreg(inst, STM32_I2C_CR2_OFFSET,
        I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN |
        (STM32_BOARD_HCLK / 1000000) 
    );
    
    stm32_i2c_setclock(inst, false);
    
    /* Enable I2C */
    
    stm32_i2c_putreg(inst, STM32_I2C_CR1_OFFSET, I2C_CR1_PE);
    
    return OK;
}


/** Shutdown the I2C hardware  */
static int stm32_i2c_deinit(FAR struct i2c_dev_s *inst)
{
    /* Disable I2C */

    stm32_i2c_putreg(inst, STM32_I2C_CR1_OFFSET, 0);

    switch( ((struct stm32_i2c_inst_s *)inst)->priv->base ) {
        case STM32_I2C1_BASE:
            stm32_unconfiggpio(GPIO_I2C1_SCL);
            stm32_unconfiggpio(GPIO_I2C1_SDA);
            modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_I2C1EN, 0);
            break;
            
        case STM32_I2C2_BASE:
            stm32_unconfiggpio(GPIO_I2C2_SCL);
            stm32_unconfiggpio(GPIO_I2C2_SDA);
            modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_I2C2EN, 0);
            break;
        
        default: return ERROR;
    }

    return OK;
}


/************************************************************************************
 * Interrupt Service Routines
 ************************************************************************************/

static int stm32_i2c1_isr(int irq, void *context)
{
    // ACK before return!
    return OK;
}


static int stm32_i2c2_isr(int irq, void *context)
{
    // ACK before return!
    return OK;
}


/************************************************************************************
 * Device Driver OPS - Blocking Type
 ************************************************************************************/

uint32_t stm32_i2c_setfrequency(FAR struct i2c_dev_s *inst, uint32_t frequency)
{
    stm32_i2c_sem_wait(inst);
    ((struct stm32_i2c_inst_s *)inst)->frequency = frequency;
    
    
    stm32_i2c_sem_post(inst);    
    return ((struct stm32_i2c_inst_s *)inst)->frequency;
}


int stm32_i2c_setaddress(FAR struct i2c_dev_s *inst, int addr, int nbits)
{
    stm32_i2c_sem_wait(inst);
    
    ((struct stm32_i2c_inst_s *)inst)->address = addr;
    
    
    stm32_i2c_sem_post(inst);
    return OK;
}

  
int stm32_i2c_write(FAR struct i2c_dev_s *inst, const uint8_t *buffer, int buflen)
{
    stm32_i2c_sem_wait(inst);
    stm32_i2c_sem_post(inst);
    
    return OK;
}


int stm32_i2c_read(FAR struct i2c_dev_s *inst, uint8_t *buffer, int buflen)
{
    stm32_i2c_sem_wait(inst);
    stm32_i2c_sem_post(inst);
    
    return OK;
}


#ifdef CONFIG_I2C_TRANSFER
int stm32_i2c_transfer(FAR struct i2c_dev_s *inst, FAR struct i2c_msg_s *msgs, int count)
{
    stm32_i2c_sem_wait(inst);
    stm32_i2c_sem_post(inst);
    
    return OK;
}
#endif


/************************************************************************************
 * Device Structures, Instantiation
 ************************************************************************************/

struct i2c_ops_s stm32_i2c_ops = {
    .setfrequency   = stm32_i2c_setfrequency,
    .setaddress     = stm32_i2c_setaddress,
    .write          = stm32_i2c_write,
    .read           = stm32_i2c_read
#ifdef CONFIG_I2C_TRANSFER
  , .transfer       = stm32_i2c_transfer
#endif
};


struct stm32_i2c_priv_s stm32_i2c1_priv = {
    .base       = STM32_I2C1_BASE,
    .refs       = 0
};


struct stm32_i2c_priv_s stm32_i2c2_priv = {
    .base       = STM32_I2C2_BASE,
    .refs       = 0
};



/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

FAR struct i2c_dev_s * up_i2cinitialize(int port)
{
    struct stm32_i2c_priv_s * priv = NULL;  /* private data of device with multiple instances */
    struct stm32_i2c_inst_s * inst = NULL;  /* device, single instance */
    int irqs;
    
    /* Get structure and enable power */
    
    switch(port) {
        case 1: priv = (struct stm32_i2c_priv_s *)&stm32_i2c1_priv; break;
        case 2: priv = (struct stm32_i2c_priv_s *)&stm32_i2c2_priv; break;
        default: return NULL;
    }
    
    /* Allocate instance */
    
    if ( !(inst = malloc( sizeof(struct stm32_i2c_inst_s) )) ) return NULL;
        
    /* initialize instance */
        
    inst->ops       = &stm32_i2c_ops;
    inst->priv      = priv;
    inst->frequency = 100e3;
    inst->address   = 0;

    /* Init private data for the first time, increment refs count,
     * power-up hardware and configure GPIOs. 
     */
    
    irqs = irqsave();
    
    if ((volatile int)priv->refs++ == 0) {
        stm32_i2c_sem_init( (struct i2c_dev_s *)inst );
        stm32_i2c_init( (struct i2c_dev_s *)inst );
    }
    
    irqrestore(irqs);

    return (struct i2c_dev_s *)inst;
}


int up_i2cuninitialize(FAR struct i2c_dev_s * inst)
{
    int irqs;
    
    ASSERT(inst);
    
    /* Decrement refs and check for underflow */
    
    if ( ((struct stm32_i2c_inst_s *)inst)->priv->refs == 0 ) 
        return ERROR;
        
    irqs = irqsave();
        
    if ( --((struct stm32_i2c_inst_s *)inst)->priv->refs ) {
        irqrestore(irqs);
        free(inst);
        return OK;
    }
    
    irqrestore(irqs);
    
    /* Disable power and other HW resource (GPIO's) */
    
    stm32_i2c_deinit( (struct i2c_dev_s *)inst );
    
    /* Release unused resources */

    stm32_i2c_sem_destroy( (struct i2c_dev_s *)inst );
        
    free(inst);
    return OK;
}

