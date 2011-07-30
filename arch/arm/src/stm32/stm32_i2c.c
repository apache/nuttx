/************************************************************************************
 * arch/arm/src/stm32/stm32_i2c.c
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
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained by
 *      the i2c_init(); it extends the Device structure from the nuttx/i2c/i2c.h; 
 *      Instance points to OPS, to common I2C Hardware private data and contains
 *      its own private data, as frequency, address, mode of operation (in the future)
 *  - Private: Private data of an I2C Hardware
 * 
 * \todo
 *  - Check for all possible deadlocks (as BUSY='1' I2C needs to be reset in HW using the I2C_CR1_SWRST)
 *  - SMBus support (hardware layer timings are already supported) and add SMBA gpio pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or 
 *      - 1 x 10 bit adresses + 1 x 7 bit address (?)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
 *  - DMA (to get rid of too many CPU wake-ups and interventions)
 *  - Be ready for IPMI
 **/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include "up_arch.h"

#include "stm32_rcc.h"
#include "stm32_i2c.h"
#include "stm32_waste.h"

#if defined(CONFIG_STM32_I2C1) || defined(CONFIG_STM32_I2C2)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/** I2C Device Private Data
 */
struct stm32_i2c_priv_s {
    uint32_t    base;
    int         refs;
    sem_t       sem_excl;
    sem_t       sem_isr;
    
    uint8_t     msgc;
    struct i2c_msg_s *msgv;
    uint8_t   * ptr;
    int         dcnt;
    uint16_t    flags;
    
    uint32_t    status;
};

/** I2C Device, Instance
 */
struct stm32_i2c_inst_s {
    struct i2c_ops_s        * ops;
    struct stm32_i2c_priv_s * priv;
    
    uint32_t    frequency;
    int         address;
    uint16_t    flags;
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

#if CONFIG_STM32_I2C1
struct stm32_i2c_priv_s stm32_i2c1_priv = {
    .base       = STM32_I2C1_BASE,
    .refs       = 0
};
#endif

#if CONFIG_STM32_I2C2
struct stm32_i2c_priv_s stm32_i2c2_priv = {
    .base       = STM32_I2C2_BASE,
    .refs       = 0
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/** Get register value by offset */
static inline uint16_t stm32_i2c_getreg(FAR struct stm32_i2c_priv_s *priv, uint8_t offset)
{
    return getreg16(priv->base + offset);
}

/** Put register value by offset */
static inline void stm32_i2c_putreg(FAR struct stm32_i2c_priv_s *priv, uint8_t offset, uint16_t value)
{
    putreg16(value, priv->base + offset);
}

/** Modify register value by offset */
static inline void stm32_i2c_modifyreg(FAR struct stm32_i2c_priv_s *priv, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
    modifyreg16(priv->base + offset, clearbits, setbits);
}

void inline stm32_i2c_sem_wait(FAR struct i2c_dev_s *dev)
{
    while( sem_wait( &((struct stm32_i2c_inst_s *)dev)->priv->sem_excl ) != 0 ) {
        ASSERT(errno == EINTR);
    }
}

int inline stm32_i2c_sem_waitisr(FAR struct i2c_dev_s *dev)
{
    while( sem_wait( &((struct stm32_i2c_inst_s *)dev)->priv->sem_isr ) != 0 ) {
        ASSERT(errno == EINTR);
    }
    return OK;
}

void inline stm32_i2c_sem_post(FAR struct i2c_dev_s *dev)
{
    sem_post( &((struct stm32_i2c_inst_s *)dev)->priv->sem_excl );
}

void inline stm32_i2c_sem_init(FAR struct i2c_dev_s *dev)
{
    sem_init( &((struct stm32_i2c_inst_s *)dev)->priv->sem_excl, 0, 1);
    sem_init( &((struct stm32_i2c_inst_s *)dev)->priv->sem_isr, 0, 0);
}

void inline stm32_i2c_sem_destroy(FAR struct i2c_dev_s *dev)
{
    sem_destroy( &((struct stm32_i2c_inst_s *)dev)->priv->sem_excl );
    sem_destroy( &((struct stm32_i2c_inst_s *)dev)->priv->sem_isr );
}

static void stm32_i2c_setclock(FAR struct stm32_i2c_priv_s *priv, uint32_t frequency)
{
    /* Disable Peripheral if rising time is to be changed, 
     * and restore state on return. */

    uint16_t cr1 = stm32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET);
    
    if (cr1 & I2C_CR1_PE)
        stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1 ^ I2C_CR1_PE);
        
    /* Update timing and control registers */
    
    if (frequency < 400e3) {
        
        /* Speed: 100 kHz 
         * Risetime: 1000 ns
         * Duty: t_low / t_high = 1
         */
        stm32_i2c_putreg(priv, STM32_I2C_CCR_OFFSET, STM32_BOARD_HCLK/200000);
        stm32_i2c_putreg(priv, STM32_I2C_TRISE_OFFSET, STM32_BOARD_HCLK/1000000 + 1);
    }
    else {
    
        /* Speed: 400 kHz 
         * Duty: t_low / t_high = 2
         */
        stm32_i2c_putreg(priv, STM32_I2C_CCR_OFFSET, STM32_BOARD_HCLK/1200000);
        stm32_i2c_putreg(priv, STM32_I2C_TRISE_OFFSET, 300*(STM32_BOARD_HCLK / 1000000)/1000 + 1); 
    }
        
    /* Restore state */
    
    if (cr1 & I2C_CR1_PE)
        stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1);
}

static inline void stm32_i2c_sendstart(FAR struct stm32_i2c_priv_s *priv)
{
    /* Disable ACK on receive by default and generate START */
    stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, I2C_CR1_START);
}

static inline void stm32_i2c_sendstop(FAR struct stm32_i2c_priv_s *priv)
{
    stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, I2C_CR1_STOP);
}

static inline uint32_t stm32_i2c_getstatus(FAR struct stm32_i2c_priv_s *priv)
{
   uint32_t status = stm32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);
   status |= (stm32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);
   return status;
}

/************************************************************************************
 * Interrupt Service Routines
 ************************************************************************************/

//DEBUG TO BE CLEANED
//#define NON_ISR

static int stm32_i2c_isr(struct stm32_i2c_priv_s * priv)
{
    uint32_t status = stm32_i2c_getstatus(priv);
    
#ifdef NON_ISR
    static uint32_t isr_count = 0;
    static uint32_t old_status = 0xFFFF;
    isr_count++;

    if (old_status != status) {
        printf("status = %8x, count=%d\n", status, isr_count); fflush(stdout);
        old_status = status;
    }
#endif
            
    /* Was start bit sent */
    
    if (status & I2C_SR1_SB) {
    
        /* Get run-time data */
                
        priv->ptr   = priv->msgv->buffer;
        priv->dcnt  = priv->msgv->length;
        priv->flags = priv->msgv->flags;

        /* Send address byte and define addressing mode */
        
        stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, (priv->flags & I2C_M_TEN) ?
            0 :
            ((priv->msgv->addr << 1) | (priv->flags & I2C_M_READ))
        );
                
        /* Set ACK for receive mode */
        
        if (priv->dcnt > 1 && (priv->flags & I2C_M_READ) ) {            
            stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, 0, I2C_CR1_ACK);
        }
        
        /* Increment to next pointer and decrement message count */
        
        priv->msgv++;
        priv->msgc--;
    }
    
        /* In 10-bit addressing mode, was first byte sent */
    
    else if (status & I2C_SR1_ADD10) {
        /* \todo Finish 10-bit mode addressing */
        
    }
    
        /* Was address sent, continue with ether sending or reading data */
    
    else if ( !(priv->flags & I2C_M_READ) && 
               (status & (I2C_SR1_ADDR | I2C_SR1_TXE)) ) {
               
        if (--priv->dcnt >= 0) {   /* Send a byte */
#ifdef NON_ISR
            printf("Send byte: %2x\n", *priv->ptr);
#endif
            stm32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, *priv->ptr++); 
        }
    }
    
    else if ( (priv->flags & I2C_M_READ) && (status & I2C_SR1_ADDR) ) {

        /* Enable RxNE and TxE buffers in order to receive one or multiple bytes */
        
        stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
    }

        /* More bytes to read */

    else if ( status & I2C_SR1_RXNE ) {
    
        /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */
    
#ifdef NON_ISR
        printf("dcnt=%d\n", priv->dcnt);
#endif
        if (--priv->dcnt >= 0) {
            *priv->ptr++ = stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);
#ifdef NON_ISR
            printf("Received: %2x\n", *(priv->ptr-1) );
#endif
            /* Disable acknowledge when last byte is to be received */
            if (priv->dcnt == 1) {
                stm32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, 0);  
            }
        }
            
    }
    
    /* Do we have more bytes to send, enable/disable buffer interrupts
     * (these ISRs could be replaced by DMAs)
     */
     
    if (priv->dcnt>0) {
        stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
    }
    else if (priv->dcnt==0 ) {
        stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);  
    }

    
    /* Was last byte received or sent? 
     */
    
    if (priv->dcnt<=0 && (status & I2C_SR1_BTF)) {
#ifdef NON_ISR
        printf("BTF\n");
#endif
        stm32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);    /* ACK ISR */

        /* Do we need to terminate or restart after this byte */
        
        /* If there are more messages to send, then we may:
         *  - continue with repeated start
         *  - or just continue sending writeable part
         *  - or we close down by sending the stop bit 
         */             
        if (priv->msgc) {

            if (priv->msgv->flags & I2C_M_NORESTART) {
                priv->ptr   = priv->msgv->buffer;
                priv->dcnt  = priv->msgv->length;
                priv->flags = priv->msgv->flags;
                priv->msgv++;
                priv->msgc--;
                stm32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);    // Restart this ISR!
            }
            else {
                stm32_i2c_sendstart(priv);
            }
        }
        else if (priv->msgv) {
#ifdef NON_ISR
            printf("stop2: status = %8x\n", status);
#endif
            stm32_i2c_sendstop(priv);
            sem_post( &priv->sem_isr );
            priv->msgv = NULL;              /* mark that we have stopped with this transaction */
        }
    }
    
    /* Check for errors, in which case, stop the transfer and return 
     * Note that in master reception mode AF becomes set on last byte
     * since ACK is not returned. We should ignore this error.
     */
    
    if (status & I2C_SR1_ERRORMASK) {
        stm32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);    /* clear flags */
        sem_post( &priv->sem_isr );
    }

    priv->status = status;
    return OK;
}

/* Decode ***************************************************************************/

#if CONFIG_STM32_I2C1
static int stm32_i2c1_isr(int irq, void *context)
{
    return stm32_i2c_isr(&stm32_i2c1_priv);
}
#endif

#if CONFIG_STM32_I2C2
static int stm32_i2c2_isr(int irq, void *context)
{
    return stm32_i2c_isr(&stm32_i2c2_priv);
}
#endif

/************************************************************************************
 * Private Initialization and Deinitialization
 ************************************************************************************/

/** Setup the I2C hardware, ready for operation with defaults */
static int stm32_i2c_init(FAR struct stm32_i2c_priv_s *priv)
{
    /* Power-up and configure GPIOs */

    switch( priv->base ) {
    
#if CONFIG_STM32_I2C1
        case STM32_I2C1_BASE:
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_I2C1EN);
            
            if (stm32_configgpio(GPIO_I2C1_SCL)==ERROR) return ERROR;
            if (stm32_configgpio(GPIO_I2C1_SDA)==ERROR) {
                stm32_unconfiggpio(GPIO_I2C1_SCL);
                return ERROR;
            }
            irq_attach(STM32_IRQ_I2C1EV, stm32_i2c1_isr);
            irq_attach(STM32_IRQ_I2C1ER, stm32_i2c1_isr);
            up_enable_irq(STM32_IRQ_I2C1EV);
            up_enable_irq(STM32_IRQ_I2C1ER);
            break;
#endif
      
#if CONFIG_STM32_I2C2
        case STM32_I2C2_BASE:
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_I2C2EN);
            
            if (stm32_configgpio(GPIO_I2C2_SCL)==ERROR) return ERROR;
            if (stm32_configgpio(GPIO_I2C2_SDA)==ERROR) {
                stm32_unconfiggpio(GPIO_I2C2_SCL);
                return ERROR;
            }
            irq_attach(STM32_IRQ_I2C2EV, stm32_i2c2_isr);
            irq_attach(STM32_IRQ_I2C2ER, stm32_i2c2_isr);
            up_enable_irq(STM32_IRQ_I2C2EV);
            up_enable_irq(STM32_IRQ_I2C2ER);
            break;
#endif
        
        default: return ERROR;
    }

    /* Set peripheral frequency, where it must be at least 2 MHz 
     * for 100 kHz or 4 MHz for 400 kHz. Enable interrupt generation.
     */

    stm32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET,
#ifndef NON_ISR
        I2C_CR2_ITERREN | I2C_CR2_ITEVFEN | // I2C_CR2_ITBUFEN |
#endif
        (STM32_BOARD_HCLK / 1000000) 
    );
    
    stm32_i2c_setclock(priv, 100000);
        
    /* Enable I2C */
    
    stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_PE);
    
    return OK;
}

/** Shutdown the I2C hardware  */
static int stm32_i2c_deinit(FAR struct stm32_i2c_priv_s *priv)
{
    /* Disable I2C */

    stm32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, 0);

    switch( priv->base ) {
    
#if CONFIG_STM32_I2C1
        case STM32_I2C1_BASE:
            stm32_unconfiggpio(GPIO_I2C1_SCL);
            stm32_unconfiggpio(GPIO_I2C1_SDA);
                
            up_disable_irq(STM32_IRQ_I2C1EV);
            up_disable_irq(STM32_IRQ_I2C1ER);
            irq_detach(STM32_IRQ_I2C1EV);
            irq_detach(STM32_IRQ_I2C1ER);
            
            modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_I2C1EN, 0);
            break;
#endif
      
#if CONFIG_STM32_I2C2
        case STM32_I2C2_BASE:
            stm32_unconfiggpio(GPIO_I2C2_SCL);
            stm32_unconfiggpio(GPIO_I2C2_SDA);
            
            up_disable_irq(STM32_IRQ_I2C1EV);
            up_disable_irq(STM32_IRQ_I2C1ER);
            irq_detach(STM32_IRQ_I2C1EV);
            irq_detach(STM32_IRQ_I2C1ER);

            modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_I2C2EN, 0);
            break;
#endif
        
        default: return ERROR;
    }

    return OK;
}

/************************************************************************************
 * Device Driver OPS - Blocking Type
 ************************************************************************************/

uint32_t stm32_i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
    stm32_i2c_sem_wait(dev);
    
#if STM32_BOARD_HCLK < 4000000
    ((struct stm32_i2c_inst_s *)dev)->frequency = 100000;
#else
    ((struct stm32_i2c_inst_s *)dev)->frequency = frequency;
#endif

    stm32_i2c_sem_post(dev);    
    return ((struct stm32_i2c_inst_s *)dev)->frequency;
}

int stm32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
    stm32_i2c_sem_wait(dev);
    
    ((struct stm32_i2c_inst_s *)dev)->address = addr;
    ((struct stm32_i2c_inst_s *)dev)->flags   = (nbits == 10) ? I2C_M_TEN : 0;
    
    stm32_i2c_sem_post(dev);
    return OK;
}

int stm32_i2c_process(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs, int count)
{
    struct stm32_i2c_inst_s *inst = (struct stm32_i2c_inst_s *)dev;
    uint32_t    status = 0;
    int         status_errno = 0;
    
    ASSERT(count);
                  
    /* wait as stop might still be in progress 
     * 
     * \todo GET RID OF THIS PERFORMANCE LOSS and for() loop
     */
    for (; stm32_i2c_getreg(inst->priv, STM32_I2C_CR1_OFFSET) & I2C_CR1_STOP; ) up_waste();
    
    /* Old transfers are done */
    inst->priv->msgv    = msgs;
    inst->priv->msgc    = count;
        
    /* Set clock (on change it toggles I2C_CR1_PE !) */
    stm32_i2c_setclock(inst->priv, inst->frequency);

    /* Trigger start condition, then the process moves into the ISR */
    stm32_i2c_sendstart(inst->priv);
    
        
#ifdef NON_ISR   
    do {
        do {
            stm32_i2c_isr(&stm32_i2c1_priv);
            status = inst->priv->status;
        } while( status & (I2C_SR2_BUSY<<16) );
    } 
    while( sem_trywait( &((struct stm32_i2c_inst_s *)dev)->priv->sem_isr ) != 0 );
#else
#if 1
    /* Wait for an ISR, if there was a timeout, fetch latest status to get the BUSY flag */

    if (stm32_i2c_sem_waitisr(dev) == ERROR) {
        status = stm32_i2c_getstatus(inst->priv);
    }
    else status = inst->priv->status & 0xFFFF;  /* clear SR2 (BUSY flag) as we've done successfully */
#else
    do {
        printf("%x, %d\n", inst->priv->status, isr_count );
    } 
    while( sem_trywait( &inst->priv->sem_isr ) != 0 );
#endif
#endif

    if (status & I2C_SR1_BERR) {        /* Bus Error */
        status_errno = EIO;
    }
    else if (status & I2C_SR1_ARLO) {   /* Arbitration Lost (master mode) */
        status_errno = EAGAIN;
    }
    else if (status & I2C_SR1_AF) {     /* Acknowledge Failure */
        status_errno = ENXIO;
    }
    else if (status & I2C_SR1_OVR) {    /* Overrun/Underrun */
        status_errno = EIO;
    }
    else if (status & I2C_SR1_PECERR) { /* PEC Error in reception */
        status_errno = EPROTO;
    }
    else if (status & I2C_SR1_TIMEOUT) {/* Timeout or Tlow Error */
        status_errno = ETIME;
    }
    else if (status & (I2C_SR2_BUSY<<16) ) {    /* I2C Bus is for some reason busy */
        status_errno = EBUSY;
    }
    
//  printf("end_count = %d, dcnt=%d\n", isr_count, inst->priv->dcnt); fflush(stdout);
    
    stm32_i2c_sem_post(dev);
    
    errno = status_errno;
    return -status_errno;
}

int stm32_i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen)
{
    stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */
    
    struct i2c_msg_s msgv = {
        .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
        .flags  = ((struct stm32_i2c_inst_s *)dev)->flags,
        .buffer = (uint8_t *)buffer,
        .length = buflen
    };
    
    return stm32_i2c_process(dev, &msgv, 1);
}

int stm32_i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen)
{
    stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */
    
    struct i2c_msg_s msgv = {
        .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
        .flags  = ((struct stm32_i2c_inst_s *)dev)->flags | I2C_M_READ,
        .buffer = buffer,
        .length = buflen
    };
    
    return stm32_i2c_process(dev, &msgv, 1);
}

#ifdef CONFIG_I2C_WRITEREAD
int stm32_i2c_writeread(FAR struct i2c_dev_s *dev, const uint8_t *wbuffer, int wbuflen,
                        uint8_t *buffer, int buflen)
{
    stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */

    struct i2c_msg_s msgv[2] = {
      {
        .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
        .flags  = ((struct stm32_i2c_inst_s *)dev)->flags,
        .buffer = (uint8_t *)wbuffer,          /* this is really ugly, sorry const ... */
        .length = wbuflen
      },
      {
        .addr   = ((struct stm32_i2c_inst_s *)dev)->address,
        .flags  = ((struct stm32_i2c_inst_s *)dev)->flags | ((buflen>0) ? I2C_M_READ : I2C_M_NORESTART),
        .buffer = buffer,
        .length = (buflen>0) ? buflen : -buflen
      }
    };

    return stm32_i2c_process(dev, msgv, 2);
}
#endif

#ifdef CONFIG_I2C_TRANSFER
int stm32_i2c_transfer(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs, int count)
{
    stm32_i2c_sem_wait(dev);   /* ensure that address or flags don't change meanwhile */
    return stm32_i2c_process(dev, msgs, count);
}
#endif

/************************************************************************************
 * Device Structures, Instantiation
 ************************************************************************************/

struct i2c_ops_s stm32_i2c_ops = {
    .setfrequency       = stm32_i2c_setfrequency,
    .setaddress         = stm32_i2c_setaddress,
    .write              = stm32_i2c_write,
    .read               = stm32_i2c_read
#ifdef CONFIG_I2C_WRITEREAD
  , .writeread          = stm32_i2c_writeread
#endif
#ifdef CONFIG_I2C_TRANSFER
  , .transfer           = stm32_i2c_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
  , .setownaddress      = stm32_i2c_setownaddress,
    .registercallback   = stm32_i2c_registercallback
#endif
};

/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

FAR struct i2c_dev_s * up_i2cinitialize(int port)
{
    struct stm32_i2c_priv_s * priv = NULL;  /* private data of device with multiple instances */
    struct stm32_i2c_inst_s * inst = NULL;  /* device, single instance */
    int irqs;
 
#if STM32_BOARD_HCLK < 4000000
#   warning STM32_I2C_INIT: Peripheral clock must be at least 4 MHz to support 400 kHz operation.
#endif

#if STM32_BOARD_HCLK < 2000000
#   warning STM32_I2C_INIT: Peripheral clock must be at least 2 MHz to support 100 kHz operation.
    return NULL;
#endif
    
    /* Get structure and enable power */
    
    switch(port) {
#if CONFIG_STM32_I2C1
        case 1: priv = (struct stm32_i2c_priv_s *)&stm32_i2c1_priv; break;
#endif
#if CONFIG_STM32_I2C2
        case 2: priv = (struct stm32_i2c_priv_s *)&stm32_i2c2_priv; break;
#endif
        default: return NULL;
    }
    
    /* Allocate instance */
    
    if ( !(inst = kmalloc( sizeof(struct stm32_i2c_inst_s) )) ) return NULL;
        
    /* initialize instance */
        
    inst->ops       = &stm32_i2c_ops;
    inst->priv      = priv;
    inst->frequency = 100e3;
    inst->address   = 0;
    inst->flags     = 0;

    /* Init private data for the first time, increment refs count,
     * power-up hardware and configure GPIOs. 
     */
    
    irqs = irqsave();
    
    if ((volatile int)priv->refs++ == 0) {
        stm32_i2c_sem_init( (struct i2c_dev_s *)inst );
        stm32_i2c_init( priv );
    }
    
    irqrestore(irqs);

    return (struct i2c_dev_s *)inst;
}


int up_i2cuninitialize(FAR struct i2c_dev_s * dev)
{
    int irqs;
    
    ASSERT(dev);
    
    /* Decrement refs and check for underflow */
    
    if ( ((struct stm32_i2c_inst_s *)dev)->priv->refs == 0 ) 
        return ERROR;
        
    irqs = irqsave();
        
    if ( --((struct stm32_i2c_inst_s *)dev)->priv->refs ) {
        irqrestore(irqs);
        kfree(dev);
        return OK;
    }
    
    irqrestore(irqs);
    
    /* Disable power and other HW resource (GPIO's) */
    
    stm32_i2c_deinit( ((struct stm32_i2c_inst_s *)dev)->priv );
    
    /* Release unused resources */

    stm32_i2c_sem_destroy( (struct i2c_dev_s *)dev );
        
    kfree(dev);
    return OK;
}

#endif /* defined(CONFIG_STM32_I2C1) && defined(CONFIG_STM32_I2C2) */
