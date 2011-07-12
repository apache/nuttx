/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_can.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *
 * This file is a part of NuttX:
 * 
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/can.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "lpc17_internal.h"
#include "lpc17_syscon.h"
#include "lpc17_pinconn.h"
#include "lpc17_can.h"

#if HAVE_CAN

#define CAN2_RATE  500000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN methods */

static void can_reset(FAR struct can_dev_s *dev);
static int  can_setup(FAR struct can_dev_s *dev);
static void can_shutdown(FAR struct can_dev_s *dev);
static void can_rxint(FAR struct can_dev_s *dev, bool enable);
static void can_txint(FAR struct can_dev_s *dev, bool enable);
static int  can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int  can_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool can_txempty(FAR struct can_dev_s *dev);
static int  can_interrupt(int irq, void *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canops =
{
	.co_reset =can_reset,
	.co_setup = can_setup,
	.co_shutdown = can_shutdown,
	.co_rxint = can_rxint,
	.co_txint = can_txint,
	.co_ioctl = can_ioctl,
	.co_remoterequest = can_remoterequest,
	.co_send = can_send,
	.co_txempty = can_txempty,
};

static struct can_dev_s g_candev =
{
	.cd_ops = &g_canops,
}; 

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/* Reset the CAN device.  Called early to initialize the hardware. This
* is called, before co_setup() and on error conditions.
*/
static void can_reset(FAR struct can_dev_s *dev)
{
	irqstate_t flags;
	uint32_t regval;

	flags = irqsave();
	putreg32(0x01,LPC17_CAN2_MOD);
	putreg32(0x00,LPC17_CAN2_IER);
	putreg32(0x00,LPC17_CAN2_GSR);
	putreg32(0x02,LPC17_CAN2_CMR);
	putreg32(0x25c003,LPC17_CAN2_BTR);
	putreg32(0x00,LPC17_CAN2_MOD);
	putreg32(0x01,LPC17_CAN2_IER);
	putreg32(0x02,LPC17_CANAF_AFMR);

	irqrestore(flags);

	regval  = getreg32(LPC17_CAN2_BTR);
	//dbg("BTR=%x\n",regval);
}

/* Configure the CAN. This method is called the first time that the CAN
* device is opened.  This will occur when the port is first opened. 
* This setup includes configuring and attaching CAN interrupts.  Interrupts
* are all disabled upon return.
*/
static int  can_setup(FAR struct can_dev_s *dev)
{
	int ret=0;
	ret = irq_attach(LPC17_IRQ_CAN, can_interrupt);
	if (ret == OK)
	{
	   up_enable_irq(LPC17_IRQ_CAN);
	}
	return ret;
}

/* Disable the CAN.  This method is called when the CAN device is closed.
* This method reverses the operation the setup method.
*/
static void can_shutdown(FAR struct can_dev_s *dev)
{
  up_disable_irq(LPC17_IRQ_CAN);
  irq_detach(LPC17_IRQ_CAN);
}

/* Call to enable or disable RX interrupts */
static void can_rxint(FAR struct can_dev_s *dev, bool enable)
{
    uint32_t regval;
	regval  = getreg32(LPC17_CAN2_IER);
	if(enable)
		regval |= CAN_IER_RIE;
	else
		regval &= ~CAN_IER_RIE;
	putreg32(regval, LPC17_CAN2_IER);
}

/* Call to enable or disable TX interrupts */
static void can_txint(FAR struct can_dev_s *dev, bool enable)
{
    uint32_t regval;
	regval  = getreg32(LPC17_CAN2_IER);
	if(enable)
		regval |= CAN_IER_TIE1;
	else
		regval &= ~CAN_IER_TIE1;
	putreg32(regval, LPC17_CAN2_IER);
}

/* All ioctl calls will be routed through this method */
static int  can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg)
{
	dbg("Fix me:Not Implemented\n");
	return 0;
}

/* Send a remote request */
static int  can_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
	dbg("Fix me:Not Implemented\n");
	return 0;
}

static int  can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
	uint32_t tid=CAN_ID(msg->cm_hdr);
	uint32_t tfi=CAN_DLC(msg->cm_hdr)<<16;
	if(CAN_RTR(msg->cm_hdr))
		tfi|=CAN_TFI_RTR;
	//dbg("header=%04x\n",msg->cm_hdr);
	putreg32(tfi,LPC17_CAN2_TFI1);
	putreg32(tid,LPC17_CAN2_TID1);
	putreg32(*(uint32_t *)&msg->cm_data[0],LPC17_CAN2_TDA1);
	putreg32(*(uint32_t *)&msg->cm_data[4],LPC17_CAN2_TDB1);
	putreg32(0x21,LPC17_CAN2_CMR);
	return 0;
}

static bool can_txempty(FAR struct can_dev_s *dev)
{
    uint32_t regval;
	regval  = getreg32(LPC17_CAN2_GSR);
	if( regval & CAN_GSR_TBS)
		return true;
	else
		return false;
}

static int can_interrupt(int irq, void *context)
{
    uint32_t regval;
    regval=getreg32(LPC17_CAN2_ICR);
    if (regval & CAN_ICR_RI )           //Receive interrupt
    {
		uint16_t hdr;
		uint32_t data[2];
		uint32_t rfs=getreg32(LPC17_CAN2_RFS);
		uint32_t rid=getreg32(LPC17_CAN2_RID);
        data[0]=getreg32(LPC17_CAN2_RDA);
        data[1]=getreg32(LPC17_CAN2_RDB);
        putreg32(0x04,LPC17_CAN2_CMR);           //release recieve buffer
		hdr=((rid<<5)&~0x1f)|((rfs>>16)&0x0f);
		if(rfs&CAN_RFS_RTR)
			hdr|=0x10;
        can_receive(&g_candev,hdr,data);
    }
    if ( regval & CAN_ICR_TI1)           //Transmit interrupt 1
		can_txdone(&g_candev);
    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_caninitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *up_caninitialize(int port)
{
    uint32_t regval;
	irqstate_t flags;
   flags = irqsave();

  /* Step 1: Enable power on */
	regval  = getreg32(LPC17_SYSCON_PCONP);
	regval |= SYSCON_PCONP_PCCAN2;
	putreg32(regval, LPC17_SYSCON_PCONP);

  /* Step 2: Enable clocking */
    regval  = getreg32(LPC17_SYSCON_PCLKSEL0);
    regval &= ~SYSCON_PCLKSEL0_CAN2_MASK;
    regval |= (SYSCON_PCLKSEL_CCLK4 << SYSCON_PCLKSEL0_CAN2_SHIFT);
    putreg32(regval, LPC17_SYSCON_PCLKSEL0);

  /* Step 3: Configure I/O pins */
	lpc17_configgpio(GPIO_CAN2_RD_2);
	lpc17_configgpio(GPIO_CAN2_TD_2);

  /* Step 4: Setup */
	putreg32(0x01,LPC17_CAN2_MOD);
	putreg32(0x00,LPC17_CAN2_IER);
	putreg32(0x00,LPC17_CAN2_GSR);
	putreg32(0x02,LPC17_CAN2_CMR);
	putreg32(0x49c009,LPC17_CAN2_BTR);
	putreg32(0x00,LPC17_CAN2_MOD);
	putreg32(0x01,LPC17_CAN2_IER);
    putreg32(0x02,LPC17_CANAF_AFMR);

    irqrestore(flags);
	return &g_candev;
}
#endif

