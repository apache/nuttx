/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/
#include "clock.h"
#include "gpio.h"
//#include "global_config.h"
#include "error.h"

extern uint32_t hclk,pclk;
extern uint32_t osal_sys_tick;

void hal_clk_gate_enable(MODULE_e module)
{
    if(module < MOD_CP_CPU)
    {
        AP_PCR->SW_CLK |= BIT(module);
    }
    else if(module < MOD_PCLK_CACHE)
    {
        AP_PCR->SW_CLK1 |= BIT(module-MOD_CP_CPU);
    }
    else if(module < MOD_USR0)
    {
        AP_PCR->CACHE_CLOCK_GATE |= BIT(module-MOD_PCLK_CACHE);
    }
}

void hal_clk_gate_disable(MODULE_e module)
{
    if(module < MOD_CP_CPU)
    {
        AP_PCR->SW_CLK &= ~(BIT(module));
    }
    else if(module < MOD_PCLK_CACHE)
    {
        AP_PCR->SW_CLK1 &= ~(BIT(module-MOD_CP_CPU));
    }
    else if(module < MOD_USR0)
    {
        AP_PCR->CACHE_CLOCK_GATE &= ~(BIT(module-MOD_PCLK_CACHE));
    }
}

int hal_clk_gate_get(MODULE_e module)
{
    if(module < MOD_CP_CPU)
    {
        return (AP_PCR->SW_CLK & BIT(module));
    }
    else if(module < MOD_PCLK_CACHE)
    {
        return (AP_PCR->SW_CLK1 & BIT(module-MOD_CP_CPU));
    }
    //else if(module < MOD_USR0)
    else
    {
        return (AP_PCR->CACHE_CLOCK_GATE & BIT(module-MOD_PCLK_CACHE));
    }
}

void hal_clk_get_modules_state(uint32_t* buff)
{
    *buff     = AP_PCR->SW_CLK;
    *(buff+1) = AP_PCR->SW_CLK1;
    *(buff+2) = AP_PCR->CACHE_CLOCK_GATE;
}

void hal_clk_reset(MODULE_e module)
{
    if(module < MOD_CP_CPU)
    {
        if((module >= MOD_TIMER5) &&(module <= MOD_TIMER6))
        {
            AP_PCR->SW_RESET0 &= ~BIT(5);
            AP_PCR->SW_RESET0 |= BIT(5);
        }
        else
        {
            AP_PCR->SW_RESET0 &= ~BIT(module);
            AP_PCR->SW_RESET0 |= BIT(module);
        }
    }
    else if(module < MOD_PCLK_CACHE)
    {
        if((module >= MOD_TIMER1) &&(module <= MOD_TIMER4))
        {
            AP_PCR->SW_RESET2 &= ~BIT(4);
            AP_PCR->SW_RESET2 |= BIT(4);
        }
        else
        {
            AP_PCR->SW_RESET2 &= ~BIT(module-MOD_CP_CPU);
            AP_PCR->SW_RESET2 |= BIT(module-MOD_CP_CPU);
        }
    }
    else if(module < MOD_USR0)
    {
        AP_PCR->CACHE_RST &= ~BIT(1-(module-MOD_HCLK_CACHE));
        AP_PCR->CACHE_RST |= BIT(1-(module-MOD_HCLK_CACHE));
    }
}


void hal_rtc_clock_config(CLK32K_e clk32Mode)
{
    if(clk32Mode == CLK_32K_RCOSC)
    {
        subWriteReg(&(AP_AON->PMCTL0),31,27,0x05);
        subWriteReg(&(AP_AON->PMCTL2_0),16,7,0x3fb);
        subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x01);
        //pGlobal_config[LL_SWITCH]|=RC32_TRACKINK_ALLOW|LL_RC32K_SEL;
    }
    else if(clk32Mode == CLK_32K_XTAL)
    {
        // P16 P17 for 32K XTAL input
        hal_gpio_pull_set(P16,FLOATING);
        hal_gpio_pull_set(P17,FLOATING);
        subWriteReg(&(AP_AON->PMCTL2_0),9,8,0x03);   //software control 32k_clk
        subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x00);   //disable software control
        subWriteReg(&(AP_AON->PMCTL0),31,27,0x16);
        //pGlobal_config[LL_SWITCH]&=0xffffffee;
    }

    //ZQ 20200812 for rc32k wakeup
    subWriteReg(&(AP_AON->PMCTL0),28,28,0x1);//turn on 32kxtal
    subWriteReg(&(AP_AON->PMCTL1),18,17,0x0);// reduce 32kxtl bias current
}



uint32_t hal_systick(void)
{
    return 10000;//osal_sys_tick;
}

uint32_t hal_ms_intv(uint32_t tick)
{
    uint32_t diff = 0;
/*
    if(osal_sys_tick < tick)
    {
        diff = 0xffffffff- tick;
        diff = osal_sys_tick + diff;
    }
    else
    {
        diff = osal_sys_tick - tick;
    }
*/
    return diff*625/1000;
}

/**************************************************************************************
    @fn          WaitMs

    @brief       This function process for wait program msecond,use RTC

    input parameters

    @param       uint32_t msecond: the msecond value

    output parameters

    @param       None.

    @return      None.
 **************************************************************************************/

void WaitMs(uint32_t msecond)
{
    //WaitRTCCount((msecond<<15)/1000);
}

void WaitUs(uint32_t wtTime)
{
    uint32_t T0,currTick,deltTick;
    //T0 = read_current_time();
    T0 =(TIME_BASE - ((AP_TIM3->CurrentCount) >> 2));

    while(1)
    {
        currTick = (TIME_BASE - ((AP_TIM3->CurrentCount) >> 2));
        deltTick = TIME_DELTA(currTick,T0);

        if(deltTick>wtTime)
            break;
    }
}

void hal_system_soft_reset(void)
{
    _HAL_CS_ALLOC_();
	HAL_ENTER_CRITICAL_SECTION();
    AP_PCR->SW_RESET1 = 0;

    while(1);
}

