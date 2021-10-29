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

/*******************************************************************************
    @file     flash.c
    @brief    Contains all functions support for flash driver
    @version  0.0
    @date     27. Nov. 2017
    @author   qing.han



*******************************************************************************/
#include "rom_sym_def.h"
#include <string.h>
#include "types.h"
#include "flash.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"


#define SPIF_WAIT_IDLE_CYC                          (32)

#define SPIF_STATUS_WAIT_IDLE(n)                    \
    do                                              \
    {                                               \
        while((AP_SPIF->fcmd &0x02)==0x02);         \
        {                                           \
            volatile int delay_cycle = n;           \
            while (delay_cycle--){;}                \
        }                                           \
        while ((AP_SPIF->config & 0x80000000) == 0);\
    } while (0);


#define HAL_CACHE_ENTER_BYPASS_SECTION()  do{ \
        _HAL_CS_ALLOC_();\
        HAL_ENTER_CRITICAL_SECTION();\
        AP_CACHE->CTRL0 = 0x02; \
        AP_PCR->CACHE_RST = 0x02;\
        AP_PCR->CACHE_BYPASS = 1;    \
    }while(0);


#define HAL_CACHE_EXIT_BYPASS_SECTION()  do{ \
        _HAL_CS_ALLOC_();\
        HAL_ENTER_CRITICAL_SECTION();\
        AP_CACHE->CTRL0 = 0x00;\
        AP_PCR->CACHE_RST = 0x03;\
        AP_PCR->CACHE_BYPASS = 0;\
        HAL_EXIT_CRITICAL_SECTION();\
    }while(0);

#define spif_wait_nobusy(flg, tout_ns, return_val)   {if(_spif_wait_nobusy_x(flg, tout_ns)){if(return_val){ return return_val;}}}

static xflash_Ctx_t s_xflashCtx = {.spif_ref_clk=SYS_CLK_DLL_64M,.rd_instr=XFRD_FCMD_READ_DUAL};

chipMAddr_t  g_chipMAddr;

static void hal_cache_tag_flush(void);



static void __RAMRUN hal_cache_tag_flush(void)
{
    _HAL_CS_ALLOC_();
    HAL_ENTER_CRITICAL_SECTION();
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    volatile int dly = 8;

    if(cb==0)
    {
        AP_PCR->CACHE_BYPASS = 1;
    }

    AP_CACHE->CTRL0 = 0x02;

    while (dly--) {;};

    AP_CACHE->CTRL0 = 0x03;

    dly = 8;

    while (dly--) {;};

    AP_CACHE->CTRL0 = 0x00;

    if(cb==0)
    {
        AP_PCR->CACHE_BYPASS = 0;
    }

    HAL_EXIT_CRITICAL_SECTION();
}


static uint8_t __RAMRUN _spif_read_status_reg_x(void)
{
    uint8_t status;
    spif_cmd(FCMD_RDST, 0, 2, 0, 0, 0);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_rddata(&status, 1);
    return status;
}

static int __RAMRUN _spif_wait_nobusy_x(uint8_t flg, uint32_t tout_ns)
{
    uint8_t status;
    volatile int tout = (int )(tout_ns);

    for(; tout ; tout --)
    {
        status = _spif_read_status_reg_x();

        if((status & flg) == 0)
            return PPlus_SUCCESS;

        //insert polling interval
        //5*32us
        WaitRTCCount(5);
    }

    return PPlus_ERR_BUSY;
}
void __RAMRUN hal_spif_init(void)
{
  uint32_t tmp = 0;
  //*(volatile uint32_t *) 0x4000c800 = 0x80080081;  //enable qspi(divisor 4)

  *(volatile uint32_t *) 0x4000c804 = 0x801003B;    // Device Read Instruction Register , Dual IO
  return PPlus_SUCCESS;
#if 0  
  //*(volatile uint32_t *) 0x4000c800 = 0x80280081;  //enable qspi(divisor 12)
	//return 0;
  *(volatile uint32_t *) 0x4000c890 = 0x6000001;    // config flash: WREN  06h
  //*(volatile uint32_t *) 0x4000c890 = 0x50000001;    // config flash: Write Enable for Volatile Status Reg 50h
  *(volatile uint32_t *) 0x4000c8a8 = 0x200;    // config Flash Command Write Data Register (Lower) 
  *(volatile uint32_t *) 0x4000c890 = 0x1009001;    // config flash:   write status reg 01h    QE=1
  while ((tmp&2)!= 0x2)   // QE = 1 ?
  {
    *(int *) 0x4000c890 = 0x35900001;    // Read Status Reg-2  35h
    tmp = *(uint32_t *) 0x4000c8a0;
  }
  *(volatile uint32_t *) 0x4000c828 = 0x10;         // M5-4   Mode Bit Configuration Register 
  *(volatile uint32_t *) 0x4000c804 = 0x41220EB;    // Device Read Instruction Register 
  return PPlus_SUCCESS;


  //load defualt configure
  while(1){//wait spif idle
    if(AP_SPIF->config &BIT(31)){
      WaitUs(1);
      break;
    }
  }
#endif

}

void __RAMRUN hal_cache_init(void)
{
    volatile int dly=100;
    hal_spif_init();
    //clock gate
    hal_clk_gate_enable(MOD_HCLK_CACHE);
    hal_clk_gate_enable(MOD_PCLK_CACHE);
    //cache rst ahp
    AP_PCR->CACHE_RST=0x02;

    while(dly--) {};

    AP_PCR->CACHE_RST=0x03;

    hal_cache_tag_flush();

    //cache enable
    AP_PCR->CACHE_BYPASS = 0;
}

static void __RAMRUN hw_spif_cache_config(void)
{
    spif_config(s_xflashCtx.spif_ref_clk,/*div*/1,s_xflashCtx.rd_instr,0,0);
    AP_SPIF->wr_completion_ctrl=0xff010005;//set longest polling interval
    NVIC_DisableIRQ(SPIF_IRQn);
    NVIC_SetPriority((IRQn_Type)SPIF_IRQn, IRQ_PRIO_HAL);
    hal_cache_init();
}
int __RAMRUN hal_spif_cache_init(void)//xflash_Ctx_t cfg)
{
    //memset(&(s_xflashCtx), 0, sizeof(s_xflashCtx));
    //memcpy(&(s_xflashCtx), &cfg, sizeof(s_xflashCtx));
    hw_spif_cache_config();
    hal_pwrmgr_register(MOD_SPIF, NULL,  hw_spif_cache_config);
    return PPlus_SUCCESS;
}

int  __RAMRUN hal_flash_read(uint32_t addr, uint8_t* data, uint32_t size)
{
    volatile uint8_t* u8_spif_addr = (volatile uint8_t*)((addr & 0x7ffff) | FLASH_BASE_ADDR);
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    #if(SPIF_FLASH_SIZE==FLASH_SIZE_1MB)
    uint32_t remap = addr & 0xf80000;

    if (remap)
    {
        AP_SPIF->remap = remap;
        AP_SPIF->config |= 0x10000;
    }

    #endif

    //read flash addr direct access
    //bypass cache
    HAL_CACHE_ENTER_BYPASS_SECTION();

    for(int i=0; i<size; i++)
        data[i]=u8_spif_addr[i];

    //bypass cache
    HAL_CACHE_EXIT_BYPASS_SECTION();

    #if(SPIF_FLASH_SIZE==FLASH_SIZE_1MB)

    if (remap)
    {
        AP_SPIF->remap = 0;
        AP_SPIF->config &= ~0x10000ul;
    }

    #endif
    return PPlus_SUCCESS;
}

int  __RAMRUN hal_flash_write(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint8_t retval;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_write(addr,data,size);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();
    return retval;
}

int __RAMRUN hal_flash_erase_sector(unsigned int addr)
{
    uint8_t retval;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_erase_sector(addr);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if(cb == 0)
    {
        hal_cache_tag_flush();
    }

    return retval;
}


int __RAMRUN flash_write_word(unsigned int offset, uint32_t  value)
{
    uint32_t temp = value;
    offset &= 0x00ffffff;
    return (hal_flash_write (offset, (uint8_t*) &temp, 4));
}



