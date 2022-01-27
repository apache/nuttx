

MEMORY
{
  jumptbl (rwx) : ORIGIN = 0x1fff0000, LENGTH = 1K
  gcfgtbl  (rwx) : ORIGIN = 0x1fff0400, LENGTH = 1K
  flash (rx) : ORIGIN = 0x1100e000, LENGTH = 384K
  sram (rwx) : ORIGIN = 0x1fff1880, LENGTH = 59264
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{

    .textentry : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *phy62xx_start.o(.text)
    } > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab : {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

 
    ._sjtblsstore : {
       _sjtblss = ABSOLUTE(.);
    } > flash   

   .jumptbls : {
        _sjtbls = ABSOLUTE(.);
        *jump_table*(.jumptbls)
        _ejtbls = ABSOLUTE(.);
    } > jumptbl AT >flash
    
    .gcfgtbls : {
       _sgtbls = ABSOLUTE(.);
        *jump_table*(.gcfgtbls)
       _egtbls = ABSOLUTE(.);
    } > gcfgtbl


   ._eronlystore : {
       _eronly = ABSOLUTE(.);
   } > flash   

    .data : {
        _sdata = ABSOLUTE(.);
        _stextram = ABSOLUTE(.);
        *libarch.a:*flash.o(.text .text.*)
	*libarch.a:phy62xx_exception.o(.text .text.*)
	*libarch.a:irq.o(.text.arm_ack_irq)
	*phy62xx_ble_patch.o(.text .text.*)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_hw_go1)
        //*libarch.a:phy62xx_ble_patch.o(.text.TIM1_IRQHandler1)
        //*libarch.a:phy62xx_ble_patch.o(.text.LL_IRQHandler1)
        //*libarch.a:phy62xx_ble_patch.o(.text.rf_phy_change_cfg0)
        //*libarch.a:phy62xx_ble_patch.o(.text.rf_calibrate1)
        //*libarch.a:phy62xx_ble_patch.o(.text.l2capPocessFragmentTxData)
        //*libarch.a:phy62xx_ble_patch.o(.text.LL_SetDataLengh1)
        //*libarch.a:phy62xx_ble_patch.o(.text.llProcessTxData1)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_generateTxBuffer1)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_adptive_adj_next_time1)
        //*libarch.a:phy62xx_ble_patch.o(.text.llSecAdvAllow1)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_scheduler1)
        //*libarch.a:phy62xx_ble_patch.o(.text.osal_set_event1)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_processBasicIRQ_SRX)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_processBasicIRQ_secondaryAdvTRX)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_processBasicIRQ_ScanTRX)
        //*libarch.a:phy62xx_ble_patch.o(.text.llSlaveEvt_TaskEndOk1)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_processBasicIRQ_secondaryAdvTRX0)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_processBasicIRQ_ScanTRX0)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_processBasicIRQ_SRX0)
        //*libarch.a:phy62xx_ble_patch.o(.text.ll_hw_read_rfifo1)
		

        *libphy6222_rf.a:patch.o(.text.ll_hw_go1)
        *libphy6222_rf.a:patch.o(.text.TIM1_IRQHandler1)
        *libphy6222_rf.a:patch.o(.text.LL_IRQHandler1)
        *libphy6222_rf.a:patch.o(.text.rf_phy_change_cfg0)
        *libphy6222_rf.a:patch.o(.text.rf_calibrate1)
        *libphy6222_rf.a:patch.o(.text.l2capPocessFragmentTxData)
        *libphy6222_rf.a:patch.o(.text.LL_SetDataLengh1)
        *libphy6222_rf.a:patch.o(.text.llProcessTxData1)
        *libphy6222_rf.a:patch.o(.text.ll_generateTxBuffer1)
        *libphy6222_rf.a:patch.o(.text.ll_adptive_adj_next_time1)
        *libphy6222_rf.a:patch.o(.text.llSecAdvAllow1)
        *libphy6222_rf.a:patch.o(.text.ll_scheduler1)
        *libphy6222_rf.a:patch.o(.text.osal_set_event1)
        *libphy6222_rf.a:patch.o(.text.ll_processBasicIRQ_SRX)
        *libphy6222_rf.a:patch.o(.text.ll_processBasicIRQ_secondaryAdvTRX)
        *libphy6222_rf.a:patch.o(.text.ll_processBasicIRQ_ScanTRX)
        *libphy6222_rf.a:patch.o(.text.llSlaveEvt_TaskEndOk1)
        *libphy6222_rf.a:patch.o(.text.ll_processBasicIRQ_secondaryAdvTRX0)
        *libphy6222_rf.a:patch.o(.text.ll_processBasicIRQ_ScanTRX0)
        *libphy6222_rf.a:patch.o(.text.ll_processBasicIRQ_SRX0)
        *libphy6222_rf.a:patch.o(.text.ll_hw_read_rfifo1)
		
        *libphy6222_rf.a:patch.o(.text.LL_set_default_conn_params1)
        *libphy6222_rf.a:patch.o(.text.llConnTerminate1)
        *libphy6222_rf.a:patch.o(.text.config_RTC1)
        *libphy6222_rf.a:patch.o(.text.wakeup_init1)
        *libphy6222_rf.a:patch.o(.text.LL_ENC_AES128_Encrypt1)
        *libphy6222_rf.a:patch.o(.text.LL_ENC_Encrypt1)
        *libphy6222_rf.a:patch.o(.text.LL_ENC_Decrypt1)
        *libphy6222_rf.a:patch.o(.text.ll_processBasicIRQ_secondaryScanSRX)
        *libphy6222_rf.a:patch.o(.text.llProcessSlaveControlProcedures1)
        *libphy6222_rf.a:patch.o(.text.llCalcMaxScanTime1)
        *libphy6222_rf.a:patch.o(.text.LL_SetAdvControl1)
        *libphy6222_rf.a:patch.o(.text.llSetupSecAdvEvt1)
        *libphy6222_rf.a:patch.o(.text.ll_scheduler2)
        *libphy6222_rf.a:patch.o(.text.llSetupNextSlaveEvent1)
		
	*libapps.a:flash.c.*.o(.text .text.*)
	*libapps.a:*.o(.text.drv_disable_irq1)
	*libapps.a:*.o(.text.drv_enable_irq1)

        *rf_phy_driver.o(.text.rf_phy_get_pktFoot)
        *rf_phy_driver.o(.text.rf_phy_change_cfg0 )
        *libphy6222_host.a:l2cap_util.o(.text.L2CAP_Fragment_SendDataPkt)
        *libphy6222_host.a:l2cap_util.o(.text.l2capSegmentBuffToLinkLayer)
        *libphy6222_host.a:l2cap_util.o(.text.l2capPocessFragmentTxData)
		
        *libarch.a:phy62xx_ble_hcitl.o(.text.phy62xx_ble_init)
        *libarch.a:phy62xx_ble_hcitl.o(.text.HCI_ProcessEvent1)
        *libarch.a:up_idle.o(.text .text.*)
        *libarch.a:up_idle.o(.text.up_block_task)
        *libarch.a:arm_switchcontext.o(.text .text.*)
        *libarch.a:arm_fullcontextrestore.o(.text .text.*)
        *libarch.a:arm_fullcontextrestore.o(.text .text.*)
        *libarch.a:timer.o(.text.systic_timerisr)
        *libarch.a:phy62xx_ble.o(.text.pplus_ble_recv_cb_h4 .text.pplus_ble_recv_cb_acl)
        *libarch.a:phy62xx_ble.o(.text.pplus_ble_recv_msg )
        *libarch.a:arm_doirq.o(.text.arm_doirq )
        *libarch.a:phy62xx_hardfault.o(.text.arm_hardfault )
        *libsched.a:irq_dispatch.o(.text.irq_dispatch )
		
        *libsched.a:clock_initialize.o(.text.clock_timer)
        *libsched.a:sched_processtimer.o(.text.nxsched_process_timer)
        *libsched.a:sem_wait.o(.text .text.*)
        *libsched.a:sem_holder.o(.text .text.*)
        *libsched.a:sched_yield.o(.text .text.*)
        *libsched.a:sched_lock.o(.text .text.*)
        *libsched.a:sched_unlock.o(.text .text.*)
		

        *libdrivers.a:uart_bth4.o(.text.uart_bth4_pollnotify)
        *libdrivers.a:uart_bth4.o(.text.uart_bth4_post)
        *libdrivers.a:uart_bth4.o(.text.uart_bth4_receive)
		
	*libarch.a:uart.o(.text .text.*)
		
        *libmm.a:circbuf.o(.text .text.*)
		
	*libc.a:lib_libvsprintf.o(.text .text.*)
	*libc.a:lib_printf.o(.text .text.*)
	*libc.a:lib_vfprintf.o(.text .text.*)
	*libc.a:lib_skipspace.o(.text .text.*)
        *libc.a:lib_sprintf.o(.text .text.*)
        *libc.a:lib_strlen.o(.text .text.*)
        *libc.a:lib_memcmp.o(.text .text.*)
        *libc.a:lib_memcpy.o(.text .text.*)
        *libc.a:lib_memset.o(.text .text.*)
        *libc.a:lib_memmove.o(.text .text.*)
        
	*libapps.a:zblue.o(.text.k_yield .text.k_sleep .text.z_tick_get)

        _etextram = ABSOLUTE(.);


        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash



    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram
    
   .common_text : {
       *(.text .text.*)
       *(.rodata .rodata.*)
       *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
   } > flash   
   
   
    /* Stabs debugging sections. */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}

