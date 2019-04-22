define hookpost-load

 if &g_readytorun != 0
  eval "monitor nuttx.pid_offset %d", &((struct tcb_s *)(0))->pid
  eval "monitor nuttx.xcpreg_offset %d", &((struct tcb_s *)(0))->xcp.regs
  eval "monitor nuttx.state_offset %d", &((struct tcb_s *)(0))->task_state
  eval "monitor nuttx.name_offset %d", &((struct tcb_s *)(0))->name
  eval "monitor nuttx.name_size %d", sizeof(((struct tcb_s *)(0))->name)
 end

end

define connect
  target remote | openocd -f interface/cmsis-dap.cfg -f cxd5602.cfg -c "gdb_port pipe; log_output openocd.log"
end
