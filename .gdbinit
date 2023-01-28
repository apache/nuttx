define reboot
 file nuttx
 set $pc=_sinit
 cont
end

define regs
   p/x (*(SysTick_Type *) 0xE000F000)
end

b ch32v_main
b HardFault_Handler
b ch32v_uart_gpio_init
b ch32v_uart_configure
b riscv_lowputc
b ch32v_lowsetup


file nuttx

target extended-remote localhost:3333

set remotetimeout 10000
