#include <arch/board/generated/csr.h>

void uart_isr();
void uart_isr()
{

}

void isr(void);
void isr(void)
{
  unsigned int irqs;
  
  irqs = irq_pending() & irq_getmask();
  
  if (irqs & (1 << UART_INTERRUPT))
    {
      uart_isr();
    }
}
