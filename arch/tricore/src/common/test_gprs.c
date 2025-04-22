
#include <nuttx/config.h>
#include <stdio.h>
#include <arch/irq.h>

#include "tricore_internal.h"

#include "IfxInt_reg.h"
#include "IfxSrc.h"
#include "IfxCpu.h"

#define gpsr_irq TRICORE_GPSR_IRQNUM(up_this_cpu(), up_this_cpu())

volatile int gprs_irq_flag = 0;

static int gpsr_interrupt(int irq, void *context, void *arg)
{
  gprs_irq_flag++;
  return 0;
}

int gpsr_interrupt_test(void)
{
  irq_attach(gpsr_irq, gpsr_interrupt, NULL);
  up_enable_irq(gpsr_irq);

  cpu_set_t  cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(up_this_cpu(), &cpuset);
  up_trigger_irq(gpsr_irq, cpuset);
  return 0;
}
