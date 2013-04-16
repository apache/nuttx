void blueled(int on)
{
  volatile unsigned int *SIM_COPC = ((volatile unsigned int *)0x40048100);
  volatile unsigned int *SIM_SCGC5 = ((volatile unsigned int *)0x40048038);
  volatile unsigned int *PORTD_PCR1 = ((volatile unsigned int *)0x4004C004);
  volatile unsigned int *GPIOD_PSOR = ((volatile unsigned int *)0x400FF0C4);
  volatile unsigned int *GPIOD_PCOR = ((volatile unsigned int *)0x400FF0C8);
  volatile unsigned int *GPIOD_PDDR = ((volatile unsigned int *)0x400FF0D4);

  /*acassis: disable SIM_COP*/
  *SIM_COPC = 0;

  /* enable clocks for PORTD */
  *SIM_SCGC5 = 0x1000;

  /* set D1 to GPIO */
  *PORTD_PCR1 = 0x100;

  /* set D1 DDR to output */
  *GPIOD_PDDR |= 2;

  if(on)
    *GPIOD_PCOR = 2;
  else
      *GPIOD_PSOR = 2;

}

