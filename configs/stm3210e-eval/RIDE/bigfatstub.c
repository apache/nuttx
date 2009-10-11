#include "up_internal.h"

void os_start(void)
{
	up_lowputc('X');
	up_lowputc('\n');
	for (;;);
}