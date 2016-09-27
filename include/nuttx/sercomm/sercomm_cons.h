#ifndef __INCLUDE_NUTTX_SERCOMM_SERCOMM_CONS_H
#define __INCLUDE_NUTTX_SERCOMM_SERCOMM_CONS_H

/* how large buffers do we allocate? */
#define SERCOMM_CONS_ALLOC	256

int sercomm_puts(const char *s);
int sercomm_putchar(int c);

#endif /* __INCLUDE_NUTTX_SERCOMM_SERCOMM_CONS_H */
