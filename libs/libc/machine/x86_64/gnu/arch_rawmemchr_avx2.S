#ifndef RAWMEMCHR
# define RAWMEMCHR	rawmemchr
#endif
#define USE_AS_RAWMEMCHR	1
#define MEMCHR	RAWMEMCHR

#include "arch_memchr_avx2.S"
