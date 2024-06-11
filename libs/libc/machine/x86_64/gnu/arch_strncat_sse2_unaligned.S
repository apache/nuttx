#ifndef STRNCAT
# define STRNCAT	strncat
#endif

#define USE_AS_STRNCAT
#define STRCAT	STRNCAT
#include "arch_strcat_sse2_unaligned.S"
