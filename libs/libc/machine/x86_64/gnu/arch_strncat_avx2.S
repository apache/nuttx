#ifndef STRNCAT
# define STRNCAT	strncat
#endif

#define USE_AS_STRNCAT
#define STRCAT	STRNCAT
#include "arch_strcat_avx2.S"
