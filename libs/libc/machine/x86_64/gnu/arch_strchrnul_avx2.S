#ifndef STRCHRNUL
# define STRCHRNUL	strchrnul
#endif

#define STRCHR	STRCHRNUL
#define USE_AS_STRCHRNUL 1

#include "arch_strchr_avx2.S"
