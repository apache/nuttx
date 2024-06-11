#ifndef STRNLEN
# define STRNLEN strnlen
#endif

#define USE_AS_STRNLEN 1
#define STRLEN	STRNLEN

#include "arch_strlen_avx2.S"
