#ifndef __HOSTDEFS_H
#define __HOSTDEFS_H

# include <stdio.h>

typedef unsigned char ubyte;
typedef unsigned short uint16;
typedef unsigned int  uint32;
typedef unsigned char boolean;

# define HTONS(a) htons(a)
# define HTONL(a) htonl(a)
# define CONFIG_CPP_HAVE_WARNING   1
# define CONFIG_HAVE_GETHOSTBYNAME 1
# define FAR

# define ndbg(...) printf(__VA_ARGS__)
# define nvdbg(...) printf(__VA_ARGS__)

# define TRUE  (1)
# define FALSE (0)
# define ERROR (-1)
# define OK    (0)

#endif /* __HOSTDEFS_H */
