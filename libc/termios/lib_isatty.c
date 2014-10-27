/* Added by Alan Carvalho de Assis
   This code is from:
	https://github.com/raggi/apue.2e/blob/master/termios/isatty.c
   based on Advanced Programming in the UNIX Environment
*/

#include <termios.h>

int isatty(int fd)
{
  struct termios ts;
  return(tcgetattr(fd, &ts) != -1); /* true if no error (is a tty) */
}

