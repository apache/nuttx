#include <sys/stat.h>
#include <sys/boardctl.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>

 int nsh_main(int argc, FAR char *argv[])
{
	fprintf(stderr, "ERROR: Failed to start TELNET daemon: \n");
	return 0;
}
