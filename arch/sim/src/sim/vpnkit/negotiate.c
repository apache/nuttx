#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "protocol.h"

#define ERROR(fmt, ...) \
        fprintf(stderr, "up_vpnkit: " fmt "\r\n", ##__VA_ARGS__)
#define INFO(fmt, ...) \
        fprintf(stderr, "up_vpnkit: " fmt "\r\n", ##__VA_ARGS__)

/* Negotiate a vmnet connection, returns 0 on success and 1 on error. */
int negotiate(int fd, struct vif_info *vif)
{
	enum command command = ethernet;
	struct init_message *me;
	struct ethernet_args args;
	struct init_message you;
	char *txt;

	me = create_init_message();
	if (!me)
		goto err;

	if (write_init_message(fd, me) == -1)
		goto err;

	if (read_init_message(fd, &you) == -1)
		goto err;

	if (me->version != you.version) {
		ERROR("Server did not accept our protocol version (client: %d, server: %d)", me->version, you.version);
		goto err;
	}

	txt = print_init_message(&you);
	if (!txt)
		goto err;

	INFO("Server reports %s", txt);
	free(txt);

	if (write_command(fd, &command) == -1)
		goto err;

	/* We don't need a uuid */
	memset(&args.uuid_string[0], 0, sizeof(args.uuid_string));
	if (write_ethernet_args(fd, &args) == -1)
		goto err;

	if (read_vif_response(fd, vif) == -1)
		goto err;

	return 0;
err:
	ERROR("Failed to negotiate vmnet connection");
	return 1;
}
