#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <debug.h>

#define LINE_LEN 16

void bcmf_hexdump(uint8_t *data, unsigned int len, unsigned long offset)
{
	unsigned int i;
	unsigned int char_count = 0;
	char char_line[20];
	char hex_line[64];

	for(i = 0; i < len; i++)
	{
		if (char_count >= LINE_LEN) {
			/* Flush line */
			_info("%08x: %s%s\n", offset+i-char_count, hex_line, char_line);
			char_count = 0;
		}

		sprintf(hex_line+3*char_count, "%02x ", data[i]);
		sprintf(char_line+char_count, "%c",
				data[i] < 0x20 || data[i] >= 0x7f? '.': data[i]);
		char_count ++;
	}

	if (char_count > 0) {
		/* Flush last line */
		memset(hex_line+3*char_count, ' ', 3*(LINE_LEN-char_count));
		hex_line[3*LINE_LEN] = 0;
		_info("%08x: %s%s\n", offset+i-char_count, hex_line, char_line);
	}
}