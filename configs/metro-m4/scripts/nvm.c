#include <stdio.h>
#include <stdint.h>

const uint8_t nvm[20] =
{
  0x14,                                           /* Count 20 bytes */
  0x80, 0x40, 0x00,                               /* 24-address : 804000 */
  0x39, 0x92, 0x9a, 0xfe, 0x80, 0xff, 0xec, 0xae, /* 16-bytes of NVM data */
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

int main(int argc, char **argv)
{
  unsigned int csum;
  int i;

  printf("S2");

  for (i = 0, csum = 0; i < 20; i++)
    {
      csum += nvm[i];
      printf("%02X", (unsigned int)nvm[i]);
    }

  printf("%02X\r\n", ~csum & 0xff);
  printf("S9030000FC\r\n");
  return 0;
}
