/****************************************************************************
 * tools/rp2040/make_flash_fs.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <ctype.h>
#include <dirent.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include <sys/dir.h>
#include <sys/errno.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_NAME_LEN     16
#define MAX_SECTOR_DATA  (1024 - 10)
#define MAX_DIR_COUNT    (MAX_SECTOR_DATA / 24)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct dir_item_s
{
  struct dir_item_s *prior;
  bool               is_directory;
  int                init_sector;
  int                permissions;
  char               name[0];
} dir_item_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

int         max_name_len   = MAX_NAME_LEN;
int         root_sector    = 3;
char        path[4096];
uint8_t    *buffer;

const char *preamble =
  "\n     .macro      sector        num, type, used=0xffff, next=0xffff"
  "\n     .balign     1024, 0xff"
  "\n     .hword      \\num, 0"
  "\n     .byte       0b01101001, \\type"
  "\n 1:"
  "\n     .hword      \\next, \\used"
  "\n     .endm"
  "\n"
  "\n     .macro      dir_entry     perm, addr, time, name"
  "\n     .hword      \\perm | 0x7e00, \\addr"
  "\n     .word       \\time"
  "\n 0:"
  "\n     .ascii      \"\\name\""
  "\n .=      0b + name_length"
  "\n     .endm"
  "\n"
  "\n     .macro      file_entry     perm, addr, time, name"
  "\n     .hword      \\perm | 0x5e00, \\addr"
  "\n     .word       \\time"
  "\n 0:"
  "\n     .ascii      \"\\name\""
  "\n .=      0b + name_length"
  "\n     .endm"
  "\n"
  "\n     .cpu        cortex-m0plus"
  "\n     .thumb"
  "\n"
  "\n     .section    .flash.init, \"ax\""
  "\n     .balign     4096"
  "\n     .global     rp2040_smart_flash_start"
  "\n rp2040_smart_flash_start:"
  "\n     .ascii      \"2040\""
  "\n     .byte       0b01101001"
  "\n     .ascii      \"SMRT\""
  "\n     .byte       0x01, 0x10, 0"
  "\n"
  "\n     .balign     4096, 0xff";

const char *postamble =
  "\n"
  "\n     .balign     4096, 0xff"
  "\n     .global     rp2040_smart_flash_end"
  "\n rp2040_smart_flash_end:"
  "\n"
  "\n      .end";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void put_name(const char * cp)
{
  putchar('"');

  for (; *cp != 0; ++cp)
    {
      switch (*cp)
        {
          case '\"': printf("\\\""); break;
          case '\'': printf("\\\'"); break;
          case '\\': printf("\\\\"); break;
          case '\a': printf("\\a");  break;
          case '\b': printf("\\b");  break;
          case '\n': printf("\\n");  break;
          case '\t': printf("\\t");  break;
          default:
            if (iscntrl(*cp))
              {
                printf("\\%03o", *cp);
              }
            else
              {
                putchar(*cp);
              }
        }
    }

  putchar('"');
}

/****************************************************************************
 * Name: copy_file
 ****************************************************************************/

int copy_file(int in_sector, const char * in_name)
{
  FILE         *data_file = fopen(path, "r");
  struct stat   file_stat;
  int           sector_len;
  uint8_t      *bp;

  if (data_file == NULL) return in_sector;

  lstat(path, &file_stat);

  while (file_stat.st_size > 0)
    {
      bp         = buffer;
      sector_len = fread(buffer, 1, MAX_SECTOR_DATA, data_file);

      file_stat.st_size -= sector_len;

      if (file_stat.st_size > 0)
        {
          printf("\n     sector   %4d, dir, used=%d, next=%d",
                 in_sector,
                 sector_len,
                 in_sector + 1);
        }
      else
        {
          printf("\n     sector   %4d, file, used=%d",
                in_sector,
                sector_len);
        }

      in_sector += 1;

      for (; sector_len > 0; sector_len -= 8)
        {
          printf("\n     .byte      ");

          for (int i = 0; i < 8  &&  i < sector_len; ++i)
            {
              if (i != 0) printf(", ");

              printf("0x%02x", *bp++);
            }
        }

      printf("\n");
    }

  return in_sector;
}

/****************************************************************************
 * Name: dir_entry
 *
 * On entry:
 *  The global "path" will be path to the prototype directory.
 ****************************************************************************/

int scan_dir(int in_sector)
{
  dir_item_t    *an_item    = NULL;
  dir_item_t    *prior_item = NULL;
  int            path_len   = strlen(path);
  int            item_count = 0;
  int            sector     = in_sector + 1;
  DIR           *input_dir;
  struct dirent *a_dirent;
  struct stat    stat;
  int            name_len;

  if (name_len > max_name_len)
    {
      fprintf(stderr, "directory name to big. skipped. (%s)\n", path);
      return -1;
    }

  input_dir = opendir(path);

  if (input_dir == NULL)
    {
      fprintf(stderr,
              "could not open directory %s  %s\n",
              path,
              strerror(errno));

      return -1;
    }

  /* scan directory to create directory entries */

  while ((a_dirent = readdir(input_dir)) != NULL)
    {
      if (strcmp(a_dirent->d_name, ".")  == 0) continue;
      if (strcmp(a_dirent->d_name, "..") == 0) continue;

      path[path_len] = '/';

      strncpy(&path[path_len + 1], a_dirent->d_name, 4094 - path_len);

      if (a_dirent->d_type == DT_DIR  ||  a_dirent->d_type == DT_REG)
        {
          name_len = strlen(a_dirent->d_name);

          if (name_len > max_name_len)
            {
              fprintf(stderr, "Skipped item. Name too long: %s\n", path);
              continue;
            }

          item_count += 1;

          an_item = malloc(sizeof(dir_item_t) + name_len + 1);

          an_item->prior = prior_item;
          prior_item     = an_item;

          strncpy(an_item->name, a_dirent->d_name, name_len);

          an_item->init_sector  = sector;

          if (a_dirent->d_type == DT_DIR)
            {
              an_item->is_directory = true;
              an_item->permissions  = 0777;

              sector = scan_dir(sector);
            }
          else
            {
              an_item->is_directory = false;
              an_item->permissions  = 0666;

              sector = copy_file(sector, an_item->name);
            }
        }
      else
        {
          fprintf(stderr, "Skipped unusable item: %s\n", path);
        }
    }

  path[path_len] = 0;

  closedir(input_dir);

  /* Generate the directory sector for this directory. */

  if (item_count > MAX_DIR_COUNT)
    {
      printf("\n     sector   %4d, dir, next=%d", in_sector, sector);
    }
  else
    {
      printf("\n     sector   %4d, dir", in_sector);
    }

  while (an_item != NULL)
    {
      for (int i = 0; i < MAX_DIR_COUNT && an_item != NULL; ++i)
        {
          if (an_item->is_directory)
            {
              printf("\n     dir_entry   0%03o, 0x%04x, 0, ",
                    an_item->permissions,
                    an_item->init_sector);

              put_name(an_item->name);
            }
          else
            {
              printf("\n     file_entry  0%03o, 0x%04x, 0, ",
                    an_item->permissions,
                    an_item->init_sector);

              put_name(an_item->name);
            }

          prior_item = an_item->prior;
          free(an_item);
          an_item = prior_item;
          item_count -= 1;
        }

      if (an_item != NULL)
        {
          printf("\n     sector   %4d, dir, used=%d",
                sector,
                (item_count > MAX_DIR_COUNT) ? (sector + 1) : 0xffff);

          sector += 1;
        }
    }

  printf("\n");

  return sector;
}

/****************************************************************************
 * Name: print_help
 ****************************************************************************/

void print_help(const char * name)
{
  fprintf(stderr, "\nusage: %s [-h][-n name-len] source-path\n\n", name);

  fprintf(stderr, "-h  : print this help and exit.\n\n");

  fprintf(stderr, "-n  : name length is the length of file names in\n");
  fprintf(stderr, "      the smart filesystem.  It must match the\n");
  fprintf(stderr, "      CONFIG_SMARTFS_MAXNAMLEN with which NuttX was\n");
  fprintf(stderr, "      built.  The default is %d.\n\n", max_name_len);

  fprintf(stderr, "source-path is the path to a prototype directory tree\n");
  fprintf(stderr, "      that will be used to build the initial smart\n");
  fprintf(stderr, "      filesystem.  Any items that are not normal\n");
  fprintf(stderr, "      files or directories will be ignored.\n\n");

  fprintf(stderr, "Output is to stdout.\n\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  FILE         *output_file;
  const char   *input_path   = NULL;

  /* parse arguments */

  for (int i = 1; i < argc; ++i)
    {
      if (argv[i][0] == '-')
        {
          if (argv[i][1] == 'h')
            {
              print_help(argv[0]);
              return 0;
            }

          if (argv[i][1] == 'n')
            {
              if (argv[i][2] == 0  &&  argc > i + 1)
                {
                  i += 1;

                  if (!isdigit(argv[i][0]))
                    {
                      fprintf(stderr, "invalid name length.\n");
                      print_help(argv[0]);
                      return 1;
                    }

                  max_name_len = atoi(argv[i]);
                }
              else if (isdigit(argv[i][2]))
                {
                  max_name_len = atoi(&(argv[i][2]));
                }
              else
                {
                  fprintf(stderr, "invalid name length.\n");
                  print_help(argv[0]);
                  return 1;
                }
            }
          else
            {
              fprintf(stderr, "unknown argument: %c\n", argv[i][1]);
              print_help(argv[0]);
              return 1;
            }
        }
      else if (input_path == NULL)
        {
          input_path = argv[i];
        }
    }

  if (input_path == NULL)
    {
      print_help(argv[0]);
      return 1;
    }

  buffer = malloc(MAX_SECTOR_DATA);

  if (buffer == NULL)
    {
      fprintf(stderr, "could not allocate file buffer.\n");
      return 1;
    }

  strncpy(path, input_path, 4096);

  /* remove any stray trailing slash characters. */

  for (int c = strlen(path) - 1; c >= 0; --c)
    {
      if (path[c] != '/') break;
      path[c] = 0;
    }

  /* Output the defined constants */

  printf("dir=          1\n"
         "file=         2\n"
         "name_length= %d\n",
         max_name_len);

  /* Output the macro definitions and block zero. */

  puts(preamble);

  /* Scan the prototype directory tree. */

  scan_dir(root_sector);

  /* Wrap things up. */

  puts(postamble);

  return 0;
}
