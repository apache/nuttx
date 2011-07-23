/****************************************************************************
 * tools/bdf-converter.c
 *
 *   Copyright (C) 2011 NX Engineering, S.A., All rights reserved.
 *   Author: Jose Pablo Carballo Gomez <jcarballo@nx-engineering.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

// BDF Specification Version 2.2:
// This version lifts the restriction on line length. In this version, the new
// maximum length of a value of the type string is 65535 characters, and hence
// lines may now be at least this long.

#define BDF_MAX_LINE_LENGTH 65535

/* Ranges of 7-bit and 8-bit fonts */

#define NXFONT_MIN7BIT 33
#define NXFONT_MAX7BIT 126

#define NXFONT_MIN8BIT 161
#define NXFONT_MAX8BIT 255

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure holds information about a glyph */

typedef struct glyphinfo_s
{
  char *name;     /* Name for they glyph */
  int   encoding; /* The Adobe Standard Encoding value */
  int   bb_w;     /* The width of the black pixels in x */
  int   bb_h;     /* The height of the black pixels in y */
  int   bb_x_off; /* X displacement of the lower left corner
                   * of the bitmap from origin 0 */
  int   bb_y_off; /* Y displacement of the lower left corner
                   * of the bitmap from origin 0 */
  uint32_t *bitmap; /* Hexadecimal data for the  */
} glyphinfo_t;

/* This structures provides the metrics for one glyph */

typedef struct nx_fontmetric_s
{
  uint32_t stride   : 2;    /* Width of one font row in bytes */
  uint32_t width    : 6;    /* Width of the font in bits */
  uint32_t height   : 6;    /* Height of the font in rows */
  uint32_t xoffset  : 6;    /* Top, left-hand corner X-offset in pixels */
  uint32_t yoffset  : 6;    /* Top, left-hand corner y-offset in pixels */
  uint32_t unused   : 6;
} nx_fontmetric_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void trimLine(char *line)
{
  char *str;
  str = line;
  char *strEnd;
  for (strEnd = str + strlen(str) - 1;
       strEnd >= str && isspace((int)(*strEnd));
       strEnd--);
  *(strEnd + 1) = 0;
}

/****************************************************************************
 * Name: bdf_parseIntLine
 *
 * Description:
 *   Parses a line containing a BDF property followed by integers. It will
 *   ignore the first token that corresponds to the property name.
 *
 * Input Parameters:
 *   line  - A line with a BDF property followed by integers, i.e.:
 *                  "FONTBOUNDINGBOX 8 13 0 -2"
 *   count - How many integers are specified by the BDF property. In the
 *           example above, count = 4.
 *   info  - A pointer to memory provided by the caller in which to
 *           return the array of integers. For the example above:
 *                info[0] =  8
 *                info[1] = 13
 *                info[2] =  0
 *                info[3] = -2
 *
 ****************************************************************************/
static void bdf_parseintline(char *line, unsigned int count, int *info)
{
  char *str, *token, *saveptr1;
  str = line;
  
  /* Ignore the key */
  
  token = (char *)strtok_r(str, " ", &saveptr1);
  
  while ((token = (char *)strtok_r(NULL, " ", &saveptr1)) && count--)
    {
      *(info++) = atoi(token);
    }
}

static void bdf_printglyphinfo(const glyphinfo_t *ginfo)
{
  printf("NAME     = %s\n", ginfo->name);
  printf("ENCODING = %d\n", ginfo->encoding);
  printf("BB_W     = %d\n", ginfo->bb_w);
  printf("BB_H     = %d\n", ginfo->bb_h);
  printf("BB_X_OFF = %d\n", ginfo->bb_x_off);
  printf("BB_Y_OFF = %d\n", ginfo->bb_y_off);
  int i;
  for (i = 0; i < ginfo->bb_h; i++)
    {
      printf("BITMAP[%d] = %x\n", i, ginfo->bitmap[i]);
    }
}

static void bdf_printnxmetricinfo(const nx_fontmetric_t *info)
{
  printf("STRIDE  = %d\n", info->stride);
  printf("WIDTH   = %d\n", info->width);
  printf("HEIGHT  = %d\n", info->height);
  printf("XOFFSET = %d\n", info->xoffset);
  printf("YOFFSET = %d\n", info->yoffset);
}

static void bdf_getglyphinfo(FILE *file, glyphinfo_t *ginfo)
{
  char line[BDF_MAX_LINE_LENGTH];
  char lineCopy[BDF_MAX_LINE_LENGTH];
  char *str, *token, *saveptr1;
  bool done;
  
  done = false;
  
  while(fgets(line, BDF_MAX_LINE_LENGTH, file) != NULL && !done)
    {
      trimLine(line);
      strcpy(lineCopy, line);
      str = line;
      
      while ((token = (char *)strtok_r(str, " ", &saveptr1)))
        {

          /* ENCODING information */
          
          if(strcmp(token, "ENCODING") == 0)
            {
              token = (char *)strtok_r(NULL, " ", &saveptr1);
              ginfo->encoding = atoi(token);
            }
            
          /* BBX information */
          
          else if(strcmp(token, "BBX") == 0)
            {
              int bbxinfo[4];
              bdf_parseintline(lineCopy, 4, bbxinfo);
              ginfo->bb_w     = bbxinfo[0];
              ginfo->bb_h     = bbxinfo[1];
              ginfo->bb_x_off = bbxinfo[2];
              ginfo->bb_y_off = bbxinfo[3];
              
              /* This is the last BDF property of interest*/
              
              done = true;
            }

          str = NULL;
        }
      
    }
}

static void bdf_getglyphbitmap(FILE *file, glyphinfo_t *ginfo)
{
  char line[BDF_MAX_LINE_LENGTH];
  uint32_t *bitmap;
  bool readingbitmap;
  
  bitmap = ginfo->bitmap;
  readingbitmap = true;
  
  while (readingbitmap)
    {
      if (fgets(line, BDF_MAX_LINE_LENGTH, file) != NULL)
      {
        trimLine(line);
      
        if(strcmp(line, "ENDCHAR") == 0)
          {
            readingbitmap = false;
          }
        else
          {
            char *endptr;
            *bitmap = strtoul(line, &endptr, 16);
            bitmap++;
          }
          
      }
      else
      {
        /* error condition */
        
        readingbitmap = false;
      }
       
    }
}

static void bdf_getstride(glyphinfo_t *ginfo, uint32_t *stride)
{
  *stride = (ginfo->bb_w % 8 == 0) ? ginfo->bb_w / 8 : ginfo->bb_w / 8 + 1 ;
}

static void bdf_printoutput(FILE *out, 
                            glyphinfo_t *ginfo,
                            nx_fontmetric_t *nxmetric)
{

  /* Only interested in the 7 and 8 bit ranges */
  
  if ((ginfo->encoding >= NXFONT_MIN7BIT  &&
       ginfo->encoding <= NXFONT_MAX7BIT) ||
      (ginfo->encoding >= NXFONT_MIN8BIT  &&
       ginfo->encoding <= NXFONT_MAX8BIT))
    {
      
      /* Glyph general info */
      
      fprintf(out, "/* %s (%d) */\n", ginfo->name, ginfo->encoding);
      
      /* Glyph metrics */
      
      fprintf(out,
              "#define NXFONT_METRICS_%d {%d, %d, %d, %d, %d, 0}\n",
              ginfo->encoding,
              nxmetric->stride,
              nxmetric->width,
              nxmetric->height,
              nxmetric->xoffset,
              nxmetric->yoffset);
              
      /* Glyph bitmap */
      
      fprintf(out, "#define NXFONT_BITMAP_%d {", ginfo->encoding);
      int i;
      for (i = 0; i < ginfo->bb_h - 1; i++)
        {
          fprintf(out, "0x%x, ", ginfo->bitmap[i]);
        }
      fprintf(out, "0x%x}\n", ginfo->bitmap[i]);
      
      fprintf(out, "\n");
    }

}

/****************************************************************************
 * Main
 ****************************************************************************/

int main(int argc, char **argv)
{
  FILE *file, *out;
  char line[BDF_MAX_LINE_LENGTH];
  char lineCopy[BDF_MAX_LINE_LENGTH];
  char *str, *token, *saveptr1;
  
  /* FONTBOUNDINGBOX properties*/
  int fbb_x, fbb_y, fbb_x_off, fbb_y_off;
  
  file = fopen("8x13.bdf", "r");
  out  = fopen("out.txt", "w");
  
  if (file == NULL)
    {
      perror("Error opening file");
    }
  else
    {
      while (fgets(line, BDF_MAX_LINE_LENGTH, file) != NULL)
        {
          printf("--\n");
          
          // Save a copy of the line
          
          strcpy(lineCopy,line);
          
          // Clean it
          
          trimLine(line);
          str = line;

          while ((token = (char *)strtok_r(str, " ", &saveptr1)))
            {
            
              /* FONTBOUNDINGBOX - Global font information */
            
              if (strcmp(token, "FONTBOUNDINGBOX") == 0)
                {
                  int fbbinfo[4];
                  bdf_parseintline(lineCopy, 4, fbbinfo);
                  fbb_x     = fbbinfo[0];
                  fbb_y     = fbbinfo[1];
                  fbb_x_off = fbbinfo[2];
                  fbb_y_off = fbbinfo[3];
                }
                
              /* STARTCHAR - Individual glyph information */
                
              if (strcmp(token, "STARTCHAR") == 0)
                {
                  glyphinfo_t ginfo;
                  
                  /* Glyph name */
                  
                  ginfo.name = (char *)strtok_r(NULL, " ", &saveptr1);
                  
                  /* Glyph information:
                  *    ENCODING
                  *    BBX
                  */
                  
                  bdf_getglyphinfo(file, &ginfo);
                  
                  /* Glyph bitmap */
                  
                  ginfo.bitmap = malloc(sizeof(uint32_t) * ginfo.bb_h);
                  bdf_getglyphbitmap(file, &ginfo);
                  
                  bdf_printglyphinfo(&ginfo);
                  
                  /* Convert to nxfonts */
                  
                  nx_fontmetric_t nxmetric;
                  uint32_t stride;
                  bdf_getstride(&ginfo, &stride);
                  nxmetric.stride  = stride;
                  nxmetric.width   = ginfo.bb_w;
                  nxmetric.height  = ginfo.bb_h;
                  nxmetric.xoffset = (-fbb_x_off) + ginfo.bb_x_off;
                  nxmetric.yoffset = fbb_y + fbb_y_off - ginfo.bb_y_off;
                  bdf_printnxmetricinfo(&nxmetric);
                  
                  bdf_printoutput(out, &ginfo, &nxmetric);
                  
                  /* Free memory */
                  
                  free(ginfo.bitmap);
                  
                }
              
              str = NULL;
            }
          
        }
      fclose(file);
      fclose(out);
    }
    
  return EXIT_SUCCESS;
}
