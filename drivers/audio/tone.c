/****************************************************************************
 * drivers/audio/tone.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * This driver is based on Tone Alarm driver from PX4 project. It was
 * modified to become a NuttX driver and to use the Oneshot Timer API.
 *
 * The PX4 driver is here:
 * https://github.com/PX4/Firmware/blob/master/src/drivers/stm32/tone_alarm/tone_alarm.cpp
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/audio/tone.h>

#include <arch/irq.h>

#ifdef CONFIG_AUDIO_TONE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define tone modes */

#define MODE_NORMAL   1
#define MODE_LEGATO   2
#define MODE_STACCATO 3

/* Max tune string length*/

#define MAX_TUNE_LEN (1 * 256)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct tone_upperhalf_s
{
  uint8_t crefs;                       /* The number of times the device has been
                                        * opened */
#ifdef CONFIG_PWM_MULTICHAN
  uint8_t channel;                     /* Output channel that drives the tone. */
#endif
  volatile bool started;               /* True: pulsed output is being generated */
  sem_t exclsem;                       /* Supports mutual exclusion */
  struct pwm_info_s tone;              /* Pulsed output for Audio Tone */
  struct pwm_lowerhalf_s *devtone;
  struct oneshot_lowerhalf_s *oneshot;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Buffer to store the tune */

static char tune_buf[MAX_TUNE_LEN];

/* Semitone offsets from C for the characters 'A'-'G' */

static const uint8_t g_note_tab[] = { 9, 11, 0, 2, 4, 5, 7 };

/* Notes in Frequency */

static const uint16_t g_notes_freq[84] =
{
  0x0041, 0x0045, 0x0049, 0x004d, 0x0052, 0x0057, 0x005c,
  0x0061, 0x0067, 0x006e, 0x0074, 0x007b, 0x0082, 0x008a,
  0x0092, 0x009b, 0x00a4, 0x00ae, 0x00b8, 0x00c3, 0x00cf,
  0x00dc, 0x00e9, 0x00f6, 0x0105, 0x0115, 0x0125, 0x0137,
  0x0149, 0x015d, 0x0171, 0x0187, 0x019f, 0x01b8, 0x01d2,
  0x01ed, 0x020b, 0x022a, 0x024b, 0x026e, 0x0293, 0x02ba,
  0x02e3, 0x030f, 0x033e, 0x0370, 0x03a4, 0x03db, 0x0416,
  0x0454, 0x0496, 0x04dc, 0x0526, 0x0574, 0x05c7, 0x061f,
  0x067d, 0x06e0, 0x0748, 0x07b7, 0x082d, 0x08a9, 0x092d,
  0x09b9, 0x0a4d, 0x0ae9, 0x0b8f, 0x0c3f, 0x0cfa, 0x0dc0,
  0x0e91, 0x0f6f, 0x105a, 0x1152, 0x125a, 0x1372, 0x149a,
  0x15d3, 0x171f, 0x187f, 0x19f4, 0x1b80, 0x1d22, 0x1ede
};

/* Global variable used by the tone generator */

static const char *g_tune;
static const char *g_next;
static uint8_t g_tempo;
static uint8_t g_note_mode;
static uint32_t g_note_length;
static uint32_t g_silence_length;
static uint8_t g_octave;
static bool g_repeat;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                             FAR void *arg);
static uint32_t note_duration(FAR uint32_t *silence, uint32_t note_length,
                              uint32_t dots);
static uint32_t rest_duration(uint32_t rest_length, uint32_t dots);
static void start_note(FAR struct tone_upperhalf_s *upper, uint8_t note);
static void stop_note(FAR struct tone_upperhalf_s *upper);
static void start_tune(FAR struct tone_upperhalf_s *upper, const char *tune);
static void next_note(FAR struct tone_upperhalf_s *upper);
static int next_char(void);
static uint8_t next_number(void);
static uint8_t next_dots(void);

static int tone_open(FAR struct file *filep);
static int tone_close(FAR struct file *filep);
static ssize_t tone_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t tone_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_toneops =
{
  tone_open,                    /* open */
  tone_close,                   /* close */
  tone_read,                    /* read */
  tone_write,                   /* write */
  0,                            /* seek */
  0                             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0                           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0                           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_callback
 ****************************************************************************/

static void oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                             FAR void *arg)
{
  FAR struct tone_upperhalf_s *upper = (FAR struct tone_upperhalf_s *)arg;

  audinfo("Oneshot timer expired!\n");

  /* Play the next note */

  next_note(upper);
}

/****************************************************************************
 * Name: note_duration
 *
 * Description:
 *   This function calculates the duration in microseconds of play and
 *   silence for a note given the current tempo, length and mode and the
 *   number of dots following in the play string.
 *
 ****************************************************************************/

static uint32_t note_duration(FAR uint32_t *silence, uint32_t note_length,
                              uint32_t dots)
{
  uint32_t whole_note_period = (60 * 1000000 * 4) / g_tempo;
  uint32_t note_period;
  uint32_t dot_extension;

  if (note_length == 0)
    {
      note_length = 1;
    }

  note_period = whole_note_period / note_length;

  switch (g_note_mode)
  {
    case MODE_NORMAL:
      *silence = note_period / 8;
      break;

    case MODE_STACCATO:
      *silence = note_period / 4;
      break;

    case MODE_LEGATO:
      *silence = 0;
      break;

    default:
      auderr("Mode undefined!\n");
      break;
  }

  note_period -= *silence;
  dot_extension = note_period / 2;

  while (dots--)
    {
      note_period += dot_extension;
      dot_extension /= 2;
    }

  return note_period;
}

/****************************************************************************
 * Name: rest_duration
 *
 * Description:
 *   This function calculates the duration in microseconds of a rest
 *   corresponding to a given note length.
 *
 ****************************************************************************/

static uint32_t rest_duration(uint32_t rest_length, uint32_t dots)
{
  uint32_t whole_note_period = (60 * 1000000 * 4) / g_tempo;
  uint32_t rest_period;
  uint32_t dot_extension;

  if (rest_length == 0)
    {
      rest_length = 1;
    }

  rest_period = whole_note_period / rest_length;

  dot_extension = rest_period / 2;

  while (dots--)
    {
      rest_period += dot_extension;
      dot_extension /= 2;
    }

  return rest_period;
}

/****************************************************************************
 * Name: start_note
 ****************************************************************************/

static void start_note(FAR struct tone_upperhalf_s *upper, uint8_t note)
{
  FAR struct pwm_lowerhalf_s *tone = upper->devtone;

  upper->tone.frequency           = g_notes_freq[note - 1];
#ifdef CONFIG_PWM_MULTICHAN
  upper->tone.channels[0].channel = upper->channel;
  upper->tone.channels[0].duty    = b16HALF;
#else
  upper->tone.duty                = b16HALF;
#endif

  /* REVISIT: Should check the return value */

  tone->ops->start(tone, &upper->tone);
}

/****************************************************************************
 * Name: stop_note
 ****************************************************************************/

static void stop_note(FAR struct tone_upperhalf_s *upper)
{
  FAR struct pwm_lowerhalf_s *tone = upper->devtone;

  tone->ops->stop(tone);
}

/****************************************************************************
 * Name: start_tune
 *
 * Description:
 *   This function starts playing the note.
 *
 ****************************************************************************/

static void start_tune(FAR struct tone_upperhalf_s *upper, const char *tune)
{
  FAR struct timespec ts;

  /* Kill any current playback */

  ONESHOT_CANCEL(upper->oneshot, &ts);

  /* Record the tune */

  g_tune           = tune;
  g_next           = tune;

  /* Initialise player state */

  g_tempo          = 120;
  g_note_length    = 4;
  g_note_mode      = MODE_NORMAL;
  g_octave         = 4;
  g_silence_length = 0;
  g_repeat         = false;

  /* Schedule a callback to start playing */

  ts.tv_sec        = 1;
  ts.tv_nsec       = 0;

  ONESHOT_START(upper->oneshot, oneshot_callback, upper, &ts);
}

/****************************************************************************
 * Name: next_note
 *
 * Description:
 *   This function parses the next note out of the string and play it.
 *
 ****************************************************************************/

static void next_note(FAR struct tone_upperhalf_s *upper)
{
  uint32_t note;
  uint32_t note_length;
  uint32_t duration;
  uint32_t sec;
  uint32_t nsec;
  FAR struct timespec ts;

  /* Do we have an inter-note gap to wait for? */

  if (g_silence_length > 0)
    {
      stop_note(upper);

      duration = g_silence_length;

      /* Setup the time duration */

      sec = duration / USEC_PER_SEC;
      nsec = ((duration) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

      ts.tv_sec = (time_t) sec;
      ts.tv_nsec = (unsigned long)nsec;

      ONESHOT_START(upper->oneshot, oneshot_callback, upper, &ts);

      g_silence_length = 0;
      return;
    }

  /* Make sure we still have a tune - may be removed by the write / ioctl
   * handler */

  if ((g_next == NULL) || (g_tune == NULL))
    {
      stop_note(upper);
      return;
    }

  /* Parse characters out of the string until we have resolved a note */

  note = 0;
  note_length = g_note_length;

  while (note == 0)
    {
      /* We always need at least one character from the string */

      int c = next_char();

      if (c == 0)
        {
          goto tune_end;
        }

      g_next++;

      switch (c)
        {
          uint8_t nt;

          /* Select note length */

        case 'L':
          g_note_length = next_number();
          if (g_note_length < 1)
            {
              auderr("note length too short!\n");
              goto tune_error;
            }
          break;

          /* Select octave */

        case 'O':
          g_octave = next_number();
          if (g_octave > 6)
            {
              g_octave = 6;
            }
          break;

          /* Decrease octave */

        case '<':
          if (g_octave > 0)
            {
              g_octave--;
            }
          break;

          /* Increase octave */

        case '>':
          if (g_octave < 6)
            {
              g_octave++;
            }
          break;

          /* Select inter-note gap */

        case 'M':
          c = next_char();

          if (c == 0)
            {
              auderr("no character after M!\n");
              goto tune_error;
            }

          g_next++;

          switch (c)
            {
            case 'N':
              g_note_mode = MODE_NORMAL;
              break;

            case 'L':
              g_note_mode = MODE_LEGATO;
              break;

            case 'S':
              g_note_mode = MODE_STACCATO;
              break;

            case 'F':
              g_repeat = false;
              break;

            case 'B':
              g_repeat = true;
              break;

            default:
              auderr("unknown symbol: %c!\n", c);
              goto tune_error;
              break;
            }

          /* Pause for a note length */

        case 'P':

          stop_note(upper);

          duration = rest_duration(next_number(), next_dots());

          /* Setup the time duration */

          sec = duration / USEC_PER_SEC;
          nsec = ((duration) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

          ts.tv_sec = (time_t) sec;
          ts.tv_nsec = (unsigned long)nsec;

          ONESHOT_START(upper->oneshot, oneshot_callback, upper, &ts);
          return;

          /* Change tempo */

        case 'T':
          nt = next_number();

          if ((nt >= 32) && (nt <= 255))
            {
              g_tempo = nt;
            }
          else
            {
              auderr("T is out of range 32-255!\n");
              goto tune_error;
            }
          break;

          /* Play an arbitrary note */

        case 'N':
          note = next_number();
          if (note > 84)
            {
              auderr("Note higher than 84!\n");
              goto tune_error;
            }

          /* This is a rest - pause for the current note length */

          if (note == 0)
            {
              duration = rest_duration(g_note_length, next_dots());

              /* Setup the time duration */

              sec = duration / USEC_PER_SEC;
              nsec = ((duration) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

              ts.tv_sec = (time_t) sec;
              ts.tv_nsec = (unsigned long)nsec;

              ONESHOT_START(upper->oneshot, oneshot_callback, upper, &ts);

              return;
            }
          break;

          /* Play a note in the current octave */

        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
        case 'G':
          note = g_note_tab[c - 'A'] + (g_octave * 12) + 1;

          c = next_char();

          switch (c)
            {
              /* Up a semitone */

            case '#':
            case '+':
              if (note < 84)
                {
                  note++;
                }

              g_next++;
              break;

              /* Down a semitone */

            case '-':
              if (note > 1)
                {
                  note--;
                }

              g_next++;
              break;

              /* No next char here is OK */

            default:
              break;
            }

          /* Shorthand length notation */

          note_length = next_number();

          if (note_length == 0)
            {
              note_length = g_note_length;
            }

          break;

        default:
          goto tune_error;
        }
    }

  /* Compute the duration of the note and the following silence (if any) */

  duration = note_duration(&g_silence_length, note_length, next_dots());

  /* Start playing the note */

  start_note(upper, note);

  /* Setup time duration */

  sec = duration / USEC_PER_SEC;
  nsec = ((duration) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  ts.tv_sec = (time_t) sec;
  ts.tv_nsec = (unsigned long)nsec;

  /* And arrange a callback when the note should stop */

  ONESHOT_START(upper->oneshot, oneshot_callback, upper, &ts);
  return;

  /* Tune looks bad (unexpected EOF, bad character, etc.) */

tune_error:
  auderr("tune error\n");

  /* Don't loop on error */

  g_repeat = false;

  /* Stop (and potentially restart) the tune */

tune_end:
  stop_note(upper);

  if (g_repeat)
    {
      start_tune(upper, g_tune);
    }
  else
    {
      g_tune = NULL;
    }
}

/****************************************************************************
 * Name: next_char
 *
 * Description:
 *   This function find the next character in the string, discard any
 *   whitespace and return the canonical (uppercase) version.
 *
 ****************************************************************************/

static int next_char(void)
{
  while (isspace(*g_next))
    {
      g_next++;
    }

  return toupper(*g_next);
}

/****************************************************************************
 * Name: next_number
 *
 * Description:
 *   This function extract a number from the string, consuming all the digit
 *   characters.
 *
 ****************************************************************************/

static uint8_t next_number(void)
{
  uint8_t number = 0;
  int c;

  for (;;)
    {
      c = next_char();

      if (!isdigit(c))
        {
          return number;
        }

      g_next++;
      number = (number * 10) + (c - '0');
    }

  return number;
}

/****************************************************************************
 * Name: next_dots
 *
 * Description:
 *   This function consumes dot characters from the string, returning the
 *   number consumed.
 *
 ****************************************************************************/

static uint8_t next_dots(void)
{
  uint8_t dots = 0;

  while (next_char() == '.')
    {
      g_next++;
      dots++;
    }

  return dots;
}

/****************************************************************************
 * Name: tone_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int tone_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct tone_upperhalf_s *upper = inode->i_private;
  uint8_t tmp;
  int ret;

  audinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first time
   * that the driver has been opened for this device, then initialize the
   * device. */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&upper->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: tone_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ****************************************************************************/

static int tone_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct tone_upperhalf_s *upper = inode->i_private;
  int ret;

  audinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver. */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }

  nxsem_post(&upper->exclsem);
  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: tone_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t tone_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: tone_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t tone_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct tone_upperhalf_s *upper = inode->i_private;

  /* We need to receive a string #RRGGBB = 7 bytes */

  if (buffer == NULL)
    {
      /* Well... nothing to do */

      return -EINVAL;
    }

  if (buflen >= MAX_TUNE_LEN)
    {
      /* Too big to it inside internal buffer (with extra NUL terminator) */

      return -EINVAL;
    }

  /* Copy music to internal buffer */

  memcpy(tune_buf, buffer, buflen);

  /* Failsafe NUL terminated string */

  tune_buf[buflen] = '\0';

  /* Let the music play */

  start_tune(upper, tune_buf);

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tone_register
 *
 * Description:
 *   This function binds an instance of a "lower half" PWM driver with
 *   the "upper half" Audio Tone device and registers that device so that can
 *   be used by application code.
 *
 *
 * Input Parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name of PWM driver
 *     as "/dev/tone0".
 *   tone - A pointer to an instance of lower half PWM
 *     drivers for the tone device.  This instance will be bound to the Audio
 *     tone driver and must persists as long as that driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tone_register(FAR const char *path, FAR struct pwm_lowerhalf_s *tone,
#ifdef CONFIG_PWM_MULTICHAN
                  int channel,
#endif
                  FAR struct oneshot_lowerhalf_s *oneshot)
{
  FAR struct tone_upperhalf_s *upper;

  DEBUGASSERT(path != NULL && tone != NULL);

  /* Allocate the upper-half data structure */

  upper =
    (FAR struct tone_upperhalf_s *)kmm_zalloc(sizeof(struct tone_upperhalf_s));

  if (!upper)
    {
      auderr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by
   * kmm_zalloc()).
   */

  nxsem_init(&upper->exclsem, 0, 1);
  upper->devtone = tone;
  upper->oneshot = oneshot;
#ifdef CONFIG_PWM_MULTICHAN
  upper->channel = (uint8_t)channel;
#endif

  /* Register the PWM device */

  audinfo("Registering %s\n", path);
  return register_driver(path, &g_toneops, 0666, upper);
}

#endif /* CONFIG_AUDIO_TONE */
