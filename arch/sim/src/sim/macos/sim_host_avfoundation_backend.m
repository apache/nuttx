/****************************************************************************
 * arch/sim/src/sim/macos/sim_host_avfoundation_backend.m
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "../sim_hostvideo.h"

#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WARN(fmt, ...) \
  syslog(LOG_WARNING, "sim_host_video: " fmt "\n", ##__VA_ARGS__)

#define INFO(fmt, ...) \
  syslog(LOG_INFO, "sim_host_video: " fmt "\n", ##__VA_ARGS__)

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
#define DEFAULT_FPS_DENOM 30
#define DEFAULT_FPS_NUMER 1
#define AVF_PIX_FMT_NV12 0x3231564e

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct host_video_dev_s
{
  AVCaptureSession *session;
  AVCaptureDevice *device;
  AVCaptureDeviceInput *input;
  AVCaptureVideoDataOutput *output;
  dispatch_queue_t queue;
  id delegate;
  pthread_mutex_t lock;
  pthread_cond_t cond;
  uint8_t *frame;
  size_t frame_size;
  size_t frame_capacity;
  bool frame_ready;
  bool streaming;
  bool stopping;
  bool sync_ready;
  int last_error;
  uint16_t width;
  uint16_t height;
  uint32_t fmt;
  uint32_t denom;
  uint32_t numer;
};

@interface SimAvfCaptureDelegate : NSObject<AVCaptureVideoDataOutputSampleBufferDelegate>
{
@public
  struct host_video_dev_s *_dev;
}

- (instancetype)initWithDevice:(struct host_video_dev_s *)dev;
- (void)captureOutput:(AVCaptureOutput *)output
    didDropSampleBuffer:(CMSampleBufferRef)sampleBuffer
           fromConnection:(AVCaptureConnection *)connection;
@end

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool avf_is_nv12(uint32_t fmt)
{
  return fmt == AVF_PIX_FMT_NV12;
}

static size_t avf_frame_size(uint16_t width, uint16_t height, uint32_t fmt)
{
  if (avf_is_nv12(fmt))
    {
      return (size_t)width * (size_t)height * 3 / 2;
    }

  return 0;
}

static int avf_ensure_frame_buffer(struct host_video_dev_s *dev, size_t size)
{
  uint8_t *frame;

  if (size == 0)
    {
      return -EINVAL;
    }

  if (dev->frame_capacity >= size)
    {
      return 0;
    }

  frame = realloc(dev->frame, size);
  if (frame == NULL)
    {
      return -ENOMEM;
    }

  dev->frame = frame;
  dev->frame_capacity = size;
  return 0;
}

static int avf_report_open_error(int err, const char *message)
{
  fprintf(stderr, "%s\n", message);
  return err;
}

static NSArray<AVCaptureDevice *> *avf_discover_devices(void)
{
  AVCaptureDeviceDiscoverySession *discovery;
  NSArray<AVCaptureDevice *> *devices;
  NSMutableArray<AVCaptureDevice *> *ordered;
  AVCaptureDevice *device;

  discovery = [AVCaptureDeviceDiscoverySession
    discoverySessionWithDeviceTypes:@[
      AVCaptureDeviceTypeBuiltInWideAngleCamera,
#if defined(__MAC_14_0)
      AVCaptureDeviceTypeExternal,
#else
      AVCaptureDeviceTypeExternalUnknown,
#endif
      AVCaptureDeviceTypeContinuityCamera
    ]
    mediaType:AVMediaTypeVideo
    position:AVCaptureDevicePositionUnspecified];
  devices = discovery.devices;

  if (devices == nil || [devices count] == 0)
    {
      return devices;
    }

  ordered = [NSMutableArray arrayWithCapacity:[devices count]];
  if (ordered == nil)
    {
      return devices;
    }

  for (device in devices)
    {
      if ([device.deviceType isEqualToString:
          AVCaptureDeviceTypeBuiltInWideAngleCamera])
        {
          [ordered addObject:device];
        }
    }

  for (device in devices)
    {
      if (![device.deviceType isEqualToString:
           AVCaptureDeviceTypeBuiltInWideAngleCamera])
        {
          [ordered addObject:device];
        }
    }

  return ordered;
}

static bool avf_parse_device_index(const char *dev_path, unsigned long *index_out)
{
  const char *cursor;
  char *endptr;
  unsigned long index;

  if (dev_path == NULL || index_out == NULL)
    {
      return false;
    }

  cursor = strrchr(dev_path, '/');
  cursor = cursor != NULL ? cursor + 1 : dev_path;

  while (*cursor != '\0' && (*cursor < '0' || *cursor > '9'))
    {
      cursor++;
    }

  if (*cursor == '\0')
    {
      return false;
    }

  errno = 0;
  index = strtoul(cursor, &endptr, 10);
  if (errno != 0 || endptr == cursor || *endptr != '\0')
    {
      return false;
    }

  *index_out = index;
  return true;
}

static int avf_find_device(const char *dev_path, AVCaptureDevice **device_out)
{
  NSString *path;
  NSArray<AVCaptureDevice *> *devices;
  AVCaptureDevice *device;
  AVAuthorizationStatus auth_status;
  unsigned long index;
  NSUInteger count;

  if (device_out == NULL)
    {
      return -EINVAL;
    }

  *device_out = nil;

  auth_status = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
  switch (auth_status)
    {
      case AVAuthorizationStatusAuthorized:
        break;

      case AVAuthorizationStatusNotDetermined:
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                                 completionHandler:^(BOOL granted)
        {
          if (!granted)
            {
              WARN("camera authorization denied after request; grant Camera permission to Terminal/iTerm2 in System Settings > Privacy & Security > Camera");
            }
        }];
        break;

      case AVAuthorizationStatusDenied:
        WARN("camera authorization denied; grant Camera permission to Terminal/iTerm2 in System Settings > Privacy & Security > Camera");
        break;

      case AVAuthorizationStatusRestricted:
        WARN("camera authorization restricted by system policy");
        break;
    }

  devices = avf_discover_devices();
  count = devices != nil ? [devices count] : 0;
  if (count == 0)
    {
      if (auth_status == AVAuthorizationStatusDenied ||
          auth_status == AVAuthorizationStatusRestricted)
        {
          return avf_report_open_error(-EACCES,
            "ERROR: Camera access denied for the hosting terminal app. Grant Camera permission to Terminal/iTerm2 in System Settings > Privacy & Security > Camera, then retry.");
        }

      return avf_report_open_error(-ENODEV,
        "ERROR: No camera device found. If a camera is present, ensure the hosting terminal app has Camera permission in System Settings > Privacy & Security > Camera.");
    }

  INFO("resolved %lu camera device(s) for request '%s'",
       (unsigned long)count,
       dev_path != NULL && dev_path[0] != '\0' ? dev_path : "<default>");

  if (dev_path == NULL || dev_path[0] == '\0')
    {
      *device_out = devices.firstObject;
      return 0;
    }

  if (avf_parse_device_index(dev_path, &index))
    {
      INFO("camera device request parsed as index %lu", index);
      if (index < count)
        {
          *device_out = [devices objectAtIndex:index];
          return 0;
        }

      WARN("requested camera index %lu is out of range (available: %lu)",
           index, (unsigned long)count);
      return avf_report_open_error(-ENODEV,
        "ERROR: Requested camera index is out of range for the available device list.");
    }

  path = [NSString stringWithUTF8String:dev_path];
  if (path == nil)
    {
      return avf_report_open_error(-EINVAL,
        "ERROR: Invalid camera device parameter.");
    }

  for (device in devices)
    {
      if ((device.uniqueID != nil && [device.uniqueID isEqualToString:path]) ||
          (device.localizedName != nil &&
           [device.localizedName isEqualToString:path]))
        {
          *device_out = device;
          return 0;
        }
    }

  return avf_report_open_error(-ENODEV,
    "ERROR: Requested camera device was not found. If a camera is present, ensure the hosting terminal app has Camera permission in System Settings > Privacy & Security > Camera.");
}

static int avf_apply_format_locked(struct host_video_dev_s *dev)
{
  NSError *error = nil;
  AVCaptureDeviceFormat *selected = nil;
  AVCaptureDeviceFormat *fallback = nil;
  FourCharCode pixel = kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;
  int ret;

  ret = avf_ensure_frame_buffer(dev,
                                avf_frame_size(dev->width,
                                               dev->height,
                                               dev->fmt));
  if (ret < 0)
    {
      return ret;
    }

  if (![dev->device lockForConfiguration:&error])
    {
      WARN("lockForConfiguration failed: %s",
           error.localizedDescription.UTF8String);
      return -EIO;
    }

  for (AVCaptureDeviceFormat *format in dev->device.formats)
    {
      CMFormatDescriptionRef desc = format.formatDescription;
      FourCharCode media_subtype = CMFormatDescriptionGetMediaSubType(desc);
      CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(desc);

      if (media_subtype != pixel)
        {
          continue;
        }

      if (fallback == nil)
        {
          fallback = format;
        }

      if (dims.width == dev->width && dims.height == dev->height)
        {
          selected = format;
          break;
        }
    }

  if (selected == nil)
    {
      selected = fallback;
    }

  if (selected == nil)
    {
      [dev->device unlockForConfiguration];
      WARN("no NV12 capture format available");
      return -ENOTSUP;
    }

  dev->device.activeFormat = selected;

  if (dev->denom != 0 && dev->numer != 0)
    {
      CMTime frame_duration = CMTimeMake(dev->numer, dev->denom);
      BOOL frame_duration_supported = NO;

      for (AVFrameRateRange *range in selected.videoSupportedFrameRateRanges)
        {
          if (CMTIME_IS_VALID(range.minFrameDuration) &&
              CMTIME_IS_VALID(range.maxFrameDuration) &&
              CMTIME_COMPARE_INLINE(frame_duration, >=,
                                    range.minFrameDuration) &&
              CMTIME_COMPARE_INLINE(frame_duration, <=,
                                    range.maxFrameDuration))
            {
              frame_duration_supported = YES;
              break;
            }
        }

      if (frame_duration_supported)
        {
          dev->device.activeVideoMinFrameDuration = frame_duration;
          dev->device.activeVideoMaxFrameDuration = frame_duration;
        }
    }

  [dev->device unlockForConfiguration];

  [dev->output setVideoSettings:@{
    (id)kCVPixelBufferPixelFormatTypeKey : @(pixel),
    (id)kCVPixelBufferWidthKey : @(dev->width),
    (id)kCVPixelBufferHeightKey : @(dev->height)
  }];

  return 0;
}

@implementation SimAvfCaptureDelegate

- (instancetype)initWithDevice:(struct host_video_dev_s *)dev
{
  self = [super init];
  if (self != nil)
    {
      _dev = dev;
    }

  return self;
}

- (void)captureOutput:(AVCaptureOutput *)output
    didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
           fromConnection:(AVCaptureConnection *)connection
{
  CVImageBufferRef image_buffer;
  CVPixelBufferRef pixel_buffer;
  OSType pixel_format;
  size_t width;
  size_t height;
  size_t y_stride;
  size_t uv_stride;
  size_t y_size;
  size_t uv_size;
  size_t frame_size;
  uint8_t *dst;
  uint8_t *src_y;
  uint8_t *src_uv;
  int ret;

  (void)output;
  (void)connection;

  image_buffer = CMSampleBufferGetImageBuffer(sampleBuffer);
  if (image_buffer == NULL)
    {
      return;
    }

  pixel_buffer = (CVPixelBufferRef)image_buffer;
  pixel_format = CVPixelBufferGetPixelFormatType(pixel_buffer);
  if (pixel_format != kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange &&
      pixel_format != kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)
    {
      return;
    }

  CVPixelBufferLockBaseAddress(pixel_buffer, kCVPixelBufferLock_ReadOnly);

  width = CVPixelBufferGetWidth(pixel_buffer);
  height = CVPixelBufferGetHeight(pixel_buffer);
  y_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 0);
  uv_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 1);
  y_size = width * height;
  uv_size = width * height / 2;
  frame_size = y_size + uv_size;

  pthread_mutex_lock(&_dev->lock);
  ret = avf_ensure_frame_buffer(_dev, frame_size);
  if (ret == 0)
    {
      dst = _dev->frame;
      src_y = CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 0);
      src_uv = CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 1);

      for (size_t row = 0; row < height; row++)
        {
          memcpy(dst + row * width, src_y + row * y_stride, width);
        }

      dst += y_size;
      for (size_t row = 0; row < height / 2; row++)
        {
          memcpy(dst + row * width, src_uv + row * uv_stride, width);
        }

      _dev->frame_size = frame_size;
      _dev->frame_ready = true;
      pthread_cond_broadcast(&_dev->cond);
    }
  pthread_mutex_unlock(&_dev->lock);

  pthread_mutex_lock(&_dev->lock);
  if (!_dev->sync_ready)
    {
      _dev->sync_ready = true;
      _dev->last_error = 0;
      pthread_cond_broadcast(&_dev->cond);
    }
  pthread_mutex_unlock(&_dev->lock);

  CVPixelBufferUnlockBaseAddress(pixel_buffer, kCVPixelBufferLock_ReadOnly);
}

- (void)captureOutput:(AVCaptureOutput *)output
    didDropSampleBuffer:(CMSampleBufferRef)sampleBuffer
           fromConnection:(AVCaptureConnection *)connection
{
  (void)output;
  (void)sampleBuffer;
  (void)connection;

  pthread_mutex_lock(&_dev->lock);
  if (!_dev->sync_ready)
    {
      _dev->sync_ready = true;
      _dev->last_error = -EIO;
      pthread_cond_broadcast(&_dev->cond);
    }
  pthread_mutex_unlock(&_dev->lock);
}

@end

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_video_get_device_count(void)
{
  NSArray<AVCaptureDevice *> *devices;
  NSUInteger count;

  devices = avf_discover_devices();
  count = devices != nil ? [devices count] : 0;

  return count > 0 ? (int)count : -ENODEV;
}

bool host_video_is_available(const char *host_video_dev_path)
{
  AVCaptureDevice *device;

  return avf_find_device(host_video_dev_path, &device) == 0 && device != nil;
}

struct host_video_dev_s *host_video_init(const char *host_video_dev_path)
{
  struct host_video_dev_s *dev;
  NSError *error = nil;
  int ret;

  dev = calloc(1, sizeof(*dev));
  if (dev == NULL)
    {
      perror("host_video_init failed");
      return NULL;
    }

  ret = avf_find_device(host_video_dev_path, &dev->device);
  if (ret < 0 || dev->device == nil)
    {
      if (ret >= 0)
        {
          ret = -ENODEV;
        }

      free(dev);
      errno = -ret;
      return NULL;
    }

  dev->width = DEFAULT_WIDTH;
  dev->height = DEFAULT_HEIGHT;
  dev->fmt = AVF_PIX_FMT_NV12;
  dev->denom = DEFAULT_FPS_DENOM;
  dev->numer = DEFAULT_FPS_NUMER;

  pthread_mutex_init(&dev->lock, NULL);
  pthread_cond_init(&dev->cond, NULL);

  dev->session = [[AVCaptureSession alloc] init];
  if (dev->session == nil)
    {
      WARN("failed to create capture session");
      host_video_uninit(dev);
      errno = ENOMEM;
      return NULL;
    }

  dev->input = [AVCaptureDeviceInput deviceInputWithDevice:dev->device
                                                     error:&error];
  if (dev->input == nil || error != nil)
    {
      AVAuthorizationStatus auth_status =
        [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
      const char *reason = error != nil ? error.localizedDescription.UTF8String :
                           "camera access denied or device unavailable";
      WARN("deviceInputWithDevice failed: %s", reason);
      if (auth_status == AVAuthorizationStatusDenied ||
          auth_status == AVAuthorizationStatusRestricted ||
          (error != nil &&
           error.code == AVErrorApplicationIsNotAuthorizedToUseDevice))
        {
          fprintf(stderr,
                  "ERROR: Camera access denied for the hosting terminal app. Grant Camera permission to Terminal/iTerm2 in System Settings > Privacy & Security > Camera, then retry.\n");
          host_video_uninit(dev);
          errno = EACCES;
          return NULL;
        }

      fprintf(stderr,
              "ERROR: Failed to open camera device input: %s\n",
              reason);
      host_video_uninit(dev);
      errno = EIO;
      return NULL;
    }

  if ([dev->session canAddInput:dev->input])
    {
      [dev->session addInput:dev->input];
    }
  else
    {
      WARN("cannot add camera input to capture session");
      host_video_uninit(dev);
      return NULL;
    }

  dev->output = [[AVCaptureVideoDataOutput alloc] init];
  if (dev->output == nil)
    {
      WARN("failed to create video data output");
      host_video_uninit(dev);
      errno = ENOMEM;
      return NULL;
    }

  dev->output.alwaysDiscardsLateVideoFrames = YES;
  if ([dev->session canAddOutput:dev->output])
    {
      [dev->session addOutput:dev->output];
    }
  else
    {
      WARN("cannot add camera output to capture session");
      host_video_uninit(dev);
      return NULL;
    }

  dev->queue = dispatch_queue_create("org.apache.nuttx.sim.camera",
                                     DISPATCH_QUEUE_SERIAL);
  if (dev->queue == NULL)
    {
      WARN("failed to create capture dispatch queue");
      host_video_uninit(dev);
      errno = ENOMEM;
      return NULL;
    }

  dev->delegate = [[SimAvfCaptureDelegate alloc] initWithDevice:dev];
  if (dev->delegate == nil)
    {
      WARN("failed to create capture delegate");
      host_video_uninit(dev);
      errno = ENOMEM;
      return NULL;
    }

  [dev->output setSampleBufferDelegate:dev->delegate queue:dev->queue];

  ret = avf_apply_format_locked(dev);
  if (ret < 0)
    {
      host_video_uninit(dev);
      errno = -ret;
      return NULL;
    }

  INFO("using macOS camera '%s'", dev->device.localizedName.UTF8String);
  return dev;
}

int host_video_uninit(struct host_video_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  if (dev != NULL)
    {
      host_video_stop_capture(dev);
      if (dev->output != nil)
        {
          [dev->output setSampleBufferDelegate:nil queue:NULL];
        }

      dev->delegate = nil;
      dev->output = nil;
      dev->input = nil;
      dev->session = nil;
      dev->device = nil;
      dev->queue = NULL;
      free(dev->frame);
      pthread_cond_destroy(&dev->cond);
      pthread_mutex_destroy(&dev->lock);
      free(dev);
    }

  return 0;
}

int host_video_start_capture(struct host_video_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&dev->lock);
  if (dev->streaming)
    {
      pthread_mutex_unlock(&dev->lock);
      return -EBUSY;
    }

  dev->frame_ready = false;
  dev->stopping = false;
  dev->sync_ready = false;
  dev->last_error = 0;
  pthread_mutex_unlock(&dev->lock);

  [dev->session startRunning];

  pthread_mutex_lock(&dev->lock);
  if (!dev->sync_ready)
    {
      dev->sync_ready = true;
      dev->last_error = [dev->session isRunning] ? 0 : -EIO;
      pthread_cond_broadcast(&dev->cond);
    }

  if (dev->last_error < 0)
    {
      int ret = dev->last_error;
      pthread_mutex_unlock(&dev->lock);
      return ret;
    }

  dev->streaming = true;
  pthread_mutex_unlock(&dev->lock);
  return 0;
}

int host_video_stop_capture(struct host_video_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&dev->lock);
  if (!dev->streaming)
    {
      pthread_mutex_unlock(&dev->lock);
      return 0;
    }

  dev->stopping = true;
  dev->frame_ready = false;
  pthread_cond_broadcast(&dev->cond);
  pthread_mutex_unlock(&dev->lock);

  [dev->session stopRunning];

  pthread_mutex_lock(&dev->lock);
  dev->streaming = false;
  pthread_mutex_unlock(&dev->lock);
  return 0;
}

int host_video_dqbuf(struct host_video_dev_s *dev, uint8_t *dst,
                     uint32_t dst_size)
{
  size_t copy_size;

  if (dev == NULL || dst == NULL || dst_size == 0)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&dev->lock);
  if (!dev->streaming || !dev->frame_ready)
    {
      pthread_mutex_unlock(&dev->lock);
      return 0;
    }

  copy_size = dev->frame_size;
  if (copy_size > dst_size)
    {
      copy_size = dst_size;
    }

  memcpy(dst, dev->frame, copy_size);
  dev->frame_ready = false;
  pthread_mutex_unlock(&dev->lock);
  return (int)copy_size;
}

int host_video_set_fmt(struct host_video_dev_s *dev,
                       uint16_t width, uint16_t height, uint32_t pixfmt,
                       uint32_t fps_denom, uint32_t fps_numer)
{
  int ret;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  if (!avf_is_nv12(pixfmt))
    {
      WARN("only NV12 is supported on macOS backend");
      return -ENOTSUP;
    }

  pthread_mutex_lock(&dev->lock);
  if (dev->streaming)
    {
      pthread_mutex_unlock(&dev->lock);
      return -EBUSY;
    }

  dev->width = width;
  dev->height = height;
  dev->fmt = pixfmt;
  dev->denom = fps_denom ? fps_denom : DEFAULT_FPS_DENOM;
  dev->numer = fps_numer ? fps_numer : DEFAULT_FPS_NUMER;
  ret = avf_apply_format_locked(dev);
  pthread_mutex_unlock(&dev->lock);
  return ret;
}

int host_video_try_fmt(struct host_video_dev_s *dev,
                       uint16_t width, uint16_t height, uint32_t pixfmt,
                       uint32_t fps_denom, uint32_t fps_numer)
{
  AVCaptureDeviceFormat *format;
  FourCharCode pixel = kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;

  (void)fps_denom;
  (void)fps_numer;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  if (!avf_is_nv12(pixfmt))
    {
      WARN("only NV12 is supported on macOS backend");
      return -ENOTSUP;
    }

  for (format in dev->device.formats)
    {
      CMFormatDescriptionRef desc = format.formatDescription;
      CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(desc);

      if (CMFormatDescriptionGetMediaSubType(desc) == pixel &&
          dims.width == width && dims.height == height)
        {
          return 0;
        }
    }

  WARN("requested format %ux%u NV12 is not supported", width, height);
  return -EINVAL;
}
