/****************************************************************************
 * libs/libdsp/lib_avg.c
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

#include <assert.h>
#include <dsp.h>
#include <string.h>

/* Based on video explanation of Dr. Shane Ross:
 * https://www.youtube.com/watch?v=HCd-leV8OkU
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avg_filter_data_init
 *
 * Description:
 *   Initialize the data struct used to store prev_avg and k parameter.
 *
 * Input Parameters:
 *   data     - pointer to avg_filter_data_s
 *   prev_avg - initial value of prev_avg
 *   k        - initial value of k counter
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void avg_filter_data_init(FAR struct avg_filter_data_s *data,
                          float prev_avg, float k)
{
  LIBDSP_DEBUGASSERT(k > 0.0f)

  data->prev_avg = prev_avg;
  data->k        = k;
}

/****************************************************************************
 * Name: avg_filter
 *
 * Description:
 *   Calculate the recurring average of a signal in the time
 *
 * Input Parameters:
 *   prev_avg - pointer to previous average variable
 *   k - pointer to k counter variable
 *   x - current signal value
 *
 * Returned Value:
 *   Average value
 *
 ****************************************************************************/

float avg_filter(FAR struct avg_filter_data_s *data, float x)
{
  float alpha;
  float avg;

  LIBDSP_DEBUGASSERT(data != NULL);

  alpha = (data->k - 1.0f) / data->k;

  avg = (alpha * data->prev_avg) + ((1.0f - alpha) * x);
  data->k += 1.0f;

  data->prev_avg = avg;

  return avg;
}

