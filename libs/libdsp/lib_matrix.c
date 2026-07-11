/****************************************************************************
 * libs/libdsp/lib_matrix.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * From: Djordje Vulovic's Extended-Kalman-Filter for STM32
 * Licensed as Apache 2.0 to be included on NuttX:
 * https://github.com/DjVul/Extended-Kalman-Filter---STM32/issues/1
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <dsp.h>
#include <math.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: matrix_add
 *
 * Description:
 *   Matrix addition
 *
 *   C (m x n) = A (m x n) + B (m x n)
 *
 *     A   Pointer to the first matrix (m x n)
 *     B   Pointer to the second matrix (m x n)
 *     C   Pointer to the result matrix (m x n)
 *     m   Number of rows
 *     n   Number of columns
 *
 ****************************************************************************/

void matrix_add(const float *A, const float *B, float *C,
                uint8_t m, uint8_t n)
{
  for (uint8_t i = 0; i < m; i++)
    {
      for (uint8_t j = 0; j < n; j++)
        {
          C[i * n + j] = A[i * n + j] + B[i * n + j];
        }
    }
}

/****************************************************************************
 * Name: matrix_sub
 *
 * Description:
 *   Matrix subtraction
 *     C (m x n) = A (m x n) - B (m x n)
 *
 *     A   Pointer to the first matrix (m x n)
 *     B   Pointer to the second matrix (m x n)
 *     C   Pointer to the result matrix (m x n)
 *     m   Number of rows
 *     n   Number of columns
 *
 ****************************************************************************/

void matrix_sub(const float *A, const float *B, float *C,
                uint8_t m, uint8_t n)
{
  for (uint8_t i = 0; i < m; i++)
    {
      for (uint8_t j = 0; j < n; j++)
        {
          C[i * n + j] = A[i * n + j] - B[i * n + j];
        }
    }
}

/****************************************************************************
 * Name: matrix_mul
 *
 * Description:
 *   Matrix multiplication
 *
 *   C (m x n) = A (m x n) * B (m x n)
 *
 *     A   Pointer to the first matrix (m x n)
 *     B   Pointer to the second matrix (m x n)
 *     C   Pointer to the result matrix (m x n)
 *     m   Number of rows
 *     n   Number of columns
 *
 ****************************************************************************/

void matrix_mul(const float *A, const float *B, float *C,
                uint8_t m, uint8_t n, uint8_t p)
{
  for (uint8_t i = 0; i < m; i++)
    {
      for (uint8_t j = 0; j < p; j++)
        {
          C[i * p + j] = 0.0f;

          for (uint8_t k = 0; k < n; k++)
            {
              C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

/****************************************************************************
 * Name: matrix_mul_transpose
 *
 * Description:
 *   Matrix multiplication with a transposed matrix
 *
 *   C (m x n) = A (m x n) * B (p x n)^T
 *
 *     A   Pointer to the first matrix (m x n)
 *     B   Pointer to the second matrix (p x n) - will be transposed
 *     C   Pointer to the result matrix (m x p)
 *     m   Number of rows in matrix A
 *     n   Number of columns in matrices A and B
 *     p   Number of rows in matrix B (becomes the number of columns
 *         in the result)
 *
 ****************************************************************************/

void matrix_mul_transpose(const float *A, const float *B, float *C,
                          uint8_t m, uint8_t n, uint8_t p)
{
  for (uint8_t i = 0; i < m; i++)
    {
      for (uint8_t j = 0; j < p; j++)
        {
          C[i * p + j] = 0.0f;

          for (uint8_t k = 0; k < n; k++)
            {
              /* B^T is B[j][k] */

              C[i * p + j] += A[i * n + k] * B[j * n + k];
            }
        }
    }
}

/****************************************************************************
 * Name: matrix_transpose
 *
 * Description:
 *   Matrix transposed
 *
 *   B (m x n) = A^T (m x n)^T
 *
 *     A   Pointer to the input matrix (m x n)
 *     B   Pointer to the output matrix (n x m)
 *     m   Number of rows in the input matrix
 *     n   Number of columns in the input matrix
 *
 ****************************************************************************/

void matrix_transpose(const float *A, float *B, uint8_t m, uint8_t n)
{
  for (uint8_t i = 0; i < m; i++)
    {
      for (uint8_t j = 0; j < n; j++)
        {
          B[j * m + i] = A[i * n + j];
        }
    }
}

/****************************************************************************
 * Name: matrix_scalar_mul
 *
 * Description:
 *   Matrix-scalar multiplication
 *
 *   B (m x n) = A (m x n) * s
 *
 *     A   Pointer to the input matrix (m x n)
 *     s   Scalar value (float)
 *     B   Pointer to the output matrix (m x n)
 *     m   Number of rows
 *     n   Number of columns
 *
 ****************************************************************************/

void matrix_scalar_mul(const float *A, float s, float *B,
                       uint8_t m, uint8_t n)
{
  uint16_t total = (uint16_t) m * n;
  uint16_t i;

  for (i = 0; i < total; i++)
    {
      B[i] = A[i] * s;
    }
}

/****************************************************************************
 * Name: matrix_copy
 *
 * Description:
 *   Matrix copy
 *
 *   B (m x n) = A (m x n)
 *
 *     A   Pointer to the source matrix (m x n)
 *     B   Pointer to the destination matrix (m x n)
 *     m   Number of rows
 *     n   Number of columns
 *
 ****************************************************************************/

void matrix_copy(const float *A, float *B, uint8_t m, uint8_t n)
{
  uint16_t total = (uint16_t) m * n;

  memcpy(B, A, total * sizeof(float));
}

/****************************************************************************
 * Name: matrix_inv_3x3
 *
 * Description:
 *   3x3 matrix inversion
 *
 *   inv (3x3) = A^-1 (3x3)
 *
 *     A   Input matrix (3x3)
 *     inv Output matrix (3x3) - inverse matrix
 *
 * Returns:
 *   1 if the inversion was successful,
 *   0 if the matrix is singular.
 *
 ****************************************************************************/

uint8_t matrix_inv_3x3(const float A[3][3], float inv[3][3])
{
  float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
            - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
            + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  float inv_det = 1.0f / det;

  /* Check whether the matrix is singular */

  if (fabsf(det) < 1e-10f)
    {
      return 0;
    }

  inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
  inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
  inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;

  inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
  inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
  inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;

  inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
  inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
  inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

  return 1;
}

