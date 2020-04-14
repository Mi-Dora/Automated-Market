/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   mathematical functions
 */

#ifndef TINY_MATH_H
#define TINY_MATH_H

#include <webots/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double u;
  double v;
} Vector2;

typedef struct {
  double u;
  double v;
  double w;
} Vector3;

typedef struct {
  Vector3 a;
  Vector3 b;
  Vector3 c;
} Matrix33;

// --- Vector3 functions ---
void vector3_set_values(Vector3 *vect, double u, double v, double w);

// --- Matrix33 functions ---
void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,
                         double cw);
void matrix33_set_identity(Matrix33 *m);
void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v);  // res = m * v

// --- Vector2 functions ---
void vector2_set_values(Vector2* vect, double u, double v);
double vector2_norm(const Vector2 *v);                                 // ||v||
void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2);  // v = v1-v2
double vector2_angle(const Vector2 *v1, const Vector2 *v2);            // angle between v1 and v2 -> [0, 2Pi]

// --- Other ---
double bound(double v, double a, double b);


#pragma once

#ifndef _MATH_DEFINES_DEFINED
#define _MATH_DEFINES_DEFINED
// Definitions of useful mathematical constants
//
// Define _USE_MATH_DEFINES before including <math.h> to expose these macro
// definitions for common math constants.  These are placed under an #ifdef
// since these commonly-defined names are not part of the C or C++ standards
#define M_E        2.71828182845904523536   // e
#define M_LOG2E    1.44269504088896340736   // log2(e)
#define M_LOG10E   0.434294481903251827651  // log10(e)
#define M_LN2      0.693147180559945309417  // ln(2)
#define M_LN10     2.30258509299404568402   // ln(10)
#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2
#define M_PI_4     0.785398163397448309616  // pi/4
#define M_1_PI     0.318309886183790671538  // 1/pi
#define M_2_PI     0.636619772367581343076  // 2/pi
#define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi)
#define M_SQRT2    1.41421356237309504880   // sqrt(2)
#define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2)
#endif

#ifdef __cplusplus
}
#endif

#endif
