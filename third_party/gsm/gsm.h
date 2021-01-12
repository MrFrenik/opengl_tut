/*==============================================================================================================
    * Copyright: 2020 John Jackson 
    * glm: Single-header Math utility library from gunslinger
    * File: gsm.h
    * Github: https://github.com/MrFrenik/gsm

    All Rights Reserved

    BSD 3-Clause License

    Copyright (c) 2020 John Jackson

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be used to 
    endorse or promote products derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIEDi
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=================================================================================================================*/

#ifndef __GSM_H__
#define __GSM_H__

#include <assert.h>     // assert
#include <stdarg.h>     // valist
#include <stdint.h>     // standard types
#include <math.h>       // floor, acos, sin, sqrt, tan

/*========================
// Useful Defines
========================*/

#ifndef gs_inline
    #define gs_inline static inline
#endif

#ifndef gs_force_inline
     #if (defined _WIN32 || defined _WIN64)
        #define gs_force_inline gs_inline
    #elif (defined __APPLE__ || defined _APPLE)
        #define gs_force_inline static __attribute__((always_inline))
    #else
        #define gs_force_inline gs_inline
    #endif
#endif

#if defined (__cplusplus)
    #define gsm_defval() {}
#else
    #define gsm_defval() {0}
#endif

/*========================
// GS_MATH
========================*/

// Defines
#define GS_PI       3.1415926535897932
#define GS_TAU      2.0 * GS_PI

// Useful Utility
#define gs_v2(...)  gs_vec2_ctor(__VA_ARGS__)
#define gs_v3(...)  gs_vec3_ctor(__VA_ARGS__)
#define gs_v4(...)  gs_vec4_ctor(__VA_ARGS__)

#define gs_v2s(__S)  gs_vec2_ctor((__S), (__S))
#define gs_v3s(__S)  gs_vec3_ctor((__S), (__S), (__S))
#define gs_v4s(__S)  gs_vec4_ctor((__S), (__S), (__S), (__S))

#define gs_v4_xy_v(__X, __Y, __V) gs_vec4_ctor((__X), (__Y), (__V).x, (__V).y)

#define GS_XAXIS    gs_v3(1.f, 0.f, 0.f)
#define GS_YAXIS    gs_v3(0.f, 1.f, 0.f)
#define GS_ZAXIS    gs_v3(0.f, 0.f, 1.f)

/*================================================================================
// Useful Common Math Functions
================================================================================*/

#define gs_rad2deg(__R)\
    (float)((__R * 180.0.f) / GS_PI) 

#define gs_deg2rad(__D)\
    (float)((__D * GS_PI) / 180.0f)

// Interpolation
// Source: https://codeplea.com/simple-interpolation

gs_inline float
gs_interp_linear(float a, float b, float t)
{
    return (a + t * (b - a));
}

gs_inline float
gs_interp_smooth_step(float a, float b, float t)
{
    return gs_interp_linear(a, b, t * t * (3.0f - 2.0f * t));
}

gs_inline float 
gs_interp_cosine(float a, float b, float t)
{
    return gs_interp_linear(a, b, (float)-cos(GS_PI * t) * 0.5f + 0.5f);
}

gs_inline float 
gs_interp_acceleration(float a, float b, float t) 
{
    return gs_interp_linear(a, b, t * t);
}

gs_inline float 
gs_interp_deceleration(float a, float b, float t) 
{
    return gs_interp_linear(a, b, 1.0f - (1.0f - t) * (1.0f - t));
}

gs_inline float 
gs_round(float val) 
{
    return (float)floor(val + 0.5f);
}

gs_inline float
gs_map_range(float input_start, float input_end, float output_start, float output_end, float val)
{
    float slope = (output_end - output_start) / (input_end - input_start);
    return (output_start + (slope * (val - input_start)));
}

// Easings from: https://github.com/raysan5/raylib/blob/ea0f6c7a26f3a61f3be542aa8f066ce033766a9f/examples/others/easings.h
gs_inline
float gs_ease_cubic_in(float t, float b, float c, float d) 
{ 
    t /= d; 
    return (c*t*t*t + b); 
}

gs_inline
float gs_ease_cubic_out(float t, float b, float c, float d) 
{ 
    t = t/d - 1.0f; 
    return (c*(t*t*t + 1.0f) + b); 
}

gs_inline
float gs_ease_cubic_in_out(float t, float b, float c, float d)
{
    if ((t/=d/2.0f) < 1.0f) 
    {
        return (c/2.0f*t*t*t + b);
    }
    t -= 2.0f; 
    return (c/2.0f*(t*t*t + 2.0f) + b);
}

/*================================================================================
// Vec2
================================================================================*/

typedef struct 
{
    union 
    {
        float xy[2];
        struct 
        {
            float x, y;
        };
    };
} gs_vec2;

gs_inline gs_vec2 
gs_vec2_ctor(float _x, float _y) 
{
    gs_vec2 v;
    v.x = _x;
    v.y = _y;
    return v;
}

gs_inline gs_vec2 
gs_vec2_add(gs_vec2 v0, gs_vec2 v1) 
{
    return gs_vec2_ctor(v0.x + v1.x, v0.y + v1.y);
}

gs_inline gs_vec2 
gs_vec2_sub(gs_vec2 v0, gs_vec2 v1)
{
    return gs_vec2_ctor(v0.x - v1.x, v0.y - v1.y);
}

gs_inline gs_vec2 
gs_vec2_mul(gs_vec2 v0, gs_vec2 v1) 
{
    return gs_vec2_ctor(v0.x * v1.x, v0.y * v1.y);
}

gs_inline gs_vec2 
gs_vec2_div(gs_vec2 v0, gs_vec2 v1) 
{
    return gs_vec2_ctor(v0.x / v1.x, v0.y / v1.y);
}

gs_inline gs_vec2 
gs_vec2_scale(gs_vec2 v, float s)
{
    return gs_vec2_ctor(v.x * s, v.y * s);
}

gs_inline float 
gs_vec2_dot(gs_vec2 v0, gs_vec2 v1) 
{
    return (float)(v0.x * v1.x + v0.y * v1.y);
}

gs_inline float 
gs_vec2_len(gs_vec2 v)
{
    return (float)sqrt(gs_vec2_dot(v, v));
}

gs_inline gs_vec2
gs_vec2_project_onto(gs_vec2 v0, gs_vec2 v1)
{
    float dot = gs_vec2_dot(v0, v1);
    float len = gs_vec2_dot(v1, v1);

    // Orthogonal, so return v1
    if (len == 0.f) return v1;

    return gs_vec2_scale(v1, dot / len);
}

gs_inline gs_vec2 gs_vec2_norm(gs_vec2 v) 
{
    float len = gs_vec2_len(v);
    return gs_vec2_scale(v, len != 0.f ? 1.0f / gs_vec2_len(v) : 1.f);
}

gs_inline 
float gs_vec2_dist(gs_vec2 a, gs_vec2 b)
{
    float dx = (a.x - b.x);
    float dy = (a.y - b.y);
    return (float)(sqrt(dx * dx + dy * dy));
}

gs_inline
float gs_vec2_cross(gs_vec2 a, gs_vec2 b) 
{
    return a.x * b.y - a.y * b.x;
}

gs_inline
float gs_vec2_angle(gs_vec2 a, gs_vec2 b) 
{
    return (float)acos(gs_vec2_dot(a, b) / (gs_vec2_len(a) * gs_vec2_len(b)));
}

gs_inline
uint32_t gs_vec2_equal(gs_vec2 a, gs_vec2 b)
{
    return (a.x == b.x && a.y == b.y);
}

/*================================================================================
// Vec3
================================================================================*/

typedef struct
{
    union 
    {
        float xyz[3];
        struct 
        {
            float x, y, z;
        };
    };
} gs_vec3;

gs_inline gs_vec3 
gs_vec3_ctor(float _x, float _y, float _z)
{
    gs_vec3 v;
    v.x = _x;
    v.y = _y;
    v.z = _z;
    return v;
}

gs_inline gs_vec3 
gs_vec3_add(gs_vec3 v0, gs_vec3 v1)
{
    return gs_vec3_ctor(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}

gs_inline gs_vec3 
gs_vec3_sub(gs_vec3 v0, gs_vec3 v1) 
{
    return gs_vec3_ctor(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}

gs_inline gs_vec3 
gs_vec3_mul(gs_vec3 v0, gs_vec3 v1) 
{
    return gs_vec3_ctor(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z);
}

gs_inline gs_vec3 
gs_vec3_div(gs_vec3 v0, gs_vec3 v1) 
{
    return gs_vec3_ctor(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z);
}

gs_inline gs_vec3 
gs_vec3_scale(gs_vec3 v, float s) 
{
    return gs_vec3_ctor(v.x * s, v.y * s, v.z * s);
}

gs_inline float 
gs_vec3_dot(gs_vec3 v0, gs_vec3 v1) 
{
    float dot = (float)((v0.x * v1.x) + (v0.y * v1.y) + v0.z * v1.z);
    return dot;
}

gs_inline float 
gs_vec3_len(gs_vec3 v)
{
    return (float)sqrt(gs_vec3_dot(v, v));
}

gs_inline gs_vec3
gs_vec3_project_onto(gs_vec3 v0, gs_vec3 v1)
{
    float dot = gs_vec3_dot(v0, v1);
    float len = gs_vec3_dot(v1, v1);

    // Orthogonal, so return v1
    if (len == 0.f) return v1;

    return gs_vec3_scale(v1, dot / len);
}

gs_inline 
float gs_vec3_dist(gs_vec3 a, gs_vec3 b)
{
    float dx = (a.x - b.x);
    float dy = (a.y - b.y);
    float dz = (a.z - b.z);
    return (float)(sqrt(dx * dx + dy * dy + dz * dz));
}

gs_inline gs_vec3 
gs_vec3_norm(gs_vec3 v)
{
    float len = gs_vec3_len(v);
    return len == 0.f ? v : gs_vec3_scale(v, 1.f / len);
}

gs_inline gs_vec3 
gs_vec3_cross(gs_vec3 v0, gs_vec3 v1) 
{
    return gs_vec3_ctor
    (
        v0.y * v1.z - v0.z * v1.y,
        v0.z * v1.x - v0.x * v1.z,
        v0.x * v1.y - v0.y * v1.x
    );
}

gs_inline void gs_vec3_scale_ip(gs_vec3* vp, float s)
{
    vp->x *= s;
    vp->y *= s;
    vp->z *= s;
}

/*================================================================================
// Vec4
================================================================================*/

typedef struct
{
    union 
    {
        float xyzw[4];
        struct 
        {
            float x, y, z, w;
        };
    };
} gs_vec4;

gs_inline gs_vec4 
gs_vec4_ctor(float _x, float _y, float _z, float _w)
{
    gs_vec4 v; 
    v.x = _x;
    v.y = _y; 
    v.z = _z; 
    v.w = _w;
    return v;
} 

gs_inline gs_vec4
gs_vec4_add(gs_vec4 v0, gs_vec4 v1) 
{
    return gs_vec4_ctor(v0.x + v1.y, v0.y + v1.y, v0.z + v1.z, v0.w + v1.w);
}

gs_inline gs_vec4
gs_vec4_sub(gs_vec4 v0, gs_vec4 v1) 
{
    return gs_vec4_ctor(v0.x - v1.y, v0.y - v1.y, v0.z - v1.z, v0.w - v1.w);
}

gs_inline gs_vec4
gs_vec4_div(gs_vec4 v0, gs_vec4 v1) 
{
    return gs_vec4_ctor(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z, v0.w / v1.w);
}

gs_inline gs_vec4
gs_vec4_scale(gs_vec4 v, float s) 
{
    return gs_vec4_ctor(v.x / s, v.y / s, v.z / s, v.w / s);
}

gs_inline float
gs_vec4_dot(gs_vec4 v0, gs_vec4 v1) 
{
    return (float)(v0.x * v1.x + v0.y * v1.y + v0.z * v1.z + v0.w * v1.w);
}

gs_inline float
gs_vec4_len(gs_vec4 v) 
{
    return (float)sqrt(gs_vec4_dot(v, v));
}

gs_inline gs_vec4
gs_vec4_project_onto(gs_vec4 v0, gs_vec4 v1)
{
    float dot = gs_vec4_dot(v0, v1);
    float len = gs_vec4_dot(v1, v1);

    // Orthogonal, so return v1
    if (len == 0.f) return v1;

    return gs_vec4_scale(v1, dot / len);
}

gs_inline gs_vec4
gs_vec4_norm(gs_vec4 v) 
{
    return gs_vec4_scale(v, 1.0f / gs_vec4_len(v));
}

gs_inline float
gs_vec4_dist(gs_vec4 v0, gs_vec4 v1)
{
    float dx = (v0.x - v1.x);
    float dy = (v0.y - v1.y);
    float dz = (v0.z - v1.z);
    float dw = (v0.w - v1.w);
    return (float)(sqrt(dx * dx + dy * dy + dz * dz + dw * dw));
}

/*================================================================================
// Useful Vector Functions
================================================================================*/

gs_inline
gs_vec3 gs_v4_to_v3(gs_vec4 v) 
{
    return gs_v3(v.x, v.y, v.z);
}

/*================================================================================
// Mat4x4
================================================================================*/

/*
    Matrices are stored in linear, contiguous memory and assume a column-major ordering.
*/

typedef struct gs_mat4
{
    float elements[16];
} gs_mat4;

gs_inline gs_mat4 
gs_mat4_diag(float val)
{
    gs_mat4 m;
    memset(m.elements, 0, sizeof(m.elements));
    m.elements[0 + 0 * 4] = val;
    m.elements[1 + 1 * 4] = val;
    m.elements[2 + 2 * 4] = val;
    m.elements[3 + 3 * 4] = val;
    return m;
}

#define gs_mat4_identity()\
    gs_mat4_diag(1.0f)

gs_inline gs_mat4
gs_mat4_ctor() {
    gs_mat4 mat = gsm_defval();
    return mat;
}

gs_inline
gs_mat4 gs_mat4_elem(const float elements[16])
{
    gs_mat4 mat = gs_mat4_ctor();
    memcpy(mat.elements, elements, sizeof(float) * 16);
    return mat;
}

gs_inline gs_mat4 
gs_mat4_mul(gs_mat4 m0, gs_mat4 m1)
{
    gs_mat4 m_res = gs_mat4_ctor(); 
    for (uint32_t y = 0; y < 4; ++y)
    {
        for (uint32_t x = 0; x < 4; ++x)
        {
            float sum = 0.0f;
            for (uint32_t e = 0; e < 4; ++e)
            {
                sum += m0.elements[x + e * 4] * m1.elements[e + y * 4];
            }
            m_res.elements[x + y * 4] = sum;
        }
    }

    return m_res;
}

gs_inline 
gs_mat4 gs_mat4_mul_list(uint32_t count, ...)
{
    va_list ap;
    gs_mat4 m = gs_mat4_identity();
    va_start(ap, count);
    for (uint32_t i = 0; i < count; ++i) {
        m = gs_mat4_mul(m, va_arg(ap, gs_mat4));
    }
    va_end(ap);
    return m;
}

gs_inline
void gs_mat4_set_elements(gs_mat4* m, float* elements, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        m->elements[i] = elements[i];
    }
}

gs_inline
gs_mat4 gs_mat4_transpose(gs_mat4 m)
{
    gs_mat4 t = gs_mat4_identity();

    // First row
    t.elements[0 * 4 + 0] = m.elements[0 * 4 + 0];
    t.elements[1 * 4 + 0] = m.elements[0 * 4 + 1];
    t.elements[2 * 4 + 0] = m.elements[0 * 4 + 2];
    t.elements[3 * 4 + 0] = m.elements[0 * 4 + 3];

    // Second row
    t.elements[0 * 4 + 1] = m.elements[1 * 4 + 0];
    t.elements[1 * 4 + 1] = m.elements[1 * 4 + 1];
    t.elements[2 * 4 + 1] = m.elements[1 * 4 + 2];
    t.elements[3 * 4 + 1] = m.elements[1 * 4 + 3];

    // Third row
    t.elements[0 * 4 + 2] = m.elements[2 * 4 + 0];
    t.elements[1 * 4 + 2] = m.elements[2 * 4 + 1];
    t.elements[2 * 4 + 2] = m.elements[2 * 4 + 2];
    t.elements[3 * 4 + 2] = m.elements[2 * 4 + 3];

    // Fourth row
    t.elements[0 * 4 + 3] = m.elements[3 * 4 + 0];
    t.elements[1 * 4 + 3] = m.elements[3 * 4 + 1];
    t.elements[2 * 4 + 3] = m.elements[3 * 4 + 2];
    t.elements[3 * 4 + 3] = m.elements[3 * 4 + 3];

    return t;
}

gs_inline
gs_mat4 gs_mat4_inverse(gs_mat4 m)
{
    gs_mat4 res = gs_mat4_identity();

    float temp[16];

    temp[0] = m.elements[5] * m.elements[10] * m.elements[15] -
        m.elements[5] * m.elements[11] * m.elements[14] -
        m.elements[9] * m.elements[6] * m.elements[15] +
        m.elements[9] * m.elements[7] * m.elements[14] +
        m.elements[13] * m.elements[6] * m.elements[11] -
        m.elements[13] * m.elements[7] * m.elements[10];

    temp[4] = -m.elements[4] * m.elements[10] * m.elements[15] +
        m.elements[4] * m.elements[11] * m.elements[14] +
        m.elements[8] * m.elements[6] * m.elements[15] -
        m.elements[8] * m.elements[7] * m.elements[14] -
        m.elements[12] * m.elements[6] * m.elements[11] +
        m.elements[12] * m.elements[7] * m.elements[10];

    temp[8] = m.elements[4] * m.elements[9] * m.elements[15] -
        m.elements[4] * m.elements[11] * m.elements[13] -
        m.elements[8] * m.elements[5] * m.elements[15] +
        m.elements[8] * m.elements[7] * m.elements[13] +
        m.elements[12] * m.elements[5] * m.elements[11] -
        m.elements[12] * m.elements[7] * m.elements[9];

    temp[12] = -m.elements[4] * m.elements[9] * m.elements[14] +
        m.elements[4] * m.elements[10] * m.elements[13] +
        m.elements[8] * m.elements[5] * m.elements[14] -
        m.elements[8] * m.elements[6] * m.elements[13] -
        m.elements[12] * m.elements[5] * m.elements[10] +
        m.elements[12] * m.elements[6] * m.elements[9];

    temp[1] = -m.elements[1] * m.elements[10] * m.elements[15] +
        m.elements[1] * m.elements[11] * m.elements[14] +
        m.elements[9] * m.elements[2] * m.elements[15] -
        m.elements[9] * m.elements[3] * m.elements[14] -
        m.elements[13] * m.elements[2] * m.elements[11] +
        m.elements[13] * m.elements[3] * m.elements[10];

    temp[5] = m.elements[0] * m.elements[10] * m.elements[15] -
        m.elements[0] * m.elements[11] * m.elements[14] -
        m.elements[8] * m.elements[2] * m.elements[15] +
        m.elements[8] * m.elements[3] * m.elements[14] +
        m.elements[12] * m.elements[2] * m.elements[11] -
        m.elements[12] * m.elements[3] * m.elements[10];

    temp[9] = -m.elements[0] * m.elements[9] * m.elements[15] +
        m.elements[0] * m.elements[11] * m.elements[13] +
        m.elements[8] * m.elements[1] * m.elements[15] -
        m.elements[8] * m.elements[3] * m.elements[13] -
        m.elements[12] * m.elements[1] * m.elements[11] +
        m.elements[12] * m.elements[3] * m.elements[9];

    temp[13] = m.elements[0] * m.elements[9] * m.elements[14] -
        m.elements[0] * m.elements[10] * m.elements[13] -
        m.elements[8] * m.elements[1] * m.elements[14] +
        m.elements[8] * m.elements[2] * m.elements[13] +
        m.elements[12] * m.elements[1] * m.elements[10] -
        m.elements[12] * m.elements[2] * m.elements[9];

    temp[2] = m.elements[1] * m.elements[6] * m.elements[15] -
        m.elements[1] * m.elements[7] * m.elements[14] -
        m.elements[5] * m.elements[2] * m.elements[15] +
        m.elements[5] * m.elements[3] * m.elements[14] +
        m.elements[13] * m.elements[2] * m.elements[7] -
        m.elements[13] * m.elements[3] * m.elements[6];

    temp[6] = -m.elements[0] * m.elements[6] * m.elements[15] +
        m.elements[0] * m.elements[7] * m.elements[14] +
        m.elements[4] * m.elements[2] * m.elements[15] -
        m.elements[4] * m.elements[3] * m.elements[14] -
        m.elements[12] * m.elements[2] * m.elements[7] +
        m.elements[12] * m.elements[3] * m.elements[6];

    temp[10] = m.elements[0] * m.elements[5] * m.elements[15] -
        m.elements[0] * m.elements[7] * m.elements[13] -
        m.elements[4] * m.elements[1] * m.elements[15] +
        m.elements[4] * m.elements[3] * m.elements[13] +
        m.elements[12] * m.elements[1] * m.elements[7] -
        m.elements[12] * m.elements[3] * m.elements[5];

    temp[14] = -m.elements[0] * m.elements[5] * m.elements[14] +
        m.elements[0] * m.elements[6] * m.elements[13] +
        m.elements[4] * m.elements[1] * m.elements[14] -
        m.elements[4] * m.elements[2] * m.elements[13] -
        m.elements[12] * m.elements[1] * m.elements[6] +
        m.elements[12] * m.elements[2] * m.elements[5];

    temp[3] = -m.elements[1] * m.elements[6] * m.elements[11] +
        m.elements[1] * m.elements[7] * m.elements[10] +
        m.elements[5] * m.elements[2] * m.elements[11] -
        m.elements[5] * m.elements[3] * m.elements[10] -
        m.elements[9] * m.elements[2] * m.elements[7] +
        m.elements[9] * m.elements[3] * m.elements[6];

    temp[7] = m.elements[0] * m.elements[6] * m.elements[11] -
        m.elements[0] * m.elements[7] * m.elements[10] -
        m.elements[4] * m.elements[2] * m.elements[11] +
        m.elements[4] * m.elements[3] * m.elements[10] +
        m.elements[8] * m.elements[2] * m.elements[7] -
        m.elements[8] * m.elements[3] * m.elements[6];

    temp[11] = -m.elements[0] * m.elements[5] * m.elements[11] +
        m.elements[0] * m.elements[7] * m.elements[9] +
        m.elements[4] * m.elements[1] * m.elements[11] -
        m.elements[4] * m.elements[3] * m.elements[9] -
        m.elements[8] * m.elements[1] * m.elements[7] +
        m.elements[8] * m.elements[3] * m.elements[5];

    temp[15] = m.elements[0] * m.elements[5] * m.elements[10] -
        m.elements[0] * m.elements[6] * m.elements[9] -
        m.elements[4] * m.elements[1] * m.elements[10] +
        m.elements[4] * m.elements[2] * m.elements[9] +
        m.elements[8] * m.elements[1] * m.elements[6] -
        m.elements[8] * m.elements[2] * m.elements[5];

    float determinant = m.elements[0] * temp[0] + m.elements[1] * temp[4] + m.elements[2] * temp[8] + m.elements[3] * temp[12];
    determinant = 1.0f / determinant;

    for (int i = 0; i < 4 * 4; i++)
        res.elements[i] = (float)(temp[i] * (float)determinant);

    return res;
}

/*
    float l : left
    float r : right
    float b : bottom
    float t : top
    float n : near
    float f : far
*/
gs_inline gs_mat4 
gs_mat4_ortho(float l, float r, float b, float t, float n, float f)
{
    gs_mat4 m_res = gs_mat4_identity();     

    // Main diagonal
    m_res.elements[0 + 0 * 4] = 2.0f / (r - l);
    m_res.elements[1 + 1 * 4] = 2.0f / (t - b);
    m_res.elements[2 + 2 * 4] = -2.0f / (f - n);

    // Last column
    m_res.elements[0 + 3 * 4] = -(r + l) / (r - l);
    m_res.elements[1 + 3 * 4] = -(t + b) / (t - b);
    m_res.elements[2 + 3 * 4] = -(f + n) / (f - n);

    return m_res;
}

gs_inline gs_mat4 
gs_mat4_perspective(float fov, float asp_ratio, float n, float f)
{
    // Zero matrix
    gs_mat4 m_res = gs_mat4_ctor();

    float q = 1.0f / (float)tan(gs_deg2rad(0.5f * fov));
    float a = q / asp_ratio;
    float b = (n + f) / (n - f);
    float c = (2.0f * n * f) / (n - f);

    m_res.elements[0 + 0 * 4] = a;
    m_res.elements[1 + 1 * 4] = q;
    m_res.elements[2 + 2 * 4] = b;
    m_res.elements[2 + 3 * 4] = c;
    m_res.elements[3 + 2 * 4] = -1.0f;

    return m_res;
}

gs_inline gs_mat4 
gs_mat4_translatev(const gs_vec3 v)
{
    gs_mat4 m_res = gs_mat4_identity();

    m_res.elements[0 + 4 * 3] = v.x;
    m_res.elements[1 + 4 * 3] = v.y;
    m_res.elements[2 + 4 * 3] = v.z;

    return m_res;
}

gs_inline gs_mat4 
gs_mat4_translate(float x, float y, float z)
{
    return gs_mat4_translatev(gs_v3(x, y, z));
}

gs_inline gs_mat4 
gs_mat4_scalev(const gs_vec3 v)
{
    gs_mat4 m_res = gs_mat4_identity();
    m_res.elements[0 + 0 * 4] = v.x;
    m_res.elements[1 + 1 * 4] = v.y;
    m_res.elements[2 + 2 * 4] = v.z;
    return m_res;
}

gs_inline gs_mat4
gs_mat4_scale(float x, float y, float z)
{
    return (gs_mat4_scalev(gs_v3(x, y, z)));
}

// Assumes normalized axis
gs_inline gs_mat4 gs_mat4_rotatev(float angle, gs_vec3 axis)
{
    gs_mat4 m_res = gs_mat4_identity();

    float a = angle;
    float c = (float)cos(a);
    float s = (float)sin(a);

    gs_vec3 naxis = gs_vec3_norm(axis);
    float x = naxis.x;
    float y = naxis.y;
    float z = naxis.z;

    //First column
    m_res.elements[0 + 0 * 4] = x * x * (1 - c) + c;    
    m_res.elements[1 + 0 * 4] = x * y * (1 - c) + z * s;    
    m_res.elements[2 + 0 * 4] = x * z * (1 - c) - y * s;    
    
    //Second column
    m_res.elements[0 + 1 * 4] = x * y * (1 - c) - z * s;    
    m_res.elements[1 + 1 * 4] = y * y * (1 - c) + c;    
    m_res.elements[2 + 1 * 4] = y * z * (1 - c) + x * s;    
    
    //Third column
    m_res.elements[0 + 2 * 4] = x * z * (1 - c) + y * s;    
    m_res.elements[1 + 2 * 4] = y * z * (1 - c) - x * s;    
    m_res.elements[2 + 2 * 4] = z * z * (1 - c) + c;    

    return m_res;
}

gs_inline gs_mat4
gs_mat4_rotate(float angle, float x, float y, float z)
{
    return gs_mat4_rotatev(angle, gs_v3(x, y, z));
}

gs_inline gs_mat4 
gs_mat4_look_at(gs_vec3 position, gs_vec3 target, gs_vec3 up)
{
    gs_vec3 f = gs_vec3_norm(gs_vec3_sub(target, position));
    gs_vec3 s = gs_vec3_norm(gs_vec3_cross(f, up));
    gs_vec3 u = gs_vec3_cross(s, f);

    gs_mat4 m_res = gs_mat4_identity();
    m_res.elements[0 * 4 + 0] = s.x;
    m_res.elements[1 * 4 + 0] = s.y;
    m_res.elements[2 * 4 + 0] = s.z;

    m_res.elements[0 * 4 + 1] = u.x;
    m_res.elements[1 * 4 + 1] = u.y;
    m_res.elements[2 * 4 + 1] = u.z;

    m_res.elements[0 * 4 + 2] = -f.x;
    m_res.elements[1 * 4 + 2] = -f.y;
    m_res.elements[2 * 4 + 2] = -f.z;

    m_res.elements[3 * 4 + 0] = -gs_vec3_dot(s, position);;
    m_res.elements[3 * 4 + 1] = -gs_vec3_dot(u, position);
    m_res.elements[3 * 4 + 2] = gs_vec3_dot(f, position); 

    return m_res;
}

gs_inline
gs_vec3 gs_mat4_mul_vec3(gs_mat4 m, gs_vec3 v)
{
    m = gs_mat4_transpose(m);
    return gs_vec3_ctor
    (
        m.elements[0 * 4 + 0] * v.x + m.elements[0 * 4 + 1] * v.y + m.elements[0 * 4 + 2] * v.z,  
        m.elements[1 * 4 + 0] * v.x + m.elements[1 * 4 + 1] * v.y + m.elements[1 * 4 + 2] * v.z,  
        m.elements[2 * 4 + 0] * v.x + m.elements[2 * 4 + 1] * v.y + m.elements[2 * 4 + 2] * v.z
    );
}
    
gs_inline
gs_vec4 gs_mat4_mul_vec4(gs_mat4 m, gs_vec4 v)
{
    m = gs_mat4_transpose(m);
    return gs_vec4_ctor
    (
        m.elements[0 * 4 + 0] * v.x + m.elements[0 * 4 + 1] * v.y + m.elements[0 * 4 + 2] * v.z + m.elements[0 * 4 + 3] * v.w,  
        m.elements[1 * 4 + 0] * v.x + m.elements[1 * 4 + 1] * v.y + m.elements[1 * 4 + 2] * v.z + m.elements[1 * 4 + 3] * v.w,  
        m.elements[2 * 4 + 0] * v.x + m.elements[2 * 4 + 1] * v.y + m.elements[2 * 4 + 2] * v.z + m.elements[2 * 4 + 3] * v.w,  
        m.elements[3 * 4 + 0] * v.x + m.elements[3 * 4 + 1] * v.y + m.elements[3 * 4 + 2] * v.z + m.elements[3 * 4 + 3] * v.w
    );
}

/*================================================================================
// Quaternion
================================================================================*/

typedef struct
{
    union 
    {
        float xyzw[4];
        struct 
        {
            float x, y, z, w;
        };
    };
} gs_quat;

gs_inline
gs_quat gs_quat_default()
{
    gs_quat q;
    q.x = 0.f;  
    q.y = 0.f;  
    q.z = 0.f;  
    q.w = 1.f;  
    return q;
}

gs_inline
gs_quat gs_quat_ctor(float _x, float _y, float _z, float _w)
{
    gs_quat q;
    q.x = _x;
    q.y = _y;
    q.z = _z;
    q.w = _w;
    return q;
}

gs_inline gs_quat 
gs_quat_add(gs_quat q0, gs_quat q1) 
{
    return gs_quat_ctor(q0.x + q1.x, q0.y + q1.y, q0.z + q1.z, q0.w + q1.w);
}

gs_inline gs_quat 
gs_quat_sub(gs_quat q0, gs_quat q1)
{
    return gs_quat_ctor(q0.x - q1.x, q0.y - q1.y, q0.z - q1.z, q0.w - q1.w);
}

gs_inline gs_quat
gs_quat_mul(gs_quat q0, gs_quat q1)
{
    return gs_quat_ctor(
        q0.w * q1.x + q1.w * q0.x + q0.y * q1.z - q1.y * q0.z,
        q0.w * q1.y + q1.w * q0.y + q0.z * q1.x - q1.z * q0.x,
        q0.w * q1.z + q1.w * q0.z + q0.x * q1.y - q1.x * q0.y,
        q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z
    );
}

gs_inline 
gs_quat gs_quat_mul_list(uint32_t count, ...)
{
    va_list ap;
    gs_quat q = gs_quat_default();
    va_start(ap, count);
    for (uint32_t i = 0; i < count; ++i)
    {
        q = gs_quat_mul(q, va_arg(ap, gs_quat));
    }
    va_end(ap);
    return q;
}

gs_inline gs_quat 
gs_quat_mul_quat(gs_quat q0, gs_quat q1)
{
    return gs_quat_ctor(
        q0.w * q1.x + q1.w * q0.x + q0.y * q1.z - q1.y * q0.z,
        q0.w * q1.y + q1.w * q0.y + q0.z * q1.x - q1.z * q0.x,
        q0.w * q1.z + q1.w * q0.z + q0.x * q1.y - q1.x * q0.y,
        q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z
    );
}

gs_inline 
gs_quat gs_quat_scale(gs_quat q, float s)
{
    return gs_quat_ctor(q.x * s, q.y * s, q.z * s, q.w * s);
}

gs_inline float 
gs_quat_dot(gs_quat q0, gs_quat q1)
{
    return (float)(q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w);
}

gs_inline 
gs_quat gs_quat_conjugate(gs_quat q)
{
    return (gs_quat_ctor(-q.x, -q.y, -q.z, q.w));
}

gs_inline float
gs_quat_len(gs_quat q)
{
    return (float)sqrt(gs_quat_dot(q, q));
}

gs_inline gs_quat
gs_quat_norm(gs_quat q) 
{
    return gs_quat_scale(q, 1.0f / gs_quat_len(q));
}

gs_inline gs_quat
gs_quat_cross(gs_quat q0, gs_quat q1)
{
    return gs_quat_ctor (                                           
        q0.x * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y,  
        q0.w * q1.y + q0.y * q1.w + q0.z * q1.x - q0.x * q1.z,  
        q0.w * q1.z + q0.z * q1.w + q0.x * q1.y - q0.y * q1.x,  
        q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z   
    );
}

// Inverse := Conjugate / Dot;
gs_inline
gs_quat gs_quat_inverse(gs_quat q)
{
    return (gs_quat_scale(gs_quat_conjugate(q), 1.0f / gs_quat_dot(q, q)));
}

gs_inline gs_vec3 gs_quat_rotate(gs_quat q, gs_vec3 v)
{
    // nVidia SDK implementation
    gs_vec3 qvec = gs_vec3_ctor(q.x, q.y, q.z);
    gs_vec3 uv = gs_vec3_cross(qvec, v);
    gs_vec3 uuv = gs_vec3_cross(qvec, uv);
    uv = gs_vec3_scale(uv, 2.f * q.w);
    uuv = gs_vec3_scale(uuv, 2.f);
    return (gs_vec3_add(v, gs_vec3_add(uv, uuv)));
}

gs_inline gs_quat gs_quat_angle_axis(float rad, gs_vec3 axis)
{
    // Normalize axis
    gs_vec3 a = gs_vec3_norm(axis);

    // Get scalar
    float half_angle = 0.5f * rad;
    float s = (float)sin(half_angle);

    return gs_quat_ctor(a.x * s, a.y * s, a.z * s, (float)cos(half_angle));
}

gs_inline
gs_quat gs_quat_slerp(gs_quat a, gs_quat b, float t)
{
    float c = gs_quat_dot(a, b);
    gs_quat end = b;

    if (c < 0.0f)
    {
        // Reverse all signs
        c *= -1.0f;
        end.x *= -1.0f;
        end.y *= -1.0f;
        end.z *= -1.0f;
        end.w *= -1.0f;
    }

    // Calculate coefficients
    float sclp, sclq;
    if ((1.0f - c) > 0.0001f)
    {
        float omega = (float)acosf(c);
        float s = (float)sinf(omega);
        sclp = (float)sinf((1.0f - t) * omega) / s;
        sclq = (float)sinf(t * omega) / s; 
    }
    else
    {
        sclp = 1.0f - t;
        sclq = t;
    }

    gs_quat q;
    q.x = sclp * a.x + sclq * end.x;
    q.y = sclp * a.y + sclq * end.y;
    q.z = sclp * a.z + sclq * end.z;
    q.w = sclp * a.w + sclq * end.w;

    return q;
}

#define quat_axis_angle(__AXS, __ANG)\
    gs_quat_angle_axis(__ANG, __AXS)

/*
* @brief Convert given quaternion param into equivalent 4x4 rotation matrix
* @note: From http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm 
*/
gs_inline gs_mat4 gs_quat_to_mat4(gs_quat _q)
{
    gs_mat4 mat = gs_mat4_identity();
    gs_quat q = gs_quat_norm(_q);

    float xx = q.x * q.x; 
    float yy = q.y * q.y; 
    float zz = q.z * q.z; 
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    mat.elements[0 * 4 + 0] = 1.0f - 2.0f * (yy + zz);
    mat.elements[1 * 4 + 0] = 2.0f * (xy - wz);
    mat.elements[2 * 4 + 0] = 2.0f * (xz + wy);

    mat.elements[0 * 4 + 1] = 2.0f * (xy + wz);
    mat.elements[1 * 4 + 1] = 1.0f - 2.0f * (xx + zz);
    mat.elements[2 * 4 + 1] = 2.0f * (yz - wx);

    mat.elements[0 * 4 + 2] = 2.0f * (xz - wy);
    mat.elements[1 * 4 + 2] = 2.0f * (yz + wx);
    mat.elements[2 * 4 + 2] = 1.0f - 2.0f * (xx + yy);

    return mat;
}

gs_inline 
gs_quat gs_quat_from_euler(float yaw_deg, float pitch_deg, float roll_deg)
{
    float yaw = gs_deg2rad(yaw_deg);
    float pitch = gs_deg2rad(pitch_deg);
    float roll = gs_deg2rad(roll_deg);

    gs_quat q;
    float cy = (float)cos(yaw * 0.5f);
    float sy = (float)sin(yaw * 0.5f);
    float cr = (float)cos(roll * 0.5f);
    float sr = (float)sin(roll * 0.5f);
    float cp = (float)cos(pitch * 0.5f);
    float sp = (float)sin(pitch * 0.5f);

    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
    q.w = cy * cr * cp + sy * sr * sp;

    return q;
}

/*================================================================================
// Transform (Non-Uniform Scalar VQS)
================================================================================*/

/*
    - This follows a traditional 'VQS' structure for complex object transformations, 
        however it differs from the standard in that it allows for non-uniform 
        scaling in the form of a vec3.
*/
// Source: https://www.eurosis.org/cms/files/conf/gameon-asia/gameon-asia2007/R-SESSION/G1.pdf

typedef struct 
{
    gs_vec3     position;
    gs_quat     rotation;
    gs_vec3     scale;      
} gs_vqs;

gs_inline gs_vqs gs_vqs_ctor(gs_vec3 tns, gs_quat rot, gs_vec3 scl)
{
    gs_vqs t;   
    t.position = tns;
    t.rotation = rot;
    t.scale = scl;
    return t;
}

gs_inline 
gs_vqs gs_vqs_default()
{
    gs_vqs t = gs_vqs_ctor
    (
        gs_vec3_ctor(0.0f, 0.0f, 0.0f),
        gs_quat_ctor(0.0f, 0.0f, 0.0f, 1.0f),
        gs_vec3_ctor(1.0f, 1.0f, 1.0f)
    );
    return t;
}

// AbsScale = ParentScale * LocalScale
// AbsRot   = LocalRot * ParentRot
// AbsTrans = ParentPos + [ParentRot * (ParentScale * LocalPos)]
gs_inline gs_vqs gs_vqs_absolute_transform(const gs_vqs* local, const gs_vqs* parent)
{
    // Normalized rotations
    gs_quat p_rot_norm = gs_quat_norm(parent->rotation);
    gs_quat l_rot_norm = gs_quat_norm(local->rotation);

    // Scale
    gs_vec3 scl = gs_vec3_mul(local->scale, parent->scale);
    // Rotation
    gs_quat rot = gs_quat_norm(gs_quat_mul(p_rot_norm, l_rot_norm));
    // position
    gs_vec3 tns = gs_vec3_add(parent->position, gs_quat_rotate(p_rot_norm, gs_vec3_mul(parent->scale, local->position)));

    return gs_vqs_ctor(tns, rot, scl);
}

// RelScale = AbsScale / ParentScale 
// RelRot   = Inverse(ParentRot) * AbsRot
// RelTrans = [Inverse(ParentRot) * (AbsPos - ParentPosition)] / ParentScale;
gs_inline gs_vqs gs_vqs_relative_transform(const gs_vqs* absolute, const gs_vqs* parent)
{
    // Get inverse rotation normalized
    gs_quat p_rot_inv = gs_quat_norm(gs_quat_inverse(parent->rotation));
    // Normalized abs rotation
    gs_quat a_rot_norm = gs_quat_norm(absolute->rotation);

    // Scale
    gs_vec3 scl = gs_vec3_div(absolute->scale, parent->scale);
    // Rotation
    gs_quat rot = gs_quat_norm(gs_quat_mul(p_rot_inv, a_rot_norm));
    // position
    gs_vec3 tns = gs_vec3_div(gs_quat_rotate(p_rot_inv, gs_vec3_sub(absolute->position, parent->position)), parent->scale);

    return gs_vqs_ctor(tns, rot, scl);
}

gs_inline gs_mat4 gs_vqs_to_mat4(const gs_vqs* transform)
{
    gs_mat4 mat = gs_mat4_identity();
    gs_mat4 trans = gs_mat4_translatev(transform->position);
    gs_mat4 rot = gs_quat_to_mat4(transform->rotation);
    gs_mat4 scl = gs_mat4_scalev(transform->scale);
    mat = gs_mat4_mul(mat, trans);
    mat = gs_mat4_mul(mat, rot);
    mat = gs_mat4_mul(mat, scl);
    return mat;
}

/*================================================================================
// Ray
================================================================================*/

typedef struct 
{
    gs_vec3 point;
    gs_vec3 direction;  
} gs_ray;

gs_inline gs_ray gs_ray_ctor(gs_vec3 pt, gs_vec3 dir)
{
    gs_ray r;
    r.point = pt;
    r.direction = dir;
    return r;
}

/*================================================================================
// Plane
================================================================================*/

typedef struct gs_plane_t
{
    union
    {
        gs_vec3 n;
        struct 
        {
            float a;
            float b;
            float c;
        };
    };

    float d;
} gs_plane_t;

#endif // __GSM_H__
