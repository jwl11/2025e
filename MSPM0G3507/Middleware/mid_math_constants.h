#ifndef __MID_MATH_CONSTANTS_H
#define __MID_MATH_CONSTANTS_H

/* ================================================================
 *  FOC 数学常量
 * ================================================================ */

/* Clamp macro */
#define _constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/* Pi family */
#define PI          3.14159265359f
#define _PI         3.14159265359f
#define _PI_2       1.57079632679f
#define _PI_3       1.04719755120f
#define _PI_4       0.78539816339f
#define _PI_6       0.52359877559f
#define _2PI        6.28318530718f
#define _3PI_2      4.71238898038f

/* Square roots */
#define _SQRT3      1.73205080757f
#define _SQRT3_2    0.86602540378f
#define _1_SQRT3    0.57735026919f
#define _2_SQRT3    1.15470053838f
#define _SQRT2      1.41421356237f

/* Radian precomputes */
#define _120_D2R    2.09439510239f   /* 120 deg to rad */

#endif /* __MID_MATH_CONSTANTS_H */
