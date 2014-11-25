#ifndef QKFPU_H_
#define QKFPU_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "fpu/C/fpu.h"

uint32_t native_fadd(uint32_t, uint32_t);
uint32_t native_fsub(uint32_t, uint32_t);
uint32_t native_fmul(uint32_t, uint32_t);
uint32_t native_itof(uint32_t);
uint32_t native_ftoi(uint32_t);
int native_feq(uint32_t, uint32_t);
int native_flt(uint32_t, uint32_t);
int native_fle(uint32_t, uint32_t);
uint32_t native_finv(uint32_t);
uint32_t native_fdiv(uint32_t, uint32_t);
uint32_t native_fsqrt(uint32_t);
uint32_t native_ffloor(uint32_t);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* QKFPU_H_ */
