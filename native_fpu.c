#include <stdint.h>
#include <math.h>

union uf {
  float f;
  uint32_t u;
};

inline float reinterp_f(uint32_t a) {
  union uf uf;
  uf.u = a;
  return uf.f;
}
inline uint32_t reinterp_u(float a) {
  union uf uf;
  uf.f = a;
  return uf.u;
}

uint32_t native_fadd(uint32_t a, uint32_t b) {
  return reinterp_u(reinterp_f(a) + reinterp_f(b));
}
uint32_t native_fsub(uint32_t a, uint32_t b) {
  return reinterp_u(reinterp_f(a) - reinterp_f(b));
}
uint32_t native_fmul(uint32_t a, uint32_t b) {
  return reinterp_u(reinterp_f(a) * reinterp_f(b));
}
uint32_t native_itof(uint32_t a) { return reinterp_u((float)a); }
uint32_t native_ftoi(uint32_t a) { return (int32_t)reinterp_f(a); }

int native_feq(uint32_t a, uint32_t b) {
  return reinterp_f(a) == reinterp_f(b);
}
int native_flt(uint32_t a, uint32_t b) {
  return reinterp_f(a) < reinterp_f(b);
}
int native_fle(uint32_t a, uint32_t b) {
  return reinterp_f(a) <= reinterp_f(b);
}
uint32_t native_finv(uint32_t a) {
  return reinterp_u(1.0f / reinterp_f(a));
}
uint32_t native_fdiv(uint32_t a, uint32_t b) {
  return reinterp_u(reinterp_f(a) / reinterp_f(b));
}
uint32_t native_fsqrt(uint32_t a) {
  return reinterp_u(sqrtf(reinterp_f(a)));
}
uint32_t native_ffloor(uint32_t a) {
  return reinterp_u(floorf(reinterp_f(a)));
}
