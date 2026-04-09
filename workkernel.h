#ifndef WORKKERNEL_H
#define WORKKERNEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t WorkKernel(uint32_t budget_cycles, uint32_t seed);

#ifdef __cplusplus
}
#endif

#endif