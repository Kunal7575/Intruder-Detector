#ifndef __dmaarouf_util_H
#define __dmaarouf_util_H
#ifdef __cplusplus
 extern "C" {
#endif

#define STM32F429_MAX_RAM 0x20030000

#include "malloc.h"

void * my_malloc(size_t size);

#ifdef __cplusplus
}
#endif
#endif /*__ __dmaarouf_util_H */
