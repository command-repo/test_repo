
#ifndef MACROS_H
#define MACROS_H

#include <stdint.h>

extern void assert_failed(char *file, uint32_t line);

#define ASSERT(expr) ((expr) ? (void)0 : assert_failed(__FILE__, __LINE__))

#ifndef MIN
#define MIN(a,b) ((a) <= (b) ? (a):(b))
#endif

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a):(b))
#endif

#endif