#pragma once
#include <cstdlib>
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
inline void* heap_caps_malloc(std::size_t bytes, unsigned) { return std::malloc(bytes); }
inline void heap_caps_free(void* memory) { std::free(memory); }
