#pragma once
#include <cstdio>
#define ESP_LOGE(tag, format, ...) std::fprintf(stderr, "%s: " format "\n", tag, ##__VA_ARGS__)
