#pragma once
#include "esp_err.h"
using esp_console_cmd_func_t = int (*)(int, char**);
struct esp_console_cmd_t { const char* command; const char* help; esp_console_cmd_func_t func; };
struct esp_console_repl_t {};
struct esp_console_repl_config_t { const char* prompt; int max_cmdline_length; };
#define ESP_CONSOLE_REPL_CONFIG_DEFAULT() {}
esp_err_t esp_console_cmd_register(const esp_console_cmd_t*);
esp_err_t esp_console_new_repl_stdio(const esp_console_repl_config_t*, esp_console_repl_t**);
esp_err_t esp_console_start_repl(esp_console_repl_t*);
