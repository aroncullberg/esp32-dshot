#include <cstdio>

void run_frame_codec_tests();
void run_command_sequence_tests();
void run_telemetry_tests();

int main() {
    run_frame_codec_tests();
    std::puts("PASS: frame codec");
    run_command_sequence_tests();
    std::puts("PASS: command sequence");
    run_telemetry_tests();
}
