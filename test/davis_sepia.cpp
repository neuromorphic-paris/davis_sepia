#include "../source/davis_sepia.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        sepia::capture_exception capture_exception;
        auto camera = davis_sepia::make_camera(
            [](sepia::dvs_event) {
                std::cout << ".";
                std::cout.flush();
            },
            [](davis_sepia::frame&) {
                std::cout << "|";
                std::cout.flush();
            },
            [](davis_sepia::imu_event) {
                std::cout << "i";
                std::cout.flush();
            },
            [](davis_sepia::external_input) {
                std::cout << "e";
                std::cout.flush();
            },
            std::ref(capture_exception));
        capture_exception.wait();
        capture_exception.rethrow_unless<>();
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << std::endl;
        return 1;
    }
    return 0;
}
