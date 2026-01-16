/// @file main.cpp
/// @brief InfMVS CLI 入口

#include "InfMeshPipeline.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    std::cout << "InfMVS v" << inf::core::version() << "\n";
    
    if (argc < 2) {
        std::cerr << "Usage: infmvs_info <colmap_model_dir>\n";
        return 1;
    }
    
    // TODO: 实现 CLI 功能
    std::cout << "Model path: " << argv[1] << "\n";
    return 0;
}
