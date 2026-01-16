# InfCore - 基础设施库
# 职责：数学类型、内存管理、日志、错误处理

add_library(InfCore
  core.cpp
)
add_library(InfMVS::Core ALIAS InfCore)

target_include_directories(InfCore
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfCore
  PUBLIC  Eigen3::Eigen
  PRIVATE spdlog::spdlog fmt::fmt
)

target_compile_definitions(InfCore PRIVATE INFCORE_EXPORTS)
