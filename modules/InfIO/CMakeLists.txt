# InfIO - I/O 库
# 职责：COLMAP/MVS 格式读写、图像加载

add_library(InfIO
  io.cpp
)
add_library(InfMVS::IO ALIAS InfIO)

target_include_directories(InfIO
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfIO
  PUBLIC  InfCore
  PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json
)

target_compile_definitions(InfIO PRIVATE INFIO_EXPORTS)
