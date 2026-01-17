# CLI 应用程序

add_executable(infmvs_info main.cpp)
target_link_libraries(infmvs_info PRIVATE InfMeshPipeline)

# 平面检测工具
add_executable(plane_detect plane_detect.cpp)
target_link_libraries(plane_detect PRIVATE InfPointCloud InfIO)
target_include_directories(plane_detect PRIVATE ${CMAKE_SOURCE_DIR}/modules)
