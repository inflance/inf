# InfPointCloud - 点云处理库
# 职责：滤波、配准、法向估计

add_library(InfPointCloud
  point_cloud.cpp
)
add_library(InfMVS::PointCloud ALIAS InfPointCloud)

target_include_directories(InfPointCloud
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfPointCloud
  PUBLIC InfCore
)

target_compile_definitions(InfPointCloud PRIVATE INFPOINTCLOUD_EXPORTS)

# Install
install(TARGETS InfPointCloud
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES InfPointCloud.hpp point_cloud.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
