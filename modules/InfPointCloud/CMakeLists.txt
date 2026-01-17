# InfPointCloud - 点云处理库
# 职责：滤波、配准、法向估计

file(GLOB_RECURSE INFPC_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE INFPC_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")

add_library(InfPointCloud ${INFPC_SOURCES} ${INFPC_HEADERS})
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

install(FILES ${INFPC_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
