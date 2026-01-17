# InfPointCloud - 点云处理库
# 职责：滤波、配准、法向估计、下采样
# 后端：PCL (Point Cloud Library)

file(GLOB_RECURSE INFPC_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE INFPC_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")

add_library(InfPointCloud ${INFPC_SOURCES} ${INFPC_HEADERS})
add_library(InfMVS::PointCloud ALIAS InfPointCloud)

# 查找 PCL
find_package(PCL REQUIRED COMPONENTS common filters features surface search kdtree)

target_include_directories(InfPointCloud
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
  PRIVATE ${PCL_INCLUDE_DIRS}
)

target_link_libraries(InfPointCloud
  PUBLIC  InfCore
  PRIVATE ${PCL_LIBRARIES}
)

target_compile_definitions(InfPointCloud
  PRIVATE INFPOINTCLOUD_EXPORTS
          ${PCL_DEFINITIONS}
)

# PCL 可能需要的编译选项
if(MSVC)
  target_compile_options(InfPointCloud PRIVATE /bigobj)
endif()

# Install
install(TARGETS InfPointCloud
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES ${INFPC_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
