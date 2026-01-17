# InfPointCloud - 点云处理库
# 职责：滤波、配准、法向估计、下采样、平面检测
# 后端：PCL (Point Cloud Library), CGAL (Shape Detection)
if(INFMVS_USE_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "InfPointCloud: OpenMP enabled")
  else()
    message(WARNING "InfPointCloud: OpenMP requested but not found, disabling")
  endif()
endif()

file(GLOB_RECURSE INFPC_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE INFPC_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")

add_library(InfPointCloud ${INFPC_SOURCES} ${INFPC_HEADERS})
add_library(InfMVS::PointCloud ALIAS InfPointCloud)


# 查找 PCL
find_package(PCL REQUIRED COMPONENTS common filters features surface search kdtree)

# 查找 CGAL (用于平面检测) - CGAL::CGAL 会自动链接 GMP/MPFR
find_package(CGAL CONFIG REQUIRED)

# 查找 TBB (用于 CGAL 并行加速)
find_package(TBB CONFIG REQUIRED)

if(INFMVS_USE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(InfPointCloud PRIVATE OpenMP::OpenMP_CXX)
  target_compile_definitions(InfPointCloud PRIVATE INFMVS_USE_OPENMP)
endif()

target_include_directories(InfPointCloud
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
  PRIVATE ${PCL_INCLUDE_DIRS}
)

target_link_libraries(InfPointCloud
  PUBLIC  InfCore
  PRIVATE ${PCL_LIBRARIES}
          CGAL::CGAL
          TBB::tbb
)

target_compile_definitions(InfPointCloud
  PRIVATE INFPOINTCLOUD_EXPORTS
          ${PCL_DEFINITIONS}
          CGAL_LINKED_WITH_TBB  # 启用 CGAL 并行模式
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
