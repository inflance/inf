# InfMesh - 网格处理库
# 职责：数据结构、简化、细分、修复

add_library(InfMesh
  mesh.cpp
)
add_library(InfMVS::Mesh ALIAS InfMesh)

target_include_directories(InfMesh
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfMesh
  PUBLIC InfCore InfPointCloud
)

target_compile_definitions(InfMesh PRIVATE INFMESH_EXPORTS)

# Install
install(TARGETS InfMesh
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES InfMesh.hpp mesh.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
