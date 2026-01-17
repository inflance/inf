# InfMeshPipeline - 流水线库
# 职责：端到端重建、批处理

file(GLOB_RECURSE INFPIPE_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE INFPIPE_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")

add_library(InfMeshPipeline ${INFPIPE_SOURCES} ${INFPIPE_HEADERS})
add_library(InfMVS::Pipeline ALIAS InfMeshPipeline)

target_include_directories(InfMeshPipeline
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfMeshPipeline
  PUBLIC InfCore InfIO InfPointCloud InfMesh InfTexture
)

target_compile_definitions(InfMeshPipeline PRIVATE INFPIPELINE_EXPORTS)

# Install
install(TARGETS InfMeshPipeline
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES ${INFPIPE_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
