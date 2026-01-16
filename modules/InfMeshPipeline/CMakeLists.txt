# InfMeshPipeline - 流水线库
# 职责：端到端重建、批处理

add_library(InfMeshPipeline
  pipeline.cpp
)
add_library(InfMVS::Pipeline ALIAS InfMeshPipeline)

target_include_directories(InfMeshPipeline
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfMeshPipeline
  PUBLIC InfCore InfIO InfPointCloud InfMesh InfTexture
)

target_compile_definitions(InfMeshPipeline PRIVATE INFPIPELINE_EXPORTS)
