# InfTexture - 纹理处理库
# 职责：UV 展开、纹理映射、Atlas 生成

add_library(InfTexture
  texture.cpp
)
add_library(InfMVS::Texture ALIAS InfTexture)

target_include_directories(InfTexture
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfTexture
  PUBLIC  InfCore InfMesh
  PRIVATE ${OpenCV_LIBS}
)

target_compile_definitions(InfTexture PRIVATE INFTEXTURE_EXPORTS)
