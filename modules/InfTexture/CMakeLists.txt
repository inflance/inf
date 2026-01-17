# InfTexture - 纹理处理库
# 职责：UV 展开、纹理映射、Atlas 生成

file(GLOB_RECURSE INFTEX_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE INFTEX_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")

add_library(InfTexture ${INFTEX_SOURCES} ${INFTEX_HEADERS})
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

# Install
install(TARGETS InfTexture
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES ${INFTEX_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
