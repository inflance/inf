# InfCore - 基础设施库
# 职责：数学类型、内存管理、日志、错误处理

file(GLOB_RECURSE INFCORE_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE INFCORE_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")

add_library(InfCore ${INFCORE_SOURCES} ${INFCORE_HEADERS})
add_library(InfMVS::Core ALIAS InfCore)

target_include_directories(InfCore
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfCore
  PUBLIC  Eigen3::Eigen spdlog::spdlog magic_enum::magic_enum opencv_core
  PRIVATE fmt::fmt opencv_calib3d opencv_imgproc
)

target_compile_definitions(InfCore PRIVATE INFCORE_EXPORTS)

# Install
install(TARGETS InfCore
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES ${INFCORE_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
