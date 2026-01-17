# InfIO - I/O 库
# 职责：COLMAP/MVS 格式读写、图像加载

add_library(InfIO
  io.cpp
)
add_library(InfMVS::IO ALIAS InfIO)

target_include_directories(InfIO
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfIO
  PUBLIC  InfCore InfMesh InfPointCloud
  PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json assimp::assimp ${PCL_LIBRARIES}
)

target_include_directories(InfIO PRIVATE ${PCL_INCLUDE_DIRS})

target_compile_definitions(InfIO PRIVATE INFIO_EXPORTS)

# Install
install(TARGETS InfIO
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES InfIO.hpp io.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
