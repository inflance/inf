# InfIO - I/O 库
# 职责：COLMAP/MVS 格式读写、图像加载

if(INFMVS_USE_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "InfIO: OpenMP enabled")
  else()
    message(WARNING "InfIO: OpenMP requested but not found, disabling")
  endif()
endif()

file(GLOB_RECURSE INFIO_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE INFIO_HEADERS CONFIGURE_DEPENDS "*.hpp" "*.h")

add_library(InfIO ${INFIO_SOURCES} ${INFIO_HEADERS})
add_library(InfMVS::IO ALIAS InfIO)

target_include_directories(InfIO
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
          $<INSTALL_INTERFACE:include/InfMVS>
)

target_link_libraries(InfIO
  PUBLIC  InfCore InfMesh InfPointCloud
  PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json assimp::assimp ${PCL_LIBRARIES}
)

if(INFMVS_USE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(InfIO PRIVATE OpenMP::OpenMP_CXX)
  target_compile_definitions(InfIO PRIVATE INFMVS_USE_OPENMP)
endif()

target_include_directories(InfIO PRIVATE ${PCL_INCLUDE_DIRS})

target_compile_definitions(InfIO PRIVATE INFIO_EXPORTS)

# Install
install(TARGETS InfIO
  EXPORT InfMVSTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES ${INFIO_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InfMVS)
