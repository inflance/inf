# 测试

find_package(GTest CONFIG REQUIRED)

# testdata 目录路径
set(TESTDATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../testdata")

add_executable(infmvs_tests
  test_core.cpp
  test_io.cpp
)

target_compile_definitions(infmvs_tests
  PRIVATE
    TESTDATA_DIR="${TESTDATA_DIR}"
)

target_link_libraries(infmvs_tests
  PRIVATE
    InfCore
    InfIO
    GTest::gtest
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(infmvs_tests)
