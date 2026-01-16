# 测试

find_package(GTest CONFIG REQUIRED)

add_executable(infmvs_tests
  test_core.cpp
)

target_link_libraries(infmvs_tests
  PRIVATE
    InfCore
    GTest::gtest
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(infmvs_tests)
