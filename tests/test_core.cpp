/// @file test_core.cpp
#include "InfCore.hpp"
#include <gtest/gtest.h>

TEST(InfCore, Version) {
    auto ver = inf::core::version();
    EXPECT_FALSE(ver.empty());
    EXPECT_EQ(ver, "0.1.0");
}

TEST(InfCore, ErrorDefault) {
    inf::core::Error err;
    EXPECT_TRUE(err.ok());
    EXPECT_EQ(err.code, inf::core::ErrorCode::kSuccess);
}
