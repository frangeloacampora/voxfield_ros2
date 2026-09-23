#ifndef EIGEN_CHECKS_ENTRYPOINT_H_
#define EIGEN_CHECKS_ENTRYPOINT_H_

// Minimal test-only shim for eigen-checks/entrypoint.h (see gtest.h in this
// directory). voxfield's tests that include this header define their own
// main() rather than invoking UNITTEST_ENTRYPOINT, so this only needs to
// compile.

#include <glog/logging.h>
#include <gtest/gtest.h>

#define UNITTEST_ENTRYPOINT                \
  int main(int argc, char** argv) {        \
    ::testing::InitGoogleTest(&argc, argv); \
    google::InitGoogleLogging(argv[0]);    \
    return RUN_ALL_TESTS();                \
  }

#endif  // EIGEN_CHECKS_ENTRYPOINT_H_
