#ifndef EIGEN_CHECKS_GTEST_H_
#define EIGEN_CHECKS_GTEST_H_

// Minimal test-only shim for the `eigen-checks` macros used by voxfield's
// gtests (EIGEN_MATRIX_EQUAL, EIGEN_MATRIX_NEAR). The original
// ethz-asl/eigen_checks package is catkin-only; this reimplements just the
// two macros voxfield's tests actually call.

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sstream>

namespace eigen_checks {
namespace internal {

template <typename DerivedA, typename DerivedB>
::testing::AssertionResult CompareMatrices(
    const ::Eigen::MatrixBase<DerivedA>& a,
    const ::Eigen::MatrixBase<DerivedB>& b, double tolerance,
    const char* a_expr, const char* b_expr) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    return ::testing::AssertionFailure()
           << "Size mismatch: " << a_expr << " is " << a.rows() << "x"
           << a.cols() << ", " << b_expr << " is " << b.rows() << "x"
           << b.cols();
  }
  const double max_abs_diff =
      a.size() == 0 ? 0.0
                    : (a.template cast<double>() - b.template cast<double>())
                          .cwiseAbs()
                          .maxCoeff();
  if (max_abs_diff > tolerance) {
    std::ostringstream os;
    os << "Matrices " << a_expr << " and " << b_expr << " differ by more than "
       << tolerance << ".\n"
       << a_expr << " =\n"
       << a << "\n"
       << b_expr << " =\n"
       << b;
    return ::testing::AssertionFailure() << os.str();
  }
  return ::testing::AssertionSuccess();
}

}  // namespace internal
}  // namespace eigen_checks

#define EIGEN_MATRIX_EQUAL(a, b) \
  ::eigen_checks::internal::CompareMatrices((a), (b), 0.0, #a, #b)
#define EIGEN_MATRIX_NEAR(a, b, tolerance) \
  ::eigen_checks::internal::CompareMatrices((a), (b), (tolerance), #a, #b)

#endif  // EIGEN_CHECKS_GTEST_H_
