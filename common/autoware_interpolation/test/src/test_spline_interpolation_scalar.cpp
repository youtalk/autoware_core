// Copyright 2026 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/interpolation/spline_interpolation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using autoware::interpolation::SplineInterpolation;

// The scalar overloads must return exactly what the vector overloads return for the
// same query key. They are the per-knot allocation-free evaluation seam that the
// SplineInterpolationPoints2d per-knot loops route through.
TEST(spline_interpolation_scalar, value_matches_vector_overload)
{
  const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
  const std::vector<double> query_keys{-1.5, 0.0, 8.0, 12.0, 18.0, 20.0};

  const SplineInterpolation s(base_keys, base_values);

  const auto vector_values = s.getSplineInterpolatedValues(query_keys);
  for (size_t i = 0; i < query_keys.size(); ++i) {
    EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(query_keys.at(i)), vector_values.at(i));
  }

  // Independent external truth (not just agreement with the vector overload): a natural cubic
  // spline interpolates its knots exactly, so evaluating at a base key returns the base value.
  EXPECT_NEAR(s.getSplineInterpolatedValue(base_keys.front()), base_values.front(), 1e-9);
  EXPECT_NEAR(s.getSplineInterpolatedValue(base_keys.back()), base_values.back(), 1e-9);
}

TEST(spline_interpolation_scalar, diff_value_matches_vector_overload)
{
  const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
  const std::vector<double> query_keys{-1.5, 0.0, 8.0, 12.0, 18.0, 20.0};

  const SplineInterpolation s(base_keys, base_values);

  const auto vector_values = s.getSplineInterpolatedDiffValues(query_keys);
  for (size_t i = 0; i < query_keys.size(); ++i) {
    EXPECT_DOUBLE_EQ(s.getSplineInterpolatedDiffValue(query_keys.at(i)), vector_values.at(i));
  }
}

TEST(spline_interpolation_scalar, quad_diff_value_matches_vector_overload)
{
  const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
  const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
  const std::vector<double> query_keys{-1.5, 0.0, 8.0, 12.0, 18.0, 20.0};

  const SplineInterpolation s(base_keys, base_values);

  const auto vector_values = s.getSplineInterpolatedQuadDiffValues(query_keys);
  for (size_t i = 0; i < query_keys.size(); ++i) {
    EXPECT_DOUBLE_EQ(s.getSplineInterpolatedQuadDiffValue(query_keys.at(i)), vector_values.at(i));
  }
}

// Single-segment (n == 2) evaluation: the scalar overload clamps the segment index via
// get_index() and evaluates the cubic in place, matching the vector overload's per-key math.
TEST(spline_interpolation_scalar, value_at_two_knot_segment)
{
  const std::vector<double> base_keys{0.0, 2.0};
  const std::vector<double> base_values{0.0, 4.0};
  const SplineInterpolation s(base_keys, base_values);

  // n == 2 path: c_[0] = (4-0)/(2-0) = 2, d_[0] = 0 -> value = 2 * dx
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(0.0), 0.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(1.0), 2.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(2.0), 4.0);
  // first derivative is the constant slope, second derivative is zero on this segment
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedDiffValue(1.0), 2.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedQuadDiffValue(1.0), 0.0);
}

// Calling a scalar overload on an un-built (default-constructed) spline must throw rather than
// read out of bounds. The vector overloads already throw via validateKeys() for too-few knots;
// the scalar overloads skip query-key range validation but enforce the same built-spline
// precondition through get_index().
TEST(spline_interpolation_scalar, throws_on_unbuilt_spline)
{
  const SplineInterpolation s;  // default-constructed: base_keys_ is empty
  EXPECT_THROW(s.getSplineInterpolatedValue(0.0), std::runtime_error);
  EXPECT_THROW(s.getSplineInterpolatedDiffValue(0.0), std::runtime_error);
  EXPECT_THROW(s.getSplineInterpolatedQuadDiffValue(0.0), std::runtime_error);
}

// Characterization of the vector getters at the front/back epsilon band. validateKeys() accepts a
// query key up to 1e-3 outside [front, back], and the getters then evaluate the cubic with the RAW
// (unclamped) key: a front key in [front - 1e-3, front) yields dx < 0 and a mildly extrapolated
// value, NOT the clamped knot value d_[0]. This pins the behavior-preserving contract: the vector
// overload result for such a key equals the raw scalar evaluation, and is distinct from the value
// at the clamped front/back knot.
TEST(spline_interpolation_scalar, vector_getters_do_not_clamp_epsilon_boundary)
{
  // n == 2 segment: value(k) = 2 * (k - 0.0), so the math is exact (no cubic round-off).
  const std::vector<double> base_keys{0.0, 2.0};
  const std::vector<double> base_values{0.0, 4.0};
  const SplineInterpolation s(base_keys, base_values);

  // Front key just below base_keys.front(), inside the 1e-3 acceptance band.
  const double front_key = base_keys.front() - 5e-4;  // -0.0005, dx = -0.0005
  // Back key just above base_keys.back(), inside the 1e-3 acceptance band.
  const double back_key = base_keys.back() + 5e-4;  // 2.0005, dx (from knot 0) = 2.0005

  const auto values = s.getSplineInterpolatedValues({front_key, back_key});
  ASSERT_EQ(values.size(), 2u);

  // Raw (unclamped) extrapolation: value = 2 * dx.
  EXPECT_DOUBLE_EQ(values.at(0), 2.0 * front_key);  // -0.001, not 0.0 (the clamped front knot)
  EXPECT_DOUBLE_EQ(values.at(1), 2.0 * back_key);   // 4.001, not 4.0 (the clamped back knot)

  // The vector getter routes through the raw scalar evaluation for these boundary keys.
  EXPECT_DOUBLE_EQ(values.at(0), s.getSplineInterpolatedValue(front_key));
  EXPECT_DOUBLE_EQ(values.at(1), s.getSplineInterpolatedValue(back_key));

  // And is distinct from the clamped-to-knot value (what iterating the validated copy would give).
  EXPECT_NE(values.at(0), s.getSplineInterpolatedValue(base_keys.front()));
  EXPECT_NE(values.at(1), s.getSplineInterpolatedValue(base_keys.back()));

  // Diff/quad-diff getters likewise use the raw key; on this linear segment they are constant.
  const auto diff_values = s.getSplineInterpolatedDiffValues({front_key, back_key});
  EXPECT_DOUBLE_EQ(diff_values.at(0), 2.0);
  EXPECT_DOUBLE_EQ(diff_values.at(1), 2.0);
  const auto quad_diff_values = s.getSplineInterpolatedQuadDiffValues({front_key, back_key});
  EXPECT_DOUBLE_EQ(quad_diff_values.at(0), 0.0);
  EXPECT_DOUBLE_EQ(quad_diff_values.at(1), 0.0);
}

// The scalar overloads intentionally skip validateKeys() (per their header contract). For a key
// well outside the knot range -- beyond the 1e-3 band the vector overload would reject -- the
// scalar overload does NOT throw: get_index() clamps the segment index to the nearest end segment
// and the cubic is extrapolated from there. Pin the actual extrapolated value so this documented
// divergence from the vector overload is locked down.
TEST(spline_interpolation_scalar, out_of_range_key_extrapolates_without_throwing)
{
  // n == 2 segment: value(k) = 2 * k, diff = 2, quad-diff = 0 everywhere (extrapolation included).
  const std::vector<double> base_keys{0.0, 2.0};
  const std::vector<double> base_values{0.0, 4.0};
  const SplineInterpolation s(base_keys, base_values);

  // Far below the front knot (well outside the 1e-3 band -> vector overload would throw).
  EXPECT_NO_THROW(s.getSplineInterpolatedValue(-10.0));
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(-10.0), 2.0 * -10.0);  // -20.0
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedDiffValue(-10.0), 2.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedQuadDiffValue(-10.0), 0.0);

  // Far above the back knot: still extrapolated from the single segment (idx clamped to 0).
  EXPECT_NO_THROW(s.getSplineInterpolatedValue(12.0));
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedValue(12.0), 2.0 * 12.0);  // 24.0
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedDiffValue(12.0), 2.0);
  EXPECT_DOUBLE_EQ(s.getSplineInterpolatedQuadDiffValue(12.0), 0.0);

  // Contrast: the vector overload rejects the same out-of-band key.
  EXPECT_THROW(s.getSplineInterpolatedValues({-10.0}), std::invalid_argument);
}

// A NaN query key flows through std::lower_bound + std::clamp (unspecified ordering for NaN) and
// the cubic evaluation; the scalar overloads neither throw nor sanitize, so the result is NaN.
// Pin this so any future change to NaN handling is a conscious, reviewed decision.
TEST(spline_interpolation_scalar, nan_key_propagates_nan)
{
  const std::vector<double> base_keys{0.0, 2.0};
  const std::vector<double> base_values{0.0, 4.0};
  const SplineInterpolation s(base_keys, base_values);

  const double nan_key = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NO_THROW(s.getSplineInterpolatedValue(nan_key));
  EXPECT_TRUE(std::isnan(s.getSplineInterpolatedValue(nan_key)));
  EXPECT_TRUE(std::isnan(s.getSplineInterpolatedDiffValue(nan_key)));
  EXPECT_TRUE(std::isnan(s.getSplineInterpolatedQuadDiffValue(nan_key)));
}
