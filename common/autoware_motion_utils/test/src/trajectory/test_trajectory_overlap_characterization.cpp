// Copyright 2026 The Autoware Contributors
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

// Characterization tests that pin the exact outputs of the functions whose
// internal overlap handling is being optimized (calcLongitudinalOffsetToSegment,
// calcLateralOffset and calcSignedArcLengthPartialSum). They focus on inputs
// that contain coincident / overlapping points and reversed index ranges so the
// overlap-skipping and reversed-range semantics stay locked across the refactor.

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::create_point;

constexpr double kTestTolerance = 1e-9;

TrajectoryPoint makePoint(const double x, const double y)
{
  TrajectoryPoint p;
  p.pose.position = create_point(x, y, 0.0);
  p.pose.orientation.w = 1.0;
  return p;
}

// A trajectory whose points contain several coincident (overlapping) points so
// that the overlap-removal index mapping is non-trivial.
//   idx: 0      1      2      3      4      5      6
//   pt : (0,0) (0,0) (1,0) (1,0) (1,0) (2,0) (3,0)
std::vector<TrajectoryPoint> overlappingPoints()
{
  return {makePoint(0.0, 0.0), makePoint(0.0, 0.0), makePoint(1.0, 0.0), makePoint(1.0, 0.0),
          makePoint(1.0, 0.0), makePoint(2.0, 0.0), makePoint(3.0, 0.0)};
}
}  // namespace

// ---------------------------------------------------------------------------
// calcLongitudinalOffsetToSegment: overlap-skip semantics
// ---------------------------------------------------------------------------
TEST(trajectory_overlap_characterization, calcLongitudinalOffsetToSegment_overlap_skip)
{
  using autoware::motion_utils::calcLongitudinalOffsetToSegment;

  const auto points = overlappingPoints();
  const auto target = create_point(0.5, 4.0, 0.0);

  // seg_idx = 0 points at (0,0); the next non-coincident point is (1,0) at idx 2.
  // The segment is therefore (0,0)->(1,0) and the longitudinal offset is 0.5.
  EXPECT_NEAR(calcLongitudinalOffsetToSegment(points, 0, target), 0.5, kTestTolerance);

  // seg_idx = 1 also points at (0,0); same skip behavior, same segment, same result.
  EXPECT_NEAR(calcLongitudinalOffsetToSegment(points, 1, target), 0.5, kTestTolerance);

  // seg_idx = 2 points at (1,0); the next non-coincident point is (2,0) at idx 5.
  // The segment is (1,0)->(2,0); for target x=0.5 the offset is -0.5.
  EXPECT_NEAR(calcLongitudinalOffsetToSegment(points, 2, target), -0.5, kTestTolerance);

  // seg_idx = 3 and 4 also point at (1,0): identical skip, identical result.
  EXPECT_NEAR(calcLongitudinalOffsetToSegment(points, 3, target), -0.5, kTestTolerance);
  EXPECT_NEAR(calcLongitudinalOffsetToSegment(points, 4, target), -0.5, kTestTolerance);

  // seg_idx = 5 points at (2,0); next non-coincident is (3,0): segment (2,0)->(3,0).
  EXPECT_NEAR(calcLongitudinalOffsetToSegment(points, 5, target), -1.5, kTestTolerance);
}

TEST(trajectory_overlap_characterization, calcLongitudinalOffsetToSegment_trailing_overlap_nan)
{
  using autoware::motion_utils::calcLongitudinalOffsetToSegment;

  // (0,0) (1,0) (1,0) (1,0): seg_idx 1 points at (1,0) and every later point is
  // coincident with it, so there is no valid back point -> NaN (no-throw path).
  const std::vector<TrajectoryPoint> points = {
    makePoint(0.0, 0.0), makePoint(1.0, 0.0), makePoint(1.0, 0.0), makePoint(1.0, 0.0)};
  const auto target = create_point(2.0, 0.0, 0.0);

  EXPECT_TRUE(std::isnan(calcLongitudinalOffsetToSegment(points, 1, target)));

  // The same situation but with throw_exception = true must raise runtime_error.
  EXPECT_THROW(calcLongitudinalOffsetToSegment(points, 1, target, true), std::runtime_error);
}

// Degenerate coordinate (NaN / Inf) characterization. removeOverlapPoints treats a NaN-coordinate
// point as non-coincident (its skip condition `abs(dx) < eps && abs(dy) < eps` is false under IEEE
// NaN), so it is kept as the back point and the resulting segment vector contains a NaN, yielding a
// NaN offset. This pins the overlap scan to that exact base behavior: it must keep, not skip, the
// NaN/Inf point. The break condition is written as the negation of the skip condition so it stays
// the De Morgan complement of removeOverlapPoints under IEEE NaN.
TEST(trajectory_overlap_characterization, calcLongitudinalOffsetToSegment_nan_back_point)
{
  using autoware::motion_utils::calcLongitudinalOffsetToSegment;

  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  const auto target = create_point(2.0, 1.0, 0.0);

  // seg_idx = 0 at (0,0); idx 1 has a NaN x; idx 2 is finite and non-coincident.
  // The NaN point at idx 1 is kept as the back point -> NaN result.
  const std::vector<TrajectoryPoint> nan_x = {
    makePoint(0.0, 0.0), makePoint(nan_v, 0.0), makePoint(5.0, 0.0)};
  EXPECT_TRUE(std::isnan(calcLongitudinalOffsetToSegment(nan_x, 0, target)));

  // Same with a NaN y coordinate at idx 1.
  const std::vector<TrajectoryPoint> nan_y = {
    makePoint(0.0, 0.0), makePoint(0.0, nan_v), makePoint(5.0, 0.0)};
  EXPECT_TRUE(std::isnan(calcLongitudinalOffsetToSegment(nan_y, 0, target)));

  // An Inf-coordinate back point likewise propagates: abs(Inf) >= eps so it is kept as the back
  // point and segment_vec.norm() is Inf -> the offset is NaN (finite / Inf or Inf / Inf).
  const double inf_v = std::numeric_limits<double>::infinity();
  const std::vector<TrajectoryPoint> inf_x = {makePoint(0.0, 0.0), makePoint(inf_v, 0.0)};
  EXPECT_TRUE(std::isnan(calcLongitudinalOffsetToSegment(inf_x, 0, target)));
}

// ---------------------------------------------------------------------------
// calcLateralOffset (seg_idx overload): overlap handling + index clamping
// ---------------------------------------------------------------------------
TEST(trajectory_overlap_characterization, calcLateralOffset_seg_idx_overlap)
{
  using autoware::motion_utils::calcLateralOffset;

  const auto points = overlappingPoints();
  // Overlap-removed sequence is (0,0) (1,0) (2,0) (3,0) -> size 4, p_indices = 2.
  const auto target = create_point(0.5, 4.0, 0.0);

  // seg_idx clamped to p_indices = 2 -> segment (2,0)->(3,0); lateral offset of
  // a point with y = 4 against a segment along +x is +4.
  EXPECT_NEAR(calcLateralOffset(points, target, static_cast<size_t>(5)), 4.0, kTestTolerance);
  EXPECT_NEAR(calcLateralOffset(points, target, static_cast<size_t>(2)), 4.0, kTestTolerance);

  // seg_idx = 0 -> segment (0,0)->(1,0); same lateral offset of +4.
  EXPECT_NEAR(calcLateralOffset(points, target, static_cast<size_t>(0)), 4.0, kTestTolerance);
}

TEST(trajectory_overlap_characterization, calcLateralOffset_seg_idx_all_coincident_nan)
{
  using autoware::motion_utils::calcLateralOffset;

  // All points coincide -> overlap removal collapses to a single point -> NaN.
  const std::vector<TrajectoryPoint> points = {
    makePoint(1.0, 1.0), makePoint(1.0, 1.0), makePoint(1.0, 1.0)};
  const auto target = create_point(2.0, 2.0, 0.0);

  EXPECT_TRUE(std::isnan(calcLateralOffset(points, target, static_cast<size_t>(1))));
  EXPECT_THROW(calcLateralOffset(points, target, static_cast<size_t>(1), true), std::runtime_error);
}

// ---------------------------------------------------------------------------
// calcLateralOffset (auto-segment overload): overlap handling
// ---------------------------------------------------------------------------
TEST(trajectory_overlap_characterization, calcLateralOffset_auto_overlap)
{
  using autoware::motion_utils::calcLateralOffset;

  const auto points = overlappingPoints();

  // Target lies laterally off the (1,0)->(2,0) segment region.
  EXPECT_NEAR(calcLateralOffset(points, create_point(1.5, 2.0, 0.0)), 2.0, kTestTolerance);
  EXPECT_NEAR(calcLateralOffset(points, create_point(2.5, -3.0, 0.0)), -3.0, kTestTolerance);

  const std::vector<TrajectoryPoint> all_coincident = {makePoint(1.0, 1.0), makePoint(1.0, 1.0)};
  EXPECT_TRUE(std::isnan(calcLateralOffset(all_coincident, create_point(2.0, 2.0, 0.0))));
}

// ---------------------------------------------------------------------------
// calcSignedArcLengthPartialSum: forward and reversed index ranges
// ---------------------------------------------------------------------------
TEST(trajectory_overlap_characterization, calcSignedArcLengthPartialSum_forward_and_reversed)
{
  using autoware::motion_utils::calcSignedArcLengthPartialSum;

  // Distances between consecutive points: 0,1,0,0,1,1 (overlaps included).
  const auto points = overlappingPoints();

  // Forward range [0, 6): partial sums are the running cumulative arc length.
  const auto forward = calcSignedArcLengthPartialSum(points, 0, 6);
  ASSERT_EQ(forward.size(), static_cast<size_t>(6));
  EXPECT_NEAR(forward.at(0), 0.0, kTestTolerance);
  EXPECT_NEAR(forward.at(1), 0.0, kTestTolerance);  // (0,0)->(0,0)
  EXPECT_NEAR(forward.at(2), 1.0, kTestTolerance);  // ->(1,0)
  EXPECT_NEAR(forward.at(3), 1.0, kTestTolerance);  // ->(1,0)
  EXPECT_NEAR(forward.at(4), 1.0, kTestTolerance);  // ->(1,0)
  EXPECT_NEAR(forward.at(5), 2.0, kTestTolerance);  // ->(2,0)

  // Reversed range (src_idx > dst_idx): the function recurses with the
  // arguments swapped and returns the forward partial sum of [2, 5).
  const auto reversed = calcSignedArcLengthPartialSum(points, 5, 2);
  const auto expected = calcSignedArcLengthPartialSum(points, 2, 5);
  ASSERT_EQ(reversed.size(), expected.size());
  for (size_t i = 0; i < reversed.size(); ++i) {
    EXPECT_NEAR(reversed.at(i), expected.at(i), kTestTolerance);
  }
}

// src_idx == dst_idx is a zero-length range. It must return a single-element {0.0}
// partial-sum vector and must not recurse into itself (which previously caused a
// stack overflow because src_idx + 1 > dst_idx held for the equal-index case).
TEST(trajectory_overlap_characterization, calcSignedArcLengthPartialSum_equal_indices)
{
  using autoware::motion_utils::calcSignedArcLengthPartialSum;

  const auto points = overlappingPoints();

  for (const size_t idx :
       {static_cast<size_t>(0), static_cast<size_t>(3), static_cast<size_t>(6)}) {
    const auto partial = calcSignedArcLengthPartialSum(points, idx, idx);
    ASSERT_EQ(partial.size(), static_cast<size_t>(1));
    EXPECT_NEAR(partial.at(0), 0.0, kTestTolerance);
  }

  // Precondition-violation guard: an empty container is rejected by validateNonEmpty before the
  // equal-index base case, so the result is an empty vector {} (not {0.0}). This pins the
  // documented guard return alongside the equal-index postcondition.
  const std::vector<TrajectoryPoint> empty_points;
  EXPECT_TRUE(calcSignedArcLengthPartialSum(empty_points, 0, 0).empty());

  // Re-pin the reversed-range (src_idx > dst_idx) behavior: it equals the forward
  // partial sum over the swapped, ascending range and is unaffected by the new base case.
  const auto reversed = calcSignedArcLengthPartialSum(points, 6, 0);
  const auto forward = calcSignedArcLengthPartialSum(points, 0, 6);
  ASSERT_EQ(reversed.size(), forward.size());
  for (size_t i = 0; i < reversed.size(); ++i) {
    EXPECT_NEAR(reversed.at(i), forward.at(i), kTestTolerance);
  }
}
