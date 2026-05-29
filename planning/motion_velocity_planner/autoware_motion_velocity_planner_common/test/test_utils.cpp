// Copyright 2025 TIER IV, Inc.
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

#include "autoware/motion_velocity_planner_common/utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::motion_velocity_planner::utils::calc_distance_to_front_object;
using autoware::motion_velocity_planner::utils::calc_object_possible_max_dist_from_center;
using autoware::motion_velocity_planner::utils::classification_types_from_flags;
using autoware::motion_velocity_planner::utils::concat_vectors;
using autoware::motion_velocity_planner::utils::get_extended_trajectory_points;
using autoware::motion_velocity_planner::utils::get_index_with_longitudinal_offset;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;

// Build a straight trajectory along +x with identity orientation and a forward longitudinal
// velocity so that direction detection returns "driving forward".
std::vector<TrajectoryPoint> make_straight_forward_trajectory(
  const size_t num_points, const double step)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i) * step;
    p.pose.position.y = 0.0;
    p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
    p.longitudinal_velocity_mps = 1.0;
    points.push_back(p);
  }
  return points;
}

geometry_msgs::msg::Point make_point(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}
}  // namespace

// ----------------------------- classification_types_from_flags -----------------------------

TEST(MvpUtilsClassification, MapsEnabledFlagsToClassificationValues)
{
  const std::map<std::string, bool> flags{
    {"unknown", false}, {"car", true},        {"truck", false},  {"bus", true},
    {"trailer", false}, {"motorcycle", true}, {"bicycle", true}, {"pedestrian", false}};

  const auto types = classification_types_from_flags(flags);

  // Output uses the fixed canonical ordering (unknown, car, truck, bus, trailer, motorcycle,
  // bicycle, pedestrian), filtered to the enabled entries.
  const std::vector<uint8_t> expected{
    ObjectClassification::CAR, ObjectClassification::BUS, ObjectClassification::MOTORCYCLE,
    ObjectClassification::BICYCLE};
  EXPECT_EQ(types, expected);
}

TEST(MvpUtilsClassification, AllEnabledReturnsAllTypesInCanonicalOrder)
{
  const std::map<std::string, bool> flags{
    {"unknown", true}, {"car", true},        {"truck", true},   {"bus", true},
    {"trailer", true}, {"motorcycle", true}, {"bicycle", true}, {"pedestrian", true}};

  const auto types = classification_types_from_flags(flags);

  const std::vector<uint8_t> expected{
    ObjectClassification::UNKNOWN, ObjectClassification::CAR,
    ObjectClassification::TRUCK,   ObjectClassification::BUS,
    ObjectClassification::TRAILER, ObjectClassification::MOTORCYCLE,
    ObjectClassification::BICYCLE, ObjectClassification::PEDESTRIAN};
  EXPECT_EQ(types, expected);
}

TEST(MvpUtilsClassification, NoneEnabledReturnsEmpty)
{
  const std::map<std::string, bool> flags{
    {"unknown", false}, {"car", false},        {"truck", false},   {"bus", false},
    {"trailer", false}, {"motorcycle", false}, {"bicycle", false}, {"pedestrian", false}};

  EXPECT_TRUE(classification_types_from_flags(flags).empty());
}

TEST(MvpUtilsClassification, UnknownClassNamesAreIgnored)
{
  const std::map<std::string, bool> flags{{"car", true}, {"spaceship", true}, {"submarine", true}};

  const auto types = classification_types_from_flags(flags);
  const std::vector<uint8_t> expected{ObjectClassification::CAR};
  EXPECT_EQ(types, expected);
}

TEST(MvpUtilsClassification, MissingFlagIsTreatedAsDisabled)
{
  // Only "truck" is present; all other classes default to disabled.
  const std::map<std::string, bool> flags{{"truck", true}};

  const auto types = classification_types_from_flags(flags);
  const std::vector<uint8_t> expected{ObjectClassification::TRUCK};
  EXPECT_EQ(types, expected);
}

// ----------------------------- calc_object_possible_max_dist_from_center -----------------------

TEST(MvpUtilsMaxDist, BoundingBoxReturnsHalfDiagonal)
{
  Shape shape;
  shape.type = Shape::BOUNDING_BOX;
  shape.dimensions.x = 4.0;
  shape.dimensions.y = 3.0;
  // half-diagonal = hypot(2, 1.5) = 2.5
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 2.5, 1e-9);
}

TEST(MvpUtilsMaxDist, CylinderReturnsRadius)
{
  Shape shape;
  shape.type = Shape::CYLINDER;
  shape.dimensions.x = 6.0;  // diameter
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 3.0, 1e-9);
}

TEST(MvpUtilsMaxDist, PolygonReturnsFarthestPointDistance)
{
  Shape shape;
  shape.type = Shape::POLYGON;
  const std::vector<std::pair<double, double>> corners{{1.0, 0.0}, {0.0, 2.0}, {-3.0, -4.0}};
  for (const auto & [x, y] : corners) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    shape.footprint.points.push_back(p);
  }
  // farthest point is (-3, -4): hypot = 5
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 5.0, 1e-6);
}

TEST(MvpUtilsMaxDist, EmptyPolygonReturnsZero)
{
  Shape shape;
  shape.type = Shape::POLYGON;
  EXPECT_NEAR(calc_object_possible_max_dist_from_center(shape), 0.0, 1e-9);
}

TEST(MvpUtilsMaxDist, UnsupportedShapeThrowsLogicError)
{
  Shape shape;
  shape.type = 255;  // not a supported shape type
  EXPECT_THROW(calc_object_possible_max_dist_from_center(shape), std::logic_error);
}

// ----------------------------- get_index_with_longitudinal_offset ------------------------------

TEST(MvpUtilsLongitudinalOffset, EmptyPointsThrows)
{
  const std::vector<TrajectoryPoint> empty_points;
  EXPECT_THROW(
    get_index_with_longitudinal_offset(empty_points, 1.0, std::nullopt), std::logic_error);
}

TEST(MvpUtilsLongitudinalOffset, StartIndexOutOfRangeThrows)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);
  EXPECT_THROW(
    get_index_with_longitudinal_offset(points, 1.0, std::optional<size_t>(3)), std::out_of_range);
}

TEST(MvpUtilsLongitudinalOffset, ForwardRoundsToNearerEndpoint)
{
  // points at x = 0, 1, 2, 3, 4 (1 m spacing). For a forward offset the function finds the segment
  // [i, i+1] whose cumulative length first reaches the offset, then returns the endpoint of that
  // segment that is closer to the offset position.
  const auto points = make_straight_forward_trajectory(5, 1.0);

  // offset 2.4: reached on segment [2, 3] (cumulative sum = 3.0 at i = 2). distance from the offset
  // to point 2 (front_length) = 0.4, to point 3 (back_length) = 0.6 -> point 2 is closer.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 2.4, std::nullopt), 2u);

  // offset 2.1: front_length = 0.1, back_length = 0.9 -> point 2 is closer.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 2.1, std::nullopt), 2u);

  // offset 2.6: front_length = 0.6, back_length = 0.4 -> point 3 is closer.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 2.6, std::nullopt), 3u);
}

TEST(MvpUtilsLongitudinalOffset, ForwardOffsetBeyondEndReturnsLastIndex)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // total length 4.0
  EXPECT_EQ(get_index_with_longitudinal_offset(points, 100.0, std::nullopt), 4u);
}

TEST(MvpUtilsLongitudinalOffset, BackwardFromDefaultStart)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);

  // Negative offset and no start_idx -> start from the last index (4) and walk backward.
  // offset -1.4: accumulate backward from index 4: i=4 covers segment [3, 4] (sum=1.0 < 1.4), i=3
  // covers segment [2, 3] (sum=2.0 >= 1.4), so the threshold is reached on segment [2, 3].
  // back_length = sum + offset = 2.0 - 1.4 = 0.6 (distance from offset to point 2 = points[i-1]),
  // front_length = seg - back = 1.0 - 0.6 = 0.4 (distance from offset to point 3 = points[i]).
  // front_length < back_length -> the offset is closer to point 3, so index i = 3 is returned.
  EXPECT_EQ(get_index_with_longitudinal_offset(points, -1.4, std::nullopt), 3u);
}

TEST(MvpUtilsLongitudinalOffset, BackwardOffsetBeyondStartReturnsZero)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // total length 4.0
  EXPECT_EQ(get_index_with_longitudinal_offset(points, -100.0, std::nullopt), 0u);
}

// ----------------------------- get_extended_trajectory_points ----------------------------------

TEST(MvpUtilsExtend, ShortExtendDistanceReturnsInputUnchanged)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);
  // extend_distance below the internal min_step_length (0.1) -> input returned as-is.
  const auto result = get_extended_trajectory_points(points, 0.05, 2.0);
  ASSERT_EQ(result.size(), points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_DOUBLE_EQ(result[i].pose.position.x, points[i].pose.position.x);
  }
}

TEST(MvpUtilsExtend, AppendsIntermediateAndFinalPoints)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);  // last point at x = 2.0
  const double extend_distance = 5.0;
  const double step_length = 2.0;

  const auto result = get_extended_trajectory_points(points, extend_distance, step_length);

  // loop pushes one point at extend_sum = 2.0 (2.0 < 5.0 - 2.0 = 3.0), then the final point at
  // exactly extend_distance.
  ASSERT_EQ(result.size(), points.size() + 2);
  EXPECT_NEAR(result[points.size()].pose.position.x, 2.0 + 2.0, 1e-6);      // x = 4.0
  EXPECT_NEAR(result.back().pose.position.x, 2.0 + extend_distance, 1e-6);  // x = 7.0
  // velocity is carried over from the goal point.
  EXPECT_DOUBLE_EQ(
    result.back().longitudinal_velocity_mps, points.back().longitudinal_velocity_mps);
}

TEST(MvpUtilsExtend, OnlyFinalPointWhenStepLargerThanDistance)
{
  const auto points = make_straight_forward_trajectory(3, 1.0);  // last point at x = 2.0
  const double extend_distance = 1.0;
  const double step_length = 2.0;

  // The loop condition (step_length < extend_distance - step_length -> 2 < -1) is false, so only
  // the single final point at extend_distance is appended.
  const auto result = get_extended_trajectory_points(points, extend_distance, step_length);
  ASSERT_EQ(result.size(), points.size() + 1);
  EXPECT_NEAR(result.back().pose.position.x, 2.0 + extend_distance, 1e-6);  // x = 3.0
}

// ----------------------------- calc_distance_to_front_object -----------------------------------

TEST(MvpUtilsFrontObject, ReturnsArcLengthForObjectAhead)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // x = 0..4
  // ego at index 1 (x = 1), obstacle ahead at x = 3.2 (nearest index 3).
  const auto dist = calc_distance_to_front_object(points, 1, make_point(3.2, 0.0));
  ASSERT_TRUE(dist.has_value());
  // signed arc length from index 1 (x=1) to nearest index of obstacle (x=3) is 2.0.
  EXPECT_NEAR(*dist, 2.0, 1e-6);
}

TEST(MvpUtilsFrontObject, ReturnsNulloptForObjectBehind)
{
  const auto points = make_straight_forward_trajectory(5, 1.0);  // x = 0..4
  // ego at index 3 (x = 3), obstacle behind at x = 0.2 (nearest index 0) -> negative arc length.
  const auto dist = calc_distance_to_front_object(points, 3, make_point(0.2, 0.0));
  EXPECT_FALSE(dist.has_value());
}

// ----------------------------- concat_vectors --------------------------------------------------

TEST(MvpUtilsConcat, ConcatenatesInOrder)
{
  const std::vector<int> a{1, 2, 3};
  const std::vector<int> b{4, 5};
  const auto result = concat_vectors(a, b);
  const std::vector<int> expected{1, 2, 3, 4, 5};
  EXPECT_EQ(result, expected);
}

TEST(MvpUtilsConcat, HandlesEmptyInputs)
{
  const std::vector<int> empty;
  const std::vector<int> b{7, 8};
  EXPECT_EQ(concat_vectors(empty, b), b);
  EXPECT_EQ(concat_vectors(b, empty), b);
  EXPECT_TRUE(concat_vectors(empty, empty).empty());
}
