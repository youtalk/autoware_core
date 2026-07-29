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

#ifndef SPEC_TEST_UTILS_HPP_
#define SPEC_TEST_UTILS_HPP_

#include "autoware/component_interface_specs/concepts.hpp"
#include "gtest/gtest.h"

#include <rclcpp/qos.hpp>

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace autoware::component_interface_specs::test_utils
{

/// True when T is one of the alternatives of the std::tuple Tuple.
template <typename T, typename Tuple>
struct has_type;
template <typename T, typename... Us>
struct has_type<T, std::tuple<Us...>> : std::disjunction<std::is_same<T, Us>...>
{
};

/// Pin a topic spec's declared QoS members and the rclcpp::QoS that get_qos<Spec>()
/// derives from them. Every expected value is supplied by the caller as a literal, so the
/// assertions are never checked against the spec's own fields.
template <class Spec>
void expect_topic_qos(
  const char * expected_name, std::size_t expected_depth,
  rmw_qos_reliability_policy_t expected_reliability,
  rmw_qos_durability_policy_t expected_durability)
{
  EXPECT_STREQ(Spec::name, expected_name);
  EXPECT_EQ(Spec::depth, expected_depth);
  EXPECT_EQ(Spec::reliability, expected_reliability);
  EXPECT_EQ(Spec::durability, expected_durability);

  const auto qos = get_qos<Spec>();
  EXPECT_EQ(qos.depth(), expected_depth);
  EXPECT_EQ(qos.reliability(), static_cast<rclcpp::ReliabilityPolicy>(expected_reliability));
  EXPECT_EQ(qos.durability(), static_cast<rclcpp::DurabilityPolicy>(expected_durability));
}

}  // namespace autoware::component_interface_specs::test_utils

#endif  // SPEC_TEST_UTILS_HPP_
