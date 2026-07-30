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

#include "autoware/component_interface_specs/qos_compatibility.hpp"
#include "autoware/component_interface_specs/system.hpp"
#include "gtest/gtest.h"

namespace cis = autoware::component_interface_specs;

namespace
{
constexpr auto kReliable = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
constexpr auto kBestEffort = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
constexpr auto kVolatile = RMW_QOS_POLICY_DURABILITY_VOLATILE;
constexpr auto kTransientLocal = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
}  // namespace

// The RMW enum integers do NOT follow semantic strength order
// (BEST_EFFORT = 2 > RELIABLE = 1); the ranks must invert that.
TEST(qos_compatibility, ranks_follow_strength_not_enum_integers)
{
  static_assert(kBestEffort > kReliable, "enum-integer trap the ranks must not fall into");
  EXPECT_GT(cis::reliability_rank(kReliable), cis::reliability_rank(kBestEffort));
  EXPECT_GT(cis::durability_rank(kTransientLocal), cis::durability_rank(kVolatile));
}

TEST(qos_compatibility, at_least_is_the_dds_partial_order)
{
  EXPECT_TRUE(cis::reliability_at_least(kReliable, kBestEffort));
  EXPECT_TRUE(cis::reliability_at_least(kReliable, kReliable));
  EXPECT_FALSE(cis::reliability_at_least(kBestEffort, kReliable));
  EXPECT_TRUE(cis::durability_at_least(kTransientLocal, kVolatile));
  EXPECT_FALSE(cis::durability_at_least(kVolatile, kTransientLocal));
}

// A connection forms iff offered >= requested on every axis.
TEST(qos_compatibility, compatibility_needs_every_axis)
{
  EXPECT_TRUE(cis::is_qos_compatible({kReliable, kTransientLocal}, {kBestEffort, kVolatile}));
  EXPECT_TRUE(cis::is_qos_compatible({kReliable, kVolatile}, {kReliable, kVolatile}));
  // The user-facing canonical case: a BEST_EFFORT offer cannot serve a RELIABLE request.
  EXPECT_FALSE(cis::is_qos_compatible({kBestEffort, kVolatile}, {kReliable, kVolatile}));
  EXPECT_FALSE(cis::is_qos_compatible({kReliable, kVolatile}, {kReliable, kTransientLocal}));
}

// SYSTEM_DEFAULT / UNKNOWN are incomparable: every check fails (fail closed).
TEST(qos_compatibility, unknown_policies_fail_closed)
{
  constexpr auto kSysRel = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  constexpr auto kSysDur = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  EXPECT_FALSE(cis::reliability_at_least(kSysRel, kBestEffort));
  EXPECT_FALSE(cis::reliability_at_least(kReliable, kSysRel));
  EXPECT_FALSE(cis::is_qos_compatible({kSysRel, kVolatile}, {kBestEffort, kVolatile}));
  EXPECT_FALSE(cis::is_qos_compatible({kReliable, kSysDur}, {kReliable, kVolatile}));
}

// Pivot rule: provider offers >= pivot, consumer requests <= pivot.
TEST(qos_compatibility, pivot_rule)
{
  constexpr cis::QosPair pivot{kReliable, kVolatile};
  EXPECT_TRUE(cis::provider_satisfies_pivot({kReliable, kTransientLocal}, pivot));
  EXPECT_TRUE(cis::provider_satisfies_pivot({kReliable, kVolatile}, pivot));
  EXPECT_FALSE(cis::provider_satisfies_pivot({kBestEffort, kVolatile}, pivot));
  EXPECT_TRUE(cis::consumer_satisfies_pivot({kBestEffort, kVolatile}, pivot));
  EXPECT_FALSE(cis::consumer_satisfies_pivot({kReliable, kTransientLocal}, pivot));
}

// pivot_of<Spec>() reads the spec's declared members; everything is constexpr.
TEST(qos_compatibility, pivot_of_spec_is_constexpr)
{
  using Spec = cis::system::OperationModeState;
  constexpr auto pivot = cis::pivot_of<Spec>();
  static_assert(pivot.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  static_assert(cis::provider_satisfies_pivot(pivot, pivot));
  EXPECT_EQ(pivot.reliability, Spec::reliability);
}
