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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__QOS_COMPATIBILITY_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__QOS_COMPATIBILITY_HPP_

#include <rmw/types.h>

namespace autoware::component_interface_specs
{

/// Reliability/durability pair: the two QoS axes DDS checks for endpoint
/// compatibility, and the two every spec declares. Depth/history are
/// endpoint-local and deliberately absent.
struct QosPair
{
  rmw_qos_reliability_policy_t reliability;
  rmw_qos_durability_policy_t durability;
};

/// Strength rank of a reliability policy. RELIABLE delivers everything
/// BEST_EFFORT does and more, so it ranks higher. The RMW enum integer values
/// do NOT follow this order (BEST_EFFORT = 2 > RELIABLE = 1) -- never compare
/// raw enums. Policies outside the two the specs can declare (SYSTEM_DEFAULT,
/// UNKNOWN, ...) rank -1: incomparable, so every check involving them fails
/// (fail closed).
///
/// The admission package restates these ranks on the JSON string encoding
/// (it is a no-dependency leaf that cannot include this header). Change one
/// and the other must follow: autoware_component_interface_admission,
/// admission_rule.hpp.
constexpr int reliability_rank(rmw_qos_reliability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return 0;
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return 1;
    default:
      return -1;
  }
}

constexpr int durability_rank(rmw_qos_durability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return 0;
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return 1;
    default:
      return -1;
  }
}

constexpr bool reliability_at_least(rmw_qos_reliability_policy_t a, rmw_qos_reliability_policy_t b)
{
  return reliability_rank(a) >= 0 && reliability_rank(b) >= 0 &&
         reliability_rank(a) >= reliability_rank(b);
}

constexpr bool durability_at_least(rmw_qos_durability_policy_t a, rmw_qos_durability_policy_t b)
{
  return durability_rank(a) >= 0 && durability_rank(b) >= 0 &&
         durability_rank(a) >= durability_rank(b);
}

/// DDS request-vs-offered compatibility: a connection forms iff the offered
/// QoS is at least as strong as the requested QoS on every axis.
constexpr bool is_qos_compatible(const QosPair & offered, const QosPair & requested)
{
  return reliability_at_least(offered.reliability, requested.reliability) &&
         durability_at_least(offered.durability, requested.durability);
}

/// The pivot rule. A spec's declared QoS is a pivot, not an exact-match
/// requirement: a provider must offer at least the pivot and a consumer must
/// request at most the pivot. Transitivity of the DDS partial order then
/// guarantees every conforming provider/consumer pair connects
/// (offered >= pivot >= requested).
constexpr bool provider_satisfies_pivot(const QosPair & offered, const QosPair & pivot)
{
  return is_qos_compatible(offered, pivot);
}

constexpr bool consumer_satisfies_pivot(const QosPair & requested, const QosPair & pivot)
{
  return is_qos_compatible(pivot, requested);
}

/// The pivot a topic spec declares.
template <class SpecT>
constexpr QosPair pivot_of()
{
  return QosPair{SpecT::reliability, SpecT::durability};
}

}  // namespace autoware::component_interface_specs

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__QOS_COMPATIBILITY_HPP_
