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

#include "autoware/component_interface_utils/rclcpp.hpp"
#include "gtest/gtest.h"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <memory>

namespace ciu = autoware::component_interface_utils;

namespace test_specs
{
// Local spec with a reliable/volatile pivot.
struct PivotRV
{
  using Message = autoware_adapi_v1_msgs::msg::ResponseStatus;
  static constexpr char name[] = "/test/pivot_rv";
  static constexpr std::size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};
}  // namespace test_specs

class QosOverrideTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_qos_override");
    adaptor_ = std::make_unique<ciu::NodeAdaptor>(node_.get());
  }
  void TearDown() override { rclcpp::shutdown(); }
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<ciu::NodeAdaptor> adaptor_;
};

TEST_F(QosOverrideTest, stronger_offer_is_accepted_and_recorded)
{
  auto pub =
    adaptor_->create_publisher<test_specs::PivotRV>(rclcpp::QoS(1).reliable().transient_local());
  const auto manifest = adaptor_->manifest();
  ASSERT_EQ(manifest.size(), 1u);
  EXPECT_EQ(manifest[0].durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  (void)pub;
}

TEST_F(QosOverrideTest, weaker_offer_throws)
{
  EXPECT_THROW(
    adaptor_->create_publisher<test_specs::PivotRV>(rclcpp::QoS(1).best_effort()),
    ciu::QosContractViolation);
  // Pins that validation precedes registration, not just creation: a
  // refactor that moved the check after the publisher was constructed but
  // before register_interface() would still throw here and still pass, but
  // would leave a live publisher on the ROS graph.
  EXPECT_TRUE(adaptor_->manifest().empty());
}

TEST_F(QosOverrideTest, weaker_request_is_accepted)
{
  auto sub = adaptor_->create_subscription<test_specs::PivotRV>(
    [](const test_specs::PivotRV::Message &) {}, rclcpp::QoS(1).best_effort());
  const auto manifest = adaptor_->manifest();
  ASSERT_EQ(manifest.size(), 1u);
  // Pins that the override actually reaches rclcpp::create_subscription: the
  // pivot is RELIABLE and InterfaceRecord defaults reliability to
  // SYSTEM_DEFAULT, so BEST_EFFORT can only appear here if the requested QoS
  // -- not the spec's -- was applied and reflected back from the live
  // subscription.
  EXPECT_EQ(manifest[0].reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  (void)sub;
}

TEST_F(QosOverrideTest, stronger_request_throws)
{
  EXPECT_THROW(
    adaptor_->create_subscription<test_specs::PivotRV>(
      [](const test_specs::PivotRV::Message &) {}, rclcpp::QoS(1).reliable().transient_local()),
    ciu::QosContractViolation);
  EXPECT_TRUE(adaptor_->manifest().empty());
}

TEST_F(QosOverrideTest, incomparable_offer_throws)
{
  // SystemDefaultsQoS carries RMW_QOS_POLICY_*_SYSTEM_DEFAULT on both axes,
  // which ranks outside the two policies the pivot compares (fail closed):
  // neither "at least" nor "at most" the pivot can be established, so the
  // provider check must reject it rather than silently accept it.
  EXPECT_THROW(
    adaptor_->create_publisher<test_specs::PivotRV>(rclcpp::SystemDefaultsQoS()),
    ciu::QosContractViolation);
  EXPECT_TRUE(adaptor_->manifest().empty());
}

TEST_F(QosOverrideTest, depth_is_free)
{
  auto pub = adaptor_->create_publisher<test_specs::PivotRV>(rclcpp::QoS(42).reliable());
  const auto manifest = adaptor_->manifest();
  ASSERT_EQ(manifest.size(), 1u);
  EXPECT_EQ(manifest[0].depth, 42u);
  (void)pub;
}
