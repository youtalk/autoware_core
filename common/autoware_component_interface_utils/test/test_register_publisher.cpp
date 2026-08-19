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

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace
{

// Test-local spec: the method is spec-generic, so no real domain spec is needed.
struct TestCloudSpec
{
  using Message = sensor_msgs::msg::PointCloud2;
  static constexpr char name[] = "/test/obstacle_cloud";
  static constexpr size_t depth = 5;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

using autoware::component_interface_utils::InterfaceRecord;
using autoware::component_interface_utils::NodeAdaptor;

class RegisterPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(RegisterPublisherTest, RegistersProviderWhenResolvedNameIsCanonical)
{
  auto node = std::make_shared<rclcpp::Node>("producer");
  NodeAdaptor adaptor(node.get());
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    TestCloudSpec::name, rclcpp::QoS(5).reliable().durability_volatile());
  EXPECT_TRUE(adaptor.register_publisher<TestCloudSpec>(pub));
  const auto records = adaptor.manifest();
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records.front().role, InterfaceRecord::Role::Provide);
  EXPECT_EQ(records.front().interface_name, std::string{TestCloudSpec::name});
  EXPECT_EQ(records.front().resolved_name, std::string{TestCloudSpec::name});
  EXPECT_EQ(records.front().depth, 5u);
}

TEST_F(RegisterPublisherTest, NoOpWhenResolvedNameDiffers)
{
  auto node = std::make_shared<rclcpp::Node>("producer");
  NodeAdaptor adaptor(node.get());
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/some/other/topic", rclcpp::QoS(5).reliable().durability_volatile());
  EXPECT_FALSE(adaptor.register_publisher<TestCloudSpec>(pub));
  EXPECT_TRUE(adaptor.manifest().empty());
}

TEST_F(RegisterPublisherTest, ThrowsWhenCanonicalNameCarriesNonSpecQos)
{
  auto node = std::make_shared<rclcpp::Node>("producer");
  NodeAdaptor adaptor(node.get());
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    TestCloudSpec::name, rclcpp::QoS(1).best_effort());
  EXPECT_THROW(adaptor.register_publisher<TestCloudSpec>(pub), std::runtime_error);
  EXPECT_TRUE(adaptor.manifest().empty());
}

}  // namespace
