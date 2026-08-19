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

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

// Test-local spec: the class under test is spec-generic, so no real domain spec is needed.
// depth > 1 is deliberate: the real ObstacleSegmentation spec is not on this branch, and depth 1
// would not exercise take_all()'s ability to drain more than one queued message.
struct TestCloudSpec
{
  using Message = sensor_msgs::msg::PointCloud2;
  static constexpr char name[] = "/test/polling_cloud";
  static constexpr size_t depth = 5;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

using autoware::component_interface_utils::NodeAdaptor;
using Message = TestCloudSpec::Message;
using SubscriptionPtr = autoware::component_interface_utils::Subscription<TestCloudSpec>::SharedPtr;
using PublisherPtr = autoware::component_interface_utils::Publisher<TestCloudSpec>::SharedPtr;

/// Graph discovery of even local endpoints is asynchronous, so poll until the topic has at
/// least one publisher (or the timeout elapses) rather than assume it is immediately visible.
bool wait_for_publisher(const rclcpp::Node::SharedPtr & node, const std::string & topic)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!node->get_publishers_info_by_topic(topic).empty()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

/// Poll take_all() until it has drained at least `count` messages (or the timeout elapses),
/// accumulating across calls since a single call may race the DDS delivery of a batch publish.
std::vector<Message::ConstSharedPtr> wait_and_take_all(const SubscriptionPtr & sub, size_t count)
{
  std::vector<Message::ConstSharedPtr> collected;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (collected.size() < count && std::chrono::steady_clock::now() < deadline) {
    auto batch = sub->take_all();
    collected.insert(collected.end(), batch.begin(), batch.end());
    if (collected.size() < count) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  return collected;
}

/// Poll take() until it returns non-null (or the timeout elapses).
Message::ConstSharedPtr wait_and_take(const SubscriptionPtr & sub)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto msg = sub->take()) {
      return msg;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return nullptr;
}

Message make_message(uint32_t width)
{
  Message msg;
  msg.width = width;
  return msg;
}

class TopicSubscriptionPollingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_topic_subscription_polling");
    NodeAdaptor adaptor(node_.get());
    pub_ = adaptor.create_publisher<TestCloudSpec>();
    sub_ = adaptor.create_subscription<TestCloudSpec>(nullptr);
    ASSERT_TRUE(wait_for_publisher(node_, TestCloudSpec::name));
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  PublisherPtr pub_;
  SubscriptionPtr sub_;
};

TEST_F(TopicSubscriptionPollingTest, TakeAllReturnsEveryQueuedMessageInOrder)
{
  pub_->publish(make_message(1));
  pub_->publish(make_message(2));
  pub_->publish(make_message(3));

  const auto collected = wait_and_take_all(sub_, 3);

  ASSERT_EQ(collected.size(), 3u);
  EXPECT_EQ(collected[0]->width, 1u);
  EXPECT_EQ(collected[1]->width, 2u);
  EXPECT_EQ(collected[2]->width, 3u);
}

TEST_F(TopicSubscriptionPollingTest, TakeAllReturnsEmptyVectorWhenNothingQueued)
{
  const auto collected = sub_->take_all();
  EXPECT_TRUE(collected.empty());
}

TEST_F(TopicSubscriptionPollingTest, TakeStillReturnsOnlyTheNewestMessage)
{
  pub_->publish(make_message(1));
  pub_->publish(make_message(2));
  pub_->publish(make_message(3));

  // Give all three time to actually queue up before taking, since take() (unlike take_all())
  // is not accumulated across polls in this test -- it is meant to drain in one shot.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  const auto msg = sub_->take();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->width, 3u);
}

TEST_F(TopicSubscriptionPollingTest, TimestampIsNulloptBeforeAnythingIsTaken)
{
  EXPECT_FALSE(sub_->last_taken_data_timestamp().has_value());
}

TEST_F(TopicSubscriptionPollingTest, TakeClearsTimestampOnSubsequentFailure)
{
  pub_->publish(make_message(1));
  ASSERT_NE(wait_and_take(sub_), nullptr);
  EXPECT_TRUE(sub_->last_taken_data_timestamp().has_value());

  // Nothing new was published, so this take() fails and must clear the timestamp
  // (Newest parity).
  EXPECT_EQ(sub_->take(), nullptr);
  EXPECT_FALSE(sub_->last_taken_data_timestamp().has_value());
}

TEST_F(TopicSubscriptionPollingTest, TakeAndUpdateRetainsTimestampOnSubsequentFailure)
{
  pub_->publish(make_message(1));

  Message::ConstSharedPtr ptr;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool took = false;
  while (!took && std::chrono::steady_clock::now() < deadline) {
    took = sub_->take_and_update(ptr);
    if (!took) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_TRUE(took);
  ASSERT_TRUE(sub_->last_taken_data_timestamp().has_value());
  const auto first_timestamp = sub_->last_taken_data_timestamp().value();

  // Nothing new was published, so this take_and_update() fails, but the timestamp must be
  // retained (Latest parity) -- this asymmetry with take() is the crux of the parity mapping.
  EXPECT_FALSE(sub_->take_and_update(ptr));
  ASSERT_TRUE(sub_->last_taken_data_timestamp().has_value());
  EXPECT_EQ(sub_->last_taken_data_timestamp().value(), first_timestamp);
}

TEST_F(TopicSubscriptionPollingTest, TakeAllClearsTimestampWhenItReturnsEmpty)
{
  pub_->publish(make_message(1));
  ASSERT_FALSE(wait_and_take_all(sub_, 1).empty());
  EXPECT_TRUE(sub_->last_taken_data_timestamp().has_value());

  // Nothing new was published, so this take_all() returns empty and must clear the timestamp
  // (All parity).
  EXPECT_TRUE(sub_->take_all().empty());
  EXPECT_FALSE(sub_->last_taken_data_timestamp().has_value());
}

}  // namespace
