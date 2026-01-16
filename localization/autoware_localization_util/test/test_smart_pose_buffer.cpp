// Copyright 2023- Autoware Foundation
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

#include "autoware/localization_util/smart_pose_buffer.hpp"
#include "autoware/localization_util/util_func.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using SmartPoseBuffer = autoware::localization_util::SmartPoseBuffer;

bool compare_pose(
  const PoseWithCovarianceStamped & pose_a, const PoseWithCovarianceStamped & pose_b)
{
  return pose_a.header.stamp == pose_b.header.stamp &&
         pose_a.header.frame_id == pose_b.header.frame_id &&
         pose_a.pose.covariance == pose_b.pose.covariance &&
         pose_a.pose.pose.position.x == pose_b.pose.pose.position.x &&
         pose_a.pose.pose.position.y == pose_b.pose.pose.position.y &&
         pose_a.pose.pose.position.z == pose_b.pose.pose.position.z &&
         pose_a.pose.pose.orientation.x == pose_b.pose.pose.orientation.x &&
         pose_a.pose.pose.orientation.y == pose_b.pose.pose.orientation.y &&
         pose_a.pose.pose.orientation.z == pose_b.pose.pose.orientation.z &&
         pose_a.pose.pose.orientation.w == pose_b.pose.pose.orientation.w;
}

TEST(TestSmartPoseBuffer, interpolate_pose)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double pose_timeout_sec = 10.0;
  const double pose_distance_tolerance_meters = 100.0;
  SmartPoseBuffer smart_pose_buffer(logger, pose_timeout_sec, pose_distance_tolerance_meters);

  // first data
  PoseWithCovarianceStamped::SharedPtr old_pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  old_pose_ptr->header.stamp.sec = 10;
  old_pose_ptr->header.stamp.nanosec = 0;
  old_pose_ptr->pose.pose.position.x = 10.0;
  old_pose_ptr->pose.pose.position.y = 20.0;
  old_pose_ptr->pose.pose.position.z = 0.0;
  old_pose_ptr->pose.pose.orientation =
    autoware::localization_util::rpy_deg_to_quaternion(0.0, 0.0, 0.0);
  smart_pose_buffer.push_back(old_pose_ptr);

  // second data
  PoseWithCovarianceStamped::SharedPtr new_pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  new_pose_ptr->header.stamp.sec = 20;
  new_pose_ptr->header.stamp.nanosec = 0;
  new_pose_ptr->pose.pose.position.x = 20.0;
  new_pose_ptr->pose.pose.position.y = 40.0;
  new_pose_ptr->pose.pose.position.z = 0.0;
  new_pose_ptr->pose.pose.orientation =
    autoware::localization_util::rpy_deg_to_quaternion(0.0, 0.0, 90.0);
  smart_pose_buffer.push_back(new_pose_ptr);

  // interpolate
  builtin_interfaces::msg::Time target_ros_time_msg;
  target_ros_time_msg.sec = 15;
  target_ros_time_msg.nanosec = 0;
  const std::optional<SmartPoseBuffer::InterpolateResult> & interpolate_result =
    smart_pose_buffer.interpolate(target_ros_time_msg);
  ASSERT_TRUE(interpolate_result.has_value());
  const SmartPoseBuffer::InterpolateResult result = interpolate_result.value();

  // check old
  EXPECT_TRUE(compare_pose(result.old_pose, *old_pose_ptr));

  // check new
  EXPECT_TRUE(compare_pose(result.new_pose, *new_pose_ptr));

  // check interpolated
  EXPECT_EQ(result.interpolated_pose.header.stamp.sec, 15);
  EXPECT_EQ(result.interpolated_pose.header.stamp.nanosec, static_cast<uint32_t>(0));
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.x, 15.0);
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.y, 30.0);
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.z, 0.0);
  const auto rpy = autoware::localization_util::get_rpy(result.interpolated_pose.pose.pose);
  EXPECT_NEAR(rpy.x * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.y * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.z * 180 / M_PI, 45.0, 1e-6);
}

TEST(TestSmartPoseBuffer, empty_buffer)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 1.0, 1.0);

  builtin_interfaces::msg::Time target_time;
  target_time.sec = 0;
  target_time.nanosec = 0;

  // Test empty buffer
  auto result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());

  // Test buffer with only one element
  auto pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  pose_ptr->header.stamp = target_time;
  smart_pose_buffer.push_back(pose_ptr);

  result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());
}

TEST(TestSmartPoseBuffer, timeout_validation)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double timeout = 1.0;  // 1 second timeout
  SmartPoseBuffer smart_pose_buffer(logger, timeout, 100.0);

  // Add two poses with 0.5 sec difference
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 0;
  pose1->header.stamp.nanosec = 0;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 0;
  pose2->header.stamp.nanosec = 500000000;  // 0.5 sec
  smart_pose_buffer.push_back(pose2);

  // Test target time within timeout
  builtin_interfaces::msg::Time target_time1;
  target_time1.sec = 0;
  target_time1.nanosec = 250000000;  // 0.25 sec
  auto result1 = smart_pose_buffer.interpolate(target_time1);
  EXPECT_TRUE(result1.has_value());

  // Test target time beyond timeout
  builtin_interfaces::msg::Time target_time2;
  target_time2.sec = 2;  // 2 sec (beyond 1 sec timeout)
  target_time2.nanosec = 0;
  auto result2 = smart_pose_buffer.interpolate(target_time2);
  EXPECT_FALSE(result2.has_value());
}

TEST(TestSmartPoseBuffer, position_tolerance_validation)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double tolerance = 1.0;  // 1 meter tolerance
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, tolerance);

  // Add two poses within tolerance
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 0;
  pose1->pose.pose.position.x = 0;
  pose1->pose.pose.position.y = 0;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 1;
  pose2->pose.pose.position.x = 0.5;  // 0.5m distance
  pose2->pose.pose.position.y = 0;
  smart_pose_buffer.push_back(pose2);

  builtin_interfaces::msg::Time target_time;
  target_time.sec = 0;
  target_time.nanosec = 500000000;  // 0.5 sec
  auto result1 = smart_pose_buffer.interpolate(target_time);
  EXPECT_TRUE(result1.has_value());

  // Add a pose beyond tolerance
  auto pose3 = std::make_shared<PoseWithCovarianceStamped>();
  pose3->header.stamp.sec = 2;
  pose3->pose.pose.position.x = 2.0;  // 2m distance (beyond 1m tolerance)
  pose3->pose.pose.position.y = 0;
  smart_pose_buffer.push_back(pose3);

  target_time.sec = 1;
  auto result2 = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result2.has_value());
}

TEST(TestSmartPoseBuffer, buffer_operations)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, 10.0);

  // Test pop_old
  for (int i = 0; i < 5; ++i) {
    auto pose = std::make_shared<PoseWithCovarianceStamped>();
    pose->header.stamp.sec = i;
    smart_pose_buffer.push_back(pose);
  }

  builtin_interfaces::msg::Time pop_time;
  pop_time.sec = 2;
  smart_pose_buffer.pop_old(pop_time);

  builtin_interfaces::msg::Time target_time;
  target_time.sec = 1;
  auto result1 = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result1.has_value());  // Should fail because we popped too much

  target_time.sec = 3;
  auto result2 = smart_pose_buffer.interpolate(target_time);
  EXPECT_TRUE(result2.has_value());

  // Test clear
  smart_pose_buffer.clear();
  auto result3 = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result3.has_value());
}

TEST(TestSmartPoseBuffer, non_chronological_timestamps)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, 10.0);

  // Add poses in order
  for (int i = 0; i < 3; ++i) {
    auto pose = std::make_shared<PoseWithCovarianceStamped>();
    pose->header.stamp.sec = i;
    smart_pose_buffer.push_back(pose);
  }

  // Add pose with older timestamp (should clear buffer)
  auto old_pose = std::make_shared<PoseWithCovarianceStamped>();
  old_pose->header.stamp.sec = 0;
  smart_pose_buffer.push_back(old_pose);

  // Buffer should now only contain the old_pose
  builtin_interfaces::msg::Time target_time;
  target_time.sec = 1;
  auto result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());  // Not enough poses in buffer
}

TEST(TestSmartPoseBuffer, target_time_before_first_pose)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 10.0, 10.0);

  // Add two poses
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 10;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 20;
  smart_pose_buffer.push_back(pose2);

  // Test target time before first pose
  builtin_interfaces::msg::Time target_time;
  target_time.sec = 5;
  auto result = smart_pose_buffer.interpolate(target_time);
  EXPECT_FALSE(result.has_value());
}

TEST(TestSmartPoseBuffer, target_time_after_last_pose)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double timeout = 1.0;
  SmartPoseBuffer smart_pose_buffer(logger, timeout, 10.0);

  // Add two poses
  auto pose1 = std::make_shared<PoseWithCovarianceStamped>();
  pose1->header.stamp.sec = 10;
  smart_pose_buffer.push_back(pose1);

  auto pose2 = std::make_shared<PoseWithCovarianceStamped>();
  pose2->header.stamp.sec = 11;
  smart_pose_buffer.push_back(pose2);

  // Test target time slightly after last pose (within timeout)
  builtin_interfaces::msg::Time target_time1;
  target_time1.sec = 11;
  target_time1.nanosec = 500000000;  // 11.5 sec
  auto result1 = smart_pose_buffer.interpolate(target_time1);
  EXPECT_TRUE(result1.has_value());

  // Test target time well after last pose (beyond timeout)
  builtin_interfaces::msg::Time target_time2;
  target_time2.sec = 12;  // 12 sec (beyond 1 sec timeout)
  auto result2 = smart_pose_buffer.interpolate(target_time2);
  EXPECT_FALSE(result2.has_value());
}

// ==============================================================================
// Multithreading Tests
// ==============================================================================

TEST(TestSmartPoseBuffer, concurrent_push_back)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 1000.0, 1000.0);

  constexpr int num_threads = 4;
  constexpr int poses_per_thread = 100;
  std::vector<std::thread> threads;

  // Each thread pushes poses with non-overlapping timestamps
  for (int t = 0; t < num_threads; ++t) {
    threads.emplace_back([&smart_pose_buffer, t]() {
      for (int i = 0; i < poses_per_thread; ++i) {
        auto pose = std::make_shared<PoseWithCovarianceStamped>();
        // Use unique timestamps: thread 0 gets 0-99, thread 1 gets 100-199, etc.
        pose->header.stamp.sec = t * poses_per_thread + i;
        pose->pose.pose.position.x = static_cast<double>(t * poses_per_thread + i);
        smart_pose_buffer.push_back(pose);
      }
    });
  }

  for (auto & thread : threads) {
    thread.join();
  }

  // Due to non-chronological timestamp handling, buffer should still be functional
  // The buffer clears when non-chronological data arrives, so we just verify no crash occurred
  SUCCEED();
}

TEST(TestSmartPoseBuffer, concurrent_read_write)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 100.0, 1000.0);

  // Pre-populate with initial poses
  for (int i = 0; i < 10; ++i) {
    auto pose = std::make_shared<PoseWithCovarianceStamped>();
    pose->header.stamp.sec = i;
    pose->pose.pose.position.x = static_cast<double>(i);
    smart_pose_buffer.push_back(pose);
  }

  std::atomic<bool> stop_flag{false};
  std::atomic<int> write_count{0};
  std::atomic<int> read_count{0};

  // Writer thread: continuously pushes new poses
  std::thread writer([&]() {
    int timestamp = 10;
    while (!stop_flag.load()) {
      auto pose = std::make_shared<PoseWithCovarianceStamped>();
      pose->header.stamp.sec = timestamp++;
      pose->pose.pose.position.x = static_cast<double>(timestamp);
      smart_pose_buffer.push_back(pose);
      write_count.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  // Reader thread: continuously reads/interpolates
  std::thread reader([&]() {
    while (!stop_flag.load()) {
      builtin_interfaces::msg::Time target_time;
      target_time.sec = 5;
      target_time.nanosec = 0;
      // Just call interpolate - we don't care about the result, just that it doesn't crash
      [[maybe_unused]] auto result = smart_pose_buffer.interpolate(target_time);
      read_count.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });

  // Run for a short duration
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_flag.store(true);

  writer.join();
  reader.join();

  // Verify both threads did work
  EXPECT_GT(write_count.load(), 0);
  EXPECT_GT(read_count.load(), 0);
}

TEST(TestSmartPoseBuffer, concurrent_pop_and_push)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 1000.0, 1000.0);

  // Pre-populate with initial poses
  for (int i = 0; i < 100; ++i) {
    auto pose = std::make_shared<PoseWithCovarianceStamped>();
    pose->header.stamp.sec = i;
    pose->pose.pose.position.x = static_cast<double>(i);
    smart_pose_buffer.push_back(pose);
  }

  std::atomic<bool> stop_flag{false};
  std::atomic<int> push_count{0};
  std::atomic<int> pop_count{0};

  // Pusher thread: adds new poses with increasing timestamps
  std::thread pusher([&]() {
    int timestamp = 100;
    while (!stop_flag.load()) {
      auto pose = std::make_shared<PoseWithCovarianceStamped>();
      pose->header.stamp.sec = timestamp++;
      pose->pose.pose.position.x = static_cast<double>(timestamp);
      smart_pose_buffer.push_back(pose);
      push_count.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  // Popper thread: removes old poses
  std::thread popper([&]() {
    int pop_time = 0;
    while (!stop_flag.load()) {
      builtin_interfaces::msg::Time target_time;
      target_time.sec = pop_time++;
      target_time.nanosec = 0;
      smart_pose_buffer.pop_old(target_time);
      pop_count.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::microseconds(150));
    }
  });

  // Run for a short duration
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_flag.store(true);

  pusher.join();
  popper.join();

  // Verify both threads did work
  EXPECT_GT(push_count.load(), 0);
  EXPECT_GT(pop_count.load(), 0);
}

TEST(TestSmartPoseBuffer, concurrent_clear_and_operations)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 1000.0, 1000.0);

  std::atomic<bool> stop_flag{false};
  std::atomic<int> operation_count{0};

  // Thread 1: push_back and interpolate
  std::thread operator_thread([&]() {
    int timestamp = 0;
    while (!stop_flag.load()) {
      auto pose = std::make_shared<PoseWithCovarianceStamped>();
      pose->header.stamp.sec = timestamp++;
      pose->pose.pose.position.x = static_cast<double>(timestamp);
      smart_pose_buffer.push_back(pose);

      builtin_interfaces::msg::Time target_time;
      target_time.sec = timestamp / 2;
      [[maybe_unused]] auto result = smart_pose_buffer.interpolate(target_time);

      operation_count.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });

  // Thread 2: periodically clears the buffer
  std::thread clearer([&]() {
    while (!stop_flag.load()) {
      smart_pose_buffer.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // Run for a short duration
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_flag.store(true);

  operator_thread.join();
  clearer.join();

  EXPECT_GT(operation_count.load(), 0);
}

TEST(TestSmartPoseBuffer, stress_test_all_operations)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  SmartPoseBuffer smart_pose_buffer(logger, 1000.0, 1000.0);

  constexpr int num_threads = 8;
  std::atomic<bool> stop_flag{false};
  std::atomic<int> total_operations{0};
  std::vector<std::thread> threads;

  // Create threads that perform various operations
  for (int t = 0; t < num_threads; ++t) {
    threads.emplace_back([&, t]() {
      int local_timestamp = t * 1000;
      while (!stop_flag.load()) {
        switch (t % 4) {
          case 0: {
            // push_back
            auto pose = std::make_shared<PoseWithCovarianceStamped>();
            pose->header.stamp.sec = local_timestamp++;
            smart_pose_buffer.push_back(pose);
            break;
          }
          case 1: {
            // interpolate
            builtin_interfaces::msg::Time target;
            target.sec = local_timestamp / 2;
            [[maybe_unused]] auto r = smart_pose_buffer.interpolate(target);
            break;
          }
          case 2: {
            // pop_old
            builtin_interfaces::msg::Time target;
            target.sec = local_timestamp / 4;
            smart_pose_buffer.pop_old(target);
            break;
          }
          case 3: {
            // clear (less frequently)
            if (local_timestamp % 100 == 0) {
              smart_pose_buffer.clear();
            }
            local_timestamp++;
            break;
          }
        }
        total_operations.fetch_add(1);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    });
  }

  // Run stress test for a longer duration
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop_flag.store(true);

  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_GT(total_operations.load(), 100);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
