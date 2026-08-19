// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::component_interface_utils
{

/// The wrapper class of a subscription. The handle type comes from the node's
/// create_subscription(). take(), take_and_update() and take_all() additionally need the handle
/// to provide rclcpp's take(), and being ordinary members they are only instantiated when
/// called.
template <class SpecT, class NodeT = rclcpp::Node>
class Subscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)
  using SpecType = SpecT;
  using NodeType = NodeT;
  using WrapSharedPtr =
    decltype(std::declval<NodeT &>().template create_subscription<typename SpecT::Message>(
      std::declval<const std::string &>(), std::declval<const rclcpp::QoS &>(),
      std::declval<std::function<void(const typename SpecT::Message &)>>()));
  using WrapType = typename WrapSharedPtr::element_type;

  /// Constructor.
  explicit Subscription(WrapSharedPtr subscription)
  {
    subscription_ = subscription;  // to keep the reference count
  }

  /// Take the newest queued message, discarding any older ones still queued (Newest policy
  /// parity). On success, last_taken_data_timestamp() is set to the taken message's DDS source
  /// timestamp; on failure (nothing queued), it is cleared to std::nullopt.
  typename SpecType::Message::ConstSharedPtr take()
  {
    const auto result = drain_newest();
    if (!result) {
      last_taken_data_timestamp_ = std::nullopt;
      return nullptr;
    }
    last_taken_data_timestamp_ = result->second;
    return result->first;
  }

  /// Take the newest queued message like take(), but write it into an existing pointer instead
  /// of returning it (Latest policy parity). On success, last_taken_data_timestamp() is set; on
  /// failure it is left untouched (sticky), unlike take().
  bool take_and_update(typename SpecType::Message::ConstSharedPtr & ptr)
  {
    const auto result = drain_newest();
    if (!result) {
      return false;
    }
    ptr = result->first;
    last_taken_data_timestamp_ = result->second;
    return true;
  }

  /// Take the newest queued message like take(), but write it into an existing reference instead
  /// of returning it (Latest policy parity). On success, last_taken_data_timestamp() is set; on
  /// failure it is left untouched (sticky), unlike take().
  bool take_and_update(typename SpecType::Message & ref)
  {
    const auto result = drain_newest();
    if (!result) {
      return false;
    }
    ref = *result->first;
    last_taken_data_timestamp_ = result->second;
    return true;
  }

  /// Take every currently queued message, oldest first (All policy parity). Unlike take(), this
  /// drains without a depth cap, so nothing queued is silently discarded. On success,
  /// last_taken_data_timestamp() is set to the last message taken; if nothing was queued, an
  /// empty vector is returned and the timestamp is cleared to std::nullopt.
  std::vector<typename SpecType::Message::ConstSharedPtr> take_all()
  {
    std::vector<typename SpecType::Message::ConstSharedPtr> data;
    rclcpp::MessageInfo info;
    for (;;) {
      auto datum = std::make_shared<typename SpecType::Message>();
      if (!subscription_->take(*datum, info)) {
        break;
      }
      data.push_back(datum);
      last_taken_data_timestamp_ =
        rclcpp::Time(info.get_rmw_message_info().source_timestamp, RCL_ROS_TIME);
    }
    if (data.empty()) {
      last_taken_data_timestamp_ = std::nullopt;
    }
    return data;
  }

  /// The DDS source timestamp of the last message taken, following the per-method clearing
  /// rules documented on take(), take_and_update() and take_all(). std::nullopt before anything
  /// has been taken.
  std::optional<rclcpp::Time> last_taken_data_timestamp() const
  {
    return last_taken_data_timestamp_;
  }

private:
  RCLCPP_DISABLE_COPY(Subscription)

  /// Drain up to depth queued messages, keeping only the newest -- the same depth-capped loop
  /// take() has always used. Returns the newest message together with its DDS source timestamp
  /// on success, or std::nullopt if nothing was queued. Does not touch
  /// last_taken_data_timestamp_; callers apply their own policy (take() clears on failure,
  /// take_and_update() retains).
  std::optional<std::pair<typename SpecType::Message::ConstSharedPtr, rclcpp::Time>> drain_newest()
  {
    rclcpp::MessageInfo info;
    auto data = std::make_shared<typename SpecType::Message>();
    bool flag = false;
    for (size_t i = 0; i < subscription_->get_actual_qos().depth(); ++i) {
      if (!subscription_->take(*data, info)) {
        break;
      }
      flag = true;  // Whether there is at least one data.
    }
    if (!flag) {
      return std::nullopt;
    }
    return std::make_pair(
      typename SpecType::Message::ConstSharedPtr(data),
      rclcpp::Time(info.get_rmw_message_info().source_timestamp, RCL_ROS_TIME));
  }

  WrapSharedPtr subscription_;
  std::optional<rclcpp::Time> last_taken_data_timestamp_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_
