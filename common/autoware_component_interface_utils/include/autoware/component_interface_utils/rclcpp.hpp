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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_

#include <autoware/component_interface_specs/qos_compatibility.hpp>
#include <autoware/component_interface_utils/rclcpp/create_interface.hpp>
#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <autoware/component_interface_utils/rclcpp/service_client.hpp>
#include <autoware/component_interface_utils/rclcpp/service_server.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_publisher.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_subscription.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::component_interface_utils
{

class NodeAdaptor
{
private:
  using CallbackGroup = rclcpp::CallbackGroup::SharedPtr;

  template <class SharedPtrT, class InstanceT>
  using MessagePtrCallback =
    void (InstanceT::*)(const typename SharedPtrT::element_type::SpecType::Message::ConstSharedPtr);
  template <class SharedPtrT, class InstanceT>
  using MessageRefCallback =
    void (InstanceT::*)(const typename SharedPtrT::element_type::SpecType::Message &);

  template <class SharedPtrT, class InstanceT>
  using ServiceCallback = void (InstanceT::*)(
    const typename SharedPtrT::element_type::SpecType::Service::Request::SharedPtr,
    const typename SharedPtrT::element_type::SpecType::Service::Response::SharedPtr);

public:
  /// Constructor.
  explicit NodeAdaptor(rclcpp::Node * node) { interface_ = std::make_shared<NodeInterface>(node); }

  /// Create a client wrapper for logging.
  template <class SharedPtrT>
  [[deprecated("use create_client<Spec>()")]] void init_cli(
    SharedPtrT & cli, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    cli = create_client_impl<SpecT>(interface_, group);
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class CallbackT>
  [[deprecated("use create_service<Spec>()")]] void init_srv(
    SharedPtrT & srv, CallbackT && callback, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    srv = create_service_impl<SpecT>(interface_, std::forward<CallbackT>(callback), group);
  }

  /// Create a publisher using traits like services.
  template <class SharedPtrT>
  [[deprecated("use create_publisher<Spec>()")]] void init_pub(SharedPtrT & pub) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    pub = create_publisher_impl<SpecT>(interface_);
  }

  /// Create a subscription using traits like services.
  template <class SharedPtrT, class CallbackT>
  [[deprecated("use create_subscription<Spec>()")]] void init_sub(
    SharedPtrT & sub, CallbackT && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    sub = create_subscription_impl<SpecT>(interface_, std::forward<CallbackT>(callback));
  }

  /// Relay message. Goes through the impls directly rather than init_pub/init_sub
  /// so that this helper neither triggers a deprecation warning nor bypasses
  /// interface registration.
  template <class P, class S>
  void relay_message(P & pub, S & sub) const
  {
    using SpecT = typename P::element_type::SpecType;
    using MsgT = typename SpecT::Message::ConstSharedPtr;
    pub = create_publisher_impl<SpecT>(interface_);
    using SubSpecT = typename S::element_type::SpecType;
    sub = create_subscription_impl<SubSpecT>(interface_, [pub](MsgT msg) { pub->publish(*msg); });
  }

  /// Relay service. Goes through the impls directly rather than init_cli/init_srv
  /// so that this helper neither triggers a deprecation warning nor bypasses
  /// interface registration. The client is deliberately left out of `group`:
  /// only the service goes into the caller's (typically MutuallyExclusive)
  /// callback group, so the client's response can still be taken while the
  /// service callback that issued the call is blocked in Client::call.
  /// Putting both in the same group reintroduces the deadlock `group` exists
  /// to prevent.
  template <class C, class S>
  void relay_service(
    C & cli, S & srv, CallbackGroup group, std::optional<double> timeout = std::nullopt) const
  {
    using SpecT = typename C::element_type::SpecType;
    cli = create_client_impl<SpecT>(interface_);
    using SrvSpecT = typename S::element_type::SpecType;
    srv = create_service_impl<SrvSpecT>(
      interface_, [cli, timeout](auto req, auto res) { *res = *cli->call(req, timeout); }, group);
  }

  /// Create a subscription wrapper for pointer callback.
  template <class SharedPtrT, class InstanceT>
  [[deprecated("use create_subscription<Spec>()")]] void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessagePtrCallback<SharedPtrT, InstanceT> && callback) const
  {
    using std::placeholders::_1;
    init_sub(sub, std::bind(callback, instance, _1));
  }

  /// Create a subscription wrapper for reference callback.
  template <class SharedPtrT, class InstanceT>
  [[deprecated("use create_subscription<Spec>()")]] void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessageRefCallback<SharedPtrT, InstanceT> && callback) const
  {
    using std::placeholders::_1;
    init_sub(sub, std::bind(callback, instance, _1));
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class InstanceT>
  [[deprecated("use create_service<Spec>()")]] void init_srv(
    SharedPtrT & srv, InstanceT * instance, ServiceCallback<SharedPtrT, InstanceT> && callback,
    CallbackGroup group = nullptr) const
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    init_srv(srv, std::bind(callback, instance, _1, _2), group);
  }

  /// Create a publisher and register it in this node's interface manifest.
  template <class SpecT>
  typename Publisher<SpecT>::SharedPtr create_publisher()
  {
    return create_publisher_impl<SpecT>(interface_);
  }

  /// Create a publisher with an explicit QoS override, validated against the
  /// spec's pivot before the publisher is constructed: a provider must offer
  /// at least the pivot, so a weaker override is rejected before anything is
  /// created or registered. Depth is not a compatibility axis and is free. A
  /// policy the pivot cannot rank against (e.g. SYSTEM_DEFAULT, BEST_AVAILABLE)
  /// is rejected too: incomparable fails closed rather than passing silently.
  template <class SpecT>
  typename Publisher<SpecT>::SharedPtr create_publisher(const rclcpp::QoS & offered)
  {
    namespace specs = autoware::component_interface_specs;
    const auto & profile = offered.get_rmw_qos_profile();
    if (!specs::provider_satisfies_pivot(
          specs::QosPair{profile.reliability, profile.durability}, specs::pivot_of<SpecT>())) {
      throw QosContractViolation(
        std::string(SpecT::name) + ": the offered QoS is weaker than the spec pivot");
    }
    return create_publisher_impl<SpecT>(interface_, offered);
  }

  /// Create a subscription and register it in this node's interface manifest.
  template <class SpecT, class CallbackT>
  typename Subscription<SpecT>::SharedPtr create_subscription(CallbackT && callback)
  {
    return create_subscription_impl<SpecT>(interface_, std::forward<CallbackT>(callback));
  }

  /// Create a subscription with an explicit QoS override, validated against
  /// the spec's pivot before the subscription is constructed: a consumer must
  /// request at most the pivot, so a stronger override is rejected before
  /// anything is created or registered. Depth is not a compatibility axis and
  /// is free. A policy the pivot cannot rank against (e.g. SYSTEM_DEFAULT,
  /// BEST_AVAILABLE) is rejected too: incomparable fails closed rather than
  /// passing silently.
  template <class SpecT, class CallbackT>
  typename Subscription<SpecT>::SharedPtr create_subscription(
    CallbackT && callback, const rclcpp::QoS & requested)
  {
    namespace specs = autoware::component_interface_specs;
    const auto & profile = requested.get_rmw_qos_profile();
    if (!specs::consumer_satisfies_pivot(
          specs::QosPair{profile.reliability, profile.durability}, specs::pivot_of<SpecT>())) {
      throw QosContractViolation(
        std::string(SpecT::name) + ": the requested QoS is stronger than the spec pivot");
    }
    return create_subscription_impl<SpecT>(
      interface_, std::forward<CallbackT>(callback), requested);
  }

  /// Create a service server and register it in this node's interface manifest.
  template <class SpecT, class CallbackT>
  typename Service<SpecT>::SharedPtr create_service(
    CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return create_service_impl<SpecT>(interface_, std::forward<CallbackT>(callback), group);
  }

  /// Create a service client and register it in this node's interface manifest.
  template <class SpecT>
  typename Client<SpecT>::SharedPtr create_client(rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return create_client_impl<SpecT>(interface_, group);
  }

  /// Snapshot of every interface this adaptor registered.
  std::vector<InterfaceRecord> manifest() const { return interface_->manifest(); }

private:
  // Use a node pointer because shared_from_this cannot be used in constructor.
  NodeInterface::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
