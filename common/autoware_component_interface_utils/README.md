# autoware_component_interface_utils

## Features

This is a utility package that provides the following features:

- Instantiation of the wrapper class
- Polling a subscription for the newest message, an accumulated latest, or every queued message, each with DDS source-timestamp tracking
- Registration of every interface a node creates into a queryable manifest
- Serialization of that manifest to the admission manifest document schema
- Opt-in service introspection for service and client
- Service exception for response
- Relays for topic and service
- A test helper for pinning a package's committed manifest fragment against what a node actually registers

## Design

This package provides the wrappers for the interface classes of rclcpp.
The wrappers limit the usage of the original class to enforce the processing recommended by the component interface.
Do not inherit the class of rclcpp, and forward or wrap the member function that is allowed to be used.

## Instantiation of the wrapper class

The wrapper class requires interface information in this format.

```cpp
struct SampleService
{
  using Service = sample_msgs::srv::ServiceType;
  static constexpr char name[] = "/sample/service";
};

struct SampleMessage
{
  using Message = sample_msgs::msg::MessageType;
  static constexpr char name[] = "/sample/message";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};
```

Create the wrapper by asking the adaptor for it directly, giving the specification as an explicit template argument.
This is the current style: these are non-const member functions, so the adaptor must not be declared `const` when they are used.

```cpp
// source file
auto node = autoware::component_interface_utils::NodeAdaptor(this);
srv_ = node.create_service<SampleService>(callback);
cli_ = node.create_client<SampleService>();
pub_ = node.create_publisher<SampleMessage>();
sub_ = node.create_subscription<SampleMessage>(callback);
```

The adaptor also provides deducing forms that take the output variable instead of an explicit template argument (`init_srv`, `init_cli`, `init_pub`, `init_sub`).
They create the same wrappers as the `create_*` methods above, but they are the legacy form; see [Migrating from init_\*](#migrating-from-init_) for the `create_*` replacement of each overload.

```cpp
// header file
autoware::component_interface_utils::Service<SampleService>::SharedPtr srv_;
autoware::component_interface_utils::Client<SampleService>::SharedPtr cli_;
autoware::component_interface_utils::Publisher<SampleMessage>::SharedPtr pub_;
autoware::component_interface_utils::Subscription<SampleMessage>::SharedPtr sub_;

// source file
const auto node = autoware::component_interface_utils::NodeAdaptor(this);
node.init_srv(srv_, callback);  // legacy form, prefer create_service<SampleService>(callback)
node.init_cli(cli_);            // legacy form, prefer create_client<SampleService>()
node.init_pub(pub_);            // legacy form, prefer create_publisher<SampleMessage>()
node.init_sub(sub_, callback);  // legacy form, prefer create_subscription<SampleMessage>(callback)
```

## Migrating from init_*

Every `init_*` overload is the legacy form and has a `create_*` replacement that returns the wrapper instead of writing it into an output parameter.
Both styles register the interface identically, so migrating a call site is a mechanical rewrite with no behavior change.

| Legacy overload                                               | Replacement                                                                                   |
| ------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| `init_pub(pub)`                                               | `pub = create_publisher<Spec>()`                                                              |
| `init_sub(sub, callback)`                                     | `sub = create_subscription<Spec>(callback)`                                                   |
| `init_sub(sub, instance, &Instance::pointer_callback)`        | `sub = create_subscription<Spec>(std::bind(&Instance::pointer_callback, instance, _1))`       |
| `init_sub(sub, instance, &Instance::reference_callback)`      | `sub = create_subscription<Spec>(std::bind(&Instance::reference_callback, instance, _1))`     |
| `init_cli(cli, group)`                                        | `cli = create_client<Spec>(group)`                                                            |
| `init_srv(srv, callback, group)`                              | `srv = create_service<Spec>(callback, group)`                                                 |
| `init_srv(srv, instance, &Instance::service_callback, group)` | `srv = create_service<Spec>(std::bind(&Instance::service_callback, instance, _1, _2), group)` |

The `init_*` overloads are not removed, so existing call sites keep compiling as-is; `create_*` is preferred for new code, and existing call sites are expected to migrate over time.

## Polling a subscription

`Subscription` provides three ways to pull messages out of the middleware queue instead of dispatching through a callback:

- `take()` drains up to the subscription's QoS depth and returns only the **newest** message, discarding any older ones that were queued; it returns `nullptr` if nothing was queued.
- `take_and_update(ptr)` / `take_and_update(ref)` do the same drain-and-keep-newest as `take()`, but write the result into an existing pointer or reference instead of returning it, and report success as a `bool`.
- `take_all()` drains **every** currently queued message, oldest first, returning a `std::vector` of all of them; it returns an empty vector if nothing was queued.

`last_taken_data_timestamp()` returns the DDS **source timestamp** (`rclcpp::Time(message_info.get_rmw_message_info().source_timestamp, RCL_ROS_TIME)`) of the last message taken -- not the message header stamp and not the local reception time.
Its behavior on an unsuccessful take differs per method, matching each method's semantics:

| Method                 | On success                           | On failure / empty                       |
| ---------------------- | ------------------------------------ | ---------------------------------------- |
| `take()`               | set to the taken message's timestamp | **cleared** to `std::nullopt`            |
| `take_and_update(...)` | set                                  | **left untouched** (sticky)              |
| `take_all()`           | set to the **last** message taken    | **cleared** to `std::nullopt` when empty |

Before anything has been taken, `last_taken_data_timestamp()` is `std::nullopt`.

### Migrating from `autoware_utils_rclcpp::InterProcessPollingSubscriber`

`autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT, PollingPolicy>` offers the same three drain behaviors as its `polling_policy` template parameter, and `Subscription` mirrors their semantics exactly:

| `InterProcessPollingSubscriber` policy | `Subscription` equivalent                                                   |
| -------------------------------------- | --------------------------------------------------------------------------- |
| `polling_policy::Latest`               | `take_and_update(ptr)` / `take_and_update(ref)` + a member you keep updated |
| `polling_policy::Newest`               | `take()`                                                                    |
| `polling_policy::All`                  | `take_all()`                                                                |

One constraint motivates keeping `Subscription` around as the more general option: `InterProcessPollingSubscriber`'s `Latest` and `Newest` policies call `check_qos()` at construction, which throws `std::invalid_argument` when the QoS depth is greater than 1 -- so those two policies cannot be used at all with a spec that declares depth > 1.
`Subscription` has no such restriction; `take()` and `take_and_update()` both drain up to the actual QoS depth regardless of what that depth is.

## Opt-in service introspection for service and client

This package does not trace service calls by itself.
What it provides is a single node-scoped switch that turns on the standard [ROS 2 service introspection](https://docs.ros.org/en/jazzy/Tutorials/Demos/Service-Introspection.html) for every wrapper of that node, so the wrappers do not have to be configured one by one.
Each node that creates wrappers through `NodeAdaptor` declares the following parameter, and its value is applied to all the `Service` and `Client` wrappers that the adaptor creates.

| Name                                        | Type     | Default | Description                                                                       |
| ------------------------------------------- | -------- | ------- | --------------------------------------------------------------------------------- |
| `component_interface.service_introspection` | `string` | `off`   | Introspection mode of the service wrappers. One of `off`, `metadata`, `contents`. |

Introspection is disabled by default, so nothing is recorded unless the parameter is set.
When the mode is not `off`, each wrapper exposes the service event topic `<service name>/_service_event`, which is `/sample/service/_service_event` for the specification above.
The `metadata` mode records the event types and timestamps, and the `contents` mode also records the request and response contents.

Service introspection requires ROS 2 Iron (rclcpp 21) or later.
On ROS 2 Humble the parameter is not declared and the wrappers are created without introspection.

The `autoware_universe` version of this package published a `tier4_system_msgs/ServiceLog` message on a single `/service_log` topic for every request, response and error, and that tracer is not part of this package.
Service introspection is not a drop-in replacement for it: the events are published per service instead of being aggregated on one topic, they use the event types that ROS 2 generates instead of `tier4_system_msgs/ServiceLog`, and they are off unless enabled.
An unready service and a service timeout are still reported by `Client::call` as the `ServiceUnready` and `ServiceTimeout` exceptions.

## Service exception for response

If the wrapper class is used and the service response has status, throwing `ServiceException` will automatically catch and set it to status.
This is useful when returning an error from a function called from the service callback.

```cpp
void service_callback(Request req, Response res)
{
   function();
   res->status.success = true;
}

void function()
{
   throw ServiceException(ERROR_CODE, "message");
}
```

If the wrapper class is not used or the service response has no status, manually catch the `ServiceException` as follows.

```cpp
void service_callback(Request req, Response res)
{
   try {
      function();
      res->status.success = true;
   } catch (const ServiceException & error) {
      res->status = error.status();
   }
}
```

## Relays for topic and service

There are utilities for relaying services and messages of the same type.

```cpp
const auto node = autoware::component_interface_utils::NodeAdaptor(this);
service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
node.relay_message(pub_, sub_);
node.relay_service(cli_, srv_, service_callback_group_);
```

`relay_service`'s `group` argument is applied only to the relayed service server, never to the relayed client, and that split is intentional rather than an oversight.
A relayed service callback blocks in `Client::call` until the downstream response arrives.
If the client were placed in the same `MutuallyExclusive` group as that blocked server callback, a `MultiThreadedExecutor` could never run the client's response callback while the group's one thread is still stuck inside the server callback that is waiting for it, so the relay would deadlock for as long as the downstream call takes.
Keeping the client out of `group` is what avoids that deadlock.

## Registration and the node manifest

Every `create_*` (and, for now, every `init_*`) call registers an `InterfaceRecord` for the endpoint it creates: which spec, in which role (`Provide` for a publisher or service server, `Require` for a subscription or service client), the remap-resolved name, the rosidl type name, whether the spec declares a domain version (and if so, its major/minor/patch), and the actual QoS the middleware ended up using.
`autoware/component_interface_utils/rclcpp/registration.hpp` defines `InterfaceRecord` and `make_record<Spec>()`; the create/init implementation layer builds one record per endpoint and appends it under a mutex, so composed nodes that construct interfaces from multiple callback groups are safe.

`NodeAdaptor::manifest()` returns a snapshot vector of every record registered so far.
It is complete by the time the node's constructor returns, since every entry point registers unconditionally.
Every endpoint is created with the QoS its spec declares -- there is no per-call override -- so a record's QoS is the spec's QoS as the middleware resolved it.

`autoware/component_interface_utils/manifest_json.hpp` turns that vector into the admission manifest document schema: `to_manifest_json(node_name, records)` builds it from a raw vector, and `to_manifest_json(adaptor, node_name)` is the convenience form for a live adaptor.
The document has an `owner`, a `node_name`, and `provided`/`required` arrays; each entry carries `interface_name`/`resolved_name`/`type_name` and a `qos` object (`reliability` is `"reliable"` or `"best_effort"`, `durability` is `"volatile"` or `"transient_local"`, plus `depth`).
An unnameable QoS policy (for example the record's own `SYSTEM_DEFAULT` default, meaning nothing was ever actually applied) makes serialization throw `std::invalid_argument` rather than emit a placeholder.
Version fields are conditional on `has_version`: a versioned `provided[]` entry carries `major`/`minor`/`patch`, a versioned `required[]` entry carries the accepted range (`accept_major_min`, `accept_major_max`, `min_minor`), and an unversioned record omits all of them rather than emitting zeros.

## Registering an externally created publisher

`create_publisher<Spec>()` is the normal way to get a registered, spec-conformant publisher: the adaptor picks the spec's name and QoS itself, so the endpoint cannot diverge from the specification.
It requires the node to know the spec at construction time, which some nodes cannot: a generic pipeline node whose output topic is a relative name (for example `output`) repointed by launch-time remapping does not know until it is actually constructed -- and remapped -- whether the resulting resolved name happens to be a boundary this deployment publishes on a versioned interface.
For that case, `NodeAdaptor` provides `register_publisher<Spec>(publisher)`, which takes a publisher the node already created some other way and folds it into the manifest as that spec's provider, but only if the publisher's own resolved topic name actually is the spec's canonical name:

```cpp
auto node = autoware::component_interface_utils::NodeAdaptor(this);
auto pub = create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::QoS(5).reliable());
node.register_publisher<MyCloudSpec>(pub);  // no-op if "output" did not resolve to MyCloudSpec::name
```

- If the resolved name differs from `Spec::name`, `register_publisher` registers nothing and returns `false`; this is the ordinary case for most deployments of a generic node, where the topic is not the boundary.
- If the resolved name matches but the publisher's actual QoS (reliability, durability, depth) does not match the spec's QoS, it throws `std::runtime_error` instead of registering a record. A node that happens to land on the canonical name without meeting the spec's QoS is a mis-declared boundary, and this fails node startup loudly rather than letting the manifest record a provider that silently diverges from the specification.
- On a name match with conforming QoS, it registers a `Provide` record exactly like `create_publisher<Spec>()` would and returns `true`.

Prefer `create_publisher<Spec>()` whenever the node can name its spec at construction time; reach for `register_publisher<Spec>()` only at the boundary case above, where the publisher already exists and the spec match can only be known after remapping.

## Interface manifest fragments

A package that wants its nodes' manifests checked by a deploy-time admission gate commits an interface manifest fragment: `config/interface_manifest_fragment.json`, containing either one manifest document (matching the schema above) for a single-node package, or a JSON array of them for a package with multiple nodes.
The fragment is installed to `share/<pkg>/` like any other data file, so the gate can discover it alongside every other installed package's fragment without the package needing to register itself anywhere else.

The gate-discoverable location is the fixed relative path `share/<package_name>/interface_manifest_fragment.json` -- install the file directly there with a `FILES` rule, not through a subdirectory-copying rule such as `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`, which would nest it one level deeper (`share/<package_name>/config/interface_manifest_fragment.json`) instead, where the gate does not look:

```cmake
install(
  FILES config/interface_manifest_fragment.json
  DESTINATION share/${PROJECT_NAME}
)
```

Pin the fragment against reality in a unit test, using `expect_manifest_matches` from `autoware/component_interface_utils/testing/manifest_drift.hpp`:

```cpp
#include "autoware/component_interface_utils/testing/manifest_drift.hpp"

TEST(my_node, manifest_matches_the_committed_fragment)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("my_node");
  autoware::component_interface_utils::NodeAdaptor adaptor(node.get());
  auto pub = adaptor.create_publisher<MySpec>();  // the same registrations MyNode's constructor makes
  autoware::component_interface_utils::testing::expect_manifest_matches(
    adaptor, node->get_fully_qualified_name(), MY_FRAGMENT_PATH);
  rclcpp::shutdown();
}
```

`expect_manifest_matches` parses the fragment with nlohmann and compares it to the node's actual manifest as JSON values, so formatting and key order in the fragment do not matter; on a mismatch it fails with `EXPECT_EQ` and dumps the actual document.
The header pulls in gtest, so any target that includes it must link gtest itself; this package adds no new dependency of its own for it.

The test above builds its own node and its own `NodeAdaptor`, rather than reaching into an already-constructed instance of a package's real node class, and that is not a stylistic choice.
`NodeAdaptor::manifest()` only reflects what has been registered through that same adaptor instance, and every in-tree node builds its `NodeAdaptor` as a local inside the node's own constructor, where it goes out of scope the moment construction returns; there is no accessor that reaches it afterward.
The fragment-pinning convention works only because the unit test constructs the node and the adaptor itself and keeps both alive across the call to `expect_manifest_matches` -- it is not a way to query the manifest of an already-running node.
