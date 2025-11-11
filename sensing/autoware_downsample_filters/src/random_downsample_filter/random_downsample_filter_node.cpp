// Copyright 2024 TIER IV, Inc.
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

#include "random_downsample_filter_node.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::downsample_filters
{
RandomDownsampleFilter::RandomDownsampleFilter(const rclcpp::NodeOptions & options)
: Node("RandomDownsampleFilter", options),
  tf_input_frame_(declare_parameter<std::string>("input_frame")),
  tf_output_frame_(declare_parameter<std::string>("output_frame")),
  sample_num_(static_cast<size_t>(declare_parameter<int64_t>("sample_num"))),
  max_queue_size_(static_cast<size_t>(declare_parameter<int64_t>("max_queue_size")))
{
  {
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Filter (as Component) successfully created with the following parameters:"
        << std::endl
        << " - max_queue_size   : " << max_queue_size_ << std::endl
        << " - sample_num       : " << sample_num_);
  }

  // Set publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);

    published_time_publisher_ =
      std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
  }

  // Set subscriber
  {
    sub_input_ = create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
      std::bind(&RandomDownsampleFilter::input_callback, this, std::placeholders::_1));
    transform_listener_ = std::make_unique<autoware_utils_tf::TransformListener>(this);
  }

  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

void RandomDownsampleFilter::input_callback(const PointCloud2ConstPtr cloud)
{
  // Null pointer check for input cloud
  if (!cloud) {
    RCLCPP_ERROR(this->get_logger(), "[input_callback] Received null pointer for cloud data");
    return;
  }

  // If cloud is given, check if it's valid
  if (!is_valid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_callback] Invalid input!");
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "[input_callback] PointCloud with %d data points and frame %s on input topic "
    "received.",
    cloud->width * cloud->height, cloud->header.frame_id.c_str());

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2ConstPtr cloud_tf;

  try {
    if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
      RCLCPP_DEBUG(
        this->get_logger(), "[input_callback] Transforming input dataset from %s to %s.",
        cloud->header.frame_id.c_str(), tf_input_frame_.c_str());

      // Save the original frame ID
      // Convert the cloud into the different frame
      auto cloud_transformed = std::make_unique<PointCloud2>();

      // FIX: Use cloud->header instead of uninitialized cloud_tf->header
      auto tf_ptr = transform_listener_->get_transform(
        tf_input_frame_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(1.0));
      if (!tf_ptr) {
        RCLCPP_ERROR(
          this->get_logger(), "[input_callback] Error converting dataset from %s to %s.",
          cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
        return;
      }

      auto eigen_tf = tf2::transformToEigen(*tf_ptr);
      pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *cloud, *cloud_transformed);
      cloud_tf = std::move(cloud_transformed);

    } else {
      cloud_tf = cloud;
    }

    // Validation: ensure cloud_tf was properly assigned
    if (!cloud_tf) {
      RCLCPP_ERROR(this->get_logger(), "[input_callback] Failed to set cloud_tf");
      return;
    }

    compute_publish(cloud_tf);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[input_callback] Exception occurred during processing: %s", e.what());
    return;
  }
}

bool RandomDownsampleFilter::is_valid(const PointCloud2ConstPtr & cloud)
{
  // Check for null pointer
  if (!cloud) {
    RCLCPP_WARN(this->get_logger(), "Invalid PointCloud: received null pointer");
    return false;
  }

  // Validate point cloud data consistency
  if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, "
      "and frame %s received!",
      cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str());
    return false;
  }
  return true;
}

void RandomDownsampleFilter::compute_publish(const PointCloud2ConstPtr & input)
{
  // Validate input
  if (!input) {
    RCLCPP_ERROR(this->get_logger(), "[compute_publish] Received null pointer for input cloud");
    return;
  }

  try {
    auto output = std::make_unique<PointCloud2>();
    filter(input, *output);

    if (!output) {
      RCLCPP_ERROR(this->get_logger(), "[compute_publish] Filter produced null output");
      return;
    }

    if (!convert_output_costly(output)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[compute_publish] Output frame conversion failed, skipping publication");
      return;
    }

    // Copy timestamp to keep it
    output->header.stamp = input->header.stamp;

    // Publish a boost shared ptr
    pub_output_->publish(std::move(output));
    published_time_publisher_->publish_if_subscribed(pub_output_, input->header.stamp);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[compute_publish] Exception during compute_publish: %s", e.what());
    return;
  }
}

bool RandomDownsampleFilter::convert_output_costly(std::unique_ptr<PointCloud2> & output)
{
  // In terms of performance, we should avoid using pcl_ros library function,
  // but this code path isn't reached in the main use case of Autoware, so it's left as is for now.
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s to %s.",
      output->header.frame_id.c_str(), tf_output_frame_.c_str());

    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<PointCloud2>();

    auto tf_ptr = transform_listener_->get_transform(
      tf_output_frame_, output->header.frame_id, output->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
    if (!tf_ptr) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s to %s.",
        output->header.frame_id.c_str(), tf_output_frame_.c_str());
      return false;
    }

    auto eigen_tf = tf2::transformToEigen(*tf_ptr);
    pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *output, *cloud_transformed);
    output = std::move(cloud_transformed);
  }

  // Same as the comment above
  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    // No tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s back to %s.",
      output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());

    auto cloud_transformed = std::make_unique<sensor_msgs::msg::PointCloud2>();

    auto tf_ptr = transform_listener_->get_transform(
      tf_input_orig_frame_, output->header.frame_id, output->header.stamp,
      rclcpp::Duration::from_seconds(1.0));

    if (!tf_ptr) {
      return false;
    }

    auto eigen_tf = tf2::transformToEigen(*tf_ptr);
    pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *output, *cloud_transformed);
    output = std::move(cloud_transformed);
  }

  return true;
}

void RandomDownsampleFilter::filter(const PointCloud2ConstPtr & input, PointCloud2 & output)
{
  // Validate input pointer
  if (!input) {
    RCLCPP_ERROR(this->get_logger(), "[filter] Received null pointer for input cloud");
    return;
  }

  try {
    std::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert ROS message to PCL point cloud
    pcl::fromROSMsg(*input, *pcl_input);

    // Validate that conversion was successful
    if (!pcl_input || pcl_input->empty()) {
      RCLCPP_WARN(this->get_logger(), "[filter] PCL conversion resulted in empty point cloud");
      output = *input;  // Pass through the original cloud if filtering fails
      return;
    }

    pcl_output->points.reserve(pcl_input->points.size());
    pcl::RandomSample<pcl::PointXYZ> filter;
    filter.setInputCloud(pcl_input);
    // filter.setSaveLeafLayout(true);
    filter.setSample(sample_num_);
    filter.filter(*pcl_output);

    pcl::toROSMsg(*pcl_output, output);
    output.header = input->header;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "[filter] Exception during filtering: %s", e.what());
    // Graceful degradation: output the input cloud as-is if filtering fails
    output = *input;
  }
}
}  // namespace autoware::downsample_filters
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::downsample_filters::RandomDownsampleFilter)
