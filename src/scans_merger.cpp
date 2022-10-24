/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "laser_scan_merger/scans_merger.h"

using namespace laser_scan_merger;
using namespace std;

ScansMerger::ScansMerger(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local) {
  nh_ = nh;
  nh_local_ = nh_local;
  p_active_ = false;

  front_scan_received_ = false;
  rear_scan_received_ = false;

  front_scan_error_ = false;
  rear_scan_error_ = false;

  params_srv_ = nh_->create_service<std_srvs::srv::Empty>("params", 
                                                          std::bind(
                                                                &ScansMerger::updateParams,
                                                                this, 
                                                                std::placeholders::_1,
                                                                std::placeholders::_2,
                                                                std::placeholders::_3
                                                          ));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  initialize();
}

ScansMerger::~ScansMerger() {
//   nh_local_.deleteParam("active");
//   nh_local_.deleteParam("publish_scan");
//   nh_local_.deleteParam("publish_pcl");

//   nh_local_.deleteParam("ranges_num");

//   nh_local_.deleteParam("min_scanner_range");
//   nh_local_.deleteParam("max_scanner_range");

//   nh_local_.deleteParam("min_x_range");
//   nh_local_.deleteParam("max_x_range");
//   nh_local_.deleteParam("min_y_range");
//   nh_local_.deleteParam("max_y_range");

//   nh_local_.deleteParam("fixed_frame_id");
//   nh_local_.deleteParam("target_frame_id");
}

void ScansMerger::updateParamsUtil(){
  bool prev_active = p_active_;
  nh_->declare_parameter("active", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("publish_scan", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("publish_pcl", rclcpp::PARAMETER_BOOL);

  nh_->declare_parameter("ranges_num", rclcpp::PARAMETER_INTEGER);

  nh_->declare_parameter("min_scanner_range", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_scanner_range", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("min_x_range", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_x_range", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("min_y_range", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_y_range", rclcpp::PARAMETER_DOUBLE);

  nh_->declare_parameter("fixed_frame_id", rclcpp::PARAMETER_STRING);
  nh_->declare_parameter("fixed_frame_id", rclcpp::PARAMETER_STRING);

  nh_->get_parameter_or("active", p_active_, true);
  nh_->get_parameter_or("publish_scan", p_publish_scan_, false);
  nh_->get_parameter_or("publish_pcl", p_publish_pcl_, true);

  nh_->get_parameter_or("ranges_num", p_ranges_num_, 1000);
  
  nh_->get_parameter_or("min_scanner_range", p_min_scanner_range_, 0.05);
  nh_->get_parameter_or("max_scanner_range", p_max_scanner_range_, 10.0);
  nh_->get_parameter_or("min_x_range", p_min_x_range_, -10.0);
  nh_->get_parameter_or("max_x_range", p_max_x_range_,  10.0);
  nh_->get_parameter_or("min_y_range", p_min_y_range_, -10.0);
  nh_->get_parameter_or("max_y_range", p_max_y_range_,  10.0);

  nh_->get_parameter_or("fixed_frame_id", p_fixed_frame_id_, std::string{"map"});
  nh_->get_parameter_or("target_frame_id", p_target_frame_id_, std::string{"robot"});

  if (p_active_ != prev_active) {
    if (p_active_) {
      front_scan_sub_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        "front_scan", 10, std::bind(&ScansMerger::frontScanCallback, this, std::placeholders::_1));
      rear_scan_sub_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        "rear_scan", 10, std::bind(&ScansMerger::rearScanCallback, this, std::placeholders::_1));
      scan_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      pcl_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("pcl", 10);
    }
    else {
    //   front_scan_sub_.shutdown();
    //   rear_scan_sub_.shutdown();
    //   scan_pub_.shutdown();
    //   pcl_pub_.shutdown();
    }
  }
}

void ScansMerger::updateParams(const std::shared_ptr<rmw_request_id_t> request_header, 
                               const std::shared_ptr<std_srvs::srv::Empty::Request> &req, 
                               const std::shared_ptr<std_srvs::srv::Empty::Response> &res) {
  updateParamsUtil();
}

void ScansMerger::frontScanCallback(sensor_msgs::msg::LaserScan::SharedPtr front_scan) {
  projector_.projectLaser(*front_scan, front_pcl_);

  front_scan_received_ = true;
  front_scan_error_ = false;

  if (rear_scan_received_ || rear_scan_error_)
    publishMessages();
  else
    rear_scan_error_ = true;
}

void ScansMerger::rearScanCallback(sensor_msgs::msg::LaserScan::SharedPtr rear_scan) {
  projector_.projectLaser(*rear_scan, rear_pcl_);

  rear_scan_received_ = true;
  rear_scan_error_ = false;

  if (front_scan_received_ || front_scan_error_)
    publishMessages();
  else
    front_scan_error_ = true;
}

void ScansMerger::publishMessages() {
  auto now = nh_->get_clock()->now();

  vector<float> ranges;
//   vector<geometry_msgs::msg::Point32> points;
  vector<float> range_values;
  sensor_msgs::msg::ChannelFloat32 range_channel;
  range_channel.name = "range";

  sensor_msgs::msg::PointCloud2 new_front_pcl, new_rear_pcl;

  ranges.assign(p_ranges_num_, nanf("")); // Assign nan values

  if (!front_scan_error_) {
    // tf::StampedTransform target_to_lidar;
    geometry_msgs::msg::TransformStamped target_to_lidar;
    try {
    //   tf_ls_.waitForTransform(p_target_frame_id_, now, front_pcl_.header.frame_id, front_pcl_.header.stamp, p_fixed_frame_id_, ros::Duration(0.05));
    //   tf_ls_.lookupTransform(p_target_frame_id_, now, front_pcl_.header.frame_id, front_pcl_.header.stamp, p_fixed_frame_id_, target_to_lidar);
    //   tf_ls_.transformPointCloud(p_target_frame_id_, now, front_pcl_, p_fixed_frame_id_, new_front_pcl);
      target_to_lidar = tf_buffer_->lookupTransform(p_target_frame_id_, front_pcl_.header.frame_id, tf2::TimePointZero);
    //   tf2::fromMsg(target_to_lidar);
      tf2::doTransform(front_pcl_, new_front_pcl, target_to_lidar);
    }
    catch (tf2::TransformException& ex) {
      return;
    }
    tf2::Transform target_to_lidar_tf;
    tf2::convert<geometry_msgs::msg::Transform>(target_to_lidar.transform, target_to_lidar_tf);
    const tf2::Vector3 target_to_lidar_origin = target_to_lidar_tf.getOrigin();
    RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Front lidar origin (frame " << front_pcl_.header.frame_id << ") is at (" << target_to_lidar_origin.getX() << ", " << target_to_lidar_origin.getY() << ") w.r.t. frame " << p_target_frame_id_);
    // ROS_INFO_STREAM_ONCE("Front lidar origin (frame " << front_pcl_.header.frame_id << ") is at (" << target_to_lidar_origin.getX() << ", " << target_to_lidar_origin.getY() << ") w.r.t. frame " << p_target_frame_id_);

    const size_t number_of_points = new_front_pcl.height * new_front_pcl.width;
    sensor_msgs::PointCloud2Iterator<float> iter_x(new_front_pcl, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(new_front_pcl, "y");
    for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y){
    //   auto point_copy = Point((*iter_x)), (*iter_y));      
      
      double point_x = (*iter_x);
      double point_y = (*iter_y);
      const double range_x = point_x - target_to_lidar_origin.getX();
      const double range_y = point_y - target_to_lidar_origin.getY();
      const double range = sqrt(pow(range_x, 2.0) + pow(range_y, 2.0));

      if (range_x > p_min_x_range_ && range_x < p_max_x_range_ &&
          range_y > p_min_y_range_ && range_y < p_max_y_range_ &&
          range > p_min_scanner_range_ && range < p_max_scanner_range_) {
        // if (p_publish_pcl_) {
        //   points.push_back(point);
        //   range_channel.values.push_back(range);
        // }

        if (p_publish_scan_) {
          double angle = atan2(range_y, range_x);
          size_t idx = static_cast<int>(p_ranges_num_ * (angle + M_PI) / (2.0 * M_PI));
          if (isnan(ranges[idx]) || range < ranges[idx])
            ranges[idx] = range;
        }
      }
    }
  }

  if (!rear_scan_error_) {
    geometry_msgs::msg::TransformStamped target_to_lidar;
    try {
     target_to_lidar = tf_buffer_->lookupTransform(p_target_frame_id_, rear_pcl_.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(rear_pcl_, new_rear_pcl, target_to_lidar);
    }
    catch (tf2::TransformException& ex) {
      return;
    }
    tf2::Transform target_to_lidar_tf;
    tf2::convert<geometry_msgs::msg::Transform>(target_to_lidar.transform, target_to_lidar_tf);
    const tf2::Vector3 target_to_lidar_origin = target_to_lidar_tf.getOrigin();
    RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Rear lidar origin (frame " << front_pcl_.header.frame_id << ") is at (" << target_to_lidar_origin.getX() << ", " << target_to_lidar_origin.getY() << ") w.r.t. frame " << p_target_frame_id_);
    
    const size_t number_of_points = new_rear_pcl.height * new_rear_pcl.width;
    sensor_msgs::PointCloud2Iterator<float> iter_x(new_rear_pcl, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(new_rear_pcl, "y");
    for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y){
      double point_x = (*iter_x);
      double point_y = (*iter_y);
      const double range_x = point_x - target_to_lidar_origin.getX();
      const double range_y = point_y - target_to_lidar_origin.getY();
      const double range = sqrt(pow(range_x, 2.0) + pow(range_y, 2.0));

      if (range_x > p_min_x_range_ && range_x < p_max_x_range_ &&
          range_y > p_min_y_range_ && range_y < p_max_y_range_ &&
          range > p_min_scanner_range_ && range < p_max_scanner_range_) {
        // if (p_publish_pcl_) {
        //   points.push_back(point);
        //   range_channel.values.push_back(range);
        // }

        if (p_publish_scan_) {
          double angle = atan2(range_y, range_x);
          size_t idx = static_cast<int>(p_ranges_num_ * (angle + M_PI) / (2.0 * M_PI));
          if (isnan(ranges[idx]) || range < ranges[idx])
            ranges[idx] = range;
        }
      }
    }
  }
  if (p_publish_scan_) {
    sensor_msgs::msg::LaserScan::SharedPtr scan_msg(new sensor_msgs::msg::LaserScan);
    
    scan_msg->header.frame_id = p_target_frame_id_;
    scan_msg->header.stamp = now;
    scan_msg->angle_min = -M_PI;
    scan_msg->angle_max = M_PI;
    scan_msg->angle_increment = 2.0 * M_PI / (p_ranges_num_ - 1);
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = 0.1;
    scan_msg->range_min = p_min_scanner_range_;
    scan_msg->range_max = p_max_scanner_range_;
    scan_msg->ranges.assign(ranges.begin(), ranges.end());

    scan_pub_->publish(*scan_msg);
  }

  if (p_publish_pcl_) {

    auto pcl_msg = new_front_pcl;

    // Merge metadata
    pcl_msg.width += new_rear_pcl.width;

    // Re-size the merged data array to make space for the new points
    uint64_t OriginalSize = pcl_msg.data.size();
    pcl_msg.data.resize(pcl_msg.data.size() + new_rear_pcl.data.size());

    // Copy the new points from Cloud1 into the second half of the MergedCloud array
    std::copy(
    new_rear_pcl.data.begin(),
    new_rear_pcl.data.end(),
    pcl_msg.data.begin() + OriginalSize);

    pcl_msg.header.frame_id = p_target_frame_id_;
    pcl_msg.header.stamp = now;
    // pcl_msg.points.assign(points.begin(), points.end());
    // pcl_msg.channels.push_back(range_channel);
    // assert(range_channel.values.size() == points.size());

    pcl_pub_->publish(pcl_msg);
  }

  front_scan_received_ = false;
  rear_scan_received_ = false;
}
