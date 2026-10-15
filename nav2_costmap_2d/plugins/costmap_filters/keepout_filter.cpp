/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#include <string>
#include <memory>
#include <algorithm>
#include <array>
#include <limits>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

KeepoutFilter::KeepoutFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr), mask_costmap_(nullptr),
  mask_frame_(""), global_frame_("")
{
}

void KeepoutFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::filterInfoCallback, this, std::placeholders::_1));

  global_frame_ = layered_costmap_->getGlobalFrameID();
}

void KeepoutFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  // Checking that base and multiplier are set to their default values
  if (msg->base != BASE_DEFAULT || msg->multiplier != MULTIPLIER_DEFAULT) {
    RCLCPP_ERROR(
      logger_,
      "KeepoutFilter: For proper use of keepout filter base and multiplier"
      " in CostmapFilterInfo message should be set to their default values (%f and %f)",
      BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::maskCallback, this, std::placeholders::_1));
}

void KeepoutFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_costmap_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    mask_costmap_.reset();
  }

  // Making a new mask_costmap_
  mask_costmap_ = std::make_unique<Costmap2D>(*msg);
  mask_frame_ = msg->header.frame_id;

  // 更新窗口在 updateBounds() 中按掩码实际世界范围计算（需要 TF），此处只置位
  has_updated_data_ = true;
}

void KeepoutFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  CostmapFilter::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  // maskCallback() 与 updateBounds() 运行在不同线程，掩码及标志位统一由该锁保护
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!has_updated_data_ || !mask_costmap_) {
    return;
  }

  // T_global_mask：将 mask_frame_ 下的点变换到 global_frame_（同帧时为单位变换）
  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();
  if (mask_frame_ != global_frame_) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        global_frame_, mask_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      // 保留 has_updated_data_，下个更新周期重试，避免新掩码一直得不到刷新
      RCLCPP_WARN_THROTTLE(
        logger_, *(clock_), 2000,
        "KeepoutFilter: Failed to get mask frame (%s) transformation to costmap frame (%s) "
        "for update bounds, will retry: %s",
        mask_frame_.c_str(), global_frame_.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf2_transform);
  }

  // 掩码在 mask_frame_ 下为轴对齐矩形（Costmap2D 忽略原点朝向），
  // 四个角点变换到 global_frame_ 后取轴对齐包围盒
  const double mask_min_x = mask_costmap_->getOriginX();
  const double mask_min_y = mask_costmap_->getOriginY();
  const double mask_max_x =
    mask_min_x + mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution();
  const double mask_max_y =
    mask_min_y + mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution();
  const std::array<tf2::Vector3, 4> corners{
    tf2::Vector3(mask_min_x, mask_min_y, 0.0), tf2::Vector3(mask_max_x, mask_min_y, 0.0),
    tf2::Vector3(mask_min_x, mask_max_y, 0.0), tf2::Vector3(mask_max_x, mask_max_y, 0.0)};

  double new_min_x = std::numeric_limits<double>::max();
  double new_min_y = std::numeric_limits<double>::max();
  double new_max_x = std::numeric_limits<double>::lowest();
  double new_max_y = std::numeric_limits<double>::lowest();
  for (const tf2::Vector3 & corner : corners) {
    const tf2::Vector3 point = tf2_transform * corner;
    new_min_x = std::min(new_min_x, point.x());
    new_min_y = std::min(new_min_y, point.y());
    new_max_x = std::max(new_max_x, point.x());
    new_max_y = std::max(new_max_y, point.y());
  }

  // 更新窗口 = 新掩码范围 ∪ 上次掩码范围：旧掩码区域被重置后才能清掉残留的禁行代价
  *min_x = std::min(*min_x, new_min_x);
  *min_y = std::min(*min_y, new_min_y);
  *max_x = std::max(*max_x, new_max_x);
  *max_y = std::max(*max_y, new_max_y);
  if (has_applied_bounds_) {
    *min_x = std::min(*min_x, applied_min_x_);
    *min_y = std::min(*min_y, applied_min_y_);
    *max_x = std::max(*max_x, applied_max_x_);
    *max_y = std::max(*max_y, applied_max_y_);
  }

  applied_min_x_ = new_min_x;
  applied_min_y_ = new_min_y;
  applied_max_x_ = new_max_x;
  applied_max_y_ = new_max_y;
  has_applied_bounds_ = true;
  has_updated_data_ = false;
}

void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose2D & /*pose*/)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!mask_costmap_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "KeepoutFilter: Filter mask was not received");
    return;
  }

  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();  // initialize by identical transform
  int mg_min_x, mg_min_y;  // masger_grid indexes of bottom-left window corner
  int mg_max_x, mg_max_y;  // masger_grid indexes of top-right window corner

  if (mask_frame_ != global_frame_) {
    // Filter mask and current layer are in different frames:
    // prepare frame transformation if mask_frame_ != global_frame_
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        mask_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "KeepoutFilter: Failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame_.c_str(), mask_frame_.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf2_transform);

    mg_min_x = min_i;
    mg_min_y = min_j;
    mg_max_x = max_i;
    mg_max_y = max_j;
  } else {
    // Filter mask and current layer are in the same frame:
    // apply the following optimization - iterate only in overlapped
    // (min_i, min_j)..(max_i, max_j) & mask_costmap_ area.
    //
    //           mask_costmap_
    //       *----------------------------*
    //       |                            |
    //       |                            |
    //       |      (2)                   |
    // *-----+-------*                    |
    // |     |///////|<- overlapped area  |
    // |     |///////|   to iterate in    |
    // |     *-------+--------------------*
    // |    (1)      |
    // |             |
    // *-------------*
    //  master_grid (min_i, min_j)..(max_i, max_j) window
    //
    // ToDo: after costmap rotation will be added, this should be re-worked.

    double wx, wy;  // world coordinates

    // Calculating bounds corresponding to bottom-left overlapping (1) corner
    // mask_costmap_ -> master_grid intexes conversion
    const double half_cell_size = 0.5 * mask_costmap_->getResolution();
    wx = mask_costmap_->getOriginX() + half_cell_size;
    wy = mask_costmap_->getOriginY() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
    // Calculation of (1) corner bounds
    if (mg_min_x >= max_i || mg_min_y >= max_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);

    // Calculating bounds corresponding to top-right window (2) corner
    // mask_costmap_ -> master_grid intexes conversion
    wx = mask_costmap_->getOriginX() +
      mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution() + half_cell_size;
    wy = mask_costmap_->getOriginY() +
      mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
    // Calculation of (2) corner bounds
    if (mg_max_x <= min_i || mg_max_y <= min_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
  }

  // unsigned<-signed conversions.
  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;  // master_grid iterators
  unsigned int index;  // corresponding index of master_grid
  double gl_wx, gl_wy;  // world coordinates in a global_frame_
  double msk_wx, msk_wy;  // world coordinates in a mask_frame_
  unsigned int mx, my;  // mask_costmap_ coordinates
  unsigned char data, old_data;  // master_grid element data

  // Main master_grid updating loop
  // Iterate in costmap window by master_grid indexes
  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];
      // Calculating corresponding to (i, j) point at mask_costmap_:
      // Get world coordinates in global_frame_
      master_grid.mapToWorld(i, j, gl_wx, gl_wy);
      if (mask_frame_ != global_frame_) {
        // Transform (i, j) point from global_frame_ to mask_frame_
        tf2::Vector3 point(gl_wx, gl_wy, 0);
        point = tf2_transform * point;
        msk_wx = point.x();
        msk_wy = point.y();
      } else {
        // In this case master_grid and filter-mask are in the same frame
        msk_wx = gl_wx;
        msk_wy = gl_wy;
      }
      // Get mask coordinates corresponding to (i, j) point at mask_costmap_
      if (mask_costmap_->worldToMap(msk_wx, msk_wy, mx, my)) {
        data = mask_costmap_->getCost(mx, my);
        // Update if mask_ data is valid and greater than existing master_grid's one
        if (data == NO_INFORMATION) {
          continue;
        }
        if (data > old_data || old_data == NO_INFORMATION) {
          master_array[index] = data;
        }
      }
    }
  }
}

void KeepoutFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
}

bool KeepoutFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (mask_costmap_) {
    return true;
  }
  return false;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
